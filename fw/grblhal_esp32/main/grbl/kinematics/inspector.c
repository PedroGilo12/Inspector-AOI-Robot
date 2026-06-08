/*
  inspector.c - custom kinematics implementation

  Part of grblHAL
*/

#include "../grbl.h"

#if INSPECTOR_ROBOT

#include <math.h>
#include <string.h>
#include <stdio.h>

#include "../hal.h"
#include "../settings.h"
#include "../planner.h"
#include "../kinematics.h"
#include "../motion_control.h"
#include "../gcode.h"
#include "inspector.h"

/* Motores do CoreXY. */
#define A_MOTOR X_AXIS
#define B_MOTOR Y_AXIS

/* Escala de q4 (relacao de polia/correia) em GRAUS/mm. O angulo do punho e
   comandado/reportado em graus (M100 D, G0 B) e convertido em deslocamento dos
   motores. Constante mecanica fixa, independente de $100/$101. Calibrar comandando
   um angulo conhecido (ex.: M100 D90) e medindo o giro real: SCALE *= real/comandado. */
#define INSPECTOR_Y_SCALE 6.88f

/* Angulos sao em graus na interface; a trigonometria (IK/FK) roda em radianos. */
#define DEG2RAD (3.14159265f / 180.0f)
#define RAD2DEG (180.0f / 3.14159265f)

/* M-code de controle direto das juntas. P=q1 (base), Q=q2 (Z), R=q3 (linear), D=q4 (punho). */
#define INSPECTOR_MCODE_JOINTS ((user_mcode_t)100)

/* Comprimentos dos elos em metros (a IK roda em metros). */
#define L1 0.20f
#define L2 0.06f
#define L3 0.06f
#define L4 0.03f
#define LC 0.00f
#define DF 0.02f
#define D  (L4 + LC + DF)

/* Limites dos trilhos prismaticos (mm) e suas versoes em metros. */
#define LINK2_LIMIT_LOW 0.0
#define LINK2_LIMIT_HIGH 180.0
#define LINK3_LIMIT_LOW 0.0
#define LINK3_LIMIT_HIGH 180.0
#define LINK2_LIMIT_LOW_M  (LINK2_LIMIT_LOW  / 1000.0f)
#define LINK2_LIMIT_HIGH_M (LINK2_LIMIT_HIGH / 1000.0f)
#define LINK3_LIMIT_LOW_M  (LINK3_LIMIT_LOW  / 1000.0f)
#define LINK3_LIMIT_HIGH_M (LINK3_LIMIT_HIGH / 1000.0f)

/* Limites angulares em radianos: q4 (punho, +/-100 deg) e q1 (base, +/-180 deg). */
#define LINK4_LIMIT_ANGLE_LOW -1.74533
#define LINK4_LIMIT_ANGLE_HIGH 1.74533
#define LINK1_LIMIT_ANGLE_LOW -3.14159265
#define LINK1_LIMIT_ANGLE_HIGH 3.14159265

/* Tolerancia para juntas exatamente no limite (ex.: q1 = +/-pi) nao serem
   rejeitadas por arredondamento de ponto flutuante. */
#define LINK_LIMIT_EPS 1e-3f

/* Cadeia de handlers de M-code previamente registrados. */
static user_mcode_ptrs_t user_mcode;

/* M100 ativa este modo: a entrada ja esta em espaco de juntas, entao faz so a
   mistura CoreXY e pula a cinematica inversa cartesiana. */
static bool joint_mode = false;

/* Marcado quando alguma junta calculada fica fora dos limites; segment_line usa
   para invalidar o alvo do planner e abortar o movimento. */
static bool joint_limit_violation = false;

/* Cinematica direta: motores -> pose cartesiana. Desfaz a mistura CoreXY para
   recuperar as juntas e reconstroi a pose pelas equacoes de posicao da IK. */
static inline float *transform_to_cartesian (float *target, float *position)
{
    float a = position[A_MOTOR];
    float b = position[B_MOTOR];
    float q3 = -(a + b) * 0.5f / 1000.0f;                /* q3 (trilho linear) em metros: modo comum */
    float q4_deg = (b - a) * 0.5f * INSPECTOR_Y_SCALE;   /* q4 (punho) em graus: diferencial, +q4 = cima */
    float q4 = q4_deg * DEG2RAD;
    float q2 = position[Z_AXIS] / 1000.0f;               /* q2 (vertical) em metros */

#ifdef A_AXIS
    float q1 = position[A_AXIS];   /* base (rad) */
#else
    float q1 = 0.0f;
#endif

    /* FK do modelo DH (artigo): posicao radial gamma (eqs. 12-13) e altura pz (eq. 14).
       Tudo em metros, depois converte p/ mm. */
    float gamma = q3 + D * cosf(q4) - L1 + L3;       /* FK: gamma radial (artigo eqs. 12-13) */
    float pz = q2 + L2 - D * sinf(q4);               /* FK: altura pz (artigo eq. 14) */

    target[X_AXIS] = gamma * cosf(q1) * 1000.0f;     /* FK: px = gamma*cos(q1) */
    target[Y_AXIS] = gamma * sinf(q1) * 1000.0f;     /* FK: py = gamma*sin(q1) */
    target[Z_AXIS] = pz * 1000.0f;

    /* Reporta a orientacao: A = pitch (q4). B = yaw (q1) e SO informativo/read-only
       (a IK nunca le B como entrada; o yaw e determinado pela posicao XY). */
#ifdef A_AXIS
    target[A_AXIS] = q4_deg;          /* pitch, em graus (FK reporta A = q4) */
#endif
#ifdef B_AXIS
    target[B_AXIS] = q1 / DEG2RAD;    /* yaw (base) informativo, em graus — nunca lido como entrada */
#endif
#ifdef C_AXIS
    target[C_AXIS] = 0.0f;
#endif

    return target;
}

/* Cinematica inversa: pose cartesiana -> coordenadas dos motores.
   O robo tem 4 DOF: a entrada cartesiana e X, Y, Z e A (pitch). O yaw NAO e
   grau de liberdade independente — q1 e determinado pela posicao XY (artigo
   eq. 25), entao a word B nao e lida. q4 = pitch direto (artigo eq. 20, que e o
   round-trip da FK eq. 11, onde r31=cos q4 e r33=-sin q4); isso casa com a FK
   (que reporta A = q4) e elimina o offset de -90 da antiga matriz RPY. */
static inline float *transform_from_cartesian(float *target, float *position)
{
    /* M100: a entrada ja e espaco de juntas -> so a mistura CoreXY, sem IK cartesiana. */
    if(joint_mode) {
        joint_limit_violation = false;   /* controle direto: nao checa limites aqui */
        float jq3 = position[X_AXIS];
        float jq4 = position[Y_AXIS];
        target[A_MOTOR] = -jq3 - (jq4 / INSPECTOR_Y_SCALE);   /* q3 = modo comum, q4 = diferencial (+q4 = cima) */
        target[B_MOTOR] = -jq3 + (jq4 / INSPECTOR_Y_SCALE);
        target[Z_AXIS]  = position[Z_AXIS];
#ifdef A_AXIS
        target[A_AXIS] = position[A_AXIS];   /* q1 base */
#endif
#ifdef B_AXIS
        target[B_AXIS] = 0.0f;
#endif
        return target;
    }

    /* Translacao (mm -> m). */
    float px = position[X_AXIS] / 1000.0f;
    float py = position[Y_AXIS] / 1000.0f;
    float pz = position[Z_AXIS] / 1000.0f;

    /* IK: q4 = pitch direto (artigo eq. 20 — round-trip da FK eq. 11). A FK
       reporta A = q4, entao A = 0 => q4 = 0 (sem o offset de -90 da matriz RPY). */
#ifdef A_AXIS
    float q4 = position[A_AXIS] * DEG2RAD;   /* A = pitch */
#else
    float q4 = 0.0f;
#endif

    /* IK: q2 (artigo eq. 32). */
    float q2 = pz - L2 + D * sinf(q4);

    /* IK: q1 a partir da posicao (artigo eq. 25) com ambiguidade de ramo de gamma
       (eq. 28: gamma = +/-raiz(px^2+py^2)). Como o braco estende em -x (a=-L1),
       gamma costuma ser NEGATIVO -> dois candidatos:
         Ramo +: gamma=+raiz, q1=atan2(py,px)
         Ramo -: gamma=-raiz, q1=atan2(py,px)±pi
       Para cada ramo, q3 = gamma - D*cos(q4) + L1 - L3 (artigo eq. 30). Escolhe-se
       o ramo cujo q1 E q3 caem DENTRO dos limites. Tenta o ramo - primeiro
       (fisico); se nenhum couber, mantem o ramo - e a checagem de limites abaixo
       rejeita o movimento. */
    float r = sqrtf(px * px + py * py);
    float q1 = 0.0f, q3 = 0.0f;
    bool branch_found = false;
    for(int s = 0; s < 2 && !branch_found; s++) {
        float sign  = (s == 0) ? -1.0f : 1.0f;   /* ramo - antes do + */
        float gamma = sign * r;
        float c1 = (fabsf(gamma) < 1e-9f) ? 0.0f : atan2f(py / gamma, px / gamma);
        float c3 = gamma - D * cosf(q4) + L1 - L3;   /* IK: q3 (artigo eq. 30) */
        bool c1_in = (c1 >= LINK1_LIMIT_ANGLE_LOW - LINK_LIMIT_EPS) &&
                     (c1 <= LINK1_LIMIT_ANGLE_HIGH + LINK_LIMIT_EPS);
        bool c3_in = (c3 * 1000.0f >= LINK3_LIMIT_LOW  - LINK_LIMIT_EPS) &&
                     (c3 * 1000.0f <= LINK3_LIMIT_HIGH + LINK_LIMIT_EPS);
        if((c1_in && c3_in) || s == 0) { q1 = c1; q3 = c3; }   /* default: ramo - */
        if(c1_in && c3_in) branch_found = true;
    }

    float q2_mm = q2 * 1000.0f;
    float q3_mm = q3 * 1000.0f;

    /* Debug: imprime as juntas calculadas. */
    {
        char buf[128];
        snprintf(buf, sizeof(buf),
                 "[IK q1=%.4f rad (%.2f deg) q2=%.4f mm q3=%.4f mm q4=%.4f rad (%.2f deg)]" ASCII_EOL,
                 q1, q1 * RAD2DEG, q2_mm, q3_mm, q4, q4 * RAD2DEG);
        hal.stream.write(buf);
    }

    /* Checagem de limites: rejeita o movimento se alguma junta sair da faixa. */
    bool q1_ok    = (q1    >= LINK1_LIMIT_ANGLE_LOW - LINK_LIMIT_EPS) && (q1    <= LINK1_LIMIT_ANGLE_HIGH + LINK_LIMIT_EPS);
    bool q2_ok_mm = (q2_mm >= LINK2_LIMIT_LOW       - LINK_LIMIT_EPS) && (q2_mm <= LINK2_LIMIT_HIGH      + LINK_LIMIT_EPS);
    bool q3_ok_mm = (q3_mm >= LINK3_LIMIT_LOW       - LINK_LIMIT_EPS) && (q3_mm <= LINK3_LIMIT_HIGH      + LINK_LIMIT_EPS);
    bool q4_ok    = (q4    >= LINK4_LIMIT_ANGLE_LOW  - LINK_LIMIT_EPS) && (q4    <= LINK4_LIMIT_ANGLE_HIGH + LINK_LIMIT_EPS);

    joint_limit_violation = !(q1_ok && q2_ok_mm && q3_ok_mm && q4_ok);

    if(joint_limit_violation) {
        char buf[160];
        hal.stream.write("[IK ERROR: joint out of limits, move rejected]" ASCII_EOL);
        if(!q1_ok) {
            snprintf(buf, sizeof(buf), "[  q1=%.4f rad (%.2f deg) limit [%.2f,%.2f] deg]" ASCII_EOL,
                     q1, q1 * RAD2DEG, LINK1_LIMIT_ANGLE_LOW * RAD2DEG, LINK1_LIMIT_ANGLE_HIGH * RAD2DEG);
            hal.stream.write(buf);
        }
        if(!q2_ok_mm) {
            snprintf(buf, sizeof(buf), "[  q2=%.4f mm limit [%.2f,%.2f] mm]" ASCII_EOL,
                     q2_mm, (float)LINK2_LIMIT_LOW, (float)LINK2_LIMIT_HIGH);
            hal.stream.write(buf);
        }
        if(!q3_ok_mm) {
            snprintf(buf, sizeof(buf), "[  q3=%.4f mm limit [%.2f,%.2f] mm]" ASCII_EOL,
                     q3_mm, (float)LINK3_LIMIT_LOW, (float)LINK3_LIMIT_HIGH);
            hal.stream.write(buf);
        }
        if(!q4_ok) {
            snprintf(buf, sizeof(buf), "[  q4=%.4f rad (%.2f deg) limit [%.2f,%.2f] deg]" ASCII_EOL,
                     q4, q4 * RAD2DEG, LINK4_LIMIT_ANGLE_LOW * RAD2DEG, LINK4_LIMIT_ANGLE_HIGH * RAD2DEG);
            hal.stream.write(buf);
        }
    }

    /* Mistura CoreXY: q3 = modo comum, q4 (rad->graus)/SCALE = diferencial (+q4 = cima).
       Camada de hardware (acoplamento mecanico dos motores), fora do modelo DH do artigo. */
    target[X_AXIS] = -q3_mm - (q4 * RAD2DEG / INSPECTOR_Y_SCALE);
    target[Y_AXIS] = -q3_mm + (q4 * RAD2DEG / INSPECTOR_Y_SCALE);
    target[Z_AXIS] = q2_mm;

    /* q1 (base) e acionado diretamente pelo motor A_AXIS (nao passa pela CoreXY). */
#ifdef A_AXIS
    target[A_AXIS] = q1;
#endif

    return target;
}

/* Converte passos dos motores em posicao cartesiana da maquina. */
static float *inspector_convert_array_steps_to_mpos (float *position, int32_t *steps)
{
    uint_fast8_t idx;
    float mpos[N_AXIS];

    for(idx = 0; idx < N_AXIS; idx++) {
        mpos[idx] = steps[idx] / settings.axis[idx].steps_per_mm;
    }

    return transform_to_cartesian(position, mpos);
}

/* Segmenta uma linha cartesiana em alvos de motor para o planner. */
static float *inspector_segment_line (float *target, float *position, plan_line_data_t *pl_data, bool init)
{
    static uint_fast8_t iterations;
    static float trsf[N_AXIS];

    if(init) {
        iterations = 2;
        transform_from_cartesian(trsf, target);

        /* Fora dos limites: invalida o alvo para o mc_line abortar o movimento. */
        if(joint_limit_violation && pl_data) {
            pl_data->condition.target_validated = On;
            pl_data->condition.target_valid     = Off;
        }
    }

    return iterations-- == 0 ? NULL : trsf;
}

/* --- Controle direto das juntas via M100 ----------------------------------- */

/* Informa se este M-code e tratado aqui. */
static user_mcode_type_t mcode_check (user_mcode_t mcode)
{
    return mcode == INSPECTOR_MCODE_JOINTS
                     ? UserMCode_Normal
                     : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Unsupported);
}

/* Valida os parametros do M100 e reivindica as words para o parser nao as marcar como nao usadas. */
static status_code_t mcode_validate (parser_block_t *gc_block)
{
    status_code_t state = Status_OK;

    if(gc_block->user_mcode == INSPECTOR_MCODE_JOINTS) {

        /* P/Q/R/D sao alvos absolutos de junta; rejeita valores nao finitos. */
        if((gc_block->words.p && isnan(gc_block->values.p)) ||
           (gc_block->words.q && isnan(gc_block->values.q)) ||
           (gc_block->words.r && isnan(gc_block->values.r)) ||
           (gc_block->words.d && isnan(gc_block->values.d))) {
               state = Status_BadNumberFormat;
           }

        /* Reivindica as words consumidas (deixa F para o tratamento modal normal). */
        gc_block->words.p = gc_block->words.q = gc_block->words.r = gc_block->words.d = Off;

    } else {
        state = Status_Unhandled;
    }

    return state == Status_Unhandled && user_mcode.validate ? user_mcode.validate(gc_block) : state;
}

/* Executa o M100: move direto em espaco de juntas. A mistura CoreXY converte o
   alvo de junta em passos de motor, igual a um G0/G1 normal. */
static void mcode_execute (uint_fast16_t state, parser_block_t *gc_block)
{
    if(gc_block->user_mcode == INSPECTOR_MCODE_JOINTS) {

        float target[N_AXIS];
        plan_line_data_t plan_data = {0};

        /* Recupera as juntas atuais da posicao (motor) do planner e sobrescreve so
           as informadas. Desfaz a mistura CoreXY para q3/q4; q2/q1 sao diretos. */
        float *mpos = plan_get_position();
        float q3 = -(mpos[A_MOTOR] + mpos[B_MOTOR]) * 0.5f;                      /* modo comum */
        float q4 = (mpos[B_MOTOR] - mpos[A_MOTOR]) * 0.5f * INSPECTOR_Y_SCALE;   /* diferencial (+q4 = cima) */

        target[X_AXIS] = gc_block->words.r ? gc_block->values.r : q3;             /* q3 (linear) */
        target[Y_AXIS] = gc_block->words.d ? gc_block->values.d : q4;             /* q4 (punho) */
        target[Z_AXIS] = gc_block->words.q ? gc_block->values.q : mpos[Z_AXIS];   /* q2 (vertical) */
#ifdef A_AXIS
        target[A_AXIS] = gc_block->words.p ? gc_block->values.p : mpos[A_AXIS];   /* q1 (base) */
#endif
#ifdef B_AXIS
        target[B_AXIS] = 0.0f;
#endif

        /* Replica a configuracao do planner do G0/G1 (gc_execute_block). */
        plan_data.overrides = gc_state.modal.override_ctrl;
        plan_data.condition.target_validated = plan_data.condition.target_valid = sys.soft_limits.mask == 0;
        plan_data.spindle.hal = gc_state.spindle->hal;

        if(gc_state.feed_rate > 0.0f)
            plan_data.feed_rate = gc_state.feed_rate;       /* movimento controlado (G1) */
        else
            plan_data.condition.rapid_motion = On;          /* rapido (G0) se sem feed */

        /* Pula a IK cartesiana (o alvo ja e espaco de juntas). */
        joint_mode = true;
        mc_line(target, &plan_data);
        joint_mode = false;

        /* Sincroniza a posicao cartesiana do parser com o novo estado de juntas. */
        transform_to_cartesian(gc_state.position, plan_get_position());

    } else if(user_mcode.execute) {
        user_mcode.execute(state, gc_block);
    }
}

/* Inicializa os ponteiros da API para a cinematica INSPECTOR. */
void inspector_init (void)
{
    kinematics.transform_steps_to_cartesian = inspector_convert_array_steps_to_mpos;
    kinematics.transform_from_cartesian     = transform_from_cartesian;
    kinematics.segment_line                 = inspector_segment_line;

    /* Registra o M-code de junta direta (M100), encadeando handlers existentes. */
    memcpy(&user_mcode, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));
    grbl.user_mcode.check    = mcode_check;
    grbl.user_mcode.validate = mcode_validate;
    grbl.user_mcode.execute  = mcode_execute;
}

#endif // INSPECTOR_ROBOT

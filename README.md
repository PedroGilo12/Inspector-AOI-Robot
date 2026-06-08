# Inspector V1 — Cinemática

Modelagem e validação da cinemática do manipulador robótico **Inspector V1**, um
robô voltado para **Inspeção Óptica Automatizada (AOI — _Automated Optical
Inspection_)**. O projeto cobre a cinemática direta (via parametrização de
Denavit-Hartenberg) e a cinemática inversa analítica (em forma fechada),
implementadas em Python com a [Robotics Toolbox for Python](https://github.com/petercorke/robotics-toolbox-python)
de Peter Corke.

O documento de referência com toda a dedução matemática está em
[docs/Inspecto_V1_Kinematic.pdf](docs/Inspecto_V1_Kinematic.pdf).

> Artigo: _Cinemática Inspector V1_ (2026, v-1.0.0) — Pedro Henrique Vieira Giló,
> Thiago Fellype Marques Laurentino, Caio Oliveira França dos Anjos e José
> Anderson da Silva (UFAL).

## Visão geral do robô

O Inspector V1 é um manipulador de 4 juntas atuadas que posiciona um cabeçote de
câmera no espaço operacional:

- **Sistema CoreXY adaptado** para o controle da junta horizontal e da junta
  rotativa da câmera (conceito comum em impressoras 3D), traduzindo o movimento
  combinado de correias e motores em deslocamentos lineares e angulares precisos.
- **Distribuição de carga otimizada no eixo Z**: os 4 motores ficam na coluna
  vertical, reduzindo a massa suspensa, a deflexão mecânica do braço e a inércia
  durante as paradas para inspeção.
- **Interface serial (UART)** a 115200 bps (também acessível por Wi-Fi, ver
  [Firmware](#firmware-fw)), com comandos de movimentação linear cartesiana
  (`G0`/`G1 X Y Z A`, que usam a cinemática inversa interna) e de movimentação
  direta das juntas (`M100`, com `P=q1`, `Q=q2`, `R=q3`, `D=q4`).

### Espaço de configuração

O vetor de juntas é `Q = {q1, q2, q3, q4}`:

| Junta | Tipo       | Descrição                            | Limites           |
| ----- | ---------- | ------------------------------------ | ----------------- |
| `q1`  | Rotativa   | Rotação da base                      | (-π, π)           |
| `q2`  | Prismática | Deslocamento vertical (eixo Z)       | [0.0, 0.18] m     |
| `q3`  | Prismática | Extensão horizontal do braço         | [0.0, 0.18] m     |
| `q4`  | Rotativa   | Rotação do cabeçote da câmera        | [-π/2, π/2] rad   |

A cadeia DH usada no código possui 6 elos: as 4 juntas físicas mais **duas juntas
virtuais** (`v1`, `v2`, travadas em 0) para alinhar corretamente os sistemas de
coordenadas intermediários.

## Estrutura do repositório

```
.
├── docs/
│   └── Inspecto_V1_Kinematic.pdf   # Artigo com a dedução completa da cinemática
├── fw/
│   └── grblhal_esp32/              # Firmware embarcado (grblHAL/ESP32) do robô real
├── kinematics/
│   ├── inspector_fkine.py          # Cinemática direta (simbólica + visualização)
│   ├── inspector_ikine.py          # Cinemática inversa analítica + validação
│   ├── ikine_loop.py               # Envio de trajetórias G-code via serial (teste)
│   └── ikine_debug.py              # Depuração da cinemática inversa via serial
├── requirements.txt                # Dependências Python (versões fixadas)
└── README.md
```

## Cinemática direta

[kinematics/inspector_fkine.py](kinematics/inspector_fkine.py) constrói o modelo
DH (`DHRobot`) e demonstra a cinemática direta de duas formas:

1. **Derivação simbólica** (SymPy): monta a matriz de transformação homogênea
   global `⁰T₆` em função dos símbolos `q1…q4` e `L1, L2, L3, L4, LC, DF`,
   imprimindo a matriz de pose final simplificada e cada transformação
   intermediária entre juntas consecutivas — exatamente as equações deduzidas
   no artigo.
2. **Visualização interativa**: instancia o robô com valores numéricos dos
   comprimentos dos elos e abre o `teach` da Robotics Toolbox para manipulação
   dos eixos.

Parâmetros de Denavit-Hartenberg (Tabela 1 do artigo):

| Elo | θᵢ        | dᵢ        | αᵢ    | aᵢ                |
| --- | --------- | --------- | ----- | ----------------- |
| e1  | q1        | L2        | 0     | -L1               |
| e2  | 0         | q2        | -π/2  | 0                 |
| v1  | -π/2      | 0         | -π/2  | 0                 |
| e3  | 0         | q3 + L3   | π/2   | 0                 |
| e4  | q4 - π/2  | 0         | 0     | -(L4 + LC + DF)   |
| v2  | π/2       | 0         | -π/2  | 0                 |

## Cinemática inversa

[kinematics/inspector_ikine.py](kinematics/inspector_ikine.py) implementa a
solução **analítica em forma fechada** (sem iteração numérica), obtida por
decomposição geométrica da matriz de pose `⁰T₆`:

- `q4 = atan2(-r33, r31)`
- `q1 = atan2(py, px)`
- `q3 = ±√(px² + py²) - (L4 + LC + DF)·cos(q4) + L1 - L3`
- `q2 = pz - L2 + (L4 + LC + DF)·sin(q4)`

A função `ikine_inspector(T, branch=...)` resolve a pose alvo `T` (SE(3) 4×4). O
sinal de γ gera dois ramos de solução; com `branch="auto"` ambos são testados e
mantém-se o que melhor reconstrói `T` (a orientação desfaz a ambiguidade de 180°).

A função `validar()` gera configurações aleatórias dentro dos limites das juntas,
calcula a pose pela cinemática direta, recupera as juntas pela inversa e verifica
o erro de reconstrução (tolerância de `1e-9`).

## Firmware (`fw/`)

A pasta [fw/grblhal_esp32](fw/grblhal_esp32) contém o **firmware embarcado** que
roda no robô físico: uma versão do [grblHAL](https://github.com/grblHAL) para
**ESP32**, configurada para a placa **MKS TinyBee V1**. É o elo de hardware do
ciclo descrito no artigo (teoria matemática → simulação no CoppeliaSim →
protótipo real): as mesmas equações de cinemática deduzidas no PDF são embarcadas
aqui e executadas em tempo real sobre os comandos G-code recebidos.

### O que foi adicionado/alterado para o Inspector

A base é o grblHAL/ESP32 original; a customização do Inspector se concentra em um
**plugin de cinemática novo** mais pequenos ajustes no núcleo:

**Arquivos novos**

- [`main/grbl/kinematics/inspector.c`](fw/grblhal_esp32/main/grbl/kinematics/inspector.c)
  — plugin de cinemática do Inspector (FK, IK, mistura CoreXY e o M-code `M100`).
- [`main/grbl/kinematics/inspector.h`](fw/grblhal_esp32/main/grbl/kinematics/inspector.h)
  — declara `inspector_init()`.

**Arquivos modificados em relação ao upstream**

- `main/grbl/config.h`:
  - `N_AXIS` 3 → **5** (`X Y Z` + `A B`); o eixo C/_yaw_ é descartado porque o
    robô não possui esse grau de liberdade — coerente com a conclusão do artigo
    de que `q1` (yaw) não é um DOF independente.
  - Habilita `KINEMATICS_API` e define `INSPECTOR_ROBOT On`.
  - `DEFAULT_STEPPER_IDLE_LOCK_TIME` 25 → **255** ms (motores permanecem
    energizados, evitando perda de posição entre movimentos).
  - `DEFAULT_{X,Y,Z}_STEPS_PER_MM` 250 → **80**.
- `main/grbl/grbllib.c`: inclui `kinematics/inspector.h` e chama `inspector_init()`
  na inicialização (`grbl_enter()`), sob `#if INSPECTOR_ROBOT`.
- `main/CMakeLists.txt`: adiciona `grbl/kinematics/inspector.c` à lista de fontes.
- `main/my_machine.h`: seleciona `BOARD_MKS_TINYBEE_V1` e habilita Wi-Fi em modo
  Soft-AP (`WIFI_ENABLE`, `WIFI_SOFTAP`) e a interface web (`WEBUI_ENABLE`).

### Relação com o artigo

O plugin implementa exatamente as equações deduzidas em
[docs/Inspecto_V1_Kinematic.pdf](docs/Inspecto_V1_Kinematic.pdf):

| Código (`inspector.c`)                | Função              | Artigo                                   |
| ------------------------------------- | ------------------- | ---------------------------------------- |
| `transform_to_cartesian`              | Cinemática direta   | `gamma` (eqs. 12–13), `pz` (eq. 14)      |
| `transform_from_cartesian`            | Cinemática inversa  | `q4` (eq. 20), `q1` (eq. 25), ramo ± de `gamma` (eq. 28), `q3` (eq. 30), `q2` (eq. 32) |
| Mistura CoreXY (`A_MOTOR`/`B_MOTOR`)  | Acoplamento físico  | Camada de hardware — fora do modelo DH   |
| M-code `M100`                         | Juntas diretas      | "Movimentação Direta das Juntas (M100)"  |

Pontos-chave herdados da análise do artigo:

- O **_yaw_ não é entrada** do sistema: `q1` é determinado univocamente pela
  posição cartesiana `XY` (eq. 25), então `G0`/`G1` recebem apenas `X Y Z` e o
  _pitch_ `A` (= `q4`). A word `B` reportada pela FK é apenas informativa.
- A IK tem **dois ramos** (sinal de `gamma`, eq. 28); o firmware escolhe
  automaticamente o ramo cujas juntas caem dentro dos limites e **rejeita o
  movimento** se nenhuma solução for válida.
- A **mistura CoreXY** (`q3` = modo comum, `q4` = diferencial dos motores X/Y) é
  um acoplamento mecânico do hardware, aplicado por cima do modelo DH.

### Interfaces de comunicação

O firmware aceita G-code por dois meios equivalentes (mesmo protocolo grbl):

- **Serial (USB/UART)** a 115200 bps.
- **Wi-Fi em modo Soft-AP**: o robô cria a rede `InspectorRobotV1` (sem roteador
  externo) e fica acessível no IP fixo `192.168.5.1`, expondo os serviços:

  | Serviço   | Porta | Finalidade                                   |
  | --------- | ----- | -------------------------------------------- |
  | Telnet    | 23    | Envio de G-code e respostas (igual ao serial) |
  | WebSocket | 81    | Mesmo fluxo do Telnet, usado pela interface web |
  | HTTP      | 80    | Interface gráfica (WebUI) no navegador        |
  | FTP       | 21    | Transferência de arquivos de G-code           |

Comandos de movimentação:

- `G0`/`G1 X Y Z A` — movimento cartesiano (a IK interna calcula as juntas).
- `M100 P Q R D` — controle direto das juntas: `P=q1` (base), `Q=q2` (vertical),
  `R=q3` (trilho linear), `D=q4` (punho). Útil para calibração e testes de
  hardware, pois pula a IK cartesiana.

## Compilação e gravação do firmware

O firmware é compilado com **ESP-IDF v4.3**. A forma mais simples é usar a imagem
Docker oficial da Espressif, sem instalar a toolchain localmente.

A partir da raiz do repositório, entre na pasta do firmware e suba o container
(o `$(pwd)` montado precisa ser a pasta `fw/grblhal_esp32`):

```bash
cd fw/grblhal_esp32
docker run -it --rm \
  -v $(pwd):/grbl \
  -w /grbl \
  --privileged \
  -v /dev:/dev \
  espressif/idf:release-v4.3 \
  /bin/bash
```

> `--privileged` e `-v /dev:/dev` expõem a porta serial do host dentro do
> container; `release-v4.3` é a versão de ESP-IDF exigida por este driver.

Já dentro do container, compile e grave na placa:

```bash
idf.py build
idf.py -p /dev/ttyUSB0 flash
idf.py -p /dev/ttyUSB0 flash monitor
```

> Ajuste `/dev/ttyUSB0` para a porta correta da sua placa. `flash monitor` grava
> e abre o monitor serial em seguida; saia do monitor com `Ctrl+]`.

## Instalação

Requer Python 3.12. Recomenda-se um ambiente virtual:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Uso

Cinemática direta (matrizes simbólicas + janela interativa `teach`):

```bash
python kinematics/inspector_fkine.py
```

Cinemática inversa (validação automática + exemplo de recuperação de juntas):

```bash
python kinematics/inspector_ikine.py
```

## Referências

- CORKE, P. _Robotics, Vision and Control_. 1. ed. Cham: Springer, 2011.
  (Springer Tracts in Advanced Robotics, v. 73).
- Repositório: <https://github.com/PedroGilo12/Inspector-AOI-Robot>

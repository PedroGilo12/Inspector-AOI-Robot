import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3

# Réplica fiel do firmware inspector.c para comparação JUSTA: mesma convenção de
# entrada (q4 = pitch direto, sem o offset de -90 do SE3.RPY), mesma escolha de
# ramo de gamma (viabilidade de limites, ramo - antes do +) e os MESMOS limites.

# --- Limites e constantes IDÊNTICOS ao firmware (inspector.c) ---
LINK1_LIMITS = [-np.pi, np.pi]          # q1 base (rad)         = +-180 graus
LINK2_LIMITS = [0.0, 0.18]              # q2 vertical (m)       = [0,180] mm
LINK3_LIMITS = [0.0, 0.18]              # q3 trilho linear (m)  = [0,180] mm
LINK4_LIMITS = [-1.74533, 1.74533]      # q4 punho (rad)        = +-100 graus
LIMIT_EPS    = 1e-3                      # mesma tolerância de limite do firmware

L1 = 0.2
L2 = 0.06
L3 = 0.06
L4 = 0.03
LC = 0.0
DF = 0.02

D = L4 + LC + DF

link1 = rtb.RevoluteDH(d=L2, alpha=0, a=-L1)
link2 = rtb.PrismaticDH(theta=0.0, alpha=-np.pi/2, a=0.0, qlim=LINK2_LIMITS)
virtual1 = rtb.PrismaticDH(theta=-np.pi/2, alpha=-np.pi/2, a=0.0, qlim=[0, 0])
link3 = rtb.PrismaticDH(theta=0.0, alpha=np.pi/2, a=0.0, qlim=LINK3_LIMITS, offset=L3)
link4 = rtb.RevoluteDH(d=0, alpha=0, a=-(L4 + LC + DF), offset=-np.pi/2, qlim=LINK4_LIMITS)
virtual2 = rtb.PrismaticDH(theta=np.pi/2, alpha=-np.pi/2, a=0.0, qlim=[0, 0])

inspector_robot = rtb.DHRobot([link1, link2, virtual1, link3, link4, virtual2], name="InspectorDH")


def to_full(q1, q2, q3, q4):
    return [q1, q2, 0.0, q3, q4, 0.0]


# Cinemática direta cartesiana - RÉPLICA do firmware transform_to_cartesian.
# Devolve (X, Y, Z) em mm e A = pitch (= q4) em graus, como o G-code reporta.
def fkine_cartesian(q1, q2, q3, q4):
    gamma = q3 + D * np.cos(q4) - L1 + L3        # FK: gamma radial (artigo eqs. 12-13)
    px = gamma * np.cos(q1)
    py = gamma * np.sin(q1)
    pz = q2 + L2 - D * np.sin(q4)                # FK: altura pz (artigo eq. 14)
    return px * 1000.0, py * 1000.0, pz * 1000.0, np.degrees(q4)


# Escolha de ramo de gamma - RÉPLICA EXATA do firmware transform_from_cartesian.
# px, py, pz em metros; q4 = pitch (rad). Tenta o ramo - antes do +; fica no
# primeiro cujo q1 E q3 caem nos limites; default ramo - se nenhum couber.
def _select_branch(px, py, pz, q4):
    q2 = pz - L2 + D * np.sin(q4)                # IK: q2 (artigo eq. 32)
    r = np.hypot(px, py)
    q1, q3 = 0.0, 0.0
    found = False
    for s, sign in enumerate((-1.0, +1.0)):      # ramo - antes do +
        if found:
            break
        gamma = sign * r                         # IK: ramo de gamma (artigo eq. 28)
        c1 = 0.0 if abs(gamma) < 1e-9 else np.arctan2(py / gamma, px / gamma)  # IK: q1 (eq. 25)
        c3 = gamma - D * np.cos(q4) + L1 - L3     # IK: q3 (artigo eq. 30)
        c1_in = LINK1_LIMITS[0] - LIMIT_EPS <= c1 <= LINK1_LIMITS[1] + LIMIT_EPS
        c3_in = LINK3_LIMITS[0] - LIMIT_EPS <= c3 <= LINK3_LIMITS[1] + LIMIT_EPS
        if (c1_in and c3_in) or s == 0:          # default: ramo -
            q1, q3 = c1, c3
        if c1_in and c3_in:
            found = True
    return (q1, q2, q3, q4)


# Entrada IDÊNTICA ao G-code do firmware: X, Y, Z em mm e A = pitch (= q4) em graus.
# Retorna (q1 [rad], q2 [m], q3 [m], q4 [rad]) exatamente como o inspector.c calcula.
def ikine_firmware(X_mm, Y_mm, Z_mm, A_deg):
    px, py, pz = X_mm / 1000.0, Y_mm / 1000.0, Z_mm / 1000.0
    q4 = np.radians(A_deg)                       # IK: q4 = pitch (artigo eq. 20)
    return _select_branch(px, py, pz, q4)


# Cinemática inversa a partir de uma pose T (SE(3) 4x4). q4 vem da própria pose:
# quando T é gerada pela FK do robô (não por SE3.RPY), atan2(-r33, r31) = q4
# (round-trip da FK eq. 11), então o resultado é idêntico ao do firmware.
# branch: "+"/"-" força o sinal de gamma; "auto" usa a MESMA lógica do firmware.
def ikine_inspector(T, branch="auto"):
    px, py, pz = T[0, 3], T[1, 3], T[2, 3]
    r31, r33 = T[2, 0], T[2, 2]

    q4 = np.arctan2(-r33, r31)                   # IK: q4 (artigo eq. 20)

    if branch in ("+", "-"):
        sign = 1.0 if branch == "+" else -1.0
        gamma = sign * np.hypot(px, py)
        q1 = 0.0 if abs(gamma) < 1e-12 else np.arctan2(py / gamma, px / gamma)
        q3 = gamma - D * np.cos(q4) + L1 - L3
        return (q1, pz - L2 + D * np.sin(q4), q3, q4)

    return _select_branch(px, py, pz, q4)        # "auto" = mesma escolha do firmware


# Validação de round-trip nas QUANTIDADES CONTROLÁVEIS (X, Y, Z, A) - as mesmas
# que o robô de 4 DOF comanda. (Comparar a matriz de pose inteira não seria justo:
# o yaw não é grau de liberdade, então a orientação completa não é controlável.)
def validar(n=8, seed=0, verbose=True):
    rng = np.random.default_rng(seed)
    max_err = 0.0
    if verbose:
        print("\nValidação round-trip FK->IK->FK em (X,Y,Z,A)\n" + "-" * 60)
    for t in range(n):
        q1 = rng.uniform(*LINK1_LIMITS)
        q2 = rng.uniform(*LINK2_LIMITS)
        q3 = rng.uniform(*LINK3_LIMITS)
        q4 = rng.uniform(*LINK4_LIMITS)

        X, Y, Z, A = fkine_cartesian(q1, q2, q3, q4)
        qr = ikine_firmware(X, Y, Z, A)
        X2, Y2, Z2, A2 = fkine_cartesian(*qr)

        err = max(abs(X - X2), abs(Y - Y2), abs(Z - Z2), abs(A - A2))
        max_err = max(max_err, err)
        if verbose:
            print(f"teste {t}:  erro (X,Y,Z em mm, A em deg) = {err:.2e}")

    if verbose:
        print("-" * 60)
        print(f"Maior erro: {max_err:.2e}  -> {'OK' if max_err < 1e-6 else 'FALHOU'}")
    return max_err


if __name__ == "__main__":
    validar(n=8)

    # IK com a MESMA entrada do G-code do firmware (X,Y,Z em mm, A=pitch em graus).
    print("\nIK com a mesma entrada do firmware (G0 X Y Z A)\n" + "-" * 70)
    poses = [
        # (X_mm, Y_mm, Z_mm, A_deg)
        (-30.000,   0.000, 160.000,   0.0),
        (-71.340,   0.000, 135.000,  30.0),
        (-22.161, -22.161, 185.000, -30.0),
        (  0.000, -50.000, 160.000,   0.0),
        ( -6.464,  11.197, 102.929,  45.0),
    ]
    for X, Y, Z, A in poses:
        q1, q2, q3, q4 = ikine_firmware(X, Y, Z, A)
        print(f"G0 X{X:8.3f} Y{Y:8.3f} Z{Z:8.3f} A{A:5.1f}  ->  "
              f"q1={np.degrees(q1):+7.2f} q2={q2*1000:7.2f}mm  "
              f"q3={q3*1000:7.2f}mm  q4={np.degrees(q4):+7.2f}")

    # Visualização da primeira pose.
    q1, q2, q3, q4 = ikine_firmware(*poses[0])
    inspector_robot.plot(to_full(q1, q2, q3, q4), block=True)

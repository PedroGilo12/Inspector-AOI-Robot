import numpy as np
import roboticstoolbox as rtb

LINK2_LIMITS = [0.0, 0.18]
LINK3_LIMITS = [0.0, 0.18]
LINK4_LIMITS = [-np.pi/2, np.pi/2]

# Cinemática inversa analítica do Inspector e validação contra a direta
# -------
L1 = 0.2
L2 = 0.06
L3 = 0.06
L4 = 0.01
LC = 0.0
DF = 0.0

D = L4 + LC + DF  # termo repetido nas equações da IK

link1 = rtb.RevoluteDH(d=L2, alpha=0, a=-L1)
link2 = rtb.PrismaticDH(theta=0.0, alpha=-np.pi/2, a=0.0, qlim=LINK2_LIMITS)
virtual1 = rtb.PrismaticDH(theta=-np.pi/2, alpha=-np.pi/2, a=0.0, qlim=[0, 0])
link3 = rtb.PrismaticDH(theta=0.0, alpha=np.pi/2, a=0.0, qlim=LINK3_LIMITS, offset=L3)
link4 = rtb.RevoluteDH(d=0, alpha=0, a=-(L4 + LC + DF), offset=-np.pi/2, qlim=LINK4_LIMITS)
virtual2 = rtb.PrismaticDH(theta=np.pi/2, alpha=-np.pi/2, a=0.0, qlim=[0, 0])

inspector_robot = rtb.DHRobot([link1, link2, virtual1, link3, link4, virtual2], name="InspectorDH")
print(inspector_robot)


# Mapeia entre as 4 juntas atuadas e o vetor de 6 posições do toolbox
# (insere/remove as juntas virtuais v1 e v2, travadas em 0)
def to_full(q1, q2, q3, q4):
    return [q1, q2, 0.0, q3, q4, 0.0]


def from_full(q_full):
    return [q_full[0], q_full[1], q_full[3], q_full[4]]


# Cinemática inversa analítica (eqs. 18, 23, 28 e 30 do artigo)
def ikine_inspector(T, branch="auto"):
    """Resolve a IK do Inspector a partir de uma pose alvo T (SE(3) 4x4).

    branch: "+"/"-" força o sinal de gamma; "auto" testa os dois e mantém
    o que melhor reconstrói T (a orientação desfaz a ambiguidade de 180°).
    """
    px, py, pz = T[0, 3], T[1, 3], T[2, 3]
    r31, r33 = T[2, 0], T[2, 2]

    q4 = np.arctan2(-r33, r31)
    q2 = pz - L2 + D * np.sin(q4)

    def solve_branch(sign):
        gamma = sign * np.sqrt(px**2 + py**2)
        q1 = 0.0 if abs(gamma) < 1e-12 else np.arctan2(py / gamma, px / gamma)
        q3 = gamma - D * np.cos(q4) + L1 - L3
        return (q1, q2, q3, q4)

    if branch == "+":
        return solve_branch(+1)
    if branch == "-":
        return solve_branch(-1)

    best, best_err = None, np.inf
    for sign in (+1, -1):
        sol = solve_branch(sign)
        err = np.linalg.norm(inspector_robot.fkine(to_full(*sol)).A - T)
        if err < best_err:
            best, best_err = sol, err
    return best


def validar(n=8, seed=0, verbose=True):
    rng = np.random.default_rng(seed)
    max_pose_err = 0.0
    if verbose:
        print("\nValidação da cinemática inversa\n" + "-" * 60)
    for t in range(n):
        q1 = rng.uniform(-np.pi, np.pi)
        q2 = rng.uniform(*LINK2_LIMITS)
        q3 = rng.uniform(*LINK3_LIMITS)
        q4 = rng.uniform(*LINK4_LIMITS)

        T = inspector_robot.fkine(to_full(q1, q2, q3, q4)).A
        q_ik = ikine_inspector(T, branch="auto")
        T_chk = inspector_robot.fkine(to_full(*q_ik)).A

        pose_err = np.linalg.norm(T_chk - T)
        max_pose_err = max(max_pose_err, pose_err)
        if verbose:
            print(f"teste {t}:  erro de pose = {pose_err:.2e}")

    if verbose:
        print("-" * 60)
        print(f"Maior erro de pose: {max_pose_err:.2e}  "
              f"-> {'OK' if max_pose_err < 1e-9 else 'FALHOU'}")
    return max_pose_err


if __name__ == "__main__":
    validar(n=8)

    print("\nExemplo de uso direto da IK\n" + "-" * 60)
    q_alvo = (0.5, 0.10, 0.08, 0.4)
    T_alvo = inspector_robot.fkine(to_full(*q_alvo)).A
    print("Pose alvo T:\n", np.round(T_alvo, 4))

    q_rec = ikine_inspector(T_alvo, branch="auto")
    print("\nq desejado  :", np.round(q_alvo, 4))
    print("q recuperado:", np.round(q_rec, 4))

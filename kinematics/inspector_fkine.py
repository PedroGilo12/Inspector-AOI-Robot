import numpy as np
import roboticstoolbox as rtb
import sympy as sp

LINK2_LIMITS = [0.0, 0.18]
LINK3_LIMITS = [0.0, 0.18]
LINK4_LIMITS = [-np.pi/2, np.pi/2]

# Demonstração da cinemática direta simplificada
# -------

L1, L2, L3, L4, LC,DF = sp.symbols('L1 L2 L3 L4 LC DF')
q1, q2, q3, q4 = sp.symbols('q1 q2 q3 q4')

link1 = rtb.RevoluteDH(d=L2, alpha=0, a=-L1)
link2 = rtb.PrismaticDH(theta=0.0, alpha=-sp.pi/2, a=0.0, qlim=LINK2_LIMITS)
virtual1 = rtb.PrismaticDH(theta=-sp.pi/2, alpha=-sp.pi/2, a=0.0, qlim=[0, 0])
link3 = rtb.PrismaticDH(theta=0.0, alpha=sp.pi/2, a=0.0, qlim=LINK3_LIMITS, offset=L3)
link4 = rtb.RevoluteDH(d=0, alpha=0, a=-(L4 + LC + DF), offset=-sp.pi/2)
virtual2 = rtb.PrismaticDH(theta=sp.pi/2, alpha=-sp.pi/2, a=0.0, qlim=[0, 0])

inspector_robot = rtb.DHRobot([link1, link2, virtual1, link3, link4, virtual2], name="InspectorDH")

q_sym = [q1, q2, 0, q3, q4, 0]

T = inspector_robot.fkine(q_sym)

matriz_pura = sp.Matrix(T.A)

matriz_limpa = matriz_pura.applyfunc(lambda x: sp.N(x, chop=1e-10))

matriz_final = sp.trigsimp(matriz_limpa)

print("--- Matriz de Transformação Homogênea (SE3) ---")
sp.pprint(matriz_final)

# Matrizes de transformação entre juntas consecutivas (frame i -> frame i+1)
def limpa(matriz):
    return sp.trigsimp(matriz.applyfunc(lambda x: sp.N(x, chop=1e-10)))

for i, link in enumerate(inspector_robot.links):
    A_i = limpa(sp.Matrix(link.A(q_sym[i]).A))
    print(f"\n--- Transformação da junta {i} -> {i+1} ---")
    sp.pprint(A_i)
# -------


# Demonstração da cinemática interativa
# -------
L1 = 0.2
L2 = 0.06
L3 = 0.06
L4 = 0.01
LC = 0.0
DF = 0.0

link1 = rtb.RevoluteDH(d=L2, alpha=0, a=-L1)
link2 = rtb.PrismaticDH(theta=0.0, alpha=-np.pi/2, a=0.0, qlim=LINK2_LIMITS)
virtual1 = rtb.PrismaticDH(theta=-np.pi/2, alpha=-np.pi/2, a=0.0, qlim=[0, 0])
link3 = rtb.PrismaticDH(theta=0.0, alpha=np.pi/2, a=0.0, qlim=LINK3_LIMITS, offset=L3)
link4 = rtb.RevoluteDH(d=0, alpha=0, a=-(L4 + LC + DF), offset=-np.pi/2, qlim=LINK4_LIMITS)
virtual2 = rtb.PrismaticDH(theta=np.pi/2, alpha=-np.pi/2, a=0.0, qlim=[0, 0])

inspector_robot = rtb.DHRobot([link1, link2, virtual1, link3, link4, virtual2], name="InspectorDH")
print(inspector_robot)

q_test = [0, 0, 0, 0, 0, 0]

inspector_robot.teach(q_test, block=True)

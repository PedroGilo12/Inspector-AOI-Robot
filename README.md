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
- **Interface serial (UART)** a 115200 bps, com comandos de movimentação linear
  cartesiana (`G0`/`G1`, que usam a cinemática inversa interna) e de movimentação
  direta das juntas (`G7`, com parâmetros `A`, `B`, `C`, `D`).

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
├── kinematics/
│   ├── inspector_fkine.py          # Cinemática direta (simbólica + visualização)
│   └── inspector_ikine.py          # Cinemática inversa analítica + validação
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

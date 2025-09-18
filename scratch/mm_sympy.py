# import sympy as sp

# # Define symbols
# A, B, C, H, N, i = sp.symbols('A B C H N i')
# # First matrix

# T_fill = sp.Matrix([
#     [0,0,0,1]
# ])

# R_OH = sp.Matrix([
#     [sp.sin(H), sp.cos(H), 0],
#     [-sp.cos(H), sp.sin(H), 0],
#     [0,0,1]
# ])
# P_OHi = sp.Matrix([
#     [sp.cos(60*(4-i))],
#     [sp.sin(60*(4-i))],
#     [0]
# ])
# T_OHi = R_OH.row_join(P_OHi).col_join(T_fill)

# R_HA = sp.Matrix([
#     [0,0,1],
#     [sp.cos(A), -sp.sin(A), 0],
#     [sp.sin(A), sp.cos(A), 0]
# ])
# P_HA = sp.Matrix([
#     [0],
#     [52.2],
#     [25.7]
# ])
# T_HA = R_HA.row_join(P_HA).col_join(T_fill)

# R_AB = sp.Matrix([
#     [0,0,1],
#     [-sp.sin(B), -sp.cos(B), 0],
#     [sp.cos(B), -sp.sin(B), 0]
# ])
# P_AB = sp.Matrix([
#     [38.2],
#     [0],
#     [0]
# ])
# T_AB = R_AB.row_join(P_AB).col_join(T_fill)

# R_BC = sp.Matrix([
#     [0,0,1],
#     [-sp.sin(C), -sp.cos(C), 0],
#     [sp.cos(C), -sp.sin(C), 0]
# ])
# P_BC = sp.Matrix([
#     [0],
#     [10],
#     [64.2]
# ])
# T_BC = R_BC.row_join(P_BC).col_join(T_fill)

# R_CD = sp.Matrix([
#     [0,1,0],
#     [0,0,1],
#     [1,0,0]
# ])
# P_CD = sp.Matrix([
#     [120],
#     [0],
#     [0]
# ])


# print("Transformation Matrices:\n")
# print("\nT_OHi:")
# sp.pretty_print(T_OHi)
# print("\nT_HA:")
# sp.pretty_print(T_HA)
# print("\nT_AB:")
# sp.pretty_print(T_AB)
# print("\nT_BC:")
# sp.pretty_print(T_BC)


# print("\nT_OC = T_OHi * T_HA * T_AB * T_BC:")
# T_OC = T_OHi * T_HA * T_AB * T_BC
# sp.pretty_print(T_OC, wrap_line=False)

# print("\nT_OC*P_CD:")
# M1 = T_OC * (P_CD.col_join(sp.Matrix([[1]])))
# sp.pretty_print(M1, wrap_line=False)


# print("\nR_OD = R_OH * R_HA * R_AB * R_BC * R_CD:")
# R_OD = R_OH * R_HA * R_AB * R_BC * R_CD
# sp.pretty_print(R_OD, wrap_line=False)


import sympy as sp

# ----------------------------
# Step 0: Define symbols
# ----------------------------
A, B, C, H, N, i = sp.symbols('A B C H N i')
X, Y, Z = sp.symbols('X Y Z')       # symbolic end-effector target

# ----------------------------
# Step 1: Define transformation matrices
# ----------------------------
T_fill = sp.Matrix([[0,0,0,1]])

R_OH = sp.Matrix([
    [sp.sin(H), sp.cos(H), 0],
    [-sp.cos(H), sp.sin(H), 0],
    [0,0,1]
])
o_x, o_y = sp.symbols('o_x o_y')
# P_OHi = sp.Matrix([[sp.cos(60*(4-i))],
#                    [sp.sin(60*(4-i))],
#                    [0]])
P_OHi = sp.Matrix([[o_x],[o_y],[0]])
T_OHi = R_OH.row_join(P_OHi).col_join(T_fill)

R_HA = sp.Matrix([
    [0,0,1],
    [sp.cos(A), -sp.sin(A), 0],
    [sp.sin(A), sp.cos(A), 0]
])
ay, az = sp.symbols('a_y a_z')
# P_HA = sp.Matrix([[0],[52.2],[25.7]])
P_HA = sp.Matrix([[0],[ay],[az]])
T_HA = R_HA.row_join(P_HA).col_join(T_fill)

R_AB = sp.Matrix([
    [0,0,1],
    [-sp.sin(B), -sp.cos(B), 0],
    [sp.cos(B), -sp.sin(B), 0]
])
bx = sp.symbols('b_x')
# P_AB = sp.Matrix([[38.2],[0],[0]])
P_AB = sp.Matrix([[bx],[0],[0]])
T_AB = R_AB.row_join(P_AB).col_join(T_fill)

R_BC = sp.Matrix([
    [0,0,1],
    [-sp.sin(C), -sp.cos(C), 0],
    [sp.cos(C), -sp.sin(C), 0]
])
c_y, c_z = sp.symbols('c_y c_z')
# P_BC = sp.Matrix([[0],[10],[64.2]])
P_BC = sp.Matrix([[0],[c_y],[c_z]])
T_BC = R_BC.row_join(P_BC).col_join(T_fill)

R_CD = sp.Matrix([
    [0,1,0],
    [0,0,1],
    [1,0,0]
])
dx = sp.symbols('d_x')
# P_CD = sp.Matrix([[120],[0],[0]])
P_CD = sp.Matrix([[dx],[0],[0]])
# Forward Kinematics
T_OC = T_OHi * T_HA * T_AB * T_BC
M1 = T_OC * P_CD.col_join(sp.Matrix([[1]]))



print("Transformation Matrices:\n")
print("\nT_OHi:")
sp.pretty_print(T_OHi)
print("\nT_HA:")
sp.pretty_print(T_HA)
print("\nT_AB:")
sp.pretty_print(T_AB)
print("\nT_BC:")
sp.pretty_print(T_BC)

print("\nT_OC = T_OHi * T_HA * T_AB * T_BC:")
sp.pretty_print(T_OC, wrap_line=False)

print("\nT_OC*P_CD:")
sp.pretty_print(M1, wrap_line=False)


print("\nR_OD = R_OH * R_HA * R_AB * R_BC * R_CD:")
R_OD = R_OH * R_HA * R_AB * R_BC * R_CD
sp.pretty_print(R_OD, wrap_line=False)


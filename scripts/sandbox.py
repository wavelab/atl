#!/usr/bin/env python
import sympy
from sympy import pprint

# # TWO WHEEL MODEL
# f1, f2, f3, f4, f5 = sympy.symbols("f1,f2,f3,f4,f5")
# x1, x2, x3, x4, x5 = sympy.symbols("x1,x2,x3,x4,x5")
# dt = sympy.symbols("dt")
#
# # x, y, theta, v, omega
# f1 = x1 + x4 * sympy.cos(x3) * dt
# f2 = x2 + x4 * sympy.sin(x3) * dt
# f3 = x3 + x5 * dt
# f4 = x4
# f5 = x5
#
# F = sympy.Matrix([f1, f2, f3, f4, f5])
# pprint(F.jacobian([x1, x2, x3, x4, x5]))


# f1, f2, f3, f4, f5, f6, f7 = sympy.symbols("f1,f2,f3,f4,f5,f6,f7")
# x1, x2, x3, x4, x5, x6, x7 = sympy.symbols("x1,x2,x3,x4,x5,x6,x7")
# dt = sympy.symbols("dt")
#
# # x, y, z, theta, v, omega, vz
# f1 = x1 + x4 * sympy.cos(x4) * dt
# f2 = x2 + x4 * sympy.sin(x4) * dt
# f3 = x3 + x7 * dt
# f4 = x4 + x6 * dt
# f5 = x5
# f6 = x6
# f7 = x7
#
# F = sympy.Matrix([f1, f2, f3, f4, f5, f6, f7])
# pprint(F.jacobian([x1, x2, x3, x4, x5, x6, x7]))

x1, x2, x3 = sympy.symbols("x1,x2,x3")
theta = sympy.symbols("theta")

R = sympy.Matrix([
    [sympy.cos(x3), -sympy.sin(x3), 0],
    [sympy.sin(x3), sympy.cos(x3), 0],
    [0, 0, 1]
])

pprint(R.jacobian(R))

# R.jacobian(sympy.Matrix([
#     [x1, x1, x1],
#     [x1, x1, x1],
#     [x1, x1, x1]
# ]))

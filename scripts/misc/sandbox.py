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
# # x1 - x
# # x2 - y
# # x3 - z
# # x4 - theta
# # x5 - v
# # x6 - omega
# # x7 - vz
#
# # x, y, z, theta, v, omega, vz
# f1 = x1 + x5 * sympy.cos(x4) * dt
# f2 = x2 + x5 * sympy.sin(x4) * dt
# f3 = x3 + x7 * dt
# f4 = x4 + x6 * dt
# f5 = x5
# f6 = x6
# f7 = x7
#
# F = sympy.Matrix([f1, f2, f3, f4, f5, f6, f7])
# pprint(F.jacobian([x1, x2, x3, x4, x5, x6, x7]))

functions = sympy.symbols("f1,f2,f3,f4,f5,f6,f7,f8,f9")
variables = sympy.symbols("x1,x2,x3,x4,x5,x6,x7,x8,x9")

f1, f2, f3, f4, f5, f6, f7, f8, f9 = functions
x1, x2, x3, x4, x5, x6, x7, x8, x9 = variables
dt = sympy.symbols("dt")

# x1 - x
# x2 - y
# x3 - z

# x4 - theta
# x5 - v
# x6 - vz

# x7 - omega
# x8 - a
# x9 - az

f1 = x1 + x5 * sympy.cos(x4) * dt
f2 = x2 + x5 * sympy.sin(x4) * dt
f3 = x3 + x6 * dt

f4 = x4 + x7 * dt
f5 = x5 + x8 * dt
f6 = x6 + x9 * dt

f7 = x7
f8 = x8
f9 = x9

F = sympy.Matrix([f1, f2, f3, f4, f5, f6, f7, f8, f9])
pprint(F.jacobian([x1, x2, x3, x4, x5, x6, x7, x8, x9]))

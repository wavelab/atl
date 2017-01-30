#!/usr/bin/env python
import sympy
from sympy import pprint

f1, f2, f3, f4, f5 = sympy.symbols("f1,f2,f3,f4,f5")
x1, x2, x3, x4, x5 = sympy.symbols("x1,x2,x3,x4,x5")
dt = sympy.symbols("dt")

# x, y, theta, v, omega
f1 = x1 + x4 * sympy.cos(x3) * dt
f2 = x2 + x4 * sympy.sin(x3) * dt
f3 = x3 + x5 * dt
f4 = x4
f5 = x5

F = sympy.Matrix([f1, f2, f3, f4, f5])
pprint(F.jacobian([x1, x2, x3, x4, x5]))

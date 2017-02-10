#!/usr/bin/env python3
from math import sqrt

import numpy as np


def cross_track_error(pos, start, end):
    x0, y0 = pos
    x1, y1 = start
    x2, y2 = end

    numerator = ((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1)
    denominator = sqrt(pow(y2 - y1, 2) + pow(x2 - x1, 2))
    return numerator / denominator


pos = np.array([2, 2])
start = np.array([1, 1])
end = np.array([5, 5])
print(cross_track_error(pos, start, end))


# quad = np.array([3, 3])
# result = (start - quad)
# print(result)

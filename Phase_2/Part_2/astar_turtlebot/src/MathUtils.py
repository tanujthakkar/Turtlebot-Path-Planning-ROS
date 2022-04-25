import numpy as np
import math
from Obstacle import *


def half_round(n):
    val = round(2*n)/2
    if (val == 10):
      val -= 0.5
    return val

def to_rad(deg):
    return np.pi * deg / 180

def to_deg(rad):
    return 180 * rad / np.pi
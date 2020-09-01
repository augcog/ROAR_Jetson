from scipy.optimize import minimize
from scipy.optimize import curve_fit

from numpy import log, exp
from scipy.optimize.minpack import leastsq

import numpy as np

def f(x, a, b, c, d):
    return a * np.exp(b * x) + c * np.exp(d * x)

# def fit(x, y):
#     a, b, c, d = curve_fit(f, x, y, p0=(1, 1, 1, 1))
#     return a, b, c, d

## regression function
def _exp(a, b, c, d):
    """
    Exponential function y = a * exp(b * x) + c * exp(d * x)
    """
    return lambda x: a * exp(b * x) + c * exp(d * x)

def fit(x, y):
    fun = _exp
    df = {'x': x, 'y': y}
    resid = lambda p, x, y: y - fun(*p)(x)
    ls = leastsq(resid, [1.0, 1.0, 1.0, 1.0], args=(df['x'], df['y']))
    a, b, c, d = ls[0]
    return a, b, c, d

def construct_f(a, b, c, d):
    def f(x):
        # print(type(a), type(b), type(c), type(p), type(q))
        return a*np.exp(b*x) + c*np.exp(d*x)
    return f

## interpolation
def interpolate(x, df, fun=_exp):
    """
    Interpolate Y from X based on df, a dataframe with columns 'x' and 'y'.
    """
    resid = lambda p, x, y: y - fun(*p)(x)
    ls = leastsq(resid, [1.0, 1.0, 1.0, 1.0], args=(df['x'], df['y']))
    a, b, c, d = ls[0]
    y = fun(a, b, c, d)(x)
    return y, a, b, c, d
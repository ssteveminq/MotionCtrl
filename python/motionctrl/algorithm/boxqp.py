import numpy as np
import os
import sys
sys.path.append(os.path.abspath('../'))
from utility.general_utils import deleteRowCol


def boxQP(H, g, lower, upper, x0):
    n = H.shape[0]
    clamped = np.zeros(n, dtype=bool)
    free = np.ones(n, dtype=bool)
    oldvalue = 0
    result = 0
    gnorm = 0
    nfactor = 0
    Hfree = np.zeros(n)
    x = np.clip(x0, lower, upper)
    y = np.zeros(n)
    np.isfinite(x, y)
    for e in y:
        if e == 0:
            x[e] = 0
    maxIter = 100
    minGrad = 1e-7
    minRelImprove = 1e-7
    stepDec = 0.6
    minStep = 1e-22
    Armijo = 0.1
    value = np.dot(x.T, g) + 0.5 * np.dot(x.T, np.dot(H, x))

    for it in range(maxIter):
        if result != 0:
            break

        if (it > 0) and ((oldvalue - value) < minRelImprove*np.abs(oldvalue)):
            result = 4
            break

        oldvalue = value
        grad = g + np.dot(H, x)
        old_clamped = np.copy(clamped)
        clamped = np.zeros(n, dtype=bool)

        for i in range(n):
            if ((x[i] == lower[i]) and (grad[i] > 0)) or ((x[i] == upper[i]) and (grad[i] < 0)):
                clamped[i] = True
        free = ~clamped

        if clamped.all():
            result = 6
            break

        if it == 0:
            factorize = True
        else:
            factorize = (old_clamped != clamped).any()

        if factorize:
            H_free = deleteRowCol(H, clamped)
            Hfree = np.linalg.cholesky(H_free)

        gnorm = np.linalg.norm(grad[free])
        if gnorm < minGrad:
            result = 5
            break

        grad_clamped = g + np.dot(H, x*clamped)
        search = np.zeros(n)
        search[free] = np.dot(-np.linalg.inv(Hfree),
                np.dot(np.linalg.inv(Hfree.T), grad_clamped[free])) - x[free]

        sdotg = np.sum(search*grad)
        if sdotg >= 0:
            break
        step = 1
        nstep = 0
        xc = np.clip(x+step*search, lower, upper)
        vc = np.dot(xc, g) + 0.5*np.dot(np.dot(xc, H), xc)
        while (vc - oldvalue)/(step*sdotg) < Armijo:
            step *= stepDec
            nstep += 1
            xc = np.clip(x+step*search, lower, upper)
            vc = np.dot(xc, g) + 0.5*np.dot(np.dot(xc, H), xc)
            if step<minStep:
                result=2
                break
        x = xc
        value = vc
    if it >= maxIter:
        result=1

    return x, result, Hfree, free

if __name__ == "__main__":
    H = np.array([[1.02, 0.0], [0.0, 1.0002]])
    g = np.array([0.0237, 0.0075])
    lower = np.array([-0.6, -2.1])
    upper = np.array([0.4, 1.9])
    x0 = np.array([0, 0])
    x, result, Hfree, free = boxQP(H, g, lower, upper, x0)
    print(x)
    print(result)
    print(Hfree)
    print(free)

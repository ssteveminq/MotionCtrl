import numpy as np

def sabs(x, p):
    return np.sqrt(x*x + p*p) - p

def isDiverge(x, thres):
    return (np.abs(x) > thres).any()

def generate_noise(policy):
    raise NotImplementedError("TODO generate noise")

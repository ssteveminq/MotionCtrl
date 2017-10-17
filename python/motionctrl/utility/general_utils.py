import numpy as np

def sabs(x, p):
    return np.sqrt(x*x + p*p) - p

def isDiverge(x, thres):
    return (np.abs(x) > thres).any()

def generate_noise(policy):
    raise NotImplementedError(" TODO : generate noise")

def finite_differences(func, inputs, func_output_shape=(), epsilon=2**(-17)):
    """
    Computes gradients via finite differences.
    derivative = (func(x+epsilon) - func(x-epsilon)) / (2*epsilon)
    Args:
        func: Function to compute gradient of. Inputs and outputs can be
            arbitrary dimension.
        inputs: Vector value to compute gradient at.
        func_output_shape: Shape of the output of func. Default is
            empty-tuple, which works for scalar-valued functions.
        epsilon: Difference to use for computing gradient.
    Returns:
        Gradient vector of each dimension of func with respect to each
        dimension of input.
    """
    gradient = np.zeros(inputs.shape+func_output_shape)
    for idx, _ in np.ndenumerate(inputs):
        test_input = np.copy(inputs)
        test_input[idx] += epsilon
        obj_d1 = func(test_input)
        assert obj_d1.shape == func_output_shape
        test_input = np.copy(inputs)
        test_input[idx] -= epsilon
        obj_d2 = func(test_input)
        assert obj_d2.shape == func_output_shape
        diff = (obj_d1 - obj_d2) / (2 * epsilon)
        gradient[idx] += diff
    return gradient

def deleteRowCol(A, idx_array):
    """
    Delete rows and cols of 2D array
    Args:
        A: 2D numpy array
        idx_array: 1D numpy boolean array
                   Rows and Cols corresponding to 1 is deleted
    Returns:
        A_del : Deleted 2D array
    """
    A_del = np.copy(A)
    idx = (np.where(idx_array==True))[0]
    for i in np.flip(idx, axis=0):
        A_del = np.delete(np.delete(A_del, i, axis=0), i, axis=1)
    return A_del


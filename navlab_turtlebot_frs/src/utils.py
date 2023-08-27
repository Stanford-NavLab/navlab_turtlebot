import numpy as np

def remove_zero_columns(A):
    """Remove all zeros columns from an array
    
    Parameters
    ----------
    A : np.array (2D)
        Input array
    
    Returns
    -------
    np.array 
        Array with all zeros columns removed

    """
    zero_idx = np.argwhere(np.all(A[...,:]==0, axis=0))
    return np.delete(A, zero_idx, axis=1)
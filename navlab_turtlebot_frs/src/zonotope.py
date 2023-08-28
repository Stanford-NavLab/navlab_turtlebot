import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import itertools

from utils import remove_zero_columns

class Zonotope(object):
    """ Zonotope class

    Attributes
    ----------
    dim : int
        Dimension (denoted as n)
    order : int
        Number of generators (denoted as m)
    c : np.array (n x 1)
        Center
    G : np.array (n x m)
        Generators
    Z : np.array (n x m+1)
        Matrix form [c, G]

    Methods
    -------


    Example usage:
        z = Zonotope(np.zeros((2,1)),np.eye(2))

    """
    __array_priority__ = 1000  # Prioritize class mul over numpy array mul

    def __init__(self, center, generators):
        """ Constructor"""
        self.c = center 
        self.G = generators
        self.Z = np.hstack((center, generators))
        self.dim = center.shape[0]
        self.n_gen = generators.shape[1]


    ### ====== Printing ====== ###
    def __str__(self):
        #return "center:\n {0} \n generators:\n {1}".format(self.c, self.G)
        np.set_printoptions(precision=3)
        ind = '\t'
        c_str = ind + str(self.c).replace('\n','\n' + ind)
        G_str = ind + str(self.G).replace('\n','\n' + ind)
        print_str = 'Center:\n' + c_str + '\nGenerators:\n' + G_str 
        return print_str


    ### ====== Operations ====== ###
    def __add__(self, other):
        """ Minkowski addition (overloads '+') """
        # Other is a vector
        if type(other) == np.ndarray:
            c = self.c + other
            G = self.G 
        # Other is a zonotope
        else:
            c = self.c + other.c
            G = np.hstack((self.G, other.G))
        return Zonotope(c,G)


    def __rmul__(self, other):
        """ Right linear map (overloads '*') """
        # Other is a scalar
        if np.isscalar(other):
            c = other * self.c
            G = other * self.G 
        # Other is a matrix
        elif type(other) is np.ndarray:
            c = np.matmul(other, self.c)
            G = np.matmul(other, self.G)
        return Zonotope(c,G) 
    

    def __mul__(self, other):
        """ (Left) linear map (overloads '*') """
        # Other is a scalar
        if np.isscalar(other):
            c = other * self.c
            G = other * self.G 
        # Other is a matrix
        elif type(other) is np.ndarray:
            c = np.matmul(self.c, other)
            G = np.matmul(self.G, other)
        return Zonotope(c,G) 


    def sample(self, n_points):
        """Sample
        
        Randomly sample points from the interior of the zonotope

        """
        c = self.c
        G = self.G 
        factors = -1 + 2 * np.random.rand((G.shape[1],n_points))
        p = c + np.matmul(G, factors)
        return p

    
    def augment(self, Z):
        """Augment with another zonotope

        Stacks center and generators together to form new zonotope
        
        Parameters
        ----------
        Z : Zonotope
            Zonotope to augment with (must have same order)

        Returns
        -------
        Zonotope
            Augmented zonotope

        """
        c = np.vstack((self.c, Z.c))
        G = np.vstack((self.G, Z.G))
        return Zonotope(c,G)

    
    def slice(self, dim, slice_pt):
        """Slice zonotope along dim 
        
        Parameters
        ----------
        dim : int or list
            Dimension(s) to slice
        slice_pt : np.array
            Point to slice at
        
        Returns
        -------
        Zonotope
            Sliced zonotope

        """
        c = self.c; G = self.G
        slice_idx = []

        for i in range(len(dim)):
            myidxs = np.nonzero(G[dim[i]])[0]  # Find non-zero generators in slice dimension
            if len(myidxs) != 1:
                if len(myidxs) == 0:
                    print('No generator for slice index')
                    return None
                else:
                    print('Multiple generators for slice index')
            slice_idx.append(myidxs[0])

        slice_c = c[dim]
        slice_G = G[dim,:][:,slice_idx]
        slice_lambda = np.linalg.solve(slice_G, slice_pt - slice_c)  # Calculate coefficients for slice
        if slice_lambda.shape[1] > 1:
            print('slice_lambda is not 1D')
        if np.any(np.abs(slice_lambda ) > 1):
            print('Slice point is outside of bound of zonotope')

        newG = G
        newG = np.delete(newG, slice_idx, axis=1)
        newc = c+ np.matmul(G[:,slice_idx], slice_lambda)

        return Zonotope(newc, newG)

    
    def view(self, dim):
        """View zonotope in sub dimensions
        
        Parameters
        ----------
        dim : int or list
            Dimension(s) to view

        Returns
        -------
        Zonotope
            Sub-zonotope

        """
        c = self.c[dim]
        G = self.G[dim]
        return Zonotope(c,G)


    def halfspace(self):
        """Generate halfspace representation A*x <= b

        Supports dim <= 3 (i.e. 1,2,3). Intended for full rank zonotopes (rank(G) >= n).
        
        Returns
        -------
        A : np.array () 
        b : np.array ()

        """
        # Extract variables
        c = self.c
        G = self.G
        n = self.dim
        m = self.n_gen

        assert n <= 3, "Dimension not supported."   
        assert np.linalg.matrix_rank(G) >= n, "Generator matrix is not full rank." + str(G)

        if n > 1:
            # Build C matrices
            if n == 2:
                C = G
                C = np.vstack((-C[1,:], C[0,:]))  # get perpendicular vector
            elif n == 3:
                comb = np.asarray(list(itertools.combinations(np.arange(m), n-1)))
                # Cross-product in matrix form
                Q = np.vstack((G[:,comb[:,0]], G[:,comb[:,1]]))
                C = np.vstack((Q[1,:] * Q[5,:] - Q[2,:] * Q[4,:],
                             -(Q[0,:] * Q[5,:] - Q[2,:] * Q[3,:]),
                               Q[0,:] * Q[4,:] - Q[1,:] * Q[3,:]))
            # TODO: remove nans
        else:
            C = G
        
        # Normalize normal vectors
        C = np.divide(C, np.linalg.norm(C, axis=0)).T
        # Build d vector
        deltaD = np.sum(np.abs(np.matmul(C, G)).T, axis=0)[:,None]
        # Compute dPos, dNeg
        d = np.matmul(C, c)

        A = np.vstack((C, -C))
        b = np.vstack((d + deltaD, -d + deltaD))
        return A, b


    def contains(self, x):
        """Check if point x is contained in zonotope.

        Method 1: convert to halfspace
        Method 2: solve for coefficients (minimization)
        
        Parameters
        ----------
        x : np.array (dim x 1)
            Point to check for containment

        Returns
        -------
        bool
            True if x in zonotope, False if not.

        """
        A, b = self.halfspace()
        return np.all(np.matmul(A, x) <= b)


    def delete_zeros(self):
        """Remove all zeros generators
        
        """
        self.G = remove_zero_columns(self.G)


    ### ====== Properties ====== ### 
    def vertices(self):
        """ Vertices of zonotope 
        
        Adapted from CORA \@zonotope\vertices.m and \@zonotope\polygon.m
        Tested on 2D zonotopes (n==2)

        Returns
        -------
        V : np.array
            Vertices 

        """
        # Extract variables
        c = self.c
        G = self.G
        n = self.dim
        m = self.n_gen

        if n == 1:
            # Compute the two vertices for 1-dimensional case
            temp = np.sum(np.abs(self.G))
            V = np.array([self.c - temp, self.c + temp])
        elif n == 2:
            # Obtain size of enclosing intervalhull of first two dimensions
            xmax = np.sum(np.abs(G[0,:]))
            ymax = np.sum(np.abs(G[1,:]))

            # Z with normalized direction: all generators pointing "up"
            Gnorm = G
            Gnorm[:,G[1,:]<0] = Gnorm[:,G[1,:]<0] * -1

            # Compute angles
            angles = np.arctan2(G[1,:],G[0,:])
            angles[angles<0] = angles[angles<0] + 2 * np.pi

            # Sort all generators by their angle
            IX = np.argsort(angles)

            # Cumsum the generators in order of angle
            V = np.zeros((2,m+1))
            for i in range(m):
                V[:,i+1] = V[:,i] + 2 * Gnorm[:,IX[i]] 

            V[0,:] = V[0,:] + xmax - np.max(V[0,:])
            V[1,:] = V[1,:] - ymax 

            # Flip/mirror upper half to get lower half of zonotope (point symmetry)
            V = np.block([[V[0,:], V[0,-1] + V[0,0] - V[0,1:]],
                          [V[1,:], V[1,-1] + V[1,0] - V[1,1:]]])

            # Consider center
            V[0,:] = c[0] + V[0,:]
            V[1,:] = c[1] + V[1,:]

        else:
            #TODO: delete aligned and all-zero generators

            # Check if zonotope is full-dimensional
            if self.n_gen < n:
                #TODO: verticesIterateSVG
                print("Vertices for non full-dimensional zonotope not implemented yet - returning empty array")
                V = np.empty()
                return V
            
            # Generate vertices for a unit parallelotope
            vert = np.array(np.meshgrid([1, -1], [1, -1], [1, -1])).reshape(3,-1)
            V = c + np.matmul(G[:,:n], vert)
            
            #TODO: rest unimplemented

        return V
            

    ### ====== Plotting ====== ###
    def plot(self, ax=None, color='b', alpha=0.2, line_alpha=1):
        """Plot function 
        
        Parameters 
        ----------
        ax : matplotlib.axes
            Axes to plot on, if unspecified, will generate and plot on new set of axes
        color : color 
            Plot color
        alpha : float (from 0 to 1)
            Patch transparency

        """
        V = self.vertices()
        xmin = np.min(V[0,:]); xmax = np.max(V[0,:])
        ymin = np.min(V[1,:]); ymax = np.max(V[1,:])

        if ax == None:
            fig, ax = plt.subplots()
        poly = Polygon(V.T, closed=True, fill=True, color=color, alpha=alpha)
        poly_edge = Polygon(V.T, closed=True, fill=False, color=color, alpha=line_alpha)
        ax.add_patch(poly)
        ax.add_patch(poly_edge)

        # Recompute the ax.dataLim
        ax.relim()
        # Update ax.viewLim using the new dataLim
        ax.autoscale_view()
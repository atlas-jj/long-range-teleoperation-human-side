import numpy as np
from scipy.spatial.distance import cdist

def best_fit_transform(A, B):
    '''
    Calculates the least-squares best-fit transform between corresponding 3D points A->B
    Input:
      A: Nx3 numpy array of corresponding 3D points
      B: Nx3 numpy array of corresponding 3D points
    Returns:
      T: 4x4 homogeneous transformation matrix
      R: 3x3 rotation matrix
      t: 3x1 column vector
    '''

    assert len(A) == len(B)

    # translate points to their centroids
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - centroid_A
    BB = B - centroid_B

    # rotation matrix
    H = np.dot(AA.T, BB)
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)

    # special reflection case
    if np.linalg.det(R) < 0:
       Vt[2,:] *= -1
       R = np.dot(Vt.T, U.T)

    # translation
    t = centroid_B.T - np.dot(R,centroid_A.T)

    # homogeneous transformation
    T = np.identity(4)
    T[0:3, 0:3] = R
    T[0:3, 3] = t

    return T, R, t

def nearest_neighbor(src, dst):
    '''
    Find the nearest (Euclidean) neighbor in dst for each point in src
    Input:
        src: Nx3 array of points
        dst: Nx3 array of points
    Output:
        distances: Euclidean distances of the nearest neighbor
        indices: dst indices of the nearest neighbor
    '''

    all_dists = cdist(src, dst, 'euclidean')
    indices = all_dists.argmin(axis=1)
    distances = all_dists[np.arange(all_dists.shape[0]), indices]
    return distances, indices

def icp(A, B, init_pose=None, max_iterations=20, tolerance=0.001):
    '''
    The Iterative Closest Point method
    Input:
        A: Nx3 numpy array of source 3D points
        B: Nx3 numpy array of destination 3D point
        init_pose: 4x4 homogeneous transformation
        max_iterations: exit algorithm after max_iterations
        tolerance: convergence criteria
    Output:
        T: final homogeneous transformation
        distances: Euclidean distances (errors) of the nearest neighbor
    '''

    # make points homogeneous, copy them so as to maintain the originals
    src = np.ones((4,A.shape[0]))
    dst = np.ones((4,B.shape[0]))
    src[0:3,:] = np.copy(A.T)
    dst[0:3,:] = np.copy(B.T)

    # apply the initial pose estimation
    if init_pose is not None:
        src = np.dot(init_pose, src)

    prev_error = 0

    for i in range(max_iterations):
        # find the nearest neighbours between the current source and destination points
        distances, indices = nearest_neighbor(src[0:3,:].T, dst[0:3,:].T)

        # compute the transformation between the current source and nearest destination points
        T,_,_ = best_fit_transform(src[0:3,:].T, dst[0:3,indices].T)

        # update the current source
        src = np.dot(T, src)

        # check error
        mean_error = np.sum(distances) / distances.size
        if abs(prev_error-mean_error) < tolerance:
            break
        prev_error = mean_error

    # calculate final transformation
    T,_,_ = best_fit_transform(A, src[0:3,:].T)

    return T, distances

f1 = open('p2.txt')
lines = f1.readlines()
A = []
B = []
count = 1
for line in lines:
    data = line.strip().split(' ')
    A.append([float(data[2]),float(data[3]),float(data[4])])
    B.append([float(data[5]),float(data[6]),float(data[7])])
    count += 1

A = np.array(A)
B = np.array(B)
'''
A =np.array([[ 0.11707315,-0.13439331,0.73559262],
[0.11942668,-0.028190688,0.68043212],
[0.02285160, -0.17894039, 0.77051856],
[0.14316984,-0.1449433,0.7756667],
[ 0.1165853,0.0011135939,0.57949385],
[ 0.16015121,0.049821702, 0.56056792]
])



B =np.array([[-0.151,-0.465,0.449],
[-0.239,-0.382,0.413],
[-0.031,-0.433,0.437],
[-0.143,-0.509,0.439],
[-0.290,-0.305,0.473],
[-0.363,-0.303,0.465]
])'''



T2, R2, t2=best_fit_transform(A,B)


'''
T, distances=icp(A,B)
print ("T ICP")
print (T)
print ("distances ICP")
print (distances)
'''

print ("T2")
print (T2)
print ("R2")
print (R2)
print ("t2")
print (t2)




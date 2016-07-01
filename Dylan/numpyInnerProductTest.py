# script to test NumPy array operations

import numpy as np

# comment/uncomment to test different cases. Matrices are generated with 2*randn(4) matrices in Matlab

# first random array can use either to test diff cases.
# A = np.array([[-2.1377, 0.6504, -0.2045,  -1.7298],
			# [-1.6190, -1.5099, -0.4829, -0.0601],
			# [-5.8886, 2.7406, 0.6384, -0.3298],
			# [2.8768, -3.4230, 0.6257, 1.2554]])

# second random array can use either to test diff cases.
A = np.array([[1.0753, 0.6375, 7.1568, 1.4508],
			[3.6678, -2.6154, 5.5389, -0.1261],
			[-4.5177, -0.8672, -2.6998, 1.4295],
			[1.7243, 0.6852, 6.0698, -0.4099]])

A1n = np.sqrt(np.inner(A[0,:],A[0,:]))
A2n = np.sqrt(np.inner(A[1,:],A[1,:]))
A3n = np.sqrt(np.inner(A[2,:],A[2,:]))
A4n = np.sqrt(np.inner(A[3,:],A[3,:]))

print "---------------------------------------------------------------"
print "check the single values for each row's magnitude"
print ""
print A1n, A2n, A3n, A4n

print "---------------------------------------------------------------"
print "double check the format of a row of Anorm -> should be 1x4 array, checks norm too"
print ""
print A[0,:]/A1n
print ""
print "norm", np.linalg.norm(A[0,:]/A1n)

print "---------------------------------------------------------------"
print "proper way to recreate '.*' operator from Matlab (elementwise matrix multiply)"
print ""
A1nNorm = np.multiply(A[0,:],A[0,:])
print A1nNorm

print "---------------------------------------------------------------"
print "check the matrix Anorm -> it return weird structure that's not a 4x4 array"
print ""
Anorm = [A[0,:]/A1n, A[1,:]/A2n, A[2,:]/A3n, A[3,:]/A4n]

print Anorm

print "---------------------------------------------------------------"
print "restructuring the array, this is incorrect: 3D (4x1x4) array, not 2D (4x4)"
print ""
array = np.array([[A[0,:]/A1n], [A[1,:]/A2n], [A[2,:]/A3n], [A[3,:]/A4n]])

print array
print array.shape

print "---------------------------------------------------------------"
print "correct, but not as easy to tell which direction it's concatenated"
print ""
concat = np.concatenate(([A[0,:]/A1n], [A[1,:]/A2n], [A[2,:]/A3n], [A[3,:]/A4n]), axis=0)

print concat
print concat.shape

print "---------------------------------------------------------------"
print "correct, and easy to tell how it is concatenated"
print ""
stackedArray = np.vstack(([A[0,:]/A1n], [A[1,:]/A2n], [A[2,:]/A3n], [A[3,:]/A4n]))

print stackedArray
print stackedArray.shape


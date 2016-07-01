import numpy as np

A = np.array([[-2.1377, 0.6504, -0.2045,  -1.7298],[-1.6190, -1.5099, -0.4829, -0.0601],[-5.8886, 2.7406, 0.6384, -0.3298],[2.8768, -3.4230, 0.6257, 1.2554]])

A1n = np.sqrt(np.inner(A[0,:],A[0,:]))
A2n = np.sqrt(np.inner(A[1,:],A[1,:]))
A3n = np.sqrt(np.inner(A[2,:],A[2,:]))
A4n = np.sqrt(np.inner(A[3,:],A[3,:]))

print "---------------------------------------------------------------"

print A1n, A2n, A3n, A4n

print "---------------------------------------------------------------"

Anorm = [A[0,:]/A1n, A[1,:]/A2n, A[2,:]/A3n, A[3,:]/A4n]

print Anorm

print "---------------------------------------------------------------"

print A[0,:]/A1n

print "---------------------------------------------------------------"

concat = np.concatenate(([A[0,:]/A1n], [A[1,:]/A2n], [A[2,:]/A3n], [A[3,:]/A4n]), axis=0)

print concat
print concat.shape

print "---------------------------------------------------------------"

array = np.array([[A[0,:]/A1n], [A[1,:]/A2n], [A[2,:]/A3n], [A[3,:]/A4n]])

print array
print array.shape

print "---------------------------------------------------------------"

stackedArray = np.vstack(([A[0,:]/A1n], [A[1,:]/A2n], [A[2,:]/A3n], [A[3,:]/A4n]))

print stackedArray
print stackedArray.shape

print "---------------------------------------------------------------"

dotProd = np.multiply(A[0,:],A[0,:])
print dotProd
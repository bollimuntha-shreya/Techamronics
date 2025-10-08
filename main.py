import numpy as np
import matplotlib.pyplot as plt
import time

li = 20                # Input Link (cranck)
l1 = 3.33 * li
l2 = 2.77 * li
l3 = 3.72 * li
l4 = 2.67 * li
l5 = 2.63 * li
l6 = 4.13 * li
l7 = 2.62 * li
l8 = 2.45 * li
l9 = 3.27 * li
l10 = 4.38 * li
a = 0.52 * li          # Ground Link
b = 2.53 * li          # Ground Link and Origin is at this link's joint

# Angles of Link with respect to Ground
tht_i = 0
tht_1 = 2.4609
tht_2 = 1.2217
tht_3 = 3.5779
tht_4 = 2.7401
tht_5 = 5.1138
tht_6 = 3.9618
tht_7 = 4.9916
tht_8 = 2.6529
tht_9 = 4.2807
tht_10 = 1.8151

# Number of Iterations and Time Step
t = 100
dt = 2 * np.pi / t

# Initialize arrays to store positions
A_x = np.zeros(t)
A_y = np.zeros(t)
B_x = np.zeros(t)
B_y = np.zeros(t)
C_x = np.zeros(t)
C_y = np.zeros(t)
D_x = np.zeros(t)
D_y = np.zeros(t)
E_x = np.zeros(t)
E_y = np.zeros(t)
P_x = np.zeros(t)
P_y = np.zeros(t)

plt.figure(figsize=(10, 10))


# Note - Newton-Raphson Method is not applied because it leads to
#        significant Errors and therefore wrong Results.
for i in range(t):
    # Jacobian Matrix and RHS matrix for Angular Velocity
    J1 = np.array([[-l1*np.cos(tht_1), l2*np.cos(tht_2)],
                   [-l1*np.sin(tht_1), l2*np.sin(tht_2)]])
    
    RHS_J1_v = np.array([[li*np.cos(tht_i)], 
                         [li*np.sin(tht_i)]])
    
    J2 = np.array([[-l6*np.cos(tht_6), l7*np.cos(tht_7)],
                    [l6*np.sin(tht_6), -l7*np.sin(tht_7)]])
    
    RHS_J2_v = np.array([[li*np.cos(tht_i)],
                         [-li*np.sin(tht_i)]])
    
    J3 = np.array([[l8*np.cos(tht_8), -l5*np.cos(tht_5)],
                    [l8*np.sin(tht_8), -l5*np.sin(tht_5)]])
    
    RHS_J3_v = np.array([[l4*np.cos(tht_4), -l7*np.cos(tht_7)],
                          [l4*np.sin(tht_4), -l7*np.sin(tht_7)]]) 
    
    # ------------------- Angular Velocity Analysis --------------------
   
    # Asuming Angular Velocity of Crank to be 1 and fixed link to be 0
    omega_i = 1
    
    # Angular Velocity for Link 1 and 2
    temp = np.linalg.solve(J1, RHS_J1_v) * omega_i 
    omega_1 = temp[0][0]
    omega_2 = temp[1][0]
    omega_4 = omega_2          # Common Centre and fixed angle
    
    # Angular Velocity for Link 6 and 7
    temp = np.linalg.solve(J2, RHS_J2_v) * omega_i 
    omega_6 = temp[0][0]
    omega_7 = temp[1][0]
    
    # Angular Velocity for Link 8 and 5
    temp = np.linalg.solve(J3, RHS_J3_v @ np.array([[omega_4], [omega_7]])) 
    omega_8 = temp[0][0]
    omega_5 = temp[1][0]
    
    tht_i = tht_i + omega_i * dt                   # Input Link (cranck)
    tht_1 = tht_1 + omega_1 * dt   
    tht_2 = tht_2 + omega_2 * dt 
    tht_4 = tht_4 + omega_4 * dt
    tht_5 = tht_5 + omega_5 * dt
    tht_6 = tht_6 + omega_6 * dt
    tht_7 = tht_7 + omega_7 * dt
    tht_8 = tht_8 + omega_8 * dt
    tht_9 = tht_9 + omega_8 * dt
    
    # Plotting on the Graph 
    plt.clf()
    plt.xlim(-150, 150)
    plt.ylim(-150, 150)
    O1 = np.array([0, 0])
    Ri = li * np.array([np.cos(tht_i), np.sin(tht_i)]) + np.array([b, a])
    R1 = Ri + l1 * np.array([np.cos(tht_1), np.sin(tht_1)])
    R2 = l2 * np.array([np.cos(tht_2), np.sin(tht_2)])
    R3 = R2 + l3 * np.array([np.cos(tht_3), np.sin(tht_3)])
    R4 = l4 * np.array([np.cos(tht_4), np.sin(tht_4)])
    R5 = R4 + l5 * np.array([np.cos(tht_5), np.sin(tht_5)])
    R6 = Ri + l6 * np.array([np.cos(tht_6), np.sin(tht_6)])
    R7 = l7 * np.array([np.cos(tht_7), np.sin(tht_7)])
    R8 = R7 + l8 * np.array([np.cos(tht_8), np.sin(tht_8)])
    R9 = R7 + l9 * np.array([np.cos(tht_9), np.sin(tht_9)])
    
    A_x[i] = Ri[0]
    A_y[i] = Ri[1]
    B_x[i] = R1[0]
    B_y[i] = R1[1]
    C_x[i] = R4[0]
    C_y[i] = R4[1]
    D_x[i] = R7[0]
    D_y[i] = R7[1]
    E_x[i] = R8[0]
    E_y[i] = R8[1]
    P_x[i] = R9[0]
    P_y[i] = R9[1]
    
    plt.plot([b, Ri[0]], [a, Ri[1]], '-b', linewidth=4)
    plt.plot([O1[0], R2[0]], [O1[1], R2[1]], '-r', linewidth=4)
    plt.plot([Ri[0], R1[0]], [Ri[1], R1[1]], '-g', linewidth=4)
    plt.plot([O1[0], R4[0]], [O1[1], R4[1]], '-r', linewidth=4)
    plt.plot([O1[0], R7[0]], [O1[1], R7[1]], '-r', linewidth=4)
    plt.plot([Ri[0], R7[0]], [Ri[1], R7[1]], '-b', linewidth=4)
    plt.plot([R4[0], R5[0]], [R4[1], R5[1]], '-g', linewidth=4)
    plt.plot([R7[0], R8[0]], [R7[1], R8[1]], '-b', linewidth=4)
    plt.plot([R2[0], R4[0]], [R2[1], R4[1]], '-b', linewidth=4)
    plt.plot([R7[0], R9[0]], [R7[1], R9[1]], '-b', linewidth=4)
    plt.plot([R8[0], R9[0]], [R8[1], R9[1]], '-b', linewidth=4)
    
    plt.plot(O1[0], O1[1], 'o', markersize=12)
    plt.plot(Ri[0], Ri[1], 'o', markersize=12)
    plt.plot(R1[0], R1[1], 'o', markersize=12)
    plt.plot(R2[0], R2[1], 'o', markersize=12)
    plt.plot(R4[0], R4[1], 'o', markersize=12)
    plt.plot(R5[0], R5[1], 'o', markersize=12)
    plt.plot(R6[0], R6[1], 'o', markersize=12)
    plt.plot(R7[0], R7[1], 'o', markersize=12)
    plt.plot(R8[0], R8[1], 'o', markersize=12)
    plt.plot(R9[0], R9[1], 'o', markersize=12)
    
    plt.pause(0.05)

# Plot all the trajectories
plt.figure()
plt.plot(P_x, P_y)
plt.title('Locus of point P')
plt.xlabel('X Coordinate of P')
plt.ylabel('Y Coordinate of P')
plt.grid(True)

plt.figure()
plt.plot(A_x, A_y)
plt.title('Locus of point 1')
plt.xlabel('X Coordinate of 1')
plt.ylabel('Y Coordinate of 1')
plt.grid(True)

plt.figure()
plt.plot(B_x, B_y)
plt.title('Locus of point 2')
plt.xlabel('X Coordinate of 2')
plt.ylabel('Y Coordinate of 2')
plt.grid(True)

plt.figure()
plt.plot(C_x, C_y)
plt.title('Locus of point 4')
plt.xlabel('X Coordinate of 4')
plt.ylabel('Y Coordinate of 4')
plt.grid(True)

plt.figure()
plt.plot(D_x, D_y)
plt.title('Locus of point 5')
plt.xlabel('X Coordinate of 5')
plt.ylabel('Y Coordinate of 5')
plt.grid(True)

plt.figure()
plt.plot(E_x, E_y)
plt.title('Locus of point 6')
plt.xlabel('X Coordinate of 6')
plt.ylabel('Y Coordinate of 6')
plt.grid(True)

plt.show()
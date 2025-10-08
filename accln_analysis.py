import numpy as np
import matplotlib.pyplot as plt


li = 20                # Input Link (crank)
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

# Initialize angle arrays
t = 50
tht_i = np.zeros(t+1)
tht_1 = np.zeros(t+1)
tht_2 = np.zeros(t+1)
tht_3 = np.zeros(t+1)
tht_4 = np.zeros(t+1)
tht_5 = np.zeros(t+1)
tht_6 = np.zeros(t+1)
tht_7 = np.zeros(t+1)
tht_8 = np.zeros(t+1)
tht_9 = np.zeros(t+1)
tht_10 = np.zeros(t+1)

# Initial angles
tht_i[0] = 0
tht_1[0] = 2.4609
tht_2[0] = 1.2217
tht_3[0] = 3.5779
tht_4[0] = 2.7401
tht_5[0] = 5.1138
tht_6[0] = 3.9618
tht_7[0] = 4.9916
tht_8[0] = 2.6529
tht_9[0] = 4.2807
tht_10[0] = 1.8151

dt = 2 * np.pi / t

# Initialize angular velocity and acceleration arrays
omega_1 = np.zeros(t)
omega_2 = np.zeros(t)
omega_4 = np.zeros(t)
omega_5 = np.zeros(t)
omega_6 = np.zeros(t)
omega_7 = np.zeros(t)
omega_8 = np.zeros(t)

alpha_1 = np.zeros(t)
alpha_2 = np.zeros(t)
alpha_4 = np.zeros(t)
alpha_5 = np.zeros(t)
alpha_6 = np.zeros(t)
alpha_7 = np.zeros(t)
alpha_8 = np.zeros(t)

tht_input = np.zeros(t)

for i in range(t):
    # Jacobian Matrix and RHS matrix for Angular Velocity
    J1 = np.array([[-l1*np.cos(tht_1[i]), l2*np.cos(tht_2[i])],
                   [-l1*np.sin(tht_1[i]), l2*np.sin(tht_2[i])]])
    
    RHS_J1_v = np.array([[li*np.cos(tht_i[i])], 
                         [li*np.sin(tht_i[i])]])
        
    J2 = np.array([[-l6*np.cos(tht_6[i]), l7*np.cos(tht_7[i])],
                    [l6*np.sin(tht_6[i]), -l7*np.sin(tht_7[i])]])
    
    RHS_J2_v = np.array([[li*np.cos(tht_i[i])],
                         [-li*np.sin(tht_i[i])]])
      
    J3 = np.array([[l8*np.cos(tht_8[i]), -l5*np.cos(tht_5[i])],
                    [l8*np.sin(tht_8[i]), -l5*np.sin(tht_5[i])]])
    
    RHS_J3_v = np.array([[l4*np.cos(tht_4[i]), -l7*np.cos(tht_7[i])],
                          [l4*np.sin(tht_4[i]), -l7*np.sin(tht_7[i])]]) 
   
    # Assuming Angular Velocity of Crank to be 1 and fixed link to be 0
    omega_i = 1
    
    # Angular Velocity for Link 1 and 2
    temp = np.linalg.solve(J1, RHS_J1_v) * omega_i 
    omega_1[i] = temp[0][0]
    omega_2[i] = temp[1][0]
    omega_4[i] = omega_2[i]          # Common Centre and fixed angle
    
    # Angular Velocity for Link 6 and 7
    temp = np.linalg.solve(J2, RHS_J2_v) * omega_i 
    omega_6[i] = temp[0][0]
    omega_7[i] = temp[1][0]
    
    # Angular Velocity for Link 8 and 5
    temp = np.linalg.solve(J3, RHS_J3_v @ np.array([[omega_4[i]], [omega_7[i]]])) 
    omega_8[i] = temp[0][0]
    omega_5[i] = temp[1][0]
     
    # Jacobian Matrix and RHS matrix for Angular Acceleration
    RHS_J1_a = np.array([[-li*omega_i*np.sin(tht_i[i]) - l1*omega_1[i]*np.sin(tht_1[i]) - 
                          l2*omega_2[i]*np.sin(tht_2[i])],
                         [-li*omega_i*np.cos(tht_i[i]) + l1*omega_1[i]*np.cos(tht_1[i]) - 
                          l2*omega_2[i]*np.cos(tht_2[i])]])
              
    # Angular Acceleration for Link 1 and 2
    temp = np.linalg.solve(J1, RHS_J1_a) 
    alpha_1[i] = temp[0][0]
    alpha_2[i] = temp[1][0]
    alpha_4[i] = alpha_2[i]                   # Common Centre and fixed angle     

    RHS_J2_a = np.array([[-li*omega_i*np.sin(tht_i[i]) - l6*omega_6[i]*np.sin(tht_6[i]) + 
                          l7*omega_7[i]*np.sin(tht_7[i])],
                         [li*omega_i*np.cos(tht_i[i]) - l6*omega_6[i]*np.cos(tht_6[i]) + 
                          l7*omega_7[i]*np.cos(tht_7[i])]])
          
    # Angular Acceleration for Link 6 and 7
    temp = np.linalg.solve(J2, RHS_J2_a) 
    alpha_6[i] = temp[0][0]
    alpha_7[i] = temp[1][0]          
          
    RHS_J3_a = np.array([[l4*alpha_4[i]*np.cos(tht_4[i]) - l4*omega_4[i]*np.sin(tht_4[i]) - 
                         l7*alpha_7[i]*np.cos(tht_7[i]) + l7*omega_7[i]*np.sin(tht_7[i]) + 
                         l8*omega_8[i]*np.sin(tht_8[i]) - l5*omega_5[i]*np.sin(tht_5[i])],
                        [l4*alpha_4[i]*np.sin(tht_4[i]) + l4*omega_4[i]*np.cos(tht_4[i]) - 
                         l7*alpha_7[i]*np.sin(tht_7[i]) - l7*omega_7[i]*np.cos(tht_7[i]) - 
                         l8*omega_8[i]*np.cos(tht_8[i]) + l5*omega_5[i]*np.cos(tht_5[i])]])

    # Angular Acceleration for Link 8 and 5
    temp = np.linalg.solve(J3, RHS_J3_a) 
    alpha_8[i] = temp[0][0]
    alpha_5[i] = temp[1][0]
    
    # Update angles for next iteration
    tht_i[i+1] = tht_i[i] + omega_i * dt                   # Input Link (crank)
    tht_1[i+1] = tht_1[i] + omega_1[i] * dt   
    tht_2[i+1] = tht_2[i] + omega_2[i] * dt 
    tht_4[i+1] = tht_4[i] + omega_4[i] * dt
    tht_5[i+1] = tht_5[i] + omega_5[i] * dt
    tht_6[i+1] = tht_6[i] + omega_6[i] * dt
    tht_7[i+1] = tht_7[i] + omega_7[i] * dt
    tht_8[i+1] = tht_8[i] + omega_8[i] * dt
    
    tht_input[i] = tht_i[i]

# Plotting Various Graphs
plt.figure()
plt.grid(True)
plt.plot(tht_input, alpha_1)
plt.title(r'$\alpha_j$ vs $\theta_m$')
plt.xlabel(r'$\theta_m \rightarrow$')
plt.ylabel(r'$\alpha_j \rightarrow$')

plt.figure()
plt.grid(True)
plt.plot(tht_input, alpha_2)
plt.title(r'$\alpha_{\Delta bde}$ vs $\theta_m$')
plt.xlabel(r'$\theta_m \rightarrow$')
plt.ylabel(r'$\alpha_{\Delta bde} \rightarrow$')

plt.figure()
plt.grid(True)
plt.plot(tht_input, alpha_5)
plt.title(r'$\alpha_f$ vs $\theta_m$')
plt.xlabel(r'$\theta_m \rightarrow$')
plt.ylabel(r'$\alpha_f \rightarrow$')

plt.figure()
plt.grid(True)
plt.plot(tht_input, alpha_6)
plt.title(r'$\alpha_k$ vs $\theta_m$')
plt.xlabel(r'$\theta_m \rightarrow$')
plt.ylabel(r'$\alpha_k \rightarrow$')

plt.figure()
plt.grid(True)
plt.plot(tht_input, alpha_7)
plt.title(r'$\alpha_c$ vs $\theta_m$')
plt.xlabel(r'$\theta_m \rightarrow$')
plt.ylabel(r'$\alpha_c \rightarrow$')

plt.figure()
plt.grid(True)
plt.plot(tht_input, alpha_8)
plt.title(r'$\alpha_{\Delta ghi}$ vs $\theta_m$')
plt.xlabel(r'$\theta_m \rightarrow$')
plt.ylabel(r'$\alpha_{\Delta ghi} \rightarrow$')

plt.show()
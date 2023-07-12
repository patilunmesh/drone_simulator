import numpy as np
# The dynamics is from pp. 17, Eq. (2.22), https://www.kth.se/polopoly_fs/1.588039.1550155544!/Thesis%20KTH%20-%20Francesco%20Sabatino.pdf
# The constants are from Different Linearization Control Techniques for a Quadrotor System

# quadrotor physical constants
g = 9.81
m = 1.0
Ix = 8.1 * 1e-3
Iy = 8.1 * 1e-3
Iz = 14.2 * 1e-3

def f(x, u):
    x1, x2, y1, y2, z1, z2, phi1, phi2, theta1, theta2, psi1, psi2 = x.reshape(-1).tolist()
    ft, tau_x, tau_y, tau_z = u.reshape(-1).tolist()
    dot_x = np.array([
     x2,
     ft/m*(np.sin(phi1)*np.sin(psi1)+np.cos(phi1)*np.cos(psi1)*np.sin(theta1)),
     y2,
     ft/m*(np.cos(phi1)*np.sin(psi1)*np.sin(theta1)-np.cos(psi1)*np.sin(phi1)),
     z2,
     -g+ft/m*np.cos(phi1)*np.cos(theta1),
     phi2,
     (Iy-Iz)/Ix*theta2*psi2+tau_x/Ix,
     theta2,
     (Iz-Ix)/Iy*phi2*psi2+tau_y/Iy,
     psi2,
     (Ix-Iy)/Iz*phi2*theta2+tau_z/Iz])
    return dot_x
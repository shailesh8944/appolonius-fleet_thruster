import casadi as ca

L = 230
B = 32.2
d_em = 10.8
rho = 1025
g = 9.80665

Cb = 0.651
Dsp = Cb * L * B * d_em
Fn = 0.26
U_des = Fn * ca.sqrt(g * L)
xG = -3.404
kzzp = 0.25

# Surge Hydrodynamic Derivatives in non-dimensional form
X0 = -0.0167
Xbb = -0.0549
Xbr_minus_my = -0.1084
Xrr = -0.0120
Xbbbb = -0.0417

# Sway Hydrodynamic Derivatives in non-dimensional form
Yb = 0.2252
Yr_minus_mx = 0.0398
Ybbb = 1.7179
Ybbr = -0.4832
Ybrr = 0.8341
Yrrr = -0.0050

# Yaw Hydrodynamic Derivatives in non-dimensional form
Nb = 0.1111
Nr = -0.0465
Nbbb = 0.1752
Nbbr = -0.6168
Nbrr = 0.0512
Nrrr = -0.0387

n_prop = 115.5 / 60
Dp = 7.9
wp = 1 - 0.645  # Effective Wake Fraction of the Propeller
tp = 1 - 0.793  # Thrust Deduction Factor

eps = 0.956
eta = 0.7979
kappa = 0.633
xp_P = -0.4565  # Assuming propeller location is 10 m ahead of AP (Rudder Location)
xp_R = -0.5

mp = Dsp / (0.5 * (L ** 2) * d_em)
xGp = xG / L
# Added Mass and Mass Moment of Inertia (from MDLHydroD)
mxp = 1790.85 / (0.5 * (L ** 2) * d_em)
myp = 44324.18 / (0.5 * (L ** 2) * d_em)
Jzzp = 140067300 / (0.5 * (L ** 4) * d_em)
Izzp = mp * (kzzp ** 2) + mp * (xGp ** 2)

a0 = 0.5228
a1 = -0.4390
a2 = -0.0609

tR = 1 - 0.742
aH = 0.361
xp_H = -0.436

A_R = L * d_em / 54.86
Lamda = 2.164
f_alp = 6.13 * Lamda / (2.25 + Lamda)




def _mmgder(v, delta_c):
    # Assume v is a CASadi symbolic vector (e.g. SX or MX)
    up    = v[0]
    vp    = v[1]
    rp    = v[2]
    xp    = v[3]
    yp    = v[4]
    psi   = v[5]
    delta = v[6]
    n_prop = v[7]
    
    #print("shape of delta_c",delta_c.shape)
    n_prop_cmd = delta_c[0]
    rudder_cmd = delta_c[1]
    # Derived kinematic variable: drift angle

    b = ca.atan2(-vp, up)

    # ----------------------------------------------------
    # Hull Force Calculation
    # ----------------------------------------------------
    Xp_H = X0 * (up**2) + Xbb * (b**2) + Xbr_minus_my * b * rp + Xrr * (rp**2) + Xbbbb * (b**4)
    Yp_H = Yb * b + Yr_minus_mx * rp + Ybbb * (b**3) + Ybbr * (b**2) * rp + Ybrr * b * (rp**2) + Yrrr * (rp**3)
    Np_H = Nb * b + Nr * rp + Nbbb * (b**3) + Nbbr * (b**2) * rp + Nbrr * b * (rp**2) + Nrrr * (rp**3)

    # ----------------------------------------------------
    # Propulsion Force Calculation
    # ----------------------------------------------------
    J = (up * U_des) * (1 - wp) / (n_prop * Dp)
    Kt = a0 + a1 * J + a2 * (J**2)
    X_P = (1 - tp) * rho * Kt * (Dp**4) * (n_prop**2)
    Xp_P = X_P / (0.5 * rho * L * d_em * (U_des**2))

    # ----------------------------------------------------
    # Rudder Force Calculation
    # ----------------------------------------------------
    b_p = b - xp_P * rp
    gamma_R = ca.if_else(b_p > 0, 0.492, 0.338)
    lp_R = -0.755
    cond = J != 0
    up_R = ca.if_else(cond, eps * (1 - wp) * up * ca.sqrt(eta * (1 + kappa * (ca.sqrt(1 + 8 * Kt / (ca.pi * (J ** 2))) - 1)) ** 2 + (1 - eta)), eps * (1 - wp) * up)
    vp_R = ca.if_else(cond, gamma_R * (vp + rp * lp_R), gamma_R * (vp + rp * lp_R))
    
    Up_R = ca.sqrt(up_R**2 + vp_R**2)
    alpha_R = delta - ca.atan2(-vp_R, up_R)
    F_N = A_R / (L * d_em) * f_alp * (Up_R**2) * ca.sin(alpha_R)

    Xp_R = - (1 - tR) * F_N * ca.sin(delta)
    Yp_R = - (1 + aH) * F_N * ca.cos(delta)
    Np_R = - (xp_R + aH * xp_H) * F_N * ca.cos(delta)

    # ----------------------------------------------------
    # Coriolis terms
    # ----------------------------------------------------
    mp  = Dsp / (0.5 * (L**2) * d_em)
    xGp = xG / L

    Xp_C = mp * vp * rp + mp * xGp * (rp**2)
    Yp_C = -mp * up * rp
    Np_C = -mp * xGp * up * rp

    # ----------------------------------------------------
    # Wind Force Calculation (set to zero)
    # ----------------------------------------------------
    Xp_W = 0 
    Yp_W = 0
    Np_W = 0

    # Net forces and moment
    Xp = Xp_H + Xp_R + Xp_C + Xp_W + Xp_P
    Yp = Yp_H + Yp_R + Yp_C + Yp_W
    Np = Np_H + Np_R + Np_C + Np_W

    # Assign net forces/moment
    X = Xp
    Y = Yp
    N = Np

    # ----------------------------------------------------
    # Added Mass and Moment of Inertia
    # ----------------------------------------------------
    mxp   = 1790.85 / (0.5 * (L**2) * d_em)
    myp   = 44324.18 / (0.5 * (L**2) * d_em)
    Jzzp  = 140067300 / (0.5 * (L**4) * d_em)
    Izzp  = mp * (kzzp**2) + mp * (xGp**2)

    Mmat = ca.SX.zeros(3, 3)
    Mmat[0, 0] = mp + mxp
    Mmat[1, 1] = mp + myp
    Mmat[2, 2] = Izzp + Jzzp
    Mmat[1, 2] = mp * xGp
    Mmat[2, 1] = mp * xGp

    Mmatinv = ca.inv(Mmat)
    tau = ca.vertcat(X, Y, N)
    vel_der = ca.mtimes(Mmatinv, tau)

    # ----------------------------------------------------
    # Kinematic Equations
    # ----------------------------------------------------
    dx   = up * ca.cos(psi) - vp * ca.sin(psi)
    dy   = up * ca.sin(psi) + vp * ca.cos(psi)
    dpsi = rp
    n_prop_dot = (n_prop_cmd - n_prop) /10
    n_prop_dot_max = .01
    n_prop_dot = ca.if_else(ca.fabs(n_prop_dot) > n_prop_dot_max, 
                           ca.sign(n_prop_dot) * n_prop_dot_max,
                           n_prop_dot)
    # Commanded Rudder Angle Dynamics
    # ----------------------------------------------------
    T_rud = 0.1  # Time constant scaled by L/U_des
    deltad = (rudder_cmd - delta) / T_rud
    deltad_max = 5* ca.pi / (180 * (L / U_des))
    deltad = ca.if_else(ca.fabs(deltad) > deltad_max, ca.sign(deltad) * deltad_max, deltad)

    # ----------------------------------------------------
    # Assemble the State Derivative Vector
    # ----------------------------------------------------
    vd = ca.vertcat(vel_der[0],
                    vel_der[1],
                    vel_der[2],
                    dx,
                    dy,
                    dpsi,
                    deltad,n_prop_dot)

    return vd

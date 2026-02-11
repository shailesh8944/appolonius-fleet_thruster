import numpy as np
import module_kinematics as kin
import pandas as pd

def ode_options(scale=1, 
    wind_flag=False, wind_speed=0.0, wind_dir=180.0,
    wave_flag=False, wave_comp=None,
    current_flag=False, current_speed=0.0, current_dir=180.0,
    deltad_max=100, T_rud=1,  T_prop=1, nd_max=0.1):

    options = {}

    options['scale'] = scale
    options['wind_flag'] = wind_flag
    options['wind_speed'] = wind_speed
    options['wind_dir'] = wind_dir

    options['wave_flag'] = wave_flag
    options['wave_comp'] = wave_comp

    options['current_flag'] = current_flag
    options['current_speed'] = current_speed
    options['current_dir'] = current_dir

    options['deltad_max'] = deltad_max
    options['T_rud'] = T_rud
    options['T_prop'] = T_prop
    options['nd_max'] = nd_max

    return options

def kcs_ode(t, v, delta_c, n_c, options):

    scale = options['scale']
    wind_flag = options['wind_flag']
    wind_speed = options['wind_speed']
    wind_dir = options['wind_dir']

    wave_flag = options['wave_flag']
    wave_comp = options['wave_comp']

    current_flag = options['current_flag']
    current_speed = options['current_speed']
    current_dir = options['current_dir']

    deltad_max = options['deltad_max']
    T_rud = options['T_rud']
    T_prop = options['T_prop']
    nd_max = options['nd_max']

    # Ship Geometry and Constants
    L = 230 / scale
    B = 32.2 / scale
    d_em = 10.8 / scale
    rho = 1025
    g = 9.80665

    Cb = 0.651
    Dsp = Cb * L * B * d_em
    Fn = 0.26
    U_des = Fn * np.sqrt(g * L)
    xG = -3.404 / scale
    kzzp = 0.25

    # Nondimensional State Space Variables

    up = v[0]
    vp = v[1]
    rp = v[5]
    xp = v[6]
    yp = v[7]
    q = v[9:13]
    eul = kin.quat_to_eul(q)
    psi = eul[2]
    delta = v[13]
    n_prop = v[14]

    # Derived kinematic variables
    b = np.arctan2(-vp, up)     # Drift angle


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

    # Non-dimensional Surge Hull Hydrodynamic Force
    Xp_H = X0 * (up ** 2) \
        + Xbb * (b ** 2) + Xbr_minus_my * b * rp \
        + Xrr * (rp ** 2) + Xbbbb * (b ** 4)

    # Non-dimensional Sway Hull Hydrodynamic Force
    Yp_H = Yb * b + Yr_minus_mx * rp + Ybbb * (b ** 3) \
        + Ybbr * (b ** 2) * rp + Ybrr * b * (rp ** 2) \
        + Yrrr * (rp ** 3)

    # Non-dimensional Yaw Hull Hydrodynamic Moment
    Np_H = Nb * b + Nr * rp + Nbbb * (b ** 3) \
        + Nbbr * (b ** 2) * rp + Nbrr * b * (rp ** 2) \
        + Nrrr * (rp ** 3)

    # Propulsion Force Calculation

    # The value self propulsion RPM is taken from Yoshimura's SIMMAN study
    # Analysis of steady hydrodynamic force components and prediction of
    # manoeuvering ship motion with KVLCC1, KVLCC2 and KCS
    # n_prop = 115.5 / 60
    Dp = 7.9 / scale
    wp = 1 - 0.645  # Effective Wake Fraction of the Propeller
    tp = 1 - 0.793  # Thrust Deduction Factor

    if n_prop > 1e-3:

        J = (up * U_des) * (1 - wp) / (n_prop * Dp)     # Advance Coefficient

        a0 = 0.5228
        a1 = -0.4390
        a2 = -0.0609
        Kt = a0 + a1 * J + a2 * (J ** 2)        # Thrust Coefficient

        # Dimensional Propulsion Force
        X_P = (1 - tp) * rho * Kt * (Dp ** 4) * (n_prop ** 2)
        
    else:
        J = 1.0
        Kt = 0.0
        X_P = 0.0

    # Non-dimensional Propulsion Force
    Xp_P = X_P / (0.5 * rho * L * d_em * (U_des ** 2))

    # Rudder Force Calculation
    A_R = L * d_em / 54.86
    Lamda = 2.164
    f_alp = 6.13 * Lamda / (2.25 + Lamda)

    eps = 0.956
    eta = 0.7979
    kappa = 0.633
    xp_P = -0.4565  # Assuming propeller location is 10 m ahead of AP (Rudder Location)
    xp_R = -0.5

    b_p = b - xp_P * rp

    if b_p > 0:
        gamma_R = 0.492
    else:
        gamma_R = 0.338

    lp_R = -0.755
    if J != 0.0:
        up_R = eps * (1 - wp) * up * np.sqrt(eta * (1 + kappa * \
            (np.sqrt(1 + 8 * Kt /(np.pi * (J ** 2)) ) - 1)) ** 2 + (1 - eta))

        vp_R = gamma_R * (vp + rp * lp_R)
    else:
        up_R = eps * (1 - wp) * up
        vp_R = gamma_R * (vp + rp * lp_R)

    Up_R = np.sqrt(up_R ** 2 + vp_R ** 2)
    alpha_R = delta  - np.arctan2(-vp_R, up_R)

    F_N = A_R / (L * d_em) * f_alp * (Up_R ** 2) * np.sin(alpha_R)

    tR = 1 - 0.742
    aH = 0.361
    xp_H = -0.436

    Xp_R = - (1 - tR) * F_N * np.sin(delta)
    Yp_R = - (1 + aH) * F_N * np.cos(delta)
    Np_R = - (xp_R + aH * xp_H) * F_N * np.cos(delta)

    # Coriolis terms

    mp = Dsp / (0.5 * (L ** 2) * d_em)
    xGp = xG / L

    Xp_C = mp * vp * rp + mp * xGp * (rp ** 2)
    Yp_C = -mp * up * rp
    Np_C = -mp * xGp * up * rp
    
    if wind_flag:
        Vw = wind_speed  # wind speed
        betaw = wind_dir * (np.pi/180)  # wind direction
        uw = Vw * np.cos(betaw - psi)
        vw = Vw * np.sin(betaw - psi)
        urw = up - uw
        vrw = vp - vw
        Uwr = (urw ** 2 + vrw ** 2) ** 0.5
        gammaw = np.arctan2(-vrw, -urw)

        rhow = 1025
        rhoa = 1.225
        Ax = 0.1064
        Ay = 0.7601
        
        Cwx = 1 * np.cos(gammaw)
        Cwy = 1 * np.sin(gammaw)
        Cwpsi = 0.5 * np.sin(gammaw)

        Xp_W = (Ax * Cwx * Uwr * abs(Uwr)) * rhoa / rhow
        Yp_W = (Ay * Cwy * Uwr * abs(Uwr)) * rhoa / rhow
        Np_W = (Ay * Cwpsi * Uwr * abs(Uwr)) * rhoa / rhow
    else:
        Xp_W = 0.0
        Yp_W = 0.0
        Np_W = 0.0
    
    # Net non-dimensional force and moment computation
    Xp = Xp_H + Xp_R + Xp_C + Xp_W + Xp_P
    Yp = Yp_H + Yp_R + Yp_C + Yp_W
    Np = Np_H + Np_R + Np_C + Np_W

    # Net force vector computation
    X = Xp
    Y = Yp
    N = Np

    # Added Mass and Mass Moment of Inertia (from MDLHydroD)
    mxp = 1790.85 / (0.5 * ((L * scale) ** 2) * (d_em * scale))
    myp = 44324.18 / (0.5 * ((L * scale) ** 2) * (d_em * scale))
    Jzzp = 140067300 / (0.5 * ((L * scale) ** 4) * (d_em * scale))
    Izzp = mp * (kzzp ** 2) + mp * (xGp ** 2)

    Mmat = np.zeros((3, 3))

    Mmat[0, 0] = mp + mxp
    Mmat[1, 1] = mp + myp
    Mmat[2, 2] = Izzp + Jzzp
    Mmat[1, 2] = mp * xGp
    Mmat[2, 1] = mp * xGp

    Mmatinv = np.linalg.inv(Mmat)

    tau = np.array([X, Y, N])

    vel_der = Mmatinv @ tau

    # Derivative of state vector
    vd = np.zeros(15)

    vd[0:2] = vel_der[0:2]
    vd[5] = vel_der[2]
    vd[6] = up * np.cos(psi) - vp * np.sin(psi)
    vd[7] = up * np.sin(psi) + vp * np.cos(psi)
    w = np.array([0, 0, rp])
    vd[9:13] = kin.quat_rate(q, w)

    # T_rud = 1     # Corresponds to a time constant of 1 * L / U_des = 20 seconds
    deltad = (delta_c - delta) / T_rud
    deltad_max = deltad_max * np.pi / 180 * (L / U_des)   # Maximum rudder rate of 5 degrees per second

    # Rudder rate saturation
    if np.abs(deltad) > deltad_max:
        deltad = np.sign(deltad) * deltad_max

    # T_prop = 1     # Corresponds to a time constant of 1 * L / U_des = 20 seconds
    nd_prop = (n_c - n_prop) / T_prop
    # print(nd_prop)
    # nd_max = 0.1

    # Rudder rate saturation
    if np.abs(nd_prop) > nd_max:
        nd_prop = np.sign(nd_prop) * nd_max

    vd[13] = deltad
    vd[14] = nd_prop

    # print(vd)

    return vd

def onrt_ode(t, ss, delta_c, n_c, options, euler_angle_flag=False, mmg_flag=False):

    scale = options['scale']
    wind_flag = options['wind_flag']
    wind_speed = options['wind_speed']
    wind_dir = options['wind_dir']

    wave_flag = options['wave_flag']
    wave_comp = options['wave_comp']

    current_flag = options['current_flag']
    current_speed = options['current_speed']
    current_dir = options['current_dir']

    deltad_max = options['deltad_max']
    T_rud = options['T_rud']
    T_prop = options['T_prop']
    nd_max = options['nd_max']

    # All coefficients imported below are non-dimensional

    M_RB = options['M_RB']
    M_A = options['M_A']
    Dl = options['Dl']
    K = options['K']
    M_inv = options['M_inv']

    Xuu = options['X_u_au']
    Yvv = options['Y_v_av']
    Yvr = options['Y_v_ar']
    Yrv = options['Y_r_av']
    Yrr = options['Y_r_ar']
    Nvv = options['N_v_av']
    Nvr = options['N_v_ar']
    Nrv = options['N_r_av']
    Nrr = options['N_r_ar']
    
    if mmg_flag:
        R0 = options['R0'] 
        X_v_v = options['X_v_v'] 
        X_v_r = options['X_v_r'] 
        X_r_r = options['X_r_r'] 
        X_v_v_v_v = options['X_v_v_v_v'] 
        
        Y_v = options['Y_v'] 
        Y_R = options['Y_R'] 
        Y_v_v_v = options['Y_v_v_v'] 
        Y_v_v_r = options['Y_v_v_r'] 
        Y_v_r_r = options['Y_v_r_r'] 
        Y_r_r_r = options['Y_r_r_r'] 
        
        N_v = options['N_v'] 
        N_R = options['N_R'] 
        N_v_v_v = options['N_v_v_v'] 
        N_v_v_r = options['N_v_v_r'] 
        N_v_r_r = options['N_v_r_r'] 
        N_r_r_r = options['N_r_r_r'] 

    D_prop = options['D_prop']
    
    # Now we only pass only a single propeller open water curve (pow)
    pow_coeff = options['pow_coeff']

    A_R = options['rudder_area']
    Lamda = options['rudder_aspect_ratio']

    wp = options['wp']
    tp = options['tp']
    xp_R = options['xp_R']
    tR = options['tR']
    aH = options['aH']
    xp_H = options['xp_H']
    eta = options['eta']
    eps = options['eps']
    X_by_Dp = options['X_by_Dp']
    lp_R = options['lp_R']
    gamma_R1 = options['gamma_R1']
    gamma_R2 = options['gamma_R2']

    L = options['L']
    U_des = options['U_des']

    # Nondimensional State Space Variables

    up = ss[0]; vp = ss[1]; wp = ss[2]
    pp = ss[3]; qp = ss[4]; rp = ss[5]
    xp = ss[6]; yp = ss[7]; zp = ss[8]
    
    if euler_angle_flag:
        eul = ss[9:12]
    else:
        quat = ss[9:13]
        eul = kin.quat_to_eul(quat)
    
    phi = eul[0]; theta = eul[1]; psi = eul[2]

    if euler_angle_flag:
        rud_indx = 12
        prop_indx = 13
    else:
        rud_indx = 13
        prop_indx = 14
    
    delta = ss[rud_indx]; n_prop = ss[prop_indx]

    v1 = ss[0:3]; v2 = ss[3:6]

    # Coriolis force calculation

    M11 = M_RB[0:3][:, 0:3]
    M12 = M_RB[0:3][:, 3:6]
    M21 = M_RB[3:6][:, 0:3]
    M22 = M_RB[3:6][:, 3:6]

    C_RB = np.zeros((6,6))
    C_RB[0:3][:, 3:6] = -kin.Smat(M11 @ v1 + M12 @ v2)
    C_RB[3:6][:, 0:3] = -kin.Smat(M11 @ v1 + M12 @ v2)
    C_RB[3:6][:, 3:6] = -kin.Smat(M21 @ v1 + M22 @ v2)

    A11 = M_A[0:3][:, 0:3]
    A12 = M_A[0:3][:, 3:6]
    A21 = M_A[3:6][:, 0:3]
    A22 = M_A[3:6][:, 3:6]

    C_A = np.zeros((6,6))
    C_A[0:3][:, 3:6] = -kin.Smat(A11 @ v1 + A12 @ v2)
    C_A[3:6][:, 0:3] = -kin.Smat(A11 @ v1 + A12 @ v2)
    C_A[3:6][:, 3:6] = -kin.Smat(A21 @ v1 + A22 @ v2)    

    tau_C = -(C_RB @ ss[0:6] + C_A @ ss[0:6])

    # Damping force calculation

    Dnl = np.zeros(6)
    Dnl[0] = -Xuu * up * np.abs(up)
    Dnl[1] = -(Yvv * vp * np.abs(vp) + Yrv * rp * np.abs(vp) + Yvr * vp * np.abs(rp) + Yrr * rp * np.abs(rp))
    Dnl[5] = -(Nvv * vp * np.abs(vp) + Nrv * rp * np.abs(vp) + Nvr * vp * np.abs(rp) + Nrr * rp * np.abs(rp))

    tau_D = -(Dl @ ss[0:6]) - Dnl
    
    # MMG hull hydrodynamic force calculation
    if mmg_flag:
        # Non-dimensional Surge Hull Hydrodynamic Force
        Xp_H = -R0 * (up ** 2) + X_v_v * (vp ** 2) + X_v_r * vp * rp + X_r_r * (rp ** 2) + X_v_v_v_v * (vp ** 4)

        # Non-dimensional Sway Hull Hydrodynamic Force
        Yp_H = Y_v * vp + Y_R * rp + Y_v_v_v * (vp ** 3) \
        + Y_v_v_r * (vp ** 2) * rp + Y_v_r_r * vp * (rp ** 2) + Y_r_r_r * (rp ** 3)

        # Non-dimensional Yaw Hull Hydrodynamic Moment
        Np_H = N_v * vp + N_R * rp + N_v_v_v * (vp ** 3) + N_v_v_r * (vp ** 2) * rp \
        + N_v_r_r * vp * (rp ** 2) + N_r_r_r * (rp ** 3)
        
        tau_MMG = np.zeros(6)
        tau_MMG[0] = Xp_H
        tau_MMG[1] = Yp_H
        tau_MMG[5] = Np_H

    # Stiffness force calculation

    xi = np.zeros(6)
    xi[0:3] = ss[6:9]    
    xi[3:6] = eul
    tau_K = -(K @ xi)

    # Propeller force calculation

    X_prop = pow_coeff[0] * ((up * (1 - wp)) **2) * (D_prop **2) \
                + pow_coeff[1] * (up * (1 - wp)) * (n_prop * (D_prop ** 3)) \
                + pow_coeff[2] * ((n_prop ** 2) * (D_prop ** 4))

    tau_P = np.zeros(6)
    tau_P[0] = 2 * (1 - tp) * X_prop

    # Rudder force calculation
    
    kappa_R = 0.5 + 0.5 * X_by_Dp / (X_by_Dp + 0.15)

    # pow_coeff = 0.5 * (pow_coeff_port + pow_coeff_stbd)    

    up_prop = up * (1 - wp)
    Kt_up2_by_J2 = pow_coeff[0] * (up_prop ** 2) + pow_coeff[1] * n_prop * D_prop * up_prop + pow_coeff[2] * (n_prop * D_prop) ** 2
    up_R = eps * np.sqrt( eta * (up_prop + kappa_R * (np.sqrt(up_prop ** 2 + 8/np.pi * Kt_up2_by_J2) - up_prop)) ** 2 + (1 - eta) * up_prop ** 2 )

    b = np.arctan2(-vp, up)     # Drift angle
    b_p = b - (-X_by_Dp * D_prop) * rp
    
    # gamma_R (formula from from KCS structure - values modified)
    if b_p > 0:
        gamma_R = gamma_R1 # 0.6
    else:
        gamma_R = gamma_R2 # 0.3
    
    vp_R = gamma_R * (vp + rp * lp_R)

    # Resultant fluid speed at rudder
    Up_R = np.sqrt(up_R ** 2 + vp_R ** 2)

    # Effective inflow angle to rudder
    alpha_R = delta - np.arctan2(-vp_R, up_R)

    # Rudder lift gradient coefficient
    f_alp = 6.13 * Lamda / (2.25 + Lamda)

    # Rudder normal force
    F_N = A_R * f_alp * (Up_R ** 2) * np.sin(alpha_R)

    Xp_R = - (1 - tR) * F_N * np.sin(delta)
    Yp_R = - (1 + aH) * F_N * np.cos(delta)
    Np_R = - (xp_R + aH * xp_H) * F_N * np.cos(delta)

    tau_R = np.zeros(6)
    tau_R[0] = Xp_R
    tau_R[1] = Yp_R
    tau_R[5] = Np_R

    # Total force calculation

    if mmg_flag:
        tau = tau_C + tau_K + tau_P + tau_R + tau_MMG
    else:
        tau = tau_C + tau_D + tau_K + tau_P + tau_R

    # print(tau_P[0], tau_D[0], Xuu, up)

    # Derivative of velocity vector

    veld = M_inv @ tau

    # Derivative of state vector
    if euler_angle_flag:
        n = 20  # 6 (velocities) + 3 (positions) + 3 (euler) + 6 (control surfaces) + 1 (propeller) = 19
    else:
        n = 21  # 6 (velocities) + 3 (positions) + 4 (quaternion) + 6 (control surfaces) + 1 (propeller) = 20

    ssd = np.zeros(n)
    ssd[0:6] = veld

    if euler_angle_flag:
        ssd[6:9] = kin.eul_to_rotm(eul) @ v1
        ssd[9:12] = kin.eul_rate(eul, v2)
    else:
        ssd[6:9] = kin.quat_to_rotm(quat) @ v1
        ssd[9:13] = kin.quat_rate(quat, v2)
    
    # Rudder dynamics - modified for 6x1 array
    deltad = (delta_c - delta) / T_rud  # This will be element-wise division for arrays
    deltad_max = deltad_max  # Assuming deltad_max is same for all surfaces
    
    # Rudder rate saturation - vectorized for 6x1 array
    deltad = np.clip(deltad, -deltad_max, deltad_max)  # Handles array saturation

    # Propeller dynamics

    nd_prop = (n_c - n_prop) / T_prop
    # Propeller speed rate saturation
    if np.abs(nd_prop) > nd_max:
        nd_prop = np.sign(nd_prop) * nd_max

    ssd[rud_indx] = deltad
    ssd[prop_indx] = nd_prop

    return ssd

def mavymini_ode(t, ss, delta_c, n_c, options, euler_angle_flag=False, mmg_flag=False):
    
    # State vector indices:
    # x[0:3] Linear Velocity
    # x[3:6] Angular Velocity
    # x[6:9] Linear positions
    # x[9:12] Euler Angles (Roll Pitch Yaw) or x[9:13] for quaternions
    # x[13:19] Control Surface Angles (6 surfaces)
    # x[19] Propeller RPM
    # print(euler_angle_flag)
    # Nondimensional State Space Variables
    print("Euler Angle Flag",euler_angle_flag)
    up = ss[0]; vp = ss[1]; wp = ss[2]  
    pp = ss[3]; qp = ss[4]; rp = ss[5] 
    xp = ss[6]; yp = ss[7]; zp = ss[8]  
   
    if euler_angle_flag:
        eul = ss[9:12]
        PROP_IDX = 19
    else:
        quat = ss[9:13]
        eul = kin.quat_to_eul(quat)
        PROP_IDX = 20
    
    phi = eul[0]; theta = eul[1]; psi = eul[2]

    # Get control surface angles from state vector
    delta = ss[13:19]  # All 6 control surfaces
    n_prop = ss[PROP_IDX]  # Propeller RPM
   
    v1 = ss[0:3]; v2 = ss[3:6]

    ## First let's set the mass matrix 
    ## Calculated in class_vessel.py in the function process_vessel_input()
    ## and then normalized in calculate_hydrodynamics() function in class_vessel
    M_RB = options['M_RB']      ## Rigid body mass matrix

    ## Added mass also calculated in calculate_hydrodynamics() function 
    ## Requries output from Hydra
    M_A = options['M_A']        ## Added mass matrix
    ## should be of form -diag(Xud,Yvd,Zwd,Kpd,Mqd,Nrd) (Ref Fossen)

    # np.linalg.inv(M_RB + A)
    M_inv = options['M_inv']    ## Inverse of mass matrix   

    ## Now let's do the coriolis and centripetal matrix calculation
    # Coriolis force calculation
    M11 = M_RB[0:3][:, 0:3]
    M12 = M_RB[0:3][:, 3:6]
    M21 = M_RB[3:6][:, 0:3]
    M22 = M_RB[3:6][:, 3:6]

    C_RB = np.zeros((6,6))
    C_RB[0:3][:, 3:6] = -kin.Smat(M11 @ v1 + M12 @ v2)
    C_RB[3:6][:, 0:3] = -kin.Smat(M11 @ v1 + M12 @ v2)
    C_RB[3:6][:, 3:6] = -kin.Smat(M21 @ v1 + M22 @ v2)

   
    A11 = M_A[0:3][:, 0:3]
    A12 = M_A[0:3][:, 3:6]
    A21 = M_A[3:6][:, 0:3]
    A22 = M_A[3:6][:, 3:6]

    C_A = np.zeros((6,6))
    C_A[0:3][:, 3:6] = -kin.Smat(A11 @ v1 + A12 @ v2)
    C_A[3:6][:, 0:3] = -kin.Smat(A11 @ v1 + A12 @ v2)
    C_A[3:6][:, 3:6] = -kin.Smat(A21 @ v1 + A22 @ v2)    

    tau_C = -(C_RB @ ss[0:6] + C_A @ ss[0:6])

    ## Now let us set the damping matrix
    ## Unlike for ships, potential damping for AUV underwater will be zero
    ## From Fossen's book pg.182, 6DOF Models for AUVs and ROVs (for low speed)
    ## -diagonal(Xu,Yv,Zw,Kp,Mq,Nr} Dl (Linear Damping)

    ## Coefficients, calculated from cross_flow_drag() function in class_vessel.py
    ## Need to change the v and r values in cross_flow_drag function
    ## Surge, Sway and Yaw
    Xuu = options['X_u_au']
    Yvv = options['Y_v_av']
    Yvr = options['Y_v_ar']
    Yrv = options['Y_r_av']
    Yrr = options['Y_r_ar']
    Nvv = options['N_v_av']
    Nvr = options['N_v_ar']
    Nrv = options['N_r_av']
    Nrr = options['N_r_ar']

    ## Also need to add Heave and Pitch Coefficients
    ## Need to change the q and w values in cross_flow_drag function
    Zww = options['Z_w_aw']
    Zwq = options['Z_w_aq']
    Zqw = options['Z_q_aw']
    Zqq = options['Z_q_aq']
    Mww = options['M_w_aw']
    Mwq = options['M_w_aq']
    Mqw = options['M_q_aw']
    Mqq = options['M_q_aq']

    ## Linear Damping (calculated in class_vessel)
    Dl = options['Dl'] 

    ## Non-Linear Damping, Surge and Cross Flow Drag (Sway and Heave)
    Dnl = np.zeros(6)    
    Dnl[0] = -Xuu * up * np.abs(up)
    Dnl[1] = -(Yvv * vp * np.abs(vp) + Yrv * rp * np.abs(vp) + Yvr * vp * np.abs(rp) + Yrr * rp * np.abs(rp))
    Dnl[5] = -(Nvv * vp * np.abs(vp) + Nrv * rp * np.abs(vp) + Nvr * vp * np.abs(rp) + Nrr * rp * np.abs(rp))

    Dnl[2] = -(Zww * wp * np.abs(wp) + Zqw * qp * np.abs(wp) + Zwq * wp * np.abs(qp) + Zqq * qp * np.abs(qp))
    Dnl[4] = -(Mww * wp * np.abs(wp) + Mqw * qp * np.abs(wp) + Mwq * wp * np.abs(qp) + Mqq * qp * np.abs(qp)) 

    ## This sums up the total Damping due to hydrodynamics of AUV
    tau_D = -(Dl @ ss[0:6]) - Dnl


    ## Let us now add Gravitational Forces for the AUV (Hydrostatics)
    W = options['W']
    B = options['B']
    r_bg = options['r_bg'] ## Vector from Body frame to COG
    r_bb = options['r_bb'] ## Vector from Body framr to Buoyancy
    g = gvect(W,B,ss[11],ss[10],r_bg,r_bb)

    ## Now let us derive the generalized forces due to the propeller
    ## Power Coefficients are calulated in calculate_hydrodynamics() in class_vessel.py
    
    n_prop = ss[PROP_IDX] ## Propeller RPM 
    tp = options['tp'] ## Thrust deduction coeff
    D_prop = options['D_prop']
    n_max = 2668/60


    ## 28.73 N thrust at 2668 RPM at 12V (from T200 datasheet)
    KT_at_J0 = 28.73/(1024*D_prop**4 *n_max**2)


    # Calculate advance ratio J with protection against division by zero
    denominator = n_prop * D_prop
    if abs(denominator) > 1e-6:  # Check if denominator is not too close to zero
        J = up / denominator
    else:
        J = 0.0  # Set a default value when propeller is not rotating
        
    KT = KT_at_J0*(1-J)
    
    X_prop = KT * 1024 * D_prop**4 * np.abs(n_prop) * n_prop ## Normalize this
    tau_P = np.zeros(6)
    tau_P[0] = X_prop

    ## Now let us calculate the forces due to the rudders, stern fin and bow fin   
    A_R = options['rudder_area']
    deltad_max = options['deltad_max']
       
    ## Need to add the following in class_vessel.py options
    ## Positions from Body Centre

    x_r1  = options['x_r1']      ## x position of upper rudder
    x_r2  = options['x_r2']      ## x position of down rudder
    x_sf1 = options['x_sf1']    ## x position of left fin
    x_sf2 = options['x_sf2']    ## x position of right fin
    x_b1  = options['x_b1']      ## x position of left bow fin
    x_b2  = options['x_b2']      ## x position of right bow fin

    y_r1  = options['y_r1']      ## y position of upper rudder
    y_r2  = options['y_r2']      ## y position of down rudder
    y_sf1 = options['y_sf1']     ## y position of left fin
    y_sf2 = options['y_sf2']     ## y position of right fin
    y_b1  = options['y_b1']      ## y position of left bow fin
    y_b2  = options['y_b2']      ## y position of right bow fin

    z_r1  = options['z_r1']      ## z position of upper rudder
    z_r2  = options['z_r2']      ## z position of down rudder
    z_sf1 = options['z_sf1']    ## z position of left fin
    z_sf2 = options['z_sf2']    ## z position of right fin
    z_b1  = options['z_b1']      ## z position of left bow fin
    z_b2  = options['z_b2']      ## z position of right bow fin

    ## delta = [UpperRudder,LowerRudder,LeftSternFin,RightSternFin,LeftBowFin,RightBowFin]
    # delta = ss[rud_indx] ## Rudder angle(s)
    # print("Delta: ", delta)

    ## For each particular fin/rudder, +ve delta is anticlockwise direction (outward the hull)

    ## Net flow angle is calculated accounting for the horizontal flow as well as flow due to rotation and rudder angle


    ## NACA profile of the aerofoil with data at Reynolds number 1M
    ## From Airfoilstools website
    NACAfile = "/workspaces/mavlab/ros2_ws/src/mav_simulator/mav_simulator/hullform/MAVYMINI/NACA0015.csv"


    if np.isscalar(delta):
        delta = np.zeros(6)
        
    ## Upper Rudder
    posr1 = [x_r1,y_r1,z_r1]
    fr1 = controlSurface(delta[0],ss[:6],posr1,NACAfile,[0,0,-1],A_R)
    
    ## Lower Rudder
    posr2 = [x_r2,y_r2,z_r2]
    fr2 = controlSurface(delta[1],ss[:6],posr2,NACAfile,[0,0,1],A_R)

    ##Left Stern Fin
    possf1 = [x_sf1,y_sf1,z_sf1]
    fsf1 = controlSurface(delta[2],ss[:6],possf1,NACAfile,[0,1,0],A_R)

    ##Right Stern Fin
    possf2 = [x_sf2,y_sf2,z_sf2]
    fsf2 = controlSurface(delta[3],ss[:6],possf2,NACAfile,[0,-1,0],A_R)

    ##Left Bow Fin
    posbf1 = [x_b1,y_b1,z_b1]
    fb1 = controlSurface(delta[4],ss[:6],posbf1,NACAfile,[0,1,0],A_R)

    ##Right Bow Fin
    posbf2 = [x_b2,y_b2,z_b2]
    fb2 = controlSurface(delta[5],ss[:6],posbf2,NACAfile,[0,-1,0],A_R)

    ## If angle goes above 20 degrees, that'll be a problem. (csv file does not have data for |alpha| > 20 degrees)


    ## Need to add roll moment due to propeller
    tau_R = fr1 + fr2 +fsf1 + fsf2 + fb1 + fb2

    
    T_rud = options['T_rud']
    T_prop = options['T_prop']
    nd_max = options['nd_max']
    

    L = options['L']
    U_des = options['U_des']


    # Total force calculation
    tau_C = tau_C.flatten()  # Already 1D (6,)
    tau_D = tau_D.flatten()  # Already 1D (6,)
    tau_P = tau_P.flatten()  # Already 1D (6,)
    tau_R = tau_R.flatten()  # Convert from (6,1) to (6,)
    g = g.flatten()         # Convert from (6,1) to (6,)

    # Total force calculation
    tau = tau_C + tau_D + tau_P + tau_R + g
    # print("Shape of tau: ", tau_R.shape,tau_C.shape,tau_P.shape,tau_R.shape,g.shape)

    # print(tau_P[0], tau_D[0], Xuu, up)

    # Derivative of velocity vector

    veld = M_inv @ tau

    # Derivative of state vector
    if euler_angle_flag:
        n = 20  # 6 (velocities) + 3 (positions) + 3 (euler) + 6 (control surfaces) + 1 (propeller) = 19
    else:
        n = 21  # 6 (velocities) + 3 (positions) + 4 (quaternion) + 6 (control surfaces) + 1 (propeller) = 20

    ssd = np.zeros(n)
    ssd[0:6] = veld

    if euler_angle_flag:
        ssd[6:9] = kin.eul_to_rotm(eul) @ v1
        ssd[9:12] = kin.eul_rate(eul, v2)
    else:
        ssd[6:9] = kin.quat_to_rotm(quat) @ v1
        ssd[9:13] = kin.quat_rate(quat, v2)
    
    
    # Rudder dynamics - handle all 6 control surfaces
    deltad = (delta_c - delta) / T_rud  # Element-wise for all 6 surfaces
    deltad = np.clip(deltad, -deltad_max, deltad_max)  # Apply limits to all surfaces
    # print(delta_c, delta, deltad)
    # Propeller dynamics
    nd_prop = (n_c - n_prop) / T_prop
    # Propeller speed rate saturation
    if np.abs(nd_prop) > nd_max:
        nd_prop = np.sign(nd_prop) * nd_max

    # Fill in derivatives
    ssd[0:6] = veld
    ssd[13:19] = deltad  # Control surface derivatives
    ssd[PROP_IDX] = nd_prop  # Propeller derivative

    return ssd



## delta = rudder/fin angle, sd = rudder/fin stock coordinate from CO
## vel = [u,v,w,p,q,r]
## delta convention is positive with normal coming out of the hull (normalAxis)
## Thus in case of head on flow for upper rudder, alpha = -delta

"""
Explanation of Control Surface Angle of Attack Calculation
========================================================

The following equation calculates the effective angle of attack (alpha) for AUV control surfaces:

alpha = delta * np.sum(normalAxis) + \
        normalAxis[2]*np.arctan2(netV*-(np.sum(normalAxis)),netU) + \
        normalAxis[1]*np.arctan2(netW*-(np.sum(normalAxis)),netU)

Components:
----------
1. delta: The commanded deflection angle of the control surface

2. normalAxis: A unit vector [x,y,z] indicating the direction the control surface normal points when deflection is zero
   - For vertical rudders: [0,0,±1]
   - For horizontal fins: [0,±1,0]

3. netU: Forward velocity component at the control surface
4. netV: Lateral velocity component (includes effects of yaw rate and roll rate)
5. netW: Vertical velocity component (includes effects of pitch rate and roll rate)

How it works:
------------
1. First term (delta * np.sum(normalAxis)):
   - For vertical rudders (normalAxis = [0,0,±1]): Contributes ±delta
   - For horizontal fins (normalAxis = [0,±1,0]): Contributes ±delta
   - The sign ensures proper convention for positive lift direction

2. Second term (normalAxis[2]*np.arctan2(netV*-(np.sum(normalAxis)),netU)):
   - Only active for vertical rudders (normalAxis[2] = ±1)
   - Calculates the effective inflow angle due to lateral velocity
   - Important for yaw control and sideslip

3. Third term (normalAxis[1]*np.arctan2(netW*-(np.sum(normalAxis)),netU)):
   - Only active for horizontal fins (normalAxis[1] = ±1)
   - Calculates the effective inflow angle due to vertical velocity
   - Important for pitch control and heave motion

Example scenarios:
----------------
1. Upper Rudder (normalAxis = [0,0,-1]):
   alpha = -delta + arctan2(netV,netU)
   - Positive delta creates positive sway force
   - Accounts for sideslip angle

2. Starboard Stern Fin (normalAxis = [0,-1,0]):
   alpha = -delta + arctan2(netW,netU)
   - Positive delta creates positive heave force
   - Accounts for angle of attack due to pitch/heave motion

This formulation ensures proper calculation of lift and drag forces regardless of 
control surface orientation and local flow conditions, which is crucial for 
accurate AUV dynamics simulation.
"""

def controlSurface(delta,vel,sd,NACAfile,normalAxis,A_R):
    """
    genForce = controlSurface(delta,vel,sd,NACAfile,normalAxis,A_R) computes the 6x1 vector of generalized forces due
    to control surface of an AUV. 

    Inputs:
        delta - rudder angle
        vel - velocity vector [u,v,w,p,q,r]
        sd - rudder/fin stock coordinate from CO [x,y,z]
        NACAfile - path for the NACA file from aerofoiltools
        normalAxis - the outward normal (out of hull) of the rudder with respect to the body frame coordinates
        A_R - Rudder area

    Returns:
        genForce: 6x1 vector of generalized force due to the control surface
    """
    data = pd.read_csv(NACAfile)
    genForce = np.zeros((6,1)) ## generalized Force
    u,v,w,p,q,r = vel  
    netU = u
    netV = v + r*sd[0] + p*sd[2]  ## yaw and roll effects on sway
    netW = w + p*sd[1] - q*sd[0]  ## roll and pitch effects on heave
    netVel = np.sqrt(netU**2+netV**2+netW**2)
    ## Written in a manner such that it automatically takes care of the flow plane
    alpha = delta * np.sum(normalAxis) + \
            normalAxis[2]*np.arctan2(netV*-(np.sum(normalAxis)),netU) + \
            normalAxis[1]*np.arctan2(netW*-(np.sum(normalAxis)),netU)
    Cl = np.interp(np.rad2deg(alpha), data['Alpha'], data['Cl'])
    Cd = np.interp(np.rad2deg(alpha), data['Alpha'], data['Cd'])
    
    # Drag force calculation - combines effects from both vertical or horizontal components
    # Uses absolute values since drag acts in opposition to motion regardless of direction
    # Surge 
    genForce[0] = np.abs(normalAxis[2])*(Cd*A_R*(netU**2+netV**2)) + np.abs(normalAxis[1])*(Cd*A_R*(netU**2+netW**2))
    
    # Lateral force (sway) - generated by vertical control surfaces
    # Proportional to lift coefficient and dynamic pressure (U^2 + V^2)
    genForce[1] = normalAxis[2]*(Cl*A_R*(netU**2+netV**2))
    
    # Vertical force (heave) - generated by horizontal control surfaces
    # Proportional to lift coefficient and dynamic pressure (U^2 + W^2)
    genForce[2] = normalAxis[1]*(Cl*A_R*(netU**2+netW**2))
    
    # Rolling moment - caused by both lateral and vertical forces
    # Moment arm defined by surface distance vector (sd)
    genForce[3] = genForce[1]*sd[0] + genForce[2]*sd[1]  # Roll due to Y and Z forces
    
    # Pitching moment - caused by vertical force only
    genForce[4] = genForce[2]*sd[0]                      # Pitch due to Z force
    
    # Yawing moment - caused by drag and lateral forces
    genForce[5] = genForce[1]*sd[0] + genForce[0]*sd[2]  # Yaw due to X and Y forces

    return genForce

## From FOSSEN code
def gvect(W,B,theta,phi,r_bg,r_bb):
    """
    g = gvect(W,B,theta,phi,r_bg,r_bb) computes the 6x1 vector of restoring 
    forces about an arbitrarily point CO for a submerged body. 

    Inputs:
        W, B: weight and buoyancy (kg)
        phi,theta: roll and pitch angles (rad)
        r_bg = [x_g y_g z_g]: location of the CG with respect to the CO (m)
        r_bb = [x_b y_b z_b]: location of the CB with respect to th CO (m)
        
    Returns:
        g: 6x1 vector of restoring forces about CO
    """
    sth  = np.sin(theta)
    cth  = np.cos(theta)
    sphi = np.sin(phi)
    cphi = np.cos(phi)

    g = np.array([
        (W-B) * sth,
        -(W-B) * cth * sphi,
        -(W-B) * cth * cphi,
        -(r_bg[1]*W-r_bb[1]*B) * cth * cphi + (r_bg[2]*W-r_bb[2]*B) * cth * sphi,
        (r_bg[2]*W-r_bb[2]*B) * sth         + (r_bg[0]*W-r_bb[0]*B) * cth * cphi,
        -(r_bg[0]*W-r_bb[0]*B) * cth * sphi - (r_bg[1]*W-r_bb[1]*B) * sth      
        ])
    
    return g
         


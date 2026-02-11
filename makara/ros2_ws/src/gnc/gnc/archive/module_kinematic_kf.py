import numpy as np

import sys
import os
module_path = '/workspaces/mavlab/ros2_ws/src/mav_simulator/mav_simulator'
sys.path.append(os.path.abspath(module_path))
import module_kinematics as kin

def d_by_dTheta_nb_R_b_n_v_nb_b(Theta_nb, v_nb_b):

    phi = Theta_nb[0]
    theta = Theta_nb[1]
    psi = Theta_nb[2]

    s1 = np.sin(phi); c1 = np.cos(phi)
    s2 = np.sin(theta); c2 = np.cos(theta)
    s3 = np.sin(psi); c3 = np.cos(psi)

    # R_b_n = np.array([
    #     [c2*c3, -c1*s3+s1*s2*c3, s1*s3+c1*s2*c3],
    #     [c2*s3, c1*c3+s1*s2*s3, -s1*c3+c1*s2*s3],
    #     [-s2, s1*c2, c1*c2]
    # ])

    d_by_dphi_R_b_n = np.array([
        [0, s1*s3+c1*s2*c3, c1*s3-s1*s2*c3],
        [0, -s1*c3+c1*s2*s3, -c1*c3-s1*s2*s3],
        [0, c1*c2, -s1*c2]
    ])

    d_by_dtheta_R_b_n = np.array([
        [-s2*c3, s1*c2*c3, c1*c2*c3],
        [-s2*s3, s1*c2*s3, c1*c2*s3],
        [-c2, -s1*s2, -c1*s2]
    ])

    d_by_dpsi_R_b_n = np.array([
        [-c2*s3, -c1*c3-s1*s2*s3, s1*c3-c1*s2*s3],
        [c2*c3, -c1*s3+s1*s2*c3, s1*s3+c1*s2*c3],
        [0, 0, 0]
    ])

    derivative_mat = np.zeros((3,3))
    derivative_mat[:, 0] = d_by_dphi_R_b_n @ v_nb_b
    derivative_mat[:, 1] = d_by_dtheta_R_b_n @ v_nb_b
    derivative_mat[:, 2] = d_by_dpsi_R_b_n @ v_nb_b

    return derivative_mat

def d_by_dTheta_nb_J2_w_nb_b(Theta_nb, w_nb_b):
    
    phi = Theta_nb[0]
    theta = Theta_nb[1]
    psi = Theta_nb[2]

    s1 = np.sin(phi); c1 = np.cos(phi)
    s2 = np.sin(theta); c2 = np.cos(theta)
    s3 = np.sin(psi); c3 = np.cos(psi)

    # J2 = np.array([
    #     [1, s1*s2/c2, c1*s2/c2],
    #     [0, c1, -s1],
    #     [0, s1/c2, c1/c2]
    # ])

    d_by_dphi_J2 = np.array([
        [0, c1*s2/c2, -s1*s2/c2],
        [0, -s1, -c1],
        [0, c1/c2, -s1/c2]
    ])

    d_by_dtheta_J2 = np.array([
        [0, s1/(c2**2), c1/(c2**2)],
        [0, 0, 0],
        [0, s1*s2/(c2**2), c1*s2/(c2**2)]
    ])

    d_by_dpsi_J2 = np.zeros((3,3))

    derivative_mat = np.zeros((3,3))
    derivative_mat[:, 0] = d_by_dphi_J2 @ w_nb_b
    derivative_mat[:, 1] = d_by_dtheta_J2 @ w_nb_b
    derivative_mat[:, 2] = d_by_dpsi_J2 @ w_nb_b

    return derivative_mat

def d_by_dq_b_n_R_b_n_v_nb_b(q_b_n, v_nb_b):
    eta = q_b_n[0]
    e1 = q_b_n[1]
    e2 = q_b_n[2]
    e3 = q_b_n[3]

    d_by_deta_R_b_n = np.array([
        [0, -2*e3, 2*e2],
        [2*e3, 0, -2*e1],
        [-2*e2, 2*e1, 0]
    ])

    d_by_deps1_R_b_n = np.array([
        [0, 2*e2, 2*e3],
        [2*e2, -4*e1, 2*eta],
        [2*e3, 2*eta, -4*e1]
    ])

    d_by_deps2_R_b_n = np.array([
        [-4*e2, 2*e1, 2*eta],
        [2*e1, 0, 2*e3],
        [-2*eta, 2*e3, -4*e2]
    ])

    d_by_deps3_R_b_n = np.array([
        [-4*e3, -2*eta, 2*e1],
        [2*eta, -4*e3, 2*e2],
        [2*e1, 2*e2, 0]
    ])

    derivative_mat = np.zeros((3, 4))
    derivative_mat[:, 0] = d_by_deta_R_b_n @ v_nb_b
    derivative_mat[:, 1] = d_by_deps1_R_b_n @ v_nb_b
    derivative_mat[:, 2] = d_by_deps2_R_b_n @ v_nb_b
    derivative_mat[:, 3] = d_by_deps3_R_b_n @ v_nb_b

    return derivative_mat

def d_by_dq_b_n_J2_w_nb_b(q_b_n, w_nb_b):
    
    d_by_deta_J2 = 0.5 * np.array([
        [0,0,0],
        [1,0,0],
        [0,1,0],
        [0,0,1]
    ])

    d_by_deps1_J2 = 0.5 * np.array([
        [-1,0,0],
        [0,0,0],
        [0,0,-1],
        [0,1,0]
    ])

    d_by_deps2_J2 = 0.5 * np.array([
        [0,-1,0],
        [0,0,1],
        [0,0,0],
        [-1,0,0]
    ])

    d_by_deps3_J2 = 0.5 * np.array([
        [0,0,-1],
        [0,-1,0],
        [1,0,0],
        [0,0,0]
    ])

    derivative_mat = np.zeros((4,4))
    derivative_mat[:, 0] = d_by_deta_J2 @ w_nb_b
    derivative_mat[:, 1] = d_by_deps1_J2 @ w_nb_b
    derivative_mat[:, 2] = d_by_deps2_J2 @ w_nb_b
    derivative_mat[:, 3] = d_by_deps3_J2 @ w_nb_b

    return derivative_mat

def d_by_dw_nb_b_a_ni_i(w_nb_b, r_bi_b, q_i_b):
    w1 = w_nb_b[0]
    w2 = w_nb_b[1]
    w3 = w_nb_b[2]

    R_b_i = kin.quat_to_rotm(q_i_b)    

    d_by_dw1_S2_w_nb_b = np.array([
        [0, w2, w3],
        [w2, -2*w1, 0],
        [w3, 0, -2*w1]
    ])

    d_by_dw2_S2_w_nb_b = np.array([
        [-2*w2, w1, 0],
        [w1, 0, w3],
        [0, w3, -2*w2]
    ])

    d_by_dw3_S2_w_nb_b = np.array([
        [-2*w3, 0, w1],
        [0, -2*w3, w2],
        [w1, w2, 0]
    ])

    derivative_mat = np.zeros((3,3))
    derivative_mat[:, 0] = R_b_i @ d_by_dw1_S2_w_nb_b @ r_bi_b
    derivative_mat[:, 1] = R_b_i @ d_by_dw2_S2_w_nb_b @ r_bi_b
    derivative_mat[:, 2] = R_b_i @ d_by_dw3_S2_w_nb_b @ r_bi_b

    return derivative_mat

def kinematic_ode(t, ss, delta_c, n_c, options, euler_angle_flag=True):    

    deltad_max = options['deltad_max']
    if deltad_max is None:
        deltad_max = 3 * np.pi / 180 * (options['L'] / options['U_des'])
    

    T_rud = options['T_rud']
    T_prop = options['T_prop']

    nd_max = options['nd_max']
    if nd_max is None:
        nd_max = 100

    v_nb_b = ss[0:3]
    w_nb_b = ss[3:6]
    r_nb_n = ss[6:9]
    
    if euler_angle_flag:
        Theta_nb = ss[9:12]
        R_b_n = kin.eul_to_rotm(Theta_nb)
        q_b_n = kin.rotm_to_quat(R_b_n)
    else:
        q_b_n = ss[9:13]
        Theta_nb = kin.quat_to_eul(quat)
        R_b_n = kin.quat_to_rotm(q_b_n)
    
    if euler_angle_flag:
        n = 17
        rud_indx = 12
        prop_indx = 13
    else:
        n = 18
        rud_indx = 13
        prop_indx = 14
    
    delta = ss[rud_indx]
    n_prop = ss[prop_indx]

    a_nb_b = ss[-3:]

    ssd = np.zeros(n)

    ssd[0:3] = a_nb_b - np.cross(w_nb_b, v_nb_b)
    ssd[6:9] = R_b_n @ v_nb_b    

    if euler_angle_flag:
        ssd[9:12] = kin.eul_rate_matrix(Theta_nb) @ w_nb_b
    else:
        ssd[9:13] = kin.quat_rate_matrix(q_b_n) @ w_nb_b
    
    # Rudder dynamics
    deltad = (delta_c - delta) / T_rud
    deltad_max = deltad_max
    # Rudder rate saturation
    if np.abs(deltad) > deltad_max:
        deltad = np.sign(deltad) * deltad_max

    # Propeller dynamics
    nd_prop = (n_c - n_prop) / T_prop
    # Propeller speed rate saturation
    if np.abs(nd_prop) > nd_max:
        nd_prop = np.sign(nd_prop) * nd_max

    ssd[rud_indx] = deltad
    ssd[prop_indx] = nd_prop

    ssd[-3:] = - np.cross(w_nb_b, a_nb_b)

    return ssd

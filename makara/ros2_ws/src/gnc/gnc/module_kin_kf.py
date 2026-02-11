import numpy as np
from dTheta_ns import dTheta_ns_dphi, dTheta_ns_dtheta, dTheta_ns_dpsi
from mav_simulator.module_kinematics import eul_to_rotm, Smat, rotm_to_eul
from mav_simulator.module_kinematics import eul_rate_matrix as J2mat

def S1mat(vec):
    return np.array([
        [0, vec[1], vec[2]],
        [vec[1], -2*vec[0], 0],
        [vec[2], 0, -2*vec[0]]
    ])

def S2mat(vec):
    return np.array([
        [-2*vec[1], vec[0], 0],
        [vec[0], 0, vec[2]],
        [0, vec[2], -2*vec[1]]        
    ])

def S3mat(vec):
    return np.array([
        [-2*vec[2], 0, vec[0]],
        [0, -2*vec[2], vec[1]],
        [vec[0], vec[1], 0]
    ])

def dR_dphi(eul):
    phi = eul[0]
    theta = eul[1]
    psi = eul[2]
    
    s1 = np.sin(phi); c1 = np.cos(phi)
    s2 = np.sin(theta); c2 = np.cos(theta)
    s3 = np.sin(psi); c3 = np.cos(psi)

    dR = np.array([
        [0, s1*s3 + s2*c1*c3, -s1*s2*c3 + s3*c1],
        [0, -s1*c3 + s3*s2*c1, -s1*s3*s2 - c1*c3],
        [0, c1*c2, -s1*c2]
    ])
    
    return dR

def dR_dtheta(eul):
    phi = eul[0]
    theta = eul[1]
    psi = eul[2]
    
    s1 = np.sin(phi); c1 = np.cos(phi)
    s2 = np.sin(theta); c2 = np.cos(theta)
    s3 = np.sin(psi); c3 = np.cos(psi)

    dR = np.array([
        [-s2*c3, s1*c3*c2, c1*c3*c2],
        [-s3*s2, s3*s1*c2, s3*c1*c2],
        [-c2, -s1*s2, -s2*c1]
    ])
    return dR

def dR_dpsi(eul):
    phi = eul[0]
    theta = eul[1]
    psi = eul[2]
    
    s1 = np.sin(phi); c1 = np.cos(phi)
    s2 = np.sin(theta); c2 = np.cos(theta)
    s3 = np.sin(psi); c3 = np.cos(psi)

    dR = np.array([
        [-s3*c2, -s3*s1*s2 - c1*c3, s1*c3 - s3*s2*c1],
        [c3*c2, s1*s2*c3 - s3*c1, s1*s3 + s2*c1*c3],
        [0, 0, 0]
    ])
    return dR

def dJ2_dphi(eul):
    phi = eul[0]
    theta = eul[1]
    psi = eul[2]
    
    s1 = np.sin(phi); c1 = np.cos(phi)
    s2 = np.sin(theta); c2 = np.cos(theta)
    s3 = np.sin(psi); c3 = np.cos(psi)

    dJ = np.array([
        [0, c1*s2/c2, -s1*s2/c2],
        [0, -s1, -c1],
        [0, c1/c2, -s1/c2]
    ])
    return dJ

def dJ2_dtheta(eul):
    phi = eul[0]
    theta = eul[1]
    psi = eul[2]
    
    s1 = np.sin(phi); c1 = np.cos(phi)
    s2 = np.sin(theta); c2 = np.cos(theta)
    s3 = np.sin(psi); c3 = np.cos(psi)

    dJ = np.array([
        [0, s1/(c2**2), c1/(c2**2) ],
        [0, 0, 0],
        [0, s1*s2/(c2**2), c1*s2/(c2**2)]
    ])
    return dJ

def dJ2_dpsi(eul):
    phi = eul[0]
    theta = eul[1]
    psi = eul[2]
    
    s1 = np.sin(phi); c1 = np.cos(phi)
    s2 = np.sin(theta); c2 = np.cos(theta)
    s3 = np.sin(psi); c3 = np.cos(psi)

    dJ = np.zeros((3,3))
    return dJ

def L1mat(eul, v_nb_b):
    L1 = np.zeros((3,3))
    L1[:, 0] = dR_dphi(eul) @ v_nb_b
    L1[:, 1] = dR_dtheta(eul) @ v_nb_b
    L1[:, 2] = dR_dpsi(eul) @ v_nb_b
    return L1

def L2mat(eul, omg_nb_b):
    L2 = np.zeros((3,3))
    L2[:, 0] = dJ2_dphi(eul) @ omg_nb_b
    L2[:, 1] = dJ2_dtheta(eul) @ omg_nb_b
    L2[:, 2] = dJ2_dpsi(eul) @ omg_nb_b
    return L2

def L3mat(eul, eul_s):
    L3 = np.zeros((3,3))
    L3[:, 0] = dTheta_ns_dphi(eul[0], eul[1], eul[2], eul_s[0], eul_s[1], eul_s[2]).flatten()
    L3[:, 1] = dTheta_ns_dtheta(eul[0], eul[1], eul[2], eul_s[0], eul_s[1], eul_s[2]).flatten()
    L3[:, 2] = dTheta_ns_dpsi(eul[0], eul[1], eul[2], eul_s[0], eul_s[1], eul_s[2]).flatten()
    return L3

def L4mat(omg_nb_b, r_bs_b, Theta_bs):
    L4 = np.zeros((3,3))
    R_s_b = eul_to_rotm(Theta_bs)
    R_b_s = R_s_b.T
    L4[:, 0] = R_b_s @ S1mat(omg_nb_b) @ r_bs_b
    L4[:, 1] = R_b_s @ S2mat(omg_nb_b) @ r_bs_b
    L4[:, 2] = R_b_s @ S3mat(omg_nb_b) @ r_bs_b
    return L4

def L5mat(eul, r_bs_b):
    L5 = np.zeros((3,3))
    L5[:, 0] = dR_dphi(eul) @ r_bs_b
    L5[:, 1] = dR_dtheta(eul) @ r_bs_b
    L5[:, 2] = dR_dpsi(eul) @ r_bs_b
    return L5

def state_mats(x):
    """Calculate state matrices A and E for the extended Kalman filter.
    
    Args:
        x: State vector containing positions, attitudes, velocities, angular rates, 
           accelerations, and actuator positions
    
    Returns:
        A, E: State transition and noise influence matrices
    """
    # Get state dimensions
    n_states = len(x)
    n_actuators = max(0, n_states - 15)  # Number of actuator states
    
    r_nb_n = x[0:3]
    Theta_nb = x[3:6]
    v_nb_b = x[6:9]
    omg_nb_b = x[9:12]
    acc_nb_b = x[12:15]
    
    # Create appropriately sized matrices
    A = np.zeros((n_states, n_states))
    
    # Fill the standard 15x15 part of the matrix
    A[0:3][:, 3:6] = L1mat(Theta_nb, v_nb_b)
    A[0:3][:, 6:9] = eul_to_rotm(Theta_nb)
    
    A[3:6][:, 3:6] = L2mat(Theta_nb, omg_nb_b)
    A[3:6][:, 9:12] = J2mat(Theta_nb)
    
    A[6:9][:, 6:9] = -Smat(omg_nb_b)
    A[6:9][:, 9:12] = Smat(v_nb_b)
    A[6:9][:, 12:15] = np.eye(3)

    A[12:15][:, 9:12] = Smat(acc_nb_b)
    A[12:15][:, 12:15] = -Smat(omg_nb_b)
    
    # For actuator states, dynamics are modeled as first-order with time constant
    # Actuator derivatives are proportional to difference between command and current position
    # This is handled in the plant_model function
    
    # Create noise influence matrix - always use 6 columns for compatibility with EKF
    E = np.zeros((n_states, 6))

    E[9:12][:, 0:3] = eul_to_rotm(Theta_nb).T
    E[12:15][:, 3:6] = eul_to_rotm(Theta_nb).T
    

    return A, E

def imu_mat(x, r_bs_b=np.array([0.0, 0.0, 0.0]), Theta_bs=np.array([0.0, 0.0, 0.0])):
    
    r_nb_n = x[0:3]
    Theta_nb = x[3:6]
    v_nb_b = x[6:9]
    omg_nb_b = x[9:12]
    acc_nb_b = x[12:15]
    
    C = np.zeros((9, len(x)))  # Extend for actuator states
    
    C[0:3][:, 3:6] = L3mat(Theta_nb, Theta_bs)
    C[3:6][:, 9:12] = eul_to_rotm(Theta_bs).T
    C[6:9][:, 9:12] = L4mat(omg_nb_b, r_bs_b, Theta_bs)
    C[6:9][:, 12:15] = eul_to_rotm(Theta_bs).T
    
    return C

def gnss_mat(x, r_bs_b=np.array([0.0, 0.0, 0.0]), Theta_bs=np.array([0.0, 0.0, 0.0])):

    r_nb_n = x[0:3]
    Theta_nb = x[3:6]
    v_nb_b = x[6:9]
    omg_nb_b = x[9:12]
    acc_nb_b = x[12:15]
    
    C = np.zeros((3, len(x)))  # Extend for actuator states
    C[0:3][:, 0:3] = np.eye(3)
    C[0:3][:, 3:6] = L5mat(Theta_nb, r_bs_b)

    return C

def encoder_mat(x, actuator_idx=None):
    """Create measurement matrix for encoder sensors.
    
    Args:
        x: Full state vector
        actuator_idx: Index of the actuator in the state vector
    
    Returns:
        C: Observation matrix for the encoder
    """
    if actuator_idx is None:
        raise ValueError("Actuator index must be specified for encoder")
    
    # Create the measurement matrix (just selects the actuator state directly)
    C = np.zeros((1, len(x)))
    C[0, actuator_idx] = 1.0
    
    return C

def plant_model(t, x, u):
    """State dynamics model for EKF prediction.
    
    Args:
        t: Time
        x: State vector
        u: Input vector
        
    Returns:
        xd: State derivative vector
    """
    # Get state dimensions and determine how many actuator states we have
    n_states = len(x)
    n_actuators = max(0, n_states - 15)
    
    # Extract standard states
    r_nb_n = x[0:3]
    Theta_nb = x[3:6]
    v_nb_b = x[6:9]
    omg_nb_b = x[9:12]
    a_nb_b = x[12:15]
    
    # Extract actuator states if present
    actuators = x[15:] if n_actuators > 0 else np.array([])

    R_b_n = eul_to_rotm(Theta_nb)
    J2_nb = J2mat(Theta_nb)

    # Initialize state derivative vector
    xd = np.zeros(n_states)

    # Standard dynamics for vessel states
    xd[0:3] = R_b_n @ v_nb_b
    xd[3:6] = J2_nb @ omg_nb_b
    xd[6:9] = a_nb_b + np.cross(v_nb_b, omg_nb_b)
    xd[9:12] = 0.0  # Angular acceleration is not modeled
    xd[12:15] = np.cross(a_nb_b, omg_nb_b)
    
    # Actuator dynamics - default to small decay to prevent unbounded growth
    # In reality, these would be driven by control inputs, which would typically come from measurements
    if n_actuators > 0:
        # Model as simple first-order system with long time constant (e.g., 10 seconds)
        time_constant = 0.1 #TODO need to change this and take from vessel config
        xd[15:] = -actuators / time_constant  # Slow decay toward zero when no measurements
        
    return xd

def imu_model(x, r_bs_b=np.array([0.0, 0.0, 0.0]), Theta_bs=np.array([0.0, 0.0, 0.0])):

    r_nb_n = x[0:3]
    Theta_nb = x[3:6]
    v_nb_b = x[6:9]
    omg_nb_b = x[9:12]
    a_nb_b = x[12:15]

    R_b_n = eul_to_rotm(Theta_nb)

    R_s_b = eul_to_rotm(Theta_bs)
    R_b_s = R_s_b.T

    y = np.zeros(9)
    y[0:3] = rotm_to_eul(R_b_n @ R_s_b)
    y[3:6] = R_b_s @ omg_nb_b
    y[6:9] = R_b_s @ a_nb_b + R_b_s @ np.cross(omg_nb_b, np.cross(omg_nb_b, r_bs_b))

    return y

def gnss_model(x, r_bs_b=np.array([0.0, 0.0, 0.0]), Theta_bs=np.array([0.0, 0.0, 0.0])):

    r_nb_n = x[0:3]
    Theta_nb = x[3:6]

    R_b_n = eul_to_rotm(Theta_nb)

    y = r_nb_n + R_b_n @ r_bs_b

    return y

def encoder_model(x, actuator_idx=None):
    """Measurement model for encoder sensors.
    
    Args:
        x: Full state vector
        actuator_idx: Index of the actuator in the state vector
    
    Returns:
        y: Measurement value (actuator position)
    """
    if actuator_idx is None:
        raise ValueError("Actuator index must be specified for encoder")
    
    # Simply return the actuator state
    return np.array([x[actuator_idx]])
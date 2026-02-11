import casadi as cd  
from casadi_module_kinematics import Smat

def Smat(v):
    """
    Skew-symmetric matrix for cross-product operation.
    v: 3x1 CasADi SX or MX vector
    """
    return cd.vertcat(
        cd.horzcat(0, -v[2], v[1]),
        cd.horzcat(v[2], 0, -v[0]),
        cd.horzcat(-v[1], v[0], 0)
    )

def coriolis_matrix(M, vel):
    """
    CasADi-based Coriolis matrix computation.
    
    Args:
        M (CasADi SX/MX): 6x6 mass matrix
        vel (CasADi SX/MX): 6x1 velocity vector
        
    Returns:
        CasADi SX: 6x6 Coriolis matrix
    """
    # Extract submatrices
    M11 = M[0:3, 0:3]
    M12 = M[0:3, 3:6]
    M21 = M[3:6, 0:3]
    M22 = M[3:6, 3:6]

    # Split velocity vector
    v1 = vel[0:3]
    v2 = vel[3:6]

    # Initialize C as symbolic zero matrix
    C = cd.SX.zeros(6, 6)

    # Compute components
    term1 = M11 @ v1 + M12 @ v2
    term2 = M21 @ v1 + M22 @ v2

    # Fill Coriolis matrix blocks
    C[0:3, 3:6] = -Smat(term1)
    C[3:6, 0:3] = -Smat(term1)
    C[3:6, 3:6] = -Smat(term2)
    print("Coriolis matrix computed using CasADi.",C)

    return C 

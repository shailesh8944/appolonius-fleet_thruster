import casadi as ca

# -----------------------------
# Skew-symmetric matrix
# -----------------------------
def Smat(vec):
    S = ca.SX.zeros(3,3)
    S[0,1] = -vec[2]; S[0,2] = vec[1]
    S[1,0] = vec[2];  S[1,2] = -vec[0]
    S[2,0] = -vec[1]; S[2,1] = vec[0]
    return S

# -----------------------------
# Euler angles to rotation matrix
# -----------------------------
def eul_to_rotm(angles):
    phi, theta, psi = angles[0], angles[1], angles[2]

    R_x = ca.vertcat(
        ca.horzcat(1,0,0),
        ca.horzcat(0, ca.cos(phi), -ca.sin(phi)),
        ca.horzcat(0, ca.sin(phi), ca.cos(phi))
    )

    R_y = ca.vertcat(
        ca.horzcat(ca.cos(theta),0,ca.sin(theta)),
        ca.horzcat(0,1,0),
        ca.horzcat(-ca.sin(theta),0,ca.cos(theta))
    )

    R_z = ca.vertcat(
        ca.horzcat(ca.cos(psi), -ca.sin(psi), 0),
        ca.horzcat(ca.sin(psi), ca.cos(psi),0),
        ca.horzcat(0,0,1)
    )

    return R_z @ R_y @ R_x

# -----------------------------
# Rotation matrix to Euler angles (ZYX)
# -----------------------------
def rotm_to_eul(rotm):
    theta = ca.asin(-rotm[2,0])
    eps = 1e-6
    phi = ca.if_else(ca.fabs(theta - ca.pi/2) < eps, 0, ca.atan2(rotm[2,1], rotm[2,2]))
    psi = ca.if_else(ca.fabs(theta - ca.pi/2) < eps, ca.atan2(rotm[0,1], rotm[1,1]), ca.atan2(rotm[1,0], rotm[0,0]))
    return ca.vertcat(phi, theta, psi)

# -----------------------------
# Euler rate matrix
# -----------------------------
def eul_rate_matrix(angles):
    phi, theta = angles[0], angles[1]
   # Create matrix using vertcat and horzcat instead of direct assignment
    J2 = ca.vertcat(
        ca.horzcat(1, ca.sin(phi)*ca.tan(theta), ca.cos(phi)*ca.tan(theta)),
        ca.horzcat(0, ca.cos(phi), -ca.sin(phi)),
        ca.horzcat(0, ca.sin(phi)/ca.cos(theta), ca.cos(phi)/ca.cos(theta))
    )
    
    return J2

# -----------------------------
# Euler to quaternion (ZYX)
# -----------------------------
def eul_to_quat(eul):
    phi, theta, psi = eul[0], eul[1], eul[2]
    cr, sr = ca.cos(phi/2), ca.sin(phi/2)
    cp, sp = ca.cos(theta/2), ca.sin(theta/2)
    cy, sy = ca.cos(psi/2), ca.sin(psi/2)
    qw = cr*cp*cy + sr*sp*sy
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy
    quat = ca.vertcat(qw, qx, qy, qz)
    return quat / ca.norm_2(quat)

# -----------------------------
# Quaternion to rotation matrix
# -----------------------------
def quat_to_rotm(quat):
    qw, qx, qy, qz = quat[0], quat[1], quat[2], quat[3]
    R = ca.SX(3,3)
    R[0,0] = 1 - 2*(qy**2 + qz**2); R[0,1] = 2*(qx*qy - qw*qz); R[0,2] = 2*(qx*qz + qw*qy)
    R[1,0] = 2*(qx*qy + qw*qz);     R[1,1] = 1 - 2*(qx**2 + qz**2); R[1,2] = 2*(qy*qz - qw*qx)
    R[2,0] = 2*(qx*qz - qw*qy);     R[2,1] = 2*(qy*qz + qw*qx);     R[2,2] = 1 - 2*(qx**2 + qy**2)
    return R

# -----------------------------
# Quaternion multiplication
# -----------------------------
def quat_multiply(q1,q2):
    w1,x1,y1,z1 = q1[0], q1[1], q1[2], q1[3]
    w2,x2,y2,z2 = q2[0], q2[1], q2[2], q2[3]
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    return ca.vertcat(w,x,y,z)

# -----------------------------
# Quaternion conjugate
# -----------------------------
def quat_conjugate(q):
    return ca.vertcat(q[0], -q[1], -q[2], -q[3])

# -----------------------------
# Rotate vector by quaternion
# -----------------------------
def rotate_vec_by_quat(vec, q):
    q_vec = ca.vertcat(0, vec)
    return quat_multiply(quat_multiply(q, q_vec), quat_conjugate(q))[1:4]

# -----------------------------
# Quaternion rate
# -----------------------------
def quat_rate(quat, w):
    qw,qx,qy,qz = quat[0], quat[1], quat[2], quat[3]
    wx, wy, wz = w[0], w[1], w[2]
    Omega = ca.SX(4,4)
    Omega[0,:] = [0,-wx,-wy,-wz]
    Omega[1,:] = [wx,0,wz,-wy]
    Omega[2,:] = [wy,-wz,0,wx]
    Omega[3,:] = [wz,wy,-wx,0]
    return 0.5 * Omega @ quat

# -----------------------------
# Smallest signed angle
# -----------------------------
def ssa(ang):

    # Wrap angle to [-pi, pi] using atan2(sin, cos)
    return ca.atan2(ca.sin(ang), ca.cos(ang))

# -----------------------------
# LLH <-> NED conversions
# -----------------------------
def ned_to_llh(ned, llh0):
    R_E = 6378137.0
    mu0,l0,h0 = llh0[0]*ca.pi/180, llh0[1]*ca.pi/180, llh0[2]
    mu = mu0 + ned[0]/R_E
    l = l0 + ned[1]/(R_E*ca.cos(mu0))
    h = h0 - ned[2]
    return ca.vertcat(mu*180/ca.pi, l*180/ca.pi, h)

def llh_to_ned(llh, llh0):
    R_E = 6378137.0
    mu, l, h = llh[0]*ca.pi/180, llh[1]*ca.pi/180, llh[2]
    mu0,l0,h0 = llh0[0]*ca.pi/180, llh0[1]*ca.pi/180, llh0[2]
    xn = (mu - mu0) * R_E
    yn = (l - l0) * R_E * ca.cos(mu0)
    zn = h0 - h
    return ca.vertcat(xn,yn,zn)

# -----------------------------
# LLH <-> ECEF conversions
# -----------------------------
def llh_to_ecef(llh):
    a = 6378137.0
    e2 = 6.69437999014e-3
    lat, lon, h = llh[0]*ca.pi/180, llh[1]*ca.pi/180, llh[2]
    N = a / ca.sqrt(1 - e2 * ca.sin(lat)**2)
    x = (N+h)*ca.cos(lat)*ca.cos(lon)
    y = (N+h)*ca.cos(lat)*ca.sin(lon)
    z = (N*(1-e2)+h)*ca.sin(lat)
    return ca.vertcat(x,y,z)

def ecef_to_llh(ecef):
    a = 6378137.0
    e2 = 6.69437999014e-3
    x,y,z = ecef[0], ecef[1], ecef[2]
    lon = ca.atan2(y,x)
    p = ca.sqrt(x**2 + y**2)
    lat = ca.atan2(z, p*(1-e2))
    N = a / ca.sqrt(1 - e2*ca.sin(lat)**2)
    h = p / ca.cos(lat) - N
    return ca.vertcat(lat*180/ca.pi, lon*180/ca.pi, h)

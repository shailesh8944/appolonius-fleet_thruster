import numpy as np
import casadi as ca
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ================================
# 1. NMPC Problem Definition
# ================================

nx = 4  # px, py, psi, v
nu = 2  # a, omega

N = 20       # horizon steps
dt = 0.1     # time step

Q = np.diag([10, 10, 1, 1])
R = np.diag([0.1, 0.1])

u_min = np.array([-1.0, -0.4])
u_max = np.array([ 1.0,  0.4])

x_min = np.array([-1e6, -1e6, -1e6, 0.0])
x_max = np.array([ 1e6,  1e6,  1e6, 2.0])

# ================================
# 2. Dynamics (simple unicycle ship)
# ================================

x = ca.SX.sym('x', nx)
u = ca.SX.sym('u', nu)

px, py, psi, v = x[0], x[1], x[2], x[3]
a, omega = u[0], u[1]

px_dot  = v * ca.cos(psi)
py_dot  = v * ca.sin(psi)
psi_dot = omega
v_dot   = a

xdot = ca.vertcat(px_dot, py_dot, psi_dot, v_dot)
f = ca.Function('f', [x, u], [xdot])


# ================================
# 3. NMPC Formulation
# ================================

X = ca.SX.sym('X', nx, N+1)
U = ca.SX.sym('U', nu, N)
P = ca.SX.sym('P', 2*nx)   # [x0 , xref]

x0 = P[0:nx]
xref = P[nx:2*nx]

obj = 0
g = []

# Initial constraint
g.append(X[:, 0] - x0)

for k in range(N):
    x_k = X[:, k]
    u_k = U[:, k]

    dx = x_k - xref
    obj += ca.mtimes([dx.T, Q, dx]) + ca.mtimes([u_k.T, R, u_k])

    x_next = X[:, k] + dt * f(X[:, k], U[:, k])
    g.append(X[:, k+1] - x_next)

# Terminal cost
dx_terminal = X[:, N] - xref
obj += ca.mtimes([dx_terminal.T, Q, dx_terminal])

g = ca.vertcat(*g)

opt_vars = ca.vertcat(ca.reshape(X, -1, 1),
                      ca.reshape(U, -1, 1))

nlp = {'f': obj, 'x': opt_vars, 'g': g, 'p': P}
solver = ca.nlpsol('solver', 'ipopt', nlp,
                   {"ipopt.print_level":0, "print_time":0})

# Bounds
lbg = np.zeros(g.size()[0])
ubg = np.zeros(g.size()[0])

lbx = []
ubx = []

# X bounds
for k in range(N+1):
    lbx.extend(x_min)
    ubx.extend(x_max)

# U bounds
for k in range(N):
    lbx.extend(u_min)
    ubx.extend(u_max)

lbx = np.array(lbx)
ubx = np.array(ubx)

# ================================
# 4. NMPC Step Function
# ================================

def nmpc_step(xcurr, xref):
    P_val = np.concatenate([xcurr, xref])

    # Initial guess
    X_guess = np.tile(xcurr.reshape(-1,1), (1, N+1)).reshape(-1)
    U_guess = np.zeros(nu*N)
    w0 = np.concatenate([X_guess, U_guess])

    sol = solver(x0=w0, lbx=lbx, ubx=ubx,
                 lbg=lbg, ubg=ubg, p=P_val)

    w_opt = sol["x"].full().flatten()

    X_opt = w_opt[0:nx*(N+1)].reshape(nx, N+1)
    U_opt = w_opt[nx*(N+1):].reshape(nu, N)

    return U_opt[:,0], X_opt, U_opt


# ================================
# 5. Simulation + Visualization
# ================================

x_current = np.array([0, 0, 0, 0])  # starting state
x_ref     = np.array([10, 10, 0, 1])  # go to (10,10), v=1 m/s

trajectory = [x_current]

# Matplotlib setup
fig, ax = plt.subplots()
ax.set_xlim(-2, 12)
ax.set_ylim(-2, 12)
ax.set_aspect("equal")

ship_point, = ax.plot([], [], 'bo', markersize=6)
target_point, = ax.plot(x_ref[0], x_ref[1], 'rx', markersize=10)
path_line, = ax.plot([], [], 'b-')
pred_line, = ax.plot([], [], 'g--')


def update(frame):
    global x_current

    u0, X_pred, _ = nmpc_step(x_current, x_ref)
    x_current = x_current + dt * f(x_current, u0).full().flatten()

    trajectory.append(x_current)

    # --- FIX 1: pass sequence ---
    ship_point.set_data([x_current[0]], [x_current[1]])

    path = np.array(trajectory)
    path_line.set_data(path[:,0], path[:,1])

    # Predicted trajectory
    pred_line.set_data(X_pred[0,:], X_pred[1,:])

    return ship_point, path_line, pred_line


# --- FIX 2: Disable blit ---
ani = FuncAnimation(fig, update, frames=200, interval=100, blit=False)
plt.show()


for cloning the repo 
git clone https://github.com/MarineAutonomy/apollonius-nmpc-fleet.git
cd apollonius-nmpc-fleet




**How to Run the Multi-Vessel NMPC Simulator**

There are **two terminals** involved:

---

If you are cloning this repository for the first time

You need to build the Docker image once:


docker build -t mavlab-ros2-casadi:1.0 .


 Terminal 1 — Start the Evader (NMPC)**

This must be run **inside the Docker container** using CasADi-enabled environment.

### **1. Enter the Docker environment**

```bash
./ros2_run_devdocker_casadi.sh
```

### **2. Navigate to the evader NMPC script**

The evader controller is located at:

```
/maneuvering-simulation-python-mavi/ros2_ws/mpc_apolonius/main_mpc_evader_ros2.py
```

So inside Docker, run:

```bash
cd /workspaces/mavlab/ros2_ws/mpc_apolonius
python3 main_mpc_evader_ros2.py
```

This will output:

```
Waiting for vessel data...
```

This means the NMPC evader node is active and listening for vessel state feedback from ROS2.

--Terminal 2 — Start the Makara Visual Simulator

This can be run from another terminal (host or Docker depending on your setup).

### **1. Go to Makara folder**

```bash
cd maneuvering-simulation-python-mavi/makara
```


### **2. Start the Makara simulator**

```bash
./ros2_simulator.sh
```

Makara will start the visual simulation interface showing:

* Evader (blue) using NMPC + CasADi
* Pursuers (red) using Apollonius pursuit method

---



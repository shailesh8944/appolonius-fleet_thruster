import rclpy
from class_guidance_control import GuidanceControl
from mav_simulator.class_world import World
from rclpy.executors import MultiThreadedExecutor

def main():
    world = World('/workspaces/mavlab/inputs/simulation_input.yml')
    vessels = world.vessels
    llh0 = world.gps_datum
    
    rclpy.init()

    # ROS Multi Thread Execution
    executor = MultiThreadedExecutor()
    gcs = []
    for vessel in vessels:
        gcs.append(GuidanceControl(vessel))
        executor.add_node(gcs[-1])
    
    try: 
        executor.spin()

    finally:
        executor.shutdown()
        
        for gc in gcs:
            gc.destroy_node()
        
        rclpy.shutdown()

if __name__ == '__main__':
    main()
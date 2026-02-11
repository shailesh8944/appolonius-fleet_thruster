#!/usr/bin/env python3

import numpy as np
from class_uwb import UWB
import rclpy

def main():
    uwb = UWB(portName='/dev/ttyUSB0', topic='/makara_00/uwb_00', rate=1)
    try:
        rclpy.spin(uwb.node)
    except KeyboardInterrupt:
        pass
    finally:
        uwb.node.destroy_node()
        rclpy.shutdown()        

if __name__ == "__main__":
    main()

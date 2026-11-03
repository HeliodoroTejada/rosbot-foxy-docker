#!/usr/bin/env python3
import rclpy
from serialclient import SerialClient

def main(args=None):
    rclpy.init(args=args)
    sc = None
    try:
        sc = SerialClient()
        rclpy.spin(sc)
    except KeyboardInterrupt:
        pass
    finally:
        if sc is not None:
            sc.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

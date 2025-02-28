import rclpy

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('my_node')
    print('¡Hello, ROS 2!')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

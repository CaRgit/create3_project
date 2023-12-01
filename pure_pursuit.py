import rclpy
import sys
import move_choreograph as mc

def main(args=None):
    rclpy.init(args=args)
    move_publisher = None
    try:
        move_choreographer = mc.MoveChoreographer()
        print('Tenemos move_choreographer')
        move_publisher = mc.MoveCommandPublisher (move_choreographer)
        print('Tenemos move_publisher')
        rclpy.spin(move_publisher)
    except KeyboardInterrupt:
        print('Caught keyboard interrupt')
    except BaseException:
        print('Exception in move:', file=sys.stderr)
    finally:
        if move_publisher:
            move_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

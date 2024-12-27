import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

class OrderStatusPublisher(Node):
    def __init__(self):
        super().__init__('order_status_publisher')
        self.publisher = self.create_publisher(String, '/order_status', 10)

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.publisher.publish(msg)
        self.get_logger().info(f'Published status: {status}')

    def get_user_input(self):
        while rclpy.ok():
            print("\nEnter 'c' followed by the table number (e.g., c1, c2, c3). For all orders, enter all together (e.g., c1,c2,c3).")
            try:
                user_input = input().strip().split(',')
                valid_inputs = {'c1', 'c2', 'c3'}
                input_set = set(user_input)

                if input_set == valid_inputs:
                    # All orders canceled
                    self.publish_status("All orders canceled")
                else:
                    # Handle single valid inputs
                    for entry in user_input:
                        if entry in valid_inputs:
                            table_number = entry[1]
                            status_message = f"Y{table_number} canceled"
                            self.publish_status(status_message)
                        else:
                            print(f"Invalid entry: {entry}. Please use the format c1, c2, or c3.")
            except Exception as e:
                print(f"Error processing input. Please use the correct format. Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = OrderStatusPublisher()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()
    
    try:
        node.get_user_input()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()

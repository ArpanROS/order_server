import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.callback_groups import ReentrantCallbackGroup
import threading
from std_msgs.msg import String


class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.kitchen_location = {
            'x': -4.0,
            'y': 1.0,
            'z': 0.0,
            'qx': 0.0,
            'qy': 0.0,
            'qz': 0.0,
            'qw': 1.0
        }

        self.table_locations = {
            'table1': {
                'x': 2.0,
                'y': -3.0,
                'z': 0.0,
                'qx': 0.0,
                'qy': 0.0,
                'qz': 0.0,
                'qw': 1.0
            },
            'table2': {
                'x': 3.0,
                'y': -3.0,
                'z': 0.0,
                'qx': 0.0,
                'qy': 0.0,
                'qz': 0.0,
                'qw': 1.0
            },
            'table3': {
                'x': 4.0,
                'y': -3.0,
                'z': 0.0,
                'qx': 0.0,
                'qy': 0.0,
                'qz': 0.0,
                'qw': 1.0
            }
        }

        self.home_location = {
            'x': 0.0,
            'y': 0.0,
            'z': 0.0,
            'qx': 0.0,
            'qy': 0.0,
            'qz': 0.0,
            'qw': 1.0
        }

        self.order_status_sub = self.create_subscription(
            String,
            '/order_status',
            self.order_status_callback,
            10
        )

        self.current_goal = None
        self.order_sequence = []
        self.current_table_index = 0
        self.goal_handle = None
        self.navigation_result = None
        self._navigation_complete = threading.Event()
        self.max_retries = 3

        self.order_canceled = False
        self.canceled_tables = set()
        self.current_destination = None

    def order_status_callback(self, msg):
        if msg.data == "All orders canceled":
            self.order_canceled = True
            if self.current_destination == "Kitchen":
                self.get_logger().info('All orders canceled, returning to home...')
            else:
                self.get_logger().info('All orders canceled, returning to kitchen first...')
            self._navigation_complete.set()
        elif "canceled" in msg.data:
            table_num = int(msg.data[1])
            self.canceled_tables.add(table_num)
            self.get_logger().info(f'Order for table {table_num} canceled')


    def prompt_user(self):
        while True:
            print("\nPlace your order! Please press 'y' or 'n' along with your table number (e.g; y1,y2,n1,n2)!")
            try:
                user_input = input().strip().split(',')
                if not all(entry[0] in ['y', 'n'] and entry[1] in ['1', '2', '3'] for entry in user_input):
                    print("Invalid input format. Please use format like: y1,y2,n3")
                    continue

                self.order_sequence = [int(entry[1]) for entry in user_input if entry[0] == 'y']
                
                if self.order_sequence:
                    self.current_table_index = 0
                    self.current_goal = "Kitchen"
                    return True
                else:
                    print("No orders to process. Robot will not move.")
                    continue
            except Exception as e:
                print(f"Invalid input. Please use correct format. Error: {e}")

    def send_goal_with_retry(self, location, location_name):
        retries = 0
        while retries < self.max_retries:
            if retries > 0:
                self.get_logger().info(f'Retry attempt {retries + 1} for {location_name}...')
                time.sleep(2)  # Wait before retrying
            
            success = self.send_single_goal(location, location_name)
            if success:
                return True
            
            retries += 1
            if retries < self.max_retries:
                self.get_logger().info(f'Navigation failed, will retry {location_name}...')
            else:
                self.get_logger().info(f'Navigation failed after {self.max_retries} attempts for {location_name}')
        
        return False

    def send_single_goal(self, location, location_name):
        self.order_canceled = False
        self.current_destination = location_name
        self._navigation_complete.clear()
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = location['x']
        goal_msg.pose.pose.position.y = location['y']
        goal_msg.pose.pose.position.z = location['z']
        goal_msg.pose.pose.orientation.x = location['qx']
        goal_msg.pose.pose.orientation.y = location['qy']
        goal_msg.pose.pose.orientation.z = location['qz']
        goal_msg.pose.pose.orientation.w = location['qw']

        self.action_client.wait_for_server()
        self.get_logger().info(f'Sending goal to {location_name}...')
        
        send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        
        self._navigation_complete.wait()
        return False if self.order_canceled else self.navigation_result


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected!')
            self.navigation_result = False
            self._navigation_complete.set()
            return

        self.get_logger().info('Goal accepted!')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        if status == 4:
            self.get_logger().info('Navigation succeeded!')
            self.navigation_result = True
        else:
            self.get_logger().info('Navigation failed!')
            self.navigation_result = False
        self._navigation_complete.set()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

    def process_orders(self):
        while rclpy.ok():
            if self.prompt_user():
                success = self.send_goal_with_retry(self.kitchen_location, "Kitchen")
                if not success:
                    if self.order_canceled:
                        self.send_goal_with_retry(self.home_location, "Home")
                    continue

                kitchen_response = self.handle_kitchen_prompt()
                if not kitchen_response:
                    self.send_goal_with_retry(self.home_location, "Home")
                    continue

                all_responses_yes = True
                while self.current_table_index < len(self.order_sequence):
                    table_num = self.order_sequence[self.current_table_index]
                    
                    if table_num in self.canceled_tables:
                        self.get_logger().info(f'Skipping canceled table {table_num}')
                        all_responses_yes = False
                        self.current_table_index += 1
                        continue

                    success = self.send_goal_with_retry(
                        self.table_locations[f'table{table_num}'],
                        f"Table{table_num}"
                    )

                    if not success:
                        if self.order_canceled:
                            self.send_goal_with_retry(self.kitchen_location, "Kitchen")
                            self.send_goal_with_retry(self.home_location, "Home")
                            break
                        all_responses_yes = False
                        self.current_table_index += 1
                        continue

                    table_response = self.handle_table_prompt()
                    if not table_response:
                        all_responses_yes = False
                    self.current_table_index += 1

                    if self.order_canceled:
                        self.send_goal_with_retry(self.kitchen_location, "Kitchen")
                        self.send_goal_with_retry(self.home_location, "Home")
                        break

                if not self.order_canceled:
                    if all_responses_yes:
                        self.send_goal_with_retry(self.home_location, "Home")
                    else:
                        self.send_goal_with_retry(self.kitchen_location, "Kitchen")
                        self.send_goal_with_retry(self.home_location, "Home")

                self.order_canceled = False
                self.canceled_tables.clear()

    def handle_kitchen_prompt(self):
        print("\nI am here to receive the order! Order is ready? Press 'y' or 'n'!")
        user_input = None
        try:
            import select
            import sys
            i, o, e = select.select([sys.stdin], [], [], 10)
            if i:
                user_input = sys.stdin.readline().strip().lower()
        except:
            pass

        if user_input == 'y':
            print("Order confirmed. Proceeding to tables...")
            return True
        elif user_input == 'n':
            print("Order not ready. Returning to Home...")
            return False
        print("No response received. Returning to Home...")
        return False

    def handle_table_prompt(self):
        print("\nWant to receive the order! Press 'y' or 'n'!")
        user_input = None
        try:
            import select
            import sys
            i, o, e = select.select([sys.stdin], [], [], 10)
            if i:
                user_input = sys.stdin.readline().strip().lower()
        except:
            pass

        if user_input == 'y':
            print("Order received. Processing next destination...")
            time.sleep(3)
            return True
        elif user_input == 'n':
            print("Order not received. Skipping to next table...")
            return False
        print("No response received. Skipping to next table...")
        return False

def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()
    
    try:
        node.process_orders()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        spin_thread.join()


if __name__ == '__main__':
    main()

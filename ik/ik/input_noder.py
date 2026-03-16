#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from jetson_stm32_poc_msgs.msg import DataVector
import threading

class InputNode(Node):
    def __init__(self):
        super().__init__('input_node')
        # Create publisher on topic '/input_number' with message type Int32
        self.publisher_ = self.create_publisher(DataVector, 'sender_data', 10)
        
        # Store current values to publish repeatedly (default to 0)
        self.current_values = DataVector()
        self.current_values.position_x = 0.0
        self.current_values.position_y = 0.0
        self.current_values.twist_z = 0.0
        self.default = self.current_values
        self.new_command = False
        
        # Create timer to publish repeatedly at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_current_values)
        
        self.get_logger().info('Input Node started. Type numbers to publish them.')
    
    def publish_current_values(self):
        """Publish the current values repeatedly"""
        if not self.new_command:
            return
        msg = DataVector()
        msg.position_x = self.current_values.position_x
        msg.position_y = self.current_values.position_y
        msg.twist_z = self.current_values.twist_z
        
        self.publisher_.publish(msg)

        self.new_command = False
    
    def update_values(self, num):
        """Update the values to be published"""
        self.current_values.position_x = num.position_x
        self.current_values.position_y = num.position_y
        self.current_values.twist_z = num.twist_z
        self.new_command = True
        self.get_logger().info(f'Updated values to: {num.position_x}, {num.position_y}, {num.twist_z}')
    
    def input_loop(self,node):
        """Function that runs in a separate thread to get user input"""
        while rclpy.ok():
            try:
                # Get user input (this blocks until user enters something)
                user_input = input("Enter as val1 val2 val3 (or 'q' to quit): ")
                # Check if user wants to quit
                if user_input.lower() == 'q':
                    self.get_logger().info('Quitting...')
                    rclpy.shutdown()
                    break
                # Convert to integer and publish
                lister = user_input.split()
                if len(lister) != 3:
                    print("Please enter exactly three values.")
                    continue
                num = DataVector()
                num.position_x = float(lister[0])
                num.position_y = float(lister[1])
                num.twist_z = float(lister[2])
                node.update_values(num)

            except ValueError:
                print("Invalid input! Please enter a valid integer.")

            except KeyboardInterrupt:
                print("\nShutting down...")
                rclpy.shutdown()
                break
            except Exception as e:
                print(f"Error: {e}")
def main(args=None):
    rclpy.init(args=args)
    # Create the input node
    node = InputNode()

    # Create a thread for the input loop
    # daemon=True means the thread will automatically terminate when main program exits
    input_thread = threading.Thread(target=node.input_loop, args=(node,), daemon=True)
    input_thread.start()

    try:
        # Spin the node in the main thread
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
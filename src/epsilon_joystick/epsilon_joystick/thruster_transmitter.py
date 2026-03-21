import rclpy, time
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class ThrusterTransmitter(Node):
    def __init__(self):
        super().__init__('ThrusterTransmitter')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'thrust_control', 10)
        self.readInstructionsFromFile()
    
    def readInstructionsFromFile(self, fileName='test.log'):
        data_file = Path(get_package_share_directory('epsilon_joystick')) / 'logs' / fileName
        with data_file.open('r') as file:
            for line in file:
                if line == "EOF": break
                data = line.split(' ')
                self.timer_ = data.pop()
                
                msg = Float64MultiArray()
                msg.data = [float(str) for str in data]
                self.publisher_.publish(msg)
                
                time.sleep(int(self.timer_))
        print("EOF")
        
def main(args=None):
    rclpy.init(args=args)
    node = ThrusterTransmitter()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
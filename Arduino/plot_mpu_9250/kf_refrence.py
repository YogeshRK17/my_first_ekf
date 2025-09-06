import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from scipy.spatial.transform import Rotation as R
from rclpy.time import Time
from rclpy.clock import Clock
from rclpy.duration import Duration
import tf_transformations

class amclKf(Node):
    def __init__(self):
        Node.__init__(self, "amcl_kf")

        self.stateDim   = 9
        self.measureDim = 3
        self.dt         = 0.005

        self.firstITR      = True
        self.currentTime   = 0
        self.previousTime  = 0
        self.currentTime_e = 0 #Estimated time
        self.start_publishing = False

        self.state   = np.zeros(self.stateDim).reshape(-1,1)
        self.state_e = np.zeros(self.stateDim).reshape(-1,1)  #Estimated state
        self.Qk      = 0.001*np.eye(self.stateDim)
        self.Pk      = 100*np.eye(self.stateDim)
        self.Rk      = 0.5*np.eye(self.measureDim)
        self.Hk      = np.zeros((self.measureDim, self.stateDim))
        self.Hk[0,0] = 1
        self.Hk[1,1] = 1
        self.Hk[2,2] = 1

        self.Ak = np.array([[1, 0, 0, self.dt, 0, 0, self.dt**2/2, 0, 0],
                            [0, 1, 0, 0, self.dt, 0, 0, self.dt**2/2, 0],
                            [0, 0, 1, 0, 0, self.dt, 0, 0, self.dt**2/2],
                            [0, 0, 0, 1, 0, 0, self.dt, 0, 0],
                            [0, 0, 0, 0, 1, 0, 0, self.dt, 0],
                            [0, 0, 0, 0, 0, 1, 0, 0, self.dt],
                            [0, 0, 0, 0, 0, 0, 1, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 1, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 1]]
                            )     
        
        self.amcl_subscriber = self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.amcl_callback, 10)
        self.propagate_timer = self.create_timer(0.1, self.propagate_timer_callback)
        self.amcl_kf_publisher = self.create_publisher(PoseWithCovarianceStamped, "/amcl_kf_pose", 10)

        self.amcl_data  = {'timestamp': 0.0, 'x':0.0, 'y': 0.0, 'theta':0.0}
        self.get_logger().info("Started AMCL KF Node")

    def propagate_timer_callback(self):
        self.get_logger().info("Inside timer")
        if self.start_publishing:
            self.propagate()
            self.publish_amcl_kf()
            self.get_logger().info("published amcl kf")
        else:
            self.get_logger().info("timer_not started")

    def publish_amcl_kf(self):
        print("Publishing")
        data = PoseWithCovarianceStamped()
        data.header.stamp.sec = abs(int(self.currentTime_e))
        data.header.stamp.nanosec = abs(int((self.currentTime_e - int(self.currentTime_e)) * 1e9))
        data.pose.pose.position.x = float(self.state_e[0])
        data.pose.pose.position.y = float(self.state_e[1])
        x,y,z,w = self.get_quaternion_from_euler(0, 0, self.state_e[2])
        data.pose.pose.orientation.x = float(x)
        data.pose.pose.orientation.y = float(y)
        data.pose.pose.orientation.z = float(z)
        data.pose.pose.orientation.w = float(w)
        data.pose.covariance[0]  = float(self.Pk[0,0])
        data.pose.covariance[7]  = float(self.Pk[1,1])
        data.pose.covariance[35] = float(self.Pk[2,2])
        self.amcl_kf_publisher.publish(data)

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.
        
        Input
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.
        
        Output
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return [qx, qy, qz, qw]

    def amcl_filter(self, amcl_measure):
        if self.firstITR:
            self.firstITR = False
            self.currentTime = amcl_measure["timestamp"]
        else:
            self.previousTime = self.currentTime
            self.currentTime = amcl_measure["timestamp"]
            self.DT = self.currentTime - self.previousTime

            for i in range(int(round(self.DT/self.dt))):
                self.Prediction()

            self.update(amcl_data=amcl_measure)
            
    def Prediction(self):
        self.state = np.matmul(self.Ak, self.state.reshape(-1,1))
        self.Pk = np.matmul(self.Ak, np.matmul(self.Pk, self.Ak.T)) + self.Qk

    def update(self, amcl_data):
        Measure = np.array([amcl_data['x'], amcl_data['y'], amcl_data['theta']]).reshape(-1,1)

        Kk = np.matmul(self.Pk, np.matmul(self.Hk.T, np.linalg.inv(np.matmul(self.Hk, np.matmul(self.Pk, self.Hk.T)) + self.Rk)))
        self.state = self.state + np.matmul(Kk, (Measure - np.matmul(self.Hk, self.state)))
        IKkHk = (np.eye(Kk.shape[0]) - np.matmul(Kk, self.Hk))
        self.Pk = np.matmul(IKkHk, np.matmul(self.Pk, IKkHk.T)) + np.matmul(Kk, np.matmul(self.Rk, Kk.T))

        self.state_e = self.state
        self.currentTime_e = self.currentTime

    def propagate(self):
        system_time = self.get_clock().now()

        Taue = system_time.nanoseconds*1e-9 - self.currentTime
        print(self.currentTime)
        print(Taue)
        # Taue = 0.2
        for i in range(int(round(Taue/self.dt))):
            self.state_e = np.matmul(self.Ak, self.state.reshape(-1,1)) 
            
        self.currentTime_e = self.currentTime + Taue
          
    def quaternionToEuler(self,quaternion):
        r = R.from_quat(quaternion) #quaternion = [qx,qy,qz,qw]
        euler = r.as_euler('xyz', degrees=False)
        return euler 

    def amcl_callback(self, msg):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y
        theta = self.quaternionToEuler([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]

        self.amcl_data["timestamp"] = timestamp
        self.amcl_data['x']         = pos_x
        self.amcl_data['y']         = pos_y
        self.amcl_data['theta']     = theta

        self.get_logger().info("got amcl measurements")

        self.amcl_filter(self.amcl_data)
        self.propagate()
        self.start_publishing = True
        self.get_logger().info("filtering done")

def main():
    rclpy.init()
    node = amclKf()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import time
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.time import Time
from rcl_interfaces.msg import SetParametersResult

class sharedResources():
    def __init__(self):

        #STATE=[xt, yt, psi_t, vxt, vyt, omega, ax, ay,  wx_dot]
                # 0   1     2    3    4     5   6    7     8
    
        self.sometime = time.time()

        self.stateDim     = 9
        self.laserMeasDim = 3
        self.imu_Dim      = 3
        self.wheelOdomDim = 3

        self.Qk      = 0.0001*np.eye(self.stateDim)
        self.Qk[3,3] = 0.0005
        self.Qk[4,4] = 0.0005
        self.Qk[5,5] = 0.0005
        self.Qk[6,6] = 0.005
        self.Qk[7,7] = 0.005

        self.State = np.zeros(self.stateDim)
        
        #assigning initial state of base_odom April TF
        # self.State[0] = 16.195092670906504
        # self.State[1] = -8.405607795262416
        # self.State[2] = -0.08841896

        #assigning initial state of wheel_odom April TF
        self.State[0] = 13.0946469607169
        self.State[1] = -7.934286141887813
        self.State[2] = -0.14247192

        #assigning initial state of wheel_odom robot_static2 TF
        # self.State[0] = 5.095999781950929
        # self.State[1] = -9.138515185098697
        # self.State[2] = -0.87951382

        self.Pk       = 100.0*np.eye(self.stateDim)
        self.Rk1      = 0.08*np.eye(self.laserMeasDim)
        self.Rk2      = 0.5*np.eye(self.imu_Dim)
        self.Rk2[0,0] = 0.01
        self.Rk3      = 0.01*np.eye(self.wheelOdomDim)
        self.IMU_x    = 1.38     

        self.FIRST_ITTER    = True # flag for first iteration
        self.VEL_LASER      = True # flag for velocity update for laser
        self.VEL_WHEELO_DOM = True # flag for velocity update for wheel odometry

        #buffer
        self.flags = []
        self.time_flags = []

        # #store sensor data from callback
        # self.laser_data = {'timestamp': 0.0, 'pos': 0.0, 'lin_vel': 0.0, 'ori': 0.0, 'omega_z': 0.0, 'cv': 0.0}
        # self.wheel_data = {'timestamp': 0.0, 'pos': 0.0, 'lin_vel': 0.0, 'ori': 0.0, 'omega_z': 0.0, 'cv': 0.0}
        # self.imu_data   = {'timestamp': 0.0, 'omega_z': 0.0, 'lin_accl': 0.0, 'cv': 0.0}

        # timestamp, pos, lin_vel, ori, omega_z
        # 0          1    2        3    4        
        self.laser_data = []
        # timestamp, pos, lin_vel, ori, omega_z 
        # 0          1    2        3    4       
        self.wheel_data = []
        # timestamp, omega_z, lin_accl, na, na
        # 0          1        2         3   4
        self.imu_data   = []

        #measurement parameters for plot
        self.wheel_odom       = []
        self.imu_data_measure = []
        self.laser_odom       = []
        self.state_pub        = []
        self.amcl_pose        = []

        self.laser_v_scale = 1/1.2 #1/1.2 #for bag file #for sim 2.8
        self.laser_w_scale = 1/1.6 #1/1.6 #for bag file #for sim 2.8

    def clear_resources(self):
        #clear buffer
        self.flags = []
        self.time_flags = []

        #clear sensor data
        # timestamp, pos, lin_vel, ori, omega_z
        # 0          1    2        3    4        
        self.laser_data = []
        # timestamp, pos, lin_vel, ori, omega_z 
        # 0          1    2        3    4       
        self.wheel_data = []
        # timestamp, omega_z, lin_accl, na, na
        # 0          1        2         3   4
        self.imu_data   = []


class EKF(sharedResources):
    def __init__(self):
        super().__init__()

    ##### Supporting Functions Start #####

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


    def alignYawMeas(self,State,yaw):
        # print ((State[2,0] - yaw),((State[2,0] - yaw)/abs(State[2,0] - yaw)))
        if abs(State[2,0] - yaw) > np.pi*1.9:
            return yaw + ((State[2,0] - yaw)/abs(State[2,0] - yaw))*2*np.pi
        
    def saturateYaw(self):
        if self.State[2]>np.pi*2:
            self.State[2]-=np.pi*2
        if self.State[2]<np.pi*2:
            self.State[2]+=np.pi*2
    def SetState(self,state):
        self.State=state

    def quaternionToEuler(self,quaternion):
        r = R.from_quat(quaternion) #quaternion = [qx,qy,qz,qw]
        euler = r.as_euler('xyz', degrees=False)
        return euler 

    def StateTrnsModel(self,state):
        state=state.ravel()
        #STATE=[xt, yt, psi_t, vxt, vyt, onodemega, ax, ay,  wx_dot]
                # 0   1     2    3    4     5   6    7     8

        return np.array([[state[0] + self.dt*(np.cos(state[2])*state[3]-np.sin(state[2])*state[4])],
                        [state[1] + self.dt*(np.sin(state[2])*state[3]+np.cos(state[2])*state[4])],
                        [state[2] + self.dt* (state[5])],
                        [state[3] + self.dt* (state[6])],
                        [state[4] + self.dt* (state[7])],
                        [state[5] + self.dt* (state[8])],
                        [state[6]],
                        [state[7]],
                        [state[8]]]).reshape(-1,1)

    def Jacobian(self,state):
        state = state.ravel()
        jacobian = np.eye(self.stateDim)
        jacobian[0,2] = -np.sin(state[2])*state[3]*self.dt - np.cos(state[2])*state[4]*self.dt
        jacobian[0,3] = np.cos(state[2])*self.dt
        jacobian[0,4] = - np.sin(state[2])*self.dt

        jacobian[1,2] = np.cos(state[2])*state[3]*self.dt - np.sin(state[2])*state[4]*self.dt
        jacobian[1,3] = np.sin(state[2])*self.dt
        jacobian[1,4] = np.cos(state[2])*self.dt

        jacobian[2,5] = self.dt

        jacobian[3,6] = self.dt

        jacobian[4,7] = self.dt
        jacobian[5,8] = self.dt

        return jacobian
    
    def wrapYaw(self,state):
        if state[2,0]>np.pi:
            state[2,0]= state[2,0]-2*np.pi
        if state[2,0]<-np.pi:
            state[2,0]= state[2,0]+2*np.pi
        return state

    ##### Supporting Functions End #####

    ##### Processing Functions Start #####

    def process_laser_data(self, laser_data):
        if self.FIRST_ITTER:
            self.FIRST_ITTER = False
            self.LastTime = laser_data[0]
            self.dt = 0.02# based on sensors loop rate
        else:
            self.dt = laser_data[0]-self.LastTime
            print(f"current time: {laser_data[0]}")
            print(f"last time: {self.LastTime}")
            print(f"dt laser: {self.dt}")
            self.LastTime = laser_data[0]

        self.Hk1 = np.zeros((self.laserMeasDim, self.stateDim))
        self.State = self.wrapYaw(self.StateTrnsModel(self.State))

        if self.VEL_LASER:
            self.Hk1[0,3]=1
            self.Hk1[1,4]=1
            self.Hk1[2,5]=1
            # self.Hk1[0,5]=0.316 #y laser distance
            # self.Hk1[1,5]=-1.207 #x laser distance
            velocity = laser_data[2] 
            angularVel = laser_data[4] 
            # updated_vx = velocity[0] - (angularVel*0.316)
            # updated_vy = velocity[1] + (angularVel*1.207)
            measur = np.append(velocity,[angularVel]).reshape(-1,1)
        else:
            self.Hk1[0,0]=1
            self.Hk1[1,1]=1
            self.Hk1[2,2]=1
            # self.Hk1[0,5]=0.316 #y laser distance
            # self.Hk1[1,5]=-1.207 #x laser distance
            pos = laser_data[1] 
            ori = self.quaternionToEuler(laser_data[3])[2]# using only yaw after conversion to euler,
        
            measur = np.append(pos,[ori]).reshape(-1,1)#self.laserMeasDim = 2+1
            # measur[2] = self.alignYawMeas(self.State,measur[2])
        Fkx = self.Jacobian(self.State)
        
        Lkx = np.eye(self.stateDim) #df/dw \in R^(stateDim x noiseInStateDim): y \in R^(outputDim), w \in R^(noiseInStateDim)
        self.Pk = np.matmul(Fkx, np.matmul(self.Pk, Fkx.T)) + np.matmul(Lkx,np.matmul(self.Qk,Lkx.T))
    
        #dy/dx \in R^(output x stateDim): y \in R^(outputDim), v \in R^(stateDim)
        
        Mk1 = np.eye(self.Rk1.shape[0]) #dy/dv: y \in R^(outputDim), v \in R^(noiseInOutputDim)
        Kk = np.matmul(self.Pk, np.matmul(self.Hk1.T, np.linalg.inv(np.matmul(self.Hk1, np.matmul(self.Pk, self.Hk1.T)) + np.matmul(Mk1,np.matmul(self.Rk1,Mk1.T)))))
        self.State = self.State + np.matmul(Kk, (measur - np.matmul(self.Hk1, self.State)))
        # print "StatePo",StatePo.ravel()
        ImKkHk = (np.eye(Kk.shape[0]) - np.matmul(Kk, self.Hk1))
        MkRkMkT = np.matmul(Mk1,np.matmul(self.Rk1,Mk1.T))
        self.Pk = np.matmul(ImKkHk, np.matmul(self.Pk, ImKkHk.T)) + np.matmul(Kk,np.matmul(MkRkMkT,Kk.T))
        self.state_pub.append(np.append(laser_data[0], self.State[0:10].ravel()))

    def process_wheel_data(self, wheel_data):
        if self.FIRST_ITTER:
            self.FIRST_ITTER = False
            self.LastTime = wheel_data[0]
            self.dt = 0.02# based on sensors loop rate
        else:
            self.dt = wheel_data[0] - self.LastTime
            print(f"current time: {wheel_data[0]}")
            print(f"last time: {self.LastTime}")
            print(f"dt wheel: {self.dt}")
            self.LastTime = wheel_data[0]

        if self.dt < 0.0001:
            print("discarding wheel measurement")
            # print("negative dt")
            # self.dt = 0.02
            return 0
        
        print("valid wheel measurement")
        #dy/dx \in R^(output x stateDim): y \in R^(outputDim), v \in R^(stateDim)
        self.Hk3 = np.zeros((self.wheelOdomDim, self.stateDim))
        self.State = self.wrapYaw(self.StateTrnsModel(self.State))

        if self.VEL_WHEELO_DOM:
            velocity = wheel_data[2]
            angularVel = wheel_data[4]
            measur = np.append(velocity,[angularVel]).reshape(-1,1)
            self.Hk3[0,3] = 1
            self.Hk3[1,4] = 1
            self.Hk3[2,5] = 1
        else:
            pos = wheel_data[1]
            ori = self.quaternionToEuler(wheel_data[3])[2]# using only yaw after conversion to euler,
            measur = np.append(pos,[ori]).reshape(-1,1)#self.wheelOdomDim=2+1
            self.Hk3[0,0] = 1
            self.Hk3[1,1] = 1
            self.Hk3[2,2] = 1
            # measur[2] = self.alignYawMeas(self.State,measur[2])
        
        Fkx = self.Jacobian(self.State)
        
        Lkx = np.eye(self.stateDim) #df/dw \in R^(stateDim x noiseInStateDim): y \in R^(outputDim), w \in R^(noiseInStateDim)
        self.Pk = np.matmul(Fkx, np.matmul(self.Pk, Fkx.T)) + np.matmul(Lkx,np.matmul(self.Qk,Lkx.T))
    
        Mkx = np.eye(self.Rk3.shape[0]) #dy/dv: y \in R^(outputDim), v \in R^(outputDim)
        Kk = np.matmul(self.Pk, np.matmul(self.Hk3.T, np.linalg.inv(np.matmul(self.Hk3, np.matmul(self.Pk, self.Hk3.T)) + np.matmul(Mkx,np.matmul(self.Rk3,Mkx.T)))))
        self.State = self.State + np.matmul(Kk, (measur - np.matmul(self.Hk3, self.State)))
        # print "StatePo",StatePo.ravel()
        ImKkHk = (np.eye(Kk.shape[0]) - np.matmul(Kk, self.Hk3))
        MkRkMkT = np.matmul(Mkx,np.matmul(self.Rk3,Mkx.T))
        self.Pk = np.matmul(ImKkHk, np.matmul(self.Pk, ImKkHk.T)) + np.matmul(Kk,np.matmul(MkRkMkT,Kk.T))
        self.state_pub.append(np.append(wheel_data[0], self.State[0:10].ravel()))

    def process_imu_data(self, imu_data):
        if self.FIRST_ITTER:
            self.FIRST_ITTER = False
            self.LastTime = imu_data[0]
            self.dt = 0.02# based on sensors loop rate
        else:
            self.dt = imu_data[0]-self.LastTime
            print(f"current time: {imu_data[0]}")
            print(f"last time: {self.LastTime}")
            print(f"dt imu: {self.dt}")
            self.LastTime = imu_data[0]

        if self.dt < 0.0001:
            print("discarding imu measurement")
            # print("negative dt")
            # self.dt = 0.02
            return 0

        print("valid imu measurement")
        angularVel = imu_data[1]
        linear_acc = imu_data[2]
        # measur = np.append((pos,np.append(ori,np.append(velocity,angularVel))))
        measur = np.append(-1*angularVel,linear_acc*np.array([-1,1])).reshape(-1,1)#self.imu_Dim = 1+2
        # measur = np.append(1*angularVel,linear_acc*np.array([1,1])).reshape(-1,1)#self.imu_Dim = 1+2
        self.State = self.wrapYaw(self.StateTrnsModel(self.State))
        Fkx = self.Jacobian(self.State)
        
        Lkx = np.eye(self.stateDim) #df/dw \in R^(stateDim x noiseInStateDim): y \in R^(outputDim), w \in R^(noiseInStateDim)
        self.Pk = np.matmul(Fkx, np.matmul(self.Pk, Fkx.T)) + np.matmul(Lkx,np.matmul(self.Qk,Lkx.T))
    
        #dy/dx \in R^(output x stateDim): y \in R^(outputDim), v \in R^(stateDim)
        self.Hk2 = np.zeros((self.imu_Dim, self.stateDim))
        self.Hk2[0,5] = 1
        self.Hk2[1,6] = 1
        self.Hk2[2,7] = 1
        self.Hk2[1,5] = -2*self.IMU_x*self.State[5,0]
        self.Hk2[2,8] = self.IMU_x
        
        Mkx = np.eye(self.Rk2.shape[0]) #dy/dv: y \in R^(outputDim), v \in R^(noiseInOutputDim)
        Kk = np.matmul(self.Pk, np.matmul(self.Hk2.T, np.linalg.inv(np.matmul(self.Hk2, np.matmul(self.Pk, self.Hk2.T)) + np.matmul(Mkx,np.matmul(self.Rk2,Mkx.T)))))
        self.State = self.State + np.matmul(Kk, (measur - np.matmul(self.Hk2, self.State)))
        ImKkHk = (np.eye(Kk.shape[0]) - np.matmul(Kk, self.Hk2))
        MkRkMkT = np.matmul(Mkx,np.matmul(self.Rk2,Mkx.T))
        self.Pk = np.matmul(ImKkHk, np.matmul(self.Pk, ImKkHk.T)) + np.matmul(Kk,np.matmul(MkRkMkT,Kk.T))
        self.state_pub.append(np.append(imu_data[0], self.State[0:10].ravel()))

    ##### Processing Functions End #####

class ROS(Node, EKF, sharedResources):
    def __init__(self):
        Node.__init__(self, "palpicker_ekf_one_list")
        EKF.__init__(self)
        sharedResources.__init__(self)

        self.wheel_sub = self.create_subscription(Odometry, '/mobile_base_controller/odom', self.wheel_callback, 10)
        self.laser_sub = self.create_subscription(Odometry, '/laser_scan/odom', self.laser_callback, 10)
        self.imu_sub   = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.amcl_sub  = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.read_only_amcl_callback, 10)

        self.base_odom_publisher = self.create_publisher(Odometry, '/base_odom', 10)

        self.time_period = 0.009259 #1/110HZ
        self.timer_ = self.create_timer(self.time_period, self.timer_callback)

        # transform broadcaster 
        self.br = TransformBroadcaster(self)
        self.timer_br = self.create_timer(0.1, self.broadcast_transform)

        #ros parameters
        self.declare_parameter('reset_buffer', False)

        self.reset_parameter = self.get_parameter('reset_buffer').get_parameter_value().bool_value

        self.add_on_set_parameters_callback(self.reset_callback)

    def reset_callback(self, params):
        for param in params:
            if param.name == 'reset_buffer' and param.type_ == param.Type.BOOL:
                self.reset_parameter = param.value
                self.get_logger().warn("Reset triggerd, resetting buffer...")
                try:
                    self.clear_resources()
                except Exception as e:
                    self.get_logger().warn(f"Exception occured while clearning resource: {e}")
                    return SetParametersResult(successful=False)
                finally:      
                    self.get_logger().info("Reset Done!")
                    return SetParametersResult(successful=True)

    def broadcast_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = float(self.State[0])
        t.transform.translation.y = float(self.State[1])
        t.transform.translation.z = 0.0
        x,y,z,w = self.get_quaternion_from_euler(0, 0, self.State[2])
        t.transform.rotation.x = float(x)
        t.transform.rotation.y = float(y)
        t.transform.rotation.z = float(z)
        t.transform.rotation.w = float(w)
        self.br.sendTransform(t)

    def timer_callback(self):
        print("################################################")
        print(f"length of buffer: {len(self.flags)}")
        print("################################################")
        if len(self.flags) > 0:

            print("flags: ")
            print(self.flags)

            print("times: ")
            print(self.time_flags)

            paired = list(zip(self.flags, self.time_flags))
            sorted_pair = sorted(paired, key=lambda x:x[1])

            print(sorted_pair)

            id = sorted_pair[0][0]
            time_ = sorted_pair[0][1]

            self.flags.remove(id)
            self.time_flags.remove(time_)

            if id == 0:
                self.sensor_data = self.laser_data[0]
                self.process_laser_data(self.sensor_data)
                self.publish_message(self.State, self.Pk)
                self.laser_data.pop(0)

            elif id == 1:
                self.sensor_data = self.wheel_data[0]
                self.process_wheel_data(self.sensor_data)
                self.publish_message(self.State, self.Pk)
                self.wheel_data.pop(0)

            elif id == 2:
                self.sensor_data = self.imu_data[0]
                self.process_imu_data(self.sensor_data)
                self.publish_message(self.State, self.Pk)
                self.imu_data.pop(0)

            else:
                pass

    def publish_message(self, state, Pk):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_footprint"
        msg.pose.pose.position.x = float(state[0])
        msg.pose.pose.position.y = float(state[1])
        msg.pose.pose.position.z = 0.0
        x,y,z,w = self.get_quaternion_from_euler(0, 0, state[2])
        msg.pose.pose.orientation.x = float(x)
        msg.pose.pose.orientation.y = float(y)
        msg.pose.pose.orientation.z = float(z)
        msg.pose.pose.orientation.w = float(w)
        msg.twist.twist.linear.x = float(state[3])
        msg.twist.twist.linear.y = float(state[4])
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = float(state[5])
        msg.pose.covariance[0] = float(self.Pk[0,0])
        msg.pose.covariance[7] = float(self.Pk[1,1])
        msg.pose.covariance[35] = float(self.Pk[2,2])
        msg.twist.covariance[0] = float(self.Pk[3,3])
        msg.twist.covariance[7] = float(self.Pk[4,4])
        msg.twist.covariance[35] = float(self.Pk[5,5])
        self.base_odom_publisher.publish(msg)
        
    def laser_callback(self, msg): #laser id = 0

        #fetch data from ros message
        self.laser_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.laser_x = msg.pose.pose.position.x
        self.laser_y = msg.pose.pose.position.y
        self.laser_pos = np.array([self.laser_x, self.laser_y])
        self.laser_vx = msg.twist.twist.linear.x * self.laser_v_scale  #scaling up laser, refer odom plot wheel vs laser
        self.laser_vy = msg.twist.twist.linear.y  
        self.laser_linear_vel = np.array([self.laser_vx, self.laser_vy])
        self.ori_laser = np.vstack((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)).T
        self.laser_omega_z = msg.twist.twist.angular.z * self.laser_w_scale #scaling up laser, refer odom plot wheel vs laser
        
        #update sensor data in 'laser_data'
        # self.laser_data['timestamp'] = self.laser_timestamp                                                             
        # self.laser_data[0] = self.get_clock().now().nanoseconds * 1e-9
        # self.laser_data[1] = self.laser_pos
        # self.laser_data[2] = self.laser_linear_vel
        # self.laser_data[3] = self.ori_laser
        # self.laser_data[4] = self.laser_omega_z

        laser_system_time = self.get_clock().now().nanoseconds * 1e-9

        self.flags.append(0)
        self.time_flags.append(laser_system_time)
        self.laser_data.append([laser_system_time, self.laser_pos, self.laser_linear_vel, self.ori_laser, self.laser_omega_z])

        self.laser_odom.append(np.append(laser_system_time, [self.laser_vx, self.laser_vy, self.laser_omega_z]))

    def wheel_callback(self, msg): #wheel id = 1

        #fetch data from ros message
        self.wheel_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.wheel_x = msg.pose.pose.position.x
        self.wheel_y = msg.pose.pose.position.y
        self.pos = np.array([self.wheel_x, self.wheel_y])
        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y
        self.linear_vel = np.array([self.vx, self.vy])
        self.ori_wheel = np.vstack((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)).T
        self.omega_z = msg.twist.twist.angular.z
        
        #update sensor data in 'wheel_data'
        # self.wheel_data['timestamp'] = self.wheel_timestamp
        # self.wheel_data[0] = self.get_clock().now().nanoseconds * 1e-9
        # self.wheel_data[1] = self.pos
        # self.wheel_data[2] = self.linear_vel
        # self.wheel_data[3] = self.ori_wheel
        # self.wheel_data[4] = self.omega_z

        wheel_system_time = self.get_clock().now().nanoseconds * 1e-9

        self.flags.append(1)
        self.time_flags.append(wheel_system_time)
        self.wheel_data.append([wheel_system_time, self.pos, self.linear_vel, self.ori_wheel, self.omega_z])

        self.wheel_odom.append(np.append(wheel_system_time, [self.vx, self.vy, self.omega_z]))

    def imu_callback(self, msg): #imu id = 2

        #fetch data from ros message
        self.imu_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.imu_omega_z = msg.angular_velocity.z
        self.imu_lin_accl_x = msg.linear_acceleration.x
        self.imu_lin_accl_y = msg.linear_acceleration.y
        self.imu_lin_accl = np.array([self.imu_lin_accl_x, self.imu_lin_accl_y])
        
        #update sensor data in 'imu_data'
        # self.imu_data['timestamp'] = self.imu_timestamp
        # self.imu_data[0] = self.get_clock().now().nanoseconds * 1e-9
        # self.imu_data[1] = self.imu_omega_z
        # self.imu_data[2] = self.imu_lin_accl

        imu_system_time = self.get_clock().now().nanoseconds * 1e-9

        self.flags.append(2)
        self.time_flags.append(imu_system_time)
        self.imu_data.append([imu_system_time, self.imu_omega_z, self.imu_lin_accl])

        self.imu_data_measure.append(np.append(imu_system_time, [self.imu_omega_z, self.imu_lin_accl_x, self.imu_lin_accl_y]))

    def read_only_amcl_callback(self, msg):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        amcl_system_timestamp = self.get_clock().now().nanoseconds * 1e-9
        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y
        theta = self.quaternionToEuler([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]
        self.amcl_pose.append(np.append(amcl_system_timestamp, [pos_x, pos_y, theta]))

def main(args=None):
    rclpy.init(args=args)
    node = ROS()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        np.savetxt('amcl_pose.txt',np.array(node.amcl_pose))
        np.savetxt('wheel_odom.txt', np.array(node.wheel_odom))
        np.savetxt('laser_odom.txt', np.array(node.laser_odom))
        np.savetxt('imu_data_measure.txt', np.array(node.imu_data_measure))
        np.savetxt('states_ekf_one.txt', np.array(node.state_pub))
        node.get_logger().warn(f"Data saved")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

import time
import os
from dynamixel_sdk import *  
import numpy as np

# this Process only receive and send orders to motor : dynamixel trial first
# note this process should consider both mx540-w150 and MX 106(2.0)
class Motor_Process():
    def __init__(self,memory_all,DEVICENAME =  '/dev/ttyUSB1') :
        self.time_prev = time.perf_counter()
        self.memory_all = memory_all
        print("hello motor Process")
        self.DEVICENAME = DEVICENAME
        os.system("echo \"Setting low latency on usb ports\"") 
        os.system("setserial /dev/ttyUSB0 low_latency") 
        os.system("echo -n \"/dev/ttyUSB0 latency_timer: \" && cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer") 

        # import dynamixel_sdk
        self.ADDR_TORQUE_ENABLE          = 64
        
        self.ADDR_GOAL_POSITION          = 116
        self.LEN_GOAL_POSITION           = 4         # Data Byte Length
        self.ADDR_GOAL_VELOCITY          = 104
        self.LEN_GOAL_VELOCITY           = 4         # Data Byte Length
        self.ADDR_GOAL_CURRENT          = 102
        self.LEN_GOAL_CURRENT           = 2         # Data Byte Length
        self.ADDR_GOAL_PWM               = 100
        self.LEN_GOAL_PWM                = 2

        self.ADDR_PRESENT_POSITION       = 132
        self.LEN_PRESENT_POSITION        = 4         # Data Byte Length
        self.ADDR_PRESENT_VELOCITY       = 128
        self.LEN_PRESENT_VELOCITY        = 4         # Data Byte Length
        self.ADDR_PRESENT_CURRENT       = 126
        self.LEN_PRESENT_CURRENT        = 2         # Data Byte Length
        self.ADDR_PRESENT_PWM       = 124
        self.LEN_PRESENT_PWM        = 2         # Data Byte Length
 
        self.ADDR_RETURN_DELAY_TIME      = 9
        self.LEN_RETURN_DELAY_TIME        = 1         # Data Byte Length

        self.ADDR_OPERATING_MODE       = 11
        self.LEN_OPERATING_MODE        = 1         # Data Byte Length

        # DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
        # DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual
        self.BAUDRATE                    = 3000000

        # DYNAMIXEL Protocol Version (1.0 / 2.0)
        # https://emanual.robotis.com/docs/en/dxl/protocol2/
        self.PROTOCOL_VERSION            = 2.0

        # Make sure that each DYNAMIXEL ID should have unique ID.
        self.DXL1_ID                     = 1                 # Dynamixel#1 ID : 1
        self.DXL2_ID                     = 2                 # Dynamixel#1 ID : 2
        self.DXL3_ID                     = 3                 # Dynamixel#1 ID : 1
        self.DXL4_ID                     = 4                 # Dynamixel#1 ID : 2
        self.DXL5_ID                     = 5                 # Dynamixel#1 ID : 1
        self.DXL6_ID                     = 6                 # Dynamixel#1 ID : 2

        self.DXL_ID_list = [self.DXL1_ID,self.DXL2_ID,self.DXL3_ID,self.DXL4_ID,self.DXL5_ID,self.DXL6_ID]

        self.DXL1_ID_Latch                     = 10                 # Dynamixel#1 ID : 1
        self.DXL2_ID_Latch                     = 12                 # Dynamixel#1 ID : 2

        self.DXL_ID_Latch_list = [self.DXL1_ID_Latch,self.DXL2_ID_Latch]
        # self.DXL_ID_Latch_list = [self.DXL1_ID_Latch]

        self.TORQUE_ENABLE               = 1                 # Value for enabling the torque
        self.TORQUE_DISABLE              = 0                 # Value for disabling the torque
        # DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(self.DEVICENAME)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        # Initialize GroupBulkWrite instance
        self.groupBulkWrite = GroupBulkWrite(self.portHandler, self.packetHandler)

        # Initialize GroupBulkRead instace for Present Position
        self.groupBulkRead = GroupBulkRead(self.portHandler, self.packetHandler)

        self.Open_port()
        self.Set_port_baudrate(self.BAUDRATE)


        
        self.init_position_order = np.zeros((6,))
        self.init_position = np.zeros((6,))

        # do not know why these changes::: 
        # self.gear_ratio_list = [-3.5,-3.5,3.5,-3.5,3.5,3.5]
        # new one:: since the gear box is new version !! 
        self.gear_ratio_list = [3.5,-3.5,-3.5,3.5,3.5,-3.5]
        # self.gear_ratio_direction = []
        # for i in range(len(self.gear_ratio_list)):
        #     self.gear_ratio_direction.append(self.gear_ratio_list[i]/abs(self.gear_ratio_list[i]))

        self.gear_ratio_array = np.array(self.gear_ratio_list)

        # change the operating mode 
        Current_Control_Mode = 0
        Velocity_Control_Mode = 1
        Position_Control_Mode = 3
        Extended_Position_Control_Mode = 4
        Current_based_Position_Control_Mode = 5
        PWM_Control_Mode= 16 # (Voltage Control Mode) 
        self.motor_mode_list = [Extended_Position_Control_Mode,Extended_Position_Control_Mode,Extended_Position_Control_Mode,Extended_Position_Control_Mode,Extended_Position_Control_Mode,Extended_Position_Control_Mode]
        # joint motors: 
        for i in range(len(self.DXL_ID_list)):
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID_list[i], self.ADDR_OPERATING_MODE, self.motor_mode_list[i])
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel#%d mode has been changed to Extended_Position_Control_Mode" % self.DXL_ID_list[i])
        # latch motors: 
        for i in range(len(self.DXL_ID_Latch_list)):
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID_Latch_list[i], self.ADDR_OPERATING_MODE, PWM_Control_Mode)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel#%d mode has been changed to PWM_Control_Mode" % self.DXL_ID_Latch_list[i])
        # change the return delay time : 10 is 2*10us
        for i in range(len(self.DXL_ID_list)):
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID_list[i], self.ADDR_RETURN_DELAY_TIME, 10)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel#%d return delay time has been changed to 20us" % self.DXL_ID_list[i])
        for i in range(len(self.DXL_ID_Latch_list)):
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID_Latch_list[i], self.ADDR_RETURN_DELAY_TIME, 10)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel#%d return delay time has been changed to 20us" % self.DXL_ID_Latch_list[i])
        # add parameter for reading
        for i in range(len(self.DXL_ID_list)):
            # Add parameter storage for Dynamixel#1 present position
            dxl_addparam_result = self.groupBulkRead.addParam(self.DXL_ID_list[i], self.ADDR_PRESENT_CURRENT, self.LEN_PRESENT_CURRENT + self.LEN_PRESENT_VELOCITY + self.LEN_PRESENT_POSITION)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupBulkRead addparam failed" % self.DXL_ID_list[i])
                quit()
        for i in range(len(self.DXL_ID_Latch_list)):
            # Add parameter storage for Dynamixel#1 present position
            dxl_addparam_result = self.groupBulkRead.addParam(self.DXL_ID_Latch_list[i], self.ADDR_PRESENT_PWM, self.LEN_PRESENT_PWM + self.LEN_PRESENT_CURRENT)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupBulkRead addparam failed" % self.DXL_ID_Latch_list[i])
                quit()


        
        # this might need to change :: 
        
        # self.mx_106_unlock(self.DXL1_ID_Latch,0.75)

    def Open_port(self,):
        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")

    def Set_port_baudrate(self,BAUDRATE):
        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
        
    def update_init_position(self,):
        self.disable_torque()
        while 1 :
            # print("get init pos")
            dxl_comm_result = self.groupBulkRead.txRxPacket()
            for i in range(len(self.DXL_ID_list)):
                # init_position_order[i] = np.int32(groupBulkRead.getData(DXL_ID_list[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION))
                self.init_position_order[i] = np.array(self.groupBulkRead.getData(self.DXL_ID_list[i], self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)).astype(np.int32)
                # print(type(init_position_order[i]))
                self.init_position[i] = self.order_to_position(self.init_position_order[i])/self.gear_ratio_list[i]
            # print(self.init_position)
            # key = cv2.waitKey(1)
            state = self.memory_all.get("Keyboard")
            key = state.value[0]
            # key = self.memory_all.keyboard
            if key == ord('g'):
                Data = self.memory_all.create("Keyboard")
                Data.value = np.array([0])
                self.memory_all.set("Keyboard",Data)
                # self.memory_all.keyboard = 0
                # print("init_received")
                # since the initial angle is not parallel: T_c_a = mat_util.T_Rz(-35degree)
                self.init_position[0] =  self.init_position[0] + 35.0/180.0*np.pi


                dxl_present_position, dxl_present_velocity, dxl_present_current = self.read_joint_position_velocity_current()        
                Data = self.memory_all.create("PresentJointAngle")
                Data.position = dxl_present_position
                Data.velocity = dxl_present_velocity
                Data.current = dxl_present_current
                self.memory_all.set("PresentJointAngle",Data)

                break
            else:
                # print(key)
                time.sleep(0.01)
                pass

    def order_to_pwm_mx106(self,order):# pwm:+-1
        pwm = order/886
        return pwm

    def pwm_to_order_mx106(self,pwm):
        order = pwm*886
        return int(order)

    def order_to_current_mx106(self,order):
        current = order*3.36*0.001
        return current
        
    def current_to_order_mx106(self,current):
        order = current/(3.36*0.001)
        return int(order)
        
    def order_to_current(self,order):
        current = order*2.69*0.001
        return current
        
    def current_to_order(self,current):
        order = current/(2.69*0.001)
        return int(order)
        
    def order_to_velocity(self,order): # 0.229 rpm
        velocity = order*(0.229*(2*np.pi)/60)
        return velocity
        
    def velocity_to_order(self,velocity):
        order = velocity/(0.229*(2*np.pi)/60)
        return int(order)
        
    def order_to_position(self,order):
        position = order*(2*np.pi/4096.0)
        return position
        
    def position_to_order(self,position): 
        order = position/(2*np.pi/4096.0)
        return int(order)
        
    def read_latch_current_pwm(self,):
        # read 
        # Bulkread  
        dxl_comm_result = self.groupBulkRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        for i in range(len(self.DXL_ID_Latch_list)):
            # Check if groupbulkread data of Dynamixel#1 is available
            dxl_getdata_result = self.groupBulkRead.isAvailable(self.DXL_ID_Latch_list[i], self.ADDR_PRESENT_PWM, self.LEN_PRESENT_PWM)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupBulkRead getdata failed" % self.DXL_ID_Latch_list[i])
                quit()

            dxl_getdata_result = self.groupBulkRead.isAvailable(self.DXL_ID_Latch_list[i], self.ADDR_PRESENT_CURRENT, self.LEN_PRESENT_CURRENT)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupBulkRead getdata failed" % self.DXL_ID_Latch_list[i])
                quit()

        # Get present value
        dxl_present_current_latch = np.zeros((len(self.DXL_ID_Latch_list),))
        dxl_present_pwm_latch = np.zeros((len(self.DXL_ID_Latch_list),))
        for i in range(len(self.DXL_ID_Latch_list)):
            dxl_present_current_latch[i] = self.order_to_current_mx106(np.array(self.groupBulkRead.getData(self.DXL_ID_Latch_list[i], self.ADDR_PRESENT_CURRENT, self.LEN_PRESENT_CURRENT)).astype(np.int16))
            dxl_present_pwm_latch[i] = self.order_to_pwm_mx106(np.array(self.groupBulkRead.getData(self.DXL_ID_Latch_list[i], self.ADDR_PRESENT_PWM, self.LEN_PRESENT_PWM)).astype(np.int16))


        return dxl_present_current_latch, dxl_present_pwm_latch
        
    def read_joint_position_velocity_current(self,):
        # TODO: consider the error information
        # read 
        # Bulkread  
        dxl_comm_result = self.groupBulkRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        for i in range(len(self.DXL_ID_list)):
            # Check if groupbulkread data of Dynamixel#1 is available
            dxl_getdata_result = self.groupBulkRead.isAvailable(self.DXL_ID_list[i], self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)
            if dxl_getdata_result != True:
                print(dxl_getdata_result)
                print("[ID:%03d] groupBulkRead getdata failed ADDR_PRESENT_POSITION" % self.DXL_ID_list[i])
                quit()
            dxl_getdata_result = self.groupBulkRead.isAvailable(self.DXL_ID_list[i], self.ADDR_PRESENT_VELOCITY, self.LEN_PRESENT_VELOCITY)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupBulkRead getdata failed ADDR_PRESENT_VELOCITY" % self.DXL_ID_list[i])
                quit()
            dxl_getdata_result = self.groupBulkRead.isAvailable(self.DXL_ID_list[i], self.ADDR_PRESENT_CURRENT, self.LEN_PRESENT_CURRENT)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupBulkRead getdata failed ADDR_PRESENT_CURRENT" % self.DXL_ID_list[i])
                quit()
        # quit()
        # Get present value
        dxl_present_position = np.zeros((len(self.DXL_ID_list),))
        dxl_present_velocity = np.zeros((len(self.DXL_ID_list),))
        dxl_present_current = np.zeros((len(self.DXL_ID_list),))
        for i in range(len(self.DXL_ID_list)):
            dxl_present_position[i] = self.order_to_position(np.array(self.groupBulkRead.getData(self.DXL_ID_list[i], self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)).astype(np.int32)) /self.gear_ratio_list[i] - self.init_position[i]
            dxl_present_velocity[i] = self.order_to_velocity(np.array(self.groupBulkRead.getData(self.DXL_ID_list[i], self.ADDR_PRESENT_VELOCITY, self.LEN_PRESENT_VELOCITY)).astype(np.int32)) /self.gear_ratio_list[i]
            dxl_present_current[i] = self.order_to_current(np.array(self.groupBulkRead.getData(self.DXL_ID_list[i], self.ADDR_PRESENT_CURRENT, self.LEN_PRESENT_CURRENT)).astype(np.int16)) #  * self.gear_ratio_direction[]

        
        return dxl_present_position, dxl_present_velocity, dxl_present_current
        pass
    def set_joint_position_velocity_current(self,goal_postion,goal_velocity,goal_current):
        Current_Control_Mode = 0
        Velocity_Control_Mode = 1
        Position_Control_Mode = 3
        Extended_Position_Control_Mode = 4
        Current_based_Position_Control_Mode = 5
        PWM_Control_Mode= 16 # (Voltage Control Mode) 
        for i in range(len(self.DXL_ID_list)):
            # # calculate goal current : 
            # goal_current_i = T2C(goal_torque[i])
            if self.motor_mode_list[i] == Current_Control_Mode:
                # turn it to order 
                goal_current_i_order = self.current_to_order(goal_current[i])
                # goal_current_i_order = self.current_to_order(goal_current[i]*self.gear_ratio_list[i])

                # Allocate goal position value into byte array
                param_goal_current = [DXL_LOBYTE(goal_current_i_order),DXL_HIBYTE(goal_current_i_order)]
                
                # Add Dynamixel goal current value to the Bulkwrite parameter storage
                dxl_addparam_result = self.groupBulkWrite.addParam(self.DXL_ID_list[i], self.ADDR_GOAL_CURRENT, self.LEN_GOAL_CURRENT, param_goal_current)
                if dxl_addparam_result != True:
                    print("[ID:%03d] groupBulkWrite addparam failed" % self.DXL_ID_list[i])
                    quit()
            elif self.motor_mode_list[i] == Velocity_Control_Mode:
                # turn it to order 
                goal_velocity_i_order = self.velocity_to_order(goal_velocity[i]*self.gear_ratio_list[i])

                # Allocate goal position value into byte array
                param_goal_velocity = [DXL_LOBYTE(DXL_LOWORD(goal_velocity_i_order)),DXL_HIBYTE(DXL_LOWORD(goal_velocity_i_order)),DXL_LOBYTE(DXL_HIWORD(goal_velocity_i_order)),DXL_HIBYTE(DXL_HIWORD(goal_velocity_i_order))]

                # Add Dynamixel goal current value to the Bulkwrite parameter storage
                dxl_addparam_result = self.groupBulkWrite.addParam(self.DXL_ID_list[i], self.ADDR_GOAL_VELOCITY, self.LEN_GOAL_VELOCITY, param_goal_velocity)
                if dxl_addparam_result != True:
                    print("[ID:%03d] groupBulkWrite addparam failed" % self.DXL_ID_list[i])
                    quit()
            elif self.motor_mode_list[i] == Extended_Position_Control_Mode:
                # turn it to order 
                goal_postion_i_order = self.position_to_order((goal_postion[i] + self.init_position[i])*self.gear_ratio_list[i])

                # Allocate goal position value into byte array
                param_goal_position = [DXL_LOBYTE(DXL_LOWORD(goal_postion_i_order)),DXL_HIBYTE(DXL_LOWORD(goal_postion_i_order)),DXL_LOBYTE(DXL_HIWORD(goal_postion_i_order)),DXL_HIBYTE(DXL_HIWORD(goal_postion_i_order))]

                # Add Dynamixel goal current value to the Bulkwrite parameter storage
                dxl_addparam_result = self.groupBulkWrite.addParam(self.DXL_ID_list[i], self.ADDR_GOAL_POSITION, self.LEN_GOAL_POSITION, param_goal_position)
                if dxl_addparam_result != True:
                    print("[ID:%03d] groupBulkWrite addparam failed" % self.DXL_ID_list[i])
                    quit()
            else:
                pass
            
        # Bulkwrite  
        dxl_comm_result = self.groupBulkWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        # Clear bulkwrite parameter storage
        self.groupBulkWrite.clearParam()

    def set_joint_position(self,goal_postion):
        for i in range(len(self.DXL_ID_list)):
            # # calculate goal current : 
            # goal_current_i = T2C(goal_torque[i])
            # turn it to order 
            goal_postion_i_order = self.position_to_order((goal_postion[i] + self.init_position[i])*self.gear_ratio_list[i])

            # Allocate goal position value into byte array
            param_goal_position = [DXL_LOBYTE(DXL_LOWORD(goal_postion_i_order)),DXL_HIBYTE(DXL_LOWORD(goal_postion_i_order)),DXL_LOBYTE(DXL_HIWORD(goal_postion_i_order)),DXL_HIBYTE(DXL_HIWORD(goal_postion_i_order))]

            # Add Dynamixel goal current value to the Bulkwrite parameter storage
            dxl_addparam_result = self.groupBulkWrite.addParam(self.DXL_ID_list[i], self.ADDR_GOAL_POSITION, self.LEN_GOAL_POSITION, param_goal_position)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupBulkWrite addparam failed" % self.DXL_ID_list[i])
                quit()
        # Bulkwrite  
        dxl_comm_result = self.groupBulkWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        # Clear bulkwrite parameter storage
        self.groupBulkWrite.clearParam()
        pass
    def mx_106_memory_set_force(self,latch_bool_both):
        Data_goal_latch = self.memory_all.create("GoalLatchLock")
        Data_present_latch = self.memory_all.create("PresentLatchLock")
        Data_force_latch = self.memory_all.create("ForceGoalLatchLock")

        Data_goal_latch.value = latch_bool_both
        Data_present_latch.value = latch_bool_both
        Data_force_latch.action = np.array([False])
        Data_force_latch.value = latch_bool_both

        self.memory_all.set("GoalLatchLock",Data_goal_latch)
        self.memory_all.set("PresentLatchLock",Data_present_latch)
        self.memory_all.set("ForceGoalLatchLock",Data_force_latch)

        for i in range(len(self.DXL_ID_Latch_list)):
            self.set_latch_pwm(self.DXL_ID_Latch_list[i],0)


    def mx_106_memory_set(self,ID,latch_bool):
        index = self.DXL_ID_Latch_list.index(ID)
        present_latch_lock = self.memory_all.get("PresentLatchLock").value.copy()
        Data = self.memory_all.create("PresentLatchLock")
        present_latch_lock[index] = latch_bool
        Data.value = present_latch_lock
        self.memory_all.set("PresentLatchLock",Data)



        self.set_latch_pwm(ID,0)
        
    def mx_106_lock(self,ID,pwm):
        self.time_prev = time.perf_counter()
        index = self.DXL_ID_Latch_list.index(ID)
        positive_pwm_unlock_direction_ID = [10,12]
        positive_pwm_lock_direction_ID = []
        if ID in positive_pwm_unlock_direction_ID:
            pwm = -abs(pwm)
            pass
        if ID in positive_pwm_lock_direction_ID:
            pwm = abs(pwm)
            pass
        # pwm = abs(pwm)
        self.set_latch_pwm(ID,pwm)
        while 1:
            self.time_now = time.perf_counter()
            # dxl_present_current_latch, dxl_present_pwm_latch = self.read_latch_current_pwm()
            if ((self.time_now - self.time_prev) >=1 ): #0.2856->85*3.36*0.001
            # if (((self.time_now - self.time_prev) >=2 ) and (abs(dxl_present_current_latch[0])>0.2856)): #0.2856->85*3.36*0.001
                present_latch_lock = self.memory_all.get("PresentLatchLock").value.copy()
                Data = self.memory_all.create("PresentLatchLock")
                present_latch_lock[index] = True
                Data.value = present_latch_lock
                self.memory_all.set("PresentLatchLock",Data)

                self.set_latch_pwm(ID,0)
                break
            # raise Exception("lock!",dxl_present_current_latch,dxl_present_pwm_latch)

    def mx_106_unlock(self,ID,pwm):
        self.time_prev = time.perf_counter()
        index = self.DXL_ID_Latch_list.index(ID)
        positive_pwm_unlock_direction_ID = [10,12]
        positive_pwm_lock_direction_ID = []
        if ID in positive_pwm_unlock_direction_ID:
            pwm = abs(pwm)
            pass
        if ID in positive_pwm_lock_direction_ID:
            pwm = -abs(pwm)
            pass
        # pwm = -abs(pwm)
        self.set_latch_pwm(ID,pwm)
        # while 1:
        #     self.time_now = time.perf_counter()
        #     # dxl_present_current_latch, dxl_present_pwm_latch = self.read_latch_current_pwm()
        #     if ((self.time_now - self.time_prev) >=1 ): #0.2856->85*3.36*0.001
        #     # if (((self.time_now - self.time_prev) >=2 ) and (abs(dxl_present_current_latch[0])>0.2856)): #0.2856->85*3.36*0.001
        #         present_latch_lock = self.memory_all.get("PresentLatchLock").value.copy()
        #         Data = self.memory_all.create("PresentLatchLock")
        #         present_latch_lock[index] = False
        #         Data.value = present_latch_lock
        #         self.memory_all.set("PresentLatchLock",Data)

        #         self.set_latch_pwm(ID,0)
        #         break

            
    def set_latch_pwm(self,ID,pwm):   
        pwm_order = self.pwm_to_order_mx106(pwm)  
        # Allocate goal PWM value into byte array
        param_goal_pwm = [DXL_LOBYTE(pwm_order),DXL_HIBYTE(pwm_order)]

        # Add Dynamixel goal current value to the Bulkwrite parameter storage
        dxl_addparam_result = self.groupBulkWrite.addParam(ID, self.ADDR_GOAL_PWM, self.LEN_GOAL_PWM, param_goal_pwm)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupBulkWrite addparam failed" % ID)
            quit()
        # Bulkwrite  
        dxl_comm_result = self.groupBulkWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        # Clear bulkwrite parameter storage
        self.groupBulkWrite.clearParam()

    def enable_or_disable_torque_joint_5(self,enable_or_disable):
        # Enable Dynamixel Torque 5
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID_list[4], self.ADDR_TORQUE_ENABLE, enable_or_disable)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            pass
            # print("Dynamixel#%d torque has been successfully enabled" % self.DXL_ID_list[4])

    def enable_torque(self,):
        self.enable_torque_joints()
        self.enable_torque_latch()
    def enable_torque_joints(self,):
        # Enable Dynamixel Torque
        for i in range(len(self.DXL_ID_list)):
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID_list[i], self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                pass
                # print("Dynamixel#%d torque has been successfully enabled" % self.DXL_ID_list[i])
    def enable_torque_latch(self,):     
        # Enable Dynamixel Torque
        for i in range(len(self.DXL_ID_Latch_list)):
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID_Latch_list[i], self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel#%d torque has been successfully enabled" % self.DXL_ID_Latch_list[i])


    def disable_torque(self,):
        self.disable_torque_joints()
        self.disable_torque_latch()
    def disable_torque_joints(self,):
        # Disable Dynamixel Torque
        for i in range(len(self.DXL_ID_list)):
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID_list[i], self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel#%d torque has been successfully disabled" % self.DXL_ID_list[i])
    def disable_torque_latch(self,):
        # Disable Dynamixel Torque
        for i in range(len(self.DXL_ID_Latch_list)):
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler,  self.DXL_ID_Latch_list[i], self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel#%d torque has been successfully disabled" % self.DXL_ID_Latch_list[i])
        
    def set_motor_mode(self,Motor_Id,Motor_Mode):
        # change Dynamixel motor mode 
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, Motor_Id, self.ADDR_OPERATING_MODE, Motor_Mode)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d mode has been changed" % Motor_Id)
        
    def set_motor_torque(self,Motor_Id,Torque_On_Off):
        # changed Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, Motor_Id, self.ADDR_TORQUE_ENABLE, Torque_On_Off)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d torque has been changed" % Motor_Id)
        

    def update(self,):
        key = self.memory_all.get("Keyboard").value[0]
        if key == ord("q"):
            self.disable_torque()
            quit()

        dxl_present_position, dxl_present_velocity, dxl_present_current = self.read_joint_position_velocity_current()
        
        Data = self.memory_all.create("PresentJointAngle")
        Data.position = dxl_present_position
        Data.velocity = dxl_present_velocity
        Data.current = dxl_present_current
        self.memory_all.set("PresentJointAngle",Data)

        Data = self.memory_all.create("JointAngleReadFlag")
        Data.value = np.array([True])
        self.memory_all.set("JointAngleReadFlag",Data) 

        dxl_goal_position = self.memory_all.get("GoalJointAngle").value
        dxl_goal_velocity = self.memory_all.get("GoalJointVelocity").value
        dxl_goal_current = self.memory_all.get("GoalJointCurrent").value

        # self.set_joint_position(dxl_goal_position)
        self.set_joint_position_velocity_current(dxl_goal_position,dxl_goal_velocity,dxl_goal_current)

        # NOTE: BASE0: id 12 ,PWM+: LOCK
        # NOTE: BASE0: id 10 ,PWM+: UNLOCK
        # # the following might need to change :: 
        goal_latch_lock = self.memory_all.get("GoalLatchLock").value
        present_latch_lock = self.memory_all.get("PresentLatchLock").value

        

        TimeLatchNow = time.perf_counter()
        TLatch = 1
        if ((goal_latch_lock[0] == True)and(present_latch_lock[0] == False)) :
            if (TimeLatchNow - self.latch_0_time)<TLatch:
                self.mx_106_lock(self.DXL1_ID_Latch,0.9)
            else:
                self.mx_106_memory_set(self.DXL1_ID_Latch,latch_bool = True)
        elif ((goal_latch_lock[0] == False)and(present_latch_lock[0] == True)) :
            if (TimeLatchNow - self.latch_0_time)<TLatch:
                self.mx_106_unlock(self.DXL1_ID_Latch,0.9)
            else:
                self.mx_106_memory_set(self.DXL1_ID_Latch,latch_bool = False)
        else:
            self.latch_0_time = TimeLatchNow

        if ((goal_latch_lock[1] == True)and(present_latch_lock[1] == False)) :
            if (TimeLatchNow - self.latch_1_time)<TLatch:
                self.mx_106_lock(self.DXL2_ID_Latch,0.9)
            else:
                self.mx_106_memory_set(self.DXL2_ID_Latch,latch_bool = True)
        elif ((goal_latch_lock[1] == False)and(present_latch_lock[1] == True)) :
            if (TimeLatchNow - self.latch_1_time)<TLatch:
                self.mx_106_unlock(self.DXL2_ID_Latch,0.9)
            else:
                self.mx_106_memory_set(self.DXL2_ID_Latch,latch_bool = False)
        else:
            self.latch_1_time = TimeLatchNow

        # NOTE:: do not mix the logic together between different mechanism
        # # # check if it uses force latch 
        force_latch_data = self.memory_all.get("ForceGoalLatchLock")
        if force_latch_data.action[0] == True:
            if (TimeLatchNow - self.latch_2_time ) < TLatch : 
                print("force_latch_data.value::",force_latch_data.value)
                if force_latch_data.value[0] == True:
                    self.mx_106_lock(self.DXL1_ID_Latch,0.9)
                else:
                    self.mx_106_unlock(self.DXL1_ID_Latch,0.9)

                if force_latch_data.value[1] == True:
                    self.mx_106_lock(self.DXL2_ID_Latch,0.9)
                else:
                    self.mx_106_unlock(self.DXL2_ID_Latch,0.9)
            else:
                self.mx_106_memory_set_force(force_latch_data.value)
        else:
            self.latch_2_time = TimeLatchNow



        # # # check if need to change the motor mode ::
        MotorData = self.memory_all.get("MotorState")
        SetData = self.memory_all.create("MotorState")
        if MotorData.action == True:
            SetData.action = np.array([False])
            SetData.motor_id = MotorData.motor_id
            SetData.motor_mode = MotorData.motor_mode
            SetData.torque_on_off = MotorData.torque_on_off
            self.memory_all.set("MotorState",SetData)

            Motor_Id = MotorData.motor_id
            Motor_Mode = MotorData.motor_mode
            Torque_On_Off = MotorData.torque_on_off
            self.set_motor_mode(Motor_Id,Motor_Mode)
            self.set_motor_torque(Motor_Id,Torque_On_Off)
            id_index = self.DXL_ID_list.index(Motor_Id)
            self.motor_mode_list[id_index] = Motor_Mode

        return dxl_present_position, dxl_present_velocity, dxl_present_current, dxl_goal_position

        
    
    def run(self):
        # sys.stdout=open('myDataOutput_motor.txt','w')  #此语句实现了将标准输出重定向为myDataoutput.txt
        self.update_init_position()
        ##  self.enable_torque()
        self.enable_torque_joints()
        self.enable_torque_latch()
        Data = self.memory_all.create("PresentLatchLock")
        Data.value = np.array([True,False])
        self.memory_all.set("PresentLatchLock",Data)

        Data = self.memory_all.create("GoalLatchLock")
        Data.value = np.array([True,False])
        self.memory_all.set("GoalLatchLock",Data)
        while 1:
            self.update()
            time.sleep(0.01)
          
from PyQt5.QtCore import QThread, pyqtSignal
from rebotArm_handle import reBotArm_handle
from motorbridge import Controller
import pinocchio_handle
import time
from pathlib import Path
import sys
import pinocchio as pin

def resource_path(relative_path):
    """获取资源的绝对路径，兼容开发和打包环境"""
    if hasattr(sys, '_MEIPASS'):
        # 打包后：资源在临时解压目录
        return Path(sys._MEIPASS) / relative_path
    # 开发环境，相对于当前脚本所在目录
    base_dir = Path(__file__).resolve().parent.parent
    return base_dir / relative_path

rebotArm_DM_model_path = resource_path("urdf/reBot-B601_DM_fixend/urdf/reBot-DevArm_fixend.urdf")
# rebotArm_DM_model_path = resource_path("urdf/reBot_B601_DM/urdf/reBot_B601_DM.urdf")
rebotArm_RS_model_path = resource_path("urdf/reBot-B601_RS_fixend/urdf/00-arm-rs_asm-v3.urdf")

class ArmControlThread(QThread):
    rebot_arm_joints_pos = pyqtSignal(list) 
    error_occurred = pyqtSignal(str)  # 用于传递错误信息
    log_message = pyqtSignal(str)  # 用于传递日志信息
    connected = pyqtSignal(str)  # 用于通知连接成功
    set_mode_signal = pyqtSignal(str)
    set_zero_is_success =  pyqtSignal(bool)
    wait_change_mode = pyqtSignal(bool)
    end_pose = pyqtSignal(list)  # 用于传递末端位姿信息
    def __init__(self, channel,rebot_ver):
        super().__init__()
        self.channel = channel
        self.baudrate = 921600
        self.rebot_ver = rebot_ver 
        self.rebot_handle_ver = None
        self._is_running = True
        self.handle = None
        self.ctrl = None
        self.addt = 0
        self.mode = None
        self.rec_mode = None
        self.enable_change_mode = True
        self.mode_is_running = True # 是否先安全退出模式

        # 绑定信号和槽函数
        self.set_mode_signal.connect(self._on_set_mode)
        if self.rebot_ver == "rebot-B601-DM":
            self.model = pin.buildModelFromUrdf(rebotArm_DM_model_path)
        elif self.rebot_ver == "rebot-B601-RS":
            self.model = pin.buildModelFromUrdf(rebotArm_RS_model_path)
        self.data = self.model.createData()
    def run(self):
        try:
            if "can" in self.channel.lower():
                self.ctrl = Controller(self.channel)
            elif "com" in self.channel.lower():
                self.ctrl = Controller.from_dm_serial(self.channel, self.baudrate)
            else :
                return False
            if self.rebot_ver == "rebot-B601-DM":
                config_path = resource_path("config/rebotDM.yaml")
                self.rebot_handle_ver = "rebotDM"
            elif self.rebot_ver  == "rebot-B601-RS":
                config_path = resource_path("config/rebotRS.yaml")
                self.rebot_handle_ver = "rebotRS"
            
            with reBotArm_handle(self.ctrl, self.rebot_handle_ver, config_path=config_path, log_handle=self.log_message) as handle:
                self.handle = handle 
                if self.handle.is_connected :
                    self.connected.emit(f"连接成功")
                else:
                    self.connected.emit(f"连接失败")
                    return            
                self._is_running = True
                while self._is_running:

                    if self.mode == "gravity_mode_on":
                        joints_pos = handle.return_joints_last_pos()
                        # joints_pos = handle.get_joints_state()
                        full_torques = pinocchio_handle.gravity_compensation_control(joints_pos ,self.model,self.data,self.rebot_ver)  
                        if self.rebot_ver == "rebot-B601-RS":
                            neg_pos = [x for x in joints_pos]
                            neg_torques=[x for x in full_torques]
                            handle.move_to_joint_positions(positions = neg_pos,torque=neg_torques)   
                        else: 
                            handle.move_to_joint_positions(positions = joints_pos,torque=full_torques)
                        # print("\nTORQUES:")
                        # print(full_torques)
                        print("tau_g:", full_torques[:6])
                        print("\nPOS:")
                        print(joints_pos)
                        self.enable_change_mode = False
                    elif self.mode == "gravity_mode_off":
                        self.handle.return_zero_position()
                        print("stop")
                        self.mode = None
                        self.enable_change_mode = True


                    if self.mode == "set_zero_mode_on":
                        self.enable_change_mode = False
                        self.set_zero_is_success.emit(handle.set_zero_position())
                        self.mode = None
                        self.enable_change_mode = True

                    if self.mode == "record_mode_on":
                        self.enable_change_mode = False

                    elif self.mode == "record_mode_off":
                        # self.handle.return_zero_position
                        self.enable_change_mode = True


                    if self.mode == "read_joint_mode_on":
                        self.enable_change_mode = False
                        self.rebot_arm_joints_pos.emit(handle.get_joints_state())

                    elif self.mode == "read_joint_mode_off":
                        self.enable_change_mode = True

                    time.sleep(0.002)
                    self.addt += 0.002
                    if(self.addt > 0.006):
                        self.addt = 0
                        # Matrix_t ,Matrix_r = pinocchio_handle.forwardKinematics_compute (handle,self.model,self.data)
                        # x,y,z,roll_deg,pitch_deg,yaw_deg = pinocchio_handle.deal_with_matrix_t_r(Matrix_t,Matrix_r)
                        pos = self.handle.return_joints_last_pos()
                        self.rebot_arm_joints_pos.emit(pos) 
                        # self.end_pose.emit([x,y,z,roll_deg,pitch_deg,yaw_deg]) 
                        self.wait_change_mode.emit(self.enable_change_mode) #向上层传递是否可以切换模式
                self.connected.emit(f"断开连接")
                self._is_running = False
        except Exception as e:
            self.error_occurred.emit(str(e))

    def _on_set_mode(self, mode):
        self.mode = mode
        # print(self.enable_change_mode)
        # print(mode)
        # print(self.mode)

    def stop(self):                 
        self._is_running = False    # 通知 while 循环退出
        self.wait()                 # 等待 run() 真正返回


class MotorControlThread(QThread):
    motor_pos = pyqtSignal(list) 
    def __init__(self, channel,motor_ver):
        super().__init__()
        self.channel = channel
        self.baudrate = 921600
        self.motor_ver = motor_ver 
        self.motor_handle_ver = None
        self._is_running = True
        self.handle = None
        self.ctrl = None
        self.addt = 0
        self.mode = None
        self.mode_is_running = True # 是否先安全退出模式

        # 绑定信号和槽函数
        self.set_mode_signal.connect(self._on_set_mode)

    def run(self):
        return
    
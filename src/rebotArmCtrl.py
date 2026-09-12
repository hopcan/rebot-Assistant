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

rebotArm_DM_model_path = resource_path("urdf/reBot-DevArm_fixend_description/urdf/reBot-DevArm_fixend.urdf")

model = pin.buildModelFromUrdf(rebotArm_DM_model_path)
data = model.createData()

class ArmControlThread(QThread):
    rebot_arm_joints_pos = pyqtSignal(list) 
    error_occurred = pyqtSignal(str)  # 用于传递错误信息
    log_message = pyqtSignal(str)  # 用于传递日志信息
    connected = pyqtSignal(str)  # 用于通知连接成功
    set_mode_signal = pyqtSignal(str)
    def __init__(self, channel, baudrate=921600):
        super().__init__()
        self.channel = channel
        self.baudrate = baudrate
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

    def run(self):
        try:
            self.ctrl = Controller.from_dm_serial(self.channel, self.baudrate)
            config_path = resource_path("config/rebotDM.yaml")
            with reBotArm_handle(self.ctrl, "rebotDM", config_path=config_path, log_handle=self.log_message) as handle:
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
                        motors_last_pos,full_torques = pinocchio_handle.gravity_compensation_control(joints_pos ,model,data)  
                        handle.move_to_joint_positions(positions = motors_last_pos,torque = full_torques)
                    elif self.mode == "gravity_mode_off":
                        self.handle.return_zero_position()
                        print("stop")
                        self.mode = None
                        self.enable_change_mode = True


                    if self.mode == "ctrl_mode_on":
                        self.enable_change_mode = False

                    elif self.mode == "ctrl_mode_off":
                        # self.handle.return_zero_position
                        self.enable_change_mode = True

                    if self.mode == "record_mode_on":
                        self.enable_change_mode = False

                    elif self.mode == "record_mode_off":
                        # self.handle.return_zero_position
                        self.enable_change_mode = True


                    if self.mode == "joint_ctrl_mode_on":
                        self.enable_change_mode = False

                    elif self.mode == "joint_ctrl_mode_off":
                        # self.handle.return_zero_position
                        self.enable_change_mode = True

                    time.sleep(0.002)
                    self.addt += 0.002
                    if(self.addt > 0.006):
                        self.addt = 0
                        pos = self.handle.return_joints_last_pos()
                        self.rebot_arm_joints_pos.emit(pos[:6])  
                self.connected.emit(f"断开连接")
                self._is_running = False
        except Exception as e:
            self.error_occurred.emit(str(e))

    def _on_set_mode(self, mode):
        self.mode = mode
        print(self.enable_change_mode)
        print(mode)
        print(self.mode)

    def stop(self):                 
        self._is_running = False    # 通知 while 循环退出
        self.wait()                 # 等待 run() 真正返回
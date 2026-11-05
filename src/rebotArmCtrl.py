from PyQt5.QtCore import QThread, pyqtSignal
from rebotArm_handle import reBotArm_handle
from motor_handle import motor_handle
from motorbridge import Controller
import pinocchio_handle
import time
from pathlib import Path
import sys
import pinocchio as pin
import os

base_dir = Path(__file__).resolve().parent.parent


def resource_path(relative_path):
    """获取资源的绝对路径，兼容开发和打包环境"""
    if hasattr(sys, '_MEIPASS'):
        # 打包后：资源在临时解压目录
        return Path(sys._MEIPASS) / relative_path
    # 开发环境，相对于当前脚本所在目录
    return base_dir / relative_path


rebotDM_ik_file_path = os.fspath(resource_path("config/rebotDM_ik.yaml"))
rebotDM_gravity_file_path = os.fspath(resource_path("config/rebotDM_gravity.yaml"))
rebotRS_ik_file_path = os.fspath(resource_path("config/rebotRS_ik.yaml"))
rebotRS_gravity_file_path = os.fspath(resource_path("config/rebotRS_gravity.yaml"))


rebotArm_DM_model_path = resource_path("urdf/reBot-B601_DM_fixend/urdf/reBot-DevArm_fixend.urdf")
# rebotArm_DM_model_path = resource_path("urdf/reBot_B601_DM/urdf/reBot_B601_DM.urdf")
rebotArm_RS_model_path = resource_path("urdf/reBot-B601_RS_fixend/urdf/00-arm-rs_asm-v3.urdf")

class ArmControlThread(QThread):
    rebot_arm_joints_pos = pyqtSignal(list) 
    error_occurred = pyqtSignal(str)  # 用于传递错误信息
    log_message = pyqtSignal(str)  # 用于传递日志信息
    connected = pyqtSignal(str)  # 用于通知连接成功
    set_mode_signal = pyqtSignal(str) # 用于传递模式设置
    mode_info = pyqtSignal(str) # 用于传递当前模式额外信息

    set_zero_is_success =  pyqtSignal(bool) # 用于传递设置零点是否成功
    return_zero_pos_is_success =  pyqtSignal(bool) # 用于传递回到零位是否成功
    move_end_to_pos_is_success =  pyqtSignal(bool) # 用于传递末端移动到指定位置是否成功

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

        self.Num = 0 # 轨迹点数
        self.is_generate_traj= False

        # 绑定信号和槽函数
        self.set_mode_signal.connect(self._on_set_mode)
        self.mode_info.connect(self._on_mode_info)

        # 模式参数
        self.mode_detail_info = None
        self.get_mode_info = False

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
                        if self.rebot_ver == "rebot-B601-DM":
                            handle.reconnect(config_path=rebotDM_gravity_file_path)
                        elif self.rebot_ver  == "rebot-B601-RS":
                            handle.reconnect(config_path=rebotRS_gravity_file_path)
                        joints_pos = handle.return_joints_last_pos()
                        full_torques = pinocchio_handle.gravity_compensation_control(joints_pos ,self.model,self.data,self.rebot_ver)  
                        handle.move_to_joint_positions(positions = joints_pos,torque=full_torques)
                        self.enable_change_mode = False
                    elif self.mode == "gravity_mode_off":
                        self.handle.return_zero_position()
                        time.sleep(2)
                        self.handle.move_to_joint_positions(positions = [0]*7)
                        self.mode = None
                        self.enable_change_mode = True


                    if self.mode == "set_zero_mode_on":
                        self.enable_change_mode = False
                        self.set_zero_is_success.emit(handle.set_zero_position())
                        self.mode = None
                        self.enable_change_mode = True

                    if self.mode == "return_zero_pos_mode_on":
                        self.enable_change_mode = False
                        self.return_zero_pos_is_success.emit(handle.return_zero_position())
                        time.sleep(2)
                        handle.move_to_joint_positions(positions = [0]*7)
                        self.mode = None
                        self.enable_change_mode = True

                    if self.mode == "move_end_mode_on" and self.get_mode_info == True:
                        q_joints = [0]*7
                        # target_pos_point = [0.22 ,0.18, 0.34, 0, 0.6, 0]
                        dt = 0.1
                        t = 5
                        x, y, z, roll, pitch, yaw, t = map(float, self.mode_detail_info.split())
                        target_pos_point = [x, y, z, roll, pitch, yaw]

                        if self.is_generate_traj == False:
                            self.log_message.emit("开始生成轨迹。")
                            if self.rebot_ver == "rebot-B601-DM":
                                handle.reconnect(config_path=rebotDM_ik_file_path)
                            elif self.rebot_ver == "rebot-B601-RS":
                                handle.reconnect(config_path=rebotRS_ik_file_path)
                            joints_pos = handle.return_joints_last_pos()
                            if self.rebot_ver == "rebot-B601-DM":
                                original_joints_pos = joints_pos[:6]
                            elif self.rebot_ver == "rebot-B601-RS":
                                original_joints_pos = joints_pos[:7]
                            q_traj, q_num = pinocchio_handle.generate_traj(original_joints_pos = original_joints_pos,
                            target_pos_point = target_pos_point,T = t,dt = dt,
                            model = self.model, data = self.data, rebot_ver = self.rebot_ver, frame_id = None)
                            self.is_generate_traj = True
                            if q_num == None:
                                self.log_message.emit("轨迹生成失败，可能接近奇异点，请重试。")
                            else:
                                self.log_message.emit(f"轨迹生成完毕，轨迹点数为：{q_num}。")
                        # 生成轨迹并且轨迹点数大于0，执行轨迹
                        if self.is_generate_traj == True and q_num:
                            if self.Num < q_num :
                                q_joints[:6] = q_traj[self.Num]
                                if self.Num < q_num:
                                    joints_pos = handle.return_joints_last_pos()
                                    full_torques = pinocchio_handle.gravity_compensation_control(joints_pos ,self.model,self.data,self.rebot_ver)  
                                    handle.move_to_joint_positions(positions = q_joints, torque=full_torques)
                                    time.sleep(0.02)
                                    self.Num += 1
                                self.enable_change_mode = False
                            else: # 结束执行轨迹
                                target = q_traj[-1]
                                q_joints[:6] = target
                                settle_cnt = 0
                                handle.move_to_joint_positions(positions = q_joints)
                                while settle_cnt < 50:  # 最多等 100*0.02 = 2s
                                    now = handle.return_joints_last_pos()[:6]
                                    self.rebot_arm_joints_pos.emit(now) 
                                    err = max(abs(a - b) for a, b in zip(now, target))
                                    if err < 0.02:
                                        self.move_end_to_pos_is_success.emit(True)
                                        break
                                    time.sleep(0.1)
                                    settle_cnt += 1
                                self.Num = 0
                                self.mode = None
                                self.enable_change_mode = True
                                self.is_generate_traj = False
                                self.get_mode_info = False
                                self.move_end_to_pos_is_success.emit(True)
                        elif self.is_generate_traj == True and q_num == None:
                            self.mode = None
                            self.enable_change_mode = True
                            self.is_generate_traj = False
                            self.get_mode_info = False
                            self.move_end_to_pos_is_success.emit(False)

                    time.sleep(0.002)
                    self.addt += 0.002
                    if(self.addt >= 0.002):
                        self.addt = 0
                        pos = self.handle.return_joints_last_pos()
                        self.rebot_arm_joints_pos.emit(pos) 
                        self.wait_change_mode.emit(self.enable_change_mode) #向上层传递是否可以切换模式
                self.connected.emit(f"断开连接")
        except Exception as e:
            self.error_occurred.emit(str(e))

    def _on_set_mode(self, mode):
        self.mode = mode

    def _on_mode_info(self, info):
        self.mode_detail_info = info
        self.get_mode_info = True

    def stop(self):                 
        self._is_running = False    # 通知 while 循环退出
        self.wait()                 # 等待 run() 真正返回


class MotorControlThread(QThread):
    mode_result = pyqtSignal(dict) # 给上层传递模式结果
    connected = pyqtSignal(str)  # 用于通知连接成功
    mode_config_signal = pyqtSignal(dict) #上层传递模式和参数
    error_occurred = pyqtSignal(str)  # 用于传递错误信息
    log_message = pyqtSignal(str)  # 用于传递日志信息
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
        self.mode = None # 上层选择电机功能
        self.mode_is_running = True # 是否先安全退出模式
        # 绑定信号和槽函数
        self.mode_config_signal.connect(self._mode_fuction)
    def run(self):
        try:
            # 根据断开创建总线
            if "can" in self.channel.lower():
                self.ctrl = Controller(self.channel)
            elif "com" in self.channel.lower():
                self.ctrl = Controller.from_dm_serial(self.channel, self.baudrate)
            else :
                return False
            print("motor thread is running")
            print(self.motor_ver)
            with motor_handle(self.ctrl, motor_version=self.motor_ver,log_handle=self.log_message) as handle:
                self.handle = handle
                self.connected.emit(f"连接成功")
                print("motor handle is running")          
                self._is_running = True
                while self._is_running:
                    if self.mode == "scan_motor":
                        found_motors = self.handle.scan_motor(start_can_id=self.start_id,end_can_id=self.end_id)
                        self.mode = "wait_motor"
                        # 向上层发送扫描结果
                        result = {"scan_mode":found_motors}
                        self.mode_result.emit(result)
                        print(found_motors)
                self.connected.emit(f"断开连接")
                self._is_running = False 
        except Exception as e:
            self.error_occurred.emit(str(e))
            self.connected.emit(f"连接失败")
            return

    def _mode_fuction(self,mode):
        self.mode = list(mode.keys())[0]
        self.start_id = mode[self.mode]["start id"]
        self.end_id = mode[self.mode]["end id"]
        print("key:")
        print(self.mode)
        if self.mode == None or self.mode == "wait_motor":
            self.mode = mode

    def stop(self):                 
        self._is_running = False    # 通知 while 循环退出
        self.wait()                 # 等待 run() 真正返回
    
import sys
import serial.tools.list_ports
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton , QVBoxLayout, QLabel, QSlider
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, 
                             QVBoxLayout, QHBoxLayout, QComboBox, 
                             QPushButton, QLabel, QTextEdit)  
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import Qt, QTimer
import os
import sys
import time
from pathlib import Path
import robot_pybullet
import rebotArmCtrl

import pybullet 
import pybullet_data

def resource_path(relative_path):
    """获取资源的绝对路径，兼容开发和打包环境"""
    if hasattr(sys, '_MEIPASS'):
        # 打包后：资源在临时解压目录
        return Path(sys._MEIPASS) / relative_path
    # 开发环境，相对于当前脚本所在目录
    base_dir = Path(__file__).resolve().parent.parent
    return base_dir / relative_path

class rebot_Simulation_App(QMainWindow):
    def __init__(self):
        super().__init__()

        # bullet 初始化
        self.physics_client = pybullet.connect(pybullet.DIRECT) # 无头模式
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        pybullet.setPhysicsEngineParameter(
            fixedTimeStep=1/500,      
            numSubSteps=5
        )
                # 加载地面
        plane_id = pybullet.loadURDF("plane.urdf")
        pybullet.changeVisualShape(plane_id, -1, rgbaColor=[0.8, 0.9, 1.0, 1.0]) #-1 表示修改该物体整体
        self.robot_id =  pybullet.loadURDF(str(rebotArmCtrl.rebotArm_DM_model_path), useFixedBase=True,
                                           flags = pybullet.URDF_USE_SELF_COLLISION)
        for i in range(6) :
            pybullet.changeVisualShape(self.robot_id,i, rgbaColor=[1, 1, 1, 1]) 
        pybullet.changeVisualShape(self.robot_id,-1, rgbaColor=[1, 1, 1, 1]) 

        # 过滤y碰撞组
        self.colliding = set()
        pybullet.setCollisionFilterPair(self.robot_id, self.robot_id, 3, 5, enableCollision=0)
        pybullet.setCollisionFilterPair(self.robot_id, self.robot_id, 1, 2, enableCollision=0)
        pybullet.setCollisionFilterPair(self.robot_id, self.robot_id, 2, 3, enableCollision=0)
        pybullet.setCollisionFilterPair(self.robot_id, self.robot_id, 3, 4, enableCollision=0)
        pybullet.setCollisionFilterPair(self.robot_id, self.robot_id, 4, 5, enableCollision=0)
        pybullet.setCollisionFilterPair(self.robot_id, self.robot_id, 5, 6, enableCollision=0)

        # 机械臂控制线程
        self.arm_thread = None 
        self.arm_thread_connected = False  # 标记线程是否已连接
        self.arm_mode_is_running = False

        # 设置窗口
        self.setWindowTitle("rebot Assistant")
        self.setGeometry(100, 100, 800, 600)  # x, y, width, height
        self.setWindowIcon(QIcon(str(resource_path("ico/seeed_studio.ico"))))

        # 中央部件和主布局
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        self.main_layout = QVBoxLayout(central_widget)
        self.main_layout.setContentsMargins(10, 10, 10, 10)  # 留边距

        # 顶部区域，包含设备下拉列表、扫描和连接按钮
        top_layout = QHBoxLayout()
        
        # 设备下拉列表
        self.combo = QComboBox()
        self.combo.addItems(["请选择设备"])
        self.combo.setCurrentIndex(0)  # 默认选中第一项
        top_layout.addWidget(self.combo)

        # 扫描按钮
        self.btn_scan = QPushButton("扫描")
        self.btn_scan.clicked.connect(self.on_scan)
        top_layout.addWidget(self.btn_scan)

        # 连接按钮
        self.btn_connect = QPushButton("连接")
        self.btn_connect.setStyleSheet("background-color: #4CAF50; color: white;")  # 绿色
        self.btn_connect.clicked.connect(self.on_connect)
        top_layout.addWidget(self.btn_connect)

        # 弹性空间，将控件推至左上角
        top_layout.addStretch()  # 这会让下拉列表和按钮靠左

        # 将顶部布局加入主布局
        self.main_layout.addLayout(top_layout)


        # 日志区
        self.log_area = QTextEdit()
        self.log_area.setLayout(QVBoxLayout())
        self.log_area.setPlaceholderText("日志信息...")
        self.log_area.setFixedHeight(100)  
        self.main_layout.addWidget(self.log_area)


        # 主区域
        self.rebot_main = QWidget()
        self.rebot_main_layout = QHBoxLayout(self.rebot_main)
        self.main_layout.addWidget(self.rebot_main)

        # 状态栏
        self.status_label = QLabel("等待连接")
        self.statusBar().addWidget(self.status_label)


        # 功能区
        self.selected_mode_container = QWidget()
        self.mode_layout = QVBoxLayout(self.selected_mode_container)
        self.mode_layout.setSpacing(15)                 # 按钮间距 3 像素
        self.mode_layout.setContentsMargins(0, 0, 0, 0) # 容器内边距 0

        self.gravity_mode_btn = QPushButton("开启重力补偿")
        self.gravity_mode_btn.clicked.connect(self.gravity_mode)
        self.mode_layout.addWidget(self.gravity_mode_btn)

        self.ctrl_mode_btn = QPushButton("开启运动学控制")
        self.ctrl_mode_btn.clicked.connect(self.ctrl_mode)
        self.mode_layout.addWidget(self.ctrl_mode_btn)

        self.record_mode_btn = QPushButton("开启示教")
        self.record_mode_btn.clicked.connect(self.record_mode)
        self.mode_layout.addWidget(self.record_mode_btn)

        self.joint_ctrl_mode_btn = QPushButton("开启关节控制")
        self.joint_ctrl_mode_btn.clicked.connect(self.joint_ctrl_mode)
        self.mode_layout.addWidget(self.joint_ctrl_mode_btn)

        
        self.reset_arm_btn = QPushButton("重置仿真机械臂")
        self.reset_arm_btn.clicked.connect(self.reset_arm)
        self.mode_layout.addWidget(self.reset_arm_btn)


        # 关节控制区
        self.sliders = []
        self.labels = []
        self.values = []
        self.link_value = [0]*6
        self.link_names = ["link1","link2","link3","link4","link5","link6"] #,"end_link"]
        self.slider_min = [-2.6,-3.8,-3.8, -1.56,-1.56,-3.14]
        self.slider_max = [2.6,0,0, 1.56,1.56,3.14 ]
        for i in range(6):
            # 每一行都是水平布局
            row = QHBoxLayout()

            label = QLabel(self.link_names[i]) 
            label.setFixedWidth(70)
            label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

            value = QLabel("0.00 rad")
            value.setFixedWidth(70)
            value.setAlignment(Qt.AlignCenter)

            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(int(self.slider_min[i]*100))   # -180.0°
            slider.setMaximum(int(self.slider_max[i]*100))    # 180.0°
            slider.setValue(0)
            slider.valueChanged.connect(
                lambda value, idx=i: self.on_slider_changed(idx, value)
            )
            # 松手发送关节角度
            slider.sliderReleased.connect(
                lambda idx=i: self.on_slider_release(idx)
            )
            row.addWidget(label)
            row.addWidget(value)

            self.sliders.append(slider)
            self.labels.append(label)
            self.values.append(value)
            self.mode_layout.addLayout(row)
            self.mode_layout.addWidget(slider)

        self.mode_layout.addStretch() 

        self.rebot_main_layout.addWidget(self.selected_mode_container, stretch=1)

        # 渲染区
        self.show_rebot = robot_pybullet.RobotRenderer(self.physics_client)
        self.rebot_main_layout.addWidget(self.show_rebot , stretch=4)
        # 定时更新渲染
        self.sim_timer = QTimer(self)
        self.sim_timer.timeout.connect(self.update_sim)
        self.sim_timer.start(10)   

        self.show_timer = QTimer(self)
        self.show_timer.timeout.connect(self.update_show)
        self.show_timer.start(10)   
         

    def on_scan(self):
        """处理扫描按钮点击事件"""
        self.log_area.clear()
        self.combo.clear()
        self.status_label.setText("正在扫描设备...")
        if (self.get_com_ports()):
            self.log_area.append("扫描完成,发现以下设备。")
        else:
            self.log_area.append("扫描完成,未发现设备。")

        for ports in self.ports:
            self.log_area.append(f"设备: {ports.device}, 描述: {ports.description}")
            self.combo.addItems([f"{ports.device}"])
        self.status_label.setText("扫描完成")

    def on_connect(self):
        selected = self.combo.currentText()
        self.status_label.setText(f"正在连接 {selected} ...")

        if selected == "请选择设备" or selected == "":
            self.status_label.setText("请先选择一个有效设备")
            self.log_area.append("请先选择一个有效设备。")
            return
        else:
            """处理连接按钮点击事件"""
            if self.btn_connect.text() == "连接":

                # 创建线程对象
                self.arm_thread = rebotArmCtrl.ArmControlThread(selected, 921600)
                self.arm_thread.rebot_arm_joints_pos.connect(self.on_pos_received)
                # 绑定信号
                self.arm_thread.log_message.connect(self.on_log)
                self.arm_thread.error_occurred.connect(self.on_error)
                self.arm_thread.connected.connect(self.on_thread_connected)  # 连接成功信号绑定到日志输出

                # 启动控制线程
                self.arm_thread.start()

            elif self.btn_connect.text() == "断开":
                # 执行断开操作...
                if self.arm_thread and self.arm_mode_is_running == False :
                    self.arm_thread.stop()     
                    self.arm_thread = None 
                    time.sleep(0.3)
                elif self.arm_thread and self.arm_mode_is_running == True :
                    self.log_area.append("请先关闭模式。")

    # 重置仿真机械臂
    def reset_arm(self):
        for i in range(6):
            self.sliders[i].setValue(0)
            pybullet.resetJointState(
            bodyUniqueId = self.robot_id,
            jointIndex = i,
            targetValue = 0,             # 目标角度（弧度）
            targetVelocity = 0.0                 # 目标速度
            )
            self.link_value[i] = 0
            pybullet.setJointMotorControl2(
            bodyUniqueId = self.robot_id,
            jointIndex = i,
            controlMode = pybullet.POSITION_CONTROL,   # 位置控制
            targetPosition = self.link_value[i],          # 目标角度（弧度）
            maxVelocity=100.0,
            force=500                          # 最大力矩
        )
        
            
            
    # 获取可用的串口列表
    def get_com_ports(self):
        self.ports = serial.tools.list_ports.comports()
        return self.ports

    # 打印对应关节(连杆)名称
    def print_link(self):
        num_joints = pybullet.getNumJoints(self.robot_id)
        print(f"机器人共有 {num_joints} 个关节")
        for i in range(num_joints):
            # getJointInfo 返回一个包含关节信息的列表
            joint_info = pybullet.getJointInfo(self.robot_id, i)
            link_name = joint_info[12].decode('utf-8')
            print(f"关节索引: {i}, 子连杆名称: {link_name}")
        #pybullet.changeVisualShape(self.robot_id,6, rgbaColor=[1, 0, 0, 1]) # 给对应连杆上色


    def gravity_mode(self):
        if self.arm_thread :
            if self.gravity_mode_btn.text() == "开启重力补偿" and self.arm_mode_is_running == False:
                self.arm_thread.set_mode_signal.emit("gravity_mode_on")
                self.arm_mode_is_running = True
                self.gravity_mode_btn.setText("关闭重力补偿")
                self.gravity_mode_btn.setStyleSheet("background-color: #f44336; color: white;")  # 红色
            elif self.gravity_mode_btn.text() == "关闭重力补偿" and self.arm_mode_is_running == True:
                self.arm_thread.set_mode_signal.emit("gravity_mode_off")
                self.arm_mode_is_running = False
                self.gravity_mode_btn.setText("开启重力补偿")
                self.gravity_mode_btn.setStyleSheet("")  
        return 

    def ctrl_mode(self):
        if self.arm_thread :
            if self.ctrl_mode_btn.text() == "开启运动学控制" and self.arm_mode_is_running == False:
                self.arm_thread.set_mode_signal.emit("ctrl_mode_on")
                self.arm_mode_is_running = True
                self.ctrl_mode_btn.setText("关闭运动学控制")
                self.ctrl_mode_btn.setStyleSheet("background-color: #f44336; color: white;")  # 红色
            elif self.ctrl_mode_btn.text() == "关闭运动学控制" and self.arm_mode_is_running == True:
                self.arm_thread.set_mode_signal.emit("ctrl_mode_off")
                self.arm_mode_is_running = False
                self.ctrl_mode_btn.setText("开启运动学控制")
                self.ctrl_mode_btn.setStyleSheet("")  
        return

    def record_mode(self):
        if self.arm_thread:
            if self.record_mode_btn.text() == "开启示教" and self.arm_mode_is_running == False:
                self.arm_thread.set_mode_signal.emit("record_mode_on")
                self.arm_mode_is_running = True
                self.record_mode_btn.setText("关闭示教")
                self.record_mode_btn.setStyleSheet("background-color: #f44336; color: white;")  # 红色
            elif self.record_mode_btn.text() == "关闭示教" and self.arm_mode_is_running == True:
                self.arm_thread.set_mode_signal.emit("record_mode_off")
                self.arm_mode_is_running = False
                self.record_mode_btn.setText("开启示教")
                self.record_mode_btn.setStyleSheet("")  
        return

    def joint_ctrl_mode(self):
        if self.arm_thread:
            if self.joint_ctrl_mode_btn.text() == "开启关节控制" and self.arm_mode_is_running == False:
                self.arm_thread.set_mode_signal.emit("joint_ctrl_mode_on")
                self.arm_mode_is_running = True
                self.joint_ctrl_mode_btn.setText("关闭关节控制")
                self.joint_ctrl_mode_btn.setStyleSheet("background-color: #f44336; color: white;")  # 红色
            elif self.joint_ctrl_mode_btn.text() == "关闭关节控制" and self.arm_mode_is_running == True:
                self.arm_thread.set_mode_signal.emit("joint_ctrl_mode_off")
                self.arm_mode_is_running = False
                self.joint_ctrl_mode_btn.setText("开启关节控制")
                self.joint_ctrl_mode_btn.setStyleSheet("")  
        return



    # 连杆处理
    def on_slider_changed(self,idx,value):
        rad = value / 100
        self.values[idx].setText(f"{rad:.2f}rad")
        self.link_value[idx] = rad

    def on_slider_release(self,idx):
        # pybullet.resetJointState(
        #     bodyUniqueId = self.robot_id,
        #     jointIndex = idx,
        #     targetValue = self.link_value[idx],             # 目标角度（弧度）
        #     targetVelocity = 0.0                 # 目标速度
        # )
        pybullet.setJointMotorControl2(
            bodyUniqueId = self.robot_id,
            jointIndex = idx,
            controlMode = pybullet.POSITION_CONTROL,   # 位置控制
            targetPosition = self.link_value[idx],          # 目标角度（弧度）
            maxVelocity=100.0,
            force=500                          # 最大力矩
        )
    
    # 处理日志和错误信号的槽函数
    def on_log(self, msg):
        """接收日志信号"""  
        self.log_area.append(msg)

    def on_error(self, msg):
        """接收错误信号"""
        self.log_area.append(f"[错误] {msg}")

    def on_thread_connected(self, msg):
        """接收线程连接成功信号"""
        if msg == "连接成功":
            self.btn_connect.setText("断开")
            self.btn_connect.setStyleSheet("background-color: #f44336; color: white;")  # 红色
            self.status_label.setText("已连接")
        elif msg == "连接失败" or msg == "断开连接":
            self.btn_connect.setText("连接")
            self.btn_connect.setStyleSheet("background-color: #4CAF50; color: white;")  # 绿色
            self.status_label.setText("已断开")
        self.log_area.append(msg)

    def on_pos_received(self,pos):
        for i in range(len(pos)):
        #     pybullet.resetJointState(
        #     bodyUniqueId = self.robot_id,
        #     jointIndex = i,
        #     targetValue = pos[i],             # 目标角度（弧度）
        #     targetVelocity = 0.0                 # 目标速度
        # )
            pybullet.setJointMotorControl2(
                bodyUniqueId = self.robot_id,
                jointIndex = i,
                controlMode = pybullet.POSITION_CONTROL,   # 位置控制
                targetPosition = pos[i],          # 目标角度（弧度）
                maxVelocity=100.0,
                force=500,                          # 最大力矩
                
            )
            pos_int = int(pos[i]*100)
            self.sliders[i].setValue(pos_int)
            # print(joint_pos)

    def update_sim(self):
        pybullet.stepSimulation()

    
    def update_show(self):
        if self.show_rebot.isVisible():
            self.show_rebot.update()
        contacts = pybullet.getContactPoints(bodyA=self.robot_id, bodyB=self.robot_id)
        for i in range(pybullet.getNumJoints(self.robot_id)):
            pybullet.changeVisualShape(self.robot_id, i, rgbaColor=[1, 1, 1, 1])
        if contacts:
            self.colliding.clear()
            for c in contacts:
                # if c[3] != -1 :
                #     joint_infoA = pybullet.getJointInfo(self.robot_id, c[3])
                #     link_nameA = joint_infoA[12].decode('utf-8')
                    
                # else :
                #     link_nameA = "base_link"
                # if c[4] != -1 :
                #     joint_infoB = pybullet.getJointInfo(self.robot_id, c[4])
                #     link_nameB = joint_infoB[12].decode('utf-8')
                # else :
                #     link_nameB = "base_link"
                # print(f"子连杆名称A: {link_nameA},子连杆名称B: {link_nameB} ")
                self.colliding.add(c[3])
                self.colliding.add(c[4])
            for i in range(pybullet.getNumJoints(self.robot_id)):
                if i in self.colliding:
                    # 碰撞的连杆红色
                    pybullet.changeVisualShape(self.robot_id, i, rgbaColor=[1, 0, 0, 1])
                    


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = rebot_Simulation_App()
    window.show()
    sys.exit(app.exec_())
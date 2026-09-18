import sys
import serial.tools.list_ports
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, 
                             QVBoxLayout, QHBoxLayout, QComboBox, 
                             QPushButton, QLabel, QTextEdit,QLineEdit,
                             QSlider,QOpenGLWidget,QDesktopWidget)  
from PyQt5.QtGui import QIcon,QPainter, QImage,QColor,QFont
from PyQt5.QtCore import Qt, QTimer
import os
import time
from pathlib import Path
import robot_pybullet
import rebotArmCtrl
import pybullet 
import can
import pybullet_data
import ctypes
from ctypes import wintypes

sys.path.insert(0, str(Path(__file__).resolve().parent))
from robot_opengl import GLURDFRenderer

platform = "windows" #linux or windows
sim_engine = "pybullet"
urdf = Path(__file__).resolve().parent.parent / \
        "urdf/reBot-DevArm_fixend_description/urdf/reBot-DevArm_fixend.urdf"
# urdf = Path(__file__).resolve().parent.parent / \
#         "urdf/00-arm-rs_asm-v3/urdf/00-arm-rs_asm-v3.urdf"
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
        # 通信接口
        self.serial_ports = None
        self.pcan_ports = None
        if platform == "windows":
            self.pcan_map = {f"PCAN_USBBUS{i}": f"can{i-1}" for i in range(1, 9)}

        # 机械臂控制线程
        self.arm_thread = None 
        self.arm_thread_connected = False  # 标记线程是否已连接
        self.arm_mode_is_running = False
        

        # 设置窗口
        self.setWindowTitle("rebot Assistant")
        self.setGeometry(100, 100, 800, 600)  # x, y, width, height
        self.center_on_screen()
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

        # 碰撞显示按钮
        self.show_collision_on = False # 默认不显示
        self.show_collision = QPushButton("开启碰撞显示")
        self.show_collision.setStyleSheet("background-color: #4CAF50; color: white;")  # 绿色
        self.show_collision.clicked.connect(self.on_show_collision)
        top_layout.addWidget(self.show_collision)

        # 弹性空间，将控件推至左上角
        top_layout.addStretch()  # 这会让下拉列表和按钮靠左

        # 将顶部布局加入主布局
        self.main_layout.addLayout(top_layout)


        # 日志区
        self.log_area = QTextEdit()
        self.log_area.Layout = QVBoxLayout()
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
        self.joint_angles = [0]*6
        self.link_names = ["link1","link2","link3","link4","link5","link6"] #,"end_link"]
        self.slider_min = [-2.6,-3.8,-3.8, -1.56,-1.56,-3.14]
        self.slider_max = [2.6,0,0, 1.56,1.56,3.14 ]
        self.robot_id_map = {-1:"base_link", 0:"link1",1:"link2",2:"link3",3:"link4",
                             4:"link5",5:"link6",6:"end_link"}
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
        # bullet 初始化
        if sim_engine == "pybullet":
            self.physics_client = pybullet.connect(pybullet.DIRECT) # 无头模式 pybullet.DIRECT

            pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
            pybullet.setPhysicsEngineParameter(
                fixedTimeStep=1/1000,      
                numSubSteps=20
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
            self.show_rebot = GLURDFRenderer(str(urdf))

            self.rebot_main_layout.addWidget(self.show_rebot , stretch=4)

            # 定时更新渲染
            self.sim_timer = QTimer(self)
            self.sim_timer.timeout.connect(self.update_sim)
            self.sim_timer.start(5)   

        


    def on_scan(self):
        """处理扫描按钮点击事件"""
        self.log_area.clear()
        self.combo.clear()
        self.status_label.setText("正在扫描设备...")
        if (self.get_ports()):
            self.log_area.append("扫描完成,发现以下设备。")
        else:
            self.log_area.append("扫描完成,未发现设备。")
        if platform == "windows":
            if self.serial_ports :
                for serial_ports in self.serial_ports:
                    self.log_area.append(f"设备: {serial_ports.device}, 描述: {serial_ports.description}")
                    self.combo.addItems([f"{serial_ports.device}"])

            if self.pcan_ports :
                for pcan_ports in self.pcan_ports:
                    pcan_name = pcan_ports['channel']
                    map_name = self.pcan_map[f"{pcan_name}"]
                    self.log_area.append(f"设备:{map_name}, 描述: {pcan_name}")
                    self.combo.addItems([f"{map_name}"])              
        
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
                    time.sleep(0.1)
                elif self.arm_thread and self.arm_mode_is_running == True :
                    self.log_area.append("请先关闭模式。")

    def on_show_collision(self):
        if self.show_collision.text() == "开启碰撞显示":
            self.show_collision_on = True
            self.show_collision.setText("关闭碰撞显示")
            self.show_collision.setStyleSheet("background-color: #f44336; color: white;")  # 红色
        elif self.show_collision.text() == "关闭碰撞显示":
            self.show_collision_on = False
            self.show_collision.setText("开启碰撞显示")
            self.show_collision.setStyleSheet("background-color: #4CAF50; color: white;")  # 绿色


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
    def get_ports(self):
        self.serial_ports = serial.tools.list_ports.comports()
        try:
            self.pcan_ports = can.detect_available_configs(interfaces='pcan')
        except Exception as e:
            print(f"PCAN 扫描被策略拦截，跳过: {e}")
            self.pcan_ports = []
        if self.serial_ports or self.pcan_ports:    
            return True
        else:
            return False

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

    def update_sim(self):
        pybullet.stepSimulation()
        num_joints = pybullet.getNumJoints(self.robot_id)
        joint_states = pybullet.getJointStates(self.robot_id, range(num_joints))
        self.joint_angles = [state[0] for state in joint_states]  # 所有关节的角度（弧度）列表
        self.show_rebot.set_joint_positions(self.joint_angles)
        if self.show_collision_on:
            self.update_show_collision()
        else:
            for link_name in self.robot_id_map.values():
                self.show_rebot.set_link_color(link_name, (0.627, 0.627, 0.627, 1))   


    def update_show_collision(self):
        contacts = pybullet.getContactPoints(bodyA=self.robot_id, bodyB=self.robot_id)
        for i in range(pybullet.getNumJoints(self.robot_id)):
            self.show_rebot.set_link_color(self.robot_id_map[i], (0.627, 0.627, 0.627, 1))    

        if contacts:
            self.colliding.clear()
            for c in contacts:
                self.colliding.add(c[3])
                self.colliding.add(c[4])
            for i in range(pybullet.getNumJoints(self.robot_id)):
                if i in self.colliding:
                    # 碰撞的连杆红色
                    self.show_rebot.set_link_color(self.robot_id_map[i], (1, 0, 0, 1))    # 红色



    def center_on_screen(self):
        screen = QDesktopWidget().availableGeometry()
        x = (screen.width() - 800) // 2
        y = (screen.height() - 600) // 2
        self.move(x, y)
from PyQt5.QtWidgets import QSplashScreen
from PyQt5.QtGui import QPixmap
if __name__ == "__main__":
    app = QApplication(sys.argv)

    try:
        import pyi_splash
        pyi_splash.close()
    except ImportError:
        pass

    splash = QSplashScreen(QPixmap(str(resource_path("ico/splash.png"))))
    splash.setWindowFlags(
    Qt.WindowStaysOnTopHint | Qt.FramelessWindowHint | Qt.SplashScreen
    )
    splash.show()

    app.processEvents()

    window = rebot_Simulation_App()
    splash.close()
    window.show()
    sys.exit(app.exec_())
import sys
import serial.tools.list_ports
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, 
                             QVBoxLayout, QHBoxLayout, QComboBox, 
                             QPushButton, QLabel, QTextEdit, QDialogButtonBox,
                             QSlider,QDesktopWidget,QDialog,QStackedWidget,QLineEdit,QFrame)  
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import Qt, QTimer
import os
import time
from pathlib import Path
import rebotArmCtrl
import pybullet 
import can
import pybullet_data
import style
sys.path.insert(0, str(Path(__file__).resolve().parent))
from robot_opengl import GLURDFRenderer

platform = "windows" #linux or windows
sim_engine = "pybullet"
rebot_ver = None
 
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
        print(rebot_ver)
        self.serial_ports = None
        self.pcan_ports = None
        if platform == "windows":
            self.pcan_map = {f"PCAN_USBBUS{i}": f"can{i-1}" for i in range(1, 9)}

        # 设置窗口
        self.setWindowTitle("rebot Assistant")
        self.setGeometry(100, 100, 400, 300)  # x, y, width, height
        # self.center_on_screen()
        self.setWindowIcon(QIcon(str(resource_path("ico/seeed_studio.ico"))))
        self.setStyleSheet(style.main_window_style)

        # 中央部件和主布局
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        #############################
        ##########页面管理###########
        # 页面堆叠容器
        root_layout = QVBoxLayout(central_widget)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)

        # 顶部页面切换按钮
        nav_layout = QHBoxLayout()
        nav_layout.setContentsMargins(5, 5, 5, 0)

        self.btn_page_arm = QPushButton("机械臂控制")
        self.btn_page_arm.clicked.connect(self.set_page_arm)
        self.btn_page_motor = QPushButton("电机控制")
        self.btn_page_motor.clicked.connect(self.set_page_motor)

        self.btn_page_arm.setCheckable(True)
        self.btn_page_motor.setCheckable(True)

        nav_layout.addWidget(self.btn_page_arm)
        nav_layout.addWidget(self.btn_page_motor)
        nav_layout.addStretch()

        root_layout.addLayout(nav_layout)

        self.stack = QStackedWidget()
        root_layout.addWidget(self.stack, stretch=1)

        # ---------- 第一页 机械臂----------
        self.page_arm = QWidget()
        self.main_layout = QVBoxLayout(self.page_arm)
        self.main_layout.setContentsMargins(10, 10, 10, 10)
        self.stack.addWidget(self.page_arm)

        # ---------- 第二页 电机页 ----------
        self.page_motor = QWidget()
        self.page_motor_layout = QVBoxLayout(self.page_motor)
        self.page_motor_layout.setContentsMargins(10, 10, 10, 10)
        self.stack.addWidget(self.page_motor)

        # 切换按钮绑定
        # 默认显示第一页
        self.stack.setCurrentIndex(0)
        self.rebot_arm_page_init()
        self.motor_page_init()
        ############################
        ############################

    #########################################################
    ######################切换页面逻辑########################
    def set_page_arm(self):
        self.stack.setCurrentIndex(0)
        self.btn_page_arm.setChecked(True) # 选中效果
        self.btn_page_motor.setChecked(False)

    def set_page_motor(self):
        self.stack.setCurrentIndex(1)
        self.btn_page_arm.setChecked(False)
        self.btn_page_motor.setChecked(True)
    #########################################################
    #########################################################

    def rebot_arm_page_init(self):
        # 机械臂控制线程
        self.arm_thread = None 
        self.arm_thread_connected = False  # 标记线程是否已连接
        self.enable_change_mode = True

        # 顶部区域，包含设备下拉列表、扫描和连接按钮
        self.top_layout = QHBoxLayout()
        
        # 设备下拉列表
        self.combo = QComboBox()
        self.combo.addItems(["请选择设备"])
        self.combo.setCurrentIndex(0)  # 默认选中第一项
        self.top_layout.addWidget(self.combo)

        # 扫描按钮
        self.btn_scan = QPushButton("扫描")
        self.btn_scan.clicked.connect(self.on_scan)
        self.btn_scan.setStyleSheet(style.scan_style)
        self.top_layout.addWidget(self.btn_scan)

        # 连接按钮
        self.btn_connect = QPushButton("连接")
        self.btn_connect.setStyleSheet(style.arm_connect_on_style)  # 绿色
        self.btn_connect.clicked.connect(self.on_connect)
        self.top_layout.addWidget(self.btn_connect)

        # 碰撞显示按钮
        self.show_collision_on = False # 默认不显示
        self.show_collision = QPushButton("开启碰撞显示")
        self.show_collision.setStyleSheet(style.show_collision_on_style)  
        self.show_collision.clicked.connect(self.on_show_collision)
        self.top_layout.addWidget(self.show_collision)

        # 轨迹显示按钮
        self.show_traj_on = False # 默认不显示
        self.show_traj = QPushButton("开启末端轨迹显示")
        self.show_traj.setStyleSheet(style.show_collision_on_style)  # 绿色
        self.show_traj.clicked.connect(self.on_show_traj)
        self.top_layout.addWidget(self.show_traj)

        # 弹性空间，将控件推至左上角
        self.top_layout.addStretch()  # 这会让下拉列表和按钮靠左

        # 将顶部布局加入主布局
        self.main_layout.addLayout(self.top_layout)


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
        self.mode_layout.setSpacing(5)                 # 按钮间距 3 像素
        self.mode_layout.setContentsMargins(0, 0, 0, 0) # 容器内边距 0
        self.selected_mode_container.setFixedWidth(200)

        self.gravity_mode_btn = QPushButton("开启重力补偿")
        self.gravity_mode_btn.clicked.connect(self.gravity_mode)
        self.gravity_mode_btn.setStyleSheet(style.function_btn_on_style)
        self.mode_layout.addWidget(self.gravity_mode_btn)
        if rebot_ver == "rebot-B601-RS":  
            self.gravity_mode_btn.setEnabled(True)

        self.set_zero_mode_btn = QPushButton("设置机械臂零点")
        self.set_zero_mode_btn.clicked.connect(self.set_zero_mode)
        self.set_zero_mode_btn.setStyleSheet(style.function_btn_on_style) 
        self.set_zero_mode_is_done = False
        self.mode_layout.addWidget(self.set_zero_mode_btn)

        self.record_mode_btn = QPushButton("开启示教")
        self.record_mode_btn.clicked.connect(self.record_mode)
        self.record_mode_btn.setStyleSheet(style.function_btn_on_style) 
        self.mode_layout.addWidget(self.record_mode_btn)
        self.record_mode_btn.setEnabled(False)

        self.read_joint_mode_btn = QPushButton("开启实时读取关节角度")
        self.read_joint_mode_btn.clicked.connect(self.read_joint_mode)
        self.read_joint_mode_btn.setStyleSheet(style.function_btn_on_style) 
        self.mode_layout.addWidget(self.read_joint_mode_btn)

        self.reset_arm_btn = QPushButton("重置仿真机械臂")
        self.reset_arm_btn.clicked.connect(self.reset_arm)
        self.reset_arm_btn.setStyleSheet(style.reset_sim_arm_btn_style)
        self.mode_layout.addWidget(self.reset_arm_btn)


        # 关节控制区
        self.sliders = []
        self.labels = []
        self.values = []
        self.link_value = [0]*7
        self.joint_angles = [0]*7
        self.link_names = ["link1","link2","link3","link4","link5","link6","gripper"]
        if rebot_ver == "rebot-B601-DM":
            self.slider_min = [-2.6,-3.8,-3.8, -1.56,-1.56,-3.14,-5.67]
            self.slider_max = [2.6,0,0, 1.56,1.56,3.14,0 ]
            self.robot_id_map = {-1:"base_link", 0:"link1",1:"link2",2:"link3",3:"link4",
                            4:"link5",5:"link6",6:"gripper_link",7:"gripper_left",8:"gripper_right"}
        elif rebot_ver == "rebot-B601-RS":
            self.slider_min = [-2.6,0,0, -1.56,-1.56,-3.14,-5.67]
            self.slider_max = [2.6,3.8,3.8, 1.56,1.56,3.14,0 ]
            self.robot_id_map = {-1:"base_link", 0:"link1",1:"link2",2:"link3",3:"link4",
                            4:"link5",5:"link6",6:"end_link",7:"gripper_left",8:"gripper_right"}
        for i in range(7):
            # 每一行都是水平布局
            row = QHBoxLayout()

            label = QLabel(self.link_names[i]) 
            label.setStyleSheet(style.joint_pos_label_style)
            label.setFixedWidth(100)
            label.setAlignment(Qt.AlignCenter)
            value = QLabel("0.00 rad")
            value.setFixedWidth(70)
            value.setAlignment(Qt.AlignCenter)

            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(int(self.slider_min[i]*100))   # -180.0°
            slider.setMaximum(int(self.slider_max[i]*100))    # 180.0°
            slider.setValue(0)
            slider.setStyleSheet(style.slider_style)
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
            self.robot_id =  pybullet.loadURDF(str(urdf), useFixedBase=True,
                                                flags = pybullet.URDF_USE_SELF_COLLISION)
            for i in range(6) :
                pybullet.changeVisualShape(self.robot_id,i, rgbaColor=[1, 1, 1, 1]) 
            pybullet.changeVisualShape(self.robot_id,-1, rgbaColor=[1, 1, 1, 1]) 

            # 过滤y碰撞组
            self.colliding = set()
            pybullet.setCollisionFilterPair(self.robot_id, self.robot_id, 2, 4, enableCollision=0)
            pybullet.setCollisionFilterPair(self.robot_id, self.robot_id, 1, 2, enableCollision=0)
            pybullet.setCollisionFilterPair(self.robot_id, self.robot_id, 2, 3, enableCollision=0)
            pybullet.setCollisionFilterPair(self.robot_id, self.robot_id, 3, 4, enableCollision=0)
            pybullet.setCollisionFilterPair(self.robot_id, self.robot_id, 4, 5, enableCollision=0)
            # pybullet.setCollisionFilterPair(self.robot_id, self.robot_id, 5, 6, enableCollision=0)
            pybullet.setCollisionFilterPair(self.robot_id, self.robot_id, 3, 6, enableCollision=0)
            pybullet.setCollisionFilterPair(self.robot_id, self.robot_id, 4, 6, enableCollision=0)
            pybullet.setCollisionFilterPair(self.robot_id, self.robot_id, 3, 5, enableCollision=0)
            pybullet.setCollisionFilterPair(self.robot_id, self.robot_id, 7, 8, enableCollision=0)

            self.show_rebot = GLURDFRenderer(str(urdf))
            self.rebot_main_layout.addWidget(self.show_rebot , stretch=4)
            self.show_rebot.set_end_effector("gripper_end")


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
                self.arm_thread = rebotArmCtrl.ArmControlThread(selected,rebot_ver)
                # 创建线程信号
                self.arm_thread.rebot_arm_joints_pos.connect(self.on_pos_received)
                self.arm_thread.set_zero_is_success.connect(self.on_set_zero_is_done_received)
                self.arm_thread.wait_change_mode.connect(self.on_wait_change_mode)
                # 绑定信号
                self.arm_thread.log_message.connect(self.on_log)
                self.arm_thread.error_occurred.connect(self.on_error)
                self.arm_thread.connected.connect(self.on_thread_connected)  # 连接成功信号绑定到日志输出

                # 启动控制线程
                self.arm_thread.start()

            elif self.btn_connect.text() == "断开":
                # 执行断开操作...
                if self.arm_thread and self.enable_change_mode:
                    self.arm_thread.stop()     
                    self.arm_thread = None 
                    time.sleep(0.1)
                elif self.arm_thread and self.enable_change_mode == False:
                    self.log_area.append("请先关闭模式。")

    def on_show_collision(self):
        if self.show_collision.text() == "开启碰撞显示":
            self.show_collision_on = True
            self.show_collision.setText("关闭碰撞显示")
            self.show_collision.setStyleSheet(style.show_collision_off_style)  # 红色
        elif self.show_collision.text() == "关闭碰撞显示":
            self.show_collision_on = False
            self.show_collision.setText("开启碰撞显示")
            self.show_collision.setStyleSheet(style.show_collision_on_style)  # 绿色

    def on_show_traj(self):
        if self.show_traj.text() == "开启末端轨迹显示":
            self.show_traj_on = True
            self.show_traj.setText("关闭末端轨迹显示")
            self.show_traj.setStyleSheet(style.show_collision_off_style)  # 红色
        elif self.show_traj.text() == "关闭末端轨迹显示":
            self.show_traj_on = False
            self.show_traj.setText("开启末端轨迹显示")
            self.show_traj.setStyleSheet(style.show_collision_on_style)  # 绿色  

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
        self.sliders[6].setValue(0)
        self.link_value[6] = 0
        for i in [7,8]:
            pybullet.resetJointState(
            bodyUniqueId = self.robot_id,
            jointIndex = i,
            targetValue = 0,             # 目标角度（弧度）
            targetVelocity = 0.0                 # 目标速度
            )
            pybullet.setJointMotorControl2(
            bodyUniqueId = self.robot_id,
            jointIndex = i,
            controlMode = pybullet.POSITION_CONTROL,   # 位置控制
            targetPosition = self.link_value[6]*-0.0097,          # 目标角度（弧度）
            maxVelocity=100.0,
            force=500                          # 最大力矩
        )
        self.show_rebot.clear_trail()
        
            
            
    # 获取可用的串口列表
    def get_ports(self):
        self.serial_ports = serial.tools.list_ports.comports()
        try:
            self.pcan_ports = can.detect_available_configs(interfaces='pcan')
        except Exception as e:
            self.log_area.append(f"PCAN 扫描被策略拦截，跳过: {e}")
            self.pcan_ports = []
        if self.serial_ports or self.pcan_ports:    
            return True
        else:
            return False

    # 打印对应关节(连杆)名称
    def print_link(self):
        num_joints = pybullet.getNumJoints(self.robot_id)
        for j in range(num_joints):
            info = pybullet.getJointInfo(self.robot_id, j)
            # info 索引：
            # 0 jointIndex, 1 jointName, 2 jointType,
            # 8 lowerLimit, 9 upperLimit,
            # 10 maxForce, 11 maxVelocity,
            # 12 linkName, 16 axis
            joint_type = info[2]
            joint_name = info[1].decode("utf-8")
            link_name  = info[12].decode("utf-8")
            axis       = info[16]
            # type 信息：
            # 0为旋转关节 1为移动关节 4为固定关节

            if joint_type == pybullet.JOINT_REVOLUTE:
                print(f"[旋转] index={j:2d} name={joint_name:20s} "
                    f"axis={axis} range=[{info[8]:.2f}, {info[9]:.2f}]")
            else:
                print(f"[其他] index={j:2d} name={joint_name:20s} type={joint_type} range=[{info[8]:.2f}, {info[9]:.2f}]")


    def gravity_mode(self):
        if self.arm_thread :
            if self.gravity_mode_btn.text() == "开启重力补偿" and self.enable_change_mode:
                self.arm_thread.set_mode_signal.emit("gravity_mode_on")
                self.gravity_mode_btn.setText("关闭重力补偿")
                self.gravity_mode_btn.setStyleSheet(style.function_btn_off_style)  # 红色
            elif self.gravity_mode_btn.text() == "关闭重力补偿" :
                self.arm_thread.set_mode_signal.emit("gravity_mode_off")
                self.gravity_mode_btn.setText("开启重力补偿")
                self.gravity_mode_btn.setStyleSheet(style.function_btn_on_style)  
        return 

    def set_zero_mode(self):
        if self.arm_thread :
            if self.set_zero_mode_btn.text() == "设置机械臂零点" and self.enable_change_mode:
                self.arm_thread.set_mode_signal.emit("set_zero_mode_on")
                self.set_zero_mode_btn.setText("设置机械臂零点中")
                self.set_zero_mode_btn.setStyleSheet(style.function_btn_off_style)  # 红色
        return

    def record_mode(self):
        if self.arm_thread:
            if self.record_mode_btn.text() == "开启示教" and self.enable_change_mode:
                self.arm_thread.set_mode_signal.emit("record_mode_on")
                self.record_mode_btn.setText("关闭示教")
                self.record_mode_btn.setStyleSheet(style.function_btn_off_style)  # 红色
            elif self.record_mode_btn.text() == "关闭示教" :
                self.arm_thread.set_mode_signal.emit("record_mode_off")
                self.record_mode_btn.setText("开启示教")
                self.record_mode_btn.setStyleSheet(style.function_btn_on_style)  
        return

    def read_joint_mode(self):
        if self.arm_thread:
            if self.read_joint_mode_btn.text() == "开启实时读取关节角度" and self.enable_change_mode:
                self.arm_thread.set_mode_signal.emit("read_joint_mode_on")
                self.read_joint_mode_btn.setText("关闭实时读取关节角度")
                self.read_joint_mode_btn.setStyleSheet(style.function_btn_off_style)  # 红色
            elif self.read_joint_mode_btn.text() == "关闭实时读取关节角度" :
                self.arm_thread.set_mode_signal.emit("read_joint_mode_off")
                self.read_joint_mode_btn.setText("开启实时读取关节角度")
                self.read_joint_mode_btn.setStyleSheet(style.function_btn_on_style)  
        return



    # 连杆处理
    def on_slider_changed(self,idx,value):
        rad = value / 100
        self.values[idx].setText(f"{rad:.2f}rad")
        self.link_value[idx] = rad

    def on_slider_release(self,idx):
        if idx < 6:
            pybullet.setJointMotorControl2(
                bodyUniqueId = self.robot_id,
                jointIndex = idx,
                controlMode = pybullet.POSITION_CONTROL,   # 位置控制
                targetPosition = self.link_value[idx],          # 目标角度（弧度）
                maxVelocity=100.0,
                force=500                          # 最大力矩
            )
        elif idx == 6:
            pybullet.setJointMotorControl2(
                bodyUniqueId = self.robot_id,
                jointIndex = 7,
                controlMode = pybullet.POSITION_CONTROL,   # 位置控制
                targetPosition = self.link_value[6]*-0.0097,          
                maxVelocity=100.0,
                force=500                          # 最大力矩
            )
            pybullet.setJointMotorControl2(
                bodyUniqueId = self.robot_id,
                jointIndex = 8,
                controlMode = pybullet.POSITION_CONTROL,   # 位置控制
                targetPosition = self.link_value[6]*-0.0097,         # 单位m
                maxVelocity=100.0,
                force=500                          # 最大力矩
            )
            st = pybullet.getJointState(self.robot_id, 7)
            distance = st[0] 
            # print(f"distance:{distance}")
    
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
            self.btn_connect.setStyleSheet(style.arm_connect_off_style)  # 红色
            self.status_label.setText("已连接")
        elif msg == "连接失败" or msg == "断开连接":
            self.btn_connect.setText("连接")
            self.btn_connect.setStyleSheet(style.arm_connect_on_style)  # 绿色
            self.status_label.setText("已断开")
        self.log_area.append(msg)

    def on_pos_received(self,pos):
        for i in range(len(pos)):
            pos_int = int(pos[i]*100)
            self.sliders[i].setValue(pos_int)
            self.sliders[i].sliderReleased.emit()  

    def on_set_zero_is_done_received(self,is_set_zero_success):
        self.set_zero_mode_is_done = True
        self.set_zero_mode_btn.setText("设置机械臂零点")
        self.set_zero_mode_btn.setStyleSheet(style.function_btn_on_style) 
        if is_set_zero_success:
            self.log_area.append("设置零点成功。")
        else:
            self.log_area.append("设置零点失败，请重试。")
            

    def on_wait_change_mode(self,enable_change_mode):
        self.enable_change_mode = enable_change_mode
        # print(self.enable_change_mode)


    def update_sim(self):
        pybullet.stepSimulation()
        # self.print_link()
        num_joints = pybullet.getNumJoints(self.robot_id)
        joint_states = pybullet.getJointStates(self.robot_id, range(num_joints))
        # 过滤掉固定关节（JOINT_FIXED=4），只把可动关节传给 OpenGL 渲染器
        # 控制opengl渲染的机械臂
        self.joint_angles = []
        for j in range(num_joints):
            info = pybullet.getJointInfo(self.robot_id, j)
            if info[2] != 4:  # JOINT_FIXED = 4
                self.joint_angles.append(joint_states[j][0])
        self.show_rebot.set_joint_positions(self.joint_angles)

        # 显示碰撞
        if self.show_collision_on:
            self.update_show_collision()
        else:
            for link_name in self.robot_id_map.values():
                self.show_rebot.set_link_color(link_name, (0.627, 0.627, 0.627, 1))   
        # 显示末端轨迹
        if self.show_traj_on:
            self.show_rebot.set_trail_enabled(True)
            # self.show_rebot.set_target_position(0.3, 0.0, 0.2)
        else:
            self.show_rebot.clear_trail()
            self.show_rebot.set_trail_enabled(False)
            # self.show_rebot.clear_target()


            
    def update_show_collision(self):
        contacts = pybullet.getContactPoints(bodyA=self.robot_id, bodyB=self.robot_id)
        for i in range(pybullet.getNumJoints(self.robot_id)):
            self.show_rebot.set_link_color(self.robot_id_map[i], (0.627, 0.627, 0.627, 1))    

        if contacts:
            self.colliding.clear()
            for c in contacts:
                self.colliding.add(c[3])
                self.colliding.add(c[4])
                # print(f"collision group:{c[3]},{c[4]}")
            for i in range(pybullet.getNumJoints(self.robot_id)):
                if i in self.colliding:
                    # 碰撞的连杆红色
                    self.show_rebot.set_link_color(self.robot_id_map[i], (1, 0, 0, 1))    # 红色

    def center_on_screen(self):
        screen = QDesktopWidget().availableGeometry()
        x = (screen.width() - 800) // 2
        y = (screen.height() - 600) // 2
        self.move(x, y)

    def motor_page_init(self):
        # 电机控制页面初始化

        # 添加电机控制的控件和布局
        self.motor_top_layout = QHBoxLayout(self.page_motor)
        self.motor_top_layout.setContentsMargins(10, 10, 10, 10)

         # 顶部区域，包含设备下拉列表、扫描和连接按钮
        
        # 设备下拉列表
        self.motor_combo = QComboBox()
        self.motor_combo.addItems(["请选择设备"])
        self.motor_combo.setCurrentIndex(0)  # 默认选中第一项
        self.motor_top_layout.addWidget(self.motor_combo)

        # 扫描按钮
        self.motor_btn_scan = QPushButton("扫描端口")
        self.motor_btn_scan.clicked.connect(self.motor_on_scan)
        self.motor_btn_scan.setStyleSheet(style.scan_style)
        self.motor_top_layout.addWidget(self.motor_btn_scan)

        # 连接按钮
        self.motor_btn_connect = QPushButton("连接端口")
        self.motor_btn_connect.setStyleSheet(style.arm_connect_on_style)  # 绿色
        self.motor_btn_connect.clicked.connect(self.motor_on_connect)
        self.motor_top_layout.addWidget(self.motor_btn_connect)

        # 电机 ID 输入行
        self.motor_start_id_edit = QLineEdit()
        self.motor_start_id_edit.setPlaceholderText("起始ID")
        self.motor_start_id_edit.setClearButtonEnabled(True)
        self.motor_start_id_edit.setMaximumWidth(50)
        self.motor_top_layout.addWidget(self.motor_start_id_edit)

        self.motor_end_id_edit = QLineEdit()
        self.motor_end_id_edit.setPlaceholderText("结束ID")
        self.motor_end_id_edit.setClearButtonEnabled(True)
        self.motor_end_id_edit.setMaximumWidth(50)
        self.motor_top_layout.addWidget(self.motor_end_id_edit)

        self.btn_scan_motor = QPushButton("扫描电机")
        self.btn_scan_motor.clicked.connect(self.motor_on_scan_motor)
        self.motor_top_layout.addWidget(self.btn_scan_motor)

        self.motor_top_layout.addStretch()  
        self.page_motor_layout.addLayout(self.motor_top_layout)
        # 日志区
        self.motor_log_area = QTextEdit()
        self.motor_log_area.Layout = QVBoxLayout()
        self.motor_log_area.setPlaceholderText("日志信息...")
        self.motor_log_area.setFixedHeight(100)  
        self.page_motor_layout.addWidget(self.motor_log_area)

        # 电机相关区域
        self.motor_main_layout = QHBoxLayout()
        self.page_motor_layout.addLayout(self.motor_main_layout)

        # 电机卡片显示区域
        self.motor_show_card = QFrame()
        self.motor_show_card_layout = QVBoxLayout(self.motor_show_card)
        self.motor_show_card_layout.setContentsMargins(0, 0, 0, 0)
        self.motor_show_card_layout.setSpacing(10)

        self.canid=0x01
        self.masterid=0x10+0x01
        self.motor_btn_card = QPushButton(f"CAN ID: {self.canid}\nMASTER ID : {self.masterid}\nSTATE: disabled")
        self.motor_btn_card.setStyleSheet(style.motor_btn_card_enable_style)

        self.motor_show_card_layout.addWidget(self.motor_btn_card)
        self.motor_show_card_layout.addStretch()  # 弹性空间，将按钮推至顶部

        # 电机参数显示区域
        self.motor_show_param = QFrame()

        self.motor_main_layout.addWidget(self.motor_show_card,stretch=1)
        self.motor_main_layout.addWidget(self.motor_show_param,stretch=4)
        self.motor_show_card.setStyleSheet(style.motor_show_card_style)
        self.motor_show_param.setStyleSheet(style.motor_show_param_style)

    


    def motor_on_scan(self):
        """处理扫描按钮点击事件"""
        self.motor_log_area.clear()
        self.motor_combo.clear()
        self.status_label.setText("正在扫描设备...")
        if (self.get_ports()):
            self.motor_log_area.append("扫描完成,发现以下设备。")
        else:
            self.motor_log_area.append("扫描完成,未发现设备。")
        if platform == "windows":
            if self.serial_ports :
                for serial_ports in self.serial_ports:
                    self.motor_log_area.append(f"设备: {serial_ports.device}, 描述: {serial_ports.description}")
                    self.motor_combo.addItems([f"{serial_ports.device}"])

            if self.pcan_ports :
                for pcan_ports in self.pcan_ports:
                    pcan_name = pcan_ports['channel']
                    map_name = self.pcan_map[f"{pcan_name}"]
                    self.motor_log_area.append(f"设备:{map_name}, 描述: {pcan_name}")
                    self.motor_combo.addItems([f"{map_name}"])              
        
        self.status_label.setText("扫描完成")

    def motor_on_connect(self):
        selected = self.motor_combo.currentText()
        self.status_label.setText(f"正在连接 {selected} ...")

        if selected == "请选择设备" or selected == "":
            self.status_label.setText("请先选择一个有效设备")
            self.motor_log_area.append("请先选择一个有效设备。")
            return
        else:
            return 

    def motor_on_scan_motor(self):
        return 

#############################
#######选择机械臂对话框########
class rebotSelectDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("选择机械臂")
        self.setGeometry(100, 100, 800, 100)  # x, y, width, height
        self.setStyleSheet(style.dialog_style)
        self.setWindowIcon(QIcon(str(resource_path("ico/seeed_studio.ico"))))
        self.setModal(True)

        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("请选择机械臂类型"))

        self.combo = QComboBox()
        self.combo.addItems(["rebot-B601-DM", "rebot-B601-RS"])
        layout.addWidget(self.combo)

        btns = QDialogButtonBox(
            QDialogButtonBox.Ok | QDialogButtonBox.Cancel
        )
        ok_btn = btns.button(QDialogButtonBox.Ok)
        ok_btn.setStyleSheet(style.ok_btn_style)
        cancel_btn = btns.button(QDialogButtonBox.Cancel)
        cancel_btn.setStyleSheet(style.cancel_btn_style)
        
        btns.accepted.connect(self.accept)
        btns.rejected.connect(self.reject)
        layout.addWidget(btns)
        layout.addStretch()
        self.center_on_screen()

    def selected(self):
        return self.combo.currentText()       

    def center_on_screen(self):
        screen = QDesktopWidget().availableGeometry()
        x = (screen.width() - 800) // 2
        y = (screen.height() - 100) // 2
        self.move(x, y)
######################################
######################################

from PyQt5.QtWidgets import QSplashScreen
from PyQt5.QtGui import QPixmap
if __name__ == "__main__":
    app = QApplication(sys.argv)

    try:
        import pyi_splash
        pyi_splash.close()
    except ImportError:
        pass

    # splash = QSplashScreen(QPixmap(str(resource_path("ico/splash.png"))))
    # splash.setWindowFlags(
    # Qt.WindowStaysOnTopHint | Qt.FramelessWindowHint | Qt.SplashScreen
    # )
    # splash.show()

    app.processEvents()
    dlg = rebotSelectDialog()
    if dlg.exec_() != QDialog.Accepted:
        sys.exit(0)

    rebot_ver = dlg.selected()
    if rebot_ver == "rebot-B601-DM":  
        urdf = resource_path("urdf/reBot_B601_DM/urdf/reBot_B601_DM.urdf")
    elif rebot_ver == "rebot-B601-RS":
        urdf = resource_path("urdf/00-arm-rs_asm-v3/urdf/00-arm-rs_asm-v3.urdf")
    window = rebot_Simulation_App()
    # splash.close()
    window.show()
    sys.exit(app.exec_())
import numpy as np
import pybullet as p
from PyQt5.QtWidgets import QOpenGLWidget
from PyQt5.QtCore import Qt, QSize
from PyQt5.QtGui import QPainter, QImage

# 把 PyBullet 现有的场景渲染显示出来
class RobotRenderer(QOpenGLWidget):
    def __init__(self, physics_client, parent=None):
        super().__init__(parent)
        self.physics_client = physics_client
        self.setMinimumSize(QSize(640, 480))

        # 相机参数
        self.camera_distance = 1.3
        self.camera_yaw = 45.0
        self.camera_pitch = -40.0
        self.camera_target = [0, 0, 0]

        self.width = 40#640
        self.height = 30#480
        self.view_matrix = None
        self.projection_matrix = p.computeProjectionMatrixFOV(
            fov=60, aspect=1.0, nearVal=0.1, farVal=100.0
        )

    def resizeGL(self, w, h):
        self.width = w
        self.height = h
        aspect = w / h if h > 0 else 1.0
        self.projection_matrix = p.computeProjectionMatrixFOV(
            fov=60, aspect=aspect, nearVal=0.1, farVal=100.0
        )

    def paintGL(self):
        if self.physics_client is None:
            return

        self.view_matrix = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=self.camera_target,   #观察目标点
            distance=self.camera_distance, # 相机距离
            yaw=self.camera_yaw,
            pitch=self.camera_pitch,
            roll=0,  #不翻滚
            upAxisIndex=2 # Z 轴向上
        )

        #指定相机角度渲染场景
        #返回width, height, rgb, depth, segmentation
        _, _, rgb, _, _ = p.getCameraImage( 
            width=self.width,               
            height=self.height,
            viewMatrix=self.view_matrix,
            projectionMatrix=self.projection_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )
        rgb_array = np.reshape(rgb, (self.height, self.width, 4))

        painter = QPainter(self)
        painter.drawImage(0, 0, self._to_qimage(rgb_array))
        painter.end()

    def _to_qimage(self, array):  #numpy 转 Qimage
        h, w, _ = array.shape
        return QImage(array.data, w, h, 4 * w,
                      QImage.Format_RGBA8888).copy()

   
import numpy as np
import pybullet as p
from PyQt5.QtWidgets import QOpenGLWidget
from PyQt5.QtCore import pyqtSignal, QSize,QMutex,QThread
from PyQt5.QtGui import QPainter, QImage
import time

class RenderThread(QThread):
    frame_ready = pyqtSignal(object)   # numpy 数组

    def __init__(self, renderer, parent=None):
        super().__init__(parent)
        self.renderer = renderer
        self._is_running = True

    def run(self):
        while self._is_running:
            rgb_array = self.renderer.get_bullet_image()
            if rgb_array is not None:
                self.frame_ready.emit(rgb_array)
            self.msleep(10)   

    def stop(self):
        self._is_running = False
        self.wait()




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

        self.width = 40
        self.height = 30
        self.view_matrix = None
        self.projection_matrix = p.computeProjectionMatrixFOV(
            fov=60, aspect=1.0, nearVal=0.1, farVal=100.0
        )
        # 缓存最新一帧
        self._current_frame = None

        # 在这里创建并启动子线程
        self.render_thread = RenderThread(self)
        self.render_thread.frame_ready.connect(self._on_frame_ready)
        self.render_thread.start()


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
        if self._current_frame is None:
            return
        painter = QPainter(self)
        painter.drawImage(0, 0, self._to_qimage(self._current_frame))
        painter.end()

    def get_bullet_image(self):
        w = self.width      
        h = self.height      
        t0 = time.perf_counter()

        self.view_matrix = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=self.camera_target,   #观察目标点
        distance=self.camera_distance, # 相机距离
        yaw=self.camera_yaw,
        pitch=self.camera_pitch,
        roll=0,  #不翻滚
        upAxisIndex=2 # Z 轴向上
        )
        t1 = time.perf_counter()

        #指定相机角度渲染场景
        #返回width, height, rgb, depth, segmentation
        _, _, rgb, _, _ = p.getCameraImage( 
            width=w,               
            height=h,
            viewMatrix=self.view_matrix,
            projectionMatrix=self.projection_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL,
            flags=p.ER_NO_SEGMENTATION_MASK
        )
        t2 = time.perf_counter()
        rgb_array = np.reshape(rgb, (h, w, 4))
        t3 = time.perf_counter()
        print(f"矩阵: {(t1-t0)*1000:.1f}ms | 渲染: {(t2-t1)*1000:.1f}ms | reshape: {(t3-t2)*1000:.1f}ms")
        return rgb_array

    def _to_qimage(self, array):  #numpy 转 Qimage
        h, w, _ = array.shape
        return QImage(array.data, w, h, 4 * w,
                      QImage.Format_RGBA8888).copy()
    
    def _on_frame_ready(self, rgb_array):
        self._current_frame = rgb_array
        self.update()   # 触发 paintGL


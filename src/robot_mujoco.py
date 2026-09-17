# import mujoco
# from PyQt5.QtWidgets import QOpenGLWidget
# from PyQt5.QtGui import QIcon,QPainter, QImage
# model = mujoco.MjModel.from_xml_path(
#     r"D:\rebotArm-Assistant\rebot-Assistant\models\rebotarm_b601_colored.xml"
# )
# data = mujoco.MjData(model)
# renderer = mujoco.Renderer(model, height=480, width=640)
# mujoco.mj_forward(model, data)
# renderer.update_scene(data, camera="overhead_rgb")
# rgb = renderer.render()

# renderer.close()


# class MuJoCoWidget(QOpenGLWidget):
#     def __init__(self, parent=None):
#         super().__init__(parent)
#         self.model = mujoco.MjModel.from_xml_path(
#             "D:\\rebotArm-Assistant\\rebot-Assistant\\models\\rebotarm_b601_colored.xml")
#         self.data = mujoco.MjData(self.model)
#         # self.renderer = mujoco.Renderer(self.model, height=480, width=640)
#         mujoco.mj_forward(self.model, self.data)

#     def initializeGL(self):
#         self.renderer = mujoco.Renderer(self.model, height=480, width=640)

#     def paintEvent(self, event):
#         self.renderer.update_scene(self.data, camera="overhead_rgb")
#         rgb = self.renderer.render()
#         image = QImage(rgb.data, rgb.shape[1], rgb.shape[0], QImage.Format_RGB888)
#         painter = QPainter(self)
#         painter.drawImage(0, 0, image)
#         painter.end()


#     def request_repaint(self):
#         self.update()

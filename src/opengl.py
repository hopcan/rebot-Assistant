import numpy as np
import trimesh
from urdfpy import URDF
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtWidgets, QtCore

class URDFViewer(QtWidgets.QWidget):
    def __init__(self, urdf_path):
        super().__init__()
        self.layout = QtWidgets.QVBoxLayout(self)

        # 1. 创建 GLViewWidget
        self.view = gl.GLViewWidget()
        self.view.setCameraPosition(distance=2)  # 设置相机初始距离
        self.layout.addWidget(self.view)

        # 2. 解析 URDF
        try:
            self.robot = URDF.load(urdf_path)
        except Exception as e:
            print(f"加载URDF失败: {e}")
            return

        # 3. 为每个链接创建对应的 GLMeshItem
        self.link_items = []
        for link in self.robot.links:
            if link.visuals:
                # 假设每个链接只有一个视觉网格
                visual = link.visuals[0]
                mesh = visual.geometry.mesh

                if mesh is not None:
                    # 加载网格文件 (支持 STL, DAE, OBJ 等)
                    try:
                        tm_mesh = trimesh.load(mesh.filename)
                    except Exception as e:
                        print(f"加载网格 {mesh.filename} 失败: {e}")
                        continue

                    # 转换为 pyqtgraph 的 MeshData
                    md = gl.MeshData(vertexes=tm_mesh.vertices, faces=tm_mesh.faces)

                    # 创建 GLMeshItem
                    item = gl.GLMeshItem(
                        meshdata=md,
                        smooth=True,
                        color=(0.7, 0.7, 0.7, 1.0),
                        shader='shaded'
                    )
                    self.view.addItem(item)
                    self.link_items.append((link.name, item))

        # 4. 添加网格和坐标轴以便观察
        self.view.addItem(gl.GLGridItem())
        self.view.addItem(gl.GLAxisItem())
        self.view.show()

if __name__ == '__main__':
    app = QtWidgets.QApplication([])
    # 请替换为你的 URDF 文件路径
    viewer = URDFViewer("D:\rebotArm-Assistant\rebot-Assistant\urdf\reBot-DevArm_fixend_description\urdf\reBot-DevArm_fixend.urdf")
    viewer.resize(800, 600)
    viewer.show()
    app.exec_()
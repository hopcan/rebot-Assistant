import sys
import struct
from pathlib import Path

import numpy as np
import xml.etree.ElementTree as ET

from OpenGL.GL import *
from OpenGL.GLU import gluPerspective, gluLookAt
from PyQt5.QtCore import Qt, QPoint
from PyQt5.QtGui import QMouseEvent, QWheelEvent
from PyQt5.QtWidgets import (
    QOpenGLWidget, QApplication, QMainWindow,
    QVBoxLayout, QHBoxLayout, QSlider, QLabel, QWidget,
)


# ═══════════════════════════════════════════════════════════════
# 工具函数
# ═══════════════════════════════════════════════════════════════

def parse_binary_stl(filepath: str):
    """二进制 STL → (vertices[N,3], normals[N,3])  float32"""
    with open(filepath, "rb") as f:
        f.read(80)
        n_tri = struct.unpack("<I", f.read(4))[0]
        verts = np.empty((n_tri * 3, 3), dtype=np.float32)
        norms = np.empty((n_tri * 3, 3), dtype=np.float32)
        for i in range(n_tri):
            head = struct.unpack("<12f", f.read(48))
            f.read(2)
            n = head[:3]
            k = i * 3
            norms[k] = norms[k + 1] = norms[k + 2] = n
            verts[k] = head[3:6]
            verts[k + 1] = head[6:9]
            verts[k + 2] = head[9:12]
    return verts, norms


def _parse_origin(elem):
    if elem is None:
        return np.zeros(3), np.zeros(3)
    xyz = np.array([float(v) for v in elem.get("xyz", "0 0 0").split()])
    rpy = np.array([float(v) for v in elem.get("rpy", "0 0 0").split()])
    return xyz, rpy


def _rpy_to_mat(rpy):
    r, p, y = rpy
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def _origin_to_mat(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = _rpy_to_mat(rpy)
    T[:3, 3] = xyz
    return T


def _rotation_axis(axis, angle):
    ax = np.array(axis, dtype=np.float64)
    norm = np.linalg.norm(ax)
    if norm < 1e-10 or abs(angle) < 1e-10:
        return np.eye(4)
    ax /= norm
    c, s = np.cos(angle), np.sin(angle)
    v = 1 - c
    x, y, z = ax
    return np.array([
        [x * x * v + c,   x * y * v - z * s, x * z * v + y * s, 0],
        [y * x * v + z * s, y * y * v + c,   y * z * v - x * s, 0],
        [z * x * v - y * s, z * y * v + x * s, z * z * v + c,   0],
        [0, 0, 0, 1],
    ])


# ═══════════════════════════════════════════════════════════════
# URDF 连杆树
# ═══════════════════════════════════════════════════════════════

class LinkNode:
    __slots__ = (
        "name", "joint_origin", "joint_axis", "joint_limit",
        "visual_mesh", "visual_origin", "color",
        "vertices", "normals", "n_verts",
        "_vert_data", "_norm_data",
        "parent", "children", "is_fixed",
    )

    def __init__(self, name):
        self.name = name
        self.joint_origin = np.eye(4)
        self.joint_axis = np.array([0, 0, 1.0])
        self.joint_limit = (-3.14, 3.14)
        self.is_fixed = False
        self.visual_mesh = None
        self.visual_origin = np.eye(4)
        self.color = [0.7, 0.7, 0.7, 1.0]
        self.vertices = None
        self.normals = None
        self.n_verts = 0
        self.parent = None
        self.children = []


def parse_urdf(urdf_path: str) -> LinkNode:
    tree = ET.parse(urdf_path)
    robot = tree.getroot()
    urdf_dir = Path(urdf_path).parent
    pkg_dir = urdf_dir.parent

    def _resolve_mesh(filename: str) -> str | None:
        for base in (urdf_dir, pkg_dir):
            p = base / filename
            if p.exists():
                return str(p)
        return None

    # 收集所有 link
    links = {}
    for elem in robot:
        if elem.tag == "link":
            name = elem.get("name")
            node = LinkNode(name)
            vis = elem.find("visual")
            if vis is not None:
                xyz, rpy = _parse_origin(vis.find("origin"))
                node.visual_origin = _origin_to_mat(xyz, rpy)
                geom = vis.find("geometry")
                if geom is not None:
                    mesh = geom.find("mesh")
                    if mesh is not None:
                        node.visual_mesh = _resolve_mesh(mesh.get("filename"))
                mat = vis.find("material")
                if mat is not None:
                    col = mat.find("color")
                    if col is not None:
                        node.color = [float(v) for v in col.get("rgba", "0.7 0.7 0.7 1").split()]
            links[name] = node

    # 处理 joint
    for elem in robot:
        if elem.tag == "joint":
            parent = elem.find("parent")
            child = elem.find("child")
            if parent is not None and child is not None:
                p_name, c_name = parent.get("link"), child.get("link")
                if p_name in links and c_name in links:
                    cn = links[c_name]
                    cn.parent = p_name
                    xyz, rpy = _parse_origin(elem.find("origin"))
                    cn.joint_origin = _origin_to_mat(xyz, rpy)
                    axis_elem = elem.find("axis")
                    if axis_elem is not None:
                        cn.joint_axis = np.array([float(v) for v in axis_elem.get("xyz", "0 0 1").split()])
                    limit_elem = elem.find("limit")
                    if limit_elem is not None:
                        cn.joint_limit = (
                            float(limit_elem.get("lower", -3.14)),
                            float(limit_elem.get("upper", 3.14)),
                        )
                    if elem.get("type", "revolute") == "fixed":
                        cn.is_fixed = True

    # 建立 children
    for name, node in links.items():
        if node.parent and node.parent in links:
            links[node.parent].children.append(node)

    root = next((n for n in links.values() if n.parent is None), list(links.values())[0])

    # 加载 STL
    for node in links.values():
        if node.visual_mesh and Path(node.visual_mesh).exists():
            try:
                node.vertices, node.normals = parse_binary_stl(node.visual_mesh)
                node.n_verts = len(node.vertices)
            except Exception:
                node.vertices = node.normals = node.n_verts = None

    return root


def _collect_transforms(root: LinkNode, joint_angles: dict, T_parent=np.eye(4)):
    transforms = {root.name: T_parent}
    for child in root.children:
        if child.is_fixed:
            T_joint = np.eye(4)
        else:
            angle = joint_angles.get(child.name, 0.0)
            T_joint = _rotation_axis(child.joint_axis, angle)
        T_child = T_parent @ child.joint_origin @ T_joint
        transforms.update(_collect_transforms(child, joint_angles, T_child))
    return transforms


# ═══════════════════════════════════════════════════════════════
# 顶点数据准备
# ═══════════════════════════════════════════════════════════════

def _prepare_data(node):
    """标记有顶点数据可用"""
    if node.vertices is not None and node.n_verts > 0:
        node._vert_data = np.ascontiguousarray(node.vertices, dtype=np.float32)
        node._norm_data = np.ascontiguousarray(node.normals, dtype=np.float32)
    else:
        node._vert_data = node._norm_data = None
    for child in node.children:
        _prepare_data(child)


class GLURDFRenderer(QOpenGLWidget):
    def __init__(self, urdf_path, parent=None):
        super().__init__(parent)
        self.setMinimumSize(640, 480)

        self.root = parse_urdf(urdf_path)
        self.joint_angles = {f"link{i}": 0.0 for i in range(1, 7)}

        self.cam_dist = 1.5
        self.cam_yaw = 0.0
        self.cam_pitch = 50.0
        self.cam_target = np.array([0.0, 0.0, 0.3])
        self.last_mouse = QPoint()
        self.light_pos = [2.0, 2.0, 3.0, 1.0]
        self._data_ready = False
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.StrongFocus)

    # ── API ──
    def set_joint_angle(self, index: int, angle: float):
        if 0 <= index < 6:
            self.joint_angles[f"link{index + 1}"] = angle
            self.update()

    def set_joint_positions(self, positions):
        for i, v in enumerate(positions[:6]):
            self.joint_angles[f"link{i + 1}"] = v
        self.update()

    def set_link_color(self, link_name: str, rgba: tuple):
        """设置单个连杆颜色 (r, g, b, a)，例 set_link_color('link1', (1, 0.3, 0, 1))"""
        def _find(node, name):
            if node.name == name: return node
            for c in node.children:
                r = _find(c, name)
                if r: return r
            return None
        node = _find(self.root, link_name)
        if node:
            node.color = list(rgba)
            self.update()

    def set_link_colors(self, mapping: dict):
        """批量设置颜色，例 set_link_colors({'link1':(1,0,0,1), 'link2':(0,1,0,1), 'base_link':(0.5,0.5,0.5,1)})"""
        for name, rgba in mapping.items():
            self.set_link_color(name, rgba)

    # ── OpenGL ──
    def initializeGL(self):
        glClearColor(0.16, 0.17, 0.18, 1.0)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        glLightfv(GL_LIGHT0, GL_POSITION, self.light_pos)
        glLightfv(GL_LIGHT0, GL_DIFFUSE, [0.9, 0.9, 0.9, 1.0])
        glLightfv(GL_LIGHT0, GL_AMBIENT, [0.25, 0.25, 0.25, 1.0])
        glLightModelfv(GL_LIGHT_MODEL_AMBIENT, [0.2, 0.2, 0.2, 1.0])
        glShadeModel(GL_SMOOTH)
        glEnable(GL_NORMALIZE)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

        _prepare_data(self.root)
        self._data_ready = True

    def resizeGL(self, w, h):
        glViewport(0, 0, w, h)

    def paintGL(self):
        if not self._data_ready:
            return
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        # 投影
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        aspect = self.width() / max(self.height(), 1)
        gluPerspective(50, aspect, 0.01, 20.0)

        # 视图
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        cy, sy = np.cos(np.radians(self.cam_yaw)), np.sin(np.radians(self.cam_yaw))
        cp, sp = np.cos(np.radians(self.cam_pitch)), np.sin(np.radians(self.cam_pitch))
        # 球坐标轨道相机: pitch=0 水平, pitch=90 正上方
        eye = self.cam_target + self.cam_dist * np.array([cp * sy, cp * cy, sp])
        gluLookAt(*eye, *self.cam_target, 0, 0, 1)
        glLightfv(GL_LIGHT0, GL_POSITION, self.light_pos)

        # 地面
        self._draw_ground()

        # 机器人 — 客户端顶点数组
        transforms = _collect_transforms(self.root, self.joint_angles)
        glEnableClientState(GL_VERTEX_ARRAY)
        glEnableClientState(GL_NORMAL_ARRAY)
        self._draw_link_array(self.root, transforms)
        glDisableClientState(GL_NORMAL_ARRAY)
        glDisableClientState(GL_VERTEX_ARRAY)

    def _draw_ground(self):
        glDisable(GL_LIGHTING)
        glColor4f(0.3, 0.3, 0.35, 0.5)
        glBegin(GL_LINES)
        for i in range(-10, 11):
            glVertex3f(i * 0.1, -1.0, 0)
            glVertex3f(i * 0.1, 1.0, 0)
            glVertex3f(-1.0, i * 0.1, 0)
            glVertex3f(1.0, i * 0.1, 0)
        glEnd()
        glEnable(GL_LIGHTING)

    def _draw_link_array(self, node, transforms):
        if node._vert_data is not None and node.n_verts > 0:
            glPushMatrix()
            T = transforms.get(node.name, np.eye(4))
            glMultMatrixf(T.T.flatten())
            glMultMatrixf(node.visual_origin.T.flatten())

            r, g, b, a = node.color
            glColor4f(r, g, b, a)

            glVertexPointerf(node._vert_data)
            glNormalPointerf(node._norm_data)
            glDrawArrays(GL_TRIANGLES, 0, node.n_verts)

            glPopMatrix()

        for child in node.children:
            self._draw_link_array(child, transforms)

    # ── 鼠标交互 ──
    def mousePressEvent(self, e: QMouseEvent):
        self.last_mouse = e.pos()

    def mouseMoveEvent(self, e: QMouseEvent):
        dx = e.x() - self.last_mouse.x()
        dy = e.y() - self.last_mouse.y()
        if e.buttons() & Qt.LeftButton:
            self.cam_yaw += dx * 0.3
            self.cam_pitch -= dy * 0.3
            self.cam_pitch = max(-89, min(89, self.cam_pitch))
        self.last_mouse = e.pos()
        self.update()

    def wheelEvent(self, e: QWheelEvent):
        self.cam_dist -= e.angleDelta().y() * 0.001
        self.cam_dist = max(0.3, min(5.0, self.cam_dist))
        self.update()


# ═══════════════════════════════════════════════════════════════
# 测试窗口（直接运行本文件时启动）
# ═══════════════════════════════════════════════════════════════

if __name__ == "__main__":
    URDF_PATH = Path(__file__).resolve().parent.parent / \
        "urdf/reBot-DevArm_fixend_description/urdf/reBot-DevArm_fixend.urdf"

    app = QApplication(sys.argv)

    win = QMainWindow()
    win.setWindowTitle("URDF OpenGL Renderer")
    win.resize(900, 700)

    central = QWidget()
    win.setCentralWidget(central)
    hbox = QHBoxLayout(central)

    viewer = GLURDFRenderer(str(URDF_PATH))
    hbox.addWidget(viewer, 4)

    slider_box = QVBoxLayout()
    slider_box.addWidget(QLabel("关节控制"))
    for i, name in enumerate(["关节1", "关节2", "关节3", "关节4", "关节5", "关节6"]):
        row = QHBoxLayout()
        lb = QLabel(name)
        lb.setFixedWidth(40)
        val_lb = QLabel("0.00")
        val_lb.setFixedWidth(50)
        s = QSlider(Qt.Horizontal)
        s.setRange(-314, 314)
        s.setValue(0)
        s.valueChanged.connect(lambda v, idx=i, vl=val_lb: (
            vl.setText(f"{v / 100:.2f}"),
            viewer.set_joint_angle(idx, v / 100),
        ))
        row.addWidget(lb)
        row.addWidget(s)
        row.addWidget(val_lb)
        slider_box.addLayout(row)
    slider_box.addStretch()
    hbox.addLayout(slider_box, 1)

    win.show()
    sys.exit(app.exec_())
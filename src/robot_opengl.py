import sys
import struct
from pathlib import Path

import numpy as np
import xml.etree.ElementTree as ET

from OpenGL.GL import *
from OpenGL.GLU import gluPerspective, gluLookAt, gluNewQuadric, gluSphere, gluDeleteQuadric
from PyQt5.QtCore import Qt, QPoint
from PyQt5.QtGui import (
    QMouseEvent, QWheelEvent,
    QColor, QFont, QFontMetrics, QImage, QPainter,
)
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


def _mat_to_rpy(R):
    """_rpy_to_mat 的逆：把旋转矩阵转回 roll/pitch/yaw（弧度），约定 R = Rz(y)·Ry(p)·Rx(r)"""
    sp = max(-1.0, min(1.0, -R[2, 0]))
    p = np.arcsin(sp)
    cp = np.cos(p)
    if cp > 1e-8:
        r = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(R[1, 0], R[0, 0])
    else:
        # 万向节锁（pitch≈±90°）：roll/yaw 退化，取一个等价解
        r = np.arctan2(-R[1, 2], R[1, 1])
        y = 0.0
    return np.array([r, p, y])


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


def _tint_colors(colors, factor=0.5):
    """把顶点颜色向红色偏：压暗绿/蓝通道，保留亮度结构与 alpha（用于碰撞红晕）。"""
    out = colors.copy()
    out[:, 1] *= factor  # G
    out[:, 2] *= factor  # B
    return out


# ═══════════════════════════════════════════════════════════════
# URDF 连杆树
# ═══════════════════════════════════════════════════════════════

class LinkNode:
    __slots__ = (
        "name", "joint_origin", "joint_axis", "joint_limit", "joint_type",
        "visual_mesh", "visual_origin", "color",
        "vertices", "normals", "n_verts",
        "_vert_data", "_norm_data", "_vert_colors", "_col_data", "_visuals",
        "parent", "children", "is_fixed",
        "mimic_link", "mimic_mult", "mimic_offset", "highlight",
        "tint", "_tint_col_data", "_tint_key",
    )

    def __init__(self, name):
        self.name = name
        self.joint_origin = np.eye(4)
        self.joint_axis = np.array([0, 0, 1.0])
        self.joint_limit = (-3.14, 3.14)
        self.joint_type = "revolute"
        self.is_fixed = False
        self.visual_mesh = None
        self.visual_origin = np.eye(4)
        self.color = [0.7, 0.7, 0.7, 1.0]
        self.vertices = None
        self.normals = None
        self.n_verts = 0
        self.parent = None
        self.children = []
        self.mimic_link = None
        self.mimic_mult = 1.0
        self.mimic_offset = 0.0
        self.highlight = None
        self.tint = None
        self._tint_col_data = None
        self._tint_key = None


def parse_urdf(urdf_path: str) -> LinkNode:
    tree = ET.parse(urdf_path)
    robot = tree.getroot()
    urdf_dir = Path(urdf_path).parent
    pkg_dir = urdf_dir.parent

    # 收集顶层命名材质定义（<material name="X"><color rgba=.../></material>）
    materials = {}
    for elem in robot:
        if elem.tag == "material":
            mname = elem.get("name")
            col = elem.find("color")
            if mname and col is not None:
                materials[mname] = [float(v) for v in col.get("rgba", "0.7 0.7 0.7 1").split()]

    def _material_color(mat_elem, default):
        if mat_elem is None:
            return default
        col = mat_elem.find("color")
        if col is not None:
            return [float(v) for v in col.get("rgba", "0.7 0.7 0.7 1").split()]
        mname = mat_elem.get("name")
        if mname and mname in materials:
            return materials[mname]
        return default

    def _resolve_mesh(filename: str) -> str | None:
        if filename and filename.startswith("package://"):
            # package://<pkg>/<rest> → 取 "description" 之后的部分，相对本工程 description 目录解析
            parts = filename.split("/", 2)[-1].split("/")
            if "description" in parts:
                sub = "/".join(parts[parts.index("description") + 1:])
                p = pkg_dir / sub
                if p.exists():
                    return str(p)
            return None
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
            # 收集所有 <visual> 元素（一个 link 可能有多个 visual）
            visuals = elem.findall("visual")
            if not visuals:
                links[name] = node
                continue
            # 用第一个 visual 的颜色作为 link 默认色
            node.color = _material_color(visuals[0].find("material"), [0.7, 0.7, 0.7, 1.0])
            node._visuals = []  # 存储 (mesh_path, xyz, rpy, color)
            for vis in visuals:
                xyz, rpy = _parse_origin(vis.find("origin"))
                geom = vis.find("geometry")
                if geom is not None:
                    mesh = geom.find("mesh")
                    if mesh is not None:
                        mp = _resolve_mesh(mesh.get("filename"))
                        vis_color = _material_color(vis.find("material"), node.color)
                        if mp:
                            node._visuals.append((mp, xyz, rpy, vis_color))
            links[name] = node

    # 处理 joint
    # 先建立 joint 名 → 子连杆名 映射（用于 mimic 关节取值）
    joint_to_link = {}
    for elem in robot:
        if elem.tag == "joint":
            child = elem.find("child")
            if child is not None:
                joint_to_link[elem.get("name")] = child.get("link")

    for elem in robot:
        if elem.tag == "joint":
            parent = elem.find("parent")
            child = elem.find("child")
            if parent is not None and child is not None:
                p_name, c_name = parent.get("link"), child.get("link")
                if p_name in links and c_name in links:
                    cn = links[c_name]
                    cn.parent = p_name
                    joint_type = elem.get("type", "revolute")
                    cn.joint_type = joint_type
                    if joint_type == "fixed":
                        cn.is_fixed = True
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
                    mimic_elem = elem.find("mimic")
                    if mimic_elem is not None:
                        cn.mimic_link = joint_to_link.get(mimic_elem.get("joint"))
                        cn.mimic_mult = float(mimic_elem.get("multiplier", "1"))
                        cn.mimic_offset = float(mimic_elem.get("offset", "0"))

    # 建立 children
    for name, node in links.items():
        if node.parent and node.parent in links:
            links[node.parent].children.append(node)

    root = next((n for n in links.values() if n.parent is None), list(links.values())[0])

    # 加载 STL（支持多 visual 合并）
    for node in links.values():
        visuals = getattr(node, '_visuals', None)
        if not visuals:
            # 兼容旧的单 visual 格式
            if node.visual_mesh and Path(node.visual_mesh).exists():
                try:
                    node.vertices, node.normals = parse_binary_stl(node.visual_mesh)
                    node.n_verts = len(node.vertices)
                except Exception:
                    node.vertices = node.normals = node.n_verts = None
            continue
        # 合并多个 visual 的 mesh
        all_verts = []
        all_norms = []
        all_colors = []
        for mesh_path, xyz, rpy, vis_color in visuals:
            if not Path(mesh_path).exists():
                continue
            try:
                v, n = parse_binary_stl(mesh_path)
                T = _origin_to_mat(xyz, rpy)
                R = T[:3, :3]
                t = T[:3, 3]
                # 旋转顶点
                v = (R @ v.T).T + t
                # 旋转法线
                n = (R @ n.T).T
                all_verts.append(v)
                all_norms.append(n)
                # 为每个顶点记录颜色
                all_colors.append(np.tile(vis_color, (len(v), 1)))
            except Exception:
                pass
        if all_verts:
            node.vertices = np.vstack(all_verts).astype(np.float32)
            node.normals = np.vstack(all_norms).astype(np.float32)
            node.n_verts = len(node.vertices)
            # 将颜色数据也存储到节点上
            node._vert_colors = np.vstack(all_colors).astype(np.float32)
            node.visual_origin = np.eye(4)  # 已经烘焙到顶点里了
        else:
            node.vertices = node.normals = node.n_verts = node._vert_colors = None

    return root


def _collect_transforms(root: LinkNode, joint_angles: dict, T_parent=np.eye(4)):
    transforms = {root.name: T_parent}
    for child in root.children:
        if child.is_fixed:
            T_joint = np.eye(4)
        else:
            # 关节值：带 mimic 的关节用源关节值推导，否则取自身
            if getattr(child, "mimic_link", None):
                src = joint_angles.get(child.mimic_link, 0.0)
                q = child.mimic_mult * src + child.mimic_offset
            else:
                q = joint_angles.get(child.name, 0.0)
            if child.joint_type == "prismatic":
                # 滑动关节：沿 joint_axis 平移
                T_joint = np.eye(4)
                ax = np.array(child.joint_axis, dtype=np.float64)
                norm = np.linalg.norm(ax)
                if norm > 1e-10:
                    T_joint[:3, 3] = (ax / norm) * q
            else:
                # 旋转关节
                T_joint = _rotation_axis(child.joint_axis, q)
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
        if hasattr(node, '_vert_colors') and node._vert_colors is not None:
            node._col_data = np.ascontiguousarray(node._vert_colors, dtype=np.float32)
        else:
            node._col_data = None
    else:
        node._vert_data = node._norm_data = None
    for child in node.children:
        _prepare_data(child)


class GLURDFRenderer(QOpenGLWidget):
    def __init__(self, urdf_path, parent=None):
        super().__init__(parent)
        self.setMinimumSize(640, 480)

        self.root = parse_urdf(urdf_path)
        # 自动收集所有非固定关节名
        def _collect_joints(node):
            names = {}
            for child in node.children:
                if not child.is_fixed:
                    names[child.name] = 0.0
                names.update(_collect_joints(child))
            return names
        self.joint_angles = _collect_joints(self.root)
        self._joint_order = list(self.joint_angles.keys())

        self.cam_dist = 1.5
        self.cam_yaw = 0.0
        self.cam_pitch = 50.0
        self.cam_target = np.array([0.0, 0.0, 0.3])
        self.last_mouse = QPoint()
        self.light_pos = [2.0, 2.0, 3.0, 1.0]
        self._data_ready = False
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.StrongFocus)

        # 末端轨迹
        self.trail_points = []       # [(x,y,z), ...]
        self.trail_max = 3000
        self.trail_enabled = True
        self.trail_color = (1.0, 0.3, 0.0, 1.0)  # 橙色
        self._end_effector = None    # None = 自动找最后一个末端连杆

        # 轨迹绘制在两个连杆的中点（如两个夹爪之间）
        self._end_midpoint = None  # (link_a, link_b)
        self._tip_offset = np.zeros(3)  # 从末端连杆原点到指尖的偏移（连杆局部坐标系）

        # 目标位置小球
        self._target_pos = None        # (x, y, z) or None（不显示）
        self._target_color = (1.0, 0.2, 0.2, 0.9)  # 红色

        # 右上角末端位置数据框
        self._hud_enabled = True

    # ── API ──
    def set_joint_angle(self, index: int, angle: float):
        if index < len(self._joint_order):
            self.joint_angles[self._joint_order[index]] = angle
            self.update()

    def set_joint_positions(self, positions):
        for i in range(min(len(positions), len(self._joint_order))):
            self.joint_angles[self._joint_order[i]] = positions[i]
        self.update()

    def set_link_color(self, link_name: str, rgba):
        """设置/清除单个连杆高亮色。rgba 传 None 恢复自然色（多彩网格），否则覆盖为单色。
        例 set_link_color('link1', (1, 0.3, 0, 1))"""
        def _find(node, name):
            if node.name == name: return node
            for c in node.children:
                r = _find(c, name)
                if r: return r
            return None
        node = _find(self.root, link_name)
        if node:
            node.highlight = tuple(rgba) if rgba is not None else None
            self.update()

    def set_link_colors(self, mapping: dict):
        """批量设置颜色，例 set_link_colors({'link1':(1,0,0,1), 'link2':(0,1,0,1), 'base_link':(0.5,0.5,0.5,1)})"""
        for name, rgba in mapping.items():
            self.set_link_color(name, rgba)

    def set_link_tint(self, link_name: str, rgba):
        """给连杆叠加一层偏红晕色，并保留原色层次。rgba 传 None 恢复自然色。
        例 set_link_tint('link2', (1, 0, 0, 1))"""
        def _find(node, name):
            if node.name == name: return node
            for c in node.children:
                r = _find(c, name)
                if r: return r
            return None
        node = _find(self.root, link_name)
        if not node:
            return
        if rgba is None:
            node.tint = None
        else:
            key = tuple(rgba)
            node.tint = tuple(rgba)
            if getattr(node, "_tint_key", None) != key:
                base = getattr(node, "_vert_colors", None)
                node._tint_col_data = (
                    np.ascontiguousarray(_tint_colors(base), dtype=np.float32)
                    if base is not None else None
                )
                node._tint_key = key
        self.update()

    def clear_trail(self):
        """清除末端轨迹"""
        self.trail_points.clear()
        self.update()

    def set_trail_enabled(self, enabled: bool):
        """启用/禁用轨迹绘制"""
        self.trail_enabled = enabled
        self.update()

    def set_trail_color(self, rgba: tuple):
        """设置轨迹颜色，例 set_trail_color((0, 1, 0, 1))"""
        self.trail_color = tuple(rgba)
        self.update()

    def set_end_effector(self, link_name: str):
        """手动指定末端连杆名，例 set_end_effector('link6')"""
        self._end_effector = link_name
        self._end_midpoint = None

    def set_end_effector_midpoint(self, link_a: str, link_b: str, tip_offset=(0, 0, 0)):
        """轨迹取两个连杆指尖的中点。tip_offset 为从连杆原点到指尖的局部坐标偏移"""
        self._end_midpoint = (link_a, link_b)
        self._tip_offset = np.array(tip_offset, dtype=np.float64)
        self._end_effector = None

    def set_target_position(self, x, y, z):
        """设置末端目标位置，显示为红色小球"""
        self._target_pos = (float(x), float(y), float(z))
        self.update()

    def clear_target(self):
        """清除目标位置小球"""
        self._target_pos = None
        self.update()

    def set_hud_enabled(self, enabled: bool):
        """显示/隐藏右上角末端位置数据框"""
        self._hud_enabled = enabled
        self.update()

    def _get_end_link_name(self):
        """自动找到末端连杆名（树中最深的叶子节点）"""
        if self._end_effector:
            return self._end_effector
        def _deepest(node, depth=0):
            if not node.children:
                return (node.name, depth)
            best = (node.name, depth)
            for c in node.children:
                cand = _deepest(c, depth + 1)
                if cand[1] > best[1]:
                    best = cand
            return best
        return _deepest(self.root)[0]

    # ── OpenGL ──
    def initializeGL(self):
        glClearColor(0.9, 0.9, 0.9, 1.0)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        glLightfv(GL_LIGHT0, GL_POSITION, self.light_pos)
        # 漫反射，主光强度
        glLightfv(GL_LIGHT0, GL_DIFFUSE, [1.0, 1.0, 1.0, 1.0])
        # 环境光
        glLightfv(GL_LIGHT0, GL_AMBIENT, [0.55, 0.55, 0.55, 1.0])
        glLightModelfv(GL_LIGHT_MODEL_AMBIENT, [0.1, 0.1, 0.1, 1.0])
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

        # 末端轨迹
        if self._end_midpoint:
            T_a = transforms.get(self._end_midpoint[0], np.eye(4))
            T_b = transforms.get(self._end_midpoint[1], np.eye(4))
            # 指尖 = 连杆原点 + 旋转后的局部偏移
            tip_a = T_a[:3, 3] + T_a[:3, :3] @ self._tip_offset
            tip_b = T_b[:3, 3] + T_b[:3, :3] @ self._tip_offset
            pos = (tip_a + tip_b) / 2.0
            rot = T_a[:3, :3].copy()
        else:
            end_name = self._get_end_link_name()
            T_end = transforms.get(end_name, np.eye(4))
            pos = T_end[:3, 3].copy()
            rot = T_end[:3, :3].copy()
        if self.trail_enabled and len(self.trail_points) < self.trail_max:
            if not self.trail_points or np.linalg.norm(pos - self.trail_points[-1]) > 1e-5:
                self.trail_points.append(pos)
        if self.trail_points:
            self._draw_trail()

        # 目标位置红色小球
        if self._target_pos is not None:
            self._draw_target_sphere()

        # 右上角末端位姿数据框（屏幕空间 HUD）
        if self._hud_enabled:
            self._draw_hud(pos, _mat_to_rpy(rot))

    def _draw_ground(self):
        glDisable(GL_LIGHTING)
        glColor4f(0.72, 0.72, 0.75, 0.6)
        glBegin(GL_LINES)
        for i in range(-10, 11):
            glVertex3f(i * 0.1, -1.0, 0)
            glVertex3f(i * 0.1, 1.0, 0)
            glVertex3f(-1.0, i * 0.1, 0)
            glVertex3f(1.0, i * 0.1, 0)
        glEnd()
        glEnable(GL_LIGHTING)

    def _draw_trail(self):
        """绘制末端轨迹（不受光照影响）"""
        glDisable(GL_LIGHTING)
        glLineWidth(2.0)
        glColor4f(*self.trail_color)
        pts = self.trail_points
        glBegin(GL_LINE_STRIP)
        for p in pts:
            glVertex3f(*p)
        glEnd()
        glPointSize(4.0)
        glBegin(GL_POINTS)
        for p in pts:
            glVertex3f(*p)
        glEnd()
        glEnable(GL_LIGHTING)

    def _draw_target_sphere(self):
        """在目标位置绘制红色半透明小球"""
        glDisable(GL_LIGHTING)
        glColor4f(*self._target_color)
        glPushMatrix()
        glTranslatef(*self._target_pos)
        quad = gluNewQuadric()
        gluSphere(quad, 0.015, 16, 16)
        gluDeleteQuadric(quad)
        glPopMatrix()
        # 十字标记，更显眼
        s = 0.03
        glLineWidth(2.0)
        glBegin(GL_LINES)
        glVertex3f(-s, 0, 0); glVertex3f(s, 0, 0)
        glVertex3f(0, -s, 0); glVertex3f(0, s, 0)
        glVertex3f(0, 0, -s); glVertex3f(0, 0, s)
        glEnd()
        glEnable(GL_LIGHTING)

    def _draw_hud(self, pos, rpy):
        """右上角半透明数据框：尺寸随窗口缩放；行距/宽度按字体实测，防止重叠或出界。"""
        w, h = self.width(), self.height()
        if w < 8 or h < 8:
            return

        r, p, yaw = np.degrees(rpy)
        title = "末端位姿 (end pose)"
        rows = [
            ("X:  %.4f m" % pos[0], (0.80, 0.42, 0.14, 1.0)),
            ("Y:  %.4f m" % pos[1], (0.80, 0.42, 0.14, 1.0)),
            ("Z:  %.4f m" % pos[2], (0.80, 0.42, 0.14, 1.0)),
            ("Roll:  %.1f°" % r,   (0.10, 0.42, 0.33, 1.0)),
            ("Pitch: %.1f°" % p,   (0.10, 0.42, 0.33, 1.0)),
            ("Yaw:   %.1f°" % yaw, (0.10, 0.42, 0.33, 1.0)),
        ]

        def measure(scale):
            """按缩放系数算出字号与框尺寸；行距用字体实测高度，保证贴图不叠。"""
            px_t = max(6, int(round(13 * scale)))
            px_v = max(6, int(round(14 * scale)))
            f_t = QFont(); f_t.setPointSize(px_t); fm_t = QFontMetrics(f_t)
            f_v = QFont(); f_v.setPointSize(px_v); fm_v = QFontMetrics(f_v)
            gap = max(6, int(6 * scale))
            line_h = fm_v.height() + gap
            title_h = fm_t.height() + gap
            pad_x, pad_y = 10 * scale, 8 * scale
            content_w = max([fm_v.horizontalAdvance(t) for t, _ in rows]
                            + [fm_t.horizontalAdvance(title)])
            return dict(px_t=px_t, px_v=px_v, line_h=line_h, title_h=title_h,
                        pad_x=pad_x, pad_y=pad_y,
                        box_w=content_w + 2 * pad_x,
                        box_h=pad_y + title_h + len(rows) * line_h + pad_y)

        # ① 以 1200×1000 为基准随窗口等比缩放，限制在 [0.8, 1.6]
        scale = max(0.8, min(1.6, min(w / 1200.0, h / 1000.0)))
        L = measure(scale)
        margin = 12 * scale

        # ② 窗口装不下时再等比缩小，保证整框+边距一定放得下
        fit = min(1.0,
                  (w - 2 * margin) / max(L["box_w"], 1.0),
                  (h - 2 * margin) / max(L["box_h"], 1.0))
        if fit < 1.0:
            scale *= fit
            L = measure(scale)
            margin = 12 * scale

        box_w, box_h = L["box_w"], L["box_h"]
        x0 = max(margin, w - box_w - margin)      # 右上角
        y0 = margin
        if y0 + box_h > h:                         # 高度不够时上移夹紧
            y0 = max(margin, h - box_h - margin)

        # 切到屏幕空间：原点在左上角，y 向下
        glMatrixMode(GL_PROJECTION)
        glPushMatrix()
        glLoadIdentity()
        glOrtho(0, w, h, 0, -1, 1)
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        glLoadIdentity()

        glDisable(GL_DEPTH_TEST)
        glDisable(GL_LIGHTING)
        glEnable(GL_BLEND)

        # 底框（背景色，白底）
        glColor4f(1.0, 1.0, 1.0, 1.0)
        glBegin(GL_QUADS)
        glVertex2f(x0, y0); glVertex2f(x0 + box_w, y0)
        glVertex2f(x0 + box_w, y0 + box_h); glVertex2f(x0, y0 + box_h)
        glEnd()

        # 边框
        glColor4f(0.52, 0.52, 0.52, 1.0)
        glLineWidth(1.5)
        glBegin(GL_LINE_LOOP)
        glVertex2f(x0, y0); glVertex2f(x0 + box_w, y0)
        glVertex2f(x0 + box_w, y0 + box_h); glVertex2f(x0, y0 + box_h)
        glEnd()

        # 文字：逐行按实测行高排布，绝不重叠
        ty = y0 + L["pad_y"]
        self._draw_text(x0 + L["pad_x"], ty, title, (0.25, 0.30, 0.42, 1.0), L["px_t"])
        ty += L["title_h"]
        for text, color in rows:
            self._draw_text(x0 + L["pad_x"], ty, text, color, L["px_v"])
            ty += L["line_h"]

        # 恢复 3D 状态
        glMatrixMode(GL_PROJECTION)
        glPopMatrix()
        glMatrixMode(GL_MODELVIEW)
        glPopMatrix()
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)

    def _draw_text(self, x, y, text, color=(1, 1, 1, 1), px=14):
        """QPainter 栅格化文字 → GL 纹理 → 屏幕空间贴图（原始 OpenGL 不能直接画字）"""
        cache = getattr(self, "_text_cache", None)
        if cache is None:
            cache = self._text_cache = {}
        key = (text, px)
        if key not in cache:
            font = QFont()               # 应用默认字体，和按钮一致
            font.setPointSize(px)
            fm = QFontMetrics(font)
            img = QImage(fm.horizontalAdvance(text), fm.height() + 4, QImage.Format_ARGB32)
            img.fill(Qt.transparent)
            p = QPainter(img)
            p.setFont(font)
            p.setPen(QColor(255, 255, 255, 255))
            p.drawText(0, fm.ascent(), text)
            p.end()
            img = img.convertToFormat(QImage.Format_RGBA8888)
            tw, th = img.width(), img.height()
            ptr = img.constBits()
            ptr.setsize(img.byteCount())
            buf = bytes(ptr)
            tex = glGenTextures(1)
            glBindTexture(GL_TEXTURE_2D, tex)
            glPixelStorei(GL_UNPACK_ALIGNMENT, 4)
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, tw, th, 0, GL_RGBA, GL_UNSIGNED_BYTE, buf)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
            cache[key] = (tex, tw, th)

        tex, tw, th = cache[key]
        glEnable(GL_TEXTURE_2D)
        glBindTexture(GL_TEXTURE_2D, tex)
        glColor4f(*color)
        glBegin(GL_QUADS)
        glTexCoord2f(0, 0); glVertex2f(x, y)
        glTexCoord2f(1, 0); glVertex2f(x + tw, y)
        glTexCoord2f(1, 1); glVertex2f(x + tw, y + th)
        glTexCoord2f(0, 1); glVertex2f(x, y + th)
        glEnd()
        glDisable(GL_TEXTURE_2D)

    def _draw_link_array(self, node, transforms):
        if node._vert_data is not None and node.n_verts > 0:
            glPushMatrix()
            T = transforms.get(node.name, np.eye(4))
            glMultMatrixf(T.T.flatten())
            glMultMatrixf(node.visual_origin.T.flatten())

            glVertexPointerf(node._vert_data)
            glNormalPointerf(node._norm_data)

            highlight = getattr(node, "highlight", None)
            tint = getattr(node, "tint", None)

            use_col_array = False
            color_data = node._col_data
            if tint is not None and node._tint_col_data is not None:
                # 碰撞红晕：用叠加(偏红)后的顶点色，保留原色层次
                use_col_array = True
                color_data = node._tint_col_data
            elif highlight is None:
                use_col_array = node._col_data is not None

            if use_col_array:
                glEnableClientState(GL_COLOR_ARRAY)
                glColorPointerf(color_data)
            elif highlight is not None:
                glColor4f(*highlight)
            else:
                glColor4f(*node.color)

            glDrawArrays(GL_TRIANGLES, 0, node.n_verts)

            if use_col_array:
                glDisableClientState(GL_COLOR_ARRAY)

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
    for idx, name in enumerate(viewer._joint_order):
        row = QHBoxLayout()
        lb = QLabel(name[:6])
        lb.setFixedWidth(60)
        val_lb = QLabel("0.00")
        val_lb.setFixedWidth(50)
        s = QSlider(Qt.Horizontal)
        s.setRange(-314, 314)
        s.setValue(0)
        s.valueChanged.connect(lambda v, i=idx, vl=val_lb: (
            vl.setText(f"{v / 100:.2f}"),
            viewer.set_joint_angle(i, v / 100),
        ))
        row.addWidget(lb)
        row.addWidget(s)
        row.addWidget(val_lb)
        slider_box.addLayout(row)
    slider_box.addStretch()
    hbox.addLayout(slider_box, 1)

    win.show()
    sys.exit(app.exec_())
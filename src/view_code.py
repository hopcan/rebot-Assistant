import sys
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QListWidget, QListWidgetItem, QGraphicsView,
                             QGraphicsScene, QPlainTextEdit, QPushButton,
                             QSplitter, QLabel, QGraphicsItem, QMenu, QDialog,
                             QFormLayout, QLineEdit, QDialogButtonBox)
from PyQt5.QtCore import Qt, QRectF, QPointF, QLineF, QTimer
from PyQt5.QtGui import (QPainter, QBrush, QColor, QPen, QFont, QPainterPath)


# ============ 1. 积木定义（加新积木只改这里） ============
BLOCK_DEFS = {
    "print":  {"label": "输出",       "params": ["内容"],       "color": "#2196F3",
               "code": 'print("{0}")'},
    "assign": {"label": "变量赋值",   "params": ["变量", "值"], "color": "#FF9800",
               "code": '{0} = {1}'},
    "for":    {"label": "重复",       "params": ["次数"],       "color": "#9C27B0",
               "code": 'for _ in range({0}):', "has_body": True},
    "while":  {"label": "当条件成立", "params": ["条件"],       "color": "#009688",
               "code": 'while {0}:', "has_body": True},
    "if":     {"label": "如果",       "params": ["条件"],       "color": "#F44336",
               "code": 'if {0}:', "has_body": True},
    "wait":   {"label": "等待",       "params": ["秒"],         "color": "#607D8B",
               "code": 'time.sleep({0})'},
}

BLOCK_H = 40
SLOT_LEFT = 26
FOOTER_H = 16
MIN_W = 210
MIN_SLOT_H = 22


# ============ 2. 参数对话框 ============
class ParamDialog(QDialog):
    def __init__(self, defn, params, parent=None):
        super().__init__(parent)
        self.setWindowTitle("编辑参数")
        layout = QFormLayout(self)
        self.edits = []
        for name, val in zip(defn["params"], params):
            e = QLineEdit(val)
            layout.addRow(name + ":", e)
            self.edits.append(e)
        btns = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        btns.accepted.connect(self.accept)
        btns.rejected.connect(self.reject)
        layout.addRow(btns)

    def values(self):
        return [e.text() for e in self.edits]


# ============ 3. 积木图元 ============
class BlockItem(QGraphicsItem):
    def __init__(self, kind, params=None):
        super().__init__()
        self.kind = kind
        self.defn = BLOCK_DEFS[kind]
        self.params = list(params) if params else [""] * len(self.defn["params"])
        self.children = []
        self.setFlags(QGraphicsItem.ItemIsMovable |
                      QGraphicsItem.ItemIsSelectable |
                      QGraphicsItem.ItemSendsGeometryChanges)
        self.setZValue(1)

    # ---------- 几何 ----------
    @property
    def has_body(self):
        return self.defn.get("has_body", False)

    def header_h(self):
        return BLOCK_H

    def footer_h(self):
        return FOOTER_H if self.has_body else 0

    def body_h(self):
        h = sum(c.total_height() for c in self.children)
        if self.has_body and not self.children:
            h = MIN_SLOT_H
        return h

    def total_height(self):
        return self.header_h() + self.body_h() + self.footer_h()

    def total_width(self):
        w = MIN_W
        for c in self.children:
            w = max(w, SLOT_LEFT + c.total_width() + 14)
        return w

    def boundingRect(self):
        return QRectF(-1, -1, self.total_width() + 2, self.total_height() + 2)

    # ---------- 绘制 ----------
    def paint(self, painter, option, widget):
        painter.setRenderHint(QPainter.Antialiasing)
        W = self.total_width()
        H = self.total_height()
        hh = self.header_h()
        fh = self.footer_h()

        color = QColor(self.defn["color"])
        if self.isSelected():
            color = color.lighter(135)

        if self.has_body:
            outer = QPainterPath()
            outer.addRoundedRect(QRectF(0, 0, W, H), 7, 7)
            inner = QPainterPath()
            inner.addRect(QRectF(SLOT_LEFT, hh, W - SLOT_LEFT - 4, H - hh - fh))
            path = outer.subtracted(inner)
            painter.setBrush(QBrush(color))
            painter.setPen(QPen(QColor("#111111"), 1.5))
            painter.drawPath(path)
        else:
            painter.setBrush(QBrush(color))
            painter.setPen(QPen(QColor("#111111"), 1.5))
            painter.drawRoundedRect(QRectF(0, 0, W, H), 7, 7)

        painter.setPen(QColor("white"))
        painter.setFont(QFont("Microsoft YaHei", 10))
        text = self.defn["label"]
        for p in self.params:
            text += f"  [{p}]"
        painter.drawText(QRectF(12, 0, W - 24, hh),
                         Qt.AlignVCenter | Qt.AlignLeft, text)

    # ---------- 布局 ----------
    def _layout_children(self):
        self.prepareGeometryChange()
        y = self.header_h()
        for c in self.children:
            if c.parentItem() is not self:
                c.setParentItem(self)
            c.setPos(SLOT_LEFT, y)
            y += c.total_height()
        self.update()

    def slot_positions(self):
        result = []
        if not self.has_body:
            return result
        base = self.scenePos()
        x = base.x() + SLOT_LEFT
        y = base.y() + self.header_h()
        result.append((0, QPointF(x, y)))
        for i, c in enumerate(self.children):
            y += c.total_height()
            result.append((i + 1, QPointF(x, y)))
        return result

    # ---------- 拖动：脱离父积木，自由移动 ----------
    def mousePressEvent(self, event):
        p = self.parentItem()
        if isinstance(p, BlockItem):
            scene_pos = self.scenePos()
            self.setParentItem(None)
            self.setPos(scene_pos)
            if self in p.children:
                p.children.remove(self)
            p._layout_children()
        self.setZValue(100)
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        super().mouseReleaseEvent(event)
        self.setZValue(1)
        self._try_snap()

    def _try_snap(self):
        scene = self.scene()
        if scene is None:
            return
        my_pos = self.scenePos()
        best = None
        best_dist = 1e9
        for it in scene.items():
            if not isinstance(it, BlockItem) or it is self or not it.has_body:
                continue
            if it.isAncestorOf(self):
                continue
            for idx, slot in it.slot_positions():
                dx = abs(my_pos.x() - slot.x())
                dy = abs(my_pos.y() - slot.y())
                if dx < 60 and dy < BLOCK_H * 0.9:
                    d = dx + dy
                    if d < best_dist:
                        best_dist = d
                        best = (it, idx)

        if best:
            parent, idx = best
            parent.children.insert(idx, self)
            self.setParentItem(parent)
            parent._layout_children()

    # ---------- 代码生成 ----------
    def gen_code(self, level=0):
        ind = "    " * level
        try:
            line = self.defn["code"].format(*self.params)
        except Exception:
            line = f"# {self.kind} 参数错误"

        if not self.has_body:
            return ind + line

        lines = [ind + line]
        if not self.children:
            lines.append("    " * (level + 1) + "pass")
        else:
            for c in self.children:
                lines.append(c.gen_code(level + 1))
        return "\n".join(lines)


# ============ 4. 画布 ============
class Canvas(QGraphicsScene):
    def __init__(self):
        super().__init__()
        self.setSceneRect(0, 0, 1400, 1600)

    def drawBackground(self, painter, rect):
        painter.fillRect(rect, QColor("#252535"))
        grid = 20
        painter.setPen(QPen(QColor("#303050"), 1))
        x = int(rect.left()) - (int(rect.left()) % grid)
        y = int(rect.top()) - (int(rect.top()) % grid)
        lines = []
        while x < rect.right():
            lines.append(QLineF(x, rect.top(), x, rect.bottom()))
            x += grid
        while y < rect.bottom():
            lines.append(QLineF(rect.left(), y, rect.right(), y))
            y += grid
        painter.drawLines(lines)

    def add_block(self, kind, scene_pos=None):
        item = BlockItem(kind)
        self.addItem(item)
        if scene_pos is None:
            tops = [it for it in self.items()
                    if isinstance(it, BlockItem) and it.parentItem() is None]
            y = max([t.scenePos().y() + t.total_height() for t in tops], default=0) + 20
            item.setPos(60, y)
        else:
            item.setPos(scene_pos)
        return item

    def remove_block(self, item):
        for c in list(item.children):
            sp = c.scenePos()
            c.setParentItem(None)
            c.setPos(sp)
        item.children.clear()
        self.removeItem(item)

    def top_blocks(self):
        tops = [it for it in self.items()
                if isinstance(it, BlockItem) and it.parentItem() is None]
        tops.sort(key=lambda b: (b.scenePos().y(), b.scenePos().x()))
        return tops

    def generate_code(self):
        return "\n".join(b.gen_code(0) for b in self.top_blocks())

    def needs_time_import(self):
        def walk(b):
            return b.kind == "wait" or any(walk(c) for c in b.children)
        return any(walk(b) for b in self.top_blocks())


# ============ 5. 视图 ============
class CanvasView(QGraphicsView):
    def __init__(self, scene):
        super().__init__(scene)
        self.setRenderHint(QPainter.Antialiasing)
        self.setDragMode(QGraphicsView.RubberBandDrag)
        self.setBackgroundBrush(QColor("#252535"))
        self.setViewportUpdateMode(QGraphicsView.SmartViewportUpdate)
        self.setContextMenuPolicy(Qt.CustomContextMenu)
        self.customContextMenuRequested.connect(self._menu)

    def _menu(self, pos):
        item = self.itemAt(pos)
        if isinstance(item, BlockItem):
            menu = QMenu(self)
            edit_act = menu.addAction("编辑参数")
            menu.addSeparator()
            del_act = menu.addAction("删除积木")
            act = menu.exec_(self.mapToGlobal(pos))
            if act == edit_act:
                self._edit(item)
            elif act == del_act:
                self.scene().remove_block(item)

    def _edit(self, item):
        dlg = ParamDialog(item.defn, item.params, self)
        if dlg.exec_() == QDialog.Accepted:
            item.params = dlg.values()
            item.update()

    def mouseDoubleClickEvent(self, event):
        item = self.itemAt(event.pos())
        if isinstance(item, BlockItem):
            self._edit(item)
            return
        super().mouseDoubleClickEvent(event)


# ============ 6. 主窗口 ============
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PyQt5 积木逻辑编程")
        self.resize(1200, 740)

        # 左：积木箱
        self.palette_list = QListWidget()
        for kind, defn in BLOCK_DEFS.items():
            it = QListWidgetItem(defn["label"])
            it.setData(Qt.UserRole, kind)
            self.palette_list.addItem(it)
        self.palette_list.itemDoubleClicked.connect(self._add_from_palette)

        add_btn = QPushButton("添加到画布")
        add_btn.clicked.connect(self._add_selected)

        left = QWidget()
        lv = QVBoxLayout(left)
        lv.addWidget(QLabel("积木箱（双击添加）"))
        lv.addWidget(self.palette_list)
        lv.addWidget(add_btn)

        # 中：画布
        self.scene = Canvas()
        self.view = CanvasView(self.scene)

        # 右：代码
        self.code_edit = QPlainTextEdit()
        self.code_edit.setReadOnly(True)
        self.code_edit.setFont(QFont("Consolas", 11))

        gen_btn = QPushButton("生成 Python 代码")
        gen_btn.clicked.connect(self._generate)
        reset_btn = QPushButton("重置视图")
        reset_btn.clicked.connect(self._reset_view)
        dump_btn = QPushButton("打印场景内容")
        dump_btn.clicked.connect(self._dump)

        right = QWidget()
        rv = QVBoxLayout(right)
        rv.addWidget(QLabel("生成的代码"))
        rv.addWidget(self.code_edit)
        rv.addWidget(gen_btn)
        rv.addWidget(reset_btn)
        rv.addWidget(dump_btn)

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(left)
        splitter.addWidget(self.view)
        splitter.addWidget(right)
        splitter.setSizes([200, 640, 340])
        self.setCentralWidget(splitter)

        # 初始演示：摆两块积木
        self.scene.add_block("print")
        self.scene.add_block("for")

        # 首帧后把视图对准积木
        QTimer.singleShot(0, self._reset_view)

    def _add_from_palette(self, item):
        self.scene.add_block(item.data(Qt.UserRole))

    def _add_selected(self):
        it = self.palette_list.currentItem()
        if it:
            self.scene.add_block(it.data(Qt.UserRole))

    def _generate(self):
        code = self.scene.generate_code()
        header = "import time\n\n" if self.scene.needs_time_import() else ""
        self.code_edit.setPlainText(header + code)

    def _reset_view(self):
        rect = self.scene.itemsBoundingRect()
        if rect.isEmpty():
            return
        self.view.fitInView(rect.adjusted(-60, -60, 60, 60), Qt.KeepAspectRatio)

    def _dump(self):
        print("=== 场景中的所有积木 ===")
        for it in self.scene.items():
            if isinstance(it, BlockItem):
                print(f"  {it.kind:8s}  scenePos={it.scenePos()}  "
                      f"h={it.total_height()}  children={len(it.children)}")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())
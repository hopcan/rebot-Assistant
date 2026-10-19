import sys
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QScrollArea, QFrame
)
from PyQt5.QtCore import Qt


class Card(QFrame):
    """固定大小的卡片"""
    def __init__(self, title="卡片", parent=None):
        super().__init__(parent)
        self.setFixedSize(200, 120)          # 固定大小
        self.setFrameShape(QFrame.Box)
        self.setStyleSheet("""
            QFrame {
                background-color: #ffffff;
                border: 1px solid #cccccc;
                border-radius: 8px;
            }
            QLabel {
                border: none;
                font-size: 14px;
            }
        """)

        layout = QVBoxLayout(self)
        self.title_label = QLabel(title)
        self.title_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.title_label)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("添加卡片示例")
        self.resize(700, 500)

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        # 顶部按钮
        btn_add = QPushButton("添加卡片")
        btn_add.clicked.connect(self.add_card)
        root.addWidget(btn_add)

        # 滚动区
        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)     # 关键：容器随窗口调整
        root.addWidget(self.scroll, stretch=1)

        # 滚动区内部的容器
        self.container = QWidget()
        self.card_layout = QVBoxLayout(self.container)
        self.card_layout.setAlignment(Qt.AlignTop)   # 卡片从上往下排
        self.card_layout.setSpacing(10)
        self.card_layout.setContentsMargins(10, 10, 10, 10)

        self.scroll.setWidget(self.container)

        self.card_count = 0

    def add_card(self):
        self.card_count += 1
        card = Card(f"卡片 {self.card_count}")
        self.card_layout.addWidget(card)

        # 添加后滚动到底部（可选）
        # QTimer.singleShot(0, lambda: self.scroll.verticalScrollBar().setValue(
        #     self.scroll.verticalScrollBar().maximum()))


if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())
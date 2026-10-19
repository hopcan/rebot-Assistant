main_window_style ="""
    QMainWindow {
        background: white;
    }
    QMenuBar {
        background: qlineargradient(
        x1:0, y1:0, x2:1, y2:0,
        stop:0 #A5D6A7,  
        stop:1 #FFF9C4   ); 
    }
    QStatusBar {
        background: qlineargradient(
        x1:0, y1:0, x2:1, y2:0,
        stop:0 #C8E6C9,  
        stop:1 white); 
    }
    """
        # background: qlineargradient(
        #     x1:0, y1:0, x2:0, y2:1,
        #     stop:0 #FFFFFF,
        #     stop:1 #F1F8E9
        # );
function_btn_on_style = """
QPushButton {
    background-color: rgba(33, 150, 243, 0.5);   /* 半透明蓝 */
        color: #F5F5F5;
        border: 1px solid rgba(255, 255, 255, 0.3);
        border-radius: 8px;
        padding: 8px 10px;
        font-size: 14px;
        min-width: 100px;
        min-height: 16px;                             /* 15px 太小，建议 32 */
}
QPushButton:hover {
    background-color: #1976D2;
}
QPushButton:pressed {
    background-color: #0D47A1;
}
QPushButton:disabled {
    background-color: #BDBDBD;
    color: #757575;
}
"""

function_btn_off_style = """
QPushButton {
    background-color: rgba(233, 30, 99, 0.5);
    color: white;
    border: 1px solid rgba(255, 255, 255, 0.3);
    border-radius: 8px;
    padding: 8px 10px;
    font-size: 14px;
    min-width: 100px;
    min-height: 16px;
}
QPushButton:hover {
    background-color: #FF1493;
}
QPushButton:pressed {
    background-color: #C71585;
}
QPushButton:disabled {
    background-color: #F8BBD0;
    color: #757575;
}
"""
reset_sim_arm_btn_style = """
QPushButton {
    color: black;
    padding: 8px 20px;
    font-size: 14px;
    min-width: 100px;
    min-height: 20px;

    background-color: rgba(255, 255, 255, 0.15);
    border: 1px solid rgba(255, 255, 255, 0.3);
    border-color: #BDBDBD;
    border-radius: 12px;
}
QPushButton:hover {
    background-color: #F5F5F5;
    border-color: #9E9E9E;
}
QPushButton:pressed {
    background-color: #BDBDBD;
    border-color: #757575;
}
"""
arm_connect_on_style = """
QPushButton {
    background-color: #00C853;
    color: white;
    border: 1px;
    border-radius: 8px;
    padding: 5px 5px;
    font-size: 14px;
    min-width: 50px;
    min-height: 20px;
}
QPushButton:hover {
    background-color: #4CAF50;
}
QPushButton:pressed {
    background-color: #2E7D32;
}
QPushButton:disabled {
    background-color: #C8E6C9;
    color: #757575;
}
"""

arm_connect_off_style = """
QPushButton {
    background-color: #F44336;   
    color: white;
    border: 1px;
    border-radius: 8px;
    padding: 5px 5px;
    font-size: 14px;
    min-width: 50px;
    min-height: 20px;
}
QPushButton:hover {
    background-color: #D32F2F;     
}
QPushButton:pressed {
    background-color: #B71C1C;
}
QPushButton:disabled {
    background-color: #FFCDD2;   
    color: #757575;
}
"""

scan_style = """
QPushButton {
    background-color: #FFCDD2;
    color: black;
    border: 1px;
    border-radius: 8px;
    padding: 5px 5px;
    font-size: 14px;
    min-width: 50px;
    min-height: 20px;
}
QPushButton:hover {
    background-color: #F8BBD0;
}
QPushButton:pressed {
    background-color: #FFCDD2;
}
"""

show_collision_on_style = """
QPushButton {
    background-color: #FFEB3B;
    color: black;
    border: 1px;
    border-radius: 8px;
    padding: 5px 5px;
    font-size: 14px;
    min-width: 50px;
    min-height: 20px;
}
QPushButton:hover {
    background-color: #FDD835;
}
QPushButton:pressed {
    background-color: #FFF176;
}
"""

show_collision_off_style = """
QPushButton {
    background-color: #FF9800;
    color: black;
    border: 1px;
    border-radius: 8px;
    padding: 5px 5px;
    font-size: 14px;
    min-width: 50px;
    min-height: 20px;
}
QPushButton:hover {
    background-color: #F57C00;
}
QPushButton:pressed {
    background-color: #EF6C00;
}

"""


slider_style = """
QSlider::groove:horizontal {
    height: 6px;
    background: #E0E0E0;
    border-radius: 3px;
}
QSlider::sub-page:horizontal {
    background: #FFCDD2;      /* 已滑过：粉色 */
    border-radius: 3px;
}
QSlider::add-page:horizontal {
    background: #E0E0E0;      /* 未滑过：浅灰 */
    border-radius: 3px;
}
QSlider::handle:horizontal {
    background: #F8BBD0;      /* 滑块：深粉 */
    width: 18px;
    height: 12px;
    margin: -6px 0;           /* 让滑块盖住滑槽 */
    border-radius: 9px;
}
QSlider::handle:horizontal:hover {
    background: #C71585;
}
QSlider::handle:horizontal:pressed {
    background: #AD1457;
}
QSlider::handle:horizontal:disabled {
    background: #F8BBD0;
}
"""


joint_pos_label_style ="""
    QLabel {
        border: 2px solid #BDBDBD;
        width: 70px;
        border-radius: 8px;
        padding: 8px 12px;
        background-color: white;
        color: #333333;
    }
    """


dialog_style ="""QDialog { 
    background: qlineargradient(
            x1:0, y1:0, x2:1, y2:0,
            stop:0 #C8E6C9,  
            stop:1 #FFF9C4   ); 
}"""

ok_btn_style = """
    QPushButton {
        background-color: #4CAF50;
        color: white;
        border: none;
        border-radius: 6px;
        padding: 6px 20px;
        width: 50px;
        min-height: 10px;
        font-size: 14px;
    }
    QPushButton:hover {
        background-color: #388E3C;
    }
    QPushButton:pressed {
        background-color: #2E7D32;
    }
    QPushButton:disabled {
        background-color: #C8E6C9;
        color: #9E9E9E;
    }
    """

cancel_btn_style = """
    QPushButton {
        background-color: #F44336;
        color: white;
        border: none;
        border-radius: 6px;
        padding: 6px 20px;
        width: 50px;
        min-height: 10px;
        font-size: 14px;
    }
    QPushButton:hover {
        background-color: #D32F2F;
    }
    QPushButton:pressed {
        background-color: #B71C1C;
    }
    QPushButton:disabled {
        background-color: #FFCDD2;
        color: #9E9E9E;
    }
    """

motor_show_card_style = """
    QFrame {
        background-color: white;       
        border: 1px solid #E0E0E0;       
        border-radius: 8px;
        border-radius: 8px;
        padding: 5px;
    }
    """

motor_show_param_style = """
    QFrame {
        background-color: white;       
        border: 1px solid #E0E0E0;        
        border-radius: 8px;
        border-radius: 8px;
        padding: 5px;
    }
    """

motor_btn_card_style = """
    QPushButton {
        color: black;
        text-align: left top;
        padding: 2px 4px;
        font-size: 14px;
        min-width: 100px;
        min-height: 50px;

        background-color: rgba(255, 255, 255, 0.15);
        border: 1px solid rgba(255, 255, 255, 0.3);
        border-color: #BDBDBD;
        border-radius: 12px;
    }
    QPushButton:hover {
        background-color: #F5F5F5;
        border-color: #9E9E9E;
    }
    QPushButton:pressed {
        background-color: #BDBDBD;
        border-color: #757575;
    }
    """
motor_btn_card_enable_style = """
    QPushButton {
        color: white;
        text-align: left top;
        padding: 2px 4px;
        font-size: 14px;
        min-width: 100px;
        min-height: 50px;

        background-color: rgba(233, 30, 99, 0.4);
        border: 1px solid rgba(255, 255, 255, 0.3);
        border-color: #BDBDBD;
        border-radius: 12px;
    }
    QPushButton:hover {
          background-color: #FFCDD2;
        border-color: #9E9E9E;
    }
    QPushButton:pressed {
          background-color: #FFCDD2;
        border-color: #757575;
    }
    """
motor_btn_card_disable_style = """
    QPushButton {
        color: black;
        text-align: left top;
        padding: 2px 4px;
        font-size: 14px;
        min-width: 100px;
        min-height: 50px;

        background-color: rgba(255, 255, 255, 0.15);
        border: 1px solid rgba(255, 255, 255, 0.3);
        border-color: #BDBDBD;
        border-radius: 12px;
    }
    QPushButton:hover {
        background-color: #F5F5F5;
        border-color: #9E9E9E;
    }
    QPushButton:pressed {
        background-color: #BDBDBD;
        border-color: #757575;
    }
    """
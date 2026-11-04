import time

from motorbridge import Controller,Mode

class motor_handle:

    def __init__(self, interface,motor_version = "DM", log_handle = None):  #DM or RS
        self.is_connected = False
        self.ctrl = interface
        self.motor_version = motor_version
        self.log_handle = log_handle
        self.scan_min_can_id = None # 当前已经添加的扫描电机范围
        self.scan_max_can_id = None 
        self.motors = {}    # 电机空字典
    def enable_motor(self,motor_id):
        self.motors[motor_id].enable()
        return 

    def disable_motor(self,motor_id):
        self.motors[motor_id].disable()
        return

    def scan_motor(self,start_can_id=0x01,end_can_id=0x10):
        if self.motor_version == "DM":
            return self.scan_dmMotor(start_can_id,end_can_id)

    def disable_all_motor(self):
        # 使能所有电机
        self.ctrl.disable_all()
           
    def __enter__(self):
        return self
    
    def __exit__(self, *args):
        return

    def scan_dmMotor(self, start_can_id=0x01, end_can_id=0x10):
        found_motors = []
        # 添加电机
        if self.scan_min_can_id == None and self.scan_max_can_id == None:
            self.scan_min_can_id = start_can_id 
            self.scan_max_can_id = end_can_id
            for motor_can_id in range(start_can_id, end_can_id + 1):
                temp_motor_master_id = 0x10 + motor_can_id
                try:
                    self.motors[motor_can_id] = self.ctrl.add_damiao_motor(motor_can_id, temp_motor_master_id, "4340P")
                except Exception as e:
                    self.log_handle.emit(f"[error] motor_can_id=0x{motor_can_id:02X}: {e}")
        else:
            # 当前开始最小CAN ID 比要扫描的的大，添加缺失的电机
            if self.scan_min_can_id > start_can_id:
                for motor_can_id in range(start_can_id, self.scan_min_can_id):
                    temp_motor_master_id = 0x10 + motor_can_id
                    try:
                        self.motors[motor_can_id] = self.ctrl.add_damiao_motor(motor_can_id, temp_motor_master_id, "4340P")
                    except Exception as e:
                        self.log_handle.emit(f"[error] motor_can_id=0x{motor_can_id:02X}: {e}")
                self.scan_min_can_id = start_can_id


            # 当前结束最大CAN ID 比要扫描的的小，添加缺失的电机
            if self.scan_max_can_id < end_can_id:
                for motor_can_id in range(self.scan_max_can_id + 1, end_can_id + 1):
                    temp_motor_master_id = 0x10 + motor_can_id
                    try:
                        self.motors[motor_can_id] = self.ctrl.add_damiao_motor(motor_can_id, temp_motor_master_id, "4340P")
                    except Exception as e:
                        self.log_handle.emit(f"[error] motor_can_id=0x{motor_can_id:02X}: {e}")
                self.scan_max_can_id = end_can_id

        # 扫描已经添加的电机
        for motor_can_id in range(start_can_id, end_can_id + 1):
            try:
                # 读取寄存器以获取CAN ID
                esc_id = self.motors[motor_can_id].get_register_u32(8, timeout_ms=100)
                master_id = self.motors[motor_can_id].get_register_u32(7, timeout_ms=100)
                self.log_handle.emit(f"[find] motor_can_id=0x{esc_id:02X} motor_master_id=0x{master_id:02X}")
                id = [esc_id, master_id]
                found_motors.append(id)
                
            except Exception:
                self.log_handle.emit(f"[no respond] motor_can_id=0x{motor_can_id:02X}")
                pass
        
        print(f"\nfinish find {len(found_motors)} motor")
        print(f"scan_min_can_id: {self.scan_min_can_id}, scan_max_can_id: {self.scan_max_can_id}")
        print(f"start_can_id: {start_can_id}, end_can_id: {end_can_id}")
        return found_motors



if __name__ == "__main__":
    # Example usage
    interface = Controller.from_dm_serial("COM16", 921600)
    with motor_handle(interface, motor_version="DM") as motor_handle:
            found_motors = motor_handle.scan_dmMotor(start_can_id=0x01, end_can_id=0x07)
            print("Found motors:", found_motors)
            for found_motor in found_motors:
                motor_can_id,master_can_id = found_motor
                print(f"Enabling motor with CAN ID: 0x{motor_can_id:02X}")
                print(f"Enabling motor with Master ID: 0x{master_can_id:02X}")
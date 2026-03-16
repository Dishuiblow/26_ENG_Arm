import sys
import serial
import serial.tools.list_ports
import numpy as np
import struct
import pandas as pd
import time
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QPushButton, QComboBox, QLabel, QMessageBox, QGroupBox, 
                             QRadioButton, QButtonGroup, QTextEdit, QFileDialog)
from PyQt6.QtCore import QThread, pyqtSignal, Qt, QTimer
from PyQt6.QtGui import QFont, QColor
import pyqtgraph as pg
from scipy.interpolate import CubicSpline

# --- 1. 通信协议定义 ---
# 必须与 STM32 下位机 usb_protocol.h 保持一致
CMD_STOP          = 0x00
CMD_HOME          = 0x01
CMD_GRAVITY_COMP  = 0x02
CMD_TEACH_MODE    = 0x03
CMD_TRAJ_DATA     = 0x04 # 发送轨迹点
CMD_ENABLE        = 0x05
CMD_LOCK_CURRENT  = 0x06 # 锁死当前位置

HEADER = b'\xAA\x55'
TAIL   = b'\x0D\x0A'

class Protocol:
    @staticmethod
    def pack_cmd(cmd_id, param_bytes=b''):
        """打包指令发送给下位机"""
        data_len = len(param_bytes)
        checksum = (cmd_id + data_len)
        for b in param_bytes: checksum += b
        checksum &= 0xFF
        return HEADER + struct.pack('BB', cmd_id, data_len) + param_bytes + struct.pack('B', checksum) + TAIL

    @staticmethod
    def unpack_data(buffer):
        """解析接收到的 12 个 float 数据"""
        # 包结构: Head(2)+ID(1)+Len(1)+Data(48)+CS(1)+Tail(2) = 55 字节
        MIN_LEN = 55
        while len(buffer) >= MIN_LEN:
            idx = buffer.find(HEADER)
            if idx == -1: return b'', None
            
            buffer = buffer[idx:] #以此为开头
            if len(buffer) < MIN_LEN: return buffer, None
            
            data_len = buffer[3]
            if data_len != 48: # 简单校验长度 (6角度+6力矩 = 12*4 = 48)
                buffer = buffer[2:]; continue
                
            total_len = 4 + data_len + 1 + 2
            if len(buffer) < total_len: return buffer, None
            
            packet = buffer[:total_len]
            if packet[-2:] != TAIL: 
                buffer = buffer[2:]; continue
            
            try:
                payload = packet[4 : 4+data_len]
                # 解析 12 个 float
                values = struct.unpack('<12f', payload)
                buffer = buffer[total_len:]
                return buffer, list(values)
            except:
                buffer = buffer[2:]; continue
        return buffer, None

class SerialThread(QThread):
    sig_data = pyqtSignal(list) # 信号：传输解析后的数据列表
    
    def __init__(self):
        super().__init__()
        self.ser = None
        self.running = False
        self.buf = b''
        
    def connect(self, port):
        try:
            self.ser = serial.Serial(port, 115200, timeout=0.01)
            self.running = True
            self.start()
            return True
        except Exception as e:
            print(f"串口错误: {e}")
            return False
        
    def close(self):
        self.running = False
        self.wait()
        if self.ser: self.ser.close()
        
    def send_bytes(self, data):
        if self.ser and self.ser.is_open:
            try: self.ser.write(data)
            except: pass
            
    def run(self):
        while self.running and self.ser:
            try:
                if self.ser.in_waiting:
                    self.buf += self.ser.read(self.ser.in_waiting)
                    # 循环解析缓冲区
                    while True:
                        self.buf, values = Protocol.unpack_data(self.buf)
                        if values: self.sig_data.emit(values)
                        else: break
            except: pass
            self.msleep(2)

# --- 2. 自定义双坐标轴波形图 ---
class DualAxisPlot(pg.PlotWidget):
    def __init__(self):
        super().__init__()
        self.setBackground('w') #以此白色背景
        self.showGrid(x=True, y=True, alpha=0.3)
        self.setLabel('left', '角度 (Rad)', color='#D32F2F') # 红色左轴
        self.setLabel('bottom', '采样点')
        
        # 主轴曲线 (角度 - 红色实线)
        self.plot_angle = self.plot(pen=pg.mkPen(color='#D32F2F', width=2), name="角度")
        
        # 创建右侧副轴 (力矩 - 蓝色)
        self.p2 = pg.ViewBox()
        self.scene().addItem(self.p2)
        self.getPlotItem().layout.addItem(self.p2, 2, 2)
        self.getPlotItem().layout.setColumnFixedWidth(2, 60)
        self.p2.setGeometry(self.getPlotItem().vb.sceneBoundingRect())
        self.p2.enableAutoRange(axis=pg.ViewBox.YAxis, enable=True)
        self.p2.setXLink(self.getPlotItem()) # X轴同步
        
        self.axis_torq = pg.AxisItem('right')
        self.axis_torq.setLabel('力矩 (Nm)', color='#1976D2')
        self.axis_torq.linkToView(self.p2)
        self.getPlotItem().layout.addItem(self.axis_torq, 2, 3)
        
        # 副轴曲线 (力矩 - 蓝色虚线)
        self.curve_torq = pg.PlotCurveItem(pen=pg.mkPen(color='#1976D2', width=1.5, style=Qt.PenStyle.DashLine))
        self.p2.addItem(self.curve_torq)
        
        # 响应窗口大小变化
        self.getPlotItem().vb.sigResized.connect(self.update_views)

    def update_views(self):
        self.p2.setGeometry(self.getPlotItem().vb.sceneBoundingRect())
        self.p2.linkedViewChanged(self.getPlotItem().vb, self.p2.XAxis)

    def update_data(self, angles, torques):
        self.plot_angle.setData(angles)
        self.curve_torq.setData(torques)

# --- 3. 主窗口逻辑 ---
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Ares_Arm_Controller")
        self.resize(1100, 750)
        
        # 后台线程
        self.comm = SerialThread()
        self.comm.sig_data.connect(self.on_data)
        
        # 数据相关
        self.recording = False
        self.current_angles = [0.0]*6 # 实时存储当前角度
        self.raw_traj = []            # 录制的原始数据 (N, 12)
        self.fitted_traj = None       # 插值后的复现轨迹
        
        # 绘图缓冲区 (只用于显示，滚动窗口 300 点)
        self.buf_len = 300
        self.angle_bufs = [[] for _ in range(6)]
        self.torq_bufs  = [[] for _ in range(6)]
        
        # 自动流程控制定时器
        self.replay_timer = QTimer()
        self.replay_timer.setInterval(20) # 50Hz发送频率
        self.replay_timer.timeout.connect(self.on_replay_step)
        self.replay_idx = 0
        
        self.seq_timer = QTimer() # 用于流程等待
        self.seq_timer.setSingleShot(True)
        self.seq_timer.timeout.connect(self.on_seq_timeout)
        self.seq_state = 0 # 0:Idle, 1:HomingWait, 2:PreRunWait
        
        self.ui_init()

    def ui_init(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QHBoxLayout(main_widget)
        
        # === 左侧面板 (固定宽度) ===
        left_panel = QWidget()
        left_panel.setFixedWidth(240)
        v_left = QVBoxLayout(left_panel)
        
        # 1. 通信
        gb_conn = QGroupBox("1. 通信连接")
        v_conn = QVBoxLayout()
        self.cb_port = QComboBox()
        self.refresh_port()
        btn_conn = QPushButton("连接 / 断开")
        btn_conn.setCheckable(True)
        btn_conn.clicked.connect(self.toggle_com)
        v_conn.addWidget(self.cb_port)
        v_conn.addWidget(QPushButton("刷新串口", clicked=self.refresh_port))
        v_conn.addWidget(btn_conn)
        gb_conn.setLayout(v_conn)
        
        # 2. 控制指令
        gb_ctrl = QGroupBox("2. 机械臂控制")
        v_ctrl = QVBoxLayout()
        v_ctrl.addWidget(QPushButton("电机使能 (Enable)", clicked=lambda: self.send_cmd(CMD_ENABLE)))
        v_ctrl.addWidget(QPushButton("慢速归零 (Home)", clicked=lambda: self.send_cmd(CMD_HOME)))
        v_ctrl.addWidget(QPushButton("重力补偿 (Manual)", clicked=lambda: self.send_cmd(CMD_GRAVITY_COMP)))
        
        # 示教按钮
        self.btn_rec = QPushButton("开始示教 (Record)")
        self.btn_rec.setCheckable(True)
        self.btn_rec.setStyleSheet("text-align:left; padding:5px;")
        self.btn_rec.clicked.connect(self.toggle_record)
        v_ctrl.addWidget(self.btn_rec)
        
        # 复现按钮
        self.btn_rep = QPushButton("自动复现 (Auto)")
        self.btn_rep.setCheckable(True)
        self.btn_rep.setStyleSheet("text-align:left; padding:5px;")
        self.btn_rep.clicked.connect(self.toggle_replay_seq)
        v_ctrl.addWidget(self.btn_rep)
        
        v_ctrl.addWidget(QPushButton("导出 Excel 数据", clicked=self.export_excel))
        
        # 急停
        btn_stop = QPushButton("⚠️ 急停 (STOP)")
        btn_stop.setStyleSheet("background-color:red; color:white; font-weight:bold; height:40px;")
        btn_stop.clicked.connect(lambda: self.send_cmd(CMD_STOP))
        v_ctrl.addWidget(btn_stop)
        
        gb_ctrl.setLayout(v_ctrl)
        
        v_left.addWidget(gb_conn)
        v_left.addWidget(gb_ctrl)
        v_left.addStretch()
        
        # === 右侧面板 (自适应) ===
        right_panel = QWidget()
        v_right = QVBoxLayout(right_panel)
        
        # 1. 关节选择栏
        gb_sel = QGroupBox("波形监测选择")
        h_sel = QHBoxLayout()
        self.btn_group = QButtonGroup(self)
        self.btn_group.idToggled.connect(self.on_joint_select)
        
        for i in range(6):
            rb = QRadioButton(f"关节 J{i+1}")
            self.btn_group.addButton(rb, i) # ID: 0~5
            h_sel.addWidget(rb)
            if i == 0: rb.setChecked(True)
            
        gb_sel.setLayout(h_sel)
        
        # 2. 大波形图
        self.plot_widget = DualAxisPlot()
        
        # 3. 串口日志
        gb_log = QGroupBox("系统日志 / 串口监视")
        v_log = QVBoxLayout()
        self.txt_log = QTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.setStyleSheet("background:black; color:#00FF00; font-family:Consolas; font-size:10pt;")
        self.txt_log.setFixedHeight(120)
        v_log.addWidget(self.txt_log)
        gb_log.setLayout(v_log)
        
        v_right.addWidget(gb_sel)
        v_right.addWidget(self.plot_widget, stretch=1)
        v_right.addWidget(gb_log)
        
        layout.addWidget(left_panel)
        layout.addWidget(right_panel)

    # === 补全缺失的自动复现逻辑 ===
    
    def toggle_replay_seq(self):
        # 1. 检查是否有数据
        if not self.raw_traj and self.fitted_traj is None: 
            QMessageBox.warning(self, "无数据", "请先录制或载入轨迹！")
            self.btn_rep.setChecked(False)
            return
            
        # 如果还没有生成插值轨迹，先生成一下
        if self.fitted_traj is None: 
            self.generate_safe_traj()

        if self.btn_rep.isChecked():
            self.btn_rep.setText("⏹ 停止复现")
            
            # [阶段1] 发送归零指令
            self.log(">>> [阶段1] 系统正在归零...")
            self.send_cmd(CMD_HOME) 
            self.seq_state = 1
            
            # 估算归零等待时间 (简单给 4秒)
            self.seq_timer.start(4000) 
            
        else:
            self.stop_replay()

    def generate_safe_traj(self):
        # 简单的轨迹生成器，如果 self.raw_traj 存在，直接用它
        if self.raw_traj:
            # 提取前6列角度数据
            raw_data = np.array(self.raw_traj)[:, :6]
            self.fitted_traj = raw_data
            self.log(">>> 轨迹数据已准备就绪")
            
    def stop_replay(self):
        self.seq_timer.stop()
        self.replay_timer.stop()
        
        # 发送锁死指令保持当前位置
        self.send_cmd(CMD_LOCK_CURRENT) 
        
        self.btn_rep.setChecked(False)
        self.btn_rep.setText("自动复现 (Auto)")
        self.log(">>> 复现已手动终止")

    # --- 核心逻辑 ---

    def refresh_port(self):
        self.cb_port.clear()
        for p in serial.tools.list_ports.comports():
            self.cb_port.addItem(p.device)
            
    def toggle_com(self):
        btn = self.sender()
        if btn.isChecked():
            if self.comm.connect(self.cb_port.currentText()):
                btn.setText("断开连接")
                self.log("串口已连接")
            else:
                btn.setChecked(False)
                self.log("连接失败！")
        else:
            self.comm.close()
            btn.setText("连接 / 断开")
            self.log("串口已断开")

    def send_cmd(self, cmd, payload=b''):
        self.comm.send_bytes(Protocol.pack_cmd(cmd, payload))

    def log(self, text):
        t = time.strftime("%H:%M:%S")
        self.txt_log.append(f"[{t}] {text}")
        # 自动滚动到底部
        sb = self.txt_log.verticalScrollBar()
        sb.setValue(sb.maximum())

    # --- 数据接收与绘图 ---
    def on_data(self, values):
        # values: [Ang1..6, Torq1..6]
        angles = values[:6]
        torqs = values[6:]
        self.current_angles = angles # 更新当前值，用于锁死指令
        
        # 1. 存入绘图缓冲
        for i in range(6):
            self.angle_bufs[i].append(angles[i])
            self.torq_bufs[i].append(torqs[i])
            if len(self.angle_bufs[i]) > self.buf_len:
                self.angle_bufs[i].pop(0)
                self.torq_bufs[i].pop(0)
        
        # 2. 如果正在录制，存入原始数据
        if self.recording:
            self.raw_traj.append(values)
            
        # 3. 刷新波形 (仅刷新当前选中的关节)
        curr_id = self.btn_group.checkedId()
        if curr_id != -1:
            self.plot_widget.update_data(self.angle_bufs[curr_id], self.torq_bufs[curr_id])
            
        # 4. 只有在调试时才疯狂刷屏日志，平时可以注释掉下面这行
        # self.log(f"J1: {angles[0]:.2f} {torqs[0]:.2f}")

    def on_joint_select(self, btn, checked):
        if checked:
            idx = self.btn_group.id(btn)
            self.plot_widget.update_data(self.angle_bufs[idx], self.torq_bufs[idx])
            self.log(f"切换显示: 关节 J{idx+1}")

    # --- 示教录制 ---
    def toggle_record(self):
        if self.btn_rec.isChecked():
            self.raw_traj = []
            self.recording = True
            self.send_cmd(CMD_TEACH_MODE) # 变软
            self.btn_rec.setText("⏹ 停止并锁死")
            self.btn_rec.setStyleSheet("background-color: #FFA500; font-weight:bold;")
            self.log(">>> 开始示教录制...")
        else:
            self.recording = False
            # 【关键】发送锁死指令，带上当前位置
            payload = struct.pack('<6f', *self.current_angles)
            self.send_cmd(CMD_LOCK_CURRENT, payload)
            
            self.btn_rec.setText("开始示教 (Record)")
            self.btn_rec.setStyleSheet("")
            self.log(f">>> 录制结束，共 {len(self.raw_traj)} 点。位置已锁死。")

    # --- 导出 Excel ---
    def export_excel(self):
        if not self.raw_traj:
            QMessageBox.warning(self, "无数据", "请先进行示教录制！")
            return
        
        path, _ = QFileDialog.getSaveFileName(self, "保存数据", "motion_data.xlsx", "Excel Files (*.xlsx)")
        if path:
            try:
                cols = [f'J{i+1}_Ang' for i in range(6)] + [f'J{i+1}_Trq' for i in range(6)]
                df = pd.DataFrame(self.raw_traj, columns=cols)
                df.to_excel(path, index=False)
                self.log(f"数据已导出至: {path}")
                QMessageBox.information(self, "成功", "导出成功！")
            except Exception as e:
                QMessageBox.critical(self, "错误", str(e))

    # --- 自动复现序列 ---
    # 自动复现逻辑：先归零，等待归零物理完成，再发送数据

    def toggle_replay(self):
        if not self.taught_traj_rad:
            QMessageBox.warning(self, "无数据", "内存中无轨迹数据！")
            self.btn_replay.setChecked(False)
            return

        if self.btn_replay.isChecked():
            self.btn_replay.setText("⏹ 停止复现")
            
            # ==========================================================
            # 【核心修复】: 强制切换到 REPLAY 状态
            # ==========================================================
            # 如果机械臂之前处于 HOMING (归零) 或 GRAVITY (示教) 模式，
            # 必须先发送 CMD_LOCK_CURRENT (0x06)，让 STM32 进入 ARM_STATE_REPLAY。
            # 否则 STM32 会无视我们发送的轨迹数据。
            self.send_cmd(CMD_LOCK_CURRENT) 
            
            self.log(">>> 状态已切换至复现模式 (REPLAY)")
            # ==========================================================
            
            self.replay_idx = 0
            self.replay_timer.start()
            self.log(f">>> 开始复现，共 {len(self.taught_traj_rad)} 点...")
            
        else:
            self.replay_timer.stop()
            # 停止时，发送锁死指令保持当前位置 (比 CMD_STOP 更安全，防止软掉)
            self.send_cmd(CMD_LOCK_CURRENT) 
            self.btn_replay.setText("开始复现轨迹 (Replay)")
            self.log(">>> 复现停止")
            
    def on_seq_timeout(self):
        if self.seq_state == 1: 
            self.log(">>> [阶段2] 归零完成，开始执行轨迹...")
            # 此时机械臂应该已经在 0 位
            # 轨迹的第一点也是 0 位，无缝衔接
            self.replay_idx = 0
            self.replay_timer.start()

    def stop_replay(self):
        self.seq_timer.stop()
        self.replay_timer.stop()
        self.send_cmd(CMD_STOP)
        self.btn_rep.setChecked(False)
        self.btn_rep.setText("⑤ 自动复现 (Auto)")
        self.log(">>> 复现已手动终止")

    def generate_safe_traj(self):
        raw_data = np.array(self.raw_traj)[:, :6]
        if len(raw_data) < 2: return
        
        start_pt = raw_data[0]
        # 生成 80 个点 (约1.6s) 从 0 平滑过渡到 录制起点
        transition = np.linspace(np.zeros(6), start_pt, 80)
        
        full_data = np.vstack([transition, raw_data])
        
        x = np.arange(len(full_data))
        cs = CubicSpline(x, full_data, axis=0)
        self.fitted_traj = cs(x)
        self.log(">>> 轨迹生成完毕 (含自动归零过渡段)")

    def on_seq_timeout(self):
        if self.seq_state == 1: 
            # 归零结束 -> 等待 1s
            self.log(">>> [阶段2] 归零完成，等待稳定...")
            self.seq_state = 2
            self.seq_timer.start(1000)
            
        elif self.seq_state == 2: 
            # 等待结束 -> 开始发送数据
            self.log(">>> [阶段3] 开始执行轨迹...")
            self.replay_idx = 0
            self.replay_timer.start()

    def on_replay_step(self):
        # 数据发完 -> 结束
        if self.replay_idx >= len(self.fitted_traj):
            self.replay_timer.stop()
            self.log(">>> [阶段4] 复现完成，执行最终归零。")
            self.send_cmd(CMD_HOME)
            self.btn_rep.setChecked(False)
            self.btn_rep.setText("⑤ 自动复现 (Auto)")
            return
            
        # 发送当前点
        point = self.fitted_traj[self.replay_idx]
        payload = struct.pack('<6f', *point)
        self.send_cmd(CMD_TRAJ_DATA, payload)
        self.replay_idx += 1
    def generate_identification_traj(self):
        """
        通用重力辨识轨迹生成器
        可以随意修改下方的配置参数来针对不同电机
        """
        # ================= 配置区域 =================
        TARGET_JOINT_INDEX = 1   # 目标关节索引 (0=J1, 1=J2, 2=J3 ... 5=J6)
        START_RAD = -1.0         # 起始角度 (弧度) -1.0 rad ≈ -57度
        END_RAD   =  1.0         # 结束角度 (弧度)  1.0 rad ≈  57度
        STEPS     =  900         # 采样点数 (越多越慢，建议 400~1000)
        # ===========================================

        self.log(f">>> [辨识] 生成 J{TARGET_JOINT_INDEX+1} 扫描轨迹: {START_RAD} -> {END_RAD}")

        # 1. 生成扫描轴的数据 (线性插值)
        scan_values = np.linspace(START_RAD, END_RAD, STEPS)
        
        # 2. 构建 6轴 轨迹数据
        traj_points = []
        
        # 为了安全，我们生成一个“来回”的轨迹：去程 + 回程
        # 这样可以抵消库仑摩擦力的影响（去程摩擦力朝下，回程朝上，平均一下就是纯重力）
        full_scan = np.concatenate([scan_values, scan_values[::-1]]) # [去程, 回程]
        
        for val in full_scan:
            # 初始化一个全为 0 的 6轴数组
            pt = [0.0] * 6  
            
            # 【核心替换点】只修改目标关节的值
            pt[TARGET_JOINT_INDEX] = val 
            
            # (可选) 如果需要其他关节保持特定姿态（例如 J2 保持 0，J3 扫描）
            # pt[1] = 0.0 
            
            traj_points.append(pt)
            
        # 3. 赋值给 fitted_traj，准备通过“自动复现”功能发送
        self.fitted_traj = traj_points
        
        self.log(f">>> [辨识] 轨迹生成完毕，共 {len(traj_points)} 点。")
        self.log(">>> 请点击【自动复现】开始采集。注意：全程请勿手触碰机械臂！")
    # === 专用：重力参数辨识数据采集 ===
    def run_system_identification_scan(self):
        """
        全自动采集重力辨识数据
        前提：STM32端的 G_params 必须设为 0，且处于位置控制模式
        """
        self.log(">>> [辨识] 开始系统辨识数据采集...")
        self.raw_traj = [] # 清空旧数据
        self.recording = True # 开始记录
        
        # 生成扫描轨迹：让 J2 和 J3 分别缓慢运动
        # 1. J2 从 -1.5 扫到 1.5 (弧度)
        traj_pts = []
        scan_range = np.linspace(-1.0, 1.0, 200) # 200个点
        
        # 阶段一：扫 J2 (J3 保持 0)
        for val in scan_range:
            pt = [0.0]*6
            pt[1] = val # J2 (ID:2)
            pt[2] = 0.0 # J3 (ID:3)
            traj_pts.append(pt)
            
        # 阶段二：扫 J3 (J2 保持 0)
        for val in scan_range:
            pt = [0.0]*6
            pt[1] = 0.0
            pt[2] = val
            traj_pts.append(pt)
            
        # 发送轨迹
        self.log(f">>> [辨识] 发送 {len(traj_pts)} 个扫描点...")
        
        # 复用复现逻辑，但在发送前强制让下位机刚度变大
        # 这里我们利用 REPLAY 模式的高刚度特性 (Kp=10)
        # 注意：发送间隔要长，保证是准静态测量
        
        for i, pt in enumerate(traj_pts):
            payload = struct.pack('<6f', *pt)
            self.send_cmd(CMD_TRAJ_DATA, payload)
            
            # 关键：给足够的时间让电机稳定，消除加速度影响
            # 这里的延时要比平时复现长，比如 100ms 发一个点
            # 由于这是在主线程，直接 sleep 会卡界面，实际建议用 QTimer
            # 这里为了演示逻辑简化写，实际建议您手动慢慢发
            
            # 为了防止界面卡死，这里只是演示生成了轨迹
            # 您可以直接用 "自动复现" 功能来跑这段轨迹！
            
        # 更好的做法：
        # 1. 把这段生成的 traj_pts 赋值给 self.fitted_traj
        # 2. 调用 toggle_replay_seq()
        # 3. 只是要把 replay_timer 的间隔调大到 100ms (慢速)
        
        self.fitted_traj = traj_pts
        self.log(">>> [辨识] 扫描轨迹已生成，请点击【自动复现】开始采集！")
        self.log(">>> 注意：请确保下位机代码中 G_params 全为 0！")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setFont(QFont("Microsoft YaHei", 9))
    win = MainWindow()
    win.show()
    sys.exit(app.exec())
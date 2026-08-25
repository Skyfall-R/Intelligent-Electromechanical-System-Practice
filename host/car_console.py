#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
智能收集小车 · 调试控制台
=========================
配套协议：算法开发/02-调试协议.md  v2
主控固件：firmware/Core/Src/debug_link.c

运行:
    pip install PySide6 pyqtgraph pyserial
    python car_console.py            # 正常模式
    python car_console.py --demo     # 演示模式（不接硬件，用假数据看界面）

键位（先点「接管遥控」）:
    W / S      前进 / 后退
    A / D      左横移 / 右横移      ← 麦轮独有
    Q / E      左转 / 右转
    空格       急停
    Esc        交还自动
"""
from __future__ import annotations

import sys, os, csv, math, time, random, argparse
from collections import deque
from dataclasses import dataclass, field

# ══════════════════════════════════════════════════════════════════
#  Qt 绑定自适应
#  优先 PySide6；装不上（Windows 上常见的 DLL 版本冲突）就退回 PyQt5。
#  Anaconda 自带 PyQt5，所以这条兜底路基本一定能走通。
# ══════════════════════════════════════════════════════════════════
_W = "QWidget QMainWindow QApplication QVBoxLayout QHBoxLayout QGridLayout QLabel " \
     "QPushButton QComboBox QGroupBox QTableWidget QTableWidgetItem QPlainTextEdit " \
     "QTabWidget QSlider QCheckBox QHeaderView QAbstractItemView QMessageBox " \
     "QFileDialog QScrollArea QSplitter QFrame".split()

QT_LIB = None
_err = {}
for _lib in ("PySide6", "PyQt5"):
    try:
        _c = __import__(_lib + ".QtCore",    fromlist=["*"])
        _g = __import__(_lib + ".QtGui",     fromlist=["*"])
        _w = __import__(_lib + ".QtWidgets", fromlist=["*"])
        Qt, QThread, QTimer, QObject, QSize = _c.Qt, _c.QThread, _c.QTimer, _c.QObject, _c.QSize
        Signal = getattr(_c, "Signal", None) or _c.pyqtSignal
        QFont, QColor, QKeyEvent = _g.QFont, _g.QColor, _g.QKeyEvent
        for _n in _W:
            globals()[_n] = getattr(_w, _n)
        QT_LIB = _lib
        break
    except Exception as e:
        _err[_lib] = e

if QT_LIB is None:
    sys.stderr.write(
        "\n找不到可用的 Qt 绑定，两个都失败了：\n"
        f"  PySide6 : {_err.get('PySide6')}\n"
        f"  PyQt5   : {_err.get('PyQt5')}\n\n"
        "怎么办（任选其一）：\n"
        "  1) 装 Microsoft Visual C++ 2015-2022 Redistributable (x64)，重开终端\n"
        "  2) conda install -c conda-forge pyqt   （用 Anaconda 自带的 PyQt5）\n"
        "  3) pip install PyQt5\n\n")
    raise SystemExit(1)

os.environ.setdefault("PYQTGRAPH_QT_LIB", QT_LIB)
import pyqtgraph as pg
import serial
import serial.tools.list_ports


# ══════════════════════════════════════════════════════════════════
#  配色
# ══════════════════════════════════════════════════════════════════
C_BG      = "#0f1115"
C_PANEL   = "#171a21"
C_PANEL2  = "#1d2129"
C_BORDER  = "#2a3040"
C_TEXT    = "#e6e9ef"
C_DIM     = "#8b93a7"
C_ACCENT  = "#4da3ff"
C_OK      = "#3ddc97"
C_WARN    = "#ffb454"
C_ERR     = "#ff5c5c"
C_PURPLE  = "#b48ead"

SERIES_COLORS = ["#4da3ff", "#3ddc97", "#ffb454", "#ff7ab6", "#b48ead", "#67e8f9"]

MONO = "Cascadia Mono, Consolas, DejaVu Sans Mono, monospace"


def A(hexc: str, alpha: float) -> str:
    """#RRGGBB + 透明度 → Qt 能认的 rgba()。
    注意 Qt 的 8 位十六进制是 #AARRGGBB（alpha 在前），
    按网页习惯写 #RRGGBBAA 会被解析成完全不同的颜色。"""
    h = hexc.lstrip("#")
    r, g, b = int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)
    return f"rgba({r},{g},{b},{alpha:.2f})"


# ══════════════════════════════════════════════════════════════════
#  协议编解码（纯函数，可单独测试）
# ══════════════════════════════════════════════════════════════════
def xor_of(s: str) -> int:
    c = 0
    for b in s.encode("ascii", "ignore"):
        c ^= b
    return c


def frame(body: str) -> bytes:
    """给一行加上校验和行尾"""
    return f"{body}*{xor_of(body):02X}\r\n".encode("ascii")


def parse_line(line: str):
    """
    解析一行。返回 (类型, 字段列表) 或 None（校验失败 / 格式错）
    """
    if len(line) < 4 or line[-3] != "*":
        return None
    try:
        want = int(line[-2:], 16)
    except ValueError:
        return None
    body = line[:-3]
    if xor_of(body) != want:
        return None
    parts = body.split(",")
    return parts[0], parts[1:]


# 遥测字段顺序，必须和 debug_link.c 的 DebugLink_SendTelem 一致
TELEM_FIELDS = [
    "ms",
    "cnt_lf", "cnt_lb", "cnt_rf", "cnt_rb",       # 每 12 ms 的编码器计数
    "duty_lf", "duty_lb", "duty_rf", "duty_rb",   # PWM 占空 0~100
    "yaw_c",
    "vis_class", "vis_bear_c", "vis_dist", "vis_conf", "vision_lost",
    "tof0", "tof1", "tof2",
    "intake", "state", "n_col", "fault",
    "cmd_code",
    "srv0", "srv1", "srv2", "mode",
]

# 数字动作码，必须和 firmware/Core/Inc/motion.h 一致
CODE_BRAKE, CODE_FWD, CODE_LEFT, CODE_RIGHT, CODE_TURN_L, CODE_TURN_R = range(6)
CODE_NAMES = {
    0: "刹车", 1: "直行", 2: "左移", 3: "右移", 4: "左转", 5: "右转",
}

STATE_NAMES = {
    0: "—", 1: "初始化与标定", 2: "覆盖搜索", 3: "视觉伺服接近", 4: "扫入收集",
    5: "满载判定", 6: "返仓导航", 7: "角落对接", 8: "卸料与复位",
    9: "脱困", 10: "安全停机",
}
MODE_NAMES = {0: "AUTO", 1: "MANUAL", 2: "E-STOP"}
CLASS_NAMES = {0: "无目标", 1: "红方块", 2: "黄圆柱", 3: "目标区"}
FAULT_BITS = [
    (0, "堵转"), (1, "视觉失效"), (2, "低压"),
    (3, "遥控超时"), (4, "链路错误"),
]

PARAM_DESC = {
    "SPD_TARGET": "闭环目标 计数/12ms",
    "SPD_STEP": "占空比每 tick 增量",
    "D_NEAR_MM": "远/近分界 mm",
    "D_BLIND_MM": "拨轮盲区 mm",
    "ALIGN_TOL_C": "对准阈值 ×100°",
    "WALL_TRIG_MM": "碰壁触发 mm",
    "ROW_PITCH_MM": "弓字形行距 mm",
    "WHEEL_L_MM": "半轴距+半轮距 mm",
    "WHEEL_R_MM": "轮半径 ×100 mm",
    "ENC_PPR": "编码器 PPR (四倍频)",
    "KNOB_SPD": "拨轮工作转速",
    "TELEM_HZ": "遥测频率 Hz",
}


# DEMO 模式下的参数初值，与 debug_link.c 的 s_params[] 一致
PARAM_DEMO = {
    "SPD_TARGET": 11, "SPD_STEP": 1, "D_NEAR_MM": 490, "D_BLIND_MM": 326,
    "ALIGN_TOL_C": 100, "WALL_TRIG_MM": 300, "ROW_PITCH_MM": 1000,
    "WHEEL_L_MM": 150, "WHEEL_R_MM": 3750, "ENC_PPR": 1560,
    "KNOB_SPD": 500, "TELEM_HZ": 20,
}


# ══════════════════════════════════════════════════════════════════
#  串口线程
# ══════════════════════════════════════════════════════════════════
class SerialWorker(QThread):
    line_received = Signal(str)
    opened        = Signal(bool, str)
    closed        = Signal()

    def __init__(self, port: str, baud: int = 115200):
        super().__init__()
        self.port, self.baud = port, baud
        self._ser: serial.Serial | None = None
        self._run = True

    def run(self):
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.05)
        except Exception as e:
            self.opened.emit(False, str(e))
            return
        self.opened.emit(True, self.port)

        buf = bytearray()
        while self._run:
            try:
                data = self._ser.read(512)
            except Exception:
                break
            if data:
                buf.extend(data)
                while True:
                    i = buf.find(b"\n")
                    if i < 0:
                        break
                    raw = bytes(buf[:i]); del buf[:i + 1]
                    try:
                        s = raw.decode("ascii", "ignore").strip("\r\n\x00 ")
                    except Exception:
                        continue
                    if s:
                        self.line_received.emit(s)
            if len(buf) > 4096:      # 防呆：一直收不到换行就清掉
                buf.clear()

        try:
            if self._ser:
                self._ser.close()
        except Exception:
            pass
        self.closed.emit()

    def send(self, data: bytes):
        try:
            if self._ser and self._ser.is_open:
                self._ser.write(data)
        except Exception:
            pass

    def stop(self):
        self._run = False
        self.wait(600)


class DemoWorker(QThread):
    """演示模式：不接硬件，自己造遥测，用来看界面"""
    line_received = Signal(str)
    opened        = Signal(bool, str)
    closed        = Signal()

    def __init__(self):
        super().__init__()
        self._run = True
        self.code = 0
        self.target = 11
        self.mode = 0

    def run(self):
        self.opened.emit(True, "DEMO")
        t0 = time.time()
        cnt  = [0.0] * 4
        duty = [0.0] * 4
        # 与 motion.c 的 WHEEL_DIR 一致
        DIRS = ((0, 0, 0, 0), (1, 1, 1, 1), (-1, 1, 1, -1),
                (1, -1, -1, 1), (-1, -1, 1, 1), (1, 1, -1, -1))
        while self._run:
            t = time.time() - t0
            d = DIRS[self.code if 0 <= self.code < 6 else 0]
            for i in range(4):
                if d[i] == 0:
                    duty[i] = 0.0
                    cnt[i] += (0 - cnt[i]) * 0.5
                else:
                    # 粗略模拟那个 ±1 积分律：占空比追目标，计数跟着占空比走
                    err = self.target - cnt[i]
                    duty[i] = max(0.0, min(100.0, duty[i] + (1 if err > 0 else -1)))
                    cnt[i] += (duty[i] * self.target / 55.0 - cnt[i]) * 0.35
                    cnt[i] += random.gauss(0, 0.35)
            body = ",".join(str(x) for x in [
                "T", int(t * 1000),
                *(int(round(cnt[i])) for i in range(4)),
                *(int(duty[i]) for i in range(4)),
                int(600 * math.sin(t / 4)),
                1, int(3000 * math.sin(t / 3)), int(600 + 300 * math.sin(t / 5)), 88, 0,
                int(1500 + 400 * math.sin(t / 6)), int(800 + 200 * math.cos(t / 4)), 0,
                0, 2, int(t / 10) % 11, 0,
                self.code,
                0, 0, 0, self.mode,
            ])
            self.line_received.emit(f"{body}*{xor_of(body):02X}")
            time.sleep(0.05)
        self.closed.emit()

    def send(self, data: bytes):
        s = data.decode("ascii", "ignore").strip()
        p = parse_line(s)
        if not p:
            return
        typ, f = p
        if typ == "N":
            c = int(f[0])
            if 0 <= c <= 5:
                self.code = c
                self.mode = 1
        elif typ == "A":
            self.mode = 0; self.code = 0
        elif typ == "E":
            self.mode = 2; self.code = 0
        elif typ == "Q":
            self.line_received.emit("V,DEMO,3,72000000" + f"*{xor_of('V,DEMO,3,72000000'):02X}")
            for n, v in PARAM_DEMO.items():
                b = f"K,{n},{v}"
                self.line_received.emit(f"{b}*{xor_of(b):02X}")
        elif typ == "P":
            if f[0] == "SPD_TARGET":
                self.target = max(1, min(500, int(f[1])))
                b = f"K,SPD_TARGET,{self.target}"
            else:
                b = f"K,{f[0]},{f[1]}"
            self.line_received.emit(f"{b}*{xor_of(b):02X}")

    def stop(self):
        self._run = False
        self.wait(600)


# ══════════════════════════════════════════════════════════════════
#  小组件
# ══════════════════════════════════════════════════════════════════
class Card(QGroupBox):
    def __init__(self, title: str):
        super().__init__(title)
        self.setObjectName("card")
        self.setFlat(True)          # 不要 Fusion 自带的那条标题边线，只留样式表画的圆角框


class Stat(QWidget):
    """一行 标签 : 大数值"""
    def __init__(self, label: str, value: str = "—", color: str = C_TEXT):
        super().__init__()
        lay = QHBoxLayout(self); lay.setContentsMargins(0, 2, 0, 2)
        self.color, self.fs = color, 17
        self.k = QLabel(label)
        self.v = QLabel(value)
        self.v.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        lay.addWidget(self.k); lay.addStretch(1); lay.addWidget(self.v)
        self.restyle(17)

    def restyle(self, fs: int):
        self.fs = fs
        self.k.setStyleSheet(f"color:{C_DIM}; font-size:{fs}px;")
        self.v.setStyleSheet(f"color:{self.color}; font-family:{MONO};"
                             f"font-size:{fs + 4}px; font-weight:700;")

    def set(self, text: str, color: str | None = None):
        self.v.setText(text)
        if color and color != self.color:
            self.color = color
            self.restyle(self.fs)


class Pill(QLabel):
    def __init__(self, text="—"):
        super().__init__(text)
        self.setAlignment(Qt.AlignCenter)
        self.fs, self.mode = 17, 0
        self.restyle(17)
        self.set_state(0)

    def restyle(self, fs: int):
        self.fs = fs
        self.setFixedHeight(int(fs * 2.4))
        self.set_state(self.mode)

    def set_state(self, mode: int):
        self.mode = mode
        fs = self.fs
        color = {0: C_ACCENT, 1: C_WARN, 2: C_ERR}.get(mode, C_DIM)
        self.setText(MODE_NAMES.get(mode, "—"))
        self.setStyleSheet(
            f"background:{A(color,0.14)}; color:{color};"
            f"border:1px solid {A(color,0.45)};"
            f"border-radius:{int(fs*1.2)}px; font-weight:800;"
            f"font-size:{fs + 4}px; letter-spacing:2px;")


class LedRow(QWidget):
    """一排故障指示灯"""
    def __init__(self):
        super().__init__()
        lay = QGridLayout(self); lay.setContentsMargins(0, 4, 0, 0)
        lay.setSpacing(8)
        self.leds, self.fs, self.fault = {}, 17, 0
        for i, (bit, name) in enumerate(FAULT_BITS):
            lab = QLabel("● " + name)
            lay.addWidget(lab, i // 2, i % 2)
            self.leds[bit] = lab
        self.update_bits(0)

    def restyle(self, fs: int):
        self.fs = fs; self.update_bits(self.fault)

    def update_bits(self, fault: int):
        self.fault = fault
        for bit, _ in FAULT_BITS:
            on = bool(fault & (1 << bit))
            self.leds[bit].setStyleSheet(
                f"color:{C_ERR if on else C_DIM}; font-size:{self.fs - 2}px;"
                + ("font-weight:800;" if on else ""))


class LegendChip(QLabel):
    """图例：一段色线 + 名字。放在画布外面，不会盖住曲线"""
    def __init__(self, color: str, text: str):
        super().__init__()
        self.c, self.t = color, text
        self.restyle(17)

    def restyle(self, fs: int):
        self.setText(
            f'<span style="font-size:{fs}px; color:{self.c}">&#9644;</span>'
            f'<span style="font-size:{fs - 3}px; color:{C_DIM}"> {self.t}</span>')


class HoldButton(QPushButton):
    """按住生效的方向按钮"""
    def __init__(self, text, on_down, on_up):
        super().__init__(text)
        self.setObjectName("dpad")
        self.pressed.connect(on_down)
        self.released.connect(on_up)
        self.restyle(17)

    def restyle(self, fs: int):
        self.setFixedSize(int(fs * 3.9), int(fs * 3.0))


# ══════════════════════════════════════════════════════════════════
#  主窗口
# ══════════════════════════════════════════════════════════════════
class Console(QMainWindow):
    def __init__(self, demo=False):
        super().__init__()
        self.demo = demo
        self.worker = None
        self.fs = 17          # 基准字号，A- / A+ 可调
        self.setWindowTitle("智能收集小车 · 调试控制台")
        self.resize(1500, 900)

        # 数据缓冲（60 秒 @ 20 Hz）
        N = 1400
        self.t_buf = deque(maxlen=N)
        self.series = {k: deque(maxlen=N) for k in
                       ["cnt_lf", "cnt_lb", "cnt_rf", "cnt_rb",
                        "duty_lf", "duty_lb", "duty_rf", "duty_rb",
                        "vis_bear_deg", "vis_dist", "tof0", "tof1", "tof2",
                        "cmd_code", "yaw_deg", "spd_target"]}
        self.t0 = None
        self._last_ms = 0

        # 遥控状态
        self.remote_on = False
        self.keys: set[int] = set()
        self.btn_code = None       # 鼠标按住某个动作按钮
        self.key_order: list[int] = []
        self.tx_code  = 0          # 当前正在下发的动作码
        self.spd_target = 11       # 从 K,SPD_TARGET 回读，画在曲线上做参考线

        # 统计
        self.n_frames = 0
        self.n_bad = 0
        self.last_fps_t = time.time()
        self.fps = 0.0
        self.csv_w = None
        self.csv_f = None

        self._build_ui()
        self.set_fs(self.fs)
        self.refresh_ports()

        self.tx_timer = QTimer(self); self.tx_timer.timeout.connect(self._tx_tick)
        self.tx_timer.start(50)
        self.ui_timer = QTimer(self); self.ui_timer.timeout.connect(self._ui_tick)
        self.ui_timer.start(80)

        self.KEY_CODE = {
            Qt.Key_W: CODE_FWD,    Qt.Key_S: CODE_BRAKE,
            Qt.Key_A: CODE_LEFT,   Qt.Key_D: CODE_RIGHT,
            Qt.Key_Q: CODE_TURN_L, Qt.Key_E: CODE_TURN_R,
        }
        self.setFocusPolicy(Qt.StrongFocus)
        if demo:
            QTimer.singleShot(200, self.toggle_conn)

    # ─────────────────────────── UI ───────────────────────────
    def _build_ui(self):
        root = QWidget(); self.setCentralWidget(root)
        outer = QVBoxLayout(root); outer.setContentsMargins(12, 10, 12, 10); outer.setSpacing(10)

        # ---------- 顶栏 ----------
        top = QWidget(); top.setObjectName("topbar")
        tl = QHBoxLayout(top); tl.setContentsMargins(12, 8, 12, 8); tl.setSpacing(10)

        def vsep():
            ln = QFrame(); ln.setFrameShape(QFrame.NoFrame)
            ln.setFixedWidth(1); ln.setMinimumHeight(int(self.fs * 1.7))
            ln.setStyleSheet(f"background:{C_BORDER};")
            return ln

        title = QLabel("智能收集小车 · 调试控制台"); title.setObjectName("apptitle")
        lb_lib = QLabel(QT_LIB); lb_lib.setObjectName("hint")
        tl.addWidget(title)
        tl.addWidget(lb_lib)
        tl.addSpacing(10); tl.addWidget(vsep()); tl.addSpacing(10)

        tl.addWidget(QLabel("串口"))
        self.cb_port = QComboBox(); self.cb_port.setMinimumWidth(230)
        tl.addWidget(self.cb_port)
        self.btn_refresh = QPushButton("⟳"); self.btn_refresh.setObjectName("iconbtn")
        self.btn_refresh.clicked.connect(self.refresh_ports)
        tl.addWidget(self.btn_refresh)

        tl.addWidget(QLabel("波特率"))
        self.cb_baud = QComboBox(); self.cb_baud.addItems(["115200", "9600", "57600", "230400"])
        tl.addWidget(self.cb_baud)

        self.btn_conn = QPushButton("连接"); self.btn_conn.setObjectName("primary")
        self.btn_conn.setFixedWidth(90); self.btn_conn.clicked.connect(self.toggle_conn)
        tl.addWidget(self.btn_conn)

        self.lb_link = QLabel("● 未连接")
        self.lb_link.setStyleSheet(f"color:{C_DIM}; font-weight:600;")
        tl.addWidget(self.lb_link)

        tl.addSpacing(10); tl.addWidget(vsep()); tl.addSpacing(10)
        tl.addWidget(QLabel("窗口"))
        self.cb_win = QComboBox()
        for lb, v in [("10 s", 10), ("20 s", 20), ("60 s", 60), ("全部", 0)]:
            self.cb_win.addItem(lb, v)
        self.cb_win.setCurrentIndex(1)
        tl.addWidget(self.cb_win)

        tl.addStretch(1)

        b_am = QPushButton("A-"); b_am.setObjectName("iconbtn")
        b_ap = QPushButton("A+"); b_ap.setObjectName("iconbtn")
        b_am.clicked.connect(lambda: self.set_fs(self.fs - 1))
        b_ap.clicked.connect(lambda: self.set_fs(self.fs + 1))
        tl.addWidget(b_am); tl.addWidget(b_ap)
        tl.addSpacing(10); tl.addWidget(vsep()); tl.addSpacing(10)

        self.lb_fps = QLabel("— fps")
        self.lb_fps.setStyleSheet(f"color:{C_DIM}; font-family:{MONO}; font-size:{self.fs}px;")
        tl.addWidget(self.lb_fps)
        self.lb_bad = QLabel("坏帧 0")
        self.lb_bad.setStyleSheet(f"color:{C_DIM}; font-family:{MONO}; font-size:{self.fs}px;")
        tl.addWidget(self.lb_bad)
        outer.addWidget(top)

        # ---------- 主体 ----------
        body = QHBoxLayout(); body.setSpacing(10)
        outer.addLayout(body, 1)

        # ===== 左栏 =====
        left = QVBoxLayout(); left.setSpacing(8)
        left.setContentsMargins(0, 0, 8, 0)
        inner = QWidget(); inner.setLayout(left)

        lw = QScrollArea()
        lw.setWidget(inner); lw.setWidgetResizable(True)
        lw.setFrameShape(QFrame.NoFrame)
        lw.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.left_panel = lw
        lw.setMinimumWidth(int(self.fs * 27))
        lw.setMaximumWidth(int(self.fs * 32))
        body.addWidget(lw)

        # 状态卡
        c_state = Card("运行状态")
        gl = QVBoxLayout(c_state); gl.setSpacing(2)
        self.pill = Pill(); gl.addWidget(self.pill)
        self.st_state  = Stat("任务状态", "—")
        self.st_col    = Stat("已收集", "0 / 10", C_OK)
        self.st_vis    = Stat("视觉目标", "—")
        self.st_bear   = Stat("方位角", "—")
        self.st_dist   = Stat("距离", "—")
        self.st_up     = Stat("运行时间", "—")
        for s in (self.st_state, self.st_col, self.st_vis,
                  self.st_bear, self.st_dist, self.st_up):
            gl.addWidget(s)
        left.addWidget(c_state)

        # 故障
        c_fault = Card("故障标志")
        fl = QVBoxLayout(c_fault)
        self.leds = LedRow(); fl.addWidget(self.leds)
        left.addWidget(c_fault)

        # 急停
        self.btn_estop = QPushButton("急  停   (空格)")
        self.btn_estop.setObjectName("estop")
        self.btn_estop.clicked.connect(self.send_estop)
        left.addWidget(self.btn_estop)

        # 遥控
        c_rc = Card("手动遥控")
        rl = QVBoxLayout(c_rc); rl.setSpacing(7)

        row = QHBoxLayout()
        self.btn_take = QPushButton("接管遥控"); self.btn_take.setCheckable(True)
        self.btn_take.setObjectName("toggle")
        self.btn_take.toggled.connect(self.set_remote)
        row.addWidget(self.btn_take)
        self.btn_auto = QPushButton("交还自动 (Esc)")
        self.btn_auto.clicked.connect(self.send_auto)
        row.addWidget(self.btn_auto)
        rl.addLayout(row)

        # 六个动作按钮：按住生效，松开回 00 刹车
        grid = QGridLayout(); grid.setSpacing(5)
        mk = lambda txt, code: HoldButton(
            txt, lambda: self._hold(code), lambda: self._hold(None))
        grid.addWidget(mk("↖\nQ 左转", CODE_TURN_L), 0, 0)
        grid.addWidget(mk("↑\nW 直行", CODE_FWD),    0, 1)
        grid.addWidget(mk("↗\nE 右转", CODE_TURN_R), 0, 2)
        grid.addWidget(mk("←\nA 左移", CODE_LEFT),   1, 0)
        grid.addWidget(mk("■\nS 刹车", CODE_BRAKE),  1, 1)
        grid.addWidget(mk("→\nD 右移", CODE_RIGHT),  1, 2)
        wrap = QWidget(); wrap.setLayout(grid)
        wrap.setStyleSheet("background:transparent;")
        rl.addWidget(wrap, 0, Qt.AlignHCenter)

        hint = QLabel("W 直行 · A/D 左右横移 · Q/E 原地转 · S 刹车\n"
                      "先点「接管遥控」，键盘才生效")
        hint.setObjectName("hint"); hint.setAlignment(Qt.AlignCenter)
        rl.addWidget(hint)

        self.lb_txcode = QLabel("下发 00 · 刹车")
        self.lb_txcode.setObjectName("hint")
        self.lb_txcode.setAlignment(Qt.AlignCenter)
        rl.addWidget(self.lb_txcode)

        self.st_cmd = Stat("固件回报", "0 · 刹车")
        rl.addWidget(self.st_cmd)
        left.addWidget(c_rc)

        left.addStretch(1)

        # ===== 右栏 =====
        self.rsplit = QSplitter(Qt.Vertical)
        self.rsplit.setChildrenCollapsible(False)
        self.rsplit.setHandleWidth(8)
        body.addWidget(self.rsplit, 1)

        # 曲线
        self.tabs = QTabWidget()
        self.plots = {}
        self.curves = {}

        self._plot_list = []

        def new_plot(name, ylabel, keys, labels, span=20.0, xlabel=True):
            p = pg.PlotWidget()
            self._plot_list.append({"p": p, "ylabel": ylabel, "xlabel": xlabel,
                                    "keys": list(keys), "span": span})
            vb = p.getViewBox()
            vb.setMouseEnabled(x=False, y=False)   # 禁滚轮缩放：实时曲线被拖走就再也回不来
            vb.setMenuEnabled(False)
            vb.setDefaultPadding(0.0)
            p.setBackground(C_PANEL)
            p.showGrid(x=True, y=True, alpha=0.12)
            p.getPlotItem().setContentsMargins(6, 10, 14, 4)
            if xlabel:
                p.setLabel("bottom", "时间 (s)")
            p.setLabel("left", "")                 # 竖排中文标签又挤又难看，挪到上方标题里
            ax_l = p.getAxis("left")
            ax_l.setWidth(58)                      # 固定轴宽，多张图左边缘对齐
            for ax in (p.getAxis("bottom"), ax_l):
                ax.setPen(C_BORDER); ax.setTextPen(C_DIM)

            hdr = QHBoxLayout(); hdr.setContentsMargins(10, 0, 10, 0); hdr.setSpacing(14)
            cap = QLabel(ylabel); cap.setObjectName("plotcap")
            hdr.addWidget(cap); hdr.addStretch(1)
            for i, (k, lb) in enumerate(zip(keys, labels)):
                if k == "spd_target":           # 目标值画成灰色虚线参考线
                    col = C_DIM
                    pen = pg.mkPen(col, width=1.6, style=Qt.DashLine)
                else:
                    col = SERIES_COLORS[i % len(SERIES_COLORS)]
                    pen = pg.mkPen(col, width=2.2)
                self.curves[k] = p.plot([], [], pen=pen, name=lb)
                hdr.addWidget(LegendChip(col, lb))

            box = QWidget()
            bl = QVBoxLayout(box); bl.setContentsMargins(0, 6, 0, 0); bl.setSpacing(3)
            bl.addLayout(hdr); bl.addWidget(p, 1)
            self.plots[name] = p
            return box

        self.tabs.addTab(new_plot("wheel", "四轮计数 / 12 ms",
                                  ["cnt_lf", "cnt_lb", "cnt_rf", "cnt_rb", "spd_target"],
                                  ["左前", "左后", "右前", "右后", "目标"],
                                  span=30.0), "轮速")

        self.tabs.addTab(new_plot("duty", "PWM 占空 %",
                                  ["duty_lf", "duty_lb", "duty_rf", "duty_rb"],
                                  ["左前", "左后", "右前", "右后"],
                                  span=100.0), "占空比")

        w_vis = QWidget(); lv = QVBoxLayout(w_vis); lv.setContentsMargins(0, 0, 0, 0); lv.setSpacing(6)
        lv.addWidget(new_plot("vis_a", "角度 °", ["vis_bear_deg", "yaw_deg"],
                              ["视觉方位角", "IMU 航向"], span=20.0, xlabel=False))
        lv.addWidget(new_plot("vis_d", "距离 mm", ["vis_dist", "tof0", "tof1", "tof2"],
                              ["视觉距离", "ToF①", "ToF②", "ToF③"], span=200.0))
        self.tabs.addTab(w_vis, "视觉 / 测距")

        w_cmd = new_plot("cmd", "当前动作码", ["cmd_code"], ["动作"], span=6.0)
        self.plots["cmd"].getAxis("left").setTicks(
            [[(k, f"{k} {v}") for k, v in CODE_NAMES.items()]])
        for _e in self._plot_list:
            if _e["p"] is self.plots["cmd"]:
                _e["yfix"] = (-0.4, 5.4)
        self.tabs.addTab(w_cmd, "指令")
        self.tabs.setMinimumHeight(220)
        self.rsplit.addWidget(self.tabs)

        # 参数 + 日志
        low_w = QWidget()
        lower = QHBoxLayout(low_w); lower.setContentsMargins(0, 0, 0, 0); lower.setSpacing(10)
        low_w.setMinimumHeight(int(self.fs * 2.3 * 5) + 110)
        self.rsplit.addWidget(low_w)
        self.rsplit.setStretchFactor(0, 3)
        self.rsplit.setStretchFactor(1, 2)
        # 按比例分，别写死像素：1080p 和 768p 的合适高度差很多
        QTimer.singleShot(0, lambda: self.rsplit.setSizes(
            [int(self.rsplit.height() * 0.60), int(self.rsplit.height() * 0.40)]))

        c_par = Card("参数（双击「值」修改，回车下发）")
        pl = QVBoxLayout(c_par)
        self.tbl = QTableWidget(0, 3)
        self.tbl.setHorizontalHeaderLabels(["参数", "值", "说明"])
        self.tbl.verticalHeader().setVisible(False)
        self.tbl.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.tbl.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.tbl.horizontalHeader().setSectionResizeMode(2, QHeaderView.Stretch)
        self.tbl.setEditTriggers(QAbstractItemView.DoubleClicked | QAbstractItemView.EditKeyPressed)
        self.tbl.itemChanged.connect(self.on_param_edit)
        self.tbl.setAlternatingRowColors(True)
        self.tbl.verticalHeader().setDefaultSectionSize(int(self.fs * 2.3))
        self.tbl.setMinimumHeight(int(self.fs * 2.3 * 4))
        pl.addWidget(self.tbl)
        rowb = QHBoxLayout()
        b_sync = QPushButton("重新读取全部 (Q)"); b_sync.clicked.connect(self.send_query)
        rowb.addWidget(b_sync)
        self.chk_csv = QCheckBox("记录 CSV"); self.chk_csv.toggled.connect(self.toggle_csv)
        rowb.addWidget(self.chk_csv); rowb.addStretch(1)
        pl.addLayout(rowb)
        lower.addWidget(c_par, 3)

        c_log = Card("日志")
        ll = QVBoxLayout(c_log)
        self.log = QPlainTextEdit(); self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(500)
        self.log.setObjectName("logbox")
        ll.addWidget(self.log)
        lower.addWidget(c_log, 2)

        self._param_rows = {}
        self._suppress_edit = False

    def set_fs(self, fs: int):
        self.fs = max(11, min(30, fs))
        f = QApplication.instance().font(); f.setPixelSize(self.fs)
        QApplication.instance().setFont(f)
        for w in self.findChildren(QWidget):
            if hasattr(w, "restyle"):
                w.restyle(self.fs)
        self.btn_estop.setFixedHeight(int(self.fs * 2.7))
        self.left_panel.setMinimumWidth(int(self.fs * 26))
        self.left_panel.setMaximumWidth(int(self.fs * 34))
        self.tbl.verticalHeader().setDefaultSectionSize(int(self.fs * 2.3))
        self.tbl.setMinimumHeight(int(self.fs * 2.3 * 4))
        self._restyle_plots()
        self._apply_style()

    def _restyle_plots(self):
        tf = QFont(); tf.setPixelSize(max(10, self.fs - 3))
        for _e in self._plot_list:
            p, ylabel = _e["p"], _e["ylabel"]
            for ax in ("bottom", "left"):
                p.getAxis(ax).setStyle(tickFont=tf)
                if hasattr(p.getAxis(ax), "setTickFont"):
                    p.getAxis(ax).setTickFont(tf)
            if _e["xlabel"]:
                p.setLabel("bottom", "时间 (s)",
                           **{"font-size": f"{self.fs - 2}px", "color": C_DIM})

    def _apply_style(self):
        fs = self.fs
        self.setStyleSheet(f"""
        QMainWindow, QWidget {{ background:{C_BG}; color:{C_TEXT};
            font-family:'Microsoft YaHei UI','PingFang SC',sans-serif;
            font-size:{fs}px; }}
        QLabel {{ background:transparent; }}
        QLabel#apptitle {{ font-size:{fs + 6}px; font-weight:800; color:{C_TEXT}; }}
        QLabel#plotcap {{ color:{C_DIM}; font-size:{fs - 1}px; font-weight:700; }}
        QLabel#hint {{ color:{C_DIM}; font-size:{fs - 3}px; }}
        QLabel#vnum {{ font-family:{MONO}; color:{C_ACCENT};
            font-size:{fs + 2}px; font-weight:700; }}
        #topbar {{ background:{C_PANEL}; border:1px solid {C_BORDER}; border-radius:12px; }}
        QGroupBox#card {{ background:{C_PANEL}; border:1px solid {C_BORDER};
            border-radius:12px; margin-top:{fs + 2}px;
            padding:{fs // 2 + 6}px {fs - 5}px {fs // 2 + 2}px {fs - 5}px;
            font-size:{fs}px; font-weight:700; color:{C_DIM}; }}
        QGroupBox#card::title {{ subcontrol-origin:margin; left:14px; padding:0 8px;
            color:{C_DIM}; }}
        QPushButton {{ background:{C_PANEL2}; border:1px solid {C_BORDER};
            border-radius:9px; padding:{fs // 2}px {fs}px; color:{C_TEXT};
            font-size:{fs}px; }}
        QPushButton:hover {{ background:#242a35; border-color:{A(C_ACCENT,0.40)}; }}
        QPushButton:pressed {{ background:{A(C_ACCENT,0.20)}; }}
        QPushButton#iconbtn {{ padding:{fs // 3}px {fs // 2}px;
            min-width:{fs + 12}px; font-weight:700; }}
        QPushButton#primary {{ background:{C_ACCENT}; color:#08111d; font-weight:800;
            border:none; }}
        QPushButton#primary:hover {{ background:#68b2ff; }}
        QPushButton#estop {{ background:{A(C_ERR,0.12)}; color:{C_ERR};
            border:1px solid {A(C_ERR,0.55)};
            font-size:{fs + 5}px; font-weight:900; letter-spacing:3px; }}
        QPushButton#estop:hover {{ background:{A(C_ERR,0.22)}; }}
        QPushButton#dpad {{ padding:0px; font-size:{fs - 1}px;
            font-weight:700; background:{C_PANEL2}; }}
        QPushButton#dpad:pressed {{ background:{A(C_ACCENT,0.28)};
            border-color:{A(C_ACCENT,0.70)}; color:{C_ACCENT}; }}
        QPushButton#toggle:checked {{ background:{A(C_WARN,0.16)}; color:{C_WARN};
            border-color:{A(C_WARN,0.55)}; font-weight:800; }}
        QComboBox, QPlainTextEdit, QTableWidget {{ background:{C_PANEL2};
            border:1px solid {C_BORDER}; border-radius:9px; padding:{fs // 3}px {fs // 2}px;
            font-size:{fs}px; selection-background-color:{A(C_ACCENT,0.33)}; }}
        QComboBox::drop-down {{ border:none; width:20px; }}
        QPlainTextEdit#logbox {{ font-family:{MONO}; font-size:{fs - 2}px; }}
        QHeaderView::section {{ background:{C_PANEL}; color:{C_DIM};
            border:none; border-bottom:1px solid {C_BORDER};
            padding:{fs // 2}px; font-size:{fs - 1}px; }}
        QTableWidget {{ gridline-color:{C_BORDER}; alternate-background-color:#1a1e26;
            font-size:{fs}px; }}
        QTableWidget::item {{ padding:2px 8px; }}
        QTabWidget::pane {{ border:1px solid {C_BORDER}; border-radius:12px;
            background:{C_PANEL}; }}
        QTabBar::tab {{ background:transparent; color:{C_DIM};
            padding:{fs // 2 + 2}px {fs + 8}px; font-size:{fs}px;
            border-top-left-radius:10px; border-top-right-radius:10px; }}
        QTabBar::tab:selected {{ background:{C_PANEL}; color:{C_ACCENT};
            border:1px solid {C_BORDER}; border-bottom:none; font-weight:700; }}
        QSlider::groove:horizontal {{ height:6px; background:{C_BORDER}; border-radius:3px; }}
        QSlider::handle:horizontal {{ width:{fs + 3}px; margin:-{fs // 2}px 0;
            border-radius:{(fs + 3) // 2}px; background:{C_ACCENT}; }}
        QCheckBox {{ color:{C_DIM}; font-size:{fs}px; spacing:10px; padding:2px; }}
        QCheckBox::indicator {{ width:{fs}px; height:{fs}px; }}
        QScrollArea {{ background:transparent; border:none; }}
        QSplitter::handle {{ background:transparent; }}
        QSplitter::handle:hover {{ background:{A(C_ACCENT,0.30)}; border-radius:3px; }}
        QScrollBar:vertical {{ background:transparent; width:11px; }}
        QScrollBar::handle:vertical {{ background:{C_BORDER}; border-radius:5px; }}
        QScrollBar::add-line, QScrollBar::sub-line {{ height:0; }}
        """)

    # ─────────────────────── 串口 ───────────────────────
    def refresh_ports(self):
        cur = self.cb_port.currentText()
        self.cb_port.clear()
        for p in serial.tools.list_ports.comports():
            self.cb_port.addItem(f"{p.device} — {p.description}", p.device)
        if self.cb_port.count() == 0:
            self.cb_port.addItem("（没有可用串口）", None)
        i = self.cb_port.findText(cur)
        if i >= 0:
            self.cb_port.setCurrentIndex(i)

    def toggle_conn(self):
        if self.worker:
            self.worker.stop(); self.worker = None
            self.t0 = None                      # 重连时时间基准要重来
            self._last_ms = 0
            self.t_buf.clear()
            for dq in self.series.values(): dq.clear()
            self.btn_conn.setText("连接")
            self.lb_link.setText("● 未连接")
            self.lb_link.setStyleSheet(f"color:{C_DIM}; font-weight:600;")
            return

        if self.demo:
            self.worker = DemoWorker()
        else:
            dev = self.cb_port.currentData()
            if not dev:
                QMessageBox.warning(self, "没有串口", "先在下拉框里选一个串口。\n"
                                    "蓝牙配对后要选「传出」那个 COM 口。")
                return
            self.worker = SerialWorker(dev, int(self.cb_baud.currentText()))

        self.worker.line_received.connect(self.on_line)
        self.worker.opened.connect(self.on_opened)
        self.worker.start()

    def on_opened(self, ok: bool, msg: str):
        if ok:
            self.btn_conn.setText("断开")
            self.lb_link.setText(f"● 已连接 {msg}")
            self.lb_link.setStyleSheet(f"color:{C_OK}; font-weight:600;")
            self.logp(f"已连接 {msg}", C_OK)
            QTimer.singleShot(300, self.send_query)
        else:
            self.logp(f"打开失败：{msg}", C_ERR)
            QMessageBox.critical(self, "打开串口失败", msg)
            self.worker = None
            self.btn_conn.setText("连接")

    def tx(self, body: str):
        if self.worker:
            self.worker.send(frame(body))

    # ─────────────────────── 收到一行 ───────────────────────
    def on_line(self, line: str):
        if self.worker is None:      # 断开瞬间事件队列里还压着的旧帧，丢掉
            return
        p = parse_line(line)
        if p is None:
            self.n_bad += 1
            return
        typ, f = p

        if typ == "T":
            self.on_telem(f)
        elif typ == "K" and len(f) >= 2:
            self.set_param_row(f[0], f[1])
            if f[0] == "SPD_TARGET":
                try:
                    self.spd_target = int(f[1])
                except ValueError:
                    pass
        elif typ == "L":
            self.logp("固件：" + ",".join(f), C_WARN)
        elif typ == "V":
            self.logp(f"固件版本 {f[0]}  协议 v{f[1] if len(f) > 1 else '?'}  "
                      f"主频 {int(f[2])/1e6:.0f} MHz" if len(f) > 2 else "V " + ",".join(f), C_ACCENT)

    def on_telem(self, f: list[str]):
        if len(f) < len(TELEM_FIELDS):
            self.n_bad += 1
            return
        try:
            d = {k: int(v) for k, v in zip(TELEM_FIELDS, f)}
        except ValueError:
            self.n_bad += 1
            return

        self.n_frames += 1

        # 时间基准：主控重启 / 上一次连接的残留帧都会让 ms 倒退，
        # 倒退就把整个缓冲清掉重来，否则 x 轴会出现负数、二分查窗口也会失效。
        ms = d["ms"]
        if self.t0 is None or ms < self._last_ms or ms - self._last_ms > 10000:
            self.t0 = ms
            self.t_buf.clear()
            for dq in self.series.values():
                dq.clear()
        self._last_ms = ms
        t = (ms - self.t0) / 1000.0

        self.t_buf.append(t)
        for k in ("cnt_lf", "cnt_lb", "cnt_rf", "cnt_rb",
                  "duty_lf", "duty_lb", "duty_rf", "duty_rb",
                  "cmd_code", "tof0", "tof1", "tof2"):
            self.series[k].append(d[k])
        self.series["vis_bear_deg"].append(d["vis_bear_c"] / 100.0)
        self.series["yaw_deg"].append(d["yaw_c"] / 100.0)
        self.series["vis_dist"].append(0 if d["vis_dist"] == 65535 else d["vis_dist"])
        self.series["spd_target"].append(self.spd_target)

        self.last = d
        if self.csv_w:
            self.csv_w.writerow([d[k] for k in TELEM_FIELDS])

    # ─────────────────────── 定时刷新 ───────────────────────
    def _ui_tick(self):
        now = time.time()
        dt = now - self.last_fps_t
        if dt >= 1.0:
            self.fps = self.n_frames / dt
            self.n_frames = 0
            self.last_fps_t = now
            self.lb_fps.setText(f"{self.fps:4.1f} fps")
            self.lb_fps.setStyleSheet(
                f"color:{C_OK if self.fps > 5 else C_ERR}; font-family:{MONO};"
                f"font-size:{self.fs}px;")
            self.lb_bad.setText(f"坏帧 {self.n_bad}")
            self.lb_bad.setStyleSheet(
                f"color:{C_ERR if self.n_bad else C_DIM}; font-family:{MONO};"
                f"font-size:{self.fs}px;")

        if not self.t_buf:
            return
        xs = list(self.t_buf)
        win = self.cb_win.currentData()
        i0 = 0
        if win:
            t_end = xs[-1]
            lo, hi = 0, len(xs)
            while lo < hi:                      # 二分找窗口起点
                mid = (lo + hi) // 2
                if xs[mid] < t_end - win: lo = mid + 1
                else: hi = mid
            i0 = lo
        xw = xs[i0:]
        for k, c in self.curves.items():
            c.setData(xw, list(self.series[k])[i0:])
        if xw:
            x1 = xw[-1]
            x0 = max(xw[0], x1 - win) if win else xw[0]
            if x1 - x0 < 1.0: x0 = x1 - 1.0      # 起步阶段别把轴压成一条线
            for e in self._plot_list:
                e["p"].setXRange(x0, x1, padding=0.0)
                lo = hi = None
                for k in e["keys"]:
                    seg = list(self.series[k])[i0:]
                    if not seg:
                        continue
                    a, b = min(seg), max(seg)
                    lo = a if lo is None else min(lo, a)
                    hi = b if hi is None else max(hi, b)
                if lo is None:
                    continue
                if e.get("yfix"):                # 动作码这种离散量，量程写死
                    e["p"].setYRange(e["yfix"][0], e["yfix"][1], padding=0.0)
                    continue
                span = e["span"]
                if hi - lo < span:               # 只有噪声时别把量程放大到 ±3
                    c = (hi + lo) / 2.0
                    lo, hi = c - span / 2.0, c + span / 2.0
                pad = (hi - lo) * 0.10
                e["p"].setYRange(lo - pad, hi + pad, padding=0.0)

        d = getattr(self, "last", None)
        if not d:
            return
        self.pill.set_state(d["mode"])
        self.st_state.set(f'{d["state"]} · {STATE_NAMES.get(d["state"], "?")}')
        self.st_col.set(f'{d["n_col"]} / 10', C_OK if d["n_col"] else C_DIM)
        cls = d["vis_class"]
        self.st_vis.set(CLASS_NAMES.get(cls, "?"),
                        C_DIM if (cls == 0 or d["vision_lost"]) else C_OK)
        self.st_bear.set("—" if d["vision_lost"] else f'{d["vis_bear_c"]/100:+.2f}°')
        self.st_dist.set("—" if d["vis_dist"] == 65535 else f'{d["vis_dist"]} mm')
        self.st_up.set(f'{d["ms"]/1000:.0f} s')
        self.leds.update_bits(d["fault"])
        cc = d["cmd_code"]
        self.st_cmd.set(f'{cc} · {CODE_NAMES.get(cc, "?")}',
                        C_DIM if cc == 0 else C_WARN)

    # ─────────────────────── 遥控 ───────────────────────
    def set_remote(self, on: bool):
        self.remote_on = on
        self.btn_take.setText("● 遥控中" if on else "接管遥控")
        if not on:
            self.keys.clear(); self.btn_code = None
            self.send_auto()
        else:
            self.logp("已接管遥控：松开所有键会发 N,00 刹车；"
                      "完全断联 200 ms 主控也会自动刹车", C_WARN)
        self.setFocus()

    def _hold(self, code):
        """鼠标按住动作按钮 = 该动作；松开 = 回刹车"""
        self.btn_code = code

    KEY_CODE = None      # 在 __init__ 之后由 _init_keymap 填，避免类体里引用 Qt

    def _wanted_code(self) -> int:
        """键盘 + 鼠标合成出当前该发的动作码。多个键同时按时取后按下的那个。"""
        if self.btn_code is not None:
            return self.btn_code
        for k in reversed(self.key_order):
            if k in self.keys:
                return self.KEY_CODE[k]
        return CODE_BRAKE

    def _tx_tick(self):
        if not (self.worker and self.remote_on):
            return
        code = self._wanted_code()
        self.tx(f"N,{code:02d}")
        if code != self.tx_code:
            self.tx_code = code
            self.lb_txcode.setText(f"下发 {code:02d} · {CODE_NAMES[code]}")

    def keyPressEvent(self, e: QKeyEvent):
        if e.isAutoRepeat():
            return
        k = e.key()
        if k == Qt.Key_Space:
            self.send_estop(); return
        if k == Qt.Key_Escape:
            self.btn_take.setChecked(False); return
        if self.remote_on and k in self.KEY_CODE:
            self.keys.add(k)
            self.key_order.append(k)
        super().keyPressEvent(e)

    def keyReleaseEvent(self, e: QKeyEvent):
        if e.isAutoRepeat():
            return
        k = e.key()
        self.keys.discard(k)
        self.key_order = [x for x in self.key_order if x != k]
        super().keyReleaseEvent(e)

    def send_estop(self):
        self.btn_take.setChecked(False)
        self.tx("E")
        self.logp("已发送急停 E", C_ERR)

    def send_auto(self):
        self.tx("A")

    def send_query(self):
        self.tx("Q")

    # ─────────────────────── 参数表 ───────────────────────
    def set_param_row(self, name: str, value: str):
        self._suppress_edit = True
        if name not in self._param_rows:
            r = self.tbl.rowCount(); self.tbl.insertRow(r)
            it_n = QTableWidgetItem(name); it_n.setFlags(Qt.ItemIsEnabled)
            it_v = QTableWidgetItem(value)
            it_v.setFont(QFont("Consolas", 10))
            it_v.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
            it_d = QTableWidgetItem(PARAM_DESC.get(name, "")); it_d.setFlags(Qt.ItemIsEnabled)
            it_d.setForeground(QColor(C_DIM))
            self.tbl.setItem(r, 0, it_n); self.tbl.setItem(r, 1, it_v); self.tbl.setItem(r, 2, it_d)
            self._param_rows[name] = r
        else:
            r = self._param_rows[name]
            it_v = self.tbl.item(r, 1)
            if it_v.text() != value:
                it_v.setText(value)
            it_v.setBackground(QColor(61, 220, 151, 55))     # 闪一下绿
            QTimer.singleShot(700, lambda i=it_v: i.setBackground(QColor(0, 0, 0, 0)))
        self._suppress_edit = False

    def on_param_edit(self, item: QTableWidgetItem):
        if self._suppress_edit or item.column() != 1:
            return
        name = self.tbl.item(item.row(), 0).text()
        try:
            v = int(item.text().strip())
        except ValueError:
            self.logp(f"{name}: 只能填整数（协议用 ×1000 定点）", C_ERR)
            self.send_query(); return
        self.tx(f"P,{name},{v}")
        self.logp(f"下发 {name} = {v}，等待回读…", C_DIM)

    # ─────────────────────── CSV / 日志 ───────────────────────
    def toggle_csv(self, on: bool):
        if on:
            fn, _ = QFileDialog.getSaveFileName(
                self, "保存遥测数据",
                time.strftime("telemetry_%Y%m%d_%H%M%S.csv"), "CSV (*.csv)")
            if not fn:
                self.chk_csv.setChecked(False); return
            self.csv_f = open(fn, "w", newline="", encoding="utf-8-sig")
            self.csv_w = csv.writer(self.csv_f)
            self.csv_w.writerow(TELEM_FIELDS)
            self.logp(f"开始记录 → {os.path.basename(fn)}", C_OK)
        else:
            self.csv_w = None
            if self.csv_f:
                self.csv_f.close(); self.csv_f = None
                self.logp("停止记录", C_DIM)

    def logp(self, text: str, color: str = C_TEXT):
        ts = time.strftime("%H:%M:%S")
        self.log.appendHtml(
            f'<span style="color:{C_DIM}">{ts}</span> '
            f'<span style="color:{color}">{text}</span>')

    def closeEvent(self, e):
        if self.worker:
            self.worker.stop()
        if self.csv_f:
            self.csv_f.close()
        e.accept()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--demo", action="store_true", help="不接硬件，用假数据看界面")
    args = ap.parse_args()

    pg.setConfigOptions(antialias=True)

    # 高分屏缩放：PyQt5 默认不开，不开的话在 2K/4K 屏上字会小得看不清
    for _a in ("AA_EnableHighDpiScaling", "AA_UseHighDpiPixmaps"):
        if hasattr(Qt, _a):
            QApplication.setAttribute(getattr(Qt, _a), True)

    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    _f = app.font(); _f.setPixelSize(17); app.setFont(_f)
    w = Console(demo=args.demo)
    w.show()
    print(f"[Qt 绑定] {QT_LIB}")
    sys.exit(app.exec_() if hasattr(app, "exec_") else app.exec())


if __name__ == "__main__":
    main()

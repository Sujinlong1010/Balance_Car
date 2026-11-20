# -*- coding: utf-8 -*-
"""HC-05 Bluetooth PID tuner for the STM32F103 balance car.

Runs on D:\\Anaconda\\python.exe (Python 3.12) with ONLY tkinter + pyserial.
No matplotlib/numpy: curves are drawn directly on a tkinter Canvas.

Wire protocol (ASCII, LF terminated, x100 fixed point on MCU, no float printf):
  PC  -> MCU : SET,<kp*100>,<ki*100>,<kd*100>   角度环
               SETV,<kp*10000>,<ki*10000>       速度环
               SETT,<kp*100>,<ki*100>           转向环差速 PI
               GET / GETV / GETT                读取三组增益
               CAL   BAL,<0|1>   BST
  MCU -> PC  : ACK,..   ACKV,..   ACKT,..   LOG,<text>
               BST,<state>,<angle>,<offset>,<duty>,P,I,D,<cal%>,<vel>,<turn>
               T,<ms>,<angle>,<target>,P,I,D,<pwm>,<vel>,<turn>,<enc_tps>,<gyroz>
               其中 vel/turn/enc_tps/gyroz 已按各自单位换算，PWM 为原始占空比
  目标速度/差速、角度微调、死区、输出方向、保护、电机极性、编码器方向、轮动力
  补偿均已固化在固件常量中，不再通过上位机下发。
  Motors stay OFF on power-up: hold upright -> CAL -> BAL,1. BAL,0 = stop.

Double-click "启动PID调试助手.bat" or the desktop shortcut to launch.
"""
import collections
import csv
import json
import math
import os
import queue
import sys
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, scrolledtext, ttk

import serial
import serial.tools.list_ports

CHANNELS = [
    ("angle",  "Angle 实际角",  "#e03131"),
    ("target", "Target 目标角", "#2b8a3e"),
    ("p",      "P 分量",        "#f08c00"),
    ("i",      "I 分量",        "#adb5bd"),
    ("d",      "D 分量",        "#9c36b5"),
    ("pwm",    "PWM 输出",      "#212529"),
    ("vel",    "Vel 速度环角",  "#0b7285"),
    ("turn",   "Turn 转向环",   "#e8590c"),
]

# Extra CSV columns for offline diagnosis: encoder pulse rate, yaw rate,
# balance state, calibrated offset, and the gain snapshot that was active at
# that instant (so a CSV is self-describing even after many retunes).
EXTRA_COLS = ["enc_tps", "gyroz", "state", "offset",
              "kp", "ki", "kd", "kvp", "kvi", "ktp", "kti"]

BAUDS = ["9600", "19200", "38400", "57600", "115200", "230400", "460800", "921600"]
WINDOWS = ["5", "10", "15", "30", "60"]
EXPORT_LENS = ["10", "30", "60", "120", "240"]
BUF_POINTS = 6000          # ring buffer depth per channel
TELEM_MS = 40              # firmware PIDTUNE_TELEMETRY_MS -> 25 frames/s
BUF_SECS = BUF_POINTS * TELEM_MS / 1000.0   # 240 s = whole history
FONT = ("Microsoft YaHei", 9)
MONO = ("Consolas", 9)


def build_set_command(kp, ki, kd):
    """KP/KI/KD (float) -> 'SET,<x100>,<x100>,<x100>\\n'."""
    return "SET,%d,%d,%d\n" % (int(round(float(kp) * 100)),
                               int(round(float(ki) * 100)),
                               int(round(float(kd) * 100)))


def build_setv_command(kp, ki):
    """Speed gains -> 'SETV,<kp*10000>,<ki*10000>\\n' (0.0019 needs x10000)."""
    return "SETV,%d,%d\n" % (int(round(float(kp) * 10000)),
                             int(round(float(ki) * 10000)))


def build_sett_command(kp, ki):
    """Turn loop is encoder-difference PI in the tutorial: SETT,<kp*100>,<ki*100>."""
    return "SETT,%d,%d\n" % (int(round(float(kp) * 100)),
                             int(round(float(ki) * 100)))


BST_STATES = {0: "IDLE", 1: "CAL", 2: "RUN", 3: "FAULT"}


def parse_bst(line):
    """'BST,state,angle,offset,duty,p,i,d,cal[,vel,turn]' -> dict."""
    if not line.startswith("BST,"):
        return None
    parts = line.split(",")
    if len(parts) not in (9, 11):
        return None
    try:
        v = [int(x) for x in parts[1:]]
    except ValueError:
        return None
    return {
        "state": v[0],
        "angle": v[1] / 100.0,
        "offset": v[2] / 100.0,
        "duty": v[3],
        "p": v[4] / 100.0,
        "i": v[5] / 100.0,
        "d": v[6] / 100.0,
        "cal": v[7],
        "vel": v[8] / 100.0 if len(v) > 8 else 0.0,
        "turn": v[9] if len(v) > 9 else 0,
    }


def parse_ack(line):
    """'ACK,1250,80,4500' -> (12.5, 0.8, 45.0), or None."""
    if not line.startswith("ACK,"):
        return None
    parts = line.split(",")
    if len(parts) != 4:
        return None
    try:
        return tuple(float(x) / 100.0 for x in parts[1:])
    except ValueError:
        return None


def parse_ack_v(line):
    """'ACKV,-3800,-19' -> (-0.38, -0.0019), or None."""
    if not line.startswith("ACKV,"):
        return None
    parts = line.split(",")
    if len(parts) != 3:
        return None
    try:
        return tuple(float(x) / 10000.0 for x in parts[1:])
    except ValueError:
        return None


def parse_ack_t(line):
    """'ACKT,0,-60' -> (0.0, -0.6), or None."""
    if not line.startswith("ACKT,"):
        return None
    parts = line.split(",")
    if len(parts) != 3:
        return None
    try:
        return tuple(float(x) / 100.0 for x in parts[1:])
    except ValueError:
        return None


def parse_telemetry(line):
    """'T,<ms>,<a>,<t>,<p>,<i>,<d>,<pwm>[,<vel>,<turn>,<enc>,<gyroz>]'."""
    if not line.startswith("T,"):
        return None
    parts = line.split(",")
    if len(parts) < 8:
        return None
    try:
        v = [float(x) for x in parts[1:]]
    except ValueError:
        return None
    return {
        "t_ms": v[0],
        "angle": v[1] / 100.0,
        "target": v[2] / 100.0,
        "p": v[3] / 100.0,
        "i": v[4] / 100.0,
        "d": v[5] / 100.0,
        "pwm": v[6],   # raw PWM -7200..7200, NOT x100
        "vel": v[7] / 100.0 if len(v) > 7 else 0.0,
        "turn": v[8] if len(v) > 8 else 0.0,
        "enc_tps": v[9] if len(v) > 9 else 0.0,
        "gyroz": v[10] / 100.0 if len(v) > 10 else 0.0,
    }


def _selftest():
    assert build_set_command(12.5, 0.8, 45.0) == "SET,1250,80,4500\n"
    assert build_set_command(0, 0, 0) == "SET,0,0,0\n"
    assert build_setv_command(2.0, 0.05) == "SETV,20000,500\n"
    assert build_sett_command(4.0, 3.0) == "SETT,400,300\n"
    assert parse_ack("ACK,1250,80,4500") == (12.5, 0.8, 45.0)
    assert parse_ack_v("ACKV,20000,500") == (2.0, 0.05)
    assert parse_ack_t("ACKT,400,300") == (4.0, 3.0)
    assert parse_ack("LOG,hello") is None
    t = parse_telemetry("T,1000,1250,0,120,30,-40,3500,-150,25,600,0")
    assert t is not None
    assert t["t_ms"] == 1000.0 and t["angle"] == 12.5
    assert t["p"] == 1.2 and t["i"] == 0.3 and t["d"] == -0.4 and t["pwm"] == 3500.0
    assert t["vel"] == -1.5 and t["turn"] == 25.0 and t["enc_tps"] == 600.0
    # Old 7-value frames still parse, with zeroed new channels.
    assert parse_telemetry("T,1000,1250,0,120,30,-40,3500")["vel"] == 0.0
    assert parse_telemetry("T,1,2,3") is None
    assert parse_telemetry("ACK,1,2,3") is None
    b = parse_bst("BST,2,150,-120,345,200,0,50,100,-150,25")
    assert b is not None
    assert b["state"] == 2 and BST_STATES[b["state"]] == "RUN"
    assert b["angle"] == 1.5 and b["offset"] == -1.2
    assert b["duty"] == 345 and b["p"] == 2.0 and b["d"] == 0.5
    assert b["vel"] == -1.5 and b["turn"] == 25
    assert parse_bst("BST,0,1") is None
    print("selftest OK (protocol helpers)")


# ---------------------------------------------------------------------------
# Parameter snapshots ("存档"): persisted next to this script so they travel
# with the repo. Each snapshot: name, timestamp, the 7 gains and deadband.
# ---------------------------------------------------------------------------
PRESET_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "presets.json")
PRESET_KEYS = ("kp", "ki", "kd", "kvp", "kvi", "ktp", "kti", "deadband")


def load_presets():
    """Return a list of snapshot dicts; tolerate missing/corrupt files."""
    try:
        with open(PRESET_FILE, "r", encoding="utf-8") as fh:
            data = json.load(fh)
        if isinstance(data, list):
            return [p for p in data if isinstance(p, dict)]
    except (OSError, ValueError):
        pass
    return []


def save_presets(presets):
    """Atomically write the snapshot list; return (ok, error_message)."""
    try:
        tmp = PRESET_FILE + ".tmp"
        with open(tmp, "w", encoding="utf-8") as fh:
            json.dump(presets, fh, ensure_ascii=False, indent=2)
        os.replace(tmp, PRESET_FILE)
        return True, ""
    except OSError as exc:
        return False, str(exc)


class PIDTuner:
    def __init__(self, root):
        self.root = root
        root.title("平衡车 PID 调试助手 · HC-05 蓝牙")
        root.geometry("1120x720")
        root.minsize(940, 620)

        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure(".", font=FONT)
        style.configure("TLabelframe.Label", font=("Microsoft YaHei", 9, "bold"))
        # 选中=实心黑框、未选=空框；叉号颜色设成黑，埋进黑框里看不见
        style.configure("TCheckbutton", indicatorforeground="black")
        style.map("TCheckbutton", indicatorbackground=[("selected", "black")])

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value="115200")
        self.status_var = tk.StringVar(value="未连接")
        self.ack_var = tk.StringVar(value="参数未同步")
        self.value_var = tk.StringVar(value="Angle --   Target --   P --   I --   D --   PWM --")
        # 默认值 = 实车调出的最佳参数（已固化进固件），基于江科大三环结构微调
        # （×72 映射到本工程 7200 PWM）。角度环 3.30/0.15/3.00，
        # 速度环 0.80/0.065，转向环编码器差速 PI 3.00/4.00。
        # 角度环实际值在 CAL 后已经清零，Target 直接等于速度环输出 + TARGET
        # 微调，所以波形可直接比较。
        self.pid_vars = {"kp": tk.StringVar(value="3.30"),
                         "ki": tk.StringVar(value="0.15"),
                         "kd": tk.StringVar(value="3.00")}
        self.speed_vars = {"kvp": tk.StringVar(value="0.8000"),
                           "kvi": tk.StringVar(value="0.0650")}
        self.turn_vars = {"ktp": tk.StringVar(value="3.00"),
                          "kti": tk.StringVar(value="4.00")}
        # 运行中滑动调参：拖动滑块即刻下发对应那一组，无需再点「下发参数」。
        # (key, 显示名, 下限, 上限, 滑块粗步进, 键盘细步进, 显示格式)
        # 细步进不可小于协议的量化精度：角度/转向 x100 -> 最小 0.01；
        # 速度 x10000 -> 最小 0.0001。↑/↓ 用细步进，Shift 用粗步进，Ctrl 用 10×粗步进。
        self.slider_specs = (
            ("kp",  "角度KP", 0.0, 15.0,  0.10, 0.01,  "%.2f"),
            ("ki",  "角度KI", 0.0, 1.00,  0.01, 0.01,  "%.2f"),
            ("kd",  "角度KD", 0.0, 15.0,  0.10, 0.01,  "%.2f"),
            ("kvp", "速度KP", -2.0, 8.0,  0.10, 0.01,  "%.4f"),
            ("kvi", "速度KI", -0.2, 1.0,  0.01, 0.001, "%.4f"),
            ("ktp", "转向KP", 0.0, 15.0,  0.10, 0.01,  "%.2f"),
            ("kti", "转向KI", 0.0, 10.0,  0.10, 0.01,  "%.2f"),
        )
        self.slider_vars = {}
        self._last_live_send = {}
        # 死区补偿已固化进固件（216 counts = 3%）；此变量仅用于参数存档快照。
        self.deadband_var = tk.StringVar(value="216")
        self.ch_vars = {ch: tk.BooleanVar(value=(ch in ("angle", "target", "pwm")))
                        for ch, _, _ in CHANNELS}
        self.window_var = tk.StringVar(value="10")
        self.export_len_var = tk.StringVar(value="60")
        self.exp_all_var = tk.BooleanVar(value=False)
        self.hex_rx = tk.BooleanVar(value=False)
        self.hex_tx = tk.BooleanVar(value=False)
        self.add_lf = tk.BooleanVar(value=True)
        self.auto_scroll = tk.BooleanVar(value=True)
        self.show_ts = tk.BooleanVar(value=True)
        self.demo = tk.BooleanVar(value=False)
        self.motor_state = tk.StringVar(value="IDLE · 电机停")
        self.bal_state = tk.StringVar(value="state --")

        self.ser = None
        self.reader = None
        self.stop_evt = threading.Event()
        self.rx_queue = queue.Queue()
        self.line_buf = bytearray()
        self.data = {ch: collections.deque(maxlen=BUF_POINTS) for ch, _, _ in CHANNELS}
        self.extra = {k: collections.deque(maxlen=BUF_POINTS) for k in EXTRA_COLS}
        # Gains currently believed to be live on the MCU (updated on send/ACK).
        # Initialised to the on-car reference values now flashed into firmware.
        self.active_params = {"kp": 3.30, "ki": 0.15, "kd": 3.00,
                              "kvp": 0.80, "kvi": 0.065,
                              "ktp": 3.00, "kti": 4.00}
        # Parameter snapshots (存档), loaded from presets.json next to script.
        self.presets = load_presets()
        self.preset_name_var = tk.StringVar(value="")
        self.preset_status_var = tk.StringVar(value="")
        self.active_state = 0
        self.active_offset = 0.0
        self.events = []                    # (t_s, label) clicked buttons / BST state changes
        self._last_bal_state = None
        self.latest = {}
        self.t0 = time.monotonic()
        self.paused = False
        self._last_demo = 0.0
        self._demo_err = 0.0
        self._demo_integral = 0.0
        self._demo_deriv = 0.0
        self._demo_ms = 0
        self._last_bst_query = 0.0
        self._save_pending = 0.0            # monotonic stamp of the last SAVE, 0 = idle

        self._build_ui()
        self._refresh_ports()
        root.protocol("WM_DELETE_WINDOW", self._on_close)
        root.after(20, self._poll_queue)
        root.after(50, self._redraw)

    # ------------------------------------------------------- live slider tuning
    def _var_for(self, key):
        """Return the StringVar backing one gain, across the three groups."""
        for group in (self.pid_vars, self.speed_vars, self.turn_vars):
            if key in group:
                return group[key]
        return None

    def _sync_slider(self, key):
        """Typing in the entry box moves the slider to match."""
        spec = next((s for s in self.slider_specs if s[0] == key), None)
        if spec is None or key not in self.slider_vars:
            return
        try:
            value = float(self._var_for(key).get())
        except (TypeError, ValueError):
            return
        value = max(spec[2], min(spec[3], value))   # lo, hi
        self.slider_vars[key].set(value)

    def _on_slider(self, key, value, fmt):
        """Dragging a slider updates its number box and pushes the gains now."""
        spec = next((s for s in self.slider_specs if s[0] == key), None)
        res = spec[4] if spec is not None else 0.01
        # ttk.Scale has no resolution option; quantise here to the same step.
        value = round(round(float(value) / res) * res, 6)
        var = self._var_for(key)
        if var is not None:
            var.set(fmt % value)
        # 70 ms throttle: a fast drag generates dozens of callbacks per second;
        # the release handler below guarantees the final value is sent.
        now = time.monotonic()
        if now - self._last_live_send.get(key, 0.0) < 0.07:
            return
        self._last_live_send[key] = now
        self._send_one_live(key)

    def _flush_live(self, key):
        """Button release: always send the final slider value."""
        self._send_one_live(key)

    def _key_step(self, key, direction, scale=1):
        """Arrow-key fine tuning of one gain.

        A 104 px slider spanning 0..15 jumps ~0.14 per pixel, which is too
        coarse for tuning. Here Up/Down move by the parameter's fine step
        (0.01, or 0.001 for speed KI); Shift+Up/Down use 10x that; Ctrl+Up/
        Down use 100x for fast travel. The entry box and the slider are both
        bound, so whichever has focus works. Every step is sent immediately.
        """
        spec = next((s for s in self.slider_specs if s[0] == key), None)
        if spec is None:
            return "break"
        _key, _label, lo, hi, res, fine, fmt = spec
        var = self._var_for(key)
        if var is None:
            return "break"
        try:
            value = float(var.get())
        except (TypeError, ValueError):
            value = 0.0
        step = fine * scale
        value = round(value + direction * step, 6)
        value = max(lo, min(hi, value))          # clamp to the slider range
        var.set(fmt % value)
        self._sync_slider(key)
        self._send_one_live(key)
        return "break"                            # stop Entry from moving caret

    def _on_key(self, event, key, direction):
        """Route a key press to _key_step, reading modifier state."""
        if event.state & 0x0004:       # Ctrl  -> 100x fine (fast travel)
            scale = 100
        elif event.state & 0x0001:     # Shift -> 10x fine (coarse)
            scale = 10
        else:                          # plain arrow -> 1x fine
            scale = 1
        return self._key_step(key, direction, scale)

    def _refresh_active_params(self):
        """Snapshot what we believe is live, so CSV rows carry correct gains."""
        try:
            self.active_params.update({
                "kp": float(self.pid_vars["kp"].get()),
                "ki": float(self.pid_vars["ki"].get()),
                "kd": float(self.pid_vars["kd"].get()),
                "kvp": float(self.speed_vars["kvp"].get()),
                "kvi": float(self.speed_vars["kvi"].get()),
                "ktp": float(self.turn_vars["ktp"].get()),
                "kti": float(self.turn_vars["kti"].get()),
            })
        except (TypeError, ValueError):
            pass

    def _send_one_live(self, key):
        """Send only the group containing `key` - the core of live tuning."""
        if self.demo.get():
            self._refresh_active_params()
            self.ack_var.set("演示模式：%s 已本地生效" % key)
            return
        try:
            kp = float(self.pid_vars["kp"].get())
            ki = float(self.pid_vars["ki"].get())
            kd = float(self.pid_vars["kd"].get())
            kvp = float(self.speed_vars["kvp"].get())
            kvi = float(self.speed_vars["kvi"].get())
            ktp = float(self.turn_vars["ktp"].get())
            kti = float(self.turn_vars["kti"].get())
        except (TypeError, ValueError):
            return
        self._refresh_active_params()
        if key in ("kp", "ki", "kd"):
            text = build_set_command(kp, ki, kd)
        elif key in ("kvp", "kvi"):
            text = build_setv_command(kvp, kvi)
        else:
            text = build_sett_command(ktp, kti)
        if self._send_raw(text.encode("ascii"), text.rstrip("\n")):
            self.ack_var.set("实时下发 %s = %s" % (key, self._var_for(key).get()))
            self._mark_event("LIVE %s=%s" % (key, self._var_for(key).get()))

    # ------------------------------------------------------------------ UI
    def _build_ui(self):
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)

        conn = ttk.LabelFrame(self.root, text="串口连接", padding=6)
        conn.grid(row=0, column=0, sticky="ew", padx=8, pady=6)
        ttk.Label(conn, text="端口").pack(side="left")
        self.port_cb = ttk.Combobox(conn, textvariable=self.port_var, width=38,
                                    state="readonly")
        self.port_cb.pack(side="left", padx=(4, 10))
        ttk.Label(conn, text="波特率").pack(side="left")
        baud_cb = ttk.Combobox(conn, textvariable=self.baud_var, width=9,
                               values=BAUDS, state="readonly")
        baud_cb.pack(side="left", padx=(4, 10))
        ttk.Button(conn, text="刷新", command=self._refresh_ports).pack(side="left", padx=2)
        self.conn_btn = ttk.Button(conn, text="打开串口", command=self._toggle_port)
        self.conn_btn.pack(side="left", padx=2)
        self.status_lbl = ttk.Label(conn, textvariable=self.status_var,
                                    foreground="#c92a2a")
        self.status_lbl.pack(side="left", padx=(14, 0))

        main = ttk.Frame(self.root, padding=(8, 2))
        main.grid(row=1, column=0, sticky="nsew")
        main.columnconfigure(1, weight=1)
        main.rowconfigure(0, weight=1)

        left_outer = ttk.Frame(main, width=386)
        left_outer.grid(row=0, column=0, sticky="ns", padx=(0, 8))
        left_outer.grid_propagate(False)
        left_canvas = tk.Canvas(left_outer, width=370, highlightthickness=0)
        left_sb = ttk.Scrollbar(left_outer, orient="vertical",
                                command=left_canvas.yview)
        left_canvas.pack(side="left", fill="both", expand=True)
        left_sb.pack(side="right", fill="y")
        left_canvas.configure(yscrollcommand=left_sb.set)
        left = ttk.Frame(left_canvas)
        left_canvas.create_window((0, 0), window=left, anchor="nw")
        left.bind("<Configure>", lambda e: left_canvas.configure(
            scrollregion=left_canvas.bbox("all")))
        left_canvas.bind("<MouseWheel>", lambda e: left_canvas.yview_scroll(
            -1 * (e.delta // 120), "units"))

        pidf = ttk.LabelFrame(left, text="PID 参数（参考值 · 拖动滑块即时下发）",
                              padding=8)
        pidf.pack(fill="x", pady=(0, 6))
        # 每个参数一行：精确输入框 + 滑块。滑块 command 里直接下发对应那一
        # 组（节流 70 ms，松手补发一次），运行中不用再点下发按钮。
        # 输入框和滑块都绑定 ↑/↓：细步进微调，比拖滑块好控制得多。
        for row, (key, label, lo, hi, res, fine, fmt) in enumerate(self.slider_specs):
            var = self._var_for(key)
            ttk.Label(pidf, text=label).grid(row=row, column=0, sticky="w", pady=1)
            entry = ttk.Entry(pidf, textvariable=var, width=9, font=MONO)
            entry.grid(row=row, column=1, sticky="e", pady=1, padx=(0, 4))
            entry.bind("<Return>", lambda e, k=key: self._sync_slider(k))
            entry.bind("<FocusOut>", lambda e, k=key: self._sync_slider(k))
            for _seq, _dir in (("<Up>", 1), ("<Down>", -1)):
                entry.bind(_seq, lambda e, k=key, d=_dir: self._on_key(e, k, d))
            dvar = tk.DoubleVar(value=float(var.get()))
            self.slider_vars[key] = dvar
            sc = ttk.Scale(pidf, from_=lo, to=hi, orient="horizontal",
                           length=104, variable=dvar,
                           command=lambda v, k=key, f=fmt: self._on_slider(k, v, f))
            sc.grid(row=row, column=2, sticky="w", pady=1)
            sc.bind("<ButtonRelease-1>", lambda e, k=key: self._flush_live(k))
            for _seq, _dir in (("<Up>", 1), ("<Down>", -1)):
                sc.bind(_seq, lambda e, k=key, d=_dir: self._on_key(e, k, d))
        nrows = len(self.slider_specs)
        ttk.Label(pidf, text="↑/↓ 微调 · Shift 10× · Ctrl 100× · 拖动滑块粗调",
                  foreground="#868e96").grid(row=nrows, column=0,
                                             columnspan=3, sticky="w", pady=(2, 0))
        ttk.Button(pidf, text="读取参数", command=self._query_pid).grid(
            row=nrows + 1, column=0, sticky="ew", pady=(8, 2), padx=(0, 3))
        ttk.Button(pidf, text="下发全部", command=self._send_pid).grid(
            row=nrows + 1, column=1, columnspan=2, sticky="ew", pady=(8, 2))
        ttk.Label(pidf, textvariable=self.ack_var,
                  foreground="#0c8599").grid(
                      row=nrows + 2, column=0, columnspan=3,
                      sticky="w", pady=(4, 0))
        ttk.Button(pidf, text="恢复参考值（推荐）",
                   command=self._three_loop_preset).grid(
                       row=nrows + 3, column=0, columnspan=3,
                       sticky="ew", pady=(8, 0))
        # 把当前 7 项增益写入小车 Flash（SAVE）：断电不丢，下次上电自动恢复。
        ttk.Button(pidf, text="保存到小车（断电不丢）",
                   command=self._save_params).grid(
                       row=nrows + 4, column=0, columnspan=3,
                       sticky="ew", pady=(4, 0))

        # ---- Parameter snapshots (存档) ----
        pf = ttk.LabelFrame(left, text="参数存档（保存好参数，随时载入/对比）",
                            padding=8)
        pf.pack(fill="x", pady=(0, 6))
        name_row = ttk.Frame(pf)
        name_row.pack(fill="x")
        ttk.Label(name_row, text="名称").pack(side="left")
        ttk.Entry(name_row, textvariable=self.preset_name_var,
                  width=16).pack(side="left", padx=4)
        ttk.Button(name_row, text="保存当前",
                   command=self._preset_save).pack(side="left", padx=2)
        list_frame = ttk.Frame(pf)
        list_frame.pack(fill="x", pady=(6, 2))
        self.preset_list = tk.Listbox(list_frame, height=5,
                                      font=MONO, exportselection=False)
        self.preset_list.pack(side="left", fill="x", expand=True)
        sb = ttk.Scrollbar(list_frame, orient="vertical",
                           command=self.preset_list.yview)
        sb.pack(side="right", fill="y")
        self.preset_list.config(yscrollcommand=sb.set)
        self.preset_list.bind("<<ListboxSelect>>", self._preset_on_select)
        btns = ttk.Frame(pf)
        btns.pack(fill="x", pady=(2, 0))
        ttk.Button(btns, text="载入并下发",
                   command=self._preset_load).pack(side="left", padx=(0, 4))
        ttk.Button(btns, text="仅填入不发送",
                   command=self._preset_fill_only).pack(side="left", padx=4)
        ttk.Button(btns, text="删除",
                   command=self._preset_delete).pack(side="left", padx=4)
        ttk.Button(btns, text="导出对比CSV",
                   command=self._preset_export_csv).pack(side="left", padx=4)
        ttk.Label(pf, textvariable=self.preset_status_var,
                  foreground="#0c8599", wraplength=300).pack(anchor="w", pady=(4, 0))
        self._preset_refresh_list()

        chf = ttk.LabelFrame(left, text="曲线通道", padding=8)
        chf.pack(fill="x", pady=6)
        for idx, (ch, label, color) in enumerate(CHANNELS):
            cb = ttk.Checkbutton(chf, text=label, variable=self.ch_vars[ch],
                                 command=self._redraw_now)
            cb.grid(row=idx // 2, column=idx % 2, sticky="w", pady=3)

        setf = ttk.LabelFrame(left, text="曲线设置", padding=8)
        setf.pack(fill="x", pady=6)
        row0 = ttk.Frame(setf)
        row0.pack(fill="x")
        ttk.Label(row0, text="时间窗口").pack(side="left")
        ttk.Combobox(row0, textvariable=self.window_var, width=5,
                     values=WINDOWS, state="readonly").pack(side="left", padx=4)
        ttk.Label(row0, text="秒").pack(side="left")
        row0b = ttk.Frame(setf)
        row0b.pack(fill="x", pady=(6, 0))
        ttk.Label(row0b, text="导出时长").pack(side="left")
        ttk.Combobox(row0b, textvariable=self.export_len_var, width=5,
                     values=EXPORT_LENS).pack(side="left", padx=4)
        ttk.Label(row0b, text="秒（可手输）").pack(side="left")
        ttk.Checkbutton(setf, text="导出全部缓存（覆盖上面的时长）",
                        variable=self.exp_all_var).pack(anchor="w", pady=(6, 0))
        row1 = ttk.Frame(setf)
        row1.pack(fill="x", pady=(8, 0))
        self.pause_btn = ttk.Button(row1, text="暂停", command=self._toggle_pause)
        self.pause_btn.pack(side="left", expand=True, fill="x", padx=2)
        ttk.Button(row1, text="清空", command=self._clear_plot).pack(
            side="left", expand=True, fill="x", padx=2)
        ttk.Button(row1, text="导出 CSV", command=self._export_csv).pack(
            side="left", expand=True, fill="x", padx=2)

        right = ttk.Frame(main)
        right.grid(row=0, column=1, sticky="nsew")
        right.columnconfigure(0, weight=1)
        right.rowconfigure(0, weight=1)
        self.canvas = tk.Canvas(right, bg="#ffffff", highlightthickness=1,
                                highlightbackground="#d0d5dd")
        self.canvas.grid(row=0, column=0, sticky="nsew")
        ttk.Label(right, textvariable=self.value_var, font=MONO,
                  foreground="#343a40").grid(row=1, column=0, sticky="w", pady=(4, 0))

        rxf = ttk.LabelFrame(self.root, text="接收区", padding=4)
        rxf.grid(row=2, column=0, sticky="ew", padx=8, pady=(2, 2))
        rxf.columnconfigure(0, weight=1)
        bar = ttk.Frame(rxf)
        bar.grid(row=0, column=0, sticky="ew")
        ttk.Button(bar, text="清空接收", command=self._clear_rx).pack(side="left")
        ttk.Button(bar, text="导出 CSV", command=self._export_csv).pack(side="left", padx=4)
        ttk.Checkbutton(bar, text="自动滚动", variable=self.auto_scroll).pack(side="left", padx=6)
        ttk.Checkbutton(bar, text="时间戳", variable=self.show_ts).pack(side="left", padx=6)
        ttk.Checkbutton(bar, text="HEX 显示", variable=self.hex_rx).pack(side="left", padx=6)
        self.rx_text = scrolledtext.ScrolledText(rxf, height=8, wrap=tk.WORD,
                                                 font=MONO, bg="#fbfcfd")
        self.rx_text.grid(row=1, column=0, sticky="ew", pady=(4, 0))

        mb = ttk.LabelFrame(self.root, text="平衡控制（先扶正标定，再使能）", padding=(6, 4))
        mb.grid(row=3, column=0, sticky="ew", padx=8, pady=(4, 0))
        ttk.Button(mb, text="① 扶正标定 CAL", command=self._bal_cal).pack(side="left", padx=(2, 2))
        ttk.Button(mb, text="② 开始平衡", command=self._bal_on).pack(side="left", padx=2)
        ttk.Button(mb, text="急停 BAL,0", command=self._bal_off).pack(side="left", padx=2)
        ttk.Button(mb, text="状态", command=self._bal_query).pack(side="left", padx=2)
        ttk.Label(mb, textvariable=self.motor_state,
                  foreground="#0c8599").pack(side="left", padx=(8, 0))
        ttk.Label(mb, textvariable=self.bal_state, font=MONO,
                  foreground="#495057").pack(side="left", padx=(6, 0))

        txf = ttk.Frame(self.root, padding=(8, 4, 8, 8))
        txf.grid(row=4, column=0, sticky="ew")
        txf.columnconfigure(1, weight=1)
        ttk.Label(txf, text="发送").grid(row=0, column=0, sticky="w")
        self.tx_entry = ttk.Entry(txf, font=MONO)
        self.tx_entry.grid(row=0, column=1, sticky="ew", padx=4)
        self.tx_entry.bind("<Return>", lambda _e: self._send_text())
        ttk.Button(txf, text="发送", command=self._send_text).grid(row=0, column=2, padx=2)
        ttk.Checkbutton(txf, text="HEX 发送", variable=self.hex_tx).grid(row=0, column=3, padx=4)
        ttk.Checkbutton(txf, text="追加换行", variable=self.add_lf).grid(row=0, column=4)

    # --------------------------------------------------------------- serial
    @staticmethod
    def _port_rank(p):
        """Rank Windows COM ports so the usable HC-05 port is selected first.

        Pairing an SPP device creates TWO 'Standard Serial over Bluetooth'
        ports. The 'incoming' listener has an all-zero remote Bluetooth
        address in its hardware id and cannot actually reach the module;
        the outgoing port carries the real remote MAC (e.g. 002508E00486).
        """
        hwid = (p.hwid or "").upper()
        desc = p.description or ""
        bluetooth = ("BTHENUM" in hwid or "蓝牙" in desc
                     or "Bluetooth" in desc)
        if bluetooth:
            return 3 if "000000000000" not in hwid else 1
        return 2

    def _refresh_ports(self):
        ports = list(serial.tools.list_ports.comports())
        ranked = []
        for p in ports:
            rank = self._port_rank(p)
            tag = {3: " (推荐)", 1: " (传入口, 通常不可用)"}.get(rank, "")
            desc = (p.description or "").replace("(%s)" % p.device, "").strip()
            ranked.append((rank, p.device,
                           "%s - %s%s" % (p.device, desc, tag)))
        ranked.sort(key=lambda x: (-x[0], x[1]))
        items = [entry[2] for entry in ranked]
        self.port_cb["values"] = items
        if items:
            self.port_cb.current(0)
        else:
            self.port_var.set("")

    def _toggle_port(self):
        if self.ser is not None and self.ser.is_open:
            self._close_port()
        else:
            self._open_port()

    def _open_port(self):
        sel = self.port_var.get().strip()
        if not sel:
            messagebox.showwarning("提示", "没有可用的串口，请点击「刷新」。")
            return
        port = sel.split(" - ")[0].strip()
        try:
            self.ser = serial.Serial(port=port, baudrate=int(self.baud_var.get()),
                                     bytesize=8, parity="N", stopbits=1,
                                     timeout=0.1, write_timeout=1)
        except Exception as exc:
            self.ser = None
            messagebox.showerror("串口打开失败", str(exc))
            return
        self.stop_evt.clear()
        self.reader = threading.Thread(target=self._reader_worker, daemon=True)
        self.reader.start()
        self.conn_btn.configure(text="关闭串口")
        self.status_var.set("已连接 %s @ %s" % (port, self.baud_var.get()))
        self.status_lbl.configure(foreground="#2b8a3e")
        self._log("[已连接] %s @ %s baud\n" % (port, self.baud_var.get()))

    def _close_port(self):
        self.stop_evt.set()
        if self.reader is not None:
            self.reader.join(timeout=0.6)
            self.reader = None
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        self.conn_btn.configure(text="打开串口")
        self.status_var.set("未连接")
        self.status_lbl.configure(foreground="#c92a2a")

    def _reader_worker(self):
        while not self.stop_evt.is_set():
            try:
                n = self.ser.in_waiting
                data = self.ser.read(n if n > 0 else 1)
            except Exception as exc:
                self.rx_queue.put(("error", str(exc)))
                return
            if data:
                self.rx_queue.put(("data", data))

    def _poll_queue(self):
        while True:
            try:
                kind, payload = self.rx_queue.get_nowait()
            except queue.Empty:
                break
            if kind == "data":
                self._on_bytes(payload)
            elif kind == "error":
                self._log("[串口错误] %s\n" % payload)
                self._close_port()
        if (self._save_pending != 0.0
                and (time.monotonic() - self._save_pending) > 3.0):
            self._save_pending = 0.0
            self.ack_var.set("⚠️ 3 秒没收到回执，检查串口/固件")
            self._log("[保存] 3 秒内无回执：确认串口已连接、波特率 115200、"
                      "且固件里有 SAVE 命令\n")
        if self.demo.get():
            self._demo_tick()
        if not self.demo.get() and self.ser is not None and self.ser.is_open:
            now_t = time.monotonic()
            if (now_t - self._last_bst_query) >= 0.5:
                self._last_bst_query = now_t
                try:
                    self.ser.write(b"BST\n")   # track RUN / FAULT transitions
                except Exception:
                    pass
        self.root.after(20, self._poll_queue)

    def _mark_event(self, label):
        """Stamp an event on the same t0 clock used by telemetry samples."""
        t = time.monotonic() - self.t0
        self.events.append((t, label))
        self._log("[事件] %s\n" % label)

    # ----------------------------------------------------------------- data
    def _log(self, text, timestamp=False):
        prefix = time.strftime("[%H:%M:%S] ") if (timestamp and self.show_ts.get()) else ""
        try:
            self.rx_text.insert("end", prefix + text)
            lines = int(self.rx_text.index("end-1c").split(".")[0])
            if lines > 600:
                self.rx_text.delete("1.0", "200.0")
            if self.auto_scroll.get():
                self.rx_text.see("end")
        except tk.TclError:
            pass

    def _clear_rx(self):
        self.rx_text.delete("1.0", "end")

    def _on_bytes(self, data):
        if self.hex_rx.get():
            self.line_buf.clear()
            self._log(" ".join("%02X" % b for b in data) + "\n")
            return
        self.line_buf.extend(data)
        while True:
            idx = self.line_buf.find(b"\n")
            if idx < 0:
                break
            raw = bytes(self.line_buf[:idx])
            del self.line_buf[:idx + 1]
            line = raw.decode("utf-8", errors="replace").strip()
            if line:
                # BST is auto-polled at 2 Hz; show it only in the status label
                # so the receive area is not flooded.
                if not line.startswith("BST,"):
                    self._log(line + "\n", timestamp=True)
                self._on_line(line)

    def _on_line(self, line):
        if line.startswith("LOG,"):
            self._handle_log(line[4:])
            return
        t = parse_telemetry(line)
        if t is not None:
            now = time.monotonic() - self.t0
            for ch, _, _ in CHANNELS:
                self.data[ch].append((now, t[ch]))
            # Diagnostic columns travel with every frame so the CSV explains
            # itself: encoder rate, yaw rate, state, offset and live gains.
            self.extra["enc_tps"].append((now, t.get("enc_tps", 0.0)))
            self.extra["gyroz"].append((now, t.get("gyroz", 0.0)))
            self.extra["state"].append((now, self.active_state))
            self.extra["offset"].append((now, self.active_offset))
            for k in ("kp", "ki", "kd", "kvp", "kvi", "ktp", "kti"):
                self.extra[k].append((now, self.active_params[k]))
            self.latest = t
            self.value_var.set(
                "Angle %+.2f  Target %+.2f  P %+.2f  I %+.2f  D %+.2f"
                "  PWM %+.0f | Vel %+.2f Turn %+.0f"
                % (t["angle"], t["target"], t["p"], t["i"], t["d"],
                   t["pwm"], t["vel"], t["turn"]))
            return
        ack = parse_ack(line)
        if ack is not None:
            for key, value in zip(("kp", "ki", "kd"), ack):
                self.pid_vars[key].set("%.2f" % value)
                self._sync_slider(key)
            self._refresh_active_params()
            self.ack_var.set("角度环已同步 (ACK)")
            return
        ack_v = parse_ack_v(line)
        if ack_v is not None:
            for key, value in zip(("kvp", "kvi"), ack_v):
                self.speed_vars[key].set("%.4f" % value)
                self._sync_slider(key)
            self._refresh_active_params()
            self.ack_var.set("速度环已同步 (ACKV)")
            return
        ack_t = parse_ack_t(line)
        if ack_t is not None:
            for key, value in zip(("ktp", "kti"), ack_t):
                self.turn_vars[key].set("%.2f" % value)
                self._sync_slider(key)
            self._refresh_active_params()
            self.ack_var.set("转向环已同步 (ACKT)")
            return
        b = parse_bst(line)
        if b is not None:
            name = BST_STATES.get(b["state"], "?")
            self.active_state = b["state"]
            self.active_offset = b["offset"]
            if b["state"] != self._last_bal_state:
                self._mark_event("BST:" + name)
                self._last_bal_state = b["state"]
            self.bal_state.set("angle %+.2f  offs %+.2f  duty %+d"
                               % (b["angle"], b["offset"], b["duty"]))
            if b["state"] == 2:
                self.motor_state.set("RUN · 闭环中")
            elif b["state"] == 1:
                self.motor_state.set("CAL %d%% · 保持扶正" % b["cal"])
            elif b["state"] == 3:
                self.motor_state.set("FAULT · 已急停")
            else:
                self.motor_state.set("IDLE · 电机停")
            return

    def _handle_log(self, text):
        """把固件 LOG 回执翻译成明确结论，并清掉「保存中」等待状态。

        旧版本只把标签设成「保存中…」却从不解析回执，所以保存成功也像卡死。
        现在四种结果各有各的提示：成功 / 运行中被拒 / 写 Flash 失败 / 固件太旧。
        """
        low = text.lower()
        pending = self._save_pending != 0.0

        if "params saved to flash" in low:
            self._save_pending = 0.0
            self.ack_var.set("✅ 已存入小车 Flash，断电不丢")
            self._mark_event("SAVE 成功")
            self._log("[保存] 成功：参数已写入 Flash，下次上电自动恢复\n")
            return
        if "save flash write failed" in low:
            self._save_pending = 0.0
            self.ack_var.set("❌ Flash 写入失败（页被写保护？）")
            self._log("[保存] 固件回报写入失败：检查是否开了读保护/写保护\n")
            return
        if "save rejected" in low:
            self._save_pending = 0.0
            self.ack_var.set("❌ 小车还在运行/标定，先急停再保存")
            self._log("[保存] 被拒绝：点「急停」等状态变成 IDLE 后再点保存\n")
            return
        if "unknown cmd" in low and pending:
            self._save_pending = 0.0
            self.ack_var.set("⚠️ 固件不认识 SAVE，请烧录最新 hex")
            self._log("[保存] 当前固件没有 SAVE 命令：重新编译烧录后可用\n")
            return
        if "params restored from flash" in low:
            self.ack_var.set("小车已从 Flash 恢复参数")
            return

    # ------------------------------------------------------------------ send
    def _send_raw(self, data, echo):
        if self.ser is None or not self.ser.is_open:
            messagebox.showwarning("未连接", "请先打开串口。")
            return False
        try:
            self.ser.write(data)
        except Exception as exc:
            messagebox.showerror("发送失败", str(exc))
            return False
        self._log(">> " + echo + "\n")
        return True

    def _send_text(self):
        text = self.tx_entry.get().strip()
        if not text:
            return
        if self.hex_tx.get():
            try:
                data = bytes.fromhex(text.replace(" ", ""))
            except ValueError:
                messagebox.showerror("HEX 格式错误", "请输入类似 AA BB 0D 的十六进制字节。")
                return
            echo = " ".join("%02X" % b for b in data)
        else:
            data = text.encode("utf-8")
            echo = text
            if self.add_lf.get():
                data += b"\n"
                echo += "\\n"
        self._send_raw(data, echo)

    def _send_pid(self):
        """一键下发全部七项（实车调出的参考参数）。

        滑块拖动时只发对应那一组；这里用于上电后一次性对齐全部参数。
        目标速度/转向差速固定发 0，避免上次调参残留让车自己跑。
        """
        try:
            kp = float(self.pid_vars["kp"].get())
            ki = float(self.pid_vars["ki"].get())
            kd = float(self.pid_vars["kd"].get())
            kvp = float(self.speed_vars["kvp"].get())
            kvi = float(self.speed_vars["kvi"].get())
            ktp = float(self.turn_vars["ktp"].get())
            kti = float(self.turn_vars["kti"].get())
        except ValueError:
            messagebox.showerror("参数错误", "7 个参数都必须是数字。")
            return
        self.active_params.update({"kp": kp, "ki": ki, "kd": kd,
                                   "kvp": kvp, "kvi": kvi,
                                   "ktp": ktp, "kti": kti})
        if self.demo.get():
            self.ack_var.set("演示模式：参数已本地生效")
            return
        frames = (build_set_command(kp, ki, kd),
                  build_setv_command(kvp, kvi),
                  build_sett_command(ktp, kti))
        sent = True
        for text in frames:
            sent = self._send_raw(text.encode("ascii"),
                                  text.rstrip("\n")) and sent
        if sent:
            self.ack_var.set("已下发 7 项，等待 ACK / ACKV / ACKT ...")
            self._mark_event("PID 下发 A%.2f/%.2f/%.2f V%.2f/%.3f T%.2f/%.2f"
                             % (kp, ki, kd, kvp, kvi, ktp, kti))

    def _query_pid(self):
        for cmd in (b"GET\n", b"GETV\n", b"GETT\n"):
            self._send_raw(cmd, cmd.decode("ascii").strip())
        self.ack_var.set("查询中，等待 ACK / ACKV / ACKT ...")

    def _save_params(self):
        """SAVE writes the current gains into the car's flash (survives off)."""
        if self._motor_cmd("SAVE\n", "SAVE"):
            self._save_pending = time.monotonic()
            self.ack_var.set("保存中… 等小车回执（约 0.1 s）")
            self._mark_event("点击 保存到小车 SAVE")
        else:
            self._save_pending = 0.0

    def _three_loop_preset(self):
        """恢复实车调出的参考参数（已固化进固件），并同步滑块后立即下发。"""
        presets = {"kp": "3.30", "ki": "0.15", "kd": "3.00",
                   "kvp": "0.8000", "kvi": "0.0650",
                   "ktp": "3.00", "kti": "4.00"}
        for key, text in presets.items():
            self._var_for(key).set(text)
            self._sync_slider(key)
        self.deadband_var.set("216")
        self._send_pid()

    # ------------------------------------------------------ parameter snapshots
    def _current_snapshot_values(self):
        """Collect the 7 gains + deadband as strings from the UI vars."""
        vals = {k: self._var_for(k).get() for k in PRESET_KEYS if k != "deadband"}
        vals["deadband"] = self.deadband_var.get()
        return vals

    def _preset_refresh_list(self):
        """Rebuild the listbox from self.presets (newest first in the file)."""
        if not hasattr(self, "preset_list"):
            return
        self.preset_list.delete(0, tk.END)
        for p in self.presets:
            name = p.get("name", "(未命名)")
            ts = p.get("time", "")
            label = "%s   %s" % (name, ts) if ts else name
            self.preset_list.insert(tk.END, label)

    def _preset_save(self):
        name = self.preset_name_var.get().strip()
        if not name:
            name = time.strftime("存档%m-%d %H:%M")
        vals = self._current_snapshot_values()
        snap = {"name": name, "time": time.strftime("%Y-%m-%d %H:%M:%S")}
        snap.update(vals)
        self.presets.insert(0, snap)          # newest first
        ok, err = save_presets(self.presets)
        if not ok:
            self.presets.pop(0)
            self.preset_status_var.set("保存失败: %s" % err)
            messagebox.showerror("存档失败", err)
            return
        self._preset_refresh_list()
        self.preset_list.selection_clear(0, tk.END)
        self.preset_list.selection_set(0)
        self.preset_status_var.set("已保存「%s」" % name)
        self._log("[存档] 保存 %s" % name, timestamp=True)

    def _selected_preset(self):
        sel = self.preset_list.curselection()
        if not sel or sel[0] >= len(self.presets):
            return None
        return self.presets[sel[0]]

    def _preset_on_select(self, _event=None):
        p = self._selected_preset()
        if not p:
            return
        parts = ["%s=%s" % (k, p.get(k, "?")) for k in PRESET_KEYS]
        self.preset_status_var.set("  ".join(parts))

    def _apply_snapshot(self, p):
        """Push a snapshot's values into the UI vars + sliders (no sending)."""
        for k in PRESET_KEYS:
            if k not in p:
                continue
            text = str(p[k])
            if k == "deadband":
                self.deadband_var.set(text)
            else:
                self._var_for(k).set(text)
                self._sync_slider(k)

    def _preset_fill_only(self):
        p = self._selected_preset()
        if not p:
            self.preset_status_var.set("先选中一个存档")
            return
        self._apply_snapshot(p)
        self.preset_status_var.set("已填入「%s」，未下发（点下发全部才生效）"
                                   % p.get("name", "?"))

    def _preset_load(self):
        p = self._selected_preset()
        if not p:
            self.preset_status_var.set("先选中一个存档")
            return
        self._apply_snapshot(p)
        self._send_pid()
        self.preset_status_var.set("已载入并下发「%s」" % p.get("name", "?"))
        self._log("[存档] 载入 %s" % p.get("name", "?"), timestamp=True)

    def _preset_delete(self):
        p = self._selected_preset()
        if not p:
            self.preset_status_var.set("先选中一个存档")
            return
        name = p.get("name", "?")
        if not messagebox.askyesno("删除存档", "确定删除「%s」？" % name):
            return
        self.presets.remove(p)
        ok, err = save_presets(self.presets)
        if not ok:
            messagebox.showerror("删除失败", err)
            return
        self._preset_refresh_list()
        self.preset_status_var.set("已删除「%s」" % name)

    def _preset_export_csv(self):
        """Export all snapshots as a CSV table for side-by-side comparison."""
        if not self.presets:
            self.preset_status_var.set("没有存档可导出")
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            initialfile="参数存档对比.csv",
            filetypes=[("CSV", "*.csv"), ("All", "*.*")])
        if not path:
            return
        try:
            with open(path, "w", newline="", encoding="utf-8-sig") as fh:
                w = csv.writer(fh)
                w.writerow(["name", "time"] + list(PRESET_KEYS))
                for p in self.presets:
                    w.writerow([p.get("name", ""), p.get("time", "")] +
                               [p.get(k, "") for k in PRESET_KEYS])
            self.preset_status_var.set("已导出 %d 条到 %s"
                                       % (len(self.presets), os.path.basename(path)))
        except OSError as exc:
            messagebox.showerror("导出失败", str(exc))

    # ---------------------------------------------------------------- motors
    def _motor_cmd(self, cmd, echo):
        if self.demo.get():
            self._log("[演示] %s 只对真实设备有效\n" % echo)
            return False
        return self._send_raw(cmd.encode("ascii"), echo)

    def _bal_cal(self):
        if self.demo.get():
            self._log("[演示] CAL 只对真实设备有效\n")
            return
        if messagebox.askyesno(
                "扶正标定",
                "请先把小车扶到竖直、保持静止。\n"
                "标定期间电机不会动，约 0.5 秒完成。\n\n继续吗？"):
            if self._motor_cmd("CAL\n", "CAL"):
                self.motor_state.set("CAL 中 · 保持扶正静止")
                self._mark_event("点击 扶正标定 CAL")

    def _bal_on(self):
        if self.demo.get():
            self._log("[演示] BAL,1 只对真实设备有效\n")
            return
        if not messagebox.askyesno(
                "开始平衡",
                "确认已标定零位、车基本竖直、周围没有障碍物？\n"
                "随时点「急停 BAL,0」或发 BAL,0 停电机。\n\n继续吗？"):
            return
        if self._motor_cmd("BAL,1\n", "BAL,1"):
            self.motor_state.set("已下发 BAL,1 · 等待确认")
            self._mark_event("点击 开始平衡 BAL,1")

    def _bal_off(self):
        if self._motor_cmd("BAL,0\n", "BAL,0"):
            self.motor_state.set("IDLE · 电机停")
            self._mark_event("点击 急停 BAL,0")

    def _bal_query(self):
        self._motor_cmd("BST\n", "BST")

    def _show_help(self):
        messagebox.showinfo(
            "通信协议 (ASCII / LF / 三环增益 x100,x10000,x100)",
            "固件基于江科大教程 06 三环 PID；DLPF 关闭，10 ms 角度环 +\n"
            "50 ms 速度/转向环；CAL 后角度自动清零。\n"
            "\n"
            "PC -> 小车: SET/SETV/SETT 下发三组增益；GET/GETV/GETT 读取；\n"
            "            SAVE 存入小车 Flash(断电不丢)；CAL 扶正标定；\n"
            "            BAL,1/BAL,0 使能/急停；BST 查状态。\n"
            "小车 -> PC: ACK/ACKV/ACKT/LOG；BST 状态；T 遥测(25 帧/秒)。\n"
            "\n"
            "固定参数(不在链路上改): 目标速度/差速=0，角度微调=0，\n"
            "  死区=216 counts(3%)，输出方向、50°保护、电机极性、\n"
            "  编码器方向、轮动力补偿均为固件常量。\n"
            "\n"
            "参考值: 角度 3.30/0.15/3.00，速度 0.80/0.065，转向 3.00/4.00。\n"
            "保存判读: 点「保存到小车」后看接收区最后一行——\n"
            "  LOG,Params saved to flash        -> 成功，断电不丢\n"
            "  LOG,ERR SAVE rejected            -> 还在跑，先点急停\n"
            "  LOG,ERR unknown cmd              -> 烧的是旧 hex，重新烧录\n"
            "  也可断电重上电，接收区出现 restored from flash 即已生效。\n"
            "运行中调参: 输入框/滑块 ↑↓ 微调(Shift 10x, Ctrl 100x)或拖滑块。\n"
            "上车: 扶正静止上电 -> CAL -> 开始平衡；异常点急停 BAL,0。\n"
            "HC-05 需先配置 AT+UART=115200,0,0")

    def _on_demo_toggle(self):
        self._last_demo = 0.0
        self._demo_err = 0.0
        self._demo_integral = 0.0
        self._demo_deriv = 0.0
        if self.demo.get():
            self._log("[演示] 模拟遥测已开启，未接硬件也能查看曲线\n")
            self.status_var.set("演示模式（无硬件）")
            self.status_lbl.configure(foreground="#1971c2")
        else:
            self._log("[演示] 模拟遥测已关闭\n")
            connected = (self.ser is not None and self.ser.is_open)
            self.status_var.set("已连接" if connected else "未连接")
            self.status_lbl.configure(foreground="#2b8a3e" if connected else "#c92a2a")

    def _demo_tick(self):
        """Generate a synthetic T,... frame at 25 Hz for UI verification."""
        now = time.monotonic()
        if (now - self._last_demo) < 0.04:
            return
        self._last_demo = now
        t = now - self.t0
        try:
            kp = float(self.pid_vars["kp"].get())
            kd = float(self.pid_vars["kd"].get())
        except ValueError:
            kp, kd = 0.0, 0.0

        angle = (9.0 * math.sin(2.0 * math.pi * 0.32 * t)
                 + 0.8 * math.sin(2.0 * math.pi * 1.30 * t))
        err = -angle
        self._demo_integral = max(-50.0, min(50.0,
                                             self._demo_integral + err * 0.04))
        deriv = (err - self._demo_err) / 0.04
        self._demo_err = err
        self._demo_deriv = (0.8 * self._demo_deriv
                            + 0.2 * max(-500.0, min(500.0, deriv)))
        p = kp * err
        i = 0.0  # 角度环无 I 项，演示数据也保持 0
        d = kd * self._demo_deriv
        pwm = max(-7200.0, min(7200.0, p + i + d))
        self._demo_ms += 40
        line = "T,%d,%d,0,%d,%d,%d,%d,0,0,0,0" % (
            self._demo_ms, int(round(angle * 100)),
            int(round(p * 100)), int(round(i * 100)),
            int(round(d * 100)), int(round(pwm)))
        self._on_line(line)

    # ----------------------------------------------------------------- plot
    def _toggle_pause(self):
        self.paused = not self.paused
        self.pause_btn.configure(text="继续" if self.paused else "暂停")

    def _redraw_now(self):
        if not self.paused:
            self._draw_plot()

    def _clear_plot(self):
        for ch, _, _ in CHANNELS:
            self.data[ch].clear()
        for k in EXTRA_COLS:
            self.extra[k].clear()
        self.latest = {}
        self.events.clear()
        self._last_bal_state = None
        self.t0 = time.monotonic()
        self.value_var.set("Angle --   Target --   P --   I --   D --   PWM --")
        self._draw_plot()

    def _export_csv(self):
        path = filedialog.asksaveasfilename(defaultextension=".csv",
                                            filetypes=[("CSV 文件", "*.csv")],
                                            title="导出曲线数据")
        if not path:
            return
        try:
            # Wide format: one row per timestamp, one column per channel.
            # (The old long format stacked 6000 angle rows before the next
            #  channel, which looked like "CSV only has angle".)
            rows = collections.defaultdict(dict)
            for ch, _, _ in CHANNELS:
                for ts, val in self.data[ch]:
                    rows["%.3f" % ts][ch] = val
            for k in EXTRA_COLS:
                for ts, val in self.extra[k]:
                    rows["%.3f" % ts][k] = val
            stamps = sorted(rows, key=float)
            # Scope: last N seconds, where N comes from the "导出时长" box.
            # Tick "导出全部缓存" to dump the whole ring buffer instead.
            secs = self._export_secs()
            if not self.exp_all_var.get() and stamps:
                t_end = float(stamps[-1])
                t_beg = t_end - secs
                stamps = [s for s in stamps if float(s) >= t_beg - 1e-9]
            if not stamps:
                messagebox.showinfo("没有数据",
                                    "缓存里没有曲线数据，无法导出。\n"
                                    "先连上串口跑一段遥测再导。")
                return
            ev_by_ts = {}
            for et, label in self.events:
                if not stamps:
                    break
                nearest = min(stamps, key=lambda s: abs(float(s) - et))
                if abs(float(nearest) - et) <= 0.10:
                    ev_by_ts[nearest] = ";".join(
                        x for x in (ev_by_ts.get(nearest, ""), label) if x)
            with open(path, "w", encoding="utf-8-sig", newline="") as fh:
                writer = csv.writer(fh)
                writer.writerow(["t_s"] + [ch for ch, _, _ in CHANNELS]
                                + EXTRA_COLS + ["event"])
                for ts in stamps:
                    r = rows[ts]
                    cells = ["%.4f" % r.get(ch, 0.0)
                             for ch, _, _ in CHANNELS]
                    for k in EXTRA_COLS:
                        if k == "state":
                            cells.append("%d" % int(r.get(k, 0)))
                        else:
                            cells.append("%.4f" % r.get(k, 0.0))
                    writer.writerow([ts] + cells + [ev_by_ts.get(ts, "")])
        except OSError as exc:
            messagebox.showerror("导出失败", str(exc))
            return
        dur = float(stamps[-1]) - float(stamps[0])
        try:
            request = float(self.export_len_var.get())
        except (TypeError, ValueError):
            request = 0.0
        if self.exp_all_var.get():
            scope = "全部缓存"
        else:
            secs = self._export_secs()
            scope = "%.0fs" % secs
            if request > BUF_SECS + 0.5:
                scope += " (缓存上限 %.0fs)" % BUF_SECS
        self._log("[导出] %s (%s, %d 行, %.1f 秒)\n"
                  % (path, scope, len(stamps), dur))

    def _redraw(self):
        if not self.paused:
            self._draw_plot()
        self.root.after(50, self._redraw)

    def _window_secs(self):
        """Plot window length in seconds."""
        try:
            window = float(self.window_var.get())
        except ValueError:
            window = 10.0
        return window if window > 0 else 10.0

    def _export_secs(self):
        """Requested export length in seconds; falls back to the plot window."""
        try:
            secs = float(self.export_len_var.get())
        except ValueError:
            secs = self._window_secs()
        if secs <= 0:
            secs = self._window_secs()
        return min(secs, BUF_SECS)

    def _draw_plot(self):
        c = self.canvas
        w, h = c.winfo_width(), c.winfo_height()
        c.delete("all")
        if w < 100 or h < 80:
            return
        L, R, T, B = 58, 14, 14, 26
        pw, ph = w - L - R, h - T - B
        if pw <= 0 or ph <= 0:
            return

        window = self._window_secs()

        t_now = time.monotonic() - self.t0
        x1 = t_now
        x0 = x1 - window
        if x0 < 0.0:
            x0 = 0.0
            x1 = max(window, t_now)
        span = x1 - x0

        visible = [(ch, label, color) for ch, label, color in CHANNELS
                   if self.ch_vars[ch].get()]
        series = {}
        ymin = ymax = None
        for ch, _, _ in visible:
            pts = [(t, v) for (t, v) in self.data[ch] if x0 <= t <= x1]
            series[ch] = pts
            if pts:
                vals = [v for _, v in pts]
                lo, hi = min(vals), max(vals)
                ymin = lo if ymin is None else min(ymin, lo)
                ymax = hi if ymax is None else max(ymax, hi)
        if ymin is None:
            ymin, ymax = -1.0, 1.0
        if (ymax - ymin) < 1e-6:
            mid = (ymin + ymax) / 2.0
            ymin, ymax = mid - 1.0, mid + 1.0
        else:
            pad = (ymax - ymin) * 0.10
            ymin -= pad
            ymax += pad
        yspan = ymax - ymin

        c.create_rectangle(L, T, w - R, h - B, outline="#d0d5dd")
        for i in range(5):
            y = T + ph * i / 4.0
            c.create_line(L, y, w - R, y, fill="#e9ecef", dash=(2, 3))
            val = ymax - yspan * i / 4.0
            c.create_text(L - 6, y, text="%.1f" % val, anchor="e",
                          fill="#6b7280", font=("Consolas", 8))
        for i in range(6):
            x = L + pw * i / 5.0
            c.create_line(x, T, x, h - B, fill="#f1f3f5", dash=(2, 3))
            tval = x0 + span * i / 5.0
            c.create_text(x, h - B + 9, text="%.0fs" % tval, anchor="n",
                          fill="#6b7280", font=("Consolas", 8))

        # Event markers: clicking a button or a BST state change draws a
        # dashed vertical line, so "when did balance start" is visible.
        for et, label in self.events:
            if x0 <= et <= x1:
                x = L + (et - x0) / span * pw
                c.create_line(x, T, x, h - B, fill="#868e96", dash=(3, 3))
                c.create_text(x + 2, T + 4, text=label, anchor="nw",
                              fill="#495057", font=("Consolas", 7))

        for ch, _, color in visible:
            pts = series[ch]
            if len(pts) < 2:
                continue
            step = max(1, len(pts) // max(1, int(pw)))
            flat = []
            for idx in range(0, len(pts), step):
                t, v = pts[idx]
                flat.extend((L + (t - x0) / span * pw,
                             T + (ymax - v) / yspan * ph))
            if (len(pts) - 1) % step != 0:
                t, v = pts[-1]
                flat.extend((L + (t - x0) / span * pw,
                             T + (ymax - v) / yspan * ph))
            if len(flat) >= 4:
                if ch == "pwm":
                    # dashed near-black: unmistakable next to Target green
                    c.create_line(*flat, fill=color, width=2, dash=(6, 3))
                else:
                    c.create_line(*flat, fill=color, width=2)

        lx, ly = L + 10, T + 8
        for _, label, color in visible:
            c.create_rectangle(lx, ly, lx + 10, ly + 10, fill=color, outline="")
            c.create_text(lx + 15, ly + 5, text=label, anchor="w",
                          fill="#374151", font=("Microsoft YaHei", 8))
            ly += 16
        if self.paused:
            c.create_text(w - R - 8, T + 10, text="PAUSED", anchor="ne",
                          fill="#c92a2a", font=("Consolas", 9, "bold"))

    # ----------------------------------------------------------------- exit
    def _on_close(self):
        self._close_port()
        self.root.destroy()


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        _selftest()
    else:
        root = tk.Tk()
        PIDTuner(root)
        root.mainloop()

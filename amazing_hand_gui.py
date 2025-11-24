#!/usr/bin/env python3
"""
GUI with joystick controls for AmazingHand - control each finger individually.
"""
import sys
import os
from pathlib import Path
import tkinter as tk
from tkinter import ttk
from textwrap import dedent
import numpy as np
from rustypot import Scs0009PyController
import threading
import time
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt


BASE_DIR = Path(__file__).resolve().parent
DATA_DIR = BASE_DIR / "data"
SCENES_FILE = DATA_DIR / "poses.csv"  # CSV with named poses
KEYBOARD_HELP_TEXT = dedent(
        """
        KEYBOARD CONTROLS
        ═════════════════════════════════════════════════════

        FINGER SELECTION:
            1, 2, 3, 4    Select finger (Pointer, Middle, Ring, Thumb)

        MOVEMENT CONTROLS:
            Up Arrow      Close finger (increase position)
            Down Arrow    Open finger (decrease position)
            Right Arrow   Move finger right
            Left Arrow    Move finger left

        QUICK ACTIONS:
            Q             Fully close selected finger
            E             Fully open selected finger
            C             Center left/right position

        PRECISION MODIFIERS:
            Normal        1° per keypress (precise, default)
            Shift + Key   5° per keypress (normal movement)
            Ctrl + Key    10° per keypress (fast movement)

        EXAMPLES:
            Press 1       → Select Pointer finger
            Press ↑       → Close 1° (precise)
            Shift + ↑     → Close 5° (normal)
            Ctrl + ↑      → Close 10° (fast)
            Press Q       → Fully close to 110°
            Press E       → Fully open to 0°
            Press C       → Center to 0° left/right
        """
)


def default_serial_port():
    """Return platform-specific default serial port."""
    return 'COM9' if os.name == 'nt' else '/dev/ttyACM0'


def ensure_data_dir():
    """Create the data directory if it does not already exist."""
    DATA_DIR.mkdir(exist_ok=True)


def clamp(value, min_value, max_value):
    """Clamp a numeric value between bounds."""
    return max(min_value, min(max_value, value))


def load_scene_definitions():
    """Load pose definitions from CSV file."""
    if not SCENES_FILE.exists():
        return []
    scenes = []  # list of dicts: {name, positions[8*2], speed}
    with SCENES_FILE.open('r') as f:
        for line_num, line in enumerate(f, 1):
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            if line_num == 1 and 'scene' in line.lower():
                continue
            parts = [p.strip() for p in line.split(',')]
            if len(parts) < 10:
                continue
            try:
                scenes.append({
                    'name': parts[0],
                    'positions': [int(parts[i]) for i in range(1, 9)],
                    'speed': int(parts[9])
                })
            except (ValueError, IndexError):
                continue
    return scenes


def sequence_path(name: str) -> Path:
    """Return a safe path for a saved sequence name."""
    safe = ''.join(c for c in name if c.isalnum() or c in ('-', '_')).strip() or 'sequence'
    return DATA_DIR / f"sequence_{safe}.txt"


class Tooltip:
    """Simple tooltip displayed on widget hover."""

    def __init__(self, widget, text, delay=500):
        self.widget = widget
        self.text = text
        self.delay = delay
        self.tip_window = None
        self.after_id = None
        widget.bind('<Enter>', self.on_enter, add='+')
        widget.bind('<Leave>', self.on_leave, add='+')
        widget.bind('<ButtonPress>', self.on_leave, add='+')

    def on_enter(self, _event):
        self.schedule()

    def on_leave(self, _event):
        self.unschedule()
        self.hide_tip()

    def schedule(self):
        self.unschedule()
        self.after_id = self.widget.after(self.delay, self.show_tip)

    def unschedule(self):
        if self.after_id:
            self.widget.after_cancel(self.after_id)
            self.after_id = None

    def show_tip(self):
        if self.tip_window or not self.text:
            return
        x, y, _cx, cy = self.widget.bbox('insert') if self.widget.winfo_viewable() else (0, 0, 0, 0)
        x += self.widget.winfo_rootx() + 20
        y += self.widget.winfo_rooty() + cy + 20
        self.tip_window = tw = tk.Toplevel(self.widget)
        tw.wm_overrideredirect(True)
        tw.wm_geometry(f"+{x}+{y}")
        label = ttk.Label(tw, text=self.text, background='#ffffe0', relief='solid', borderwidth=1, padding=(4, 2))
        label.pack()

    def hide_tip(self):
        if self.tip_window:
            self.tip_window.destroy()
            self.tip_window = None


def attach_tooltip(widget, text):
    """Attach tooltip text to widget."""
    if not text:
        return
    Tooltip(widget, text)


class FingerControl:
    """Control widget for a single finger (2 servos)."""
    
    def __init__(self, parent, finger_name, servo1_id, servo2_id, controller, update_callback):
        self.servo1_id = servo1_id
        self.servo2_id = servo2_id
        self.controller = controller
        self.update_callback = update_callback
        
        # Create frame for this finger
        self.frame = ttk.LabelFrame(parent, text=finger_name, padding=10)
        
        # Mimic checkbox
        self.mimic_var = tk.BooleanVar(value=False)
        self.mimic_check = ttk.Checkbutton(
            self.frame, text="Mimic",
            variable=self.mimic_var
        )
        self.mimic_check.grid(row=0, column=0, columnspan=2, sticky='w')
        attach_tooltip(self.mimic_check, "When enabled, this finger mirrors close/open changes to other mimicking fingers.")
        
        # Position sliders
        ttk.Label(self.frame, text="Close/Open:").grid(row=1, column=0, sticky='w')
        self.pos_var = tk.IntVar(value=0)
        self.pos_slider = ttk.Scale(
            self.frame, from_=110, to=0, orient='vertical',
            variable=self.pos_var, command=self.on_position_change,
            length=200
        )
        self.pos_slider.grid(row=2, column=0, padx=5)
        self.pos_label = ttk.Label(self.frame, text="0°")
        self.pos_label.grid(row=3, column=0)
        attach_tooltip(self.pos_slider, "Drag or scroll to close/open the finger (0°=open, 110°=closed).")
        
        # Bind mouse wheel to position slider
        self.pos_slider.bind('<Button-4>', self.on_mouse_wheel)  # Linux scroll up
        self.pos_slider.bind('<Button-5>', self.on_mouse_wheel)  # Linux scroll down
        self.pos_slider.bind('<MouseWheel>', self.on_mouse_wheel)  # Windows/Mac
        
        # Side-to-side slider
        ttk.Label(self.frame, text="Left/Right:").grid(row=1, column=1, sticky='w')
        self.side_var = tk.IntVar(value=0)
        self.side_slider = ttk.Scale(
            self.frame, from_=40, to=-40, orient='horizontal',
            variable=self.side_var, command=self.on_side_change,
            length=200
        )
        self.side_slider.grid(row=2, column=1, padx=5)
        self.side_label = ttk.Label(self.frame, text="0°")
        self.side_label.grid(row=3, column=1)
        attach_tooltip(self.side_slider, "Drag to move finger laterally (negative=left, positive=right).")
        
        # Speed control
        ttk.Label(self.frame, text="Speed:").grid(row=4, column=0, columnspan=2, pady=(10,0))
        self.speed_var = tk.IntVar(value=3)
        speed_frame = ttk.Frame(self.frame)
        speed_frame.grid(row=5, column=0, columnspan=2)
        for i in range(1, 7):
            ttk.Radiobutton(
                speed_frame, text=str(i), value=i,
                variable=self.speed_var
            ).pack(side='left', padx=2)
        attach_tooltip(speed_frame, "Servo motion speed (1=slowest, 6=fastest).")
        
        # Center button
        self.center_btn = ttk.Button(
            self.frame, text="Center", 
            command=self.center_finger
        )
        self.center_btn.grid(row=6, column=0, columnspan=2, pady=(10,0), sticky='ew')
        attach_tooltip(self.center_btn, "Reset left/right offset to 0° for this finger.")
    
    def center_finger(self):
        """Center this finger's side-to-side movement."""
        self.side_var.set(0)
        self.on_side_change(0)
    
    def on_mouse_wheel(self, event):
        """Handle mouse wheel scrolling on position slider."""
        if event.num == 4 or event.delta > 0:  # Scroll up
            self.adjust_position(5)
        elif event.num == 5 or event.delta < 0:  # Scroll down
            self.adjust_position(-5)
        else:
            return

    def adjust_position(self, delta):
        """Adjust close/open slider by delta degrees."""
        current = self.pos_var.get()
        new_val = clamp(current + delta, 0, 110)
        if new_val == current:
            return None
        self.pos_var.set(new_val)
        self.on_position_change(new_val)
        return new_val

    def adjust_side(self, delta):
        """Adjust side-to-side slider by delta degrees."""
        current = self.side_var.get()
        new_val = clamp(current + delta, -40, 40)
        if new_val == current:
            return None
        self.side_var.set(new_val)
        self.on_side_change(new_val)
        return new_val
    
    def on_position_change(self, value):
        """Handle position slider change."""
        pos = int(float(value))
        self.pos_label.config(text=f"{pos}°")
        self.update_callback(mimic_source=self if self.mimic_var.get() else None)
    
    def on_side_change(self, value):
        """Handle side slider change."""
        side = int(float(value))
        self.side_label.config(text=f"{side}°")
        self.update_callback()
    
    def get_positions(self):
        """Get current positions for both servos."""
        base_pos = self.pos_var.get()
        side_offset = self.side_var.get()
        
        # Servo 1 gets base + side offset
        # Servo 2 gets base - side offset (inverted because even ID)
        pos1 = base_pos + side_offset
        pos2 = base_pos - side_offset
        
        return pos1, pos2
    
    def get_speed(self):
        """Get current speed setting."""
        return self.speed_var.get()
    
    def set_positions(self, pos1, pos2):
        """Set slider positions from servo values."""
        # Calculate base position and side offset
        base_pos = (pos1 + pos2) // 2
        side_offset = pos1 - base_pos
        
        self.pos_var.set(base_pos)
        self.side_var.set(side_offset)
        self.pos_label.config(text=f"{base_pos}°")
        self.side_label.config(text=f"{side_offset}°")


class AmazingHandGUI:
    """Main GUI application for AmazingHand control."""
    
    def __init__(self, port=None, baudrate=1000000):
        if port is None:
            port = default_serial_port()
        self.root = tk.Tk()
        self.root.title("AmazingHand Controller")
        self.root.geometry("1400x800")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Connect to controller
        print(f"Connecting to {port} at {baudrate} baud...")
        self.controller = Scs0009PyController(
            serial_port=port,
            baudrate=baudrate,
            timeout=0.5
        )
        print("Connected!")
        
        # Enable torque for all servos
        for servo_id in range(1, 9):
            self.controller.write_torque_enable(servo_id, 1)
        print("Torque enabled for all servos")
        
        # Data storage for charts
        self.max_data_points = 100
        self.rolling_chart = True  # Rolling window mode
        self.time_data = []
        self.servo_data = {
            'target_pos': [[] for _ in range(8)],
            'current_pos': [[] for _ in range(8)],
            'load': [[] for _ in range(8)],
            'temperature': [[] for _ in range(8)],
            'voltage': [[] for _ in range(8)]
        }
        self.start_time = time.time()
        
        # Sequence control flags
        self.stop_sequence = False
        self.sequence_running = False
        self.saved_sequence = []
        
        # Keyboard control
        self.selected_finger_idx = 0  # Default to first finger
        
        # Create main container with paned window
        paned = ttk.PanedWindow(self.root, orient='horizontal')
        paned.pack(fill='both', expand=True)
        
        # Left panel - controls
        left_frame = ttk.Frame(paned, padding=10)
        paned.add(left_frame, weight=1)
        
        # Right panel - charts
        right_frame = ttk.Frame(paned, padding=10)
        paned.add(right_frame, weight=2)
        
        # Title
        ttk.Label(
            left_frame, text="AmazingHand Controller",
            font=('Arial', 16, 'bold')
        ).grid(row=0, column=0, columnspan=6, pady=10)
        
        # Create finger controls
        self.fingers = []
        self.finger_names = ['Pointer finger', 'Middle finger', 'Ring finger', 'Thumb']
        servo_pairs = [(1, 2), (3, 4), (5, 6), (7, 8)]
        
        for idx, (name, (s1, s2)) in enumerate(zip(self.finger_names, servo_pairs)):
            if idx < 3:  # First row: fingers 1, 2, 3
                row = 1
                col = idx * 2
            else:  # Second row: finger 4 (thumb)
                row = 2
                col = 0
            
            finger = FingerControl(
                left_frame, name, s1, s2, self.controller, self.on_finger_update
            )
            finger.frame.grid(row=row, column=col, columnspan=2, padx=5, pady=5)
            self.fingers.append(finger)
        
        # Global controls
        control_frame = ttk.LabelFrame(left_frame, text="Global Controls", padding=10)
        control_frame.grid(row=3, column=0, columnspan=6, pady=10, sticky='ew')
        
        # Preset buttons
        self.open_all_btn = ttk.Button(
            control_frame, text="Open All", command=self.open_all
        )
        self.open_all_btn.pack(side='left', padx=5)
        attach_tooltip(self.open_all_btn, "Set every finger to fully open (0°).")
        
        self.close_all_btn = ttk.Button(
            control_frame, text="Close All", command=self.close_all
        )
        self.close_all_btn.pack(side='left', padx=5)
        attach_tooltip(self.close_all_btn, "Set every finger to fully closed (110°).")
        
        self.center_all_btn = ttk.Button(
            control_frame, text="Center All", command=self.center_all
        )
        self.center_all_btn.pack(side='left', padx=5)
        attach_tooltip(self.center_all_btn, "Reset all side-to-side offsets to 0°.")
        
        self.save_scene_btn = ttk.Button(
            control_frame, text="Save Pose", command=self.save_scene
        )
        self.save_scene_btn.pack(side='left', padx=5)
        attach_tooltip(self.save_scene_btn, "Capture current finger positions to the pose list.")

        # Pose selection dropdown + Set Pose button
        self.pose_var = tk.StringVar(value="")
        self.pose_combo = ttk.Combobox(
            control_frame,
            textvariable=self.pose_var,
            state='readonly',
            width=14,
            values=[s['name'] for s in load_scene_definitions()] or ["<no poses>"]
        )
        self.pose_combo.pack(side='left', padx=5)
        attach_tooltip(self.pose_combo, "Select a saved pose from data/poses.csv.")

        self.load_scene_btn = ttk.Button(
            control_frame, text="Set Pose", command=self.set_selected_pose
        )
        self.load_scene_btn.pack(side='left', padx=5)
        attach_tooltip(self.load_scene_btn, "Apply the selected pose to all fingers.")
        
        self.execute_scenes_btn = ttk.Button(
            control_frame, text="Execute Pose Seq", command=self.execute_scenes
        )
        self.execute_scenes_btn.pack(side='left', padx=5)
        attach_tooltip(self.execute_scenes_btn, "Build and run a sequence of poses.")
        
        self.stop_sequence_btn = ttk.Button(
            control_frame, text="Stop Sequence", command=self.stop_sequence_exec
        )
        self.stop_sequence_btn.pack(side='left', padx=5)
        self.stop_sequence_btn.state(['disabled'])
        attach_tooltip(self.stop_sequence_btn, "Interrupt the currently running pose sequence.")
        
        # Keyboard control indicator
        self.kb_label = ttk.Label(
            control_frame, 
            text=f"KB: {self.finger_names[0]}",
            relief='sunken',
            padding=5
        )
        self.kb_label.pack(side='left', padx=10)
        attach_tooltip(self.kb_label, "Indicates which finger the keyboard controls target.")
        
        # Help button for keyboard shortcuts
        self.kb_help_btn = ttk.Button(
            control_frame, text="KB Help", command=self.show_keyboard_help, width=8
        )
        self.kb_help_btn.pack(side='left', padx=2)
        attach_tooltip(self.kb_help_btn, "Show all keyboard shortcuts and modifiers.")
        
        # Bind keyboard events
        self.root.bind('<Key>', self.on_key_press)
        
        # Status bar
        self.status_var = tk.StringVar(value="Ready")
        status_bar = ttk.Label(
            left_frame, textvariable=self.status_var,
            relief='sunken', anchor='w'
        )
        status_bar.grid(row=4, column=0, columnspan=6, sticky='ew', pady=(10,0))
        attach_tooltip(status_bar, "Most recent action, warning, or error message.")
        
        # Log output
        log_frame = ttk.LabelFrame(left_frame, text="Execution Log", padding=5)
        log_frame.grid(row=5, column=0, columnspan=6, sticky='nsew', pady=(10,0))
        
        # Make row 5 expandable
        left_frame.grid_rowconfigure(5, weight=1)
        
        log_scroll = ttk.Scrollbar(log_frame)
        log_scroll.pack(side='right', fill='y')
        
        self.log_text = tk.Text(log_frame, height=8, wrap='word', yscrollcommand=log_scroll.set, state='disabled')
        self.log_text.pack(side='left', fill='both', expand=True)
        log_scroll.config(command=self.log_text.yview)
        attach_tooltip(self.log_text, "History of key actions and sequence progress (read-only).")
        
        # Chart panel setup
        self.setup_chart_panel(right_frame)
        
        # Update flag
        self.updating = False
        self.update_pending = False
        
        # Start monitoring thread
        self.monitoring = True
        self.monitor_thread = threading.Thread(target=self.monitor_servos, daemon=True)
        self.monitor_thread.start()
    
    def setup_chart_panel(self, parent):
        """Setup the chart display panel."""
        # Title
        chart_title = ttk.Label(
            parent, text="Servo Monitoring",
            font=('Arial', 14, 'bold')
        )
        chart_title.pack(pady=5)
        attach_tooltip(chart_title, "Live telemetry from all servos (position, load, temperature, voltage).")
        
        # Metric selection
        select_frame = ttk.Frame(parent)
        select_frame.pack(fill='x', padx=5, pady=5)
        
        display_label = ttk.Label(select_frame, text="Display:")
        display_label.pack(side='left', padx=5)
        attach_tooltip(display_label, "Pick which telemetry metric to plot.")
        
        self.chart_metric = tk.StringVar(value='current_pos')
        metrics = [
            ('Position', 'current_pos'),
            ('Target vs Current', 'target_vs_current'),
            ('Load/Force', 'load'),
            ('Temperature', 'temperature'),
            ('Voltage', 'voltage')
        ]
        
        metric_tips = {
            'current_pos': "Plot servo position history in degrees.",
            'target_vs_current': "Overlay commanded target vs. measured position.",
            'load': "Show load/force for each servo.",
            'temperature': "Display internal temperature readings.",
            'voltage': "Monitor supply voltage levels."
        }
        for label, value in metrics:
            metric_btn = ttk.Radiobutton(
                select_frame, text=label, value=value,
                variable=self.chart_metric,
                command=self.update_chart
            )
            metric_btn.pack(side='left', padx=2)
            attach_tooltip(metric_btn, metric_tips.get(value, ""))
        
        # Servo selection
        servo_frame = ttk.Frame(parent)
        servo_frame.pack(fill='x', padx=5, pady=5)
        
        servos_label = ttk.Label(servo_frame, text="Servos:")
        servos_label.pack(side='left', padx=5)
        attach_tooltip(servos_label, "Toggle which servo traces are visible.")
        
        self.servo_visible = [tk.BooleanVar(value=True) for _ in range(8)]
        for i in range(8):
            servo_check = ttk.Checkbutton(
                servo_frame, text=f"S{i+1}",
                variable=self.servo_visible[i],
                command=self.update_chart
            )
            servo_check.pack(side='left', padx=2)
            attach_tooltip(servo_check, f"Show/hide servo {i+1} in the plot.")
        
        # Add select/deselect all buttons
        all_btn = ttk.Button(servo_frame, text="All", command=self.select_all_servos, width=6)
        all_btn.pack(side='left', padx=5)
        attach_tooltip(all_btn, "Enable all servo traces.")
        none_btn = ttk.Button(servo_frame, text="None", command=self.deselect_all_servos, width=6)
        none_btn.pack(side='left', padx=2)
        attach_tooltip(none_btn, "Hide all servo traces (useful before selecting a subset).")
        
        # Rolling chart option
        self.rolling_var = tk.BooleanVar(value=True)
        rolling_check = ttk.Checkbutton(
            servo_frame, text="Rolling",
            variable=self.rolling_var,
            command=self.toggle_rolling
        )
        rolling_check.pack(side='left', padx=10)
        attach_tooltip(rolling_check, "Limit chart to latest samples instead of growing indefinitely.")
        
        # Clear chart button
        clear_btn = ttk.Button(servo_frame, text="Clear", command=self.clear_chart_data, width=6)
        clear_btn.pack(side='left', padx=2)
        attach_tooltip(clear_btn, "Reset collected telemetry and start fresh.")
        
        # Create matplotlib figure
        self.fig = Figure(figsize=(10, 6), dpi=100)
        self.ax = self.fig.add_subplot(111)
        
        self.canvas = FigureCanvasTkAgg(self.fig, parent)
        canvas_widget = self.canvas.get_tk_widget()
        canvas_widget.pack(fill='both', expand=True, padx=5, pady=5)
        attach_tooltip(canvas_widget, "Scroll to zoom, drag to pan; updates roughly every second.")
        
        # Initialize plot
        self.update_chart()
    
    def monitor_servos(self):
        """Background thread to monitor servo states."""
        while self.monitoring:
            try:
                current_time = time.time() - self.start_time
                
                # Collect data for all servos first
                success = False
                for servo_id in range(1, 9):
                    idx = servo_id - 1
                    
                    try:
                        # Read position
                        current_pos_rad = self.controller.read_present_position(servo_id)
                        if isinstance(current_pos_rad, np.ndarray):
                            current_pos_rad = current_pos_rad.item()
                        current_pos = np.rad2deg(current_pos_rad)
                        if servo_id % 2 == 0:
                            current_pos = -current_pos
                        
                        # Read load (force)
                        load = self.controller.read_present_load(servo_id)
                        if isinstance(load, np.ndarray):
                            load = load.item()
                        
                        # Read temperature
                        temp = self.controller.read_present_temperature(servo_id)
                        if isinstance(temp, np.ndarray):
                            temp = temp.item()
                        
                        # Read voltage
                        voltage = self.controller.read_present_voltage(servo_id)
                        if isinstance(voltage, np.ndarray):
                            voltage = voltage.item()
                        
                        # Get target position from finger controls
                        finger_idx = idx // 2
                        servo_in_finger = idx % 2
                        if finger_idx < len(self.fingers):
                            pos1, pos2 = self.fingers[finger_idx].get_positions()
                            target = pos1 if servo_in_finger == 0 else pos2
                        else:
                            target = 0
                        
                        # Store data
                        self.servo_data['current_pos'][idx].append(current_pos)
                        self.servo_data['load'][idx].append(load)
                        self.servo_data['temperature'][idx].append(temp)
                        self.servo_data['voltage'][idx].append(voltage)
                        self.servo_data['target_pos'][idx].append(target)
                        
                        success = True
                        
                    except Exception as e:
                        # On error, append None or last value to keep arrays in sync
                        for key in self.servo_data:
                            if len(self.servo_data[key][idx]) > 0:
                                # Repeat last value
                                self.servo_data[key][idx].append(self.servo_data[key][idx][-1])
                            else:
                                # First time, append 0
                                self.servo_data[key][idx].append(0)
                        print(f"Error reading servo {servo_id}: {e}")
                
                # Only add time point if at least one servo succeeded
                if success:
                    self.time_data.append(current_time)
                    
                    # Keep only recent data if rolling mode is enabled
                    if self.rolling_chart and len(self.time_data) > self.max_data_points:
                        self.time_data.pop(0)
                        for key in self.servo_data:
                            for idx in range(8):
                                if len(self.servo_data[key][idx]) > 0:
                                    self.servo_data[key][idx].pop(0)
                
                # Update chart every 10 readings
                if len(self.time_data) % 10 == 0:
                    self.root.after(0, self.update_chart)
                
                time.sleep(0.1)  # 10 Hz update rate
            
            except Exception as e:
                print(f"Monitor error: {e}")
                time.sleep(1)
    
    def update_chart(self):
        """Update the chart display."""
        try:
            self.ax.clear()
            
            metric = self.chart_metric.get()
            colors = plt.cm.tab10(np.linspace(0, 1, 8))
            
            # Check if we have any data
            if len(self.time_data) == 0:
                self.ax.text(0.5, 0.5, 'Waiting for data...', 
                           ha='center', va='center', transform=self.ax.transAxes)
                self.canvas.draw()
                return
            
            if metric == 'target_vs_current':
                # Show target and current for visible servos
                for i in range(8):
                    if not self.servo_visible[i].get():
                        continue
                    if len(self.servo_data['current_pos'][i]) == 0:
                        continue
                    
                    # Use the minimum length to avoid index errors
                    data_len = min(len(self.time_data), len(self.servo_data['current_pos'][i]))
                    if data_len == 0:
                        continue
                    
                    time_slice = self.time_data[-data_len:]
                    current_slice = self.servo_data['current_pos'][i][-data_len:]
                    target_slice = self.servo_data['target_pos'][i][-data_len:]
                    
                    self.ax.plot(time_slice, current_slice, 
                               label=f'S{i+1} Current', color=colors[i], linestyle='-')
                    self.ax.plot(time_slice, target_slice,
                               label=f'S{i+1} Target', color=colors[i], linestyle='--', alpha=0.5)
                
                self.ax.set_ylabel('Position (degrees)')
                self.ax.set_title('Target vs Current Position')
            
            else:
                # Show single metric for visible servos
                data_key = metric
                labels = {
                    'current_pos': 'Position (degrees)',
                    'load': 'Load/Force',
                    'temperature': 'Temperature (°C)',
                    'voltage': 'Voltage (V)'
                }
                
                for i in range(8):
                    if not self.servo_visible[i].get():
                        continue
                    if len(self.servo_data[data_key][i]) == 0:
                        continue
                    
                    # Use the minimum length to avoid index errors
                    data_len = min(len(self.time_data), len(self.servo_data[data_key][i]))
                    if data_len == 0:
                        continue
                    
                    time_slice = self.time_data[-data_len:]
                    data_slice = self.servo_data[data_key][i][-data_len:]
                    
                    self.ax.plot(time_slice, data_slice,
                               label=f'Servo {i+1}', color=colors[i])
                
                self.ax.set_ylabel(labels.get(data_key, data_key))
                self.ax.set_title(labels.get(data_key, data_key))
            
            self.ax.set_xlabel('Time (seconds)')
            
            # Only add legend if there are labeled lines
            if self.ax.get_legend_handles_labels()[0]:
                self.ax.legend(loc='upper right', fontsize=8, ncol=2)
            
            self.ax.grid(True, alpha=0.3)
            
            self.canvas.draw()
        
        except Exception as e:
            print(f"Chart update error: {e}")
            import traceback
            traceback.print_exc()
    
    def select_all_servos(self):
        """Select all servos for display in chart."""
        for var in self.servo_visible:
            var.set(True)
        self.update_chart()
    
    def deselect_all_servos(self):
        """Deselect all servos from chart display."""
        for var in self.servo_visible:
            var.set(False)
        self.update_chart()
    
    def toggle_rolling(self):
        """Toggle rolling chart mode."""
        self.rolling_chart = self.rolling_var.get()
        self.status_var.set(f"Chart mode: {'Rolling' if self.rolling_chart else 'Continuous'}")
    
    def clear_chart_data(self):
        """Clear all chart data."""
        self.time_data = []
        for key in self.servo_data:
            for idx in range(8):
                self.servo_data[key][idx] = []
        self.start_time = time.time()
        self.update_chart()
        self.status_var.set("Chart data cleared")
    
    def on_closing(self):
        """Handle window close event."""
        print("Shutting down...")
        self.monitoring = False
        if hasattr(self, 'monitor_thread'):
            self.monitor_thread.join(timeout=1)
        self.root.destroy()
        sys.exit(0)
    
    def on_finger_update(self, mimic_source=None):
        """Called when any finger control changes."""
        # Handle mimic functionality
        if mimic_source is not None:
            source_pos = mimic_source.pos_var.get()
            # Apply to all other fingers that have mimic enabled
            for finger in self.fingers:
                if finger != mimic_source and finger.mimic_var.get():
                    finger.pos_var.set(source_pos)
                    finger.pos_label.config(text=f"{source_pos}°")
        
        if not self.updating:
            self.update_pending = True
            self.root.after(50, self.send_positions)  # Debounce updates
    
    def send_positions(self):
        """Send current positions to servos."""
        if not self.update_pending:
            return
        
        self.update_pending = False
        self.updating = True
        
        try:
            servo_ids = []
            positions = []
            speeds = []
            
            for finger in self.fingers:
                pos1, pos2 = finger.get_positions()
                speed = finger.get_speed()
                
                servo_ids.extend([finger.servo1_id, finger.servo2_id])
                positions.extend([pos1, pos2])
                speeds.extend([speed, speed])
            
            # Set speeds
            for servo_id, speed in zip(servo_ids, speeds):
                self.controller.write_goal_speed(servo_id, speed)
            
            # Convert positions and send
            positions_rad = []
            for servo_id, pos in zip(servo_ids, positions):
                if servo_id % 2 == 0:
                    positions_rad.append(np.deg2rad(-pos))
                else:
                    positions_rad.append(np.deg2rad(pos))
            
            self.controller.sync_write_goal_position(servo_ids, positions_rad)
            
            self.status_var.set(f"Updated: {' '.join(str(p) for p in positions)}")
        
        except Exception as e:
            self.status_var.set(f"Error: {e}")
        
        finally:
            self.updating = False
    
    def open_all(self):
        """Set all fingers to open position."""
        for finger in self.fingers:
            finger.pos_var.set(0)
            finger.side_var.set(0)
            finger.on_position_change(0)
            finger.on_side_change(0)
        self.send_positions()
    
    def close_all(self):
        """Set all fingers to closed position."""
        for finger in self.fingers:
            finger.pos_var.set(110)
            finger.side_var.set(0)
            finger.on_position_change(110)
            finger.on_side_change(0)
        self.send_positions()
    
    def center_all(self):
        """Center all side-to-side movements."""
        for finger in self.fingers:
            finger.side_var.set(0)
            finger.on_side_change(0)
        self.send_positions()
    
    def stop_sequence_exec(self):
        """Stop the currently running sequence."""
        if not self.sequence_running:
            return
        self.stop_sequence = True
        self.log("Stop requested")
        self.status_var.set("Stopping sequence...")
        # Keep the button disabled until a new sequence starts
        self.stop_sequence_btn.state(['disabled'])
    
    def show_keyboard_help(self):
        """Display keyboard shortcuts help dialog."""
        help_dialog = tk.Toplevel(self.root)
        help_dialog.title("Keyboard Controls")
        help_dialog.geometry("500x400")
        text_widget = tk.Text(help_dialog, wrap='word', padx=20, pady=20)
        text_widget.pack(fill='both', expand=True)
        text_widget.insert('1.0', KEYBOARD_HELP_TEXT)
        text_widget.config(state='disabled', font=('Courier', 10))

        ttk.Button(help_dialog, text="Close", command=help_dialog.destroy).pack(pady=10)
    
    def on_key_press(self, event):
        """Handle keyboard input for finger control."""
        key = (event.char or '').lower()
        keysym = event.keysym
        
        # Select finger with keys 1-4 (use keysym so it works even when char is empty)
        if keysym in ('1', '2', '3', '4'):
            self.selected_finger_idx = int(keysym) - 1
            self.kb_label.config(text=f"KB: {self.finger_names[self.selected_finger_idx]}")
            self.status_var.set(f"Selected: {self.finger_names[self.selected_finger_idx]}")
            return
        
        # Control selected finger with arrow keys
        if self.selected_finger_idx >= len(self.fingers):
            return
        
        finger = self.fingers[self.selected_finger_idx]
        
        # Variable step size: 1° default (precise), 5° with Shift, 10° with Ctrl (fast)
        # Use Tk state bitmasks more carefully to handle Linux quirks
        # Control = 0x0004, Shift = 0x0001
        # Check Control first since it takes precedence
        has_ctrl = bool(event.state & 0x0004)
        has_shift = bool(event.state & 0x0001)
        
        if has_ctrl:  # Ctrl key held (fast)
            step = 10
            mode = "fast"
        elif has_shift:  # Shift key held (normal)
            step = 5
            mode = "normal"
        else:
            step = 1
            mode = "precise"
        
        moved = False
        direction = ""

        # Use keysym for arrow keys so it works regardless of layout
        if keysym == 'Up':  # Close (increase position)
            new_val = finger.adjust_position(step)
            if new_val is not None:
                moved = True
                direction = f"Close → {new_val}°"
        elif keysym == 'Down':  # Open (decrease position)
            new_val = finger.adjust_position(-step)
            if new_val is not None:
                moved = True
                direction = f"Open → {new_val}°"
        elif keysym == 'Right':  # Move right (increase side)
            new_val = finger.adjust_side(step)
            if new_val is not None:
                moved = True
                direction = f"Right → {new_val}°"
        elif keysym == 'Left':  # Move left (decrease side)
            new_val = finger.adjust_side(-step)
            if new_val is not None:
                moved = True
                direction = f"Left → {new_val}°"
        elif key == 'q':  # Quick close all the way
            finger.pos_var.set(110)
            finger.on_position_change(110)
            moved = True
            direction = "Fully closed"
        elif key == 'e':  # Quick open all the way
            finger.pos_var.set(0)
            finger.on_position_change(0)
            moved = True
            direction = "Fully open"
        elif key == 'c':  # Center side-to-side
            finger.side_var.set(0)
            finger.on_side_change(0)
            moved = True
            direction = "Centered"
        
        if moved:
            self.status_var.set(f"{self.finger_names[self.selected_finger_idx]} ({mode}): {direction}")
    
    def read_current(self):
        """Read current positions from servos and update sliders."""
        try:
            for finger in self.fingers:
                # Read positions
                pos1_rad = self.controller.read_present_position(finger.servo1_id)
                pos2_rad = self.controller.read_present_position(finger.servo2_id)
                
                # Convert to degrees
                if isinstance(pos1_rad, np.ndarray):
                    pos1_rad = pos1_rad.item()
                if isinstance(pos2_rad, np.ndarray):
                    pos2_rad = pos2_rad.item()
                
                pos1 = int(np.rad2deg(pos1_rad))
                pos2 = int(np.rad2deg(-pos2_rad))  # Negate for even ID
                
                # Update sliders
                finger.set_positions(pos1, pos2)
            
            self.status_var.set("Read current positions")
        
        except Exception as e:
            self.status_var.set(f"Error reading: {e}")
    
    def save_scene(self):
        """Save current position as a scene."""
        # Create dialog
        dialog = tk.Toplevel(self.root)
        dialog.title("Save Scene")
        dialog.geometry("300x150")
        
        ttk.Label(dialog, text="Scene Name:").pack(pady=10)
        name_var = tk.StringVar()
        name_entry = ttk.Entry(dialog, textvariable=name_var, width=30)
        name_entry.pack(pady=5)
        name_entry.focus()
        
        ttk.Label(dialog, text="Speed:").pack(pady=10)
        speed_var = tk.IntVar(value=3)
        speed_frame = ttk.Frame(dialog)
        speed_frame.pack()
        for i in range(1, 7):
            ttk.Radiobutton(speed_frame, text=str(i), value=i, variable=speed_var).pack(side='left')
        
        def do_save():
            name = name_var.get().strip()
            if not name:
                return
            
            try:
                # Get current positions
                positions = []
                for finger in self.fingers:
                    pos1, pos2 = finger.get_positions()
                    positions.extend([pos1, pos2])
                
                # Write to file
                ensure_data_dir()
                with SCENES_FILE.open('a') as f:
                    pos_str = ", ".join(f"{p:4d}" for p in positions)
                    f.write(f"{name}, {pos_str}, {speed_var.get():5d}\n")
                
                self.status_var.set(f"Saved pose '{name}'")
                dialog.destroy()
            
            except Exception as e:
                self.status_var.set(f"Error saving: {e}")
        
        ttk.Button(dialog, text="Save", command=do_save).pack(pady=10)

    def load_scene(self):
        """Deprecated: kept for compatibility, now routed through set_selected_pose."""
        # For backward compatibility if called somewhere else, just call new logic
        self.set_selected_pose()

    def set_selected_pose(self):
        """Apply the currently selected pose from the dropdown to the hand."""
        try:
            # Read available poses
            if not SCENES_FILE.exists():
                self.status_var.set("No poses file found")
                return

            scenes = load_scene_definitions()
            if not scenes:
                self.status_var.set("No poses found in file")
                return
            
            # If there is a pose selected in the dropdown, use it; otherwise pick first
            selected_name = self.pose_var.get().strip() if hasattr(self, 'pose_var') else ''
            scene = None
            if selected_name:
                for s in scenes:
                    if s['name'] == selected_name:
                        scene = s
                        break
            if scene is None:
                scene = scenes[0]

            # Apply positions to fingers
            for idx, finger in enumerate(self.fingers):
                pos1 = scene['positions'][idx * 2]
                pos2 = scene['positions'][idx * 2 + 1]
                finger.set_positions(pos1, pos2)
                finger.speed_var.set(scene['speed'])

            # Trigger position update
            self.update_pending = True
            self.send_positions()

            self.status_var.set(f"Set pose '{scene['name']}'")
        
        except Exception as e:
            self.status_var.set(f"Error setting pose: {e}")
    
    def execute_scenes(self):
        """Execute a sequence of poses."""
        try:
            # Read available scenes
            if not SCENES_FILE.exists():
                self.status_var.set("No scenes file found")
                return

            scenes_list = load_scene_definitions()
            if not scenes_list:
                self.status_var.set("No scenes found in file")
                return

            scenes = {scene['name']: scene for scene in scenes_list}
            
            # Create execution dialog
            dialog = tk.Toplevel(self.root)
            dialog.title("Execute Pose Sequence")
            dialog.geometry("700x600")
            
            # Main container
            main_container = ttk.Frame(dialog, padding=10)
            main_container.pack(fill='both', expand=True)
            
            ttk.Label(main_container, text="Build Pose Sequence:", font=('Arial', 12, 'bold')).pack(pady=(0,10))
            
            # Two-column layout
            columns = ttk.Frame(main_container)
            columns.pack(fill='both', expand=True)
            
            # Two-column layout
            columns = ttk.Frame(main_container)
            columns.pack(fill='both', expand=True)
            
            # Available poses
            left_frame = ttk.LabelFrame(columns, text="Available Poses", padding=10)
            left_frame.pack(side='left', fill='both', expand=True, padx=(0,5))
            
            scenes_list = tk.Listbox(left_frame, height=20)
            scenes_list.pack(fill='both', expand=True, pady=5)
            for name in sorted(scenes.keys()):
                scenes_list.insert(tk.END, name)
            
            # Sequence list
            right_frame = ttk.LabelFrame(columns, text="Pose Sequence (in order)", padding=10)
            right_frame.pack(side='left', fill='both', expand=True, padx=(5,0))
            
            sequence_list = tk.Listbox(right_frame, height=20)
            sequence_list.pack(fill='both', expand=True, pady=5)
            
            # Restore saved sequence
            for item in self.saved_sequence:
                sequence_list.insert(tk.END, item)
            
            # Controls section
            controls_frame = ttk.Frame(main_container)
            controls_frame.pack(fill='x', pady=(10,0))
            
            # Delay entry
            delay_frame = ttk.Frame(controls_frame)
            delay_frame.pack(side='left', padx=5)
            ttk.Label(delay_frame, text="Delay (sec):").pack(side='left', padx=(0,5))
            delay_var = tk.StringVar(value="1.0")
            ttk.Entry(delay_frame, textvariable=delay_var, width=8).pack(side='left')
            
            # Loop checkbox
            loop_var = tk.BooleanVar(value=False)
            ttk.Checkbutton(controls_frame, text="Loop", variable=loop_var).pack(side='left', padx=10)
            
            def add_to_sequence():
                selection = scenes_list.curselection()
                if selection:
                    scene_name = scenes_list.get(selection[0])
                    delay = delay_var.get()
                    if delay and float(delay) > 0:
                        sequence_list.insert(tk.END, f"{scene_name}|{delay}s")
                    else:
                        sequence_list.insert(tk.END, scene_name)
            
            # Bind double-click to add scene
            scenes_list.bind('<Double-Button-1>', lambda e: add_to_sequence())
            
            def add_delay():
                delay = delay_var.get()
                if delay and float(delay) > 0:
                    sequence_list.insert(tk.END, f"SLEEP:{delay}s")
            
            def remove_from_sequence():
                selection = sequence_list.curselection()
                if selection:
                    sequence_list.delete(selection[0])
            
            def clear_sequence():
                sequence_list.delete(0, tk.END)
                self.saved_sequence = []
            
            def move_up():
                selection = sequence_list.curselection()
                if selection and selection[0] > 0:
                    idx = selection[0]
                    item = sequence_list.get(idx)
                    sequence_list.delete(idx)
                    sequence_list.insert(idx - 1, item)
                    sequence_list.selection_set(idx - 1)
            
            def move_down():
                selection = sequence_list.curselection()
                if selection and selection[0] < sequence_list.size() - 1:
                    idx = selection[0]
                    item = sequence_list.get(idx)
                    sequence_list.delete(idx)
                    sequence_list.insert(idx + 1, item)
                    sequence_list.selection_set(idx + 1)
            
            # Buttons
            btn_frame = ttk.Frame(controls_frame)
            btn_frame.pack(side='left', padx=10)
            
            ttk.Button(btn_frame, text="Add Scene →", command=add_to_sequence, width=12).pack(side='left', padx=2)
            ttk.Button(btn_frame, text="Add Delay", command=add_delay, width=10).pack(side='left', padx=2)
            ttk.Button(btn_frame, text="Remove", command=remove_from_sequence, width=8).pack(side='left', padx=2)
            ttk.Button(btn_frame, text="Clear All", command=clear_sequence, width=8).pack(side='left', padx=2)
            ttk.Button(btn_frame, text="↑", command=move_up, width=3).pack(side='left', padx=2)
            ttk.Button(btn_frame, text="↓", command=move_down, width=3).pack(side='left', padx=2)
            
            def execute_sequence():
                if sequence_list.size() == 0:
                    return
                
                # Capture and save sequence before destroying dialog
                sequence_items = [sequence_list.get(i) for i in range(sequence_list.size())]
                self.saved_sequence = sequence_items  # Save for next time
                loop_enabled = loop_var.get()
                
                dialog.destroy()
                
                # Create stop flag and mark sequence as running
                self.stop_sequence = False
                self.sequence_running = True
                self.root.after(0, lambda: self.stop_sequence_btn.state(['!disabled']))
                
                # Execute sequence in background thread
                def run_sequence():
                    loop_count = 0
                    
                    while True:
                        loop_count += 1
                        if loop_enabled:
                            self.root.after(0, lambda c=loop_count: self.log(f"=== Loop iteration {c} ==="))
                        else:
                            self.root.after(0, lambda: self.log("=== Starting sequence execution ==="))
                        
                        for item in sequence_items:
                            # Check stop flag
                            if self.stop_sequence:
                                self.root.after(0, lambda: self.log("=== Sequence stopped ==="))
                                self.root.after(0, lambda: self.status_var.set("Sequence stopped"))
                                self.sequence_running = False
                                self.root.after(0, lambda: self.stop_sequence_btn.state(['disabled']))
                                return
                            
                            if item.startswith("SLEEP:"):
                                # Sleep delay
                                delay = float(item.split(':')[1].rstrip('s'))
                                self.root.after(0, lambda d=delay: self.status_var.set(f"Waiting {d}s..."))
                                self.root.after(0, lambda d=delay: self.log(f"Delay: {d}s"))
                                time.sleep(delay)
                            else:
                                # Scene execution
                                parts = item.split('|')
                                scene_name = parts[0]
                                
                                if scene_name not in scenes:
                                    print(f"Scene '{scene_name}' not found, skipping")
                                    continue
                                
                                scene = scenes[scene_name]
                                
                                # Apply to GUI and wait for it to complete
                                self.root.after(0, lambda s=scene, n=scene_name: self._apply_scene(s, n))
                                
                                # Wait for movement to complete
                                if len(parts) > 1:
                                    delay = float(parts[1].rstrip('s'))
                                    self.root.after(0, lambda d=delay: self.log(f"  → Wait: {d}s"))
                                    time.sleep(delay)
                                else:
                                    # Calculate timeout based on speed
                                    speed = scene['speed']
                                    timeout = 15.0 - (speed - 1) * 2.4
                                    self.root.after(0, lambda t=timeout: self.log(f"  → Auto-wait: {t:.1f}s"))
                                    time.sleep(timeout)
                        
                        # Check if we should loop
                        if not loop_enabled:
                            break
                        
                        # Small delay between loop iterations
                        if loop_enabled and not self.stop_sequence:
                            time.sleep(0.5)
                    
                    self.sequence_running = False
                    self.root.after(0, lambda: self.stop_sequence_btn.state(['disabled']))
                    self.root.after(0, lambda: self.status_var.set("Sequence complete"))
                    self.root.after(0, lambda: self.log("=== Sequence complete ==="))
                
                threading.Thread(target=run_sequence, daemon=True).start()
            
            def save_sequence_to_file():
                """Save current sequence to a file."""
                if sequence_list.size() == 0:
                    self.status_var.set("No sequence to save")
                    return
                
                # Create save dialog
                save_dialog = tk.Toplevel(dialog)
                save_dialog.title("Save Sequence")
                save_dialog.geometry("400x150")
                
                ttk.Label(save_dialog, text="Sequence Name:").pack(pady=10)
                name_var = tk.StringVar()
                name_entry = ttk.Entry(save_dialog, textvariable=name_var, width=30)
                name_entry.pack(pady=5)
                name_entry.focus()
                
                def do_save():
                    name = name_var.get().strip()
                    if not name:
                        return
                    
                    try:
                        sequence = [sequence_list.get(i) for i in range(sequence_list.size())]
                        ensure_data_dir()
                        filepath = sequence_path(name)

                        with filepath.open('w') as f:
                            for item in sequence:
                                f.write(f"{item}\n")

                        self.status_var.set(f"Saved sequence to {filepath.name}")
                        self.log(f"Saved sequence: {filepath}")
                        save_dialog.destroy()
                    except Exception as e:
                        self.status_var.set(f"Error saving: {e}")
                
                ttk.Button(save_dialog, text="Save", command=do_save).pack(pady=10)
            
            def load_sequence_from_file():
                """Load sequence from a file."""
                from tkinter import filedialog

                ensure_data_dir()
                filename = filedialog.askopenfilename(
                    title="Load Sequence",
                    initialdir=str(DATA_DIR),
                    filetypes=[("Text files", "*.txt"), ("All files", "*.*")]
                )
                
                if not filename:
                    return
                
                try:
                    sequence_list.delete(0, tk.END)
                    
                    with open(filename, 'r') as f:
                        for line in f:
                            line = line.strip()
                            if line:
                                sequence_list.insert(tk.END, line)
                    
                    # Update saved sequence so it persists
                    self.saved_sequence = [sequence_list.get(i) for i in range(sequence_list.size())]
                    
                    loaded_name = os.path.basename(filename)
                    self.status_var.set(f"Loaded sequence from {loaded_name}")
                    self.log(f"Loaded sequence: {loaded_name}")
                except Exception as e:
                    self.status_var.set(f"Error loading: {e}")
                    import traceback
                    traceback.print_exc()
            
            # Execute/Cancel buttons at bottom
            exec_frame = ttk.Frame(main_container)
            exec_frame.pack(pady=(15,0))
            ttk.Button(exec_frame, text="Save Sequence", command=save_sequence_to_file, width=15).pack(side='left', padx=5)
            ttk.Button(exec_frame, text="Load Sequence", command=load_sequence_from_file, width=15).pack(side='left', padx=5)
            ttk.Button(exec_frame, text="Execute Sequence", command=execute_sequence, width=18).pack(side='left', padx=5)
            ttk.Button(exec_frame, text="Cancel", command=dialog.destroy, width=10).pack(side='left', padx=5)
        
        except Exception as e:
            self.status_var.set(f"Error: {e}")
            import traceback
            traceback.print_exc()
    
    def _apply_scene(self, scene, name):
        """Apply a scene to the GUI (called from main thread)."""
        for idx, finger in enumerate(self.fingers):
            pos1 = scene['positions'][idx * 2]
            pos2 = scene['positions'][idx * 2 + 1]
            finger.set_positions(pos1, pos2)
            finger.speed_var.set(scene['speed'])
        
        # Trigger position update
        self.update_pending = True
        self.send_positions()
        self.status_var.set(f"Executing: {name}")
        self.log(f"Scene: {name} (speed={scene['speed']})")
    
    def log(self, message):
        """Add message to log output."""
        self.log_text.config(state='normal')
        self.log_text.insert(tk.END, f"{message}\n")
        self.log_text.see(tk.END)
        self.log_text.config(state='disabled')
    
    def run(self):
        """Start the GUI main loop."""
        self.root.mainloop()


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="AmazingHand GUI Controller")
    parser.add_argument('--port', default=default_serial_port(), help='Serial port (defaults to COM9 on Windows, /dev/ttyACM0 elsewhere)')
    parser.add_argument('--baudrate', type=int, default=1000000, help='Baudrate')
    
    args = parser.parse_args()
    
    app = AmazingHandGUI(args.port, args.baudrate)
    app.run()


if __name__ == '__main__':
    main()

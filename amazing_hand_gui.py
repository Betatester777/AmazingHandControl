#!/usr/bin/env python3
# Copyright 2025 AmazingHand Control Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
AmazingHand GUI - Interactive Control Interface for Robotic Hand

OVERVIEW:
=========
This GUI provides comprehensive control for an 8-servo robotic hand with 4 fingers.
Each finger has 2 servos: one for open/close movement and one for side-to-side offset.

ARCHITECTURE:
=============
- Main Window: Left panel (finger controls) + Right panel (monitoring chart)
- Control Sections: Connection, Global Controls, Pose Management, Sequence Player
- Monitoring: Real-time servo telemetry with configurable metrics and visualization
- Data Storage: YAML-based configuration (data/hand_config.yaml)

KEY COMPONENTS:
===============
1. FingerControl class: Individual finger widget with sliders and speed control
   - Position slider (0-110°): Open/close movement
   - Side slider (-20 to +20°): Left/right offset  
   - Speed control (1-6): Servo movement speed
   - Servo IDs: Odd (position), Even (side) - e.g., Finger 1 uses servos 1 & 2

2. AmazingHandGUI class: Main application window
   - Connection Management: Auto-detect serial ports, connect/disconnect
   - Global Controls: Open All, Close All, Center All buttons
   - Pose Management: Save/load/apply 8-servo position sets
   - Sequence Player: Execute multi-step animations with individual servo speeds
   - Servo Monitor: Background thread collecting position, load, temp, voltage

3. Data Format (YAML):
   poses:
     pose_name:
       positions: [pos1, pos2, ..., pos8]  # Degrees for each servo
   sequences:
     sequence_name:
       steps:
         - "pose_name:speed1,speed2,...,speed8|delay"
         - "SLEEP:duration"

SERVO MAPPING:
==============
Servo 1: Pointer finger position (0=open, 110=closed)
Servo 2: Pointer finger side (-20=left, 0=center, +20=right)
Servo 3: Middle finger position
Servo 4: Middle finger side
Servo 5: Ring finger position  
Servo 6: Ring finger side
Servo 7: Thumb position
Servo 8: Thumb side

Note: Even-numbered servos (2,4,6,8) have inverted angles in code

KEYBOARD CONTROLS:
==================
1-4: Select finger | ↑/↓: Open/Close | ←/→: Move side
Q/E: Quick open/close | C: Center | Shift/Ctrl: Speed modifiers

THREADING MODEL:
================
- Main thread: GUI event loop, user interaction
- Monitor thread: Background servo telemetry (position, load, temp, voltage)
- Sequence thread: Executes multi-step sequences without blocking UI

EXTENSIBILITY:
==============
- Add new poses: Use "Add New" button or edit hand_config.yaml
- Create sequences: Use sequence management dialog, format steps as "pose:speeds|delay"
- Customize metrics: Modify setup_chart_panel() for new telemetry displays
- Validation: validate_name() prevents YAML-breaking characters in names

DEPENDENCIES:
=============
- rustypot: Servo controller library (Scs0009PyController)
- tkinter/ttk: GUI framework
- matplotlib: Real-time telemetry charts
- PyYAML: Configuration storage
- numpy: Angle conversions and data handling
"""
import sys
import os
from pathlib import Path
import tkinter as tk
from tkinter import ttk, messagebox
from textwrap import dedent
import numpy as np
from rustypot import Scs0009PyController
import threading
import time
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import yaml


BASE_DIR = Path(__file__).resolve().parent
DATA_DIR = BASE_DIR / "data"
CONFIG_FILE = DATA_DIR / "hand_config.yaml"  # YAML with poses and sequences
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


def load_config():
    """Load configuration from YAML file.
    
    Returns:
        dict: Configuration dictionary with structure:
            {
                'poses': {
                    'pose_name': {
                        'positions': [int, int, ...] # 8 servo positions in degrees
                    },
                    ...
                },
                'sequences': {
                    'sequence_name': {
                        'steps': [str, str, ...] # Step format: "pose:s1,s2,...,s8|delay" or "SLEEP:duration"
                    },
                    ...
                }
            }
    
    Notes:
        - Returns empty structure if file doesn't exist
        - Gracefully handles malformed YAML
        - Ensures both 'poses' and 'sequences' keys are present
    """
    if not CONFIG_FILE.exists():
        return {'poses': {}, 'sequences': {}}
    
    try:
        with CONFIG_FILE.open('r') as f:
            config = yaml.safe_load(f) or {}
            # Ensure both keys exist
            if 'poses' not in config:
                config['poses'] = {}
            if 'sequences' not in config:
                config['sequences'] = {}
            return config
    except Exception as e:
        print(f"Error loading config: {e}")
        return {'poses': {}, 'sequences': {}}


def save_config(config):
    """Save configuration to YAML file with inline array formatting.
    
    Args:
        config (dict): Configuration dictionary (see load_config for structure)
    
    Returns:
        bool: True if save successful, False on error
    
    Implementation Details:
        - Uses PyYAML to generate base YAML
        - Post-processes with regex to format positions as inline arrays
        - Example output: positions: [0, 0, 0, 0, 0, 0, 0, 0]
        - Creates data directory if it doesn't exist
    
    Notes:
        - Inline formatting improves readability for 8-element position arrays
        - Preserves insertion order (sort_keys=False)
        - Prints traceback on error for debugging
    """
    try:
        ensure_data_dir()
        
        # Create YAML string with custom formatting
        yaml_str = yaml.dump(config, default_flow_style=False, sort_keys=False)
        
        # Replace positions lists with flow style
        import re
        def replace_positions(match):
            # Extract the positions values from the multi-line list
            lines = match.group(0)
            # Find all the position values
            values = re.findall(r'- (-?\d+)', lines)
            return f"    positions: [{', '.join(values)}]\n"
        
        # Pattern to match positions: followed by list items on separate lines
        yaml_str = re.sub(r'    positions:\n(?:    - -?\d+\n)+', replace_positions, yaml_str)
        
        with CONFIG_FILE.open('w') as f:
            f.write(yaml_str)
        return True
    except Exception as e:
        print(f"Error saving config: {e}")
        import traceback
        traceback.print_exc()
        return False


def load_pose_definitions():
    """Load pose definitions from YAML config.
    
    Returns:
        list: List of pose dictionaries with structure:
            [{
                'name': str,        # Pose name
                'positions': [int]  # 8 servo positions
            }, ...]
    
    Notes:
        - Converts config dict format to list for combo box population
        - Defaults to [0]*8 if positions missing (shouldn't happen)
    """
    config = load_config()
    poses = []
    for name, data in config.get('poses', {}).items():
        poses.append({
            'name': name,
            'positions': data.get('positions', [0]*8)
        })
    return poses


def validate_name(name):
    """Validate a pose/sequence name to prevent YAML corruption.
    
    Args:
        name (str): Name to validate
    
    Returns:
        tuple: (is_valid: bool, error_message: str)
            - (True, "") if valid
            - (False, "reason") if invalid
    
    Validation Rules:
        - Cannot be empty or whitespace-only
        - Maximum 50 characters
        - No YAML special characters: : { } [ ] , & * # ? | - < > = ! % @ ` " '
        - No control characters (ASCII < 32)
        - No leading/trailing spaces
    
    Why This Matters:
        - YAML uses : for key-value pairs
        - Brackets/braces for collections
        - Commas for inline arrays
        - Other chars can break parsing or cause ambiguity
    
    Example:
        valid, msg = validate_name("my_pose")
        if not valid:
            print(f"Invalid name: {msg}")
    """
    if not name or not name.strip():
        return False, "Name cannot be empty"
    
    name = name.strip()
    
    # Check length
    if len(name) > 50:
        return False, "Name too long (max 50 characters)"
    
    # YAML special characters that could cause issues
    forbidden_chars = [':', '{', '}', '[', ']', ',', '&', '*', '#', '?', '|', '-', '<', '>', '=', '!', '%', '@', '`', '"', "'"]
    
    for char in forbidden_chars:
        if char in name:
            return False, f"Name contains forbidden character: {char}"
    
    # Check for leading/trailing spaces (already stripped, but check for internal issues)
    if name != name.strip():
        return False, "Name has leading/trailing spaces"
    
    # Check for newlines or other control characters
    if any(ord(c) < 32 for c in name):
        return False, "Name contains control characters"
    
    return True, ""


class Tooltip:
    """Simple tooltip displayed on widget hover.
    
    Creates a small yellow tooltip window that appears after hovering
    over a widget for a specified delay period.
    
    Args:
        widget: Tkinter widget to attach tooltip to
        text (str): Tooltip message to display
        delay (int): Milliseconds to wait before showing tooltip (default: 500)
    
    Implementation:
        - Binds to <Enter>, <Leave>, <ButtonPress> events
        - Uses after() for delay scheduling
        - Creates toplevel window with no decorations
        - Positions below and to the right of widget
    
    Usage:
        tooltip = Tooltip(my_button, "Click to save")
        # Or use helper:
        attach_tooltip(my_button, "Click to save")
    """

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
        try:
            bbox = self.widget.bbox('insert') if self.widget.winfo_viewable() else None
            if bbox:
                x, y, _cx, cy = bbox
            else:
                x, y, cy = 0, 0, 0
        except:
            x, y, cy = 0, 0, 0
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
    """Control widget for a single finger (2 servos).
    
    Each finger consists of two servos:
    - servo1 (odd ID): Controls close/open position (0-110°)
    - servo2 (even ID): Controls side-to-side offset (-20 to +20°)
    
    Features:
    - Vertical slider for close/open (inverted: top=closed, bottom=open)
    - Horizontal slider for left/right movement
    - Speed control (1-6) shared by both servos
    - Mimic mode: When enabled, mirrors close/open changes to other mimicking fingers
    - Mouse wheel support on sliders for fine control
    - Center button to reset side offset to 0
    
    Args:
        parent: Tkinter parent widget
        finger_name (str): Display name (e.g., "Pointer", "Thumb")
        servo1_id (int): Odd servo ID for position (1, 3, 5, 7)
        servo2_id (int): Even servo ID for side (2, 4, 6, 8)
        controller: Scs0009PyController instance
        update_callback: Function to call when positions change
    
    Attributes:
        pos_var (IntVar): Close/open position (0-110)
        side_var (IntVar): Side offset (-20 to +20)
        speed_var (IntVar): Servo speed (1-6)
        mimic_var (BooleanVar): Mimic mode enabled
    
    Note: Even servo IDs have inverted angles in hardware.
    """
    
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
            self.frame, text="⊙ Center", 
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
    """Main GUI application for AmazingHand control.
    
    The main application window provides:
    - 4 finger controls (Pointer, Middle, Ring, Thumb) in left panel
    - Real-time servo monitoring chart in right panel
    - Connection management with auto-detection
    - Global controls (Open All, Close All, Center All)
    - Pose management (save/load/apply position sets)
    - Sequence player (execute multi-step animations)
    - Keyboard shortcuts for finger control
    
    Layout Structure:
        Left Panel:
            Row 0-1: Finger controls (4 fingers in 2x2 grid)
            Row 2: Control panels stacked vertically:
                - Connection Management
                - Global Controls
                - Pose Management
                - Sequence Player
            Row 5: Status bar
            Row 6: Execution log (expandable)
        
        Right Panel:
            - Servo monitoring chart with metric selection
    
    Threading:
        - Main thread: GUI event loop
        - Monitor thread: Background telemetry collection (monitor_servos)
        - Sequence thread: Executes sequences without blocking UI
    
    Data Flow:
        1. User adjusts slider → on_finger_update()
        2. on_finger_update() → controller.sync_write_goal_position()
        3. monitor_servos() → reads actual positions, updates chart
        4. update_chart() → redraws matplotlib figure
    
    Args:
        port (str, optional): Serial port path. Auto-detects if None.
        baudrate (int): Serial baudrate (default: 1000000)
    
    Attributes:
        controller (Scs0009PyController): Servo controller instance
        connected (bool): Connection state
        fingers (list): FingerControl instances [pointer, middle, ring, thumb]
        servo_data (dict): Telemetry time series for charting
        sequence_running (bool): True when sequence executing
    """
    
    def __init__(self, port=None, baudrate=1000000):
        if port is None:
            port = default_serial_port()
        self.root = tk.Tk()
        self.root.title("AmazingHand Controller")
        self.root.geometry("1400x800")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Store connection parameters
        self.initial_port = port
        self.initial_baudrate = baudrate
        self.controller = None
        self.connected = False
        
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
        self.pause_sequence = False
        self.sequence_running = False
        self.sequence_thread = None
        
        # Keyboard control
        self.selected_finger_idx = 0  # Default to first finger
        
        # Note: Tk bitmap images are unreliable across platforms here, so
        # we stick to text-only buttons (with Unicode where appropriate).

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
                parent = left_frame
            else:  # Second row: finger 4 (thumb)
                row = 2
                col = 0
                parent = left_frame
            
            finger = FingerControl(
                parent, name, s1, s2, self.controller, self.on_finger_update
            )
            finger.frame.grid(row=row, column=col, columnspan=2, padx=5, pady=5, sticky='n')
            self.fingers.append(finger)

        # Right-side stacked controls next to thumb
        right_controls_frame = ttk.Frame(left_frame)
        right_controls_frame.grid(row=2, column=2, columnspan=4, padx=(10, 0), pady=5, sticky='n')
        
        # Connection options - top of right stack
        conn_frame = ttk.LabelFrame(right_controls_frame, text="Connection", padding=5)
        conn_frame.pack(fill='x', pady=(0, 5))
        
        conn_row = ttk.Frame(conn_frame)
        conn_row.pack(fill='x')
        
        ttk.Label(conn_row, text="Port:").pack(side='left', padx=(0,2))
        
        # Detect available ports
        import glob
        import os
        available_ports = []
        if os.name == 'nt':  # Windows
            available_ports = [f'COM{i}' for i in range(1, 21)]
        else:  # Linux/Mac
            available_ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyAMA*')
            if not available_ports:
                available_ports = ['/dev/ttyACM0', '/dev/ttyUSB0']
        
        self.port_var = tk.StringVar(value=self.initial_port)
        self.port_combo = ttk.Combobox(
            conn_row, textvariable=self.port_var, values=available_ports, width=12, state='readonly'
        )
        self.port_combo.pack(side='left', padx=2)
        attach_tooltip(self.port_combo, "Select serial port for hand controller.")
        
        ttk.Label(conn_row, text="Baud:").pack(side='left', padx=(5,2))
        self.baudrate_var = tk.StringVar(value=str(self.initial_baudrate))
        baudrate_combo = ttk.Combobox(
            conn_row, textvariable=self.baudrate_var, 
            values=['9600', '115200', '1000000'], width=8, state='readonly'
        )
        baudrate_combo.pack(side='left', padx=2)
        attach_tooltip(baudrate_combo, "Serial communication speed.")
        
        self.connect_btn = ttk.Button(
            conn_row, text="▶ Connect", command=self.connect_controller, width=10
        )
        self.connect_btn.pack(side='left', padx=2)
        attach_tooltip(self.connect_btn, "Connect to the hand controller.")
        
        self.disconnect_btn = ttk.Button(
            conn_row, text="⏹ Disconnect", command=self.disconnect_controller, width=10
        )
        self.disconnect_btn.pack(side='left', padx=2)
        self.disconnect_btn.state(['disabled'])
        attach_tooltip(self.disconnect_btn, "Disconnect from the controller.")
        
        # Global controls - second in right stack
        control_frame = ttk.LabelFrame(right_controls_frame, text="Global Controls", padding=10)
        control_frame.pack(fill='x', pady=5)
        
        # Preset buttons
        self.open_all_btn = ttk.Button(
            control_frame, text="✋ Open All", command=self.open_all
        )
        self.open_all_btn.pack(side='left', padx=5)
        attach_tooltip(self.open_all_btn, "Set every finger to fully open (0°).")
        
        self.close_all_btn = ttk.Button(
            control_frame, text="✊ Close All", command=self.close_all
        )
        self.close_all_btn.pack(side='left', padx=5)
        attach_tooltip(self.close_all_btn, "Set every finger to fully closed (110°).")
        
        self.center_all_btn = ttk.Button(
            control_frame, text="⊙ Center All", command=self.center_all
        )
        self.center_all_btn.pack(side='left', padx=5)
        attach_tooltip(self.center_all_btn, "Reset all side-to-side offsets to 0°.")
        
        # Pose management section - middle of right stack
        pose_mgmt_frame = ttk.LabelFrame(right_controls_frame, text="Pose Management", padding=10)
        pose_mgmt_frame.pack(fill='x', pady=5)
        
        # First row: existing poses
        pose_row1 = ttk.Frame(pose_mgmt_frame)
        pose_row1.pack(fill='x', pady=2)
        
        ttk.Label(pose_row1, text="Pose:").pack(side='left', padx=(0,2))
        poses_list = load_pose_definitions()
        default_pose = poses_list[0]['name'] if poses_list else ""
        self.pose_var = tk.StringVar(value=default_pose)
        self.pose_combo = ttk.Combobox(
            pose_row1,
            textvariable=self.pose_var,
            state='readonly',
            width=14,
            values=[s['name'] for s in poses_list] or ["<no poses>"]
        )
        self.pose_combo.pack(side='left', padx=2)
        attach_tooltip(self.pose_combo, "Select a saved pose.")

        self.set_pose_btn = ttk.Button(
            pose_row1, text="✓ Apply", command=self.set_selected_pose, width=10
        )
        self.set_pose_btn.pack(side='left', padx=5)
        attach_tooltip(self.set_pose_btn, "Apply the selected pose to all fingers.")
        
        # Second row: add new pose
        pose_row2 = ttk.Frame(pose_mgmt_frame)
        pose_row2.pack(fill='x', pady=2)
        
        ttk.Label(pose_row2, text="Name:").pack(side='left', padx=(0,2))
        self.save_pose_name_var = tk.StringVar()
        self.save_pose_entry = ttk.Entry(pose_row2, textvariable=self.save_pose_name_var, width=14)
        self.save_pose_entry.pack(side='left', padx=2)
        attach_tooltip(self.save_pose_entry, "Enter a name for the pose. Avoid special chars: : { } [ ] , & * # ? | - < > = ! % @ ` \" '")
        
        self.save_pose_btn = ttk.Button(
            pose_row2, text="➕ Add New", command=self.save_pose, width=10
        )
        self.save_pose_btn.pack(side='left', padx=5)
        attach_tooltip(self.save_pose_btn, "Save current finger positions as a new pose.")
       
        # Sequence player section - bottom of right stack
        seq_player_frame = ttk.LabelFrame(right_controls_frame, text="Sequence Player", padding=10)
        seq_player_frame.pack(fill='x', pady=5)
        
        # First row: sequence selection
        seq_row1 = ttk.Frame(seq_player_frame)
        seq_row1.pack(fill='x', pady=2)
        
        ttk.Label(seq_row1, text="Sequence:").pack(side='left', padx=(0,5))
        
        self.sequence_var = tk.StringVar()
        self.sequences_combo = ttk.Combobox(
            seq_row1,
            textvariable=self.sequence_var,
            state='readonly',
            width=14
        )
        self.sequences_combo.pack(side='left', padx=2)
        attach_tooltip(self.sequences_combo, "Select a sequence to play.")
        
        self.manage_sequences_btn = ttk.Button(
            seq_row1, text="🔧 Manage", command=self.manage_sequences, width=10
        )
        self.manage_sequences_btn.pack(side='left', padx=5)
        attach_tooltip(self.manage_sequences_btn, "Open sequence builder and editor.")
        
        self.loop_sequence_var = tk.BooleanVar(value=False)
        self.loop_check = ttk.Checkbutton(seq_row1, text="Loop", variable=self.loop_sequence_var)
        self.loop_check.pack(side='left', padx=5)
        attach_tooltip(self.loop_check, "When enabled, selected sequence repeats continuously until stopped.")
        
        # Second row: playback controls
        seq_row2 = ttk.Frame(seq_player_frame)
        seq_row2.pack(fill='x', pady=2)
        
        self.play_btn = ttk.Button(
            seq_row2, text="▶ Play", command=self.play_selected_sequence, width=10
        )
        self.play_btn.pack(side='left', padx=2)
        attach_tooltip(self.play_btn, "Start playing the selected sequence.")
        
        self.pause_btn = ttk.Button(
            seq_row2, text="⏸ Pause", command=self.pause_sequence_exec, width=10
        )
        self.pause_btn.pack(side='left', padx=2)
        self.pause_btn.state(['disabled'])
        attach_tooltip(self.pause_btn, "Pause/resume the running sequence.")
        
        self.stop_btn = ttk.Button(
            seq_row2, text="⏹ Stop", command=self.stop_sequence_exec, width=10
        )
        self.stop_btn.pack(side='left', padx=2)
        self.stop_btn.state(['disabled'])
        attach_tooltip(self.stop_btn, "Stop the running sequence.")
        
        # Refresh sequences list
        self.refresh_sequences_list()
        
        # Bind keyboard events
        self.root.bind('<Key>', self.on_key_press)
        
        # Status bar - below all controls
        self.status_var = tk.StringVar(value="Ready")
        status_bar = ttk.Label(
            left_frame, textvariable=self.status_var,
            relief='sunken', anchor='w'
        )
        status_bar.grid(row=5, column=0, columnspan=6, sticky='ew', pady=(10,0))
        attach_tooltip(status_bar, "Most recent action, warning, or error message.")
        
        # Log output - bottom, expandable
        log_frame = ttk.LabelFrame(left_frame, text="Execution Log", padding=5)
        log_frame.grid(row=6, column=0, columnspan=6, sticky='nsew', pady=(10,0))
        
        # Make bottom row expandable
        left_frame.grid_rowconfigure(6, weight=1)
        
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
        
        # Auto-connect on startup
        self.root.after(100, self.connect_controller)
        
        # Auto-connect on startup
        self.root.after(100, self.connect_controller)
    
    def setup_chart_panel(self, parent):
        """Setup the chart display panel."""
        # Title
        chart_title = ttk.Label(
            parent, text="Servo Monitoring",
            font=('Arial', 14, 'bold')
        )
        chart_title.pack(pady=(5,0))
        attach_tooltip(chart_title, "Live telemetry from all servos (position, load, temperature, voltage).")
        
        # Metric selection
        select_frame = ttk.Frame(parent)
        select_frame.pack(fill='x', padx=5, pady=(5,2))
        
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
        servo_frame.pack(fill='x', padx=5, pady=2)
        
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
        all_btn = ttk.Button(servo_frame, text="✓ All", command=self.select_all_servos, width=6)
        all_btn.pack(side='left', padx=5)
        attach_tooltip(all_btn, "Enable all servo traces.")
        none_btn = ttk.Button(servo_frame, text="✕ None", command=self.deselect_all_servos, width=6)
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
        clear_btn = ttk.Button(servo_frame, text="⌫ Clear", command=self.clear_chart_data, width=6)
        clear_btn.pack(side='left', padx=2)
        attach_tooltip(clear_btn, "Reset collected telemetry and start fresh.")
        
        # Create matplotlib figure
        self.fig = Figure(figsize=(10, 6), dpi=100)
        self.fig.tight_layout(pad=0.5)
        self.ax = self.fig.add_subplot(111)
        
        self.canvas = FigureCanvasTkAgg(self.fig, parent)
        canvas_widget = self.canvas.get_tk_widget()
        canvas_widget.pack(fill='both', expand=True, padx=2, pady=2)
        attach_tooltip(canvas_widget, "Scroll to zoom, drag to pan; updates roughly every second.")
        
        # Initialize plot
        self.update_chart()
    
    def monitor_servos(self):
        """Background thread to monitor servo states."""
        while self.monitoring:
            try:
                if not self.connected or self.controller is None:
                    time.sleep(0.1)
                    continue
                    
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
    
    def connect_controller(self):
        """Connect to the hand controller."""
        if self.connected:
            self.status_var.set("Already connected")
            return
        
        port = self.port_var.get()
        try:
            baudrate = int(self.baudrate_var.get())
        except ValueError:
            self.status_var.set("Invalid baudrate")
            return
        
        try:
            self.log(f"Connecting to {port} at {baudrate} baud...")
            self.status_var.set(f"Connecting to {port}...")
            
            self.controller = Scs0009PyController(
                serial_port=port,
                baudrate=baudrate,
                timeout=0.5
            )
            
            # Enable torque for all servos
            for servo_id in range(1, 9):
                self.controller.write_torque_enable(servo_id, 1)
            
            self.connected = True
            self.connect_btn.state(['disabled'])
            self.disconnect_btn.state(['!disabled'])
            self.port_combo.config(state='disabled')
            
            self.log("Connected successfully!")
            self.status_var.set(f"Connected to {port}")
            
        except Exception as e:
            self.log(f"Connection failed: {e}")
            self.status_var.set(f"Connection failed: {e}")
            self.controller = None
            self.connected = False
    
    def disconnect_controller(self):
        """Disconnect from the hand controller."""
        if not self.connected:
            return
        
        try:
            if self.controller:
                # Disable torque for all servos
                for servo_id in range(1, 9):
                    try:
                        self.controller.write_torque_enable(servo_id, 0)
                    except:
                        pass
                self.controller = None
            
            self.connected = False
            self.connect_btn.state(['!disabled'])
            self.disconnect_btn.state(['disabled'])
            self.port_combo.config(state='readonly')
            
            self.log("Disconnected")
            self.status_var.set("Disconnected")
            
        except Exception as e:
            self.log(f"Disconnect error: {e}")
            self.status_var.set(f"Disconnect error: {e}")
    
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
        if not self.connected or self.controller is None:
            return
            
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
    
    def refresh_sequences_list(self):
        """Refresh the sequences dropdown with saved sequences."""
        config = load_config()
        sequences = config.get('sequences', {})
        seq_names = sorted(sequences.keys())
        self.sequences_combo['values'] = seq_names if seq_names else ['<no sequences>']
        if seq_names:
            self.sequence_var.set(seq_names[0])
        else:
            self.sequence_var.set('<no sequences>')
    
    def play_selected_sequence(self):
        """Play the selected sequence from the dropdown."""
        seq_name = self.sequence_var.get().strip()
        if not seq_name or seq_name == '<no sequences>':
            self.status_var.set("Select a sequence to play")
            return
        
        if self.sequence_running:
            self.status_var.set("A sequence is already running")
            return
        config = load_config()
        seq_data = config.get('sequences', {}).get(seq_name, {})
        
        if not seq_data:
            self.status_var.set(f"Sequence '{seq_name}' not found")
            return
        
        items = seq_data.get('steps', [])
        loop_enabled = self.loop_sequence_var.get()  # Use main window checkbox
        poses = config.get('poses', {})
        
        self._execute_sequence_items(items, loop_enabled, poses)
    
    def pause_sequence_exec(self):
        """Pause or resume the currently running sequence."""
        if not self.sequence_running:
            return
        
        self.pause_sequence = not self.pause_sequence
        
        if self.pause_sequence:
            self.pause_btn.config(text="▶ Resume")
            self.log("Sequence paused")
            self.status_var.set("Sequence paused")
        else:
            self.pause_btn.config(text="⏸ Pause")
            self.log("Sequence resumed")
            self.status_var.set("Sequence resumed")
    
    def stop_sequence_exec(self):
        """Stop the currently running sequence."""
        if not self.sequence_running:
            return
        self.stop_sequence = True
        self.pause_sequence = False
        self.log("Stop requested")
        self.status_var.set("Stopping sequence...")
    
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
    
    def save_pose(self):
        """Save current position as a pose."""
        name = self.save_pose_name_var.get().strip()
        if not name:
            self.status_var.set("Please enter a pose name")
            self.save_pose_entry.focus()
            return
        
        # Validate name
        is_valid, error_msg = validate_name(name)
        if not is_valid:
            self.status_var.set(f"Invalid name: {error_msg}")
            messagebox.showerror("Invalid Name", error_msg, parent=self.root)
            return
        
        try:
            # Get current positions only
            positions = []
            for finger in self.fingers:
                pos1, pos2 = finger.get_positions()
                positions.extend([pos1, pos2])
            
            # Load config, add pose, save
            config = load_config()
            config['poses'][name] = {
                'positions': positions
            }
            
            if save_config(config):
                # Update dropdown
                current_values = list(self.pose_combo['values'])
                if '<no poses>' in current_values:
                    current_values = []
                if name not in current_values:
                    current_values.append(name)
                self.pose_combo['values'] = sorted(current_values)
                self.pose_var.set(name)
                
                # Clear entry for next use
                self.save_pose_name_var.set('')
                
                self.status_var.set(f"Saved pose '{name}'")
                self.log(f"Saved pose: {name}")
            else:
                self.status_var.set("Error saving pose")
        
        except Exception as e:
            self.status_var.set(f"Error saving: {e}")

    def set_selected_pose(self):
        """Apply the currently selected pose from the dropdown to the hand."""
        try:
            # Load config
            config = load_config()
            poses = config.get('poses', {})
            
            if not poses:
                self.status_var.set("No poses found")
                return
            
            # Get selected pose
            selected_name = self.pose_var.get().strip() if hasattr(self, 'pose_var') else ''
            if not selected_name or selected_name not in poses:
                selected_name = list(poses.keys())[0] if poses else None
            
            if not selected_name:
                self.status_var.set("No pose selected")
                return
            
            pose_data = poses[selected_name]
            positions = pose_data.get('positions', [0]*8)

            # Apply positions to fingers (keep current speeds)
            for idx, finger in enumerate(self.fingers):
                pos1 = positions[idx * 2]
                pos2 = positions[idx * 2 + 1]
                finger.set_positions(pos1, pos2)

            # Trigger position update
            self.update_pending = True
            self.send_positions()

            self.status_var.set(f"Set pose '{selected_name}'")
        
        except Exception as e:
            self.status_var.set(f"Error setting pose: {e}")
    
    def manage_sequences(self):
        """Open sequence management dialog with list of saved sequences."""
        try:
            # Load config
            config = load_config()
            poses = config.get('poses', {})
            sequences = config.get('sequences', {})
            
            if not poses:
                self.status_var.set("No poses found - create poses first")
                return
            
            # Create management dialog
            dialog = tk.Toplevel(self.root)
            dialog.title("Sequence Management")
            dialog.geometry("900x700")
            
            main_container = ttk.Frame(dialog, padding=10)
            main_container.pack(fill='both', expand=True)
            
            # Two-panel layout
            paned = ttk.PanedWindow(main_container, orient='horizontal')
            paned.pack(fill='both', expand=True)
            
            # Left panel - Saved sequences list
            left_panel = ttk.Frame(paned)
            paned.add(left_panel, weight=1)
            
            ttk.Label(left_panel, text="Saved Sequences", font=('Arial', 12, 'bold')).pack(pady=(0,10))
            
            # Sequences listbox with scrollbar
            seq_frame = ttk.Frame(left_panel)
            seq_frame.pack(fill='both', expand=True)
            
            seq_scroll = ttk.Scrollbar(seq_frame)
            seq_scroll.pack(side='right', fill='y')
            
            sequences_listbox = tk.Listbox(seq_frame, yscrollcommand=seq_scroll.set)
            sequences_listbox.pack(side='left', fill='both', expand=True)
            seq_scroll.config(command=sequences_listbox.yview)
            attach_tooltip(sequences_listbox, "List of all saved sequences. Double-click to execute.")
            
            def refresh_sequences_list():
                sequences_listbox.delete(0, tk.END)
                config = load_config()
                for seq_name in sorted(config.get('sequences', {}).keys()):
                    sequences_listbox.insert(tk.END, seq_name)
            
            refresh_sequences_list()
            
            # Sequence action buttons
            seq_btn_frame = ttk.Frame(left_panel)
            seq_btn_frame.pack(fill='x', pady=(10,0))
            
            def execute_selected_sequence():
                selection = sequences_listbox.curselection()
                if not selection:
                    self.status_var.set("Select a sequence to execute")
                    return
                
                seq_name = sequences_listbox.get(selection[0])
                config = load_config()
                seq_data = config.get('sequences', {}).get(seq_name, {})
                
                if not seq_data:
                    self.status_var.set(f"Sequence '{seq_name}' not found")
                    return
                
                items = seq_data.get('steps', [])
                loop_enabled = False  # Always non-looping in dialog quick-execute
                
                dialog.destroy()
                self._execute_sequence_items(items, loop_enabled, poses)
            
            def delete_selected_sequence():
                selection = sequences_listbox.curselection()
                if not selection:
                    return
                
                seq_name = sequences_listbox.get(selection[0])
                if tk.messagebox.askyesno("Delete Sequence", f"Delete sequence '{seq_name}'?", parent=dialog):
                    config = load_config()
                    if seq_name in config.get('sequences', {}):
                        del config['sequences'][seq_name]
                        save_config(config)
                        refresh_sequences_list()
                        self.refresh_sequences_list()  # Update main window
                        self.status_var.set(f"Deleted sequence '{seq_name}'")
                        self.log(f"Deleted sequence: {seq_name}")
            
            def edit_selected_sequence():
                selection = sequences_listbox.curselection()
                if not selection:
                    return
                
                seq_name = sequences_listbox.get(selection[0])
                config = load_config()
                seq_data = config.get('sequences', {}).get(seq_name, {})
                
                if seq_data:
                    # Load sequence steps into builder
                    steps = seq_data.get('steps', [])
                    builder_listbox.delete(0, tk.END)
                    for step in steps:
                        # Convert old format (without speeds) to new format if needed
                        if ':' not in step and '|' in step:
                            # Old format: pose_name|delay
                            parts = step.split('|')
                            pose_name = parts[0]
                            # Use default speeds 3,3,3,3,3,3,3,3
                            speeds_str = '3,3,3,3,3,3,3,3'
                            if len(parts) > 1:
                                builder_listbox.insert(tk.END, f"{pose_name}:{speeds_str}|{parts[1]}")
                            else:
                                builder_listbox.insert(tk.END, f"{pose_name}:{speeds_str}")
                        elif ':' not in step and not step.startswith('SLEEP:'):
                            # Old format: just pose_name
                            speeds_str = '3,3,3,3,3,3,3,3'
                            builder_listbox.insert(tk.END, f"{step}:{speeds_str}")
                        else:
                            # New format or SLEEP command
                            builder_listbox.insert(tk.END, step)
                    save_name_var.set(seq_name)
                    current_seq_label.config(text=f"(Editing: {seq_name})")
                    self.status_var.set(f"Loaded '{seq_name}' for editing")
            
            exec_btn = ttk.Button(seq_btn_frame, text="▶ Execute", command=execute_selected_sequence, width=12)
            exec_btn.pack(side='left', padx=2)
            attach_tooltip(exec_btn, "Run the selected sequence immediately.")
            
            edit_btn = ttk.Button(seq_btn_frame, text="✎ Edit", command=edit_selected_sequence, width=12)
            edit_btn.pack(side='left', padx=2)
            attach_tooltip(edit_btn, "Load the selected sequence into the builder for editing.")
            
            delete_btn = ttk.Button(seq_btn_frame, text="🗑 Delete", command=delete_selected_sequence, width=12)
            delete_btn.pack(side='left', padx=2)
            attach_tooltip(delete_btn, "Permanently delete the selected sequence.")
            
            # Double-click to execute
            sequences_listbox.bind('<Double-Button-1>', lambda e: execute_selected_sequence())
            
            # Right panel - Sequence builder
            right_panel = ttk.Frame(paned)
            paned.add(right_panel, weight=2)
            
            # Builder title with current sequence name
            builder_title_frame = ttk.Frame(right_panel)
            builder_title_frame.pack(fill='x', pady=(0,10))
            
            ttk.Label(builder_title_frame, text="Sequence Builder", font=('Arial', 12, 'bold')).pack(side='left')
            
            current_seq_label = ttk.Label(builder_title_frame, text="", font=('Arial', 10, 'italic'), foreground='#666')
            current_seq_label.pack(side='left', padx=10)
            
            # Builder layout
            builder_container = ttk.Frame(right_panel)
            builder_container.pack(fill='both', expand=True)
            
            # Available poses and current sequence
            cols = ttk.Frame(builder_container)
            cols.pack(fill='both', expand=True)
            
            # Available poses
            poses_frame = ttk.LabelFrame(cols, text="Available Poses", padding=10)
            poses_frame.pack(side='left', fill='both', expand=True, padx=(0,5))
            
            poses_listbox = tk.Listbox(poses_frame, height=15)
            poses_listbox.pack(fill='both', expand=True)
            attach_tooltip(poses_listbox, "Available poses. Double-click to add to sequence.")
            for pose_name in sorted(poses.keys()):
                poses_listbox.insert(tk.END, pose_name)
            
            # Sequence being built
            builder_frame = ttk.LabelFrame(cols, text="Sequence Steps", padding=10)
            builder_frame.pack(side='left', fill='both', expand=True, padx=(5,0))
            
            builder_listbox = tk.Listbox(builder_frame, height=15)
            builder_listbox.pack(fill='both', expand=True)
            attach_tooltip(builder_listbox, "Sequence steps in order. Select a step to remove or reorder.")
            
            # Control buttons
            control_frame = ttk.Frame(builder_container)
            control_frame.pack(fill='x', pady=(10,0))
            
            ttk.Label(control_frame, text="Delay (sec):").pack(side='left', padx=(0,5))
            delay_var = tk.StringVar(value="1.0")
            delay_entry = ttk.Entry(control_frame, textvariable=delay_var, width=8)
            delay_entry.pack(side='left', padx=5)
            attach_tooltip(delay_entry, "Pause duration in seconds between poses or as standalone delay.")
            
            # Speed settings frame
            speed_frame = ttk.LabelFrame(builder_container, text="Individual Finger Speeds", padding=3)
            speed_frame.pack(fill='x', pady=(10,0))
            
            finger_labels = ['Pointer', 'Middle', 'Ring', 'Thumb']
            speed_vars = []
            
            for idx, label in enumerate(finger_labels):
                row_frame = ttk.Frame(speed_frame)
                row_frame.pack(side='left', padx=5)
                
                ttk.Label(row_frame, text=f"{label}:").pack(side='left', padx=(0,2))
                speed_var = tk.IntVar(value=3)
                speed_vars.append(speed_var)
                ttk.Spinbox(row_frame, from_=1, to=6, textvariable=speed_var, width=4).pack(side='left')
            
            # Button to copy current UI speeds
            def copy_ui_speeds():
                for idx, finger in enumerate(self.fingers):
                    speed_vars[idx].set(finger.get_speed())
                self.status_var.set("Copied speeds from UI")
            
            copy_speeds_btn = ttk.Button(speed_frame, text="⬇ Copy from UI", command=copy_ui_speeds)
            copy_speeds_btn.pack(side='left', padx=5)
            attach_tooltip(copy_speeds_btn, "Import current speed settings from main window finger controls.")
            
            def add_to_builder():
                selection = poses_listbox.curselection()
                if selection:
                    pose_name = poses_listbox.get(selection[0])
                    # Use speeds from the speed settings
                    speeds = []
                    for speed_var in speed_vars:
                        speed = speed_var.get()
                        speeds.extend([speed, speed])  # Same speed for both servos in finger
                    speeds_str = ','.join(map(str, speeds))
                    
                    delay = delay_var.get()
                    if delay and float(delay) > 0:
                        builder_listbox.insert(tk.END, f"{pose_name}:{speeds_str}|{delay}s")
                    else:
                        builder_listbox.insert(tk.END, f"{pose_name}:{speeds_str}")
            
            poses_listbox.bind('<Double-Button-1>', lambda e: add_to_builder())
            
            def add_delay():
                delay = delay_var.get()
                if delay and float(delay) > 0:
                    builder_listbox.insert(tk.END, f"SLEEP:{delay}s")
            
            def remove_step():
                selection = builder_listbox.curselection()
                if selection:
                    builder_listbox.delete(selection[0])
            
            def clear_builder():
                builder_listbox.delete(0, tk.END)
                save_name_var.set('')
                current_seq_label.config(text='')
            
            def move_up():
                selection = builder_listbox.curselection()
                if selection and selection[0] > 0:
                    idx = selection[0]
                    item = builder_listbox.get(idx)
                    builder_listbox.delete(idx)
                    builder_listbox.insert(idx - 1, item)
                    builder_listbox.selection_set(idx - 1)
            
            def move_down():
                selection = builder_listbox.curselection()
                if selection and selection[0] < builder_listbox.size() - 1:
                    idx = selection[0]
                    item = builder_listbox.get(idx)
                    builder_listbox.delete(idx)
                    builder_listbox.insert(idx + 1, item)
                    builder_listbox.selection_set(idx + 1)
            
            btn_frame = ttk.Frame(control_frame)
            btn_frame.pack(side='left', padx=10)
            
            add_btn = ttk.Button(btn_frame, text="➕ Add", command=add_to_builder, width=8)
            add_btn.pack(side='left', padx=2)
            attach_tooltip(add_btn, "Add selected pose with current speeds and delay to sequence.")
            
            delay_btn = ttk.Button(btn_frame, text="⏱ Delay", command=add_delay, width=8)
            delay_btn.pack(side='left', padx=2)
            attach_tooltip(delay_btn, "Insert a standalone pause (SLEEP) into the sequence.")
            
            remove_btn = ttk.Button(btn_frame, text="➖ Remove", command=remove_step, width=8)
            remove_btn.pack(side='left', padx=2)
            attach_tooltip(remove_btn, "Delete the selected step from the sequence.")
            
            clear_btn = ttk.Button(btn_frame, text="🗑 Clear", command=clear_builder, width=8)
            clear_btn.pack(side='left', padx=2)
            attach_tooltip(clear_btn, "Remove all steps and reset the builder.")
            
            up_btn = ttk.Button(btn_frame, text="↑", command=move_up, width=3)
            up_btn.pack(side='left', padx=2)
            attach_tooltip(up_btn, "Move selected step up in sequence order.")
            
            down_btn = ttk.Button(btn_frame, text="↓", command=move_down, width=3)
            down_btn.pack(side='left', padx=2)
            attach_tooltip(down_btn, "Move selected step down in sequence order.")
            
            # Save sequence section
            save_frame = ttk.Frame(builder_container)
            save_frame.pack(fill='x', pady=(15,0))
            
            ttk.Label(save_frame, text="Sequence Name:").pack(side='left', padx=(0,5))
            save_name_var = tk.StringVar()
            
            def on_name_change(*args):
                name = save_name_var.get().strip()
                if name:
                    current_seq_label.config(text=f"(Current: {name})")
                else:
                    current_seq_label.config(text="")
            
            save_name_var.trace('w', on_name_change)
            
            name_entry = ttk.Entry(save_frame, textvariable=save_name_var, width=20)
            name_entry.pack(side='left', padx=5)
            attach_tooltip(name_entry, "Enter a unique name. Avoid special chars: : { } [ ] , & * # ? | - < > = ! % @ ` \" '")
            
            def save_sequence():
                name = save_name_var.get().strip()
                if not name:
                    self.status_var.set("Enter a sequence name")
                    return
                
                # Validate name
                is_valid, error_msg = validate_name(name)
                if not is_valid:
                    self.status_var.set(f"Invalid name: {error_msg}")
                    tk.messagebox.showerror("Invalid Name", error_msg, parent=dialog)
                    return
                
                if builder_listbox.size() == 0:
                    self.status_var.set("Sequence is empty")
                    return
                
                steps = [builder_listbox.get(i) for i in range(builder_listbox.size())]
                
                config = load_config()
                config['sequences'][name] = {
                    'steps': steps
                }
                
                if save_config(config):
                    refresh_sequences_list()
                    self.refresh_sequences_list()  # Update main window
                    self.status_var.set(f"Saved sequence '{name}'")
                    self.log(f"Saved sequence: {name}")
                    # Don't clear the name, keep it for further edits
                    current_seq_label.config(text=f"(Saved: {name})")
                else:
                    self.status_var.set("Error saving sequence")
            
            def execute_builder():
                if builder_listbox.size() == 0:
                    return
                
                steps = [builder_listbox.get(i) for i in range(builder_listbox.size())]
                loop_enabled = False  # Test execution is non-looping
                
                dialog.destroy()
                self._execute_sequence_items(steps, loop_enabled, poses)
            
            save_btn = ttk.Button(save_frame, text="💾 Save Sequence", command=save_sequence, width=15)
            save_btn.pack(side='left', padx=5)
            attach_tooltip(save_btn, "Save the current sequence with the specified name.")
            
            exec_builder_btn = ttk.Button(save_frame, text="▶ Execute", command=execute_builder, width=12)
            exec_builder_btn.pack(side='left', padx=5)
            attach_tooltip(exec_builder_btn, "Test-run the sequence being built without saving.")
            
            # Close button
            close_btn = ttk.Button(main_container, text="Close", command=dialog.destroy, width=12)
            close_btn.pack(pady=(10,0))
            attach_tooltip(close_btn, "Close the sequence manager and return to main window.")
        
        except Exception as e:
            self.status_var.set(f"Error: {e}")
            import traceback
            traceback.print_exc()
    
    def _execute_sequence_items(self, items, loop_enabled, poses):
        """Execute a sequence of pose steps."""
        self.stop_sequence = False
        self.pause_sequence = False
        self.sequence_running = True
        self.root.after(0, lambda: self.play_btn.state(['disabled']))
        self.root.after(0, lambda: self.pause_btn.state(['!disabled']))
        self.root.after(0, lambda: self.stop_btn.state(['!disabled']))
        
        def run_sequence():
            loop_count = 0
            
            while True:
                loop_count += 1
                if loop_enabled:
                    self.root.after(0, lambda c=loop_count: self.log(f"=== Loop iteration {c} ==="))
                else:
                    self.root.after(0, lambda: self.log("=== Starting sequence execution ==="))
                
                for item in items:
                    # Handle pause
                    while self.pause_sequence and not self.stop_sequence:
                        time.sleep(0.1)
                    
                    if self.stop_sequence:
                        self.root.after(0, lambda: self.log("=== Sequence stopped ==="))
                        self.root.after(0, lambda: self.status_var.set("Sequence stopped"))
                        self.sequence_running = False
                        self.root.after(0, lambda: self.play_btn.state(['!disabled']))
                        self.root.after(0, lambda: self.pause_btn.state(['disabled']))
                        self.root.after(0, lambda: self.pause_btn.config(text="⏸ Pause"))
                        self.root.after(0, lambda: self.stop_btn.state(['disabled']))
                        return
                    
                    if item.startswith("SLEEP:"):
                        delay = float(item.split(':')[1].rstrip('s'))
                        self.root.after(0, lambda d=delay: self.status_var.set(f"Waiting {d}s..."))
                        self.root.after(0, lambda d=delay: self.log(f"Delay: {d}s"))
                        # Sleep in small increments to allow pause detection
                        elapsed = 0
                        while elapsed < delay:
                            while self.pause_sequence and not self.stop_sequence:
                                time.sleep(0.1)
                            if self.stop_sequence:
                                break
                            time.sleep(0.1)
                            elapsed += 0.1
                    else:
                        # Parse pose with speeds: pose_name:s1,s2,...,s8|delay or pose_name:s1,s2,...,s8
                        parts = item.split('|')
                        pose_part = parts[0]
                        
                        # Split pose name and speeds
                        if ':' in pose_part:
                            pose_name, speeds_str = pose_part.split(':', 1)
                            speeds = [int(s) for s in speeds_str.split(',')]
                        else:
                            # Old format without speeds - use default
                            pose_name = pose_part
                            speeds = [3] * 8
                        
                        if pose_name not in poses:
                            print(f"Pose '{pose_name}' not found, skipping")
                            continue
                        
                        pose_data = poses[pose_name]
                        self.root.after(0, lambda pd=pose_data, n=pose_name, sp=speeds: self._apply_pose_from_config(pd, n, sp))
                        
                        if len(parts) > 1:
                            delay = float(parts[1].rstrip('s'))
                            self.root.after(0, lambda d=delay: self.log(f"  → Wait: {d}s"))
                            # Sleep in small increments to allow pause detection
                            elapsed = 0
                            while elapsed < delay:
                                while self.pause_sequence and not self.stop_sequence:
                                    time.sleep(0.1)
                                if self.stop_sequence:
                                    break
                                time.sleep(0.1)
                                elapsed += 0.1
                        else:
                            # Use average speed for auto-wait calculation
                            avg_speed = sum(speeds) / len(speeds)
                            timeout = 15.0 - (avg_speed - 1) * 2.4
                            self.root.after(0, lambda t=timeout: self.log(f"  → Auto-wait: {t:.1f}s"))
                            # Sleep in small increments to allow pause detection
                            elapsed = 0
                            while elapsed < timeout:
                                while self.pause_sequence and not self.stop_sequence:
                                    time.sleep(0.1)
                                if self.stop_sequence:
                                    break
                                time.sleep(0.1)
                                elapsed += 0.1
                
                if not loop_enabled:
                    break
                
                if loop_enabled and not self.stop_sequence:
                    time.sleep(0.5)
            
            self.sequence_running = False
            self.root.after(0, lambda: self.play_btn.state(['!disabled']))
            self.root.after(0, lambda: self.pause_btn.state(['disabled']))
            self.root.after(0, lambda: self.pause_btn.config(text="⏸ Pause"))
            self.root.after(0, lambda: self.stop_btn.state(['disabled']))
            self.root.after(0, lambda: self.status_var.set("Sequence complete"))
            self.root.after(0, lambda: self.log("=== Sequence complete ==="))
        
        self.sequence_thread = threading.Thread(target=run_sequence, daemon=True)
        self.sequence_thread.start()
    
    def _apply_pose(self, pose, name):
        """Apply a pose to the GUI (called from main thread)."""
        for idx, finger in enumerate(self.fingers):
            pos1 = pose['positions'][idx * 2]
            pos2 = pose['positions'][idx * 2 + 1]
            finger.set_positions(pos1, pos2)
            finger.speed_var.set(pose['speed'])
        
        # Trigger position update
        self.update_pending = True
        self.send_positions()
        self.status_var.set(f"Executing: {name}")
        self.log(f"Pose: {name} (speed={pose['speed']})")
    
    def _apply_pose_from_config(self, pose_data, name, speeds=None):
        """Apply a pose from YAML config format."""
        positions = pose_data.get('positions', [0]*8)
        if speeds is None:
            speeds = [3] * 8  # Default speeds if not provided
        
        for idx, finger in enumerate(self.fingers):
            pos1 = positions[idx * 2]
            pos2 = positions[idx * 2 + 1]
            finger.set_positions(pos1, pos2)
            finger.speed_var.set(speeds[idx * 2])  # Set speed from sequence
        
        self.update_pending = True
        self.send_positions()
        self.status_var.set(f"Executing: {name}")
        self.log(f"Pose: {name}")
    
    def log(self, message):
        """Add message to log output with timestamp."""
        from datetime import datetime
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]  # HH:MM:SS.mmm
        self.log_text.config(state='normal')
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
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

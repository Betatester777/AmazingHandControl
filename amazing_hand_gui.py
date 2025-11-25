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
from tkinter import font as tkfont
from textwrap import dedent
import numpy as np
from rustypot import Scs0009PyController
import threading
import time
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import matplotlib.dates as mdates

APP_VERSION = "0.7"
from datetime import datetime
import yaml


BASE_DIR = Path(__file__).resolve().parent
DATA_DIR = BASE_DIR / "data"
CONFIG_FILE = DATA_DIR / "hand_config.yaml"  # YAML with poses and sequences
APP_CONFIG_FILE = BASE_DIR / "config.yaml"  # Application settings
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


def load_app_config():
    """Load application configuration from config.yaml.
    
    Returns:
        dict: Application configuration with serial ports, limits, speeds, paths
    """
    default_config = {
        'serial': {
            'port_windows': 'COM9',
            'port_linux': '/dev/ttyACM0',
            'baudrate': 1000000,
            'baudrate_options': [9600, 115200, 1000000]
        },
        'servos': {
            'pointer': [1, 2],
            'middle': [3, 4],
            'ring': [5, 6],
            'thumb': [7, 8],
            'all_ids': [1, 2, 3, 4, 5, 6, 7, 8]
        },
        'limits': {
            'servo_min': -40,
            'servo_max': 110,
            'base_min': 0,
            'base_max': 110,
            'side_min': -40,
            'side_max': 40
        },
        'speeds': {
            'default': 3,
            'min': 1,
            'max': 6
        },
        'auto_extremes': {
            'left_open': [25, -40],
            'right_open': [-40, 25],
            'left_closed': [110, 110],
            'right_closed': [110, 110],
            'center_open': [0, 0],
            'center_closed': [110, 110]
        },
        'paths': {
            'poses_sequences_file': 'data/hand_config.yaml'
        }
    }
    
    if not APP_CONFIG_FILE.exists():
        return default_config
    
    try:
        with APP_CONFIG_FILE.open('r') as f:
            config = yaml.safe_load(f) or {}
            # Merge with defaults
            for key in default_config:
                if key not in config:
                    config[key] = default_config[key]
                elif isinstance(default_config[key], dict):
                    for subkey in default_config[key]:
                        if subkey not in config[key]:
                            config[key][subkey] = default_config[key][subkey]
            return config
    except Exception as e:
        print(f"Error loading app config: {e}, using defaults")
        return default_config


def default_serial_port():
    """Return platform-specific default serial port from config."""
    config = load_app_config()
    if os.name == 'nt':
        return config['serial']['port_windows']
    return config['serial']['port_linux']


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
        speed_var (StringVar): Servo speed selection (1-6)
        mimic_var (BooleanVar): Mimic mode enabled
    
    Note: Even servo IDs have inverted angles in hardware.
    """
    
    def __init__(self, parent, finger_name, servo1_id, servo2_id, controller, update_callback):
        self.servo1_id = servo1_id
        self.servo2_id = servo2_id
        self.controller = controller
        self.update_callback = update_callback
        self._suppress_events = False
        self.mode_var = tk.StringVar(value='auto')
        self._led_mode = 'idle'
        self._led_blink_job = None
        self._led_on = False
        
        # Load app config for defaults
        self.app_config = load_app_config()
        
        # Create frame for this finger
        # Layout: Grid with 2 columns
        # Col 0: Mimic checkbox / Position slider
        # Col 1: Mode selection / Side slider / Speed control
        self.frame = ttk.LabelFrame(parent, text=finger_name, padding=3)
        self.frame.columnconfigure(0, weight=1)
        self.frame.columnconfigure(1, weight=1)
        
        # Mimic checkbox
        self.mimic_var = tk.BooleanVar(value=False)
        self.mimic_check = ttk.Checkbutton(
            self.frame, text="Mimic",
            variable=self.mimic_var
        )
        self.mimic_check.grid(row=0, column=0, sticky='w')
        attach_tooltip(self.mimic_check, "When enabled, this finger mirrors close/open changes to other mimicking fingers.")

        mode_frame = ttk.Frame(self.frame)
        mode_frame.grid(row=0, column=1, sticky='e')
        led_bg = self._resolve_background_color()
        self.led_canvas = tk.Canvas(
            mode_frame, width=16, height=16, highlightthickness=0, bd=0,
            background=led_bg
        )
        self.led_indicator = self.led_canvas.create_oval(2, 2, 14, 14, fill='#5a5a5a', outline='#222222', width=1.5)
        self.led_canvas.pack(side='left', padx=(0, 6))
        attach_tooltip(self.led_canvas, "Finger status: blinking green=moving, red=blocked, gray=idle.")
        ttk.Radiobutton(mode_frame, text="Auto", value='auto', variable=self.mode_var,
                        command=self._update_mode_visibility).pack(side='left', padx=2)
        ttk.Radiobutton(mode_frame, text="Raw", value='raw', variable=self.mode_var,
                        command=self._update_mode_visibility).pack(side='left', padx=2)
        attach_tooltip(mode_frame, "Auto: base+offset sliders. Raw: direct control of both servo targets.")
        
        # Container that holds either auto or raw controls
        self.mode_stack = ttk.Frame(self.frame)
        self.mode_stack.grid(row=1, column=0, columnspan=2, sticky='nsew', pady=(3, 0))
        self.mode_stack.columnconfigure(0, weight=1)
        self.mode_stack.columnconfigure(1, weight=1)
        
        default_speed = self.app_config['speeds']['default']
        self.default_speed = default_speed
        self.speed_var = tk.StringVar(value=str(default_speed))

        # Auto mode controls
        self.auto_frame = ttk.Frame(self.mode_stack)
        self.auto_frame.grid(row=0, column=0, columnspan=2, sticky='nsew')
        self._build_auto_controls(self.auto_frame)
        
        # Raw mode controls
        self.raw_frame = ttk.Frame(self.mode_stack)
        self.raw_frame.grid(row=0, column=0, columnspan=2, sticky='nsew')
        self._build_raw_controls(self.raw_frame)
        
        self._update_mode_visibility()
        self.update_activity_state(False, False)
        
    def _build_auto_controls(self, parent):
        parent.grid_columnconfigure(0, weight=0)
        parent.grid_columnconfigure(1, weight=1)
        parent.grid_rowconfigure(0, weight=1)
        self.pos_var = tk.IntVar(value=0)
        base_min = self.app_config['limits']['base_min']
        base_max = self.app_config['limits']['base_max']
        self.pos_slider = ttk.Scale(
            parent, from_=base_max, to=base_min, orient='vertical',
            variable=self.pos_var, command=self.on_position_change,
            length=200
        )
        self.pos_slider.grid(row=0, column=0, padx=5, sticky='ns', pady=(0, 2))
        self.pos_label = ttk.Label(parent, text="0°")
        self.pos_label.grid(row=1, column=0)
        attach_tooltip(self.pos_slider, "Drag or scroll to close/open the finger (0°=open, 110°=closed).")
        
        # Bind mouse wheel to position slider
        self.pos_slider.bind('<Button-4>', self.on_mouse_wheel)  # Linux scroll up
        self.pos_slider.bind('<Button-5>', self.on_mouse_wheel)  # Linux scroll down
        self.pos_slider.bind('<MouseWheel>', self.on_mouse_wheel)  # Windows/Mac
        
        # Side-to-side slider and speed controls share the right column
        right_col = ttk.Frame(parent)
        right_col.grid(row=0, column=1, sticky='nsew', pady=(0, 0))
        right_col.grid_columnconfigure(0, weight=1)
        self.side_var = tk.IntVar(value=0)
        side_min = self.app_config['limits']['side_min']
        side_max = self.app_config['limits']['side_max']
        self.side_slider = ttk.Scale(
            right_col, from_=side_max, to=side_min, orient='horizontal',
            variable=self.side_var, command=self.on_side_change,
            length=200
        )
        self.side_slider.grid(row=0, column=0, padx=5, sticky='new', pady=(0, 0))
        self.side_label = ttk.Label(right_col, text="0°")
        self.side_label.grid(row=1, column=0, sticky='w', padx=5)
        attach_tooltip(self.side_slider, "Drag to move finger laterally (negative=left, positive=right).")

        speed_min = self.app_config['speeds']['min']
        speed_max = self.app_config['speeds']['max']
        speed_options = [str(i) for i in range(speed_min, speed_max + 1)]
        speed_frame = ttk.Frame(right_col)
        speed_frame.grid(row=2, column=0, sticky='ew', pady=(0, 0))
        ttk.Label(speed_frame, text="Speed:").pack(side='left', padx=(0, 4))
        self.speed_combo = ttk.Combobox(
            speed_frame,
            textvariable=self.speed_var,
            values=speed_options,
            state='readonly',
            width=6,
            justify='center'
        )
        self.speed_combo.pack(side='left', padx=2)
        attach_tooltip(self.speed_combo, "Servo motion speed (1=slowest, 6=fastest).")

        self.center_btn = ttk.Button(
            speed_frame, text="⊙ Center",
            command=self.center_finger,
            width=10
        )
        self.center_btn.pack(side='left', padx=(8, 0))
        attach_tooltip(self.center_btn, "Reset left/right offset to 0° for this finger.")
        
    def _build_raw_controls(self, parent):
        parent.grid_columnconfigure(0, weight=1)
        parent.grid_columnconfigure(1, weight=1)
        parent.grid_rowconfigure(0, weight=1)
        self.raw_pos1_var = tk.IntVar(value=0)
        self.raw_pos2_var = tk.IntVar(value=0)
        servo_min = self.app_config['limits']['servo_min']
        servo_max = self.app_config['limits']['servo_max']
        self.raw_slider1 = ttk.Scale(
            parent, from_=servo_max, to=servo_min, orient='vertical',
            variable=self.raw_pos1_var, command=lambda v: self._on_raw_change(1, v),
            length=200
        )
        self.raw_slider2 = ttk.Scale(
            parent, from_=servo_max, to=servo_min, orient='vertical',
            variable=self.raw_pos2_var, command=lambda v: self._on_raw_change(2, v),
            length=200
        )
        self.raw_slider1.grid(row=0, column=0, padx=5, sticky='ns', pady=(0, 2))
        self.raw_slider2.grid(row=0, column=1, padx=5, sticky='ns', pady=(0, 2))
        self.raw_pos1_label = ttk.Label(parent, text="0°")
        self.raw_pos2_label = ttk.Label(parent, text="0°")
        self.raw_pos1_label.grid(row=1, column=0)
        self.raw_pos2_label.grid(row=1, column=1)
        attach_tooltip(self.raw_slider1, "Directly command servo 1 target angle.")
        attach_tooltip(self.raw_slider2, "Directly command servo 2 target angle.")
    
    def center_finger(self):
        """Center this finger's side-to-side movement."""
        if self.mode_var.get() == 'raw':
            return
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
        if self.mode_var.get() == 'raw':
            servo_min = self.app_config['limits']['servo_min']
            servo_max = self.app_config['limits']['servo_max']
            current = self.raw_pos1_var.get()
            new_val = clamp(current + delta, servo_min, servo_max)
            if new_val == current:
                return None
            self.raw_pos1_var.set(new_val)
            self._on_raw_change(1, new_val)
            return new_val
        base_min = self.app_config['limits']['base_min']
        base_max = self.app_config['limits']['base_max']
        current = self.pos_var.get()
        new_val = clamp(current + delta, base_min, base_max)
        if new_val == current:
            return None
        self.pos_var.set(new_val)
        self.on_position_change(new_val)
        return new_val

    def adjust_side(self, delta):
        """Adjust side-to-side slider by delta degrees."""
        if self.mode_var.get() == 'raw':
            servo_min = self.app_config['limits']['servo_min']
            servo_max = self.app_config['limits']['servo_max']
            current = self.raw_pos2_var.get()
            new_val = clamp(current + delta, servo_min, servo_max)
            if new_val == current:
                return None
            self.raw_pos2_var.set(new_val)
            self._on_raw_change(2, new_val)
            return new_val
        side_min = self.app_config['limits']['side_min']
        side_max = self.app_config['limits']['side_max']
        current = self.side_var.get()
        new_val = clamp(current + delta, side_min, side_max)
        if new_val == current:
            return None
        self.side_var.set(new_val)
        self.on_side_change(new_val)
        return new_val
    
    def on_position_change(self, value):
        """Handle position slider change."""
        if self._suppress_events:
            return
        pos = int(float(value))
        self.pos_label.config(text=f"{pos}°")
        self._sync_raw_from_auto()
        mimic = self if (self.mode_var.get() == 'auto' and self.mimic_var.get()) else None
        self.update_callback(mimic_source=mimic)
    
    def on_side_change(self, value):
        """Handle side slider change."""
        if self._suppress_events:
            return
        side = int(float(value))
        self.side_label.config(text=f"{side}°")
        self._sync_raw_from_auto()
        mimic = self if (self.mode_var.get() == 'auto' and self.mimic_var.get()) else None
        self.update_callback(mimic_source=mimic)
    
    def get_positions(self):
        """Get current positions for both servos."""
        if self.mode_var.get() == 'raw':
            return self.raw_pos1_var.get(), self.raw_pos2_var.get()
        return self._auto_positions()
    
    def get_speed(self):
        """Get current speed setting."""
        try:
            return int(float(self.speed_var.get()))
        except (TypeError, ValueError):
            return int(self.default_speed)
    
    def set_positions(self, pos1, pos2):
        """Set slider positions from servo values."""
        servo_min = self.app_config['limits']['servo_min']
        servo_max = self.app_config['limits']['servo_max']
        side_min = self.app_config['limits']['side_min']
        side_max = self.app_config['limits']['side_max']
        
        pos1 = clamp(int(pos1), servo_min, servo_max)
        pos2 = clamp(int(pos2), servo_min, servo_max)
        self._set_raw_values(pos1, pos2)
        base_pos = (pos1 + pos2) // 2
        side_offset = clamp(pos1 - base_pos, side_min, side_max)
        self._set_auto_values(base_pos, side_offset)

    def _auto_positions(self):
        base_min = self.app_config['limits']['base_min']
        base_max = self.app_config['limits']['base_max']
        side_min = self.app_config['limits']['side_min']
        side_max = self.app_config['limits']['side_max']
        servo_min = self.app_config['limits']['servo_min']
        servo_max = self.app_config['limits']['servo_max']
        
        base_pos = clamp(self.pos_var.get(), base_min, base_max)
        side_offset = clamp(self.side_var.get(), side_min, side_max)
        
        # Interpolate between extreme poses from config
        extremes = self.app_config['auto_extremes']
        left_open = extremes['left_open']
        right_open = extremes['right_open']
        left_closed = extremes['left_closed']
        right_closed = extremes['right_closed']
        
        # Normalize base_pos (0..1)
        t = base_pos / base_max if base_max != 0 else 0.0
        
        if side_offset < 0:  # moving left
            # lerp from center to left extreme
            u = abs(side_offset) / abs(side_min) if side_min != 0 else 0.0
            center = (base_pos, base_pos)
            left_target = (
                left_open[0] + t * (left_closed[0] - left_open[0]),
                left_open[1] + t * (left_closed[1] - left_open[1])
            )
            pos1 = center[0] + u * (left_target[0] - center[0])
            pos2 = center[1] + u * (left_target[1] - center[1])
        elif side_offset > 0:  # moving right
            # lerp from center to right extreme
            u = side_offset / side_max if side_max != 0 else 0.0
            center = (base_pos, base_pos)
            right_target = (
                right_open[0] + t * (right_closed[0] - right_open[0]),
                right_open[1] + t * (right_closed[1] - right_open[1])
            )
            pos1 = center[0] + u * (right_target[0] - center[0])
            pos2 = center[1] + u * (right_target[1] - center[1])
        else:
            pos1 = base_pos
            pos2 = base_pos
        
        return clamp(int(pos1), servo_min, servo_max), clamp(int(pos2), servo_min, servo_max)

    def _set_auto_values(self, base_pos, side_offset):
        base_min = self.app_config['limits']['base_min']
        base_max = self.app_config['limits']['base_max']
        side_min = self.app_config['limits']['side_min']
        side_max = self.app_config['limits']['side_max']
        
        self._suppress_events = True
        try:
            self.pos_var.set(clamp(base_pos, base_min, base_max))
            self.side_var.set(clamp(side_offset, side_min, side_max))
        finally:
            self._suppress_events = False
        self.pos_label.config(text=f"{self.pos_var.get()}°")
        self.side_label.config(text=f"{self.side_var.get()}°")

    def _set_raw_values(self, pos1, pos2):
        servo_min = self.app_config['limits']['servo_min']
        servo_max = self.app_config['limits']['servo_max']
        
        self._suppress_events = True
        try:
            self.raw_pos1_var.set(clamp(pos1, servo_min, servo_max))
            self.raw_pos2_var.set(clamp(pos2, servo_min, servo_max))
        finally:
            self._suppress_events = False
        self.raw_pos1_label.config(text=f"{self.raw_pos1_var.get()}°")
        self.raw_pos2_label.config(text=f"{self.raw_pos2_var.get()}°")

    def _sync_raw_from_auto(self):
        pos1, pos2 = self._auto_positions()
        self._set_raw_values(pos1, pos2)

    def _sync_auto_from_raw(self):
        servo_min = self.app_config['limits']['servo_min']
        servo_max = self.app_config['limits']['servo_max']
        side_min = self.app_config['limits']['side_min']
        side_max = self.app_config['limits']['side_max']
        
        pos1 = clamp(self.raw_pos1_var.get(), servo_min, servo_max)
        pos2 = clamp(self.raw_pos2_var.get(), servo_min, servo_max)
        base_pos = (pos1 + pos2) // 2
        side_offset = clamp(pos1 - base_pos, side_min, side_max)
        self._set_auto_values(base_pos, side_offset)

    def _on_raw_change(self, slider_idx, value):
        if self._suppress_events:
            return
        val = int(float(value))
        if slider_idx == 1:
            self.raw_pos1_label.config(text=f"{val}°")
        else:
            self.raw_pos2_label.config(text=f"{val}°")
        self._sync_auto_from_raw()
        self.update_callback()

    def _update_mode_visibility(self):
        is_raw = self.mode_var.get() == 'raw'
        if is_raw:
            self.auto_frame.grid_remove()
            self.raw_frame.grid()
            self.center_btn.state(['disabled'])
            self.mimic_var.set(False)
            self.mimic_check.state(['disabled'])
            self._set_raw_values(*self._auto_positions())
        else:
            self.raw_frame.grid_remove()
            self.auto_frame.grid()
            self.center_btn.state(['!disabled'])
            self.mimic_check.state(['!disabled'])
            self._sync_auto_from_raw()

    def update_activity_state(self, is_moving=False, is_blocked=False):
        if is_blocked:
            self._set_led_mode('blocked')
        elif is_moving:
            self._set_led_mode('moving')
        else:
            self._set_led_mode('idle')

    # --- LED helpers -----------------------------------------------------

    def _resolve_background_color(self):
        for widget in (self.frame, self.frame.master, self.frame.winfo_toplevel()):
            if widget is None:
                continue
            try:
                color = widget.cget('background')
            except tk.TclError:
                color = None
            if color:
                return color
        return '#d9d9d9'

    def _set_led_mode(self, mode):
        if mode == getattr(self, '_led_mode', None):
            return
        self._led_mode = mode
        if self._led_blink_job:
            self.frame.after_cancel(self._led_blink_job)
            self._led_blink_job = None
        if mode == 'moving':
            self._led_on = True
            self._apply_led_color('#18d455')
            self._schedule_led_blink()
        elif mode == 'blocked':
            self._apply_led_color('#d64242')
        else:
            self._apply_led_color('#5a5a5a')

    def _schedule_led_blink(self):
        self._led_blink_job = self.frame.after(350, self._toggle_led_blink)

    def _toggle_led_blink(self):
        if self._led_mode != 'moving':
            self._led_blink_job = None
            return
        self._led_on = not self._led_on
        color = '#18d455' if self._led_on else '#0b7f30'
        self._apply_led_color(color)
        self._led_blink_job = self.frame.after(350, self._toggle_led_blink)

    def _apply_led_color(self, color):
        if getattr(self, 'led_canvas', None) and getattr(self, 'led_indicator', None):
            try:
                self.led_canvas.itemconfig(self.led_indicator, fill=color)
            except tk.TclError:
                pass


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
        self.root.title(f"AmazingHand Controller v{APP_VERSION}")
        self.root.geometry("1920x1200")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Store connection parameters
        self.initial_port = port
        self.initial_baudrate = baudrate
        self.controller = None
        self.connected = False
        
        # Performance optimization: debouncing and throttling
        self._resize_pending = False
        self._chart_update_pending = False
        self._last_chart_update_time = 0
        self._chart_update_delay_ms = 100  # Minimum 100ms between chart updates
        self._last_feedback_update_time = 0
        self._feedback_update_delay_ms = 50  # Minimum 50ms between feedback updates
        
        # Data storage for charts
        self.max_data_points = 100
        self.rolling_chart = True  # Rolling window mode
        self.time_data = []
        self.time_axis_data = []  # Cached matplotlib date numbers for each sample
        self.servo_data = {
            'target_pos': [[] for _ in range(8)],
            'current_pos': [[] for _ in range(8)],
            'load': [[] for _ in range(8)],
            'speed': [[] for _ in range(8)],
            'temperature': [[] for _ in range(8)],
            'voltage': [[] for _ in range(8)],
            'moving': [[] for _ in range(8)]
        }
        self.start_time = time.time()
        self.time_formatter = mdates.DateFormatter('%H:%M:%S')
        self.servo_colors = plt.cm.tab10(np.linspace(0, 1, 8))
        self.chart_lines = {}
        self.chart_status_text = None
        self.latest_actual_positions = None
        self.latest_actual_timestamp = 0.0
        self.actual_pos_lock = threading.Lock()
        self.movement_poll_interval = 0.2  # Seconds between moving-flag checks
        self.movement_timeout = 6.0  # Max seconds to wait before logging anyway
        self._moving_flags_supported = True
        self._moving_failure_count = 0
        self._moving_use_sync = None
        self._moving_sync_warning_logged = False
        self.pose_log_counter = 0
        self.scope_markers = []
        self.scope_ax2 = None
        self.chart_mode = tk.StringVar(value='Multi-Servo')
        self.scope_servo_var = tk.StringVar(value='1')
        self.latest_goal_positions = [0.0] * 8
        self.feedback_lock = threading.Lock()
        self.feedback_data = {
            'position': [0.0] * 8,
            'speed': [0.0] * 8,
            'load': [0.0] * 8,
            'voltage': [0.0] * 8,
            'temperature': [0.0] * 8,
            'current': [0.0] * 8,
            'moving': [0] * 8,
            'status': [0] * 8,
            'goal': [0.0] * 8
        }
        self.feedback_update_pending = False
        self.feedback_metric_specs = [
            ('goal', 'Goal (°)', "Commanded target stored on the servo."),
            ('position', 'Position (°)', "Latest measured position."),
            ('speed', 'Speed (°/s)', "Measured rotational speed."),
            ('load', 'Torque (%)', "Present load/torque reading (signed)."),
            ('voltage', 'Voltage (V)', "Supply voltage at the servo."),
            ('current', 'Current (mA)', "Estimated draw derived from load."),
            ('temperature', 'Temperature (°C)', "Internal temperature sensor."),
            ('status', 'Status', "Status register bitfield (hex)."),
            ('moving', 'Moving', "Servo moving flag (Yes/No).")
        ]
        self.feedback_frame = None
        self.feedback_cells = {}
        self._feedback_styles_ready = False
        self.blocked_error_threshold = 8.0  # Degrees difference signaling blockage
        
        # Sequence control flags
        self.stop_sequence = False
        self.pause_sequence = False
        self.sequence_running = False
        self.sequence_thread = None
        self.chart_paused = False
        self.chart_y_scale = 1.1
        self.chart_y_offset = 0.0
        self.chart_x_zoom = 1.0  # Fraction of available history displayed
        self.chart_x_pan = 0.0   # 0=start, 1=end
        self.y_zoom_var = tk.DoubleVar(value=self.chart_y_scale)
        self.y_pan_var = tk.DoubleVar(value=self.chart_y_offset)
        self.x_zoom_var = tk.DoubleVar(value=self.chart_x_zoom)
        self.x_pan_var = tk.DoubleVar(value=self.chart_x_pan)
        
        # Unified splitter styling (dark grey)
        # Options: sashwidth (1-15), sashrelief ('flat'|'raised'|'sunken'|'groove'|'ridge'|'solid')
        self.splitter_config = {
            'sashwidth': 6,              # Splitter width in pixels
            'sashrelief': 'ridge',       # 3D effect: 'raised', 'sunken', 'groove', 'ridge', 'flat', 'solid'
            'bg': "#b6b6b6",             # Background/sash color (dark grey)
            'bd':0,                     # Border width
            'relief': 'flat',            # Overall PanedWindow relief
            'handlepad': 5,              # Padding around handle
            'handlesize': 8,             # Handle size
            'cursor': 'sb_v_double_arrow'  # Cursor hint for vertical splitter
        }
        
        # Keyboard control
        self.selected_finger_idx = 0  # Default to first finger
        
        # Note: Tk bitmap images are unreliable across platforms here, so
        # we stick to text-only buttons (with Unicode where appropriate).

        # Create main container with paned window (vertical sash between control + chart)
        # Layout: Horizontal PanedWindow
        # Left: Controls (Finger controls, Global controls, Log)
        # Right: Charts (Servo monitoring)
        paned = tk.PanedWindow(self.root, orient='horizontal', **self.splitter_config)
        paned.pack(fill='both', expand=True)
        
        # Left shell with its own vertical splitter (controls vs log)
        # Layout: Vertical PanedWindow inside Left Panel
        # Top: Finger Controls & Settings
        # Bottom: Execution Log
        left_outer = ttk.Frame(paned, padding=3)
        paned.add(left_outer)
        left_splitter = tk.PanedWindow(left_outer, orient='vertical', **self.splitter_config)
        left_splitter.pack(fill='both', expand=True)
        self.left_splitter = left_splitter
        
        left_frame = ttk.Frame(left_splitter)
        left_splitter.add(left_frame)
        log_container = ttk.Frame(left_splitter)
        left_splitter.add(log_container)
        log_container.columnconfigure(0, weight=1)
        log_container.rowconfigure(1, weight=1)
        # left_frame grid summary:
        # Columns 0-5: even weights for finger widgets + settings stack
        # Row 0: Title
        # Row 1: First three finger panels (2 cols each)
        # Row 2: Thumb panel (cols 0-1) + settings stack (cols 2-5)
        for col in range(6):
            left_frame.grid_columnconfigure(col, weight=1)
        left_frame.grid_rowconfigure(1, weight=0)
        left_frame.grid_rowconfigure(2, weight=0)
        
        # Right panel - charts
        right_frame = ttk.Frame(paned, padding=3)
        paned.add(right_frame)
        
        # Title
        # Grid Layout for Left Frame:
        # Row 0: Title
        # Row 1: Finger Controls (Pointer, Middle, Ring)
        # Row 2: Finger Controls (Thumb) + Right Controls Stack
        ttk.Label(
            left_frame, text=f"AmazingHand Controller v{APP_VERSION}",
            font=('Arial', 16, 'bold')
        ).grid(row=0, column=0, columnspan=6, pady=10)
        
        # Create finger controls
        self.fingers = []
        app_config = load_app_config()
        self.finger_names = ['Pointer finger', 'Middle finger', 'Ring finger', 'Thumb']
        servo_pairs = [
            app_config['servos']['pointer'],
            app_config['servos']['middle'],
            app_config['servos']['ring'],
            app_config['servos']['thumb']
        ]
        
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
            finger.frame.grid(row=row, column=col, columnspan=2, padx=3, pady=2, sticky='nsew')
            self.fingers.append(finger)

        # Right-side stacked controls next to thumb (span remaining columns)
        # Layout: Stacked vertically in Grid Row 2, Columns 2-5
        # 1. Connection
        # 2. Global Controls
        # 3. Pose Management
        # 4. Sequence Player
        right_controls_frame = ttk.Frame(left_frame)
        right_controls_frame.grid(row=2, column=2, columnspan=4, padx=(6, 0), pady=2, sticky='nsew')
        
        # Connection options - top of right stack
        conn_frame = ttk.LabelFrame(right_controls_frame, text="Connection", padding=3)
        conn_frame.pack(fill='x', expand=True, pady=(0, 2))
        
        conn_row = ttk.Frame(conn_frame)
        conn_row.pack(fill='x', expand=True)
        
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
        app_config = load_app_config()
        baudrate_options = [str(b) for b in app_config['serial']['baudrate_options']]
        baudrate_combo = ttk.Combobox(
            conn_row, textvariable=self.baudrate_var, 
            values=baudrate_options, width=8, state='readonly'
        )
        baudrate_combo.pack(side='left', padx=2)
        attach_tooltip(baudrate_combo, "Serial communication speed.")
        
        self.connect_btn = ttk.Button(
            conn_row, text="▶ Connect", command=self.connect_controller, width=10
        )
        self.connect_btn.pack(side='left', padx=2)
        attach_tooltip(self.connect_btn, "Connect to the hand controller.")
        
        self.disconnect_btn = ttk.Button(
            conn_row, text="⏹ Disconnect", command=self.disconnect_controller, width=14
        )
        self.disconnect_btn.pack(side='left', padx=2)
        self.disconnect_btn.state(['disabled'])
        attach_tooltip(self.disconnect_btn, "Disconnect from the controller.")
        
        # Global controls - second in right stack
        control_frame = ttk.LabelFrame(right_controls_frame, text="Global Controls", padding=3)
        control_frame.pack(fill='x', expand=True, pady=2)
        
        # First row: Preset buttons
        buttons_row = ttk.Frame(control_frame)
        buttons_row.pack(fill='x', expand=True)
        
        # Preset buttons
        self.open_all_btn = ttk.Button(
            buttons_row, text="✋ Open All", command=self.open_all
        )
        self.open_all_btn.pack(side='left', padx=5)
        attach_tooltip(self.open_all_btn, "Set every finger to fully open (0°).")
        
        self.close_all_btn = ttk.Button(
            buttons_row, text="✊ Close All", command=self.close_all
        )
        self.close_all_btn.pack(side='left', padx=5)
        attach_tooltip(self.close_all_btn, "Set every finger to fully closed (110°).")
        
        self.center_all_btn = ttk.Button(
            buttons_row, text="⊙ Center All", command=self.center_all
        )
        self.center_all_btn.pack(side='left', padx=5)
        attach_tooltip(self.center_all_btn, "Reset all side-to-side offsets to 0°.")
    
        
        ttk.Label(buttons_row, text="Global Speed:").pack(side='left', padx=(5, 2))
        self.global_speed_var = tk.StringVar(value="3")
        global_speed_combo = ttk.Combobox(
            buttons_row, textvariable=self.global_speed_var,
            values=['1', '2', '3', '4', '5', '6'], width=3, state='readonly'
        )
        global_speed_combo.pack(side='left', padx=2)
        global_speed_combo.bind('<<ComboboxSelected>>', lambda e: self.set_all_speeds())
        attach_tooltip(global_speed_combo, "Set speed for all finger controls (1=slow, 6=fast).")
        
        # Pose management section - middle of right stack
        pose_mgmt_frame = ttk.LabelFrame(right_controls_frame, text="Pose Management", padding=3)
        pose_mgmt_frame.pack(fill='x', expand=True, pady=2)
        
        # Single-row layout for pose selection and creation
        poses_list = load_pose_definitions()
        default_pose = poses_list[0]['name'] if poses_list else ""
        self.pose_var = tk.StringVar(value=default_pose)
        pose_row = ttk.Frame(pose_mgmt_frame)
        pose_row.pack(fill='x', expand=True, pady=2)
        pose_row.columnconfigure(3, weight=1)

        ttk.Label(pose_row, text="Pose:").grid(row=0, column=0, padx=(0,2))
        self.pose_combo = ttk.Combobox(
            pose_row,
            textvariable=self.pose_var,
            state='readonly',
            width=14,
            values=[s['name'] for s in poses_list] or ["<no poses>"]
        )
        self.pose_combo.grid(row=0, column=1, padx=2)
        attach_tooltip(self.pose_combo, "Select a saved pose.")

        self.set_pose_btn = ttk.Button(
            pose_row, text="✓ Apply", command=self.set_selected_pose, width=10
        )
        self.set_pose_btn.grid(row=0, column=2, padx=5)
        attach_tooltip(self.set_pose_btn, "Apply the selected pose to all fingers.")

        ttk.Label(pose_row, text="Name:").grid(row=0, column=3, padx=(10,2), sticky='e')
        self.save_pose_name_var = tk.StringVar()
        self.save_pose_entry = ttk.Entry(pose_row, textvariable=self.save_pose_name_var, width=14)
        self.save_pose_entry.grid(row=0, column=4, padx=2)
        attach_tooltip(self.save_pose_entry, "Enter a name for the pose. Avoid special chars: : { } [ ] , & * # ? | - < > = ! % @ ` \" '")

        self.save_pose_btn = ttk.Button(
            pose_row, text="➕ Add New", command=self.save_pose, width=10
        )
        self.save_pose_btn.grid(row=0, column=5, padx=5)
        attach_tooltip(self.save_pose_btn, "Save current finger positions as a new pose.")
       
        # Sequence player section - bottom of right stack
        seq_player_frame = ttk.LabelFrame(right_controls_frame, text="Sequence Player", padding=3)
        seq_player_frame.pack(fill='x', expand=True, pady=2)
        
        # First row: sequence selection
        seq_row1 = ttk.Frame(seq_player_frame)
        seq_row1.pack(fill='x', expand=True, pady=2)
        
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
        
        self.loop_sequence_var = tk.BooleanVar(value=False)
        self.loop_check = ttk.Checkbutton(seq_row1, text="Loop", variable=self.loop_sequence_var)
        self.loop_check.pack(side='left', padx=5)
        attach_tooltip(self.loop_check, "When enabled, selected sequence repeats continuously until stopped.")
        
        self.play_btn = ttk.Button(
            seq_row1, text="▶ Play", command=self.play_selected_sequence, width=10
        )
        self.play_btn.pack(side='left', padx=2)
        attach_tooltip(self.play_btn, "Start playing the selected sequence.")
        
        self.pause_btn = ttk.Button(
            seq_row1, text="⏸ Pause", command=self.pause_sequence_exec, width=10
        )
        self.pause_btn.pack(side='left', padx=2)
        self.pause_btn.state(['disabled'])
        attach_tooltip(self.pause_btn, "Pause/resume the running sequence.")
        
        self.stop_btn = ttk.Button(
            seq_row1, text="⏹ Stop", command=self.stop_sequence_exec, width=10
        )
        self.stop_btn.pack(side='left', padx=2)
        self.stop_btn.state(['disabled'])
        attach_tooltip(self.stop_btn, "Stop the running sequence.")

        self.manage_sequences_btn = ttk.Button(
            seq_row1, text="🔧 Manage", command=self.manage_sequences, width=10
        )
        self.manage_sequences_btn.pack(side='right', padx=5)
        attach_tooltip(self.manage_sequences_btn, "Open sequence builder and editor.")
        
        # Refresh sequences list
        self.refresh_sequences_list()
        
        # Bind keyboard events
        self.root.bind('<Key>', self.on_key_press)
        
        # log_container grid summary:
        # Row 0: Status bar (fixed height)
        # Row 1: Execution log frame (expands)
        # Status bar + log stacked in bottom pane
        self.status_var = tk.StringVar(value="Ready")
        status_bar = ttk.Label(
            log_container, textvariable=self.status_var,
            relief='sunken', anchor='w'
        )
        status_bar.grid(row=0, column=0, sticky='ew')
        attach_tooltip(status_bar, "Most recent action, warning, or error message.")
        
        # Log output - bottom, expandable
        log_frame = ttk.LabelFrame(log_container, text="Execution Log", padding=(3,0,3,3))
        log_frame.grid(row=1, column=0, sticky='nsew')
        
        log_scroll = ttk.Scrollbar(log_frame)
        log_scroll.pack(side='right', fill='y')
        
        self.log_text = tk.Text(log_frame, height=12, wrap='word', yscrollcommand=log_scroll.set, state='disabled')
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
        # Layout:
        # Top: Controls Row (Pause, Display Metrics, Chart Mode, Servo Selection)
        # Bottom: Matplotlib Chart Canvas
        
        # Metric selection & chart controls (single row)
        controls_row = ttk.Frame(parent)
        controls_row.pack(fill='x', padx=6, pady=(5,4))

        self.chart_pause_btn = ttk.Button(controls_row, text="⏸ Pause Chart", command=self.toggle_chart_pause, width=14)
        self.chart_pause_btn.pack(side='left', padx=(0,4))
        attach_tooltip(self.chart_pause_btn, "Stop refreshing the plot without affecting telemetry collection.")

        clear_btn = ttk.Button(controls_row, text="⌫ Clear", command=self.clear_chart_data, width=9)
        clear_btn.pack(side='left', padx=2)
        attach_tooltip(clear_btn, "Reset collected telemetry and start fresh.")

        self.rolling_var = tk.BooleanVar(value=True)
        rolling_check = ttk.Checkbutton(
            controls_row, text="Rolling",
            variable=self.rolling_var,
            command=self.toggle_rolling
        )
        rolling_check.pack(side='left', padx=10)
        attach_tooltip(rolling_check, "Limit chart to latest samples instead of growing indefinitely.")

        ttk.Label(controls_row, text="Display:").pack(side='left', padx=(4,4))
        self.display_dropdown = ttk.Menubutton(controls_row, text="Display")
        self.display_dropdown.pack(side='left', padx=(0,8))
        self.display_menu = tk.Menu(self.display_dropdown, tearoff=0)
        self.display_dropdown['menu'] = self.display_menu
        
        self.chart_metrics = {
            'current_pos': tk.BooleanVar(value=True),
            'target_vs_current': tk.BooleanVar(value=False),
            'load': tk.BooleanVar(value=False),
            'speed': tk.BooleanVar(value=False),
            'temperature': tk.BooleanVar(value=False),
            'voltage': tk.BooleanVar(value=False),
            'moving': tk.BooleanVar(value=False)
        }
        
        metrics = [
            ('Position', 'current_pos'),
            ('Target vs Current', 'target_vs_current'),
            ('Torque', 'load'),
            ('Speed', 'speed'),
            ('Temperature', 'temperature'),
            ('Voltage', 'voltage'),
            ('Moving', 'moving')
        ]
        
        metric_tips = {
            'current_pos': "Plot servo position history in degrees.",
            'target_vs_current': "Overlay commanded target vs. measured position.",
            'load': "Show load/force (torque %) for each servo.",
            'speed': "Display servo speed feedback (deg/s).",
            'temperature': "Display internal temperature readings.",
            'voltage': "Monitor supply voltage levels.",
            'moving': "Plot moving flags (1=moving, 0=idle) for each servo."
        }
        for label, value in metrics:
            var = self.chart_metrics[value]
            self.display_menu.add_checkbutton(
                label=label,
                variable=var,
                command=self._on_metric_toggle
            )
            var.trace_add('write', lambda *_: self._update_display_dropdown_label())
        attach_tooltip(self.display_dropdown, "Choose telemetry metrics to plot.")
        self._update_display_dropdown_label()
        
        # Servo selection dropdown and action buttons
        ttk.Label(controls_row, text="Servos:").pack(side='left', padx=(6,4))
        dropdown_container = ttk.Frame(controls_row)
        dropdown_container.pack(side='left')
        self.servo_visible = [tk.BooleanVar(value=True) for _ in range(8)]
        self.servo_dropdown = ttk.Menubutton(dropdown_container, text="Servos")
        self.servo_dropdown.pack(side='left')
        self.servo_menu = tk.Menu(self.servo_dropdown, tearoff=0)
        self.servo_dropdown['menu'] = self.servo_menu
        for i in range(8):
            var = self.servo_visible[i]
            self.servo_menu.add_checkbutton(
                label=f"S{i+1}",
                variable=var,
                command=self._on_servo_toggle
            )
            var.trace_add('write', lambda *_: self._update_servo_dropdown_label())
        attach_tooltip(self.servo_dropdown, "Select which servo traces are visible.")
        self._update_servo_dropdown_label()
        
        controls_btn_group = ttk.Frame(controls_row)
        controls_btn_group.pack(side='left', padx=(8,0))
        
        all_btn = ttk.Button(controls_btn_group, text="✓ All", command=self.select_all_servos, width=9)
        all_btn.pack(side='left', padx=2)
        attach_tooltip(all_btn, "Enable all servo traces.")
        none_btn = ttk.Button(controls_btn_group, text="✕ None", command=self.deselect_all_servos, width=9)
        none_btn.pack(side='left', padx=2)
        attach_tooltip(none_btn, "Hide all servo traces (useful before selecting a subset).")

        # Chart mode and servo selection controls
        ttk.Label(controls_row, text="Chart Mode:").pack(side='left', padx=(4,4))
        mode_combo = ttk.Combobox(
            controls_row,
            textvariable=self.chart_mode,
            values=['Multi-Servo', 'Scope'],
            width=12,
            state='readonly'
        )
        mode_combo.pack(side='left', padx=(0,8))
        attach_tooltip(mode_combo, "Switch between multi-servo view and single-servo oscilloscope style view.")
        mode_combo.bind('<<ComboboxSelected>>', lambda _e: self._on_chart_mode_change())
        self.chart_mode.trace_add('write', lambda *_: self._on_chart_mode_change())
        self.scope_servo_label_pack = {'side': 'left', 'padx': (0, 4)}
        self.scope_servo_label = ttk.Label(controls_row, text="Scope Servo:")
        self.scope_servo_label.pack(**self.scope_servo_label_pack)
        self.scope_servo_combo = ttk.Combobox(
            controls_row,
            textvariable=self.scope_servo_var,
            values=[str(i) for i in range(1, 9)],
            width=4,
            state='readonly'
        )
        self.scope_servo_combo_pack = {'side': 'left', 'padx': (0, 8)}
        self.scope_servo_combo.pack(**self.scope_servo_combo_pack)
        attach_tooltip(self.scope_servo_combo, "Servo channel used for scope view and feedback panel.")
        self.scope_servo_combo.bind('<<ComboboxSelected>>', lambda _e: self._on_scope_servo_change())
        self.scope_servo_var.trace_add('write', lambda *_: self._on_scope_servo_change())
        self._update_scope_servo_visibility()
        
        # Chart + feedback layout container (vertical splitter)
        chart_feedback_split = tk.PanedWindow(
            parent,
            orient='vertical',
            **self.splitter_config
        )
        # Bind resize event with debouncing
        chart_feedback_split.bind('<Configure>', self._on_pane_resize_debounce)
        chart_feedback_split.pack(fill='both', expand=True, padx=5, pady=(0,5))
        self.chart_feedback_split = chart_feedback_split

        chart_container = ttk.Frame(chart_feedback_split)
        chart_feedback_split.add(chart_container)
        chart_feedback_split.paneconfig(chart_container, stretch='always', minsize=200)

        feedback_container = ttk.Frame(chart_feedback_split)
        chart_feedback_split.add(feedback_container)
        chart_feedback_split.paneconfig(feedback_container, stretch='always', minsize=150)
        # Right side - Y axis sliders (vertical, 100% height)
        y_sliders_frame = ttk.Frame(chart_container, width=30)
        y_sliders_frame.pack(side='right', fill='y', padx=(4, 0), pady=2)

        # Chart and slider layout container
        
        # Y Zoom slider (top half)
        y_zoom_container = ttk.Frame(y_sliders_frame)
        y_zoom_container.pack(fill='both', expand=True, pady=(0,3))
        
        ttk.Label(
            y_zoom_container,
            text="Y\nZoom",
            anchor='center',
            justify='center'
        ).pack(anchor='center')
        self.y_zoom_slider = ttk.Scale(
            y_zoom_container, from_=5.0, to=0.2, orient='vertical',
            variable=self.y_zoom_var, command=self._on_y_zoom_slider
        )
        self.y_zoom_slider.pack(fill='both', expand=True, pady=1, padx=2)
        self.y_zoom_value_label = ttk.Label(
            y_zoom_container,
            text=f"{self.chart_y_scale:.2f}×",
            anchor='center',
            justify='center'
        )
        self.y_zoom_value_label.pack(anchor='center')
        attach_tooltip(self.y_zoom_slider, "Slide to zoom the Y axis in/out.")
        
        # Y Pan slider (bottom half)
        y_pan_container = ttk.Frame(y_sliders_frame)
        y_pan_container.pack(fill='both', expand=True, pady=(3,0))
        
        ttk.Label(
            y_pan_container,
            text="Y\nPan",
            anchor='center',
            justify='center'
        ).pack(anchor='center')
        self.y_pan_slider = ttk.Scale(
            y_pan_container, from_=3.0, to=-3.0, orient='vertical',
            variable=self.y_pan_var, command=self._on_y_pan_slider
        )
        self.y_pan_slider.pack(fill='both', expand=True, pady=1, padx=2)
        self.y_pan_value_label = ttk.Label(
            y_pan_container,
            text=f"{self.chart_y_offset:.2f}",
            anchor='center',
            justify='center'
        )
        self.y_pan_value_label.pack(anchor='center')
        attach_tooltip(self.y_pan_slider, "Shift the Y axis window up or down.")
        
        # Left side - chart and time sliders
        chart_and_time_frame = ttk.Frame(chart_container)
        chart_and_time_frame.pack(side='left', fill='both', expand=True)
        
        # Chart area
        chart_plot_frame = ttk.Frame(chart_and_time_frame)
        chart_plot_frame.pack(fill='both', expand=True)
        self._sync_chart_control_vars()
        
        # Create matplotlib figure
        self.fig = Figure(figsize=(10, 6), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.fig.subplots_adjust(top=0.96, bottom=0.14, left=0.08, right=0.98)
        self.ax.xaxis.set_major_formatter(self.time_formatter)
        self.ax.tick_params(axis='x', labelrotation=15)
        self.ax.xaxis_date()
        self.ax.grid(True, alpha=0.3)
        self._ensure_chart_message_artist()
        
        self.canvas = FigureCanvasTkAgg(self.fig, chart_plot_frame)
        canvas_widget = self.canvas.get_tk_widget()
        canvas_widget.pack(fill='both', expand=True)
        attach_tooltip(canvas_widget, "Scroll to zoom, drag to pan; updates roughly every second.")
        
        # Time sliders below chart (100% width)
        time_sliders_frame = ttk.Frame(chart_and_time_frame)
        time_sliders_frame.pack(fill='x', pady=(5,0))
        
        # Time Zoom slider
        time_zoom_frame = ttk.Frame(time_sliders_frame)
        time_zoom_frame.pack(side='left', padx=10, expand=True, fill='x')
        ttk.Label(time_zoom_frame, text="Time Zoom").pack(side='left', padx=2)
        self.x_zoom_slider = ttk.Scale(
            time_zoom_frame, from_=1.0, to=0.1, orient='horizontal',
            variable=self.x_zoom_var, command=self._on_x_zoom_slider
        )
        self.x_zoom_slider.pack(side='left', padx=2, fill='x', expand=True)
        self.x_zoom_value_label = ttk.Label(time_zoom_frame, text="100%", width=5)
        self.x_zoom_value_label.pack(side='left')
        attach_tooltip(self.x_zoom_slider, "Zoom horizontally to focus on recent telemetry.")
        
        # Time Pan slider
        time_pan_frame = ttk.Frame(time_sliders_frame)
        time_pan_frame.pack(side='left', padx=10, expand=True, fill='x')
        ttk.Label(time_pan_frame, text="Time Pan").pack(side='left', padx=2)
        self.x_pan_slider = ttk.Scale(
            time_pan_frame, from_=0.0, to=1.0, orient='horizontal',
            variable=self.x_pan_var, command=self._on_x_pan_slider
        )
        self.x_pan_slider.pack(side='left', padx=2, fill='x', expand=True)
        self.x_pan_value_label = ttk.Label(time_pan_frame, text="0%", width=5)
        self.x_pan_value_label.pack(side='left')
        attach_tooltip(self.x_pan_slider, "Pan the horizontal window through recorded data.")
        
        # Servo feedback panel (scope-style readouts)
        self._build_feedback_panel(feedback_container)

        # Initialize plot
        self.update_chart()
        self.update_feedback_panel()

    def _ensure_feedback_styles(self):
        """Create ttk styles used by the feedback grid once."""
        if self._feedback_styles_ready:
            return
        style = ttk.Style(self.root)
        style.configure(
            'Feedback.Header.TLabel',
            background='#f2f2f2',
            font=('TkDefaultFont', 9, 'bold'),
            padding=(6, 6)
        )
        style.configure(
            'Feedback.Metric.TLabel',
            background='#fafafa',
            font=('TkDefaultFont', 9, 'bold'),
            padding=(6, 6)
        )
        style.configure(
            'Feedback.Cell.TLabel',
            background='#ffffff',
            font=('TkDefaultFont', 9),
            padding=(6, 4)
        )
        self._feedback_styles_ready = True

    def _build_feedback_panel(self, parent):
        """Create the servo feedback panel shown under the chart."""
        if self.feedback_frame is not None:
            try:
                self.feedback_frame.destroy()
            except Exception:
                pass
        self.feedback_frame = ttk.LabelFrame(parent, text="Servo Feedback", padding=3)
        self.feedback_frame.pack(fill='both', padx=5, pady=(0, 5), expand=True)

        self._ensure_feedback_styles()
        self.feedback_cells = {}
        table_container = ttk.Frame(self.feedback_frame)
        table_container.pack(fill='both', expand=True)
        self.feedback_canvas = tk.Canvas(table_container, highlightthickness=0)
        self.feedback_canvas.pack(side='left', fill='both', expand=True)
        scrollbar = ttk.Scrollbar(table_container, orient='vertical', command=self.feedback_canvas.yview)
        scrollbar.pack(side='right', fill='y')
        self.feedback_canvas.configure(yscrollcommand=scrollbar.set)

        grid_frame = ttk.Frame(self.feedback_canvas)
        self.feedback_table_window = self.feedback_canvas.create_window((0, 0), window=grid_frame, anchor='nw')

        def _sync_feedback_scroll(_event=None):
            self.feedback_canvas.configure(scrollregion=self.feedback_canvas.bbox('all'))

        grid_frame.bind('<Configure>', _sync_feedback_scroll)
        self.feedback_canvas.bind(
            '<Configure>',
            lambda event: self.feedback_canvas.itemconfigure(self.feedback_table_window, width=event.width)
        )

        headings = ['Metric'] + [f'S{i}' for i in range(1, 9)]
        for col_index, heading in enumerate(headings):
            lbl = ttk.Label(
                grid_frame, text=heading,
                style='Feedback.Header.TLabel',
                relief='solid', borderwidth=1,
                padding=(6, 4), anchor='center'
            )
            lbl.grid(row=0, column=col_index, sticky='nsew')
            attach_tooltip(lbl, f"Column heading: {heading}")
            grid_frame.columnconfigure(col_index, weight=1)
        grid_frame.rowconfigure(0, weight=0, minsize=30)

        for row_idx, (key, label, tip) in enumerate(self.feedback_metric_specs, start=1):
            row_label = ttk.Label(
                grid_frame, text=label,
                style='Feedback.Metric.TLabel',
                relief='solid', borderwidth=1,
                padding=(6, 4), anchor='w'
            )
            row_label.grid(row=row_idx, column=0, sticky='nsew')
            attach_tooltip(row_label, tip)
            grid_frame.rowconfigure(row_idx, weight=1, minsize=28)

            for col in range(1, len(headings)):
                cell = ttk.Label(
                    grid_frame, text='—',
                    style='Feedback.Cell.TLabel',
                    relief='solid', borderwidth=1,
                    padding=(4, 2), anchor='center'
                )
                cell.grid(row=row_idx, column=col, sticky='nsew')
                self.feedback_cells[(key, col-1)] = cell
                attach_tooltip(cell, f"{label}: Servo S{col}")

    def _on_pane_resize_debounce(self, event=None):
        """Debounce resize events to avoid excessive redraws during drag."""
        if self._resize_pending:
            return
        self._resize_pending = True
        # Defer the actual canvas draw until resize stops (50ms)
        self.root.after(50, self._finalize_pane_resize)

    def _finalize_pane_resize(self):
        """Execute canvas redraw after resize events settle."""
        self._resize_pending = False
        if hasattr(self, 'canvas') and self.canvas:
            self.canvas.draw_idle()

    def _request_feedback_refresh(self):
        """Schedule a UI refresh for the feedback panel (thread-safe) with throttling."""
        if self.feedback_update_pending:
            return
        self.feedback_update_pending = True
        # Schedule with minimum time delay to avoid excessive updates
        current_time = time.time() * 1000  # milliseconds
        time_since_last = current_time - self._last_feedback_update_time
        if time_since_last < self._feedback_update_delay_ms:
            # Reschedule for later
            delay_ms = int(self._feedback_update_delay_ms - time_since_last)
            self.root.after(max(10, delay_ms), self._apply_feedback_refresh)
        else:
            self.root.after(0, self._apply_feedback_refresh)

    def _apply_feedback_refresh(self):
        """Execute the pending feedback refresh on the Tk thread."""
        self.feedback_update_pending = False
        self._last_feedback_update_time = time.time() * 1000
        self.update_feedback_panel()

    def _update_scope_servo_visibility(self):
        """Show scope servo selector only when scope mode is active."""
        visible = self.chart_mode.get() == 'Scope'
        label = getattr(self, 'scope_servo_label', None)
        combo = getattr(self, 'scope_servo_combo', None)
        if visible:
            if label and not label.winfo_manager():
                label.pack(**getattr(self, 'scope_servo_label_pack', {}))
            if combo and not combo.winfo_manager():
                combo.pack(**getattr(self, 'scope_servo_combo_pack', {}))
        else:
            if label and label.winfo_manager():
                label.pack_forget()
            if combo and combo.winfo_manager():
                combo.pack_forget()

    def _on_metric_toggle(self):
        self._update_display_dropdown_label()
        self._schedule_chart_update()

    def _update_display_dropdown_label(self):
        dropdown = getattr(self, 'display_dropdown', None)
        if dropdown is None:
            return
        total = len(self.chart_metrics)
        count = sum(1 for var in self.chart_metrics.values() if var.get())
        if count == 0:
            text = "Display (none)"
        elif count == total:
            text = "Display (all)"
        else:
            text = f"Display ({count})"
        dropdown.config(text=text)

    def _on_servo_toggle(self):
        self._update_servo_dropdown_label()
        self._schedule_chart_update()

    def _update_servo_dropdown_label(self):
        dropdown = getattr(self, 'servo_dropdown', None)
        if dropdown is None:
            return
        total = len(self.servo_visible)
        count = sum(1 for var in self.servo_visible if var.get())
        if count == 0:
            text = "Servos (none)"
        elif count == total:
            text = "Servos (all)"
        else:
            text = f"Servos ({count})"
        dropdown.config(text=text)

    def _on_chart_mode_change(self):
        """Handle chart mode combo box changes."""
        mode = self.chart_mode.get()
        self.status_var.set(f"Chart mode: {mode}")
        self._update_scope_servo_visibility()
        self._schedule_chart_update()

    def _on_scope_servo_change(self):
        """Respond when the user picks a different servo for scope/feedback."""
        value = self.scope_servo_var.get()
        try:
            servo = int(value)
        except (TypeError, ValueError):
            servo = 1
        servo = int(clamp(servo, 1, 8))
        if value != str(servo):
            self.scope_servo_var.set(str(servo))
            return
        self.status_var.set(f"Scope servo: S{servo}")
        self._schedule_chart_update()
        self.update_feedback_panel()

    def update_feedback_panel(self):
        """Update the servo feedback table."""
        if not self.feedback_cells:
            return

        data_snapshot = {}
        with self.feedback_lock:
            for key, _label, _tip in self.feedback_metric_specs:
                raw = self.feedback_data.get(key, [])
                if len(raw) < 8:
                    padded = list(raw) + [None] * (8 - len(raw))
                else:
                    padded = list(raw[:8])
                data_snapshot[key] = padded

        self._update_feedback_table(data_snapshot)
        self._update_finger_activity_indicators(data_snapshot)

    def _update_feedback_table(self, data_snapshot):
        """Refresh the rotated telemetry table (metrics as rows) - only update changed cells."""
        if not self.feedback_cells:
            return

        # Cache previous values to detect changes and avoid unnecessary config calls
        if not hasattr(self, '_feedback_cell_cache'):
            self._feedback_cell_cache = {}

        for key, label, _tip in self.feedback_metric_specs:
            metric_values = data_snapshot.get(key, [None] * 8)
            if len(metric_values) < 8:
                metric_values = list(metric_values) + [None] * (8 - len(metric_values))
            for idx in range(8):
                cell = self.feedback_cells.get((key, idx))
                if not cell:
                    continue
                
                new_text = self._format_feedback_value(key, metric_values[idx])
                cache_key = (key, idx)
                old_text = self._feedback_cell_cache.get(cache_key)
                
                # Only update if value changed
                if old_text != new_text:
                    cell.config(text=new_text)
                    self._feedback_cell_cache[cache_key] = new_text

    def _update_finger_activity_indicators(self, data_snapshot):
        if not self.fingers:
            return

        moving_values = data_snapshot.get('moving', [])
        goal_values = data_snapshot.get('goal', [])
        position_values = data_snapshot.get('position', [])
        threshold = self.blocked_error_threshold

        def _value(values, idx):
            if not isinstance(values, list):
                return None
            if idx < 0 or idx >= len(values):
                return None
            return values[idx]

        for finger_idx, finger in enumerate(self.fingers):
            servo_indices = [finger_idx * 2, finger_idx * 2 + 1]
            is_moving = False
            blocked_candidate = False
            for servo_idx in servo_indices:
                moving_flag = _value(moving_values, servo_idx)
                if moving_flag:
                    is_moving = True
                goal = _value(goal_values, servo_idx)
                pos = _value(position_values, servo_idx)
                if goal is None or pos is None:
                    continue
                try:
                    if abs(float(goal) - float(pos)) >= threshold:
                        blocked_candidate = True
                except (TypeError, ValueError):
                    continue
            is_blocked = blocked_candidate and not is_moving
            finger.update_activity_state(is_moving, is_blocked)

    def _format_feedback_value(self, key, value):
        """Convert raw feedback values into user-friendly strings."""
        if value is None:
            return '—'

        if key in ('goal', 'position'):
            return f"{float(value):.2f}°"
        if key == 'speed':
            return f"{float(value):.1f}°/s"
        if key == 'voltage':
            return f"{float(value):.2f} V"
        if key == 'temperature':
            return f"{float(value):.1f} °C"
        if key == 'current':
            return f"{float(value):.0f} mA"
        if key == 'load':
            return f"{self._load_to_percent(value):.1f} %"
        if key == 'status':
            return f"0x{int(value) & 0xFF:02X}"
        if key == 'moving':
            return "Yes" if bool(value) else "No"
        return str(value)

    def _load_to_percent(self, load_value):
        """Convert the present-load reading into a percentage."""
        try:
            load_float = float(load_value)
        except (TypeError, ValueError):
            return 0.0
        magnitude = abs(load_float)
        if magnitude <= 1.5:
            percent = magnitude * 100.0
        else:
            percent = magnitude / 10.23
        percent = clamp(percent, 0.0, 150.0)
        return percent if load_float >= 0 else -percent

    def _estimate_current_from_load(self, load_value):
        """Rudimentary current estimate derived from torque percentage."""
        percent = abs(self._load_to_percent(load_value))
        if percent <= 0.1:
            return 0.0
        # Assume roughly 1.2 A at 100% load; scale proportionally
        estimated_ma = clamp(percent / 100.0 * 1200.0, 0.0, 1500.0)
        return round(estimated_ma, 1)

    def _coerce_numeric(self, value, default=0.0):
        """Convert controller return types (arrays, lists) to floats."""
        if value is None:
            return default
        if isinstance(value, np.ndarray):
            try:
                return float(value.item())
            except Exception:
                data = value.tolist()
                return self._coerce_numeric(data[0], default) if data else default
        if isinstance(value, (list, tuple)):
            return self._coerce_numeric(value[0] if value else default, default)
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    def _coerce_angle_degrees(self, value, servo_id, default=0.0):
        """Convert a radian-based controller reading into GUI degrees."""
        radians = self._coerce_numeric(value, default)
        degrees = float(np.rad2deg(radians))
        if servo_id % 2 == 0:
            degrees = -degrees
        return degrees

    def _coerce_bool(self, value):
        """Coerce controller return types to boolean flags."""
        return bool(int(self._coerce_numeric(value, 0.0)))
    
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
                        # Present position
                        current_pos = self._coerce_angle_degrees(
                            self.controller.read_present_position(servo_id),
                            servo_id
                        )
                        precise_pos = round(float(current_pos), 2)
                        with self.actual_pos_lock:
                            if self.latest_actual_positions is None:
                                self.latest_actual_positions = [0.0] * 8
                            self.latest_actual_positions[idx] = precise_pos
                            self.latest_actual_timestamp = time.time()
                        
                        load = self._coerce_numeric(self.controller.read_present_load(servo_id), 0.0)
                        temp = self._coerce_numeric(self.controller.read_present_temperature(servo_id), 0.0)
                        voltage = self._coerce_numeric(self.controller.read_present_voltage(servo_id), 0.0)
                        
                        # Get target position from finger controls
                        finger_idx = idx // 2
                        servo_in_finger = idx % 2
                        if finger_idx < len(self.fingers):
                            pos1, pos2 = self.fingers[finger_idx].get_positions()
                            target = pos1 if servo_in_finger == 0 else pos2
                        else:
                            target = 0
                        goal_value = target
                        try:
                            goal_value = self._coerce_angle_degrees(
                                self.controller.read_goal_position(servo_id),
                                servo_id
                            )
                        except Exception:
                            goal_value = target

                        speed_value = 0.0
                        try:
                            speed_value = self._coerce_angle_degrees(
                                self.controller.read_present_speed(servo_id),
                                servo_id
                            )
                        except Exception:
                            pass

                        status_word = 0
                        try:
                            status_word = int(self._coerce_numeric(self.controller.read_status(servo_id), 0.0))
                        except Exception:
                            pass

                        moving_flag = False
                        try:
                            moving_flag = self._coerce_bool(self.controller.read_moving(servo_id))
                        except Exception:
                            pass
                        
                        # Store data
                        self.servo_data['current_pos'][idx].append(current_pos)
                        self.servo_data['load'][idx].append(load)
                        self.servo_data['temperature'][idx].append(temp)
                        self.servo_data['voltage'][idx].append(voltage)
                        self.servo_data['target_pos'][idx].append(goal_value)
                        self.servo_data['speed'][idx].append(speed_value)
                        self.servo_data['moving'][idx].append(1 if moving_flag else 0)

                        with self.feedback_lock:
                            self.feedback_data['position'][idx] = precise_pos
                            self.feedback_data['speed'][idx] = round(float(speed_value), 2)
                            self.feedback_data['load'][idx] = load
                            self.feedback_data['voltage'][idx] = voltage
                            self.feedback_data['temperature'][idx] = temp
                            self.feedback_data['moving'][idx] = 1 if moving_flag else 0
                            self.feedback_data['status'][idx] = status_word
                            self.feedback_data['goal'][idx] = goal_value
                            self.feedback_data['current'][idx] = self._estimate_current_from_load(load)
                            self.latest_goal_positions[idx] = goal_value
                        
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
                    axis_value = mdates.date2num(datetime.fromtimestamp(self.start_time + current_time))
                    self.time_axis_data.append(axis_value)
                    self._request_feedback_refresh()
                    
                    # Keep only recent data if rolling mode is enabled
                    if self.rolling_chart and len(self.time_data) > self.max_data_points:
                        points_to_remove = len(self.time_data) - self.max_data_points
                        self.time_data = self.time_data[points_to_remove:]
                        if len(self.time_axis_data) >= points_to_remove:
                            self.time_axis_data = self.time_axis_data[points_to_remove:]
                        else:
                            self.time_axis_data = []
                        for key in self.servo_data:
                            for idx in range(8):
                                if len(self.servo_data[key][idx]) > points_to_remove:
                                    self.servo_data[key][idx] = self.servo_data[key][idx][points_to_remove:]
                
                # Update chart every 10 readings with throttling
                if len(self.time_data) % 10 == 0:
                    self.root.after(0, self._schedule_chart_update)
                
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
    
    def _time_slice_to_axis(self, time_slice, start_idx=None, end_idx=None):
        """Convert relative time values to matplotlib date numbers."""
        if start_idx is not None and end_idx is not None:
            cache_len = len(self.time_axis_data)
            if cache_len >= end_idx:
                return self.time_axis_data[start_idx:end_idx]
        if not time_slice:
            return []
        absolute = [datetime.fromtimestamp(self.start_time + t) for t in time_slice]
        return mdates.date2num(absolute)

    def _ensure_chart_message_artist(self):
        """Create (or return) the status text artist used for chart messages."""
        if self.chart_status_text is None:
            self.chart_status_text = self.ax.text(
                0.5, 0.5, '',
                ha='center', va='center', transform=self.ax.transAxes,
                color='#666666', fontsize=11, fontweight='bold'
            )
            self.chart_status_text.set_visible(False)
        return self.chart_status_text

    def _show_chart_message(self, message):
        text_artist = self._ensure_chart_message_artist()
        text_artist.set_text(message)
        text_artist.set_visible(True)

    def _hide_chart_message(self):
        if self.chart_status_text is not None:
            self.chart_status_text.set_visible(False)

    def _update_chart_line(self, key, times, values, label, color, linestyle='-', alpha=1.0, drawstyle='default'):
        if len(times) == 0 or len(values) == 0:
            return None
        line = self.chart_lines.get(key)
        if line is None:
            line, = self.ax.plot([], [])
            self.chart_lines[key] = line
        line.set_data(times, values)
        line.set_color(color)
        line.set_linestyle(linestyle)
        line.set_alpha(alpha)
        line.set_label(label)
        line.set_drawstyle(drawstyle)
        line.set_visible(True)
        return line

    def _hide_unused_chart_lines(self, active_keys):
        if not self.chart_lines:
            return
        for key, line in self.chart_lines.items():
            if key not in active_keys:
                line.set_visible(False)

    def _remove_chart_legend(self):
        legend = self.ax.get_legend()
        if legend is not None:
            legend.remove()

    def _apply_chart_limits(self, values):
        """Apply custom zoom/offset to Y-axis based on plotted values."""
        finite_values = [float(v) for v in values if isinstance(v, (int, float)) and np.isfinite(v)]
        if not finite_values:
            return

        ymin = min(finite_values)
        ymax = max(finite_values)
        if abs(ymax - ymin) < 1e-6:
            margin = max(1.0, abs(ymin) * 0.1 + 1.0)
            ymin -= margin
            ymax += margin

        span = ymax - ymin
        center = (ymax + ymin) / 2.0
        offset = self.chart_y_offset * span
        half_span = max(span / 2.0 * self.chart_y_scale, 0.1)
        self.ax.set_ylim(center - half_span + offset, center + half_span + offset)

    def _get_time_window_indices(self):
        total = len(self.time_data)
        if total == 0:
            return 0, 0
        window_fraction = clamp(self.chart_x_zoom, 0.05, 1.0)
        window_size = max(2, int(total * window_fraction))
        window_size = min(window_size, total)
        max_start = total - window_size
        start = 0
        if max_start > 0:
            start = int(round(clamp(self.chart_x_pan, 0.0, 1.0) * max_start))
        end = start + window_size
        return start, end

    def _schedule_chart_update(self):
        """Throttle chart updates with debouncing."""
        if self._chart_update_pending:
            return
        current_time = time.time() * 1000  # milliseconds
        time_since_last = current_time - self._last_chart_update_time
        if time_since_last < self._chart_update_delay_ms:
            # Schedule for later
            delay_ms = int(self._chart_update_delay_ms - time_since_last)
            self._chart_update_pending = True
            self.root.after(delay_ms, self._execute_chart_update)
        else:
            self._execute_chart_update()

    def _execute_chart_update(self):
        """Execute the actual chart update."""
        self._chart_update_pending = False
        self._last_chart_update_time = time.time() * 1000
        self.update_chart()

    def _on_y_zoom_slider(self, value):
        try:
            val = float(value)
        except (TypeError, ValueError):
            return
        self.chart_y_scale = clamp(val, 0.2, 5.0)
        if hasattr(self, 'y_zoom_value_label'):
            self.y_zoom_value_label.config(text=f"{self.chart_y_scale:.2f}×")
        self.status_var.set(f"Chart zoom: {self.chart_y_scale:.2f}×")
        self._schedule_chart_update()

    def _on_y_pan_slider(self, value):
        try:
            val = float(value)
        except (TypeError, ValueError):
            return
        self.chart_y_offset = clamp(val, -3.0, 3.0)
        if hasattr(self, 'y_pan_value_label'):
            self.y_pan_value_label.config(text=f"{self.chart_y_offset:.2f}")
        self._schedule_chart_update()

    def _on_x_zoom_slider(self, value):
        try:
            val = float(value)
        except (TypeError, ValueError):
            return
        self.chart_x_zoom = clamp(val, 0.1, 1.0)
        if hasattr(self, 'x_zoom_value_label'):
            self.x_zoom_value_label.config(text=f"{self.chart_x_zoom*100:.0f}%")
        self._schedule_chart_update()

    def _on_x_pan_slider(self, value):
        try:
            val = float(value)
        except (TypeError, ValueError):
            return
        self.chart_x_pan = clamp(val, 0.0, 1.0)
        if hasattr(self, 'x_pan_value_label'):
            self.x_pan_value_label.config(text=f"{self.chart_x_pan*100:.0f}%")
        self._schedule_chart_update()

    def _sync_chart_control_vars(self):
        if hasattr(self, 'y_zoom_var'):
            self.y_zoom_var.set(self.chart_y_scale)
        if hasattr(self, 'y_zoom_value_label'):
            self.y_zoom_value_label.config(text=f"{self.chart_y_scale:.2f}×")
        if hasattr(self, 'y_pan_var'):
            self.y_pan_var.set(self.chart_y_offset)
        if hasattr(self, 'y_pan_value_label'):
            self.y_pan_value_label.config(text=f"{self.chart_y_offset:.2f}")
        if hasattr(self, 'x_zoom_var'):
            self.x_zoom_var.set(self.chart_x_zoom)
        if hasattr(self, 'x_zoom_value_label'):
            self.x_zoom_value_label.config(text=f"{self.chart_x_zoom*100:.0f}%")
        if hasattr(self, 'x_pan_var'):
            self.x_pan_var.set(self.chart_x_pan)
        if hasattr(self, 'x_pan_value_label'):
            self.x_pan_value_label.config(text=f"{self.chart_x_pan*100:.0f}%")

    def toggle_chart_pause(self):
        """Toggle chart rendering without stopping telemetry collection."""
        self.chart_paused = not self.chart_paused
        if self.chart_paused:
            self.chart_pause_btn.config(text="▶ Resume Chart")
            self.status_var.set("Chart updates paused")
        else:
            self.chart_pause_btn.config(text="⏸ Pause Chart")
            self.status_var.set("Chart updates resumed")
            self._schedule_chart_update()

    def update_chart(self):
        """Update the chart display."""
        try:
            if self.chart_paused:
                return
            
            selected_metrics = [key for key, var in self.chart_metrics.items() if var.get()]
            if not selected_metrics:
                self._hide_unused_chart_lines(set())
                self._remove_chart_legend()
                self._show_chart_message('Select at least one metric')
                self.canvas.draw_idle()
                return

            if len(self.time_data) == 0:
                self._hide_unused_chart_lines(set())
                self._remove_chart_legend()
                self._show_chart_message('Waiting for data...')
                self.canvas.draw_idle()
                return

            start_idx, end_idx = self._get_time_window_indices()
            if end_idx - start_idx < 2:
                start_idx = max(0, len(self.time_data) - 2)
                end_idx = len(self.time_data)
            if end_idx <= start_idx:
                self._hide_unused_chart_lines(set())
                self._remove_chart_legend()
                self._show_chart_message('Waiting for data...')
                self.canvas.draw_idle()
                return

            time_slice = self.time_data[start_idx:end_idx]
            if len(time_slice) == 0:
                self._hide_unused_chart_lines(set())
                self._remove_chart_legend()
                self._show_chart_message('Waiting for data...')
                self.canvas.draw_idle()
                return

            base_plot_times = self._time_slice_to_axis(time_slice, start_idx, end_idx)
            if len(base_plot_times) == 0:
                self._hide_unused_chart_lines(set())
                self._remove_chart_legend()
                self._show_chart_message('Waiting for data...')
                self.canvas.draw_idle()
                return

            x_start = base_plot_times[0]
            x_end = base_plot_times[-1]
            if x_end <= x_start:
                x_end = x_start + 1 / 86400.0  # add one second in Matplotlib date units
            self.ax.set_xlim(x_start, x_end)

            def _plot_times_for_length(count):
                if count <= 0:
                    return []
                if count == len(base_plot_times):
                    return base_plot_times
                return base_plot_times[-count:]

            plotted_values = []
            active_line_keys = []
            labels_lookup = {
                'current_pos': 'Position (degrees)',
                'target_vs_current': 'Position (degrees)',
                'load': 'Load / Torque (%)',
                'speed': 'Speed (°/s)',
                'temperature': 'Temperature (°C)',
                'voltage': 'Voltage (V)',
                'moving': 'Moving Flag (0/1)'
            }

            for metric in selected_metrics:
                if metric == 'target_vs_current':
                    for i in range(8):
                        if not self.servo_visible[i].get():
                            continue
                        if len(self.servo_data['current_pos'][i]) < end_idx:
                            continue
                        if len(self.servo_data['target_pos'][i]) < end_idx:
                            continue
                        current_slice = self.servo_data['current_pos'][i][start_idx:end_idx]
                        target_slice = self.servo_data['target_pos'][i][start_idx:end_idx]
                        if not current_slice:
                            continue
                        plot_times = _plot_times_for_length(len(current_slice))
                        if len(plot_times) != len(current_slice):
                            continue
                        plotted_values.extend(current_slice)
                        plotted_values.extend(target_slice)

                        cur_key = ('target_vs_current', i, 'current')
                        tgt_key = ('target_vs_current', i, 'target')
                        cur_line = self._update_chart_line(
                            cur_key, plot_times, current_slice,
                            f'S{i+1} Current', self.servo_colors[i], linestyle='-', alpha=1.0
                        )
                        tgt_line = self._update_chart_line(
                            tgt_key, plot_times, target_slice,
                            f'S{i+1} Target', self.servo_colors[i], linestyle='--', alpha=0.5
                        )
                        if cur_line:
                            active_line_keys.append(cur_key)
                        if tgt_line:
                            active_line_keys.append(tgt_key)
                else:
                    data_key = metric
                    for i in range(8):
                        if not self.servo_visible[i].get():
                            continue
                        if len(self.servo_data[data_key][i]) < end_idx:
                            continue
                        data_slice = self.servo_data[data_key][i][start_idx:end_idx]
                        if not data_slice:
                            continue
                        plot_times = _plot_times_for_length(len(data_slice))
                        if len(plot_times) != len(data_slice):
                            continue
                        plotted_values.extend(data_slice)

                        if len(selected_metrics) > 1:
                            label_prefix = labels_lookup.get(data_key, data_key).split('(')[0].strip()[:4]
                            servo_label = f'{label_prefix} S{i+1}'
                        else:
                            servo_label = f'Servo {i+1}'

                        drawstyle = 'steps-post' if data_key == 'moving' else 'default'
                        line_key = (data_key, i)
                        line = self._update_chart_line(
                            line_key, plot_times, data_slice,
                            servo_label, self.servo_colors[i], linestyle='-', alpha=1.0,
                            drawstyle=drawstyle
                        )
                        if line:
                            active_line_keys.append(line_key)

            active_line_set = set(active_line_keys)
            self._hide_unused_chart_lines(active_line_set)

            if not active_line_keys:
                self._remove_chart_legend()
                self._show_chart_message('No visible data')
                self.canvas.draw_idle()
                return

            self._hide_chart_message()

            if len(selected_metrics) == 1:
                metric = selected_metrics[0]
                self.ax.set_ylabel(labels_lookup.get(metric, metric))
                title_map = {
                    'target_vs_current': 'Target vs Current Position'
                }
                self.ax.set_title(title_map.get(metric, labels_lookup.get(metric, metric)))
            else:
                self.ax.set_ylabel('Multiple Metrics')
                self.ax.set_title('Multi-Metric View')

            if 'moving' in selected_metrics and len(selected_metrics) == 1:
                self.ax.set_ylim(-0.2, 1.2)
            else:
                self._apply_chart_limits(plotted_values)

            self.ax.margins(x=0, y=0.1)

            handles = []
            labels = []
            for key in active_line_keys:
                line = self.chart_lines.get(key)
                if not line or not line.get_visible():
                    continue
                label = line.get_label()
                if not label or label.startswith('_'):
                    continue
                handles.append(line)
                labels.append(label)
            if handles:
                self.ax.legend(handles, labels, loc='upper right', fontsize=8, ncol=2)
            else:
                self._remove_chart_legend()

            self.canvas.draw_idle()
        
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
        self.time_axis_data = []
        for key in self.servo_data:
            for idx in range(8):
                self.servo_data[key][idx] = []
        self.start_time = time.time()
        self.chart_y_scale = 1.1
        self.chart_y_offset = 0.0
        self.chart_x_zoom = 1.0
        self.chart_x_pan = 0.0
        self._sync_chart_control_vars()
        self._schedule_chart_update()
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

            with self.feedback_lock:
                self.latest_goal_positions = list(positions)
                for idx, value in enumerate(self.latest_goal_positions):
                    if idx < len(self.feedback_data['goal']):
                        self.feedback_data['goal'][idx] = value
            
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
    
    def set_all_speeds(self):
        """Set speed for all finger controls."""
        try:
            speed = int(self.global_speed_var.get())
            speed = clamp(speed, 1, 6)
            for finger in self.fingers:
                finger.speed_var.set(speed)
            self.status_var.set(f"Speed set to {speed} for all fingers")
        except (ValueError, AttributeError):
            pass
    
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
            target_snapshot = list(positions)
            snapshot_tuple = tuple(target_snapshot)

            # Apply positions to fingers (keep current speeds)
            for idx, finger in enumerate(self.fingers):
                pos1 = positions[idx * 2]
                pos2 = positions[idx * 2 + 1]
                finger.set_positions(pos1, pos2)

            # Trigger position update
            self.update_pending = True
            self.send_positions()
            pose_id = self._log_pose_start(selected_name, snapshot_tuple)
        
            self.status_var.set(f"Set pose '{selected_name}'")
            # Calculate delay: estimate movement time based on max position change
            # Speed ranges 1-6, assume ~200ms per 10° at speed 3
            max_movement = max(abs(positions[i] - self.fingers[i//2].get_positions()[i%2]) for i in range(8))
            avg_speed = sum(f.get_speed() for f in self.fingers) / len(self.fingers)
            # Base delay + movement-dependent delay (slower speeds need more time)
            delay_ms = int(500 + (max_movement / 10) * (200 / avg_speed) * 3)
            self.root.after(delay_ms, lambda pid=pose_id, tgt=snapshot_tuple: self._log_pose_completion(selected_name, tgt, pose_id=pid))
        
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
            
            main_container = ttk.Frame(dialog, padding=3)
            main_container.pack(fill='both', expand=True)
            
            # Two-panel layout
            paned = tk.PanedWindow(main_container, orient='horizontal', **self.splitter_config)
            paned.pack(fill='both', expand=True)
            
            # Left panel - Saved sequences list
            left_panel = ttk.Frame(paned)
            paned.add(left_panel)
            
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
            paned.add(right_panel)
            paned.paneconfig(right_panel, stretch='always', minsize=400)
            
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
            poses_frame = ttk.LabelFrame(cols, text="Available Poses", padding=3)
            poses_frame.pack(side='left', fill='both', expand=True, padx=(0,5))
            
            poses_listbox = tk.Listbox(poses_frame, height=15)
            poses_listbox.pack(fill='both', expand=True)
            attach_tooltip(poses_listbox, "Available poses. Double-click to add to sequence.")
            for pose_name in sorted(poses.keys()):
                poses_listbox.insert(tk.END, pose_name)
            
            # Sequence being built
            builder_frame = ttk.LabelFrame(cols, text="Sequence Steps", padding=3)
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
        pose_positions = pose.get('positions', [0]*8)
        for idx, finger in enumerate(self.fingers):
            pos1 = pose_positions[idx * 2]
            pos2 = pose_positions[idx * 2 + 1]
            finger.set_positions(pos1, pos2)
            finger.speed_var.set(pose['speed'])
        
        # Trigger position update
        self.update_pending = True
        self.send_positions()
        self.status_var.set(f"Executing: {name}")
        pose_id = self._log_pose_start(name, pose_positions)
        # Wait for servos to reach target (using pose speed)
        self.root.after(2000, lambda pid=pose_id, tgt=tuple(pose_positions): self._log_pose_completion(name, tgt, pose_id=pid))
    
    def _apply_pose_from_config(self, pose_data, name, speeds=None):
        """Apply a pose from YAML config format."""
        positions = list(pose_data.get('positions', [0]*8))
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
        pose_id = self._log_pose_start(name, positions)
        # Wait for servos to reach target
        self.root.after(2000, lambda pid=pose_id, tgt=tuple(positions): self._log_pose_completion(name, tgt, pose_id=pid))

    def _read_moving_flags(self):
        """Return boolean moving flags for all servos if supported."""
        if not self.connected or self.controller is None:
            return None

        if not self._moving_flags_supported:
            return None

        servo_ids = list(range(1, 9))
        controller = self.controller

        if self._moving_use_sync is None:
            self._moving_use_sync = hasattr(controller, 'sync_read_moving')

        raw = None
        last_exc = None
        attempts = []
        if self._moving_use_sync:
            attempts.append('sync')
        attempts.append('single')

        for mode in attempts:
            try:
                if mode == 'sync':
                    raw = controller.sync_read_moving(servo_ids)
                else:
                    if not hasattr(controller, 'read_moving'):
                        last_exc = RuntimeError("Controller lacks read_moving API")
                        break
                    raw = [controller.read_moving(sid) for sid in servo_ids]
                last_exc = None
                break
            except Exception as exc:
                last_exc = exc
                if mode == 'sync':
                    self._moving_use_sync = False
                    if not self._moving_sync_warning_logged:
                        self.log("sync_read_moving timed out; falling back to per-servo reads")
                        self._moving_sync_warning_logged = True
                else:
                    break

        if raw is None:
            self._moving_failure_count += 1
            if self._moving_failure_count >= 3:
                if self._moving_flags_supported:
                    msg = "Disabling movement-flag supervision after repeated read errors"
                    self.log(msg)
                self._moving_flags_supported = False
            else:
                if last_exc is not None:
                    self.log(f"Error reading moving flags: {last_exc}")
            return None

        self._moving_failure_count = 0

        def to_list(value):
            if isinstance(value, np.ndarray):
                return value.tolist()
            if isinstance(value, (list, tuple)):
                return list(value)
            return [value]

        values = None
        if isinstance(raw, dict):
            values = [raw.get(sid, 0) for sid in servo_ids]
        else:
            raw_list = to_list(raw)
            mapped = False
            if len(raw_list) == 2 and isinstance(raw_list[0], (list, tuple)) and isinstance(raw_list[1], (list, tuple)):
                ids_candidate = list(raw_list[0])
                vals_candidate = list(raw_list[1])
                if len(ids_candidate) == len(vals_candidate):
                    mapping = {}
                    for sid, val in zip(ids_candidate, vals_candidate):
                        try:
                            mapping[int(sid)] = val
                        except Exception:
                            continue
                    values = [mapping.get(sid, 0) for sid in servo_ids]
                    mapped = True
            if not mapped:
                if all(isinstance(item, (list, tuple)) and len(item) == 2 for item in raw_list):
                    mapping = {}
                    for sid, val in raw_list:
                        try:
                            mapping[int(sid)] = val
                        except Exception:
                            continue
                    values = [mapping.get(sid, 0) for sid in servo_ids]
                else:
                    values = raw_list

        if values is None:
            values = [0] * len(servo_ids)
        if len(values) < len(servo_ids):
            values = list(values) + [0] * (len(servo_ids) - len(values))
        values = values[:len(servo_ids)]

        def as_bool(entry):
            if isinstance(entry, np.ndarray):
                try:
                    entry = entry.item()
                except ValueError:
                    data = entry.tolist()
                    entry = data[0] if isinstance(data, list) and data else 0
            if isinstance(entry, (list, tuple)):
                entry = entry[-1]
            try:
                return bool(int(entry))
            except Exception:
                return bool(entry)

        return [as_bool(v) for v in values]

    def _log_pose_start(self, name, target_positions):
        """Log the beginning of a pose application and return a unique pose id."""
        self.pose_log_counter += 1
        pose_id = self.pose_log_counter
        positions = tuple(target_positions)
        target_repr = '[' + ', '.join(str(int(p)) for p in positions) + ']'
        self.log(f"Pose #{pose_id} start '{name}' → target={target_repr}")
        return pose_id

    def _read_actual_positions(self):
        """Return the latest measured servo positions, if available."""
        with self.actual_pos_lock:
            if self.latest_actual_positions is not None:
                return list(self.latest_actual_positions)

        if not self.connected or self.controller is None:
            return None

        readings = []
        try:
            for servo_id in range(1, 9):
                pos = self.controller.read_present_position(servo_id)
                if isinstance(pos, (np.ndarray, list)):
                    if isinstance(pos, np.ndarray):
                        pos = pos.item()
                    else:
                        pos = pos[0] if pos else 0
                deg = float(np.rad2deg(pos))
                if servo_id % 2 == 0:
                    deg = -deg
                readings.append(round(float(deg), 2))
            with self.actual_pos_lock:
                self.latest_actual_positions = list(readings)
                self.latest_actual_timestamp = time.time()
            return readings
        except Exception:
            return None

    def _log_pose_completion(self, name, target_positions, pose_id=None, wait_started=None, timeout=None, wait_notified=False):
        """Write target vs. actual servo positions to the log."""
        target_positions = tuple(target_positions)
        now = time.time()
        if wait_started is None:
            wait_started = now
        if timeout is None:
            timeout = self.movement_timeout

        moving_flags = self._read_moving_flags()
        if moving_flags is not None and any(moving_flags):
            if (now - wait_started) < timeout:
                if not wait_notified:
                    if pose_id is not None:
                        self.log(f"⌛ Pose #{pose_id} '{name}': waiting for servos before logging...")
                    else:
                        self.log(f"⌛ Waiting for servos to finish '{name}' before logging...")
                    wait_notified = True
                delay_ms = int(self.movement_poll_interval * 1000)
                self.root.after(
                    delay_ms,
                    lambda n=name, t=target_positions, pid=pose_id, ws=wait_started, to=timeout, wn=wait_notified: self._log_pose_completion(n, t, pid, ws, to, wn)
                )
                return
            if pose_id is not None:
                self.log(f"⚠ Pose #{pose_id} '{name}' timed out after {timeout:.1f}s waiting for servos to stop")
            else:
                self.log(f"⚠ WARNING: Timed out after {timeout:.1f}s waiting for servos to stop for '{name}'")

        target_repr = '[' + ', '.join(str(p) for p in target_positions) + ']'
        actual_positions = self._read_actual_positions()
        if actual_positions is None:
            actual_repr = '<not connected>'
        else:
            actual_repr = '[' + ', '.join(f"{float(p):.2f}" for p in actual_positions) + ']'
            if actual_positions:
                max_error = max(
                    abs(target_positions[i] - actual_positions[i])
                    for i in range(min(len(target_positions), len(actual_positions)))
                )
                if max_error > 5:
                    if pose_id is not None:
                        self.log(f"⚠ Pose #{pose_id} '{name}': Servos did not reach target! Max error: {max_error:.0f}°")
                    else:
                        self.log(f"⚠ WARNING: Servos did not reach target! Max error: {max_error:.0f}°")
                    self.log("  This may indicate: mechanical limits, servo configuration limits, or insufficient torque")
        if pose_id is not None:
            self.log(f"Pose #{pose_id} '{name}' complete → target={target_repr} current={actual_repr}")
        else:
            self.log(f"Pose '{name}' complete → target={target_repr} current={actual_repr}")
    
    def log(self, message):
        """Add message to log output with timestamp."""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]  # HH:MM:SS.mmm
        console_message = f"[{timestamp}] {message}"
        print(console_message, flush=True)
        self.log_text.config(state='normal')
        self.log_text.insert(tk.END, console_message + "\n")
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

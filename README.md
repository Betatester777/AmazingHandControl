## AmazingHand Python Tools

Python GUI and command-line tools to control the AmazingHand robot using Feetech SCS0009 servos via a serial bus controller (e.g. Waveshare USB adapter).

This project is designed for the [AmazingHand](https://github.com/pollen-robotics/AmazingHand) by Pollen Robotics.

### Requirements

- Python 3.9 or newer (Tkinter must be included for the GUI)
- External 5 V power supply for the eight servos
- USB serial bus adapter and driver installed on your computer
- Python packages listed in `requirements.txt`

Install the dependencies once (same on Windows, Linux, macOS):

```bash
python -m pip install -r requirements.txt
```

On Windows you can also use:

```bash
py -3 -m pip install -r requirements.txt
```

### Default Serial Port

Both the GUI and CLI try to choose a sensible default serial port:

- Windows: `COM9`
- Linux/macOS: `/dev/ttyACM0`

You can override this with the `--port` option, e.g. `python amazing_hand_gui.py --port COM4`.

---

### Running the GUI (`amazing_hand_gui.py`)

```bash
python amazing_hand_gui.py
```

Features:
- Per-finger sliders for open/close and left/right
- Speed selection (1-6)
- Keyboard shortcuts for quick precise movements
- Pose and sequence management using `data/hand_config.yaml`
- Live servo telemetry charts (position, load, temperature, voltage)

#### Keyboard Controls

- **1-4**: Select finger (Pointer, Middle, Ring, Thumb)
- **Arrow Keys**: Move selected finger
  - Up/Down: Close/Open
  - Left/Right: Move laterally
- **Modifiers**:
  - Normal: 1° per keypress (precise)
  - Shift: 5° per keypress (normal)
  - Ctrl: 10° per keypress (fast)
- **Quick Actions**:
  - Q: Fully close selected finger
  - E: Fully open selected finger
  - C: Center left/right position

#### Poses and Sequences

All poses and sequences are stored in `data/hand_config.yaml`:

**Saving Poses:**
1. Position fingers using sliders or keyboard
2. Enter a name in the "Name:" field
3. Set speed (1-6)
4. Click "Save Pose"

**Loading Poses:**
- Select from dropdown and click "Set Pose"

**Sequence Management:**
1. Click "Execute Pose Seq" to open the sequence manager
2. **Saved Sequences** (left panel):
   - View all saved sequences
   - Double-click or click "Execute" to run
   - Click "Edit" to load into builder for modification
   - Click "Delete" to remove
3. **Sequence Builder** (right panel):
   - Double-click poses from "Available Poses" to add
   - Enter delay (seconds) and click "Add →" or "Delay"
   - Use ↑/↓ to reorder steps
   - Check "Loop" for continuous playback
   - Enter sequence name and click "Save Sequence"
   - Click "Execute" to run without saving

**YAML Format Example:**

```yaml
poses:
  open:
    positions: [0, 0, 0, 0, 0, 0, 0, 0]
    speed: 3
  close:
    positions: [110, 110, 110, 110, 110, 110, 110, 110]
    speed: 3

sequences:
  demo:
    loop: false
    steps:
      - open|2.0s
      - close|2.0s
      - SLEEP:1.0s
  wave:
    loop: true
    steps:
      - greeting|1.5s
      - open|1.5s
```

The GUI automatically creates `data/hand_config.yaml` if it doesn't exist.

---

### Running the CLI (`amazing_hand_cmd.py`)

Basic examples:

```bash
# Set servo 1 to 45 degrees at speed 6
python amazing_hand_cmd.py --id 1 --position 45 --speed 6

# Set multiple servos synchronously
python amazing_hand_cmd.py --id 1 2 3 4 --position 10 20 30 40

# Set all 8 servos to 0 degrees and enable torque
python amazing_hand_cmd.py --all --position 0 --enable

# Read current positions
python amazing_hand_cmd.py --all --read

# Print all positions on one line (useful for scripts)
python amazing_hand_cmd.py --print
```

#### Interactive Mode

Start an interactive shell to control individual servos:

```bash
python amazing_hand_cmd.py --interactive
```

Available interactive commands:

- `set <servo_id> <position_deg> [speed]` - Set servo position
- `read <servo_id>` - Read current position
- `enable <servo_id> [1|0]` - Enable/disable torque
- `quit` / `exit` / `q` - Exit interactive mode

---

### Servo ID Configuration

Tutorial for configuring servo IDs with Feetech software and the serial bus driver:
<https://www.robot-maker.com/forum/tutorials/article/168-brancher-et-controler-le-servomoteur-feetech-sts3032-360/>

Feetech software download link:
<https://github.com/Robot-Maker-SAS/FeetechServo/tree/main/feetech%20debug%20tool%20master/FD1.9.8.2)>

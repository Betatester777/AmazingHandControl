## AmazingHand Python Tools

Python GUI and command-line tools to control the AmazingHand robot using Feetech SCS0009 servos via a serial bus controller (e.g. Waveshare USB adapter).

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
- Speed selection (16)
- Keyboard shortcuts for quick precise movements
- Scene loading/saving using `data/poses.csv`

The GUI automatically creates the `data/` folder if needed and uses `data/poses.csv` for named poses.

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

#### Scenes (`data/poses.csv`)

`amazing_hand_cmd.py` can execute named scenes from a CSV file. Example format:

```csv
scene,pos1,pos2,pos3,pos4,pos5,pos6,pos7,pos8,speed
open,0,0,0,0,0,0,0,0,6
close,90,90,90,90,90,90,90,90,3
ok,0,0,90,90,0,0,0,0,4
```

Example usage:

```bash
# Execute named scenes
python amazing_hand_cmd.py --scenes data/poses.csv --keys open close ok --enable

# Override scene speed per call
python amazing_hand_cmd.py --scenes data/poses.csv --keys "open:3" "close:6" --enable

# Add current hand pose as a new scene
python amazing_hand_cmd.py --add-scene data/poses.csv mypose --speed 3
```

#### Sequences from CSV

You can run movement sequences from a CSV file such as `example_sequence.csv`:

```csv
id,hint,pos1,pos2,pos3,pos4,pos5,pos6,pos7,pos8,speed,sleep_before,sleep_after
1,Open hand,0,0,0,0,0,0,0,0,6,0,1.0
2,Close hand,90,90,90,90,90,90,90,90,3,0,0.5
```

Run a sequence:

```bash
python amazing_hand_cmd.py --sequence example_sequence.csv --enable

# Loop continuously
python amazing_hand_cmd.py --sequence example_sequence.csv --loop --enable
```

#### Interactive Mode

Start an interactive shell to control individual servos:

```bash
python amazing_hand_cmd.py --interactive
```

Available interactive commands:

- `set <servo_id> <position_deg> [speed]`  Set servo position
- `read <servo_id>`  Read current position
- `enable <servo_id> [1|0]`  Enable/disable torque
- `quit` / `exit` / `q`  Exit interactive mode

---

### Servo ID Configuration

Tutorial for configuring servo IDs with Feetech software and the serial bus driver:
<https://www.robot-maker.com/forum/tutorials/article/168-brancher-et-controler-le-servomoteur-feetech-sts3032-360/>

Feetech software download link:
<https://github.com/Robot-Maker-SAS/FeetechServo/tree/main/feetech%20debug%20tool%20master/FD1.9.8.2)>

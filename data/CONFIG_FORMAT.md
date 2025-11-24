# Hand Configuration Format (YAML)

This document describes the YAML configuration format for AmazingHand poses and sequences.

## File Location

`data/hand_config.yaml` - All poses and sequences are stored in this file.

## YAML Structure

```yaml
poses:
  <pose_name>:
    positions: [pos1, pos2, pos3, pos4, pos5, pos6, pos7, pos8]

sequences:
  <sequence_name>:
    steps:
      - "<pose_name>:speed1,speed2,...,speed8|delay"
      - "SLEEP:duration"
```

## Poses

Each pose defines a complete hand position with 8 servo values.

### Format
```yaml
poses:
  pose_name:
    positions: [0, 0, 0, 0, 0, 0, 0, 0]
```

### Position Array
- **8 values** representing servo positions in degrees
- **Servo mapping**:
  - Servo 1: Pointer finger position (0=open, 110=closed)
  - Servo 2: Pointer finger side (-20=left, 0=center, +20=right)
  - Servo 3: Middle finger position
  - Servo 4: Middle finger side
  - Servo 5: Ring finger position
  - Servo 6: Ring finger side
  - Servo 7: Thumb position
  - Servo 8: Thumb side

- **Range**: 0-110° for position servos, -20 to +20° for side servos
- **Note**: Even-numbered servos (2,4,6,8) have inverted angles in hardware

### Naming Rules
- Letters, numbers, underscores allowed
- **Forbidden characters**: `: { } [ ] , & * # ? | - < > = ! % @ \` " '`
- Maximum 50 characters
- Case-sensitive

### Example
```yaml
poses:
  open:
    positions: [0, 0, 0, 0, 0, 0, 0, 0]
  close:
    positions: [110, 0, 110, 0, 110, 0, 110, 0]
  peace:
    positions: [0, 0, 110, 0, 0, 0, 0, 0]
```

## Sequences

Sequences define multi-step animations with individual servo speeds and delays.

### Format
```yaml
sequences:
  sequence_name:
    steps:
      - "pose_name:speed1,speed2,speed3,speed4,speed5,speed6,speed7,speed8|delay"
      - "SLEEP:duration"
```

### Step Format

**Pose with individual speeds and delay:**
```
"pose_name:s1,s2,s3,s4,s5,s6,s7,s8|delay"
```
- `pose_name`: Name of the pose to execute
- `s1-s8`: Individual speed for each servo (1-6, where 6 is fastest)
- `delay`: Time to wait after movement completes (e.g., `2.0s`)

**Pose with default speeds:**
```
"pose_name:3,3,3,3,3,3,3,3|2.0s"
```

**Sleep/pause:**
```
"SLEEP:1.5s"
```
- Pauses for specified duration without moving servos

### Speed Values
- Range: 1 (slowest) to 6 (fastest)
- Controls servo movement speed
- Each servo can have different speed in a step

### Loop Control
- Loop setting is **NOT** stored in YAML
- Controlled via checkbox in GUI sequence player
- Allows flexible playback without editing YAML

### Example
```yaml
sequences:
  demo:
    steps:
      - "open:3,3,3,3,3,3,3,3|2.0s"
      - "close:3,3,3,3,3,3,3,3|2.0s"
      - "open:3,3,3,3,3,3,3,3|1.0s"
  
  wave:
    steps:
      - "open:5,5,5,5,5,5,5,5|0.5s"
      - "SLEEP:0.3s"
      - "close:5,5,5,5,5,5,5,5|0.5s"
      - "SLEEP:0.3s"
```

## Managing Poses and Sequences

### Via GUI (`amazing_hand_gui.py`)

**Poses:**
1. Position fingers using sliders or keyboard
2. Enter name in "Name:" field
3. Click "➕ Add New" to save

**Sequences:**
1. Click "Manage" button in Sequence Player section
2. Build sequence in dialog:
   - Select poses and speeds
   - Add delays between steps
   - Reorder with ↑/↓ buttons
3. Enter sequence name and click "💾 Save"

**Execution:**
- Select sequence from dropdown
- Check "Loop" if continuous playback desired
- Click "▶ Play"

### Via CLI (`amazing_hand_cmd.py`)

**Execute a pose:**
```bash
python3 amazing_hand_cmd.py --config data/hand_config.yaml --pose open --enable
```

**Execute a sequence:**
```bash
python3 amazing_hand_cmd.py --sequence demo --enable
```

**Execute with loop:**
```bash
python3 amazing_hand_cmd.py --sequence wave --loop --enable
```

**Add current position as new pose:**
```bash
python3 amazing_hand_cmd.py --add-pose mypose
```

## Manual Editing

You can edit `data/hand_config.yaml` directly:

1. **Follow YAML syntax** - Indentation must be consistent (2 or 4 spaces)
2. **Use inline array format** for positions:
   ```yaml
   positions: [0, 0, 0, 0, 0, 0, 0, 0]
   ```
3. **Quote sequence steps** to preserve special characters:
   ```yaml
   steps:
     - "open:3,3,3,3,3,3,3,3|2.0s"
   ```
4. **Validate names** - Avoid forbidden characters
5. **Restart GUI** to reload changes
6. **Keep backups** before major edits

## Validation

The GUI and CLI automatically validate:
- Pose/sequence names (forbidden characters)
- YAML syntax on save
- Position array length (must be 8)

Invalid names will be rejected with error message showing forbidden characters.

## License

Copyright 2025 AmazingHand Control Contributors

Licensed under the Apache License, Version 2.0


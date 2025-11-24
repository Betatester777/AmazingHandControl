# Hand Configuration Format (YAML)

This file stores all poses and sequences for the AmazingHand.

## Structure

```yaml
poses:
  <pose_name>:
    positions: [pos1, pos2, pos3, pos4, pos5, pos6, pos7, pos8]
    speed: <1-6>

sequences:
  <sequence_name>:
    loop: <true/false>
    steps:
      - <pose_name>|<delay>s
      - SLEEP:<delay>s
      - <pose_name>
```

## Positions

- 8 values representing servo positions in degrees
- Servos: [1, 2, 3, 4, 5, 6, 7, 8]
- Fingers: [Pointer(1,2), Middle(3,4), Ring(5,6), Thumb(7,8)]
- Range: typically 0-110° (open to closed)
- Negative values and >110° are allowed but use caution

## Speed

- Range: 1 (slowest) to 6 (fastest)
- Affects how quickly servos move to target position

## Sequence Steps

### Pose with delay
```yaml
- open|2.0s
```
Moves to "open" pose, waits 2.0 seconds

### Pose without delay
```yaml
- close
```
Moves to "close" pose, waits auto-calculated time based on speed

### Sleep/pause
```yaml
- SLEEP:1.5s
```
Pauses for 1.5 seconds without moving

## Example

```yaml
poses:
  peace:
    positions: [0, 0, 110, 110, 0, 0, 0, 0]
    speed: 4
  fist:
    positions: [110, 110, 110, 110, 110, 110, 110, 110]
    speed: 5

sequences:
  greeting:
    loop: false
    steps:
      - open|1.0s
      - peace|2.0s
      - fist|1.0s
      - open|1.0s
  
  wave:
    loop: true
    steps:
      - open|0.5s
      - SLEEP:0.3s
      - fist|0.5s
      - SLEEP:0.3s
```

## Managing via GUI

- **Poses**: Use inline controls (Name + Speed + Save button)
- **Sequences**: Click "Execute Pose Seq" button
  - Left panel: Saved sequences (Execute/Edit/Delete)
  - Right panel: Builder (drag poses, set delays, save)
  - Double-click poses to add to sequence
  - Use ↑/↓ buttons to reorder steps

## Manual Editing

You can edit `data/hand_config.yaml` directly with any text editor:
1. Follow YAML syntax (indentation matters!)
2. Restart the GUI to reload changes
3. Keep backups before major edits

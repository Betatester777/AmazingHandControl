#!/usr/bin/env python3
"""Command-line tool to control the AmazingHand servos.

Supports single moves, synchronous multi-servo moves, reading positions,
named scenes from CSV, full movement sequences, and an interactive shell.
"""
import time
import argparse
import csv
import sys
import numpy as np
from rustypot import Scs0009PyController


def get_default_port():
    """Get a sensible default serial port based on the platform.

    Windows   -> COM9
    Linux/mac -> /dev/ttyACM0
    """
    if sys.platform.startswith('win'):
        return 'COM9'
    else:
        return '/dev/ttyACM0'


def create_controller(port=None, baudrate=1000000, timeout=0.5):
    """Create and return a servo controller."""
    if port is None:
        port = get_default_port()
    return Scs0009PyController(
        serial_port=port,
        baudrate=baudrate,
        timeout=timeout,
    )


def set_positions_sync(controller, servo_ids, positions_deg, speeds=None, wait=True, timeout=3.0):
    """
    Set multiple servo positions synchronously.
    
    Args:
        controller: Scs0009PyController instance
        servo_ids: List of servo IDs
        positions_deg: List of target positions in degrees
        speeds: List of goal speeds (1-6, where 6 is max), or single value for all
        wait: Whether to wait for positions to be reached
        timeout: Maximum wait time in seconds
    
    Returns:
        bool: True if all positions reached (or wait=False), False on timeout
    """
    if speeds is None:
        speeds = [6] * len(servo_ids)
    elif isinstance(speeds, int):
        speeds = [speeds] * len(servo_ids)
    
    # Set speeds for all servos
    for servo_id, speed in zip(servo_ids, speeds):
        controller.write_goal_speed(servo_id, speed)
    
    # Convert degrees to radians and invert for even servo IDs
    positions_rad = []
    for servo_id, pos in zip(servo_ids, positions_deg):
        if servo_id % 2 == 0:
            positions_rad.append(np.deg2rad(-pos))  # Invert for even IDs
        else:
            positions_rad.append(np.deg2rad(pos))
    
    # Synchronously set all positions
    controller.sync_write_goal_position(servo_ids, positions_rad)
    
    if wait:
        threshold = np.deg2rad(2)  # 2 degree threshold
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            all_reached = True
            for servo_id, target_pos in zip(servo_ids, positions_rad):
                current_pos = controller.read_present_position(servo_id)
                # Convert to scalar if it's an array
                if isinstance(current_pos, np.ndarray):
                    current_pos = current_pos.item()
                if abs(current_pos - target_pos) > threshold:
                    all_reached = False
                    break
            
            if all_reached:
                return True
            
            time.sleep(0.01)
        
        print(f"Warning: Not all servos reached target positions within timeout")
        return False
    
    return True


def set_position(controller, servo_id, position_deg, speed=6, wait=True, timeout=3.0):
    """
    Set servo position.
    
    Args:
        controller: Scs0009PyController instance
        servo_id: ID of the servo
        position_deg: Target position in degrees
        speed: Goal speed (1-6, where 6 is max)
        wait: Whether to wait for position to be reached
        timeout: Maximum wait time in seconds
    """
    # Set speed
    controller.write_goal_speed(servo_id, speed)
    
    # Convert degrees to radians and invert for even servo IDs
    if servo_id % 2 == 0:
        position_rad = np.deg2rad(-position_deg)  # Invert for even IDs
    else:
        position_rad = np.deg2rad(position_deg)
    
    controller.write_goal_position(servo_id, position_rad)
    
    if wait:
        threshold = np.deg2rad(2)  # 2 degree threshold
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            current_pos = controller.read_present_position(servo_id)
            # Convert to scalar if it's an array
            if isinstance(current_pos, np.ndarray):
                current_pos = current_pos.item()
            if abs(current_pos - position_rad) <= threshold:
                return True
            time.sleep(0.01)
        
        print(f"Warning: Servo {servo_id} did not reach target position within timeout")
        return False
    
    return True


def read_position(controller, servo_id):
    """Read and return current servo position in degrees."""
    position_rad = controller.read_present_position(servo_id)
    # Convert to scalar if it's an array
    if isinstance(position_rad, np.ndarray):
        position_rad = position_rad.item()
    position_deg = np.rad2deg(position_rad)
    # Ensure we return a scalar
    if isinstance(position_deg, np.ndarray):
        position_deg = position_deg.item()
    return position_deg


def load_sequence_from_csv(csv_file):
    """
    Load sequence from CSV file.
    
    CSV format (header optional):
    id,hint,pos1,pos2,pos3,pos4,pos5,pos6,pos7,pos8,speed,sleep_before,sleep_after
    
    Each line represents one step in the sequence:
    - id: Step identifier (optional)
    - hint: Description of the step (optional)
    - pos1-pos8: Target positions for servos 1-8 in degrees
    - speed: Goal speed (1-6)
    - sleep_before: Sleep time before executing this step (seconds)
    - sleep_after: Sleep time after executing this step (seconds)
    
    Returns:
        list: List of sequence steps, each as dict with 'id', 'hint', 'positions', 'speed', 'sleep_before', 'sleep_after'
    """
    sequence = []
    
    with open(csv_file, 'r') as f:
        reader = csv.reader(f)
        
        # Skip comments and find header
        id_col = None
        hint_col = None
        pos_start = 0
        
        for row in reader:
            if not row or all(cell.strip() == '' for cell in row):
                continue
            if row[0].strip().startswith('#'):
                continue
            
            # Check if this is a header row (look for 'pos1' or 'hint' keywords)
            row_lower = [c.strip().lower() for c in row]
            if 'id' in row_lower or 'pos1' in row_lower or 'hint' in row_lower:
                # This is a header row
                id_col = row_lower.index('id') if 'id' in row_lower else None
                hint_col = row_lower.index('hint') if 'hint' in row_lower else None
                pos_start = row_lower.index('pos1') if 'pos1' in row_lower else 0
                continue  # Skip header, process data rows next
            
            # This is a data row
            try:
                # Parse id and hint if columns exist
                step_id = row[id_col].strip() if id_col is not None and len(row) > id_col else str(len(sequence) + 1)
                hint = row[hint_col].strip() if hint_col is not None and len(row) > hint_col else ""
                
                # Parse positions (8 servos)
                positions = []
                for i in range(8):
                    pos_idx = pos_start + i
                    if pos_idx < len(row):
                        positions.append(float(row[pos_idx].strip()))
                    else:
                        positions.append(0.0)  # Default to 0 if missing
                
                # Parse speed (default 6 if not specified)
                speed_idx = pos_start + 8
                speed = int(row[speed_idx].strip()) if len(row) > speed_idx and row[speed_idx].strip() else 6
                
                # Parse sleep times (default 0 if not specified)
                sleep_before_idx = pos_start + 9
                sleep_after_idx = pos_start + 10
                sleep_before = float(row[sleep_before_idx].strip()) if len(row) > sleep_before_idx and row[sleep_before_idx].strip() else 0.0
                sleep_after = float(row[sleep_after_idx].strip()) if len(row) > sleep_after_idx and row[sleep_after_idx].strip() else 0.0
                
                sequence.append({
                    'id': step_id,
                    'hint': hint,
                    'positions': positions,
                    'speed': speed,
                    'sleep_before': sleep_before,
                    'sleep_after': sleep_after
                })
            except (ValueError, IndexError) as e:
                print(f"Warning: Skipping invalid data row: {row} ({e})")
                continue
    
    return sequence


def load_scenes_from_csv(csv_file):
    """
    Load named scenes from CSV file.
    
    CSV format (header optional):
    scene,pos1,pos2,pos3,pos4,pos5,pos6,pos7,pos8,speed
    
    Each line represents a named scene/pose:
    - scene: Scene name (short, no spaces recommended)
    - pos1-pos8: Target positions for servos 1-8 in degrees
    - speed: Goal speed (1-6)
    
    Returns:
        dict: Dictionary mapping scene names to scene data
    """
    scenes = {}
    
    with open(csv_file, 'r') as f:
        reader = csv.reader(f)
        
        # Skip comments and find header
        scene_col = None
        pos_start = 0
        
        for row in reader:
            if not row or all(cell.strip() == '' for cell in row):
                continue
            if row[0].strip().startswith('#'):
                continue
            
            # Check if this is a header row
            row_lower = [c.strip().lower() for c in row]
            if 'scene' in row_lower or 'name' in row_lower or 'pos1' in row_lower:
                # This is a header row
                scene_col = row_lower.index('scene') if 'scene' in row_lower else (row_lower.index('name') if 'name' in row_lower else 0)
                pos_start = row_lower.index('pos1') if 'pos1' in row_lower else 1
                continue  # Skip header, process data rows next
            
            # This is a data row
            try:
                # Parse scene name
                scene_name = row[scene_col].strip() if scene_col is not None and len(row) > scene_col else ""
                if not scene_name:
                    continue  # Skip rows without scene name
                
                # Parse positions (8 servos)
                positions = []
                for i in range(8):
                    pos_idx = pos_start + i
                    if pos_idx < len(row):
                        positions.append(float(row[pos_idx].strip()))
                    else:
                        positions.append(0.0)  # Default to 0 if missing
                
                # Parse speed (default 6 if not specified)
                speed_idx = pos_start + 8
                speed = int(row[speed_idx].strip()) if len(row) > speed_idx and row[speed_idx].strip() else 6
                
                scenes[scene_name] = {
                    'positions': positions,
                    'speed': speed
                }
            except (ValueError, IndexError) as e:
                print(f"Warning: Skipping invalid data row: {row} ({e})")
                continue
    
    return scenes


def execute_scene_sequence(controller, scenes_dict, scene_keys, servo_ids=None, loop=False):
    """
    Execute a sequence of named scenes.
    
    Args:
        controller: Scs0009PyController instance
        scenes_dict: Dictionary of scenes from load_scenes_from_csv
        scene_keys: List of scene names to execute in order (can include numbers for sleep delays)
                   Scene names can include speed override: "scenename:speed"
        servo_ids: List of servo IDs to control (default: 1-8)
        loop: Whether to loop the sequence continuously
    """
    if servo_ids is None:
        servo_ids = list(range(1, 9))
    
    try:
        while True:
            for scene_key in scene_keys:
                # Check if this is a number (sleep duration)
                try:
                    sleep_duration = float(scene_key)
                    print(f"Sleeping for {sleep_duration}s...")
                    time.sleep(sleep_duration)
                    continue
                except ValueError:
                    pass  # Not a number, treat as scene name
                
                # Parse scene name and optional speed override
                speed_override = None
                if ':' in scene_key:
                    scene_name, speed_str = scene_key.split(':', 1)
                    try:
                        speed_override = int(speed_str)
                    except ValueError:
                        print(f"Warning: Invalid speed '{speed_str}' in '{scene_key}', using default")
                        scene_name = scene_key
                else:
                    scene_name = scene_key
                
                # Execute scene
                if scene_name not in scenes_dict:
                    print(f"Warning: Scene '{scene_name}' not found, skipping")
                    continue
                
                scene = scenes_dict[scene_name]
                
                # Use speed override if provided, otherwise use scene's default speed
                speed = speed_override if speed_override is not None else scene['speed']
                
                # Calculate timeout based on speed (slower = more time needed)
                # Speed 1 = slowest (15s), Speed 6 = fastest (3s)
                timeout = 15.0 - (speed - 1) * 2.4  # 15s at speed 1, down to 3s at speed 6
                
                # Execute movement
                print(f"Scene: {scene_name} [speed: {speed}]")
                success = set_positions_sync(
                    controller,
                    servo_ids,
                    scene['positions'],
                    speed,
                    wait=True,
                    timeout=timeout
                )
                
                if not success:
                    print(f"Warning: Scene '{scene_name}' did not complete within timeout")
            
            if not loop:
                break
    except KeyboardInterrupt:
        print("\nSequence interrupted by user")


def execute_sequence(controller, sequence, servo_ids=None, loop=False):
    """
    Execute a sequence of movements.
    
    Args:
        controller: Scs0009PyController instance
        sequence: List of sequence steps from load_sequence_from_csv
        servo_ids: List of servo IDs to control (default: 1-8)
        loop: Whether to loop the sequence continuously
    """
    if servo_ids is None:
        servo_ids = list(range(1, 9))
    
    try:
        while True:
            for step in sequence:
                step_id = step.get('id', '')
                hint = step.get('hint', '')
                
                # Sleep before
                if step['sleep_before'] > 0:
                    print(f"Step {step_id}: Waiting {step['sleep_before']}s before...")
                    time.sleep(step['sleep_before'])
                
                # Execute movement
                hint_str = f": {hint}" if hint else ""
                print(f"Step {step_id}{hint_str} [speed: {step['speed']}]")
                success = set_positions_sync(
                    controller,
                    servo_ids,
                    step['positions'],
                    speeds=step['speed'],
                    wait=True,
                    timeout=5.0
                )
                
                if not success:
                    print(f"Warning: Step {step_id} did not complete within timeout")
                
                # Sleep after
                if step['sleep_after'] > 0:
                    print(f"Step {step_id}: Waiting {step['sleep_after']}s after...")
                    time.sleep(step['sleep_after'])
            
            if not loop:
                break
            
            print(f"\nSequence completed. Looping...\n")
    
    except KeyboardInterrupt:
        print(f"\nSequence interrupted")
    
    print(f"Sequence execution completed.")


def interactive_mode(controller):
    """Interactive mode for setting positions."""
    print("\n=== Interactive Position Setting Mode ===")
    print("Commands:")
    print("  set <servo_id> <position_deg> [speed]  - Set servo position")
    print("  read <servo_id>                        - Read current position")
    print("  enable <servo_id> [1|0]                - Enable/disable torque (1=on, 0=off)")
    print("  quit / exit                            - Exit interactive mode")
    print()
    
    while True:
        try:
            cmd = input("> ").strip().split()
            if not cmd:
                continue
            
            action = cmd[0].lower()
            
            if action in ['quit', 'exit', 'q']:
                break
            
            elif action == 'set':
                if len(cmd) < 3:
                    print("Usage: set <servo_id> <position_deg> [speed]")
                    continue
                
                servo_id = int(cmd[1])
                position_deg = float(cmd[2])
                speed = int(cmd[3]) if len(cmd) > 3 else 6
                
                print(f"Setting servo {servo_id} to {position_deg}° at speed {speed}...")
                success = set_position(controller, servo_id, position_deg, speed)
                if success:
                    actual_pos = read_position(controller, servo_id)
                    print(f"Done! Current position: {actual_pos:.2f}°")
            
            elif action == 'read':
                if len(cmd) < 2:
                    print("Usage: read <servo_id>")
                    continue
                
                servo_id = int(cmd[1])
                position = read_position(controller, servo_id)
                print(f"Servo {servo_id} position: {position:.2f}°")
            
            elif action == 'enable':
                if len(cmd) < 2:
                    print("Usage: enable <servo_id> [1|0]")
                    continue
                
                servo_id = int(cmd[1])
                enable = int(cmd[2]) if len(cmd) > 2 else 1
                controller.write_torque_enable(servo_id, enable)
                status = "enabled" if enable else "disabled"
                print(f"Servo {servo_id} torque {status}")
            
            else:
                print(f"Unknown command: {action}")
        
        except KeyboardInterrupt:
            print("\nExiting...")
            break
        except Exception as e:
            print(f"Error: {e}")


def main():
    parser = argparse.ArgumentParser(
        description="Command-line tool to set servo positions for AmazingHand",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Set servo 1 to 45 degrees at speed 6
  %(prog)s --id 1 --position 45 --speed 6
  
  # Set multiple servos (synchronously)
  %(prog)s --id 1 2 --position 45 45
  
  # Set all 8 servos to 0 degrees
  %(prog)s --all --position 0 --speed 3 --enable
  
  # Set specific servos with different positions
  %(prog)s --id 1 2 3 4 --position 10 20 30 40
  
  # Read current positions of all servos
  %(prog)s --all --read
  
  # Print all current positions
  %(prog)s --print
  
  # Execute sequence from CSV file
  %(prog)s --sequence movements.csv --enable
  
    # Execute named scenes
    %(prog)s --scenes data/poses.csv --keys open close ok --enable
  
    # Add current position as new scene to file
    %(prog)s --add-scene data/poses.csv newpose --speed 3
  
  # Execute sequence in a loop
  %(prog)s --sequence movements.csv --loop --enable
  
  # Interactive mode
  %(prog)s --interactive
        """
    )
    
    default_port = get_default_port()
    parser.add_argument('--port', default=default_port,
                        help=f'Serial port (default: {default_port})')
    parser.add_argument('--baudrate', type=int, default=1000000,
                        help='Baudrate (default: 1000000)')
    parser.add_argument('--id', type=int, nargs='+',
                        help='Servo ID(s)')
    parser.add_argument('--position', type=float, nargs='+',
                        help='Target position(s) in degrees')
    parser.add_argument('--speed', type=int, nargs='+',
                        help='Goal speed (1-6, default: 6)')
    parser.add_argument('--read', action='store_true',
                        help='Read current position instead of setting')
    parser.add_argument('--no-wait', action='store_true',
                        help='Do not wait for position to be reached')
    parser.add_argument('--enable', action='store_true',
                        help='Enable torque before setting position')
    parser.add_argument('--all', action='store_true',
                        help='Apply to all 8 servos (IDs 1-8)')
    parser.add_argument('--print', action='store_true',
                        help='Print all current servo positions (IDs 1-8)')
    parser.add_argument('--sequence', type=str,
                        help='CSV file with movement sequence')
    parser.add_argument('--scenes', type=str,
                        help='CSV file with named scenes/poses')
    parser.add_argument('--keys', type=str, nargs='+',
                        help='Scene names to execute in order (use with --scenes)')
    parser.add_argument('--add-scene', type=str, nargs=2, metavar=('FILE', 'SCENE_NAME'),
                        help='Add current position as new scene to CSV file')
    parser.add_argument('--loop', action='store_true',
                        help='Loop the sequence continuously (use with --sequence or --scenes)')
    parser.add_argument('--interactive', '-i', action='store_true',
                        help='Start interactive mode')
    
    args = parser.parse_args()
    
    # Create controller
    print(f"Connecting to {args.port} at {args.baudrate} baud...")
    controller = create_controller(args.port, args.baudrate)
    print("Connected!")
    
    # Interactive mode
    if args.interactive:
        interactive_mode(controller)
        return
    
    # Print mode - show all positions
    if args.print:
        positions = []
        for servo_id in range(1, 9):
            try:
                position = read_position(controller, servo_id)
                # Negate even servo IDs to show the actual input value
                if servo_id % 2 == 0:
                    position = -position
                positions.append(f"{int(position)}")
            except Exception as e:
                positions.append("ERR")
        print(" ".join(positions))
        return
    
    # Add scene mode - save current position to CSV
    if args.add_scene:
        csv_file, scene_name = args.add_scene
        
        # Read current positions
        positions = []
        for servo_id in range(1, 9):
            try:
                position = read_position(controller, servo_id)
                # Negate even servo IDs to show the actual input value
                if servo_id % 2 == 0:
                    position = -position
                positions.append(int(position))
            except Exception as e:
                print(f"Error reading servo {servo_id}: {e}")
                return
        
        # Determine speed (from --speed arg or default to 3)
        speed = args.speed[0] if args.speed else 3
        
        # Check if file exists and has header
        import os
        file_exists = os.path.exists(csv_file)
        
        # Append to file
        with open(csv_file, 'a') as f:
            # Add header if new file
            if not file_exists:
                f.write("scene, pos1, pos2, pos3, pos4, pos5, pos6, pos7, pos8, speed\n")
            
            # Write scene data
            pos_str = ", ".join(f"{p:4d}" for p in positions)
            f.write(f"{scene_name}, {pos_str}, {speed:5d}\n")
        
        print(f"Added scene '{scene_name}' to {csv_file}")
        print(f"Positions: {' '.join(str(p) for p in positions)}")
        print(f"Speed: {speed}")
        return
    
    # Scenes mode
    if args.scenes:
        print(f"Loading scenes from {args.scenes}...")
        scenes = load_scenes_from_csv(args.scenes)
        print(f"Loaded {len(scenes)} scenes: {', '.join(scenes.keys())}")
        
        # Enable torque if requested
        if args.enable:
            for servo_id in range(1, 9):
                controller.write_torque_enable(servo_id, 1)
            print("Enabled torque for all servos")
        
        # Check if scene keys provided
        if not args.keys:
            parser.error("--keys is required when using --scenes")
        
        # Execute scene sequence
        execute_scene_sequence(controller, scenes, args.keys, loop=args.loop)
        return
    
    # Sequence mode
    if args.sequence:
        print(f"Loading sequence from {args.sequence}...")
        sequence = load_sequence_from_csv(args.sequence)
        print(f"Loaded {len(sequence)} steps")
        
        # Enable torque if requested
        if args.enable:
            for servo_id in range(1, 9):
                controller.write_torque_enable(servo_id, 1)
            print("Enabled torque for all servos")
        
        # Execute sequence
        execute_sequence(controller, sequence, loop=args.loop)
        return
    
    # Handle --all flag or single position without ID
    if args.all:
        args.id = list(range(1, 9))  # Servos 1-8
    elif not args.id and args.position and len(args.position) == 1:
        # If no ID specified but single position given, apply to all servos
        args.id = list(range(1, 9))  # Servos 1-8
    
    # Non-interactive mode
    if not args.id:
        parser.error("--id is required (or use --interactive or --all)")
    
    # Enable torque if requested
    if args.enable:
        for servo_id in args.id:
            controller.write_torque_enable(servo_id, 1)
            print(f"Enabled torque for servo {servo_id}")
    
    # Read mode
    if args.read:
        for servo_id in args.id:
            position = read_position(controller, servo_id)
            print(f"Servo {servo_id}: {position:.2f}°")
        return
    
    # Set position mode
    if not args.position:
        parser.error("--position is required for setting positions")
    
    # Prepare positions and speeds
    positions = args.position
    if len(positions) == 1 and len(args.id) > 1:
        positions = positions * len(args.id)
    elif len(positions) != len(args.id):
        parser.error(f"Number of positions ({len(positions)}) must match number of IDs ({len(args.id)})")
    
    speeds = args.speed if args.speed else [6] * len(args.id)
    if len(speeds) == 1 and len(args.id) > 1:
        speeds = speeds * len(args.id)
    elif len(speeds) != len(args.id):
        parser.error(f"Number of speeds ({len(speeds)}) must match number of IDs ({len(args.id)})")
    
    # Set positions synchronously if multiple servos, otherwise set individually
    if len(args.id) > 1:
        print(f"Setting {len(args.id)} servos synchronously...")
        for servo_id, position, speed in zip(args.id, positions, speeds):
            print(f"  Servo {servo_id}: {position}° at speed {speed}")
        
        success = set_positions_sync(controller, args.id, positions, speeds, not args.no_wait)
        
        if success:
            for servo_id in args.id:
                actual_pos = read_position(controller, servo_id)
                print(f"Servo {servo_id}: {actual_pos:.2f}°")
    else:
        # Single servo
        servo_id = args.id[0]
        position = positions[0]
        speed = speeds[0]
        print(f"Setting servo {servo_id} to {position}° at speed {speed}...")
        success = set_position(controller, servo_id, position, speed, not args.no_wait)
        if success:
            actual_pos = read_position(controller, servo_id)
            print(f"Servo {servo_id}: {actual_pos:.2f}°")


if __name__ == '__main__':
    main()

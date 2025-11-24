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
"""Command-line tool to control the AmazingHand servos.

OVERVIEW:
=========
This CLI provides scriptable control for the 8-servo robotic hand.
Works with the same YAML configuration as amazing_hand_gui.py.

CAPABILITIES:
=============
- Direct servo control: Set individual or multiple servo positions
- Pose execution: Apply saved poses from YAML config
- Sequence playback: Execute multi-step sequences with individual speeds
- Position reading: Query current servo positions
- Interactive mode: REPL for manual testing
- Pose management: Add current positions to config

USAGE MODES:
============
1. Single Servo:
   python3 amazing_hand_cmd.py --id 1 --position 45 --speed 6

2. Multiple Servos (synchronous):
   python3 amazing_hand_cmd.py --id 1 2 3 --position 45 90 0

3. Execute Pose:
   python3 amazing_hand_cmd.py --config data/hand_config.yaml --pose open --enable

4. Execute Sequence:
   python3 amazing_hand_cmd.py --sequence demo --loop --enable

5. Interactive REPL:
   python3 amazing_hand_cmd.py --interactive

6. Save Current Position:
   python3 amazing_hand_cmd.py --add-pose my_pose

SERVO MAPPING:
==============
Servos 1,3,5,7: Position control (0=open, 110=closed)
Servos 2,4,6,8: Side control (-20=left, 0=center, +20=right)
Even servos have inverted angles in hardware.

YAML FORMAT:
============
poses:
  pose_name:
    positions: [p1, p2, p3, p4, p5, p6, p7, p8]

sequences:
  sequence_name:
    steps:
      - "pose_name:speed1,speed2,...,speed8|delay"
      - "SLEEP:duration"

DEPENDENCIES:
=============
- rustypot: Servo controller library
- PyYAML: Configuration parsing
- numpy: Angle conversions
"""
import time
import argparse
import sys
import re
from pathlib import Path
import numpy as np
import yaml
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


def load_config(yaml_file):
    """Load configuration from YAML file.
    
    Returns:
        dict: Configuration with 'poses' and 'sequences' keys
    """
    yaml_path = Path(yaml_file)
    if not yaml_path.exists():
        print(f"Error: Config file not found: {yaml_file}")
        return {'poses': {}, 'sequences': {}}
    
    try:
        with yaml_path.open('r') as f:
            config = yaml.safe_load(f) or {}
            if 'poses' not in config:
                config['poses'] = {}
            if 'sequences' not in config:
                config['sequences'] = {}
            return config
    except Exception as e:
        print(f"Error loading config: {e}")
        return {'poses': {}, 'sequences': {}}


def save_config(config, yaml_file):
    """Save configuration to YAML file with inline array formatting."""
    yaml_path = Path(yaml_file)
    
    try:
        # Convert to YAML string
        yaml_str = yaml.dump(config, default_flow_style=False, sort_keys=False)
        
        # Format positions arrays to be inline
        yaml_str = re.sub(
            r'positions:\s*\n\s*-\s*([\d\s\n-]+?)(?=\n\w)',
            lambda m: 'positions: [' + ', '.join(m.group(1).replace('-', '').split()) + ']\n',
            yaml_str,
            flags=re.MULTILINE
        )
        
        # Write to file
        yaml_path.parent.mkdir(parents=True, exist_ok=True)
        with yaml_path.open('w') as f:
            f.write(yaml_str)
        return True
    except Exception as e:
        print(f"Error saving config: {e}")
        return False


def parse_sequence_step(step_str, poses_dict):
    """Parse a sequence step string.
    
    Format: pose_name:speed1,speed2,...,speed8|delay
    or: SLEEP:duration
    
    Returns:
        dict: Step data with 'type', 'pose_name', 'positions', 'speeds', 'delay'
    """
    step_str = step_str.strip()
    
    # Check for SLEEP command
    if step_str.upper().startswith('SLEEP:'):
        try:
            duration = float(step_str.split(':', 1)[1].rstrip('s'))
            return {'type': 'sleep', 'duration': duration}
        except (ValueError, IndexError):
            print(f"Warning: Invalid SLEEP format: {step_str}")
            return None
    
    # Parse pose step: pose_name:speed1,speed2,...,speed8|delay
    try:
        # Split on | to separate delay
        if '|' in step_str:
            step_part, delay_str = step_str.split('|', 1)
            delay = float(delay_str.rstrip('s'))
        else:
            step_part = step_str
            delay = 0.0
        
        # Split on : to separate pose name and speeds
        if ':' in step_part:
            pose_name, speeds_str = step_part.split(':', 1)
            speeds = [int(s) for s in speeds_str.split(',')]
            if len(speeds) != 8:
                print(f"Warning: Expected 8 speeds, got {len(speeds)} in: {step_str}")
                speeds = speeds + [3] * (8 - len(speeds))  # Pad with default
        else:
            pose_name = step_part
            speeds = [3] * 8  # Default speeds
        
        # Look up pose
        if pose_name not in poses_dict:
            print(f"Warning: Pose '{pose_name}' not found")
            return None
        
        return {
            'type': 'pose',
            'pose_name': pose_name,
            'positions': poses_dict[pose_name]['positions'],
            'speeds': speeds,
            'delay': delay
        }
    except Exception as e:
        print(f"Warning: Failed to parse step '{step_str}': {e}")
        return None


def execute_sequence(controller, sequence_name, config, servo_ids=None, loop=False):
    """
    Execute a named sequence from config.
    
    Args:
        controller: Scs0009PyController instance
        sequence_name: Name of sequence to execute
        config: Config dict with 'poses' and 'sequences'
        servo_ids: List of servo IDs to control (default: 1-8)
        loop: Whether to loop the sequence continuously
    """
    if servo_ids is None:
        servo_ids = list(range(1, 9))
    
    if sequence_name not in config['sequences']:
        print(f"Error: Sequence '{sequence_name}' not found")
        return
    
    sequence = config['sequences'][sequence_name]
    steps = sequence.get('steps', [])
    
    if not steps:
        print(f"Error: Sequence '{sequence_name}' has no steps")
        return
    
    try:
        iteration = 1
        while True:
            if loop and iteration > 1:
                print(f"\n=== Loop iteration {iteration} ===")
            
            for i, step_str in enumerate(steps, 1):
                step = parse_sequence_step(step_str, config['poses'])
                if not step:
                    continue
                
                if step['type'] == 'sleep':
                    print(f"  Step {i}: Sleeping {step['duration']}s...")
                    time.sleep(step['duration'])
                
                elif step['type'] == 'pose':
                    print(f"  Step {i}: {step['pose_name']} (speeds: {','.join(map(str, step['speeds']))}) delay: {step['delay']}s")
                    
                    # Set positions with individual speeds
                    success = set_positions_sync(
                        controller,
                        servo_ids,
                        step['positions'],
                        speeds=step['speeds'],
                        wait=True,
                        timeout=5.0
                    )
                    
                    if not success:
                        print(f"    Warning: Step did not complete within timeout")
                    
                    # Delay after movement
                    if step['delay'] > 0:
                        time.sleep(step['delay'])
            
            if not loop:
                break
            
            iteration += 1
    
    except KeyboardInterrupt:
        print("\nSequence interrupted")
    
    print("Sequence execution completed.")


def execute_pose(controller, pose_name, config, servo_ids=None, speed=None):
    """
    Execute a single named pose.
    
    Args:
        controller: Scs0009PyController instance
        pose_name: Name of pose to execute
        config: Config dict with 'poses' and 'sequences'
        servo_ids: List of servo IDs to control (default: 1-8)
        speed: Speed override (1-6), or None to use default
    """
    if servo_ids is None:
        servo_ids = list(range(1, 9))
    
    if pose_name not in config['poses']:
        print(f"Error: Pose '{pose_name}' not found")
        return False
    
    pose = config['poses'][pose_name]
    positions = pose['positions']
    
    # Use provided speed or default to 3
    if speed is None:
        speed = 3
    
    speeds = [speed] * 8
    
    print(f"Executing pose: {pose_name} (speed: {speed})")
    success = set_positions_sync(
        controller,
        servo_ids,
        positions,
        speeds=speeds,
        wait=True,
        timeout=5.0
    )
    
    if success:
        print(f"✓ Pose '{pose_name}' executed")
    else:
        print(f"✗ Pose '{pose_name}' did not complete within timeout")
    
    return success


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
  
  # Execute a named pose from YAML config
  %(prog)s --config data/hand_config.yaml --pose open --enable
  
  # Execute a sequence from YAML config
  %(prog)s --config data/hand_config.yaml --sequence demo --enable
  
  # Execute sequence in a loop
  %(prog)s --config data/hand_config.yaml --sequence demo --loop --enable
  
  # Add current position as new pose to YAML config
  %(prog)s --config data/hand_config.yaml --add-pose newpose
  
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
    parser.add_argument('--config', type=str, default='data/hand_config.yaml',
                        help='YAML config file (default: data/hand_config.yaml)')
    parser.add_argument('--pose', type=str,
                        help='Execute a named pose from config')
    parser.add_argument('--sequence', type=str,
                        help='Execute a named sequence from config')
    parser.add_argument('--add-pose', type=str, metavar='POSE_NAME',
                        help='Add current position as new pose to config file')
    parser.add_argument('--loop', action='store_true',
                        help='Loop the sequence continuously (use with --sequence)')
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
    
    # Add pose mode - save current position to YAML
    if args.add_pose:
        pose_name = args.add_pose
        
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
        
        # Load existing config
        config = load_config(args.config)
        
        # Add new pose
        config['poses'][pose_name] = {
            'positions': positions
        }
        
        # Save config
        if save_config(config, args.config):
            print(f"✓ Added pose '{pose_name}' to {args.config}")
            print(f"  Positions: {positions}")
        else:
            print(f"✗ Failed to save config")
        return
    
    # Pose mode
    if args.pose:
        # Load config
        config = load_config(args.config)
        
        # Enable torque if requested
        if args.enable:
            for servo_id in range(1, 9):
                controller.write_torque_enable(servo_id, 1)
            print("Enabled torque for all servos")
        
        # Execute pose
        speed = args.speed[0] if args.speed else None
        execute_pose(controller, args.pose, config, speed=speed)
        return
    
    # Sequence mode
    if args.sequence:
        # Load config
        config = load_config(args.config)
        print(f"Loading sequence '{args.sequence}' from {args.config}...")
        
        if args.sequence not in config['sequences']:
            print(f"Error: Sequence '{args.sequence}' not found in config")
            print(f"Available sequences: {', '.join(config['sequences'].keys())}")
            return
        
        # Enable torque if requested
        if args.enable:
            for servo_id in range(1, 9):
                controller.write_torque_enable(servo_id, 1)
            print("Enabled torque for all servos")
        
        # Execute sequence
        execute_sequence(controller, args.sequence, config, loop=args.loop)
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

"""
Trajectory Player Module for ODrive CAN Control System

This module provides trajectory playback capabilities by reading time-series
position data from files and executing them with precise timing.

Supported file formats:
- CSV: timestamp,position
- JSON: [{"time": t, "position": p}, ...]
- Simple text: time position (space separated)

Features:
- High-precision timing control
- Interpolation between points
- Loop playback support
- Real-time execution monitoring
- Safety limits and bounds checking
"""

import asyncio
import threading
import time
import json
import csv
from typing import List, Dict, Tuple, Optional, Union
from pathlib import Path


class TrajectoryPoint:
    """Single trajectory point with time and position"""
    
    def __init__(self, time: float, position: float):
        self.time = time
        self.position = position
    
    def __repr__(self):
        return f"TrajectoryPoint(time={self.time:.3f}, position={self.position:.3f})"


class TrajectoryData:
    """Container for trajectory data with interpolation and validation"""
    
    def __init__(self, points: List[TrajectoryPoint]):
        self.points = sorted(points, key=lambda p: p.time)  # Sort by time
        self.duration = self.points[-1].time - self.points[0].time if points else 0.0
        self.start_time = self.points[0].time if points else 0.0
        self.end_time = self.points[-1].time if points else 0.0
        
        # Calculate statistics
        positions = [p.position for p in self.points]
        self.min_position = min(positions) if positions else 0.0
        self.max_position = max(positions) if positions else 0.0
        self.position_range = self.max_position - self.min_position
    
    def interpolate_position(self, time: float) -> Optional[float]:
        """Interpolate position at given time"""
        if not self.points:
            return None
        
        # Handle edge cases
        if time <= self.points[0].time:
            return self.points[0].position
        if time >= self.points[-1].time:
            return self.points[-1].position
        
        # Find surrounding points
        for i in range(len(self.points) - 1):
            p1, p2 = self.points[i], self.points[i + 1]
            if p1.time <= time <= p2.time:
                # Linear interpolation
                if p2.time == p1.time:
                    return p1.position
                
                alpha = (time - p1.time) / (p2.time - p1.time)
                return p1.position + alpha * (p2.position - p1.position)
        
        return None
    
    def get_statistics(self) -> Dict:
        """Get trajectory statistics"""
        return {
            'points': len(self.points),
            'duration': self.duration,
            'start_time': self.start_time,
            'end_time': self.end_time,
            'min_position': self.min_position,
            'max_position': self.max_position,
            'position_range': self.position_range
        }


class TrajectoryFileReader:
    """Reads trajectory data from various file formats"""
    
    @staticmethod
    def read_csv(filepath: Path) -> TrajectoryData:
        """Read CSV file with timestamp,position columns"""
        points = []
        
        with open(filepath, 'r') as f:
            reader = csv.reader(f)
            
            # Skip header if present
            first_row = next(reader, None)
            if first_row and not first_row[0].replace('.', '').replace('-', '').isdigit():
                pass  # Header row, already skipped
            else:
                # First row is data, process it
                if first_row and len(first_row) >= 2:
                    time_val = float(first_row[0])
                    pos_val = float(first_row[1])
                    points.append(TrajectoryPoint(time_val, pos_val))
            
            # Process remaining rows
            for row in reader:
                if len(row) >= 2:
                    try:
                        time_val = float(row[0])
                        pos_val = float(row[1])
                        points.append(TrajectoryPoint(time_val, pos_val))
                    except ValueError:
                        continue  # Skip invalid rows
        
        return TrajectoryData(points)
    
    @staticmethod
    def read_json(filepath: Path) -> TrajectoryData:
        """Read JSON file with array of {time, position} objects"""
        with open(filepath, 'r') as f:
            data = json.load(f)
        
        points = []
        for item in data:
            if isinstance(item, dict) and 'time' in item and 'position' in item:
                points.append(TrajectoryPoint(float(item['time']), float(item['position'])))
        
        return TrajectoryData(points)
    
    @staticmethod
    def read_text(filepath: Path) -> TrajectoryData:
        """Read simple text file with 'time position' space-separated values"""
        points = []
        
        with open(filepath, 'r') as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('#'):  # Skip comments
                    parts = line.split()
                    if len(parts) >= 2:
                        try:
                            time_val = float(parts[0])
                            pos_val = float(parts[1])
                            points.append(TrajectoryPoint(time_val, pos_val))
                        except ValueError:
                            continue  # Skip invalid lines
        
        return TrajectoryData(points)
    
    @staticmethod
    def read_trajectory_file(filepath: Union[str, Path]) -> TrajectoryData:
        """Auto-detect file format and read trajectory data"""
        filepath = Path(filepath)
        
        if not filepath.exists():
            raise FileNotFoundError(f"Trajectory file not found: {filepath}")
        
        extension = filepath.suffix.lower()
        
        if extension == '.csv':
            return TrajectoryFileReader.read_csv(filepath)
        elif extension == '.json':
            return TrajectoryFileReader.read_json(filepath)
        elif extension in ['.txt', '.dat']:
            return TrajectoryFileReader.read_text(filepath)
        else:
            # Try to auto-detect based on content
            try:
                return TrajectoryFileReader.read_csv(filepath)
            except:
                try:
                    return TrajectoryFileReader.read_json(filepath)
                except:
                    return TrajectoryFileReader.read_text(filepath)


class TrajectoryPlayer:
    """High-precision trajectory playback system"""
    
    def __init__(self, position_controller, telemetry_manager):
        self.position_controller = position_controller
        self.telemetry_manager = telemetry_manager
        
        # Trajectory data
        self.trajectory = None
        self.loaded_file = None
        
        # Playback control
        self.playing = False
        self.paused = False
        self.loop_mode = False
        self.playback_thread = None
        self.stop_event = threading.Event()
        
        # Timing control
        self.time_scale = 1.0  # Speed multiplier
        self.start_delay = 0.0  # Delay before starting
        
        # Safety limits
        self.position_limits = (-10.0, 10.0)
        self.max_velocity_estimate = 5.0  # turns/sec
        
        # Statistics
        self.playback_stats = {
            'start_time': None,
            'points_executed': 0,
            'max_error': 0.0,
            'avg_error': 0.0,
            'loops_completed': 0
        }
    
    def load_trajectory(self, filepath: Union[str, Path]) -> bool:
        """Load trajectory from file"""
        try:
            print(f"📂 Loading trajectory from: {filepath}")
            self.trajectory = TrajectoryFileReader.read_trajectory_file(filepath)
            self.loaded_file = str(filepath)
            
            stats = self.trajectory.get_statistics()
            print(f"✅ Trajectory loaded successfully:")
            print(f"   📊 Points: {stats['points']}")
            print(f"   ⏱️  Duration: {stats['duration']:.3f} seconds")
            print(f"   📍 Position range: {stats['min_position']:.3f} to {stats['max_position']:.3f} turns")
            print(f"   📏 Total range: {stats['position_range']:.3f} turns")
            
            # Safety check
            if (stats['min_position'] < self.position_limits[0] or 
                stats['max_position'] > self.position_limits[1]):
                print(f"⚠️ WARNING: Trajectory exceeds safety limits {self.position_limits}")
                print(f"   Use set_position_limits() to adjust if needed")
                return False
            
            return True
            
        except Exception as e:
            print(f"❌ Failed to load trajectory: {e}")
            return False
    
    def set_position_limits(self, min_pos: float, max_pos: float):
        """Set safety position limits"""
        self.position_limits = (min_pos, max_pos)
        print(f"🛡️ Position limits set: {min_pos:.3f} to {max_pos:.3f} turns")
    
    def set_playback_options(self, time_scale: float = 1.0, loop_mode: bool = False, start_delay: float = 0.0):
        """Configure playback options"""
        self.time_scale = max(0.1, min(10.0, time_scale))  # Limit to reasonable range
        self.loop_mode = loop_mode
        self.start_delay = max(0.0, start_delay)
        
        print(f"⚙️ Playback options set:")
        print(f"   🏃 Speed: {self.time_scale}x")
        print(f"   🔄 Loop mode: {'ON' if self.loop_mode else 'OFF'}")
        print(f"   ⏰ Start delay: {self.start_delay:.1f}s")
    
    def start_playback(self) -> bool:
        """Start trajectory playback"""
        if not self.trajectory:
            print("❌ No trajectory loaded")
            return False
        
        if not self.position_controller.active:
            print("❌ Position controller not active")
            return False
        
        if self.playing:
            print("⚠️ Playback already running")
            return False
        
        print(f"🎬 Starting trajectory playback...")
        print(f"📂 File: {self.loaded_file}")
        
        # Reset statistics
        self.playback_stats = {
            'start_time': time.time(),
            'points_executed': 0,
            'max_error': 0.0,
            'avg_error': 0.0,
            'loops_completed': 0
        }
        
        # Start playback thread
        self.playing = True
        self.paused = False
        self.stop_event.clear()
        self.playback_thread = threading.Thread(target=self._playback_thread, daemon=True)
        self.playback_thread.start()
        
        return True
    
    def pause_playback(self):
        """Pause trajectory playback"""
        if self.playing and not self.paused:
            self.paused = True
            print("⏸️ Trajectory playback paused")
        elif self.paused:
            self.paused = False
            print("▶️ Trajectory playback resumed")
    
    def stop_playback(self):
        """Stop trajectory playback"""
        if self.playing:
            print("🛑 Stopping trajectory playback...")
            self.playing = False
            self.stop_event.set()
            
            if self.playback_thread and self.playback_thread.is_alive():
                self.playback_thread.join(timeout=2.0)
            
            print("✅ Trajectory playback stopped")
            self._print_playback_stats()
    
    def _playback_thread(self):
        """Main playback execution thread"""
        try:
            # Initial delay
            if self.start_delay > 0:
                print(f"⏰ Waiting {self.start_delay:.1f}s before starting...")
                time.sleep(self.start_delay)
            
            loops = 0
            
            while self.playing and not self.stop_event.is_set():
                loop_start_time = time.time()
                trajectory_start_time = self.trajectory.start_time
                trajectory_duration = self.trajectory.duration
                
                print(f"🎬 Starting trajectory loop {loops + 1}")
                
                # Execute trajectory
                while self.playing and not self.stop_event.is_set():
                    current_time = time.time()
                    elapsed_time = current_time - loop_start_time
                    
                    # Handle pause
                    while self.paused and self.playing:
                        time.sleep(0.01)
                        loop_start_time = time.time() - elapsed_time  # Adjust start time
                        continue
                    
                    # Calculate trajectory time with scaling
                    trajectory_time = trajectory_start_time + (elapsed_time * self.time_scale)
                    
                    # Check if trajectory is complete
                    if trajectory_time >= self.trajectory.end_time:
                        break
                    
                    # Get target position
                    target_position = self.trajectory.interpolate_position(trajectory_time)
                    if target_position is not None:
                        # Send position command
                        success = self.position_controller.set_position(target_position)
                        if success:
                            self.playback_stats['points_executed'] += 1
                            
                            # Calculate tracking error
                            current_pos, _ = self.telemetry_manager.get_position_velocity()
                            if current_pos is not None:
                                error = abs(target_position - current_pos)
                                self.playback_stats['max_error'] = max(self.playback_stats['max_error'], error)
                                
                                # Running average error
                                n = self.playback_stats['points_executed']
                                self.playback_stats['avg_error'] = (
                                    (self.playback_stats['avg_error'] * (n - 1) + error) / n
                                )
                    
                    # High-frequency execution
                    time.sleep(0.01)  # 100Hz execution rate
                
                loops += 1
                self.playback_stats['loops_completed'] = loops
                
                if not self.loop_mode:
                    break
                
                print(f"🔄 Loop {loops} completed, restarting...")
            
            print(f"✅ Trajectory playback finished after {loops} loop(s)")
            
        except Exception as e:
            print(f"❌ Playback error: {e}")
        finally:
            self.playing = False
    
    def _print_playback_stats(self):
        """Print playback statistics"""
        if self.playback_stats['start_time']:
            duration = time.time() - self.playback_stats['start_time']
            print(f"\n📊 Playback Statistics:")
            print(f"   ⏱️  Duration: {duration:.2f}s")
            print(f"   📍 Points executed: {self.playback_stats['points_executed']}")
            print(f"   🔄 Loops completed: {self.playback_stats['loops_completed']}")
            print(f"   📏 Max tracking error: {self.playback_stats['max_error']:.4f} turns")
            print(f"   📊 Avg tracking error: {self.playback_stats['avg_error']:.4f} turns")
    
    def get_status(self) -> Dict:
        """Get current playback status"""
        return {
            'loaded': self.trajectory is not None,
            'loaded_file': self.loaded_file,
            'playing': self.playing,
            'paused': self.paused,
            'loop_mode': self.loop_mode,
            'time_scale': self.time_scale,
            'position_limits': self.position_limits,
            'stats': self.playback_stats.copy() if self.trajectory else None
        }


def create_trajectory_player(position_controller, telemetry_manager) -> TrajectoryPlayer:
    """Factory function to create a trajectory player"""
    return TrajectoryPlayer(position_controller, telemetry_manager)


# Example trajectory file generators for testing
def generate_example_trajectories():
    """Generate example trajectory files for testing"""
    
    # 1. Simple sine wave (CSV)
    import math
    with open('example_sine_trajectory.csv', 'w') as f:
        f.write('time,position\n')
        for i in range(1000):
            t = i * 0.01  # 10ms intervals
            pos = 2.0 * math.sin(2 * math.pi * 0.5 * t)  # 0.5 Hz sine wave
            f.write(f'{t:.3f},{pos:.6f}\n')
    
    # 2. Step response (JSON)
    steps = []
    for i in range(50):
        steps.append({'time': i * 0.1, 'position': 0.0})
    for i in range(50, 100):
        steps.append({'time': i * 0.1, 'position': 1.0})
    for i in range(100, 150):
        steps.append({'time': i * 0.1, 'position': -1.0})
    for i in range(150, 200):
        steps.append({'time': i * 0.1, 'position': 0.0})
    
    with open('example_step_trajectory.json', 'w') as f:
        json.dump(steps, f, indent=2)
    
    # 3. Ramp trajectory (text)
    with open('example_ramp_trajectory.txt', 'w') as f:
        f.write('# Time Position\n')
        for i in range(200):
            t = i * 0.05
            pos = t * 0.1  # Ramp up
            f.write(f'{t:.3f} {pos:.6f}\n')
    
    print("📝 Generated example trajectory files:")
    print("   - example_sine_trajectory.csv")
    print("   - example_step_trajectory.json") 
    print("   - example_ramp_trajectory.txt")


if __name__ == "__main__":
    # Generate example files for testing
    generate_example_trajectories()
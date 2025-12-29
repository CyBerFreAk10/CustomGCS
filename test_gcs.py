"""
Test/Demo Script for Enhanced GCS
Simulates drone telemetry without requiring real hardware or SITL
"""

import time
import math
import random

class SimulatedDrone:
    """Simulates a drone for testing the GCS"""
    
    def __init__(self, drone_object, start_lat, start_lon, name):
        self.drone = drone_object
        self.lat = start_lat
        self.lon = start_lon
        self.alt = 0
        self.heading = 0
        self.name = name
        self.armed = False
        self.mode = "STABILIZE"
        
    def simulate_flight(self, target_lat, target_lon, speed=0.00001):
        """Simulate flying to a target location"""
        print(f"\n[{self.name}] Flying to ({target_lat:.6f}, {target_lon:.6f})")
        
        while True:
            # Calculate direction
            dlat = target_lat - self.lat
            dlon = target_lon - self.lon
            
            distance = math.sqrt(dlat**2 + dlon**2)
            
            if distance < 0.00002:  # Approximately 2 meters
                print(f"[{self.name}] Target reached!")
                break
            
            # Move towards target
            self.lat += dlat * speed
            self.lon += dlon * speed
            
            # Update heading (0-360)
            self.heading = int((math.atan2(dlon, dlat) * 180 / math.pi) % 360)
            
            # Update telemetry
            self.update_telemetry()
            
            time.sleep(0.1)
    
    def simulate_takeoff(self, target_alt=10):
        """Simulate takeoff"""
        print(f"\n[{self.name}] Taking off to {target_alt}m")
        self.mode = "GUIDED"
        self.armed = True
        
        while self.alt < target_alt:
            self.alt += 0.5
            self.update_telemetry()
            time.sleep(0.2)
        
        print(f"[{self.name}] Takeoff complete at {self.alt}m")
    
    def update_telemetry(self):
        """Update drone telemetry"""
        self.drone.update_telemetry('lat', self.lat)
        self.drone.update_telemetry('lon', self.lon)
        self.drone.update_telemetry('relative_alt', self.alt)
        self.drone.update_telemetry('heading', self.heading)
        self.drone.update_telemetry('mode', self.mode)
        self.drone.update_telemetry('armed', self.armed)
        self.drone.update_telemetry('groundspeed', 5.0 if self.armed else 0.0)
        self.drone.update_telemetry('battery_voltage', 12.6 - (self.alt * 0.01))
        self.drone.update_telemetry('battery_remaining', max(0, 100 - int(self.alt * 2)))
        self.drone.update_telemetry('gps_fix', 3)
        self.drone.update_telemetry('gps_sats', random.randint(10, 15))


def demo_mission(app):
    """
    Runs a demo mission with simulated drones
    Call this after starting the GCS
    """
    print("\n" + "="*60)
    print("STARTING DEMO MISSION")
    print("="*60)
    
    # Starting position (San Francisco area)
    start_lat = 37.7749
    start_lon = -122.4194
    
    # Create simulated drones
    sim_drone1 = SimulatedDrone(app.drone1, start_lat, start_lon, "Drone1-Scout")
    sim_drone2 = SimulatedDrone(app.drone2, start_lat + 0.001, start_lon + 0.001, "Drone2-Delivery")
    
    # Add some test targets
    print("\n[GCS] Adding delivery targets...")
    targets = [
        ("P001", start_lat + 0.002, start_lon + 0.002),
        ("P002", start_lat + 0.003, start_lon - 0.001),
        ("P003", start_lat - 0.001, start_lon + 0.003),
    ]
    
    for pid, lat, lon in targets:
        app.payload_queue.add_target(pid, lat, lon)
        print(f"[GCS] Target {pid} added at ({lat:.6f}, {lon:.6f})")
    
    time.sleep(2)
    
    # Simulate takeoff
    print("\n" + "-"*60)
    print("PHASE 1: TAKEOFF")
    print("-"*60)
    
    import threading
    t1 = threading.Thread(target=sim_drone1.simulate_takeoff, args=(6,))
    t2 = threading.Thread(target=sim_drone2.simulate_takeoff, args=(10,))
    t1.start()
    t2.start()
    t1.join()
    t2.join()
    
    time.sleep(1)
    
    # Drone 1 scouts the area
    print("\n" + "-"*60)
    print("PHASE 2: DRONE 1 SCOUTING")
    print("-"*60)
    
    sim_drone1.simulate_flight(start_lat + 0.004, start_lon + 0.002)
    time.sleep(1)
    sim_drone1.simulate_flight(start_lat + 0.002, start_lon - 0.002)
    
    # Drone 2 delivers to targets
    print("\n" + "-"*60)
    print("PHASE 3: DRONE 2 DELIVERING PAYLOADS")
    print("-"*60)
    
    for pid, lat, lon in targets:
        print(f"\n[Drone2] Delivering to {pid}...")
        sim_drone2.simulate_flight(lat, lon, speed=0.00002)
        time.sleep(2)  # Hover time
        print(f"[GCS] Payload delivered to {pid}!")
    
    # Return to home
    print("\n" + "-"*60)
    print("PHASE 4: RETURN TO HOME")
    print("-"*60)
    
    sim_drone1.mode = "RTL"
    sim_drone2.mode = "RTL"
    sim_drone1.update_telemetry()
    sim_drone2.update_telemetry()
    
    t1 = threading.Thread(target=sim_drone1.simulate_flight, args=(start_lat, start_lon))
    t2 = threading.Thread(target=sim_drone2.simulate_flight, args=(start_lat, start_lon))
    t1.start()
    t2.start()
    t1.join()
    t2.join()
    
    # Land
    while sim_drone1.alt > 0:
        sim_drone1.alt = max(0, sim_drone1.alt - 0.5)
        sim_drone2.alt = max(0, sim_drone2.alt - 0.5)
        sim_drone1.update_telemetry()
        sim_drone2.update_telemetry()
        time.sleep(0.2)
    
    sim_drone1.armed = False
    sim_drone2.armed = False
    sim_drone1.mode = "STABILIZE"
    sim_drone2.mode = "STABILIZE"
    sim_drone1.update_telemetry()
    sim_drone2.update_telemetry()
    
    print("\n" + "="*60)
    print("MISSION COMPLETE!")
    print("="*60)
    print("\nAll payloads delivered successfully!")
    print("Check the map - all targets should be GREEN ✓")


if __name__ == "__main__":
    print("""
╔══════════════════════════════════════════════════════════╗
║          GCS Demo Script - Usage Instructions            ║
╚══════════════════════════════════════════════════════════╝

This script provides a simulated mission for testing the GCS
without requiring real drones or SITL.

USAGE:

1. First, run the GCS normally:
   python gcs_enhanced.py

2. In a Python console or script, import and run the demo:
   
   from test_gcs import demo_mission
   import tkinter as tk
   
   # Get the GCS app instance (if running interactively)
   # Or create a test instance:
   
   from gcs_enhanced import GCSApp
   import threading
   
   app = GCSApp()
   
   # Run demo in background thread
   threading.Thread(
       target=demo_mission, 
       args=(app,), 
       daemon=True
   ).start()
   
   # Start GUI
   app.mainloop()

3. Watch the dashboard and map update in real-time!

WHAT THE DEMO DOES:
- Simulates both drones taking off
- Drone 1 scouts the area
- Adds 3 test targets to the delivery queue
- Drone 2 flies to each target and delivers payload
- Targets turn GREEN when delivered
- Both drones return to home and land

NOTE: This is for testing only. For real missions, use actual
MAVLink connections to real drones or SITL instances.
""")
    
    # You can uncomment this to run a standalone test:
    # from gcs_enhanced import GCSApp
    # import threading
    # 
    # app = GCSApp()
    # threading.Thread(target=demo_mission, args=(app,), daemon=True).start()
    # app.mainloop()

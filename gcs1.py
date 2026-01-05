import tkinter as tk
from tkinter import ttk, messagebox
from pymavlink import mavutil
import threading
import time
import logging
from lawnmower_survey import SurveyWindow
import math
import queue
import re

# Try to import tkintermapview
try:
    from tkintermapview import TkinterMapView
    MAP_AVAILABLE = True
except ImportError:
    MAP_AVAILABLE = False
    print("Warning: tkintermapview not installed. Map features will be disabled.")
    print("Install with: pip install tkintermapview")

# -------------------------------
# LOGGING CONFIGURATION
# -------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)-8s | %(threadName)s | %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)
logger = logging.getLogger("GCS")

# -------------------------------
# COM Port Auto-Detection
# -------------------------------
def find_available_ports():
    """Find all available serial ports"""
    import serial.tools.list_ports
    ports = list(serial.tools.list_ports.comports())
    available = [port.device for port in ports]
    logger.info(f"Available ports: {available}")
    return available

def find_drone_port(baud=57600, timeout=3):
    """
    Scan all available ports and find one with MAVLink heartbeat
    Returns: (port_name, mavlink_connection) or (None, None)
    """
    import serial.tools.list_ports
    
    ports = list(serial.tools.list_ports.comports())
    logger.info(f"Scanning {len(ports)} ports for MAVLink heartbeat...")
    
    for port in ports:
        port_name = port.device
        try:
            logger.info(f"  Trying {port_name}...")
            
            # Try to connect
            master = mavutil.mavlink_connection(port_name, baud=baud)
            
            # Wait for heartbeat with timeout
            logger.info(f"    Waiting for heartbeat on {port_name}...")
            msg = master.wait_heartbeat(timeout=timeout)
            
            if msg:
                logger.info(f"    ✓ Heartbeat found on {port_name}!")
                return port_name, master
            
        except Exception as e:
            logger.debug(f"    ✗ {port_name}: {e}")
            continue
    
    logger.warning("No MAVLink heartbeat found on any port")
    return None, None

# -------------------------------
# Payload Queue Manager
# -------------------------------
class PayloadQueue:
    def __init__(self):
        self.targets = {}  # {person_id: {'lat': float, 'lon': float, 'delivered': bool}}
        self.lock = threading.Lock()
        
    def add_target(self, person_id, lat, lon):
        """Add or update a target person"""
        with self.lock:
            self.targets[person_id] = {
                'lat': lat,
                'lon': lon,
                'delivered': False
            }
            logger.info(f"Payload target added/updated: {person_id} at ({lat:.6f}, {lon:.6f})")
    
    def mark_delivered(self, person_id):
        """Mark a target as delivered"""
        with self.lock:
            if person_id in self.targets:
                self.targets[person_id]['delivered'] = True
                logger.info(f"Payload delivered to: {person_id}")
                return True
        return False
    
    def get_all_targets(self):
        """Get all targets as a list - thread safe copy"""
        with self.lock:
            return [(pid, t['lat'], t['lon'], t['delivered']) 
                    for pid, t in sorted(self.targets.items())]
    
    def get_next_undelivered(self):
        """Get the next undelivered target (first in queue)"""
        with self.lock:
            for pid, t in sorted(self.targets.items()):
                if not t['delivered']:
                    return (pid, t['lat'], t['lon'])
        return None
    
    def get_undelivered_targets(self):
        """Get only undelivered targets"""
        with self.lock:
            return [(pid, t['lat'], t['lon']) 
                    for pid, t in sorted(self.targets.items()) 
                    if not t['delivered']]
    
    def remove_target(self, person_id):
        """Remove a target from queue"""
        with self.lock:
            if person_id in self.targets:
                del self.targets[person_id]
                logger.info(f"Payload target removed: {person_id}")
                return True
        return False
    
    def clear_all(self):
        """Clear all targets"""
        with self.lock:
            self.targets.clear()
            logger.info("All payload targets cleared")

# -------------------------------
# MAVLink Helper Class (Enhanced)
# -------------------------------
class Drone:
    def __init__(self, port, baud=57600, name="Drone"):
        self.port = port
        self.baud = baud
        self.name = name
        self.master = None
        
        # Telemetry data
        self.telemetry = {
            'lat': 0.0,
            'lon': 0.0,
            'alt': 0.0,
            'relative_alt': 0.0,
            'heading': 0,
            'groundspeed': 0.0,
            'airspeed': 0.0,
            'battery_voltage': 0.0,
            'battery_current': 0.0,
            'battery_remaining': 0,
            'gps_fix': 0,
            'gps_sats': 0,
            'mode': 'UNKNOWN',
            'armed': False,
            'home_lat': 0.0,
            'home_lon': 0.0,
        }
        self.telemetry_lock = threading.Lock()
        
        # Person detection callback
        self.person_detection_callback = None
        
        logger.info(f"{self.name} object created (Port={port}, Baud={baud})")

    def get_telemetry(self):
        """Thread-safe telemetry getter"""
        with self.telemetry_lock:
            return self.telemetry.copy()
    
    def update_telemetry(self, key, value):
        """Thread-safe telemetry update"""
        with self.telemetry_lock:
            self.telemetry[key] = value

    def set_person_detection_callback(self, callback):
        """Set callback for person detection events"""
        self.person_detection_callback = callback

    def distance_to(self, target_lat, target_lon):
        """
        Returns distance (meters) from current position to target
        """
        try:
            telem = self.get_telemetry()
            curr_lat = telem['lat']
            curr_lon = telem['lon']
            
            if curr_lat == 0 and curr_lon == 0:
                return None
            
            # Haversine formula
            R = 6371000
            phi1 = math.radians(curr_lat)
            phi2 = math.radians(target_lat)
            dphi = math.radians(target_lat - curr_lat)
            dlambda = math.radians(target_lon - curr_lon)
            
            a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
            return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        except Exception as e:
            logger.error(f"[{self.name}] Distance calculation failed: {e}")
            return None

    def fly_waypoints_guided(self, waypoints, acceptance_radius=2.0):
        """
        Fly waypoints sequentially in GUIDED mode with proper arrival detection
        """
        logger.info(f"[{self.name}] Starting GUIDED waypoint sequence ({len(waypoints)} WPs)")
        
        # Ensure GUIDED mode
        self.set_mode("GUIDED")
        if not self.wait_for_mode("GUIDED"):
            logger.error(f"[{self.name}] Failed to enter GUIDED mode")
            return
        
        # Execute each waypoint
        for i, (lat, lon, alt) in enumerate(waypoints, 1):
            logger.info(f"[{self.name}] Flying to WP{i}/{len(waypoints)}")
            
            # Send waypoint
            self.goto(lat, lon, alt)
            
            # Wait until waypoint reached
            timeout = time.time() + 60  # 60 second timeout per waypoint
            while time.time() < timeout:
                dist = self.distance_to(lat, lon)
                if dist is not None:
                    if dist <= acceptance_radius:
                        logger.info(f"[{self.name}] WP{i} reached (distance: {dist:.1f}m)")
                        break
                time.sleep(1)
            else:
                logger.warning(f"[{self.name}] WP{i} timeout - moving to next waypoint")
        
        logger.info(f"[{self.name}] All waypoints completed!")

    def connect(self):
        try:
            logger.info(f"[{self.name}] Attempting MAVLink connection...")
            self.master = mavutil.mavlink_connection(self.port, baud=self.baud)
            logger.info(f"[{self.name}] Waiting for heartbeat...")
            self.master.wait_heartbeat()
            logger.info(f"[{self.name}] Heartbeat received - Connection established!")
            self.start_telemetry_listener()
        except Exception as e:
            logger.error(f"[{self.name}] Connection failed: {e}")

    def set_mode(self, mode):
        try:
            logger.info(f"[{self.name}] Changing mode to {mode}")
            mode_id = self.master.mode_mapping()[mode]
            self.master.set_mode(mode_id)
            logger.info(f"[{self.name}] Mode set command sent")
        except Exception as e:
            logger.error(f"[{self.name}] Mode change failed: {e}")

    def wait_for_mode(self, desired_mode, timeout=5):
        """Wait for mode change confirmation"""
        start = time.time()
        while time.time() - start < timeout:
            hb = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if not hb:
                continue
            mode = mavutil.mode_string_v10(hb)
            if mode == desired_mode:
                logger.info(f"[{self.name}] Mode CONFIRMED → {mode}")
                return True
        logger.error(f"[{self.name}] Mode change TIMEOUT → {desired_mode}")
        return False

    def arm(self):
        logger.info(f"[{self.name}] ARM requested")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        logger.info(f"[{self.name}] ARM command sent (waiting for telemetry confirmation)")

    def wait_until_armed(self, timeout=5):
        """Wait for arm confirmation"""
        start = time.time()
        while time.time() - start < timeout:
            hb = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if not hb:
                continue
            armed = hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            if armed:
                logger.info(f"[{self.name}] Vehicle ARMED confirmed")
                return True
        logger.error(f"[{self.name}] Arm confirmation TIMEOUT")
        return False

    def takeoff(self, altitude):
        try:
            logger.info(f"[{self.name}] Takeoff initiated (Alt={altitude} m)")
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0, 0, 0, 0, 0, 0, 0, altitude
            )
            logger.info(f"[{self.name}] Takeoff command sent")
        except Exception as e:
            logger.error(f"[{self.name}] Takeoff failed: {e}")

    def goto(self, lat, lon, alt, duration=6):
        """
        Fly to a GPS position in GUIDED mode (ArduCopter)
        """
        try:
            logger.info(f"[{self.name}] GUIDED goto → Lat={lat:.6f}, Lon={lon:.6f}, Alt={alt}m")
            
            # Ensure GUIDED mode
            mode_id = self.master.mode_mapping()['GUIDED']
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )
            
            # Wait until airborne (optional check)
            start = time.time()
            while True:
                msg = self.master.recv_match(
                    type='GLOBAL_POSITION_INT',
                    blocking=True,
                    timeout=1
                )
                if msg:
                    rel_alt = msg.relative_alt / 1000.0
                    if rel_alt > 1.5:
                        logger.info(f"[{self.name}] Airborne at {rel_alt:.1f} m")
                        break
                if time.time() - start > 10:
                    logger.warning(f"[{self.name}] Not airborne → GUIDED move may fail")
                    break
            
            # Stream setpoints
            type_mask = int(0b110111111000)  # Position only
            end_time = time.time() + duration
            
            while time.time() < end_time:
                self.master.mav.set_position_target_global_int_send(
                    0,  # time_boot_ms
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    type_mask,
                    int(lat * 1e7),
                    int(lon * 1e7),
                    alt,
                    0, 0, 0,  # vx, vy, vz
                    0, 0, 0,  # ax, ay, az
                    0, 0      # yaw, yaw_rate
                )
                time.sleep(0.1)
            
            logger.info(f"[{self.name}] GUIDED goto streaming complete")
            
        except Exception as e:
            logger.error(f"[{self.name}] GUIDED goto failed: {e}")

    def rtl(self):
        """Initiate Return-To-Launch (RTL)"""
        try:
            logger.info(f"[{self.name}] RTL requested")
            mode_id = self.master.mode_mapping().get("RTL")
            if mode_id is None:
                logger.error(f"[{self.name}] RTL mode not supported")
                return
            self.master.set_mode(mode_id)
            logger.info(f"[{self.name}] RTL command sent")
        except Exception as e:
            logger.error(f"[{self.name}] RTL failed: {e}")

    def drop_payload(self):
        """Drop payload using servo mechanism on AUX OUT 1 (Channel 9)"""
        try:
            logger.info(f"[{self.name}] 📦 Activating payload release servo")
            
            # OPEN SERVO (Release position)
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,              # Confirmation
                9,              # Servo channel (AUX OUT 1 = Channel 9)
                2000,           # PWM value (2000 μs = fully open)
                0, 0, 0, 0, 0   # Unused parameters
            )
            
            logger.info(f"[{self.name}] ✅ Servo OPEN - Payload released!")
            time.sleep(2.0)  # Wait 2 seconds for servo to move and payload to drop
            
            # CLOSE SERVO (Reset to secure position)
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                9,              # Servo channel
                1000,           # PWM value (1000 μs = fully closed)
                0, 0, 0, 0, 0
            )
            
            logger.info(f"[{self.name}] Servo CLOSED - Ready for next payload")
            return True
            
        except Exception as e:
            logger.error(f"[{self.name}] ❌ Payload drop failed: {e}")
            return False

    def start_telemetry_listener(self):
        """Background thread for telemetry monitoring"""
        def listener():
            logger.info(f"[{self.name}] Telemetry listener started")
            while True:
                if not self.master:
                    time.sleep(1)
                    continue
                
                msg = self.master.recv_match(blocking=True, timeout=1)
                if not msg:
                    continue
                
                msg_type = msg.get_type()
                
                # GLOBAL POSITION
                if msg_type == "GLOBAL_POSITION_INT":
                    self.update_telemetry('lat', msg.lat / 1e7)
                    self.update_telemetry('lon', msg.lon / 1e7)
                    self.update_telemetry('alt', msg.alt / 1000.0)
                    self.update_telemetry('relative_alt', msg.relative_alt / 1000.0)
                    self.update_telemetry('heading', msg.hdg / 100)
                
                # VFR_HUD (speeds)
                elif msg_type == "VFR_HUD":
                    self.update_telemetry('groundspeed', msg.groundspeed)
                    self.update_telemetry('airspeed', msg.airspeed)
                    self.update_telemetry('heading', msg.heading)
                
                # BATTERY STATUS
                elif msg_type == "SYS_STATUS":
                    self.update_telemetry('battery_voltage', msg.voltage_battery / 1000.0)
                    self.update_telemetry('battery_current', msg.current_battery / 100.0)
                    self.update_telemetry('battery_remaining', msg.battery_remaining)
                
                # GPS STATUS
                elif msg_type == "GPS_RAW_INT":
                    self.update_telemetry('gps_fix', msg.fix_type)
                    self.update_telemetry('gps_sats', msg.satellites_visible)
                
                # HOME POSITION
                elif msg_type == "HOME_POSITION":
                    self.update_telemetry('home_lat', msg.latitude / 1e7)
                    self.update_telemetry('home_lon', msg.longitude / 1e7)
                
                # HEARTBEAT
                elif msg_type == "HEARTBEAT":
                    armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    mode = mavutil.mode_string_v10(msg)
                    self.update_telemetry('armed', bool(armed))
                    self.update_telemetry('mode', mode)
                
                # STATUSTEXT - PERSON DETECTION PARSING
                elif msg_type == "STATUSTEXT":
                    text = msg.text.decode(errors="ignore").strip()
                    logger.info(f"[{self.name}] STATUS → {text}")
                    
                    # Parse person detection: "person<id>,lat,lon"
                    # Example: "person1,37.123456,-122.654321"
                    match = re.match(r'person(\d+),([-\d.]+),([-\d.]+)', text, re.IGNORECASE)
                    if match:
                        person_id = match.group(1)
                        lat = float(match.group(2))
                        lon = float(match.group(3))
                        
                        logger.info(f"[{self.name}] 🎯 Person detected: ID={person_id}, Lat={lat:.6f}, Lon={lon:.6f}")
                        
                        # Call detection callback
                        if self.person_detection_callback:
                            self.person_detection_callback(person_id, lat, lon)
                
                # COMMAND ACK
                elif msg_type == "COMMAND_ACK":
                    result_map = {
                        mavutil.mavlink.MAV_RESULT_ACCEPTED: "ACCEPTED",
                        mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED: "TEMP REJECTED",
                        mavutil.mavlink.MAV_RESULT_DENIED: "DENIED",
                        mavutil.mavlink.MAV_RESULT_UNSUPPORTED: "UNSUPPORTED",
                        mavutil.mavlink.MAV_RESULT_FAILED: "FAILED"
                    }
                    logger.info(f"[{self.name}] COMMAND_ACK → Command={msg.command}, Result={result_map.get(msg.result, 'UNKNOWN')}")
        
        threading.Thread(
            target=listener,
            daemon=True,
            name=f"{self.name}-Telemetry"
        ).start()

# -------------------------------
# Dashboard Widget
# -------------------------------
class DashboardWidget(ttk.Frame):
    def __init__(self, parent, drone, title):
        super().__init__(parent, relief=tk.RIDGE, borderwidth=2)
        self.drone = drone
        self.title = title
        
        # Title
        ttk.Label(self, text=title, font=("Arial", 12, "bold")).pack(pady=5)
        
        # Telemetry frame
        tel_frame = ttk.Frame(self)
        tel_frame.pack(padx=10, pady=5, fill=tk.BOTH, expand=True)
        
        # Create labels
        self.labels = {}
        row = 0
        
        fields = [
            ("Mode", "mode"),
            ("Armed", "armed"),
            ("Position", "position"),
            ("Altitude", "relative_alt"),
            ("Heading", "heading"),
            ("Speed", "groundspeed"),
            ("Battery", "battery"),
            ("GPS", "gps"),
        ]
        
        for label_text, key in fields:
            ttk.Label(tel_frame, text=f"{label_text}:", font=("Arial", 9, "bold")).grid(
                row=row, column=0, sticky=tk.W, padx=5, pady=2
            )
            value_label = ttk.Label(tel_frame, text="--", font=("Arial", 9))
            value_label.grid(row=row, column=1, sticky=tk.W, padx=5, pady=2)
            self.labels[key] = value_label
            row += 1
        
        # Start update loop
        self.update_display()
    
    def update_display(self):
        """Update telemetry display"""
        try:
            telem = self.drone.get_telemetry()
            
            # Update labels
            self.labels['mode'].config(text=telem['mode'])
            self.labels['armed'].config(
                text="ARMED" if telem['armed'] else "DISARMED",
                foreground="red" if telem['armed'] else "gray"
            )
            
            if telem['lat'] != 0 or telem['lon'] != 0:
                self.labels['position'].config(
                    text=f"{telem['lat']:.6f}, {telem['lon']:.6f}"
                )
            else:
                self.labels['position'].config(text="No GPS")
            
            self.labels['relative_alt'].config(text=f"{telem['relative_alt']:.1f} m")
            self.labels['heading'].config(text=f"{telem['heading']}°")
            self.labels['groundspeed'].config(text=f"{telem['groundspeed']:.1f} m/s")
            
            # Battery
            battery_text = f"{telem['battery_voltage']:.1f}V ({telem['battery_remaining']}%)"
            self.labels['battery'].config(text=battery_text)
            
            # GPS
            gps_fix_map = {
                0: "NO GPS", 1: "NO FIX", 2: "2D", 
                3: "3D", 4: "DGPS", 5: "RTK FLOAT", 6: "RTK FIX"
            }
            gps_text = f"{gps_fix_map.get(telem['gps_fix'], 'UNKNOWN')} ({telem['gps_sats']} sats)"
            self.labels['gps'].config(text=gps_text)
            
        except Exception as e:
            logger.error(f"Dashboard update error: {e}")
        
        # Schedule next update
        self.after(500, self.update_display)

# -------------------------------
# Map Widget (if available)
# -------------------------------
class MapWidget(ttk.Frame):
    def __init__(self, parent, drone1, drone2, payload_queue):
        super().__init__(parent)
        self.drone1 = drone1
        self.drone2 = drone2
        self.payload_queue = payload_queue
        
        self.drone1_marker = None
        self.drone2_marker = None
        self.target_markers = {}
        self.map_centered = False
        
        # Survey visualization
        self.geofence_path = None
        self.pattern_path = None
        self.pattern_markers = []
        
        if MAP_AVAILABLE:
            self.map_widget = TkinterMapView(self, corner_radius=0)
            self.map_widget.pack(fill=tk.BOTH, expand=True)
            self.map_widget.set_position(37.7749, -122.4194)
            self.map_widget.set_zoom(15)
            
            self.update_map()
        else:
            ttk.Label(
                self, 
                text="Map Not Available\nInstall: pip install tkintermapview",
                font=("Arial", 12)
            ).pack(expand=True)
    
    def visualize_geofence(self, polygon):
        """Visualize geofence boundary on map"""
        if not MAP_AVAILABLE or not polygon:
            return
        
        try:
            # Clear old geofence
            if self.geofence_path:
                try:
                    self.geofence_path.delete()
                except:
                    pass
            
            coords = [(lat, lon) for lat, lon in polygon]
            coords.append(polygon[0])  # Close polygon
            
            self.geofence_path = self.map_widget.set_path(
                coords,
                color="red",
                width=3
            )
            
            # Center on geofence
            lats = [p[0] for p in polygon]
            lons = [p[1] for p in polygon]
            center_lat = (min(lats) + max(lats)) / 2
            center_lon = (min(lons) + max(lons)) / 2
            self.map_widget.set_position(center_lat, center_lon)
            self.map_widget.set_zoom(16)
            
            logger.info("Geofence visualized on main GCS map (RED)")
            
        except Exception as e:
            logger.error(f"Geofence viz error: {e}")
    
    def visualize_pattern(self, waypoints):
        """Visualize survey pattern on map"""
        if not MAP_AVAILABLE or not waypoints:
            return
        
        try:
            # Clear old pattern
            if self.pattern_path:
                try:
                    self.pattern_path.delete()
                except:
                    pass
            
            for marker in self.pattern_markers:
                if marker:
                    try:
                        marker.delete()
                    except:
                        pass
            self.pattern_markers.clear()
            
            # Draw pattern path
            coords = [(lat, lon) for lat, lon, _ in waypoints]
            self.pattern_path = self.map_widget.set_path(
                coords,
                color="blue",
                width=2
            )
            
            # Add start/end markers
            if len(waypoints) > 0:
                lat, lon, _ = waypoints[0]
                start_marker = self.map_widget.set_marker(
                    lat, lon,
                    text="START",
                    marker_color_circle="green",
                    marker_color_outside="darkgreen"
                )
                self.pattern_markers.append(start_marker)
            
            if len(waypoints) > 1:
                lat, lon, _ = waypoints[-1]
                end_marker = self.map_widget.set_marker(
                    lat, lon,
                    text="END",
                    marker_color_circle="orange",
                    marker_color_outside="darkorange"
                )
                self.pattern_markers.append(end_marker)
            
            logger.info("Survey pattern visualized on main GCS map (BLUE)")
            
        except Exception as e:
            logger.error(f"Pattern viz error: {e}")
    
    def clear_survey_viz(self):
        """Clear survey visualization"""
        if not MAP_AVAILABLE:
            return
        
        try:
            if self.geofence_path:
                self.geofence_path.delete()
                self.geofence_path = None
            
            if self.pattern_path:
                self.pattern_path.delete()
                self.pattern_path = None
            
            for marker in self.pattern_markers:
                if marker:
                    try:
                        marker.delete()
                    except:
                        pass
            self.pattern_markers.clear()
            
            logger.info("Survey visualization cleared from main GCS map")
            
        except Exception as e:
            logger.error(f"Clear survey viz error: {e}")
    
    def update_map(self):
        """Update drone and target positions on map"""
        try:
            # Update Drone 1
            telem1 = self.drone1.get_telemetry()
            if telem1['lat'] != 0 and telem1['lon'] != 0:
                if self.drone1_marker:
                    try:
                        self.drone1_marker.delete()
                    except:
                        pass
                
                self.drone1_marker = self.map_widget.set_marker(
                    telem1['lat'], 
                    telem1['lon'],
                    text=f"Scout\n{telem1['relative_alt']:.1f}m",
                    marker_color_circle="blue",
                    marker_color_outside="darkblue"
                )
                
                # Auto-center on first GPS lock
                if not self.map_centered:
                    self.map_widget.set_position(telem1['lat'], telem1['lon'])
                    self.map_widget.set_zoom(17)
                    self.map_centered = True
                    logger.info(f"Map centered on Drone 1: {telem1['lat']:.6f}, {telem1['lon']:.6f}")
            
            # Update Drone 2
            telem2 = self.drone2.get_telemetry()
            if telem2['lat'] != 0 and telem2['lon'] != 0:
                if self.drone2_marker:
                    try:
                        self.drone2_marker.delete()
                    except:
                        pass
                
                self.drone2_marker = self.map_widget.set_marker(
                    telem2['lat'], 
                    telem2['lon'],
                    text=f"Delivery\n{telem2['relative_alt']:.1f}m",
                    marker_color_circle="red",
                    marker_color_outside="darkred"
                )
                
                # Auto-center if drone1 hasn't gotten lock yet
                if not self.map_centered:
                    self.map_widget.set_position(telem2['lat'], telem2['lon'])
                    self.map_widget.set_zoom(17)
                    self.map_centered = True
                    logger.info(f"Map centered on Drone 2: {telem2['lat']:.6f}, {telem2['lon']:.6f}")
            
            # Update target markers
            targets = self.payload_queue.get_all_targets()
            current_target_ids = set([pid for pid, _, _, _ in targets])
            
            # Remove deleted targets
            for pid in list(self.target_markers.keys()):
                if pid not in current_target_ids:
                    if self.target_markers[pid]:
                        try:
                            self.target_markers[pid].delete()
                        except:
                            pass
                    del self.target_markers[pid]
            
            # Add/update targets
            for person_id, lat, lon, delivered in targets:
                # Delete old marker
                if person_id in self.target_markers and self.target_markers[person_id]:
                    try:
                        self.target_markers[person_id].delete()
                    except:
                        pass
                
                color = "green" if delivered else "orange"
                outside_color = "darkgreen" if delivered else "darkorange"
                status = "✓ Delivered" if delivered else "Pending"
                
                # Create new marker
                self.target_markers[person_id] = self.map_widget.set_marker(
                    lat, lon,
                    text=f"Person {person_id}\n{status}",
                    marker_color_circle=color,
                    marker_color_outside=outside_color
                )
            
            # Check for payload delivery (Drone 2 within 5m of target)
            for person_id, lat, lon, delivered in targets:
                if not delivered:
                    dist = self.drone2.distance_to(lat, lon)
                    if dist is not None and dist < 5.0:
                        logger.info(f"🎯 Payload delivered to Person {person_id}!")
                        self.payload_queue.mark_delivered(person_id)
            
        except Exception as e:
            logger.error(f"Map update error: {e}")
        
        # Schedule next update
        self.after(1000, self.update_map)

# -------------------------------
# Payload Queue Table Widget
# -------------------------------
class PayloadQueueWidget(ttk.Frame):
    def __init__(self, parent, payload_queue):
        super().__init__(parent)
        self.payload_queue = payload_queue
        
        ttk.Label(self, text="Payload Delivery Queue", font=("Arial", 12, "bold")).pack(pady=5)
        
        # Table
        columns = ("Person ID", "Latitude", "Longitude", "Status")
        self.tree = ttk.Treeview(self, columns=columns, show='headings', height=8)
        
        for col in columns:
            self.tree.heading(col, text=col)
            self.tree.column(col, width=100)
        
        self.tree.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Scrollbar
        scrollbar = ttk.Scrollbar(self, orient=tk.VERTICAL, command=self.tree.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.tree.configure(yscrollcommand=scrollbar.set)
        
        # Control frame
        control_frame = ttk.Frame(self)
        control_frame.pack(pady=5, fill=tk.X)
        
        # Input fields
# CONTINUATION OF gcs.py - Part 2

# Payload Queue Table Widget (continued)
        input_frame = ttk.Frame(control_frame)
        input_frame.pack(fill=tk.X, padx=5)
        
        ttk.Label(input_frame, text="Person ID:").grid(row=0, column=0, padx=5, sticky=tk.W)
        self.person_id_var = tk.StringVar()
        ttk.Entry(input_frame, textvariable=self.person_id_var, width=12).grid(row=0, column=1, padx=5)
        
        ttk.Label(input_frame, text="Lat:").grid(row=0, column=2, padx=5, sticky=tk.W)
        self.lat_var = tk.DoubleVar(value=0.0)
        ttk.Entry(input_frame, textvariable=self.lat_var, width=12).grid(row=0, column=3, padx=5)
        
        ttk.Label(input_frame, text="Lon:").grid(row=0, column=4, padx=5, sticky=tk.W)
        self.lon_var = tk.DoubleVar(value=0.0)
        ttk.Entry(input_frame, textvariable=self.lon_var, width=12).grid(row=0, column=5, padx=5)
        
        # Buttons
        button_frame = ttk.Frame(control_frame)
        button_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Button(button_frame, text="Add/Update", command=self.add_target).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Remove", command=self.remove_target).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Clear All", command=self.clear_all).pack(side=tk.LEFT, padx=5)
        
        # Start update loop
        self.update_table()
    
    def add_target(self):
        """Add or update a target"""
        try:
            person_id = self.person_id_var.get().strip()
            
            if not person_id:
                messagebox.showerror("Error", "Please enter a Person ID")
                return
            
            lat = self.lat_var.get()
            lon = self.lon_var.get()
            
            if lat == 0.0 and lon == 0.0:
                messagebox.showwarning("Warning", "Coordinates are (0, 0). Is this correct?")
            
            self.payload_queue.add_target(person_id, lat, lon)
            
            # Clear entries
            self.person_id_var.set("")
            self.lat_var.set(0.0)
            self.lon_var.set(0.0)
            
            messagebox.showinfo("Success", f"Target {person_id} added/updated")
            
        except Exception as e:
            logger.error(f"Add target error: {e}")
            messagebox.showerror("Error", f"Failed to add target: {e}")
    
    def remove_target(self):
        """Remove selected target"""
        selection = self.tree.selection()
        if not selection:
            messagebox.showwarning("Warning", "Please select a target to remove")
            return
        
        try:
            for item in selection:
                values = self.tree.item(item)['values']
                person_id = str(values[0])
                self.payload_queue.remove_target(person_id)
            
            messagebox.showinfo("Success", "Target(s) removed")
            
        except Exception as e:
            logger.error(f"Remove target error: {e}")
            messagebox.showerror("Error", f"Failed to remove target: {e}")
    
    def clear_all(self):
        """Clear all targets"""
        if messagebox.askyesno("Confirm", "Remove all targets?"):
            self.payload_queue.clear_all()
            messagebox.showinfo("Success", "All targets cleared")
    
    def update_table(self):
        """Update the table display"""
        try:
            # Clear existing items
            for item in self.tree.get_children():
                self.tree.delete(item)
            
            # Add all targets
            targets = self.payload_queue.get_all_targets()
            for person_id, lat, lon, delivered in targets:
                status = "✓ Delivered" if delivered else "Pending"
                item_id = self.tree.insert("", tk.END, values=(person_id, f"{lat:.6f}", f"{lon:.6f}", status))
                
                # Color coding
                if delivered:
                    self.tree.item(item_id, tags=('delivered',))
            
            self.tree.tag_configure('delivered', background='lightgreen')
            
        except Exception as e:
            logger.error(f"Table update error: {e}")
        
        # Schedule next update
        self.after(1000, self.update_table)

# -------------------------------
# Enhanced GUI Application
# -------------------------------
class GCSApp(tk.Tk):
    def __init__(self):
        super().__init__()
        logger.info("Starting Enhanced GCS Application")
        
        self.title("Dual Drone GCS - Mission Control Dashboard")
        self.geometry("1400x900")
        
        # Create drones (ports will be auto-detected)
        self.drone1 = Drone("AUTO", 57600, "Drone1-Scout")
        self.drone2 = Drone("AUTO", 57600, "Drone2-Delivery")
        
        # Create payload queue
        self.payload_queue = PayloadQueue()
        
        # Max altitude setting (default 6m)
        self.max_altitude = tk.DoubleVar(value=6.0)
        
        # Mission state
        self.mission_active = False
        self.mission_thread = None
        self.survey_waypoints = []
        self.geofence_polygon = []  # Store geofence for visualization
        
        # Set up person detection callback
        self.drone1.set_person_detection_callback(self.on_person_detected)
        
        # Create main layout
        self.create_widgets()
        
        logger.info("GUI initialized successfully")

    def on_person_detected(self, person_id, lat, lon):
        """Callback when scout drone detects a person"""
        logger.info(f"📡 GCS received person detection: ID={person_id}, Lat={lat}, Lon={lon}")
        
        # Add to payload queue (will update if person already exists)
        self.payload_queue.add_target(person_id, lat, lon)
        
        # Show notification
        self.after(0, lambda: messagebox.showinfo(
            "Person Detected",
            f"Person {person_id} detected!\nLocation: {lat:.6f}, {lon:.6f}\n\nAdded to delivery queue."
        ))

    def auto_connect_drone(self, drone):
        """Auto-detect and connect to a drone"""
        try:
            logger.info(f"[{drone.name}] Auto-detecting port...")
            
            port, master = find_drone_port(baud=drone.baud, timeout=3)
            
            if port and master:
                drone.port = port
                drone.master = master
                logger.info(f"[{drone.name}] Connected to {port}")
                drone.start_telemetry_listener()
                messagebox.showinfo("Connected", f"{drone.name} connected to {port}")
            else:
                logger.error(f"[{drone.name}] No drone found on any port")
                messagebox.showerror("Error", f"Could not find {drone.name} on any port.\n\nMake sure:\n- Drone is powered on\n- USB cable is connected\n- Drivers are installed")
        
        except Exception as e:
            logger.error(f"[{drone.name}] Auto-connect failed: {e}")
            messagebox.showerror("Error", f"Connection failed: {e}")

    def create_widgets(self):
        # Main title
        title_frame = ttk.Frame(self)
        title_frame.pack(fill=tk.X, pady=5)
        ttk.Label(
            title_frame, 
            text="🚁 Dual Drone Mission Control Dashboard 🚁", 
            font=("Arial", 18, "bold")
        ).pack()
        
        # Main container
        main_container = ttk.PanedWindow(self, orient=tk.HORIZONTAL)
        main_container.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Left panel - Dashboards and Controls
        left_panel = ttk.Frame(main_container)
        main_container.add(left_panel, weight=1)
        
        # Right panel - Map and Queue
        right_panel = ttk.Frame(main_container)
        main_container.add(right_panel, weight=2)
        
        # === LEFT PANEL ===
        
        # Dashboards
        dashboard_frame = ttk.Frame(left_panel)
        dashboard_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        DashboardWidget(dashboard_frame, self.drone1, "Drone 1 - Scout").pack(
            fill=tk.BOTH, expand=True, padx=5, pady=5
        )
        DashboardWidget(dashboard_frame, self.drone2, "Drone 2 - Delivery").pack(
            fill=tk.BOTH, expand=True, padx=5, pady=5
        )
        
        # Control Panel
        control_frame = ttk.LabelFrame(left_panel, text="Mission Controls")
        control_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Connection buttons (AUTO-DETECT)
        conn_frame = ttk.Frame(control_frame)
        conn_frame.pack(fill=tk.X, pady=5)
        ttk.Button(
            conn_frame, text="🔍 Auto-Connect Drone 1",
            command=lambda: threading.Thread(
                target=self.auto_connect_drone, args=(self.drone1,), daemon=True
            ).start()
        ).pack(side=tk.LEFT, padx=5)
        ttk.Button(
            conn_frame, text="🔍 Auto-Connect Drone 2",
            command=lambda: threading.Thread(
                target=self.auto_connect_drone, args=(self.drone2,), daemon=True
            ).start()
        ).pack(side=tk.LEFT, padx=5)
        
        # Max Altitude Setting
        alt_frame = ttk.Frame(control_frame)
        alt_frame.pack(fill=tk.X, pady=5)
        ttk.Label(alt_frame, text="Max Altitude (m):").pack(side=tk.LEFT, padx=5)
        ttk.Entry(alt_frame, textvariable=self.max_altitude, width=8).pack(side=tk.LEFT)
        ttk.Label(alt_frame, text="(Default: 6m)", font=("Arial", 8), foreground="gray").pack(side=tk.LEFT, padx=5)
        
        # === INDIVIDUAL DRONE CONTROLS (PRE-FLIGHT CHECKS) ===
        preflight_frame = ttk.LabelFrame(control_frame, text="Pre-Flight Controls")
        preflight_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Drone 1 controls
        d1_frame = ttk.Frame(preflight_frame)
        d1_frame.pack(fill=tk.X, pady=2)
        ttk.Label(d1_frame, text="Drone 1 (Scout):", font=("Arial", 9, "bold")).pack(side=tk.LEFT, padx=5)
        ttk.Button(d1_frame, text="ARM", command=lambda: self.arm_single(self.drone1), width=10).pack(side=tk.LEFT, padx=2)
        ttk.Button(d1_frame, text="TAKEOFF", command=lambda: self.takeoff_single(self.drone1), width=10).pack(side=tk.LEFT, padx=2)
        ttk.Button(d1_frame, text="RTL", command=lambda: self.drone1.rtl(), width=10).pack(side=tk.LEFT, padx=2)
        
        # Drone 2 controls
        d2_frame = ttk.Frame(preflight_frame)
        d2_frame.pack(fill=tk.X, pady=2)
        ttk.Label(d2_frame, text="Drone 2 (Delivery):", font=("Arial", 9, "bold")).pack(side=tk.LEFT, padx=5)
        ttk.Button(d2_frame, text="ARM", command=lambda: self.arm_single(self.drone2), width=10).pack(side=tk.LEFT, padx=2)
        ttk.Button(d2_frame, text="TAKEOFF", command=lambda: self.takeoff_single(self.drone2), width=10).pack(side=tk.LEFT, padx=2)
        ttk.Button(d2_frame, text="RTL", command=lambda: self.drone2.rtl(), width=10).pack(side=tk.LEFT, padx=2)
        
         # Payload test controls
        payload_frame = ttk.Frame(preflight_frame)
        payload_frame.pack(fill=tk.X, pady=5)
        ttk.Label(payload_frame, text="Payload Test:", font=("Arial", 9, "bold")).pack(side=tk.LEFT, padx=5)
        ttk.Button(
            payload_frame, 
            text="🧪 TEST SERVO", 
            command=self.test_payload_servo,
            width=15
        ).pack(side=tk.LEFT, padx=2)
        ttk.Label(
            payload_frame, 
            text="(Ground test - no flight required)", 
            font=("Arial", 7), 
            foreground="gray"
        ).pack(side=tk.LEFT, padx=5)

        # === MAIN MISSION CONTROLS ===
        mission_frame = ttk.LabelFrame(control_frame, text="Main Mission")
        mission_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Button(
            mission_frame, 
            text="Open Survey Tool", 
            command=self.open_survey_window
        ).pack(fill=tk.X, padx=5, pady=2)
        
        ttk.Button(
            mission_frame, 
            text="🚀 START MISSION", 
            command=self.start_mission,
            style="Accent.TButton"
        ).pack(fill=tk.X, padx=5, pady=2)
        
        ttk.Button(
            mission_frame, 
            text="🛑 EMERGENCY STOP", 
            command=self.emergency_stop
        ).pack(fill=tk.X, padx=5, pady=2)
        
        # Mission status
        self.mission_status_var = tk.StringVar(value="Mission: IDLE")
        ttk.Label(
            mission_frame, 
            textvariable=self.mission_status_var,
            font=("Arial", 10, "bold"),
            foreground="gray"
        ).pack(pady=5)
        
        # === RIGHT PANEL ===
        
        # Map
        map_label_frame = ttk.LabelFrame(right_panel, text="Mission Map")
        map_label_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        if MAP_AVAILABLE:
            # Map controls at top
            map_control_frame = ttk.Frame(map_label_frame)
            map_control_frame.pack(fill=tk.X, padx=5, pady=5)
            
            self.map_widget_ref = MapWidget(map_label_frame, self.drone1, self.drone2, self.payload_queue)
            self.map_widget_ref.pack(fill=tk.BOTH, expand=True)
            
            # Add control buttons
            ttk.Button(
                map_control_frame,
                text="📍 Center on Drones",
                command=self.center_map_on_drones
            ).pack(side=tk.LEFT, padx=5)
            
            ttk.Button(
                map_control_frame,
                text="🔍 Zoom In",
                command=lambda: self.map_widget_ref.map_widget.set_zoom(
                    self.map_widget_ref.map_widget.zoom + 1
                ) if MAP_AVAILABLE else None
            ).pack(side=tk.LEFT, padx=5)
            
            ttk.Button(
                map_control_frame,
                text="🔍 Zoom Out",
                command=lambda: self.map_widget_ref.map_widget.set_zoom(
                    self.map_widget_ref.map_widget.zoom - 1
                ) if MAP_AVAILABLE else None
            ).pack(side=tk.LEFT, padx=5)
        else:
            ttk.Label(
                map_label_frame,
                text="Map not available\nInstall: pip install tkintermapview",
                font=("Arial", 12)
            ).pack(expand=True)
        
        # Payload Queue
        queue_frame = ttk.LabelFrame(right_panel, text="Delivery Targets")
        queue_frame.pack(fill=tk.BOTH, padx=5, pady=5)
        PayloadQueueWidget(queue_frame, self.payload_queue).pack(fill=tk.BOTH, expand=True)

    # -------------------------------
    # Control Methods
    # -------------------------------
    def arm_single(self, drone):
        """ARM a single drone"""
        logger.info(f"Arming {drone.name}")
        threading.Thread(target=self._arm_drone, args=(drone,), daemon=True).start()
    
    def _arm_drone(self, drone):
        drone.set_mode("GUIDED")
        drone.wait_for_mode("GUIDED")
        drone.arm()
        drone.wait_until_armed()
    
    def takeoff_single(self, drone):
        """Takeoff a single drone"""
        max_alt = self.max_altitude.get()
        logger.info(f"Takeoff {drone.name} to {max_alt}m")
        threading.Thread(target=self._takeoff_drone, args=(drone, max_alt), daemon=True).start()
    
    def _takeoff_drone(self, drone, alt):
        drone.set_mode("GUIDED")
        drone.wait_for_mode("GUIDED")
        drone.arm()
        drone.wait_until_armed()
        drone.takeoff(alt)

    def test_payload_servo(self):
        """Test payload servo mechanism (ground test)"""
        # Check if Drone 2 is connected
        if not self.drone2.master:
            messagebox.showerror(
                "Not Connected",
                "Drone 2 (Delivery) is not connected!\n\n"
                "Please connect Drone 2 first using 'Auto-Connect Drone 2'"
            )
            return
        
        # Confirmation dialog
        response = messagebox.askyesno(
            "Test Payload Servo",
            "This will test the payload release servo.\n\n"
            "⚠️ Make sure:\n"
            "• Drone 2 is connected\n"
            "• Servo is connected to AUX OUT 1\n"
            "• Payload mechanism is ready\n"
            "• NO propellers installed (ground test)\n\n"
            "The servo will:\n"
            "1. Open (release position)\n"
            "2. Wait 2 seconds\n"
            "3. Close (secure position)\n\n"
            "Continue?"
        )
        
        if not response:
            return
        
        # Run test in separate thread
        threading.Thread(
            target=self._run_servo_test, 
            daemon=True
        ).start()
    
    def _run_servo_test(self):
        """Execute servo test sequence"""
        try:
            logger.info("🧪 Starting payload servo test...")
            
            # Update status
            self.mission_status_var.set("Testing: SERVO")
            
            # Step 1: Close position (secure)
            logger.info("Step 1: Moving servo to CLOSED position (PWM 1000)")
            self.drone2.master.mav.command_long_send(
                self.drone2.master.target_system,
                self.drone2.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                9,      # Channel 9 (AUX OUT 1)
                1000,   # PWM 1000 (closed)
                0, 0, 0, 0, 0
            )
            
            self.after(0, lambda: messagebox.showinfo(
                "Servo Test - Step 1",
                "✅ Servo moved to CLOSED position (PWM 1000)\n\n"
                "Check: Payload should be SECURED\n\n"
                "Next: Opening servo in 3 seconds..."
            ))
            
            time.sleep(3)
            
            # Step 2: Open position (release)
            logger.info("Step 2: Moving servo to OPEN position (PWM 2000)")
            self.drone2.master.mav.command_long_send(
                self.drone2.master.target_system,
                self.drone2.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                9,      # Channel 9
                2000,   # PWM 2000 (open)
                0, 0, 0, 0, 0
            )
            
            self.after(0, lambda: messagebox.showinfo(
                "Servo Test - Step 2",
                "✅ Servo moved to OPEN position (PWM 2000)\n\n"
                "Check: Payload should be RELEASED\n\n"
                "Holding for 2 seconds..."
            ))
            
            time.sleep(2)
            
            # Step 3: Return to closed
            logger.info("Step 3: Returning servo to CLOSED position")
            self.drone2.master.mav.command_long_send(
                self.drone2.master.target_system,
                self.drone2.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                9,
                1000,   # Back to closed
                0, 0, 0, 0, 0
            )
            
            logger.info("✅ Servo test complete!")
            
            # Reset status
            self.mission_status_var.set("Mission: IDLE")
            
            # Final message
            self.after(0, lambda: messagebox.showinfo(
                "Servo Test Complete",
                "✅ Servo test completed successfully!\n\n"
                "Test sequence:\n"
                "1. ✓ Closed position (1000 μs)\n"
                "2. ✓ Open position (2000 μs)\n"
                "3. ✓ Returned to closed\n\n"
                "If servo moved correctly:\n"
                "• PWM 1000 = Payload SECURED\n"
                "• PWM 2000 = Payload RELEASED\n\n"
                "If servo moved backwards,\n"
                "set SERVO9_REVERSED = 1 in parameters."
            ))
            
        except Exception as e:
            logger.error(f"Servo test failed: {e}")
            self.mission_status_var.set("Mission: IDLE")
            self.after(0, lambda: messagebox.showerror(
                "Servo Test Failed",
                f"❌ Servo test failed!\n\n"
                f"Error: {str(e)}\n\n"
                f"Check:\n"
                f"• Drone 2 is connected\n"
                f"• Servo connected to AUX OUT 1\n"
                f"• SERVO9_FUNCTION = 1\n"
                f"• Check logs for details"
            ))
    
    def open_survey_window(self):
        """Open survey window"""
        logger.info("Opening Survey Window")
        SurveyWindow(self, self.drone1, self.drone2, self.max_altitude.get())
    
    
    def set_survey_pattern(self, waypoints):
        """Called by survey window to sync pattern"""
        self.survey_waypoints = waypoints
        logger.info(f"✓ Pattern synced: {len(waypoints)} waypoints")
        
        # Update map
        if MAP_AVAILABLE and hasattr(self, 'map_widget_ref'):
            self.map_widget_ref.visualize_pattern(waypoints)
        
        self.mission_status_var.set(f"Mission: READY ({len(waypoints)} WPs)")
    
    def clear_survey_data(self):
        """Called by survey window to clear data"""
        self.survey_waypoints = []
        self.geofence_polygon = []
        logger.info("Survey data cleared")
        
        if MAP_AVAILABLE and hasattr(self, 'map_widget_ref'):
            self.map_widget_ref.clear_survey_viz()
        
        self.mission_status_var.set("Mission: IDLE")
    
    def set_geofence(self, polygon):
        """Called by survey window to sync geofence"""
        self.geofence_polygon = polygon
        logger.info(f"Geofence synced to main GCS: {len(polygon)} vertices")
        
        # Update map visualization
        if MAP_AVAILABLE and hasattr(self, 'map_widget_ref'):
            self.map_widget_ref.visualize_geofence(polygon)
    
    def set_survey_pattern(self, waypoints):
        """Called by survey window to sync pattern"""
        self.survey_waypoints = waypoints
        logger.info(f"Survey pattern synced to main GCS: {len(waypoints)} waypoints")
        
        # Update map visualization  
        if MAP_AVAILABLE and hasattr(self, 'map_widget_ref'):
            self.map_widget_ref.visualize_pattern(waypoints)
        
        self.mission_status_var.set(f"Mission: READY ({len(waypoints)} WPs)")
        
        messagebox.showinfo(
            "Pattern Loaded",
            f"Survey pattern loaded!\n\n"
            f"Waypoints: {len(waypoints)}\n\n"
            f"Press 'START MISSION' to begin."
        )
    
    def clear_survey_data(self):
        """Called by survey window to clear data"""
        self.survey_waypoints = []
        self.geofence_polygon = []
        logger.info("Survey data cleared from main GCS")
        
        # Clear map visualization
        if MAP_AVAILABLE and hasattr(self, 'map_widget_ref'):
            self.map_widget_ref.clear_survey_viz()
        
        self.mission_status_var.set("Mission: IDLE")
    
    def start_mission(self):
        """Start the automated mission"""
        if self.mission_active:
            messagebox.showwarning("Mission Active", "Mission is already running!")
            return
        
        if not self.survey_waypoints:
            messagebox.showerror(
                "No Pattern",
                "No survey pattern loaded!\n\n"
                "1. Open Survey Tool\n"
                "2. Load KML file\n"
                "3. Generate pattern\n"
                "4. Close survey window\n"
                "5. Press START MISSION"
            )
            return
        
        # Confirm mission start
        response = messagebox.askyesno(
            "Start Mission",
            f"Start automated mission?\n\n"
            f"Survey waypoints: {len(self.survey_waypoints)}\n"
            f"Max altitude: {self.max_altitude.get()}m\n\n"
            f"Drone 1 (Scout) will fly survey pattern and detect people.\n"
            f"Drone 2 (Delivery) will deliver payloads automatically.\n\n"
            f"Both drones will ARM and TAKEOFF automatically.\n\n"
            f"Continue?"
        )
        
        if not response:
            return
        
        # Start mission thread
        self.mission_active = True
        self.mission_status_var.set("Mission: ACTIVE")
        self.mission_thread = threading.Thread(target=self.run_mission, daemon=True)
        self.mission_thread.start()
        
        logger.info("🚀 MISSION STARTED")
    
    def run_mission(self):
        """Main mission control loop"""
        try:
            # 1. ARM and TAKEOFF both drones
            logger.info("Phase 1: ARM and TAKEOFF")
            self.mission_status_var.set("Mission: ARMING & TAKEOFF")
            
            max_alt = self.max_altitude.get()
            
            # ARM Drone 1
            self.drone1.set_mode("GUIDED")
            self.drone1.wait_for_mode("GUIDED")
            self.drone1.arm()
            self.drone1.wait_until_armed()
            
            # ARM Drone 2
            self.drone2.set_mode("GUIDED")
            self.drone2.wait_for_mode("GUIDED")
            self.drone2.arm()
            self.drone2.wait_until_armed()
            
            # Takeoff Drone 1
            self.drone1.takeoff(max_alt)
            time.sleep(2)
            
            # Takeoff Drone 2
            self.drone2.takeoff(max_alt)
            
            # Wait for both to reach altitude
            logger.info("Waiting for drones to reach altitude...")
            time.sleep(10)
            
            # 2. Start Scout Drone Survey
            logger.info("Phase 2: Scout survey pattern")
            self.mission_status_var.set("Mission: SCOUT SURVEYING")
            
            # Start delivery drone monitor in background
            threading.Thread(target=self.delivery_monitor, daemon=True).start()
            
            # Scout flies survey pattern
            self.drone1.fly_waypoints_guided(self.survey_waypoints, acceptance_radius=2.0)
            
            # 3. Survey complete
            logger.info("Phase 3: Survey complete")
            self.mission_status_var.set("Mission: SURVEY COMPLETE")
            
            # Wait for all deliveries to complete
            logger.info("Waiting for all deliveries to complete...")
            time.sleep(5)
            
            # 4. RTL both drones
            logger.info("Phase 4: RTL")
            self.mission_status_var.set("Mission: RTL")
            self.drone1.rtl()
            self.drone2.rtl()
            
            # Mission complete
            logger.info("✅ MISSION COMPLETE")
            self.mission_status_var.set("Mission: COMPLETE")
            self.mission_active = False
            
            messagebox.showinfo(
                "Mission Complete",
                "Mission completed successfully!\n\n"
                f"Survey waypoints: {len(self.survey_waypoints)}\n"
                f"People detected: {len(self.payload_queue.get_all_targets())}\n"
                f"Deliveries: {len([t for t in self.payload_queue.get_all_targets() if t[3]])}\n\n"
                "Both drones returning to launch."
            )
            
        except Exception as e:
            logger.error(f"Mission error: {e}")
            self.mission_status_var.set(f"Mission: ERROR - {str(e)}")
            self.mission_active = False
            messagebox.showerror("Mission Error", f"Mission failed:\n{str(e)}")
    
    def delivery_monitor(self):
        """Background thread to monitor and execute deliveries with servo payload drop"""
        logger.info("🚁 Delivery monitor started")
        
        while self.mission_active:
            try:
                # Get next undelivered target from queue
                target = self.payload_queue.get_next_undelivered()
                
                if target:
                    person_id, lat, lon = target
                    logger.info(f"📦 Starting delivery to Person {person_id} at ({lat:.6f}, {lon:.6f})")
                    
                    # Update mission status in UI
                    self.mission_status_var.set(f"Mission: DELIVERING to Person {person_id}")
                    
                    # Fly to target location
                    logger.info(f"Flying to Person {person_id}...")
                    self.drone2.goto(lat, lon, self.max_altitude.get(), duration=6)
                    
                    # Wait for arrival (within 5m radius)
                    timeout = time.time() + 60  # 60 second timeout
                    arrived = False
                    
                    while time.time() < timeout:
                        # Check if mission was aborted
                        if not self.mission_active:
                            logger.warning("⚠️ Mission aborted during delivery")
                            return
                        
                        # Check distance to target
                        dist = self.drone2.distance_to(lat, lon)
                        if dist is not None:
                            logger.debug(f"Distance to Person {person_id}: {dist:.1f}m")
                            
                            if dist < 5.0:  # Within 5 meters = arrived
                                logger.info(f"✅ Arrived at Person {person_id} (distance: {dist:.1f}m)")
                                arrived = True
                                break
                        
                        time.sleep(1)
                    
                    if arrived:
                        # Stabilization hover before drop
                        logger.info(f"Hovering to stabilize before payload drop...")
                        time.sleep(3)  # 3 second stabilization
                        
                        # DROP PAYLOAD VIA SERVO
                        logger.info(f"📦 Dropping payload to Person {person_id}")
                        drop_success = self.drone2.drop_payload()
                        
                        if drop_success:
                            # Mark as delivered in queue
                            self.payload_queue.mark_delivered(person_id)
                            logger.info(f"✅ Delivery complete to Person {person_id}")
                            
                            # Show UI notification (thread-safe)
                            self.after(0, lambda pid=person_id: messagebox.showinfo(
                                "Delivery Complete",
                                f"✅ Payload delivered to Person {pid}!\n\n"
                                f"Servo activated successfully."
                            ))
                            
                            # Brief pause before next delivery
                            time.sleep(3)
                        else:
                            logger.error(f"❌ Payload drop failed for Person {person_id}")
                            self.after(0, lambda pid=person_id: messagebox.showerror(
                                "Delivery Failed",
                                f"❌ Failed to drop payload to Person {pid}\n\n"
                                f"Check servo connection and try again."
                            ))
                    else:
                        logger.warning(f"⚠️ Timeout reaching Person {person_id} - Skipping delivery")
                        self.after(0, lambda pid=person_id: messagebox.showwarning(
                            "Delivery Timeout",
                            f"⚠️ Could not reach Person {pid} within 60 seconds\n\n"
                            f"Moving to next target."
                        ))
                
                else:
                    # No targets in queue - wait
                    time.sleep(2)
                    
            except Exception as e:
                logger.error(f"❌ Delivery monitor error: {e}")
                time.sleep(2)
        
        logger.info("🛑 Delivery monitor stopped")
    
    def emergency_stop(self):
        """Emergency stop - RTL both drones"""
        response = messagebox.askyesno(
            "Emergency Stop",
            "EMERGENCY STOP\n\n"
            "This will immediately command both drones to RTL.\n\n"
            "Continue?"
        )
        
        if response:
            logger.warning("🚨 EMERGENCY STOP ACTIVATED")
            self.mission_active = False
            self.mission_status_var.set("Mission: EMERGENCY STOP")
            self.drone1.rtl()
            self.drone2.rtl()
            messagebox.showinfo("Emergency Stop", "Both drones commanded to RTL")
    
    def center_map_on_drones(self):
        """Manually center map on drones"""
        if not MAP_AVAILABLE:
            return
        
        try:
            telem1 = self.drone1.get_telemetry()
            telem2 = self.drone2.get_telemetry()
            
            # Get valid positions
            positions = []
            if telem1['lat'] != 0 and telem1['lon'] != 0:
                positions.append((telem1['lat'], telem1['lon']))
            if telem2['lat'] != 0 and telem2['lon'] != 0:
                positions.append((telem2['lat'], telem2['lon']))
            
            if not positions:
                messagebox.showinfo("No GPS", "Drones don't have GPS lock yet.\nWaiting for position data...")
                return
            
            # Calculate center point
            avg_lat = sum(p[0] for p in positions) / len(positions)
            avg_lon = sum(p[1] for p in positions) / len(positions)
            
            # Center map
            self.map_widget_ref.map_widget.set_position(avg_lat, avg_lon)
            
            # Zoom based on distance between drones
            if len(positions) == 2:
                lat1, lon1 = positions[0]
                lat2, lon2 = positions[1]
                dist = math.sqrt((lat2 - lat1)**2 + (lon2 - lon1)**2)
                
                # Auto zoom based on distance
                if dist > 0.01:
                    zoom = 13
                elif dist > 0.005:
                    zoom = 14
                elif dist > 0.002:
                    zoom = 15
                else:
                    zoom = 17
                
                self.map_widget_ref.map_widget.set_zoom(zoom)
            else:
                self.map_widget_ref.map_widget.set_zoom(17)
            
            logger.info(f"Map centered on drones: {avg_lat:.6f}, {avg_lon:.6f}")
            
        except Exception as e:
            logger.error(f"Center map error: {e}")
            messagebox.showerror("Error", f"Failed to center map: {e}")

# -------------------------------
# Run App
# -------------------------------
if __name__ == "__main__":
    logger.info("Launching Enhanced GCS main loop")
    app = GCSApp()
    app.mainloop()

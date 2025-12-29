import tkinter as tk
from tkinter import ttk, messagebox
from pymavlink import mavutil
import threading
import time
import logging
from lawnmower_survey import SurveyWindow
import math
import queue

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
    
    def mark_delivered(self, person_id):
        """Mark a target as delivered"""
        with self.lock:
            if person_id in self.targets:
                self.targets[person_id]['delivered'] = True
    
    def get_all_targets(self):
        """Get all targets as a list"""
        with self.lock:
            return [(pid, t['lat'], t['lon'], t['delivered']) 
                    for pid, t in self.targets.items()]
    
    def get_undelivered_targets(self):
        """Get only undelivered targets"""
        with self.lock:
            return [(pid, t['lat'], t['lon']) 
                    for pid, t in self.targets.items() 
                    if not t['delivered']]
    
    def remove_target(self, person_id):
        """Remove a target from queue"""
        with self.lock:
            if person_id in self.targets:
                del self.targets[person_id]

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
        
        logger.info(f"{self.name} object created (Port={port}, Baud={baud})")

    def get_telemetry(self):
        """Thread-safe telemetry getter"""
        with self.telemetry_lock:
            return self.telemetry.copy()
    
    def update_telemetry(self, key, value):
        """Thread-safe telemetry update"""
        with self.telemetry_lock:
            self.telemetry[key] = value

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
                
                # STATUS TEXT
                elif msg_type == "STATUSTEXT":
                    text = msg.text.decode(errors="ignore")
                    logger.warning(f"[{self.name}] STATUS → {text}")
        
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
        self.target_markers = {}  # {person_id: marker}
        
        if MAP_AVAILABLE:
            # Create map
            self.map_widget = TkinterMapView(self, corner_radius=0)
            self.map_widget.pack(fill=tk.BOTH, expand=True)
            
            # Set default position (you can change this)
            self.map_widget.set_position(37.7749, -122.4194)  # San Francisco
            self.map_widget.set_zoom(15)
            
            # Start update loop
            self.update_map()
        else:
            ttk.Label(
                self, 
                text="Map Not Available\nInstall: pip install tkintermapview",
                font=("Arial", 12)
            ).pack(expand=True)
    
    def update_map(self):
        """Update drone and target positions on map"""
        try:
            # Update Drone 1
            telem1 = self.drone1.get_telemetry()
            if telem1['lat'] != 0 and telem1['lon'] != 0:
                if self.drone1_marker:
                    self.drone1_marker.delete()
                self.drone1_marker = self.map_widget.set_marker(
                    telem1['lat'], 
                    telem1['lon'],
                    text="Drone 1 (Scout)",
                    marker_color_circle="blue",
                    marker_color_outside="darkblue"
                )
            
            # Update Drone 2
            telem2 = self.drone2.get_telemetry()
            if telem2['lat'] != 0 and telem2['lon'] != 0:
                if self.drone2_marker:
                    self.drone2_marker.delete()
                self.drone2_marker = self.map_widget.set_marker(
                    telem2['lat'], 
                    telem2['lon'],
                    text="Drone 2 (Delivery)",
                    marker_color_circle="red",
                    marker_color_outside="darkred"
                )
            
            # Update target markers
            targets = self.payload_queue.get_all_targets()
            current_target_ids = set([pid for pid, _, _, _ in targets])
            
            # Remove deleted targets
            for pid in list(self.target_markers.keys()):
                if pid not in current_target_ids:
                    self.target_markers[pid].delete()
                    del self.target_markers[pid]
            
            # Add/update targets
            for person_id, lat, lon, delivered in targets:
                if person_id in self.target_markers:
                    self.target_markers[person_id].delete()
                
                color = "green" if delivered else "orange"
                outside_color = "darkgreen" if delivered else "darkorange"
                status = "✓ Delivered" if delivered else "Pending"
                
                self.target_markers[person_id] = self.map_widget.set_marker(
                    lat, lon,
                    text=f"Person {person_id} - {status}",
                    marker_color_circle=color,
                    marker_color_outside=outside_color
                )
            
            # Check for payload delivery (Drone 2 within 5m of target)
            for person_id, lat, lon, delivered in targets:
                if not delivered:
                    dist = self.drone2.distance_to(lat, lon)
                    if dist is not None and dist < 5.0:
                        logger.info(f"Payload delivered to Person {person_id}!")
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
        control_frame.pack(pady=5)
        
        ttk.Label(control_frame, text="Person ID:").grid(row=0, column=0, padx=5)
        self.person_id_var = tk.StringVar()
        ttk.Entry(control_frame, textvariable=self.person_id_var, width=10).grid(row=0, column=1, padx=5)
        
        ttk.Label(control_frame, text="Lat:").grid(row=0, column=2, padx=5)
        self.lat_var = tk.DoubleVar()
        ttk.Entry(control_frame, textvariable=self.lat_var, width=12).grid(row=0, column=3, padx=5)
        
        ttk.Label(control_frame, text="Lon:").grid(row=0, column=4, padx=5)
        self.lon_var = tk.DoubleVar()
        ttk.Entry(control_frame, textvariable=self.lon_var, width=12).grid(row=0, column=5, padx=5)
        
        ttk.Button(control_frame, text="Add/Update", command=self.add_target).grid(row=0, column=6, padx=5)
        ttk.Button(control_frame, text="Remove", command=self.remove_target).grid(row=0, column=7, padx=5)
        
        # Start update loop
        self.update_table()
    
    def add_target(self):
        """Add or update a target"""
        try:
            person_id = self.person_id_var.get()
            lat = self.lat_var.get()
            lon = self.lon_var.get()
            
            if not person_id:
                messagebox.showerror("Error", "Please enter a Person ID")
                return
            
            self.payload_queue.add_target(person_id, lat, lon)
            logger.info(f"Target added/updated: Person {person_id} at ({lat}, {lon})")
            
            # Clear entries
            self.person_id_var.set("")
            self.lat_var.set(0.0)
            self.lon_var.set(0.0)
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to add target: {e}")
    
    def remove_target(self):
        """Remove selected target"""
        selection = self.tree.selection()
        if not selection:
            messagebox.showwarning("Warning", "Please select a target to remove")
            return
        
        for item in selection:
            person_id = self.tree.item(item)['values'][0]
            self.payload_queue.remove_target(person_id)
            logger.info(f"Target removed: Person {person_id}")
    
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
                self.tree.insert("", tk.END, values=(person_id, f"{lat:.6f}", f"{lon:.6f}", status))
                
                # Color coding
                if delivered:
                    item_id = self.tree.get_children()[-1]
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
        
        # Create drones
        self.drone1 = Drone("COM5", 57600, "Drone1-Scout")
        self.drone2 = Drone("COM4", 57600, "Drone2-Delivery")
        
        # Create payload queue
        self.payload_queue = PayloadQueue()
        
        # Create main layout
        self.create_widgets()
        
        logger.info("GUI initialized successfully")

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
        
        # Connection buttons
        conn_frame = ttk.Frame(control_frame)
        conn_frame.pack(fill=tk.X, pady=5)
        ttk.Button(
            conn_frame, text="Connect Drone 1",
            command=lambda: threading.Thread(
                target=self.drone1.connect, daemon=True
            ).start()
        ).pack(side=tk.LEFT, padx=5)
        ttk.Button(
            conn_frame, text="Connect Drone 2",
            command=lambda: threading.Thread(
                target=self.drone2.connect, daemon=True
            ).start()
        ).pack(side=tk.LEFT, padx=5)
        
        # Quick actions
        ttk.Button(control_frame, text="ARM Both Drones", command=self.arm_both).pack(fill=tk.X, padx=5, pady=2)
        ttk.Button(control_frame, text="Takeoff Both", command=self.takeoff_both).pack(fill=tk.X, padx=5, pady=2)
        ttk.Button(control_frame, text="RTL Both", command=self.rtl_both).pack(fill=tk.X, padx=5, pady=2)
        ttk.Button(
            control_frame, 
            text="Open Survey Tool", 
            command=self.open_survey_window
        ).pack(fill=tk.X, padx=5, pady=2)
        
        # === RIGHT PANEL ===
        
        # Map
        map_label_frame = ttk.LabelFrame(right_panel, text="Mission Map")
        map_label_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        if MAP_AVAILABLE:
            MapWidget(map_label_frame, self.drone1, self.drone2, self.payload_queue).pack(
                fill=tk.BOTH, expand=True
            )
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
    def arm_both(self):
        logger.info("Arming both drones")
        threading.Thread(target=self._arm_drone, args=(self.drone1,), daemon=True).start()
        threading.Thread(target=self._arm_drone, args=(self.drone2,), daemon=True).start()
    
    def _arm_drone(self, drone):
        drone.set_mode("GUIDED")
        drone.wait_for_mode("GUIDED")
        drone.arm()
        drone.wait_until_armed()
    
    def takeoff_both(self):
        logger.info("Takeoff both drones")
        threading.Thread(target=self._takeoff_drone, args=(self.drone1, 6), daemon=True).start()
        threading.Thread(target=self._takeoff_drone, args=(self.drone2, 10), daemon=True).start()
    
    def _takeoff_drone(self, drone, alt):
        drone.set_mode("GUIDED")
        drone.wait_for_mode("GUIDED")
        drone.arm()
        drone.wait_until_armed()
        drone.takeoff(alt)
    
    def rtl_both(self):
        logger.info("RTL both drones")
        self.drone1.rtl()
        self.drone2.rtl()
    
    def open_survey_window(self):
        logger.info("Opening Lawnmower Survey Window")
        SurveyWindow(self, self.drone1, self.drone2)

# -------------------------------
# Run App
# -------------------------------
if __name__ == "__main__":
    logger.info("Launching Enhanced GCS main loop")
    app = GCSApp()
    app.mainloop()
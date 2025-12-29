import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import xml.etree.ElementTree as ET
import math
import threading
import time
import logging

logger = logging.getLogger("Survey")
logging.basicConfig(level=logging.INFO)

# Try to import tkintermapview
try:
    from tkintermapview import TkinterMapView
    MAP_AVAILABLE = True
except ImportError:
    MAP_AVAILABLE = False
    logger.warning("tkintermapview not available - map visualization disabled")

# =====================================================
# KML + LAWNMOWER LOGIC (FIXED)
# =====================================================
class LawnmowerPattern:

    @staticmethod
    def parse_kml(file_path):
        """Parse KML file and extract polygon coordinates"""
        try:
            tree = ET.parse(file_path)
            root = tree.getroot()
            ns = {"kml": "http://www.opengis.net/kml/2.2"}

            coords = []
            # Try to find coordinates in various KML structures
            for elem in root.findall(".//kml:coordinates", ns):
                for line in elem.text.strip().split():
                    parts = line.split(",")
                    if len(parts) >= 2:
                        lon = float(parts[0])
                        lat = float(parts[1])
                        coords.append((lat, lon))

            if len(coords) < 3:
                raise ValueError("Polygon needs at least 3 points")

            # Remove duplicate closing point if present
            if coords[0] == coords[-1]:
                coords.pop()

            logger.info(f"KML parsed: {len(coords)} vertices")
            return coords
            
        except Exception as e:
            logger.error(f"KML parsing error: {e}")
            raise

    @staticmethod
    def point_in_polygon(point, polygon):
        """Check if a point is inside a polygon using ray casting algorithm"""
        x, y = point
        n = len(polygon)
        inside = False

        p1_lat, p1_lon = polygon[0]
        for i in range(1, n + 1):
            p2_lat, p2_lon = polygon[i % n]
            if y > min(p1_lon, p2_lon):
                if y <= max(p1_lon, p2_lon):
                    if x <= max(p1_lat, p2_lat):
                        if p1_lon != p2_lon:
                            xinters = (y - p1_lon) * (p2_lat - p1_lat) / (p2_lon - p1_lon) + p1_lat
                        if p1_lat == p2_lat or x <= xinters:
                            inside = not inside
            p1_lat, p1_lon = p2_lat, p2_lon

        return inside

    @staticmethod
    def get_line_polygon_intersections(lat, polygon):
        """
        Get all intersection points of a horizontal line (constant latitude) with polygon edges
        Returns sorted list of longitudes where the line crosses polygon boundaries
        """
        intersections = []
        n = len(polygon)
        
        for i in range(n):
            p1 = polygon[i]
            p2 = polygon[(i + 1) % n]
            
            lat1, lon1 = p1
            lat2, lon2 = p2
            
            # Check if the line crosses this edge
            # The line crosses if lat is between lat1 and lat2
            if lat1 == lat2:
                # Edge is horizontal, skip it
                continue
            
            if min(lat1, lat2) <= lat <= max(lat1, lat2):
                # Calculate intersection longitude using linear interpolation
                t = (lat - lat1) / (lat2 - lat1)
                lon = lon1 + t * (lon2 - lon1)
                intersections.append(lon)
        
        return sorted(intersections)

    @staticmethod
    def lawnmower_from_polygon(polygon, spacing_m, altitude):
        """
        Generate lawnmower pattern waypoints inside a polygon
        
        Args:
            polygon: List of (lat, lon) tuples defining the boundary
            spacing_m: Distance between parallel lines in meters
            altitude: Flight altitude in meters
            
        Returns:
            List of (lat, lon, alt) waypoints
        """
        if len(polygon) < 3:
            raise ValueError("Polygon must have at least 3 vertices")
        
        # Earth radius for calculations
        R = 6371000.0
        
        # Get bounding box
        lats = [p[0] for p in polygon]
        lons = [p[1] for p in polygon]
        min_lat, max_lat = min(lats), max(lats)
        min_lon, max_lon = min(lons), max(lons)
        
        # Calculate center latitude for more accurate conversion
        center_lat = (min_lat + max_lat) / 2
        
        # Convert spacing from meters to degrees
        spacing_lat = spacing_m / R * (180 / math.pi)
        
        logger.info(f"Generating pattern: {min_lat:.6f} to {max_lat:.6f}, spacing={spacing_lat:.8f}°")
        
        waypoints = []
        flip = False  # Alternate direction for efficiency
        current_lat = min_lat
        
        # Sweep from south to north
        while current_lat <= max_lat:
            # Get all intersections of this latitude line with polygon
            intersections = LawnmowerPattern.get_line_polygon_intersections(current_lat, polygon)
            
            if len(intersections) >= 2:
                # Process pairs of intersections (entry/exit points)
                for j in range(0, len(intersections) - 1, 2):
                    lon_start = intersections[j]
                    lon_end = intersections[j + 1]
                    
                    # Verify these points are actually inside the polygon
                    midpoint_lon = (lon_start + lon_end) / 2
                    if LawnmowerPattern.point_in_polygon((current_lat, midpoint_lon), polygon):
                        # Create waypoint pair for this segment
                        if flip:
                            # Reverse direction on alternating passes
                            segment = [
                                (current_lat, lon_end, altitude),
                                (current_lat, lon_start, altitude)
                            ]
                        else:
                            segment = [
                                (current_lat, lon_start, altitude),
                                (current_lat, lon_end, altitude)
                            ]
                        waypoints.extend(segment)
            
            flip = not flip
            current_lat += spacing_lat
        
        logger.info(f"Generated {len(waypoints)} waypoints")
        
        if len(waypoints) == 0:
            raise ValueError("No waypoints generated - check polygon and spacing parameters")
        
        return waypoints

    @staticmethod
    def calculate_pattern_stats(waypoints):
        """Calculate statistics about the generated pattern"""
        if len(waypoints) < 2:
            return {"total_distance": 0, "num_waypoints": len(waypoints)}
        
        total_distance = 0
        R = 6371000.0  # Earth radius
        
        for i in range(len(waypoints) - 1):
            lat1, lon1, _ = waypoints[i]
            lat2, lon2, _ = waypoints[i + 1]
            
            # Haversine distance
            phi1 = math.radians(lat1)
            phi2 = math.radians(lat2)
            dphi = math.radians(lat2 - lat1)
            dlambda = math.radians(lon2 - lon1)
            
            a = math.sin(dphi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda/2)**2
            distance = 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))
            total_distance += distance
        
        return {
            "total_distance": total_distance,
            "num_waypoints": len(waypoints),
            "avg_segment": total_distance / (len(waypoints) - 1) if len(waypoints) > 1 else 0
        }


# =====================================================
# TELEMETRY WINDOW
# =====================================================
class TelemetryWindow(tk.Toplevel):
    def __init__(self, parent, drone):
        super().__init__(parent)
        self.title(f"Telemetry – {drone.name if hasattr(drone, 'name') else drone.port}")
        self.geometry("350x300")
        self.resizable(False, False)

        # Variables
        self.lat = tk.StringVar(value="--")
        self.lon = tk.StringVar(value="--")
        self.alt = tk.StringVar(value="--")
        self.wp = tk.StringVar(value="--")
        self.status = tk.StringVar(value="IDLE")
        self.distance = tk.StringVar(value="--")
        self.speed = tk.StringVar(value="--")

        # Main frame
        frame = ttk.LabelFrame(self, text="Live Telemetry", padding=10)
        frame.pack(fill="both", expand=True, padx=10, pady=10)

        # Status indicator (larger font, colored)
        status_frame = ttk.Frame(frame)
        status_frame.grid(row=0, column=0, columnspan=2, pady=(0, 10))
        ttk.Label(status_frame, text="Status:", font=("Arial", 10, "bold")).pack(side=tk.LEFT)
        self.status_label = ttk.Label(
            status_frame, 
            textvariable=self.status, 
            font=("Arial", 12, "bold"),
            foreground="blue"
        )
        self.status_label.pack(side=tk.LEFT, padx=5)

        # Telemetry rows
        row = 1
        def add_row(label, var, r):
            ttk.Label(frame, text=label + ":", font=("Arial", 9, "bold")).grid(
                row=r, column=0, sticky="w", pady=3
            )
            ttk.Label(frame, textvariable=var, font=("Arial", 9)).grid(
                row=r, column=1, sticky="e", pady=3, padx=(10, 0)
            )

        add_row("Latitude", self.lat, row); row += 1
        add_row("Longitude", self.lon, row); row += 1
        add_row("Altitude (m)", self.alt, row); row += 1
        add_row("Waypoint", self.wp, row); row += 1
        add_row("Distance (m)", self.distance, row); row += 1
        add_row("Speed (m/s)", self.speed, row); row += 1

        # Configure grid weights
        frame.columnconfigure(1, weight=1)

        self.protocol("WM_DELETE_WINDOW", self.withdraw)

    def set_status_color(self, color):
        """Update status label color"""
        self.after(0, lambda: self.status_label.config(foreground=color))


def telemetry_loop(window, drone, stop_event, wp_info, target_wp_func):
    """
    Background thread to continuously update telemetry window
    """
    def run():
        while not stop_event.is_set():
            try:
                msg = drone.master.recv_match(
                    type="GLOBAL_POSITION_INT",
                    blocking=True,
                    timeout=1
                )
                if not msg:
                    continue

                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                alt = msg.relative_alt / 1000

                i, total = wp_info()
                
                # Calculate distance to target
                target = target_wp_func()
                if target:
                    target_lat, target_lon = target
                    dist = drone.distance_to(target_lat, target_lon)
                    dist_str = f"{dist:.1f}" if dist else "--"
                else:
                    dist_str = "--"

                # Get speed
                speed_msg = drone.master.recv_match(type="VFR_HUD", blocking=False)
                speed_str = f"{speed_msg.groundspeed:.1f}" if speed_msg else "--"

                # Update UI (must be done in main thread)
                window.after(0, lambda: window.lat.set(f"{lat:.6f}"))
                window.after(0, lambda: window.lon.set(f"{lon:.6f}"))
                window.after(0, lambda: window.alt.set(f"{alt:.1f}"))
                window.after(0, lambda: window.wp.set(f"{i}/{total}"))
                window.after(0, lambda: window.distance.set(dist_str))
                window.after(0, lambda: window.speed.set(speed_str))

            except Exception as e:
                logger.error(f"Telemetry loop error: {e}")
                time.sleep(1)

    threading.Thread(target=run, daemon=True, name="Telemetry-Loop").start()


# =====================================================
# SURVEY WINDOW WITH MAP VISUALIZATION (ENHANCED)
# =====================================================
class SurveyWindow(tk.Toplevel):

    def __init__(self, parent, drone1, drone2):
        super().__init__(parent)
        self.title("Lawnmower Survey Tool")
        self.geometry("1200x800")  # Wider for map

        self.drone1 = drone1
        self.drone2 = drone2

        # Abort flags
        self.abort = {
            drone1: threading.Event(),
            drone2: threading.Event()
        }

        # Pattern data
        self.pattern_wps = []
        self.polygon = []
        self.current_wp = {drone1: None, drone2: None}
        
        # Map visualization objects
        self.polygon_path = None
        self.waypoint_markers = []

        self.create_ui()

    def create_ui(self):
        # Title
        title_label = ttk.Label(
            self, 
            text="🗺️ Lawnmower Survey Mission Planner", 
            font=("Arial", 16, "bold")
        )
        title_label.pack(pady=10)

        # Main container with map
        main_paned = ttk.PanedWindow(self, orient=tk.HORIZONTAL)
        main_paned.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Left panel - controls
        left_panel = ttk.Frame(main_paned)
        main_paned.add(left_panel, weight=1)
        
        # Right panel - map
        right_panel = ttk.Frame(main_paned)
        main_paned.add(right_panel, weight=1)

        # === LEFT PANEL ===
        
        # === POLYGON SECTION ===
        polygon_frame = ttk.LabelFrame(left_panel, text="1. Load Survey Area", padding=10)
        polygon_frame.pack(fill="x", padx=10, pady=5)

        ttk.Button(
            polygon_frame, 
            text="📁 Load KML File", 
            command=self.load_kml,
            width=20
        ).pack(side=tk.LEFT, padx=5)

        self.polygon_info = ttk.Label(polygon_frame, text="No polygon loaded")
        self.polygon_info.pack(side=tk.LEFT, padx=10)

        # === PATTERN PARAMETERS ===
        params_frame = ttk.LabelFrame(left_panel, text="2. Configure Pattern", padding=10)
        params_frame.pack(fill="x", padx=10, pady=5)

        # Spacing
        ttk.Label(params_frame, text="Line Spacing (m):").grid(row=0, column=0, sticky="w", pady=5)
        self.spacing = tk.DoubleVar(value=20)
        spacing_entry = ttk.Entry(params_frame, textvariable=self.spacing, width=10)
        spacing_entry.grid(row=0, column=1, padx=5, pady=5)
        ttk.Label(
            params_frame, 
            text="(Distance between parallel survey lines)", 
            font=("Arial", 8),
            foreground="gray"
        ).grid(row=0, column=2, sticky="w", padx=5)

        # Altitude
        ttk.Label(params_frame, text="Flight Altitude (m):").grid(row=1, column=0, sticky="w", pady=5)
        self.altitude = tk.DoubleVar(value=10)
        alt_entry = ttk.Entry(params_frame, textvariable=self.altitude, width=10)
        alt_entry.grid(row=1, column=1, padx=5, pady=5)
        ttk.Label(
            params_frame, 
            text="(Height above takeoff point)", 
            font=("Arial", 8),
            foreground="gray"
        ).grid(row=1, column=2, sticky="w", padx=5)

        # Generate button
        ttk.Button(
            params_frame, 
            text="⚙️ Generate Pattern", 
            command=self.generate_pattern,
            width=20
        ).grid(row=2, column=0, columnspan=3, pady=10)

        # Pattern stats
        self.stats_label = ttk.Label(
            params_frame, 
            text="No pattern generated", 
            font=("Arial", 9),
            foreground="blue"
        )
        self.stats_label.grid(row=3, column=0, columnspan=3)

        # === WAYPOINT TABLE ===
        table_frame = ttk.LabelFrame(left_panel, text="3. Generated Waypoints", padding=10)
        table_frame.pack(fill="both", expand=True, padx=10, pady=5)

        # Table with scrollbar
        table_container = ttk.Frame(table_frame)
        table_container.pack(fill="both", expand=True)

        self.table = ttk.Treeview(
            table_container, 
            columns=("index", "lat", "lon", "alt"), 
            show="headings",
            height=10
        )
        
        # Configure columns
        self.table.heading("index", text="#")
        self.table.heading("lat", text="Latitude")
        self.table.heading("lon", text="Longitude")
        self.table.heading("alt", text="Altitude (m)")
        
        self.table.column("index", width=40, anchor="center")
        self.table.column("lat", width=100, anchor="center")
        self.table.column("lon", width=100, anchor="center")
        self.table.column("alt", width=70, anchor="center")

        # Scrollbars
        vsb = ttk.Scrollbar(table_container, orient="vertical", command=self.table.yview)
        hsb = ttk.Scrollbar(table_container, orient="horizontal", command=self.table.xview)
        self.table.configure(yscrollcommand=vsb.set, xscrollcommand=hsb.set)

        self.table.grid(row=0, column=0, sticky="nsew")
        vsb.grid(row=0, column=1, sticky="ns")
        hsb.grid(row=1, column=0, sticky="ew")

        table_container.rowconfigure(0, weight=1)
        table_container.columnconfigure(0, weight=1)

        # === MISSION CONTROL ===
        control_frame = ttk.LabelFrame(left_panel, text="4. Execute Mission", padding=10)
        control_frame.pack(fill="x", padx=10, pady=5)

        # Drone controls in grid
        ttk.Label(control_frame, text="Drone 1:", font=("Arial", 10, "bold")).grid(row=0, column=0, sticky="w")
        ttk.Button(
            control_frame, 
            text="🚁 Start Survey", 
            command=lambda: self.start(self.drone1),
            width=18
        ).grid(row=1, column=0, padx=5, pady=2)
        ttk.Button(
            control_frame, 
            text="🏠 RTL", 
            command=lambda: self.force_rtl(self.drone1),
            width=18
        ).grid(row=2, column=0, padx=5, pady=2)

        ttk.Label(control_frame, text="Drone 2:", font=("Arial", 10, "bold")).grid(row=0, column=1, sticky="w")
        ttk.Button(
            control_frame, 
            text="🚁 Start Survey", 
            command=lambda: self.start(self.drone2),
            width=18
        ).grid(row=1, column=1, padx=5, pady=2)
        ttk.Button(
            control_frame, 
            text="🏠 RTL", 
            command=lambda: self.force_rtl(self.drone2),
            width=18
        ).grid(row=2, column=1, padx=5, pady=2)

        # === RIGHT PANEL - MAP ===
        if MAP_AVAILABLE:
            map_label_frame = ttk.LabelFrame(right_panel, text="Survey Area Visualization")
            map_label_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
            
            self.map_widget = TkinterMapView(map_label_frame, corner_radius=0)
            self.map_widget.pack(fill=tk.BOTH, expand=True)
            
            # Set default position
            self.map_widget.set_position(37.7749, -122.4194)
            self.map_widget.set_zoom(15)
            
            # Map controls
            map_control_frame = ttk.Frame(right_panel)
            map_control_frame.pack(fill="x", padx=5, pady=5)
            
            ttk.Button(
                map_control_frame,
                text="🔍 Fit to Survey Area",
                command=self.fit_map_to_polygon
            ).pack(side=tk.LEFT, padx=5)
            
            ttk.Button(
                map_control_frame,
                text="🗑️ Clear Map",
                command=self.clear_map_visualization
            ).pack(side=tk.LEFT, padx=5)
        else:
            ttk.Label(
                right_panel,
                text="Map not available\nInstall: pip install tkintermapview",
                font=("Arial", 12)
            ).pack(expand=True)

    def load_kml(self):
        """Load KML file and parse polygon"""
        try:
            path = filedialog.askopenfilename(
                title="Select KML File",
                filetypes=[("KML Files", "*.kml"), ("All Files", "*.*")]
            )
            if not path:
                return

            self.polygon = LawnmowerPattern.parse_kml(path)
            
            # Update info
            self.polygon_info.config(
                text=f"✓ Loaded: {len(self.polygon)} vertices",
                foreground="green"
            )
            
            # Clear existing pattern
            self.pattern_wps = []
            self.table.delete(*self.table.get_children())
            self.stats_label.config(text="Pattern needs regeneration")
            
            # Visualize polygon on map
            if MAP_AVAILABLE:
                self.visualize_polygon_on_map()
            
            messagebox.showinfo(
                "KML Loaded", 
                f"Successfully loaded polygon with {len(self.polygon)} vertices.\n"
                "The boundary is now shown on the map.\n\n"
                "Next: Generate the survey pattern."
            )
            
            logger.info(f"KML loaded: {len(self.polygon)} vertices")
            
        except Exception as e:
            logger.error(f"KML load error: {e}")
            messagebox.showerror("Error", f"Failed to load KML:\n{str(e)}")

    def visualize_polygon_on_map(self):
        """Draw polygon boundary on map"""
        if not MAP_AVAILABLE or not self.polygon:
            return
        
        try:
            # Clear previous polygon
            if self.polygon_path:
                self.polygon_path.delete()
            
            # Create coordinate list for polygon (need to close it)
            polygon_coords = [(lat, lon) for lat, lon in self.polygon]
            polygon_coords.append(self.polygon[0])  # Close the polygon
            
            # Draw polygon as path
            self.polygon_path = self.map_widget.set_path(
                polygon_coords,
                color="blue",
                width=3
            )
            
            # Add markers at vertices
            for i, (lat, lon) in enumerate(self.polygon):
                self.map_widget.set_marker(
                    lat, lon,
                    text=f"V{i+1}",
                    marker_color_circle="blue",
                    marker_color_outside="darkblue"
                )
            
            # Fit map to polygon
            self.fit_map_to_polygon()
            
            logger.info("Polygon visualized on map")
            
        except Exception as e:
            logger.error(f"Polygon visualization error: {e}")

    def fit_map_to_polygon(self):
        """Zoom/pan map to fit the polygon"""
        if not MAP_AVAILABLE or not self.polygon:
            return
        
        try:
            lats = [p[0] for p in self.polygon]
            lons = [p[1] for p in self.polygon]
            
            center_lat = (min(lats) + max(lats)) / 2
            center_lon = (min(lons) + max(lons)) / 2
            
            self.map_widget.set_position(center_lat, center_lon)
            
            # Calculate appropriate zoom level based on area size
            lat_span = max(lats) - min(lats)
            lon_span = max(lons) - min(lons)
            max_span = max(lat_span, lon_span)
            
            # Rough zoom calculation
            if max_span > 0.01:
                zoom = 13
            elif max_span > 0.005:
                zoom = 14
            elif max_span > 0.002:
                zoom = 15
            else:
                zoom = 16
            
            self.map_widget.set_zoom(zoom)
            
        except Exception as e:
            logger.error(f"Fit map error: {e}")

    def visualize_waypoints_on_map(self):
        """Draw waypoints on map"""
        if not MAP_AVAILABLE or not self.pattern_wps:
            return
        
        try:
            # Clear previous waypoint markers
            for marker in self.waypoint_markers:
                marker.delete()
            self.waypoint_markers.clear()
            
            # Draw waypoint path
            waypoint_coords = [(lat, lon) for lat, lon, _ in self.pattern_wps]
            
            # Draw the path showing the survey route
            self.map_widget.set_path(
                waypoint_coords,
                color="red",
                width=2
            )
            
            # Add markers for waypoints (show every Nth to avoid clutter)
            step = max(1, len(self.pattern_wps) // 20)  # Max 20 markers
            for i in range(0, len(self.pattern_wps), step):
                lat, lon, alt = self.pattern_wps[i]
                marker = self.map_widget.set_marker(
                    lat, lon,
                    text=f"WP{i+1}",
                    marker_color_circle="red",
                    marker_color_outside="darkred"
                )
                self.waypoint_markers.append(marker)
            
            # Mark start and end specially
            if len(self.pattern_wps) > 0:
                lat, lon, alt = self.pattern_wps[0]
                start_marker = self.map_widget.set_marker(
                    lat, lon,
                    text="START",
                    marker_color_circle="green",
                    marker_color_outside="darkgreen"
                )
                self.waypoint_markers.append(start_marker)
            
            if len(self.pattern_wps) > 1:
                lat, lon, alt = self.pattern_wps[-1]
                end_marker = self.map_widget.set_marker(
                    lat, lon,
                    text="END",
                    marker_color_circle="orange",
                    marker_color_outside="darkorange"
                )
                self.waypoint_markers.append(end_marker)
            
            logger.info(f"Visualized {len(self.pattern_wps)} waypoints on map")
            
        except Exception as e:
            logger.error(f"Waypoint visualization error: {e}")

    def clear_map_visualization(self):
        """Clear all map overlays"""
        if not MAP_AVAILABLE:
            return
        
        try:
            # Clear polygon
            if self.polygon_path:
                self.polygon_path.delete()
                self.polygon_path = None
            
            # Clear waypoint markers
            for marker in self.waypoint_markers:
                marker.delete()
            self.waypoint_markers.clear()
            
            # Note: Can't easily clear markers, but they'll be replaced on next load
            logger.info("Map visualization cleared")
            
        except Exception as e:
            logger.error(f"Clear map error: {e}")

    def generate_pattern(self):
        """Generate lawnmower pattern from loaded polygon"""
        if not self.polygon:
            messagebox.showwarning("No Polygon", "Please load a KML file first")
            return

        try:
            # Generate pattern
            self.pattern_wps = LawnmowerPattern.lawnmower_from_polygon(
                self.polygon, 
                self.spacing.get(), 
                self.altitude.get()
            )
            
            # Calculate statistics
            stats = LawnmowerPattern.calculate_pattern_stats(self.pattern_wps)
            
            # Update table
            self.table.delete(*self.table.get_children())
            for i, (lat, lon, alt) in enumerate(self.pattern_wps, 1):
                self.table.insert(
                    "", "end",
                    values=(i, f"{lat:.7f}", f"{lon:.7f}", f"{alt:.1f}")
                )
            
            # Update stats
            self.stats_label.config(
                text=f"✓ {stats['num_waypoints']} waypoints | "
                     f"Total: {stats['total_distance']:.0f}m | "
                     f"Avg: {stats['avg_segment']:.1f}m",
                foreground="green"
            )
            
            # Visualize waypoints on map
            if MAP_AVAILABLE:
                self.visualize_waypoints_on_map()
            
            logger.info(f"Pattern generated: {len(self.pattern_wps)} waypoints")
            
            messagebox.showinfo(
                "Pattern Generated",
                f"Survey pattern created and displayed on map!\n\n"
                f"Waypoints: {stats['num_waypoints']}\n"
                f"Total distance: {stats['total_distance']:.0f} meters\n"
                f"Estimated time: {stats['total_distance'] / 5 / 60:.1f} minutes (at 5 m/s)\n\n"
                f"Red lines show the flight path."
            )
            
        except Exception as e:
            logger.error(f"Pattern generation error: {e}")
            messagebox.showerror("Error", f"Failed to generate pattern:\n{str(e)}")

    def start(self, drone):
        """Start survey mission on specified drone"""
        if not self.pattern_wps:
            messagebox.showwarning("No Pattern", "Please generate a pattern first")
            return
        
        threading.Thread(target=self.fly, args=(drone,), daemon=True).start()

    def force_rtl(self, drone):
        """Force return to launch (abort mission)"""
        drone_name = drone.name if hasattr(drone, 'name') else drone.port
        logger.warning(f"[{drone_name}] RTL requested by user")
        self.abort[drone].set()
        drone.rtl()

    def fly(self, drone):
        """Execute survey mission (runs in background thread)"""
        self.abort[drone].clear()
        drone_name = drone.name if hasattr(drone, 'name') else drone.port
        
        # Open telemetry window
        telemetry = TelemetryWindow(self, drone)
        telemetry.status.set("INITIALIZING")
        telemetry.set_status_color("orange")
        telemetry.deiconify()

        # Current waypoint tracking
        current = {"i": 0, "target": None}

        # Start telemetry updates
        telemetry_loop(
            telemetry,
            drone,
            self.abort[drone],
            lambda: (current["i"], len(self.pattern_wps)),
            lambda: current["target"]
        )

        try:
            # Set GUIDED mode
            logger.info(f"[{drone_name}] Setting GUIDED mode")
            telemetry.status.set("MODE CHANGE")
            drone.set_mode("GUIDED")
            if not drone.wait_for_mode("GUIDED"):
                raise Exception("Failed to enter GUIDED mode")

            telemetry.status.set("SURVEY IN PROGRESS")
            telemetry.set_status_color("green")

            # Fly waypoints
            for i, (lat, lon, alt) in enumerate(self.pattern_wps, 1):
                if self.abort[drone].is_set():
                    logger.warning(f"[{drone_name}] Mission aborted by user")
                    telemetry.status.set("ABORTED - RTL")
                    telemetry.set_status_color("red")
                    return

                current["i"] = i
                current["target"] = (lat, lon)
                
                logger.info(f"[{drone_name}] Flying to WP {i}/{len(self.pattern_wps)}")
                
                # Send waypoint command
                drone.goto(lat, lon, alt, duration=3)

                # Wait for arrival (with timeout)
                start_time = time.time()
                timeout = 60  # 60 seconds per waypoint
                
                while time.time() - start_time < timeout:
                    if self.abort[drone].is_set():
                        logger.warning(f"[{drone_name}] Mission aborted during waypoint")
                        telemetry.status.set("ABORTED - RTL")
                        telemetry.set_status_color("red")
                        return
                    
                    dist = drone.distance_to(lat, lon)
                    if dist and dist < 2.0:  # Within 2 meters
                        logger.info(f"[{drone_name}] WP {i} reached (dist: {dist:.1f}m)")
                        break
                    
                    time.sleep(0.5)
                else:
                    logger.warning(f"[{drone_name}] WP {i} timeout - continuing")

            # Mission complete
            logger.info(f"[{drone_name}] Survey complete!")
            telemetry.status.set("COMPLETE - RTL")
            telemetry.set_status_color("blue")
            drone.rtl()

        except Exception as e:
            logger.error(f"[{drone_name}] Mission error: {e}")
            telemetry.status.set(f"ERROR: {str(e)}")
            telemetry.set_status_color("red")
            messagebox.showerror("Mission Error", f"Error during survey:\n{str(e)}")
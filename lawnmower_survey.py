import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import xml.etree.ElementTree as ET
import math
import logging

logger = logging.getLogger("Survey")

# Try to import tkintermapview
try:
    from tkintermapview import TkinterMapView
    MAP_AVAILABLE = True
except ImportError:
    MAP_AVAILABLE = False
    logger.warning("tkintermapview not available - map visualization disabled")

# =====================================================
# DJI OSMO 4 PRO CAMERA SPECIFICATIONS
# =====================================================
class CameraSpecs:
    """DJI Osmo Action 4 Pro Camera Specifications"""
    
    HFOV_DEGREES = 155  # Horizontal FOV (ultra-wide)
    VFOV_DEGREES = 118  # Vertical FOV
    
    @staticmethod
    def calculate_ground_coverage(altitude_m, overlap_percent=20):
        """Calculate ground coverage and optimal line spacing"""
        hfov_rad = math.radians(CameraSpecs.HFOV_DEGREES)
        vfov_rad = math.radians(CameraSpecs.VFOV_DEGREES)
        
        coverage_width = 2 * altitude_m * math.tan(hfov_rad / 2)
        coverage_height = 2 * altitude_m * math.tan(vfov_rad / 2)
        line_spacing = coverage_width * (1 - overlap_percent / 100)
        
        return {
            'coverage_width': coverage_width,
            'coverage_height': coverage_height,
            'line_spacing': line_spacing,
            'overlap_percent': overlap_percent,
            'altitude': altitude_m
        }

class LawnmowerPattern:

    @staticmethod
    def parse_kml(file_path):
        """Parse KML file and extract polygon coordinates"""
        try:
            tree = ET.parse(file_path)
            root = tree.getroot()
            namespaces = {'kml': 'http://www.opengis.net/kml/2.2'}
            coords = []
            
            coords_elements = root.findall(".//kml:coordinates", namespaces)
            if not coords_elements:
                coords_elements = root.findall(".//coordinates")
            
            for elem in coords_elements:
                text = elem.text.strip()
                lines = text.replace('\n', ' ').replace('\r', ' ').split()
                for line in lines:
                    line = line.strip()
                    if not line: continue
                    parts = line.split(",")
                    if len(parts) >= 2:
                        try:
                            lon = float(parts[0])
                            lat = float(parts[1])
                            coords.append((lat, lon))
                        except ValueError:
                            continue

            if len(coords) < 3:
                raise ValueError(f"Need at least 3 points, found {len(coords)}")
            if len(coords) > 1 and coords[0] == coords[-1]:
                coords.pop()
            return coords
        except Exception as e:
            logger.error(f"KML parsing error: {e}")
            raise

    # --- GEOMETRY HELPERS ---

    @staticmethod
    def latlon_to_meters(lat, lon, origin_lat, origin_lon):
        R = 6378137.0
        x = (math.radians(lon) - math.radians(origin_lon)) * math.cos(math.radians(origin_lat)) * R
        y = (math.radians(lat) - math.radians(origin_lat)) * R
        return x, y

    @staticmethod
    def meters_to_latlon(x, y, origin_lat, origin_lon):
        R = 6378137.0
        d_lat = y / R
        d_lon = x / (R * math.cos(math.radians(origin_lat)))
        return origin_lat + math.degrees(d_lat), origin_lon + math.degrees(d_lon)

    @staticmethod
    def rotate_point(x, y, angle_degrees):
        rad = math.radians(angle_degrees)
        cos_a = math.cos(rad)
        sin_a = math.sin(rad)
        return x * cos_a - y * sin_a, x * sin_a + y * cos_a

    @staticmethod
    def point_in_polygon_meters(x, y, poly_meters):
        inside = False
        n = len(poly_meters)
        p1x, p1y = poly_meters[0]
        for i in range(n + 1):
            p2x, p2y = poly_meters[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                            if p1x == p2x or x <= xinters:
                                inside = not inside
            p1x, p1y = p2x, p2y
        return inside

    @staticmethod
    def shrink_polygon_meters(poly_meters, buffer_m):
        if buffer_m <= 0: return poly_meters
        cx = sum(p[0] for p in poly_meters) / len(poly_meters)
        cy = sum(p[1] for p in poly_meters) / len(poly_meters)
        shrunk = []
        for x, y in poly_meters:
            dx = x - cx
            dy = y - cy
            dist = math.sqrt(dx*dx + dy*dy)
            if dist > buffer_m:
                factor = (dist - buffer_m) / dist
                shrunk.append((cx + dx*factor, cy + dy*factor))
            else:
                shrunk.append((cx, cy))
        return shrunk

    @staticmethod
    def get_longest_edge_angle(poly_meters):
        """Find the angle of the longest edge to align grid efficiently"""
        max_dist = 0
        best_angle = 0
        n = len(poly_meters)
        for i in range(n):
            p1 = poly_meters[i]
            p2 = poly_meters[(i + 1) % n]
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            dist = math.hypot(dx, dy)
            if dist > max_dist:
                max_dist = dist
                best_angle = math.degrees(math.atan2(dy, dx))
        return best_angle

    @staticmethod
    def get_intersections(y_line, poly_meters):
        intersections = []
        n = len(poly_meters)
        for i in range(n):
            p1 = poly_meters[i]
            p2 = poly_meters[(i + 1) % n]
            if min(p1[1], p2[1]) < y_line <= max(p1[1], p2[1]):
                if abs(p2[1] - p1[1]) > 1e-9:
                    x = p1[0] + (y_line - p1[1]) * (p2[0] - p1[0]) / (p2[1] - p1[1])
                    intersections.append(x)
        return sorted(intersections)

    @staticmethod
    def lawnmower_from_polygon(polygon, spacing_m, altitude, geofence_buffer_m=0.0):
        if len(polygon) < 3: raise ValueError("Polygon < 3 vertices")
            
        logger.info(f"GENERATING: Alt={altitude}m, Space={spacing_m}m")

        # 1. Project to Meters
        avg_lat = sum(p[0] for p in polygon) / len(polygon)
        avg_lon = sum(p[1] for p in polygon) / len(polygon)
        poly_meters = [LawnmowerPattern.latlon_to_meters(lat, lon, avg_lat, avg_lon) for lat, lon in polygon]

        # 2. AUTO-ROTATION (Align with longest edge)
        # This makes lines parallel to the longest boundary = Fewer turns = Max Efficiency
        rotation_angle = -LawnmowerPattern.get_longest_edge_angle(poly_meters)
        logger.info(f"Optimal rotation: {rotation_angle:.1f} deg")
        
        rotated_poly = [LawnmowerPattern.rotate_point(x, y, rotation_angle) for x, y in poly_meters]
        work_poly = LawnmowerPattern.shrink_polygon_meters(rotated_poly, geofence_buffer_m)

        if not work_poly: raise ValueError("Buffer too large")

        # 3. CALCULATE CENTROID & SPAN
        min_y = min(p[1] for p in work_poly)
        max_y = max(p[1] for p in work_poly)
        center_y = (min_y + max_y) / 2
        total_height = max_y - min_y

        if total_height < 0.1: raise ValueError("Area too small")

        # 4. CENTROID ALIGNMENT (The "Middle of Lanes" Fix)
        # Determine how many lines fit
        num_lines = max(1, math.ceil(total_height / spacing_m))
        
        # Calculate total grid height
        grid_height = (num_lines - 1) * spacing_m
        
        # Start Y so the grid is perfectly centered
        start_y = center_y - (grid_height / 2)
        
        logger.info(f"Centering: PolyH={total_height:.1f}m, Lines={num_lines}, StartOffset={start_y - min_y:.1f}m")

        # 5. Generate Waypoints
        waypoints_rotated = []
        current_y = start_y
        flip = False

        for _ in range(num_lines):
            intersections = LawnmowerPattern.get_intersections(current_y, work_poly)
            
            # Clean up intersections (group by proximity to avoid tiny zig-zags on vertices)
            if len(intersections) >= 2:
                # Basic logic: take min and max x for this line (simple sweep)
                # Or handle complex polygons (holes):
                for i in range(0, len(intersections) - 1, 2):
                    x1, x2 = intersections[i], intersections[i+1]
                    
                    # Ensure we are actually inside (handle concave)
                    mid = (x1 + x2)/2
                    if LawnmowerPattern.point_in_polygon_meters(mid, current_y, work_poly):
                        if flip:
                            waypoints_rotated.extend([(x2, current_y), (x1, current_y)])
                        else:
                            waypoints_rotated.extend([(x1, current_y), (x2, current_y)])
            
            current_y += spacing_m
            flip = not flip

        # 6. Un-rotate and Reproject
        final_waypoints = []
        for wx, wy in waypoints_rotated:
            ox, oy = LawnmowerPattern.rotate_point(wx, wy, -rotation_angle)
            lat, lon = LawnmowerPattern.meters_to_latlon(ox, oy, avg_lat, avg_lon)
            final_waypoints.append((lat, lon, altitude))

        return final_waypoints

    # --- THIS WAS MISSING AND CAUSED THE ERROR ---
    @staticmethod
    def calculate_polygon_size(polygon):
        """Estimate size for UI display"""
        if not polygon: return {'lat_span': 0, 'lon_span': 0}
        
        lats = [p[0] for p in polygon]
        lons = [p[1] for p in polygon]
        avg_lat = sum(lats) / len(lats)
        
        # Approximate conversion (1 deg lat ~ 111.32km)
        lat_span = (max(lats) - min(lats)) * 111320
        lon_span = (max(lons) - min(lons)) * 111320 * math.cos(math.radians(avg_lat))
        
        return {
            'lat_span': lat_span,
            'lon_span': lon_span,
        }

    @staticmethod
    def calculate_pattern_stats(waypoints):
        if len(waypoints) < 2: return {"total_distance": 0, "num_waypoints": len(waypoints)}
        total_dist = 0
        R = 6378137.0
        for i in range(len(waypoints)-1):
            lat1, lon1, _ = waypoints[i]
            lat2, lon2, _ = waypoints[i+1]
            a = math.sin(math.radians(lat2-lat1)/2)**2 + math.cos(math.radians(lat1))*math.cos(math.radians(lat2))*math.sin(math.radians(lon2-lon1)/2)**2
            total_dist += R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        return {"total_distance": total_dist, "num_waypoints": len(waypoints)}
    
    
# =====================================================
# SURVEY WINDOW
# =====================================================
class SurveyWindow(tk.Toplevel):

    def __init__(self, parent, drone1, drone2, max_altitude=6.0):
        super().__init__(parent)
        self.title("Survey Pattern Generator")
        self.geometry("1200x800")
        
        self.parent = parent
        self.drone1 = drone1
        self.drone2 = drone2
        self.max_altitude = max_altitude

        self.pattern_wps = []
        self.polygon = []
        
        self.polygon_path = None
        self.pattern_path = None
        self.waypoint_markers = []

        self.create_ui()

    def create_ui(self):
        # Title
        ttk.Label(self, text="🗺️ Survey Pattern Generator", font=("Arial", 16, "bold")).pack(pady=10)
        ttk.Label(self, text="Generate pattern here. It will sync to main GCS automatically.", 
                  font=("Arial", 10, "italic"), foreground="blue").pack(pady=5)

        # Main layout
        main_paned = ttk.PanedWindow(self, orient=tk.HORIZONTAL)
        main_paned.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        left_panel = ttk.Frame(main_paned)
        main_paned.add(left_panel, weight=1)
        
        right_panel = ttk.Frame(main_paned)
        main_paned.add(right_panel, weight=1)

        # === LEFT PANEL ===
        
        # Load KML
        polygon_frame = ttk.LabelFrame(left_panel, text="1. Load Geofence", padding=10)
        polygon_frame.pack(fill="x", padx=10, pady=5)

        ttk.Button(polygon_frame, text="📁 Load KML", command=self.load_kml, width=20).pack(side=tk.LEFT, padx=5)
        self.polygon_info = ttk.Label(polygon_frame, text="No polygon loaded", foreground="gray")
        self.polygon_info.pack(side=tk.LEFT, padx=10)

        # Parameters
        params_frame = ttk.LabelFrame(left_panel, text="2. Settings", padding=10)
        params_frame.pack(fill="x", padx=10, pady=5)

        ttk.Label(params_frame, text="Altitude (m):").grid(row=0, column=0, sticky="w", pady=5, padx=5)
        self.altitude = tk.DoubleVar(value=6.0)
        ttk.Entry(params_frame, textvariable=self.altitude, width=10).grid(row=0, column=1, padx=5)

        ttk.Label(params_frame, text="Overlap (%):").grid(row=1, column=0, sticky="w", pady=5, padx=5)
        self.overlap = tk.DoubleVar(value=20)
        ttk.Entry(params_frame, textvariable=self.overlap, width=10).grid(row=1, column=1, padx=5)

        ttk.Label(params_frame, text="Buffer (m):").grid(row=2, column=0, sticky="w", pady=5, padx=5)
        self.geofence_buffer = tk.DoubleVar(value=0.0)
        ttk.Entry(params_frame, textvariable=self.geofence_buffer, width=10).grid(row=2, column=1, padx=5)
        
        # Quick buttons
        buffer_btn_frame = ttk.Frame(params_frame)
        buffer_btn_frame.grid(row=3, column=0, columnspan=3, pady=5)
        ttk.Label(buffer_btn_frame, text="Quick:", font=("Arial", 8)).pack(side=tk.LEFT, padx=5)
        ttk.Button(buffer_btn_frame, text="0m", width=6, command=lambda: self.geofence_buffer.set(0.0)).pack(side=tk.LEFT, padx=2)
        ttk.Button(buffer_btn_frame, text="0.5m", width=6, command=lambda: self.geofence_buffer.set(0.5)).pack(side=tk.LEFT, padx=2)
        ttk.Button(buffer_btn_frame, text="1m", width=6, command=lambda: self.geofence_buffer.set(1.0)).pack(side=tk.LEFT, padx=2)

        self.spacing_info = ttk.Label(params_frame, text="Auto-calculated spacing", 
                                      font=("Arial", 9, "italic"), foreground="green")
        self.spacing_info.grid(row=4, column=0, columnspan=3, pady=10)

        ttk.Button(params_frame, text="⚙️ GENERATE", command=self.generate_pattern, width=20).grid(row=5, column=0, columnspan=3, pady=10)

        self.stats_label = ttk.Label(params_frame, text="No pattern", font=("Arial", 9), foreground="blue")
        self.stats_label.grid(row=6, column=0, columnspan=3)

        # Table
        table_frame = ttk.LabelFrame(left_panel, text="3. Waypoints", padding=10)
        table_frame.pack(fill="both", expand=True, padx=10, pady=5)

        self.table = ttk.Treeview(table_frame, columns=("idx", "lat", "lon", "alt"), 
                                  show="headings", height=8)
        self.table.heading("idx", text="#")
        self.table.heading("lat", text="Lat")
        self.table.heading("lon", text="Lon")
        self.table.heading("alt", text="Alt")
        self.table.column("idx", width=40)
        self.table.column("lat", width=100)
        self.table.column("lon", width=100)
        self.table.column("alt", width=60)
        self.table.pack(fill=tk.BOTH, expand=True)

        ttk.Button(table_frame, text="🗑️ Clear All", command=self.clear_all).pack(pady=5)

        # === RIGHT PANEL - MAP ===
        if MAP_AVAILABLE:
            self.map_widget = TkinterMapView(right_panel, corner_radius=0)
            self.map_widget.pack(fill=tk.BOTH, expand=True)
            self.map_widget.set_position(37.7749, -122.4194)
            self.map_widget.set_zoom(15)
        else:
            ttk.Label(right_panel, text="Map unavailable", font=("Arial", 12)).pack(expand=True)

    def load_kml(self):
        try:
            path = filedialog.askopenfilename(title="Select KML", filetypes=[("KML", "*.kml")])
            if not path:
                return

            self.polygon = LawnmowerPattern.parse_kml(path)
            size_info = LawnmowerPattern.calculate_polygon_size(self.polygon)
            
            self.polygon_info.config(
                text=f"✓ {len(self.polygon)} vertices ({size_info['lat_span']:.1f}m x {size_info['lon_span']:.1f}m)",
                foreground="green"
            )
            
            if MAP_AVAILABLE:
                self.visualize_polygon()
            
            # Sync to main GCS
            if hasattr(self.parent, 'set_geofence'):
                self.parent.set_geofence(self.polygon)
            
            messagebox.showinfo("Loaded", f"Geofence: {size_info['lat_span']:.1f}m x {size_info['lon_span']:.1f}m")
            
        except Exception as e:
            logger.error(f"Load error: {e}")
            messagebox.showerror("Error", str(e))

    def generate_pattern(self):
        if not self.polygon:
            messagebox.showwarning("No Polygon", "Load KML first")
            return

        try:
            alt = self.altitude.get()
            if alt <= 0 or alt > self.max_altitude:
                messagebox.showerror("Invalid", f"Altitude: 0 < alt <= {self.max_altitude}m")
                return
            
            # Calculate spacing
            cam = CameraSpecs.calculate_ground_coverage(alt, self.overlap.get())
            spacing = cam['line_spacing']
            
            self.spacing_info.config(text=f"✓ Spacing: {spacing:.1f}m")
            
            # Generate
            self.pattern_wps = LawnmowerPattern.lawnmower_from_polygon(
                self.polygon, spacing, alt, self.geofence_buffer.get()
            )
            
            stats = LawnmowerPattern.calculate_pattern_stats(self.pattern_wps)
            
            # Update table
            self.table.delete(*self.table.get_children())
            for i, (lat, lon, a) in enumerate(self.pattern_wps, 1):
                self.table.insert("", "end", values=(i, f"{lat:.7f}", f"{lon:.7f}", f"{a:.1f}"))
            
            self.stats_label.config(
                text=f"✓ {stats['num_waypoints']} WPs | {stats['total_distance']:.0f}m",
                foreground="green"
            )
            
            if MAP_AVAILABLE:
                self.visualize_pattern()
            
            # Sync to main GCS
            if hasattr(self.parent, 'set_survey_pattern'):
                self.parent.set_survey_pattern(self.pattern_wps)
            
            messagebox.showinfo("Success", 
                f"Pattern: {stats['num_waypoints']} waypoints\n"
                f"Distance: {stats['total_distance']:.0f}m\n"
                f"Time: ~{stats['total_distance']/5/60:.1f} min")
            
        except Exception as e:
            logger.error(f"Generate error: {e}")
            messagebox.showerror("Error", str(e))

    def clear_all(self):
        if messagebox.askyesno("Clear", "Clear everything?"):
            self.polygon = []
            self.pattern_wps = []
            self.table.delete(*self.table.get_children())
            self.polygon_info.config(text="No polygon", foreground="gray")
            self.stats_label.config(text="No pattern", foreground="blue")
            
            if MAP_AVAILABLE:
                self.clear_map()
            
            if hasattr(self.parent, 'clear_survey_data'):
                self.parent.clear_survey_data()
            
            messagebox.showinfo("Cleared", "All data cleared")

    def visualize_polygon(self):
        if not MAP_AVAILABLE:
            return
        try:
            if self.polygon_path:
                self.polygon_path.delete()
            coords = [(lat, lon) for lat, lon in self.polygon]
            coords.append(self.polygon[0])
            self.polygon_path = self.map_widget.set_path(coords, color="red", width=3)
            self.fit_map()
        except Exception as e:
            logger.error(f"Viz error: {e}")

    def visualize_pattern(self):
        if not MAP_AVAILABLE:
            return
        try:
            for m in self.waypoint_markers:
                try:
                    m.delete()
                except:
                    pass
            self.waypoint_markers.clear()
            
            if self.pattern_path:
                self.pattern_path.delete()
            
            coords = [(lat, lon) for lat, lon, _ in self.pattern_wps]
            self.pattern_path = self.map_widget.set_path(coords, color="blue", width=2)
            
            if len(self.pattern_wps) > 0:
                lat, lon, _ = self.pattern_wps[0]
                m = self.map_widget.set_marker(lat, lon, text="START", marker_color_circle="green")
                self.waypoint_markers.append(m)
            
            if len(self.pattern_wps) > 1:
                lat, lon, _ = self.pattern_wps[-1]
                m = self.map_widget.set_marker(lat, lon, text="END", marker_color_circle="orange")
                self.waypoint_markers.append(m)
        except Exception as e:
            logger.error(f"Pattern viz error: {e}")

    def fit_map(self):
        if not MAP_AVAILABLE or not self.polygon:
            return
        try:
            lats = [p[0] for p in self.polygon]
            lons = [p[1] for p in self.polygon]
            self.map_widget.set_position((min(lats)+max(lats))/2, (min(lons)+max(lons))/2)
            self.map_widget.set_zoom(16)
        except:
            pass

    def clear_map(self):
        if not MAP_AVAILABLE:
            return
        try:
            if self.polygon_path:
                self.polygon_path.delete()
            if self.pattern_path:
                self.pattern_path.delete()
            for m in self.waypoint_markers:
                try:
                    m.delete()
                except:
                    pass
            self.waypoint_markers.clear()
        except:
            pass

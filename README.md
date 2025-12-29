# Enhanced Dual Drone Ground Control Station (GCS)

## Overview
This enhanced GCS provides a comprehensive mission control dashboard for managing two drones with real-time telemetry, map visualization, and payload delivery tracking.

## New Features

### 1. **Real-Time Dashboard**
- Live telemetry display for both drones
- Shows: Mode, Armed status, GPS position, Altitude, Heading, Speed, Battery, GPS fix
- Auto-updating every 500ms

### 2. **Interactive Mission Map**
- Real-time drone position tracking
- Drone 1 (Scout) - Blue marker
- Drone 2 (Delivery) - Red marker
- Human target markers (Orange = Pending, Green = Delivered)
- Automatic payload delivery detection (within 5m radius)

### 3. **Payload Delivery Queue**
- Add/Update/Remove human targets
- Two-column table: Person ID | Coordinates (Lat, Long) | Status
- Automatic status updates when payload delivered
- Color-coded: Green background for delivered targets

### 4. **Automatic Delivery Detection**
- When Drone 2 comes within 5 meters of a target, the system automatically:
  - Marks the target as delivered
  - Changes marker color from orange to green
  - Updates the queue table
  - Logs the delivery event

## Installation

### Required Dependencies
```bash
# Core dependencies
pip install pymavlink tkinter

# Map visualization (REQUIRED for map features)
pip install tkintermapview

# If tkintermapview installation fails, try:
pip install tkintermapview --upgrade
```

## File Structure
```
project/
├── gcs_enhanced.py          # Main enhanced GCS
├── lawnmower_survey.py      # Survey planning tool (existing)
└── README.md                # This file
```

## Usage

### Starting the GCS
```bash
python gcs_enhanced.py
```

### Basic Workflow

1. **Connect Drones**
   - Click "Connect Drone 1" and "Connect Drone 2"
   - Wait for heartbeat confirmation in logs

2. **Add Delivery Targets**
   - In the "Delivery Targets" section:
   - Enter Person ID (e.g., "P001", "1", "Person-A")
   - Enter Latitude and Longitude
   - Click "Add/Update"
   - Target appears on map with orange marker

3. **Mission Execution**
   - Use "ARM Both Drones" to arm
   - Use "Takeoff Both" to initiate takeoff
   - Drone 1 (Scout) can survey the area
   - Drone 2 (Delivery) flies to targets

4. **Automatic Delivery**
   - When Drone 2 reaches within 5m of a target:
     - Marker turns GREEN
     - Status changes to "✓ Delivered"
     - Table row highlighted in green

5. **Target Management**
   - **Update coordinates**: Enter same Person ID with new coordinates
   - **Remove target**: Select in table, click "Remove"
   - Targets persist until manually removed

### Dashboard Information

**Drone Status Panel** shows:
- **Mode**: Current flight mode (GUIDED, RTL, STABILIZE, etc.)
- **Armed**: ARMED (red) / DISARMED (gray)
- **Position**: GPS coordinates (Lat, Lon)
- **Altitude**: Relative altitude in meters
- **Heading**: Direction in degrees (0-360°)
- **Speed**: Ground speed in m/s
- **Battery**: Voltage and percentage remaining
- **GPS**: Fix type and satellite count

## Configuration

### Port Settings
Edit in `gcs_enhanced.py`:
```python
self.drone1 = Drone("COM5", 57600, "Drone1-Scout")
self.drone2 = Drone("COM4", 57600, "Drone2-Delivery")
```

### Delivery Detection Radius
Default is 5 meters. To change, edit in `MapWidget.update_map()`:
```python
if dist is not None and dist < 5.0:  # Change 5.0 to desired radius
```

### Map Default Location
Edit in `MapWidget.__init__()`:
```python
self.map_widget.set_position(37.7749, -122.4194)  # Your coordinates
self.map_widget.set_zoom(15)  # Zoom level
```

## Features in Detail

### Payload Queue System

**Data Structure:**
```python
{
    "person_id": {
        "lat": float,
        "lon": float,
        "delivered": bool
    }
}
```

**Thread-Safe Operations:**
- All queue operations use locks for thread safety
- Multiple drones can access queue simultaneously
- Real-time updates across all widgets

**Queue Methods:**
```python
payload_queue.add_target(person_id, lat, lon)        # Add or update
payload_queue.mark_delivered(person_id)              # Mark as delivered
payload_queue.get_undelivered_targets()              # Get pending only
payload_queue.remove_target(person_id)               # Remove target
```

### Telemetry System

**Auto-Updating Fields:**
- Position updates from GLOBAL_POSITION_INT messages
- Speed from VFR_HUD messages
- Battery from SYS_STATUS messages
- GPS status from GPS_RAW_INT messages
- Mode/Armed from HEARTBEAT messages

**Thread Safety:**
- All telemetry access is protected by locks
- Background listener thread continuously updates data
- GUI widgets read thread-safe copies

## Troubleshooting

### Map Not Showing
```bash
# Install tkintermapview
pip install tkintermapview

# If that fails, try:
pip install tkintermapview --force-reinstall

# On Linux, you might need:
sudo apt-get install python3-pil python3-pil.imagetk
```

### Drones Not Connecting
- Check COM port numbers
- Verify baud rate (default: 57600)
- Ensure SITL/real drone is running
- Check logs for connection errors

### Targets Not Updating
- Verify GPS lock on both drones
- Check that coordinates are valid
- Ensure map is displaying correctly
- Check logs for errors

### Delivery Not Detected
- Verify Drone 2 has valid GPS position
- Check distance calculation in logs
- Ensure target coordinates are accurate
- Try increasing detection radius (default 5m)

## Advanced Usage

### Simulating Telemetry (for testing without drones)
You can manually update telemetry for testing:
```python
# In Python console or test script
drone1.update_telemetry('lat', 37.7749)
drone1.update_telemetry('lon', -122.4194)
drone1.update_telemetry('mode', 'GUIDED')
drone1.update_telemetry('armed', True)
```

### Custom Delivery Logic
Edit `MapWidget.update_map()` to customize delivery detection:
```python
# Example: Require altitude check
if dist is not None and dist < 5.0:
    telem = self.drone2.get_telemetry()
    if telem['relative_alt'] < 3.0:  # Must be below 3m
        self.payload_queue.mark_delivered(person_id)
```

### Integrating with Mission Planner
You can receive target coordinates from external sources:
```python
# Example: Receiving targets via socket/serial
def receive_target_from_telemetry():
    # Your telemetry parsing code here
    person_id, lat, lon = parse_incoming_message()
    payload_queue.add_target(person_id, lat, lon)
```

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    GCS Application                       │
├─────────────────────────────────────────────────────────┤
│                                                           │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │  Dashboard   │  │  Map Widget  │  │ Queue Widget │  │
│  │   (Drone1)   │  │              │  │              │  │
│  └──────────────┘  └──────────────┘  └──────────────┘  │
│                                                           │
│  ┌──────────────┐                                        │
│  │  Dashboard   │     ┌────────────────────┐            │
│  │   (Drone2)   │◄────┤  PayloadQueue      │            │
│  └──────────────┘     │  (Thread-Safe)     │            │
│                       └────────────────────┘            │
│                                                           │
│  ┌────────────────────────────────────────────────────┐ │
│  │           MAVLink Telemetry Listeners              │ │
│  │  (Background threads monitoring both drones)       │ │
│  └────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────┘
                           │
                           ▼
              ┌────────────────────────┐
              │   MAVLink Connection   │
              │   (pymavlink)          │
              └────────────────────────┘
                     │           │
                     ▼           ▼
              ┌─────────┐ ┌─────────┐
              │ Drone 1 │ │ Drone 2 │
              │ (Scout) │ │(Delivery)│
              └─────────┘ └─────────┘
```

## Future Enhancements

Potential additions to consider:
1. **Mission Recording**: Save/load missions with waypoints and targets
2. **Path Planning**: Automatic route optimization for deliveries
3. **Multi-Drone Support**: Scale to 3+ drones
4. **Video Feed**: Integrate camera streams
5. **Geofencing**: Define safe flight areas
6. **Battery Alerts**: Warning when battery is low
7. **Weather Integration**: Display wind/weather data
8. **Export Reports**: Generate mission logs and statistics

## License
This is open-source software. Modify and use as needed for your drone missions.

## Support
For issues or questions, check the logs (console output) for detailed debugging information.

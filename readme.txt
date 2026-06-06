DroneGuard v4 - MAVLink Telemetry Dashboard

A real-time web-based telemetry dashboard for monitoring MAVLink-enabled drones (e.g., ArduPilot-based vehicles). This application provides comprehensive flight data visualization including attitude, IMU sensors, GPS positioning, battery status, RC channels, and vehicle control capabilities.

Features

Real-time Telemetry Streaming: Connects to drone via MAVLink protocol and streams data at configurable rates (default 20Hz)
Web Dashboard: Modern, responsive web interface with dark theme and ambient particle effects
Attitude Visualization: 3D aircraft model showing roll, pitch, and yaw in real-time
IMU Graphs: Live graphs for accelerometer and gyroscope data
GPS Mapping: Interactive Leaflet map with drone position tracking
Flight Parameters: Altitude, speed, heading, battery voltage/current/percentage
RC Channels: Real-time display of transmitter inputs (8 channels)
Vehicle Control: Arm/disarm, mode changes, takeoff, and RTL commands
Status Monitoring: Connection status, flight mode, EKF health, satellite count
Simulation Mode: Fallback simulation when drone is disconnected

Architecture

Backend: Python WebSocket server using pymavlink for MAVLink communication
Frontend: Pure JavaScript with HTML5 Canvas for 3D rendering and Leaflet for maps
Communication: WebSocket for real-time data streaming between server and browser

Prerequisites

Python 3.7+
MAVLink-enabled drone (ArduPilot, PX4, etc.)
Serial connection to drone (USB/serial adapter)

Installation

1. Clone or download the project files to your local machine

2. Install Python dependencies:
   pip install pymavlink websockets

3. Configure connection (optional):
   Set environment variables for drone connection:
   DRONE_CONNECTION: Serial port (default: COM18 on Windows)
   DRONE_BAUD: Baud rate (default: 9600)
   WS_HOST: WebSocket host (default: 0.0.0.0)
   WS_PORT: WebSocket port (default: 8765)
   STREAM_HZ: Telemetry stream rate (default: 20)

Running the Ground Control Station

To run the GCS (Ground Control Station):

1. Ensure your drone is connected via serial/USB and powered on.

2. Start the backend server by running:
   python server.py

3. Open index.html in a web browser (Chrome, Firefox, or Edge recommended).

4. The dashboard will automatically connect to the WebSocket server and begin displaying telemetry data.

5. Use the control panel to send commands to the drone, monitor flight parameters, and view real-time data.

If not then you can directly open the index.html file and use it. Later on you can then plug the drone in.
Usage

1. Connect your drone to the computer via serial/USB

2. Start the backend server:
   python server.py

3. Open the dashboard:
   Open index.html in a modern web browser
   The dashboard will automatically connect to the WebSocket server

4. Monitor and control:
   View real-time telemetry data
   Use the control panel for arming, mode changes, and commands
   Monitor GPS position on the map

Configuration

Environment Variables

Variable | Default | Description
DRONE_CONNECTION | COM18 | Serial port for drone connection
DRONE_BAUD | 9600 | Serial baud rate
WS_HOST | 0.0.0.0 | WebSocket server host
WS_PORT | 8765 | WebSocket server port
STREAM_HZ | 20 | Telemetry update frequency

Supported Flight Modes

The dashboard supports standard ArduPilot flight modes including:
STABILIZE, ACRO, ALT_HOLD, AUTO, GUIDED, LOITER, RTL, CIRCLE, LAND, DRIFT, SPORT, FLIP, AUTOTUNE, POSHOLD, BRAKE, THROW, AVOID_ADSB, GUIDED_NOGPS, SMART_RTL

File Structure

server.py: Python WebSocket server and MAVLink handler
index.html: Main dashboard HTML page
app.js: Frontend JavaScript logic
style.css: Dashboard styling and animations
readme.txt: This documentation

Dependencies

Backend
pymavlink: MAVLink protocol implementation
websockets: WebSocket server library
asyncio: Asynchronous I/O

Frontend
Leaflet: Interactive maps library
Google Fonts: Share Tech Mono, Rajdhani, Orbitron

Troubleshooting

Connection Issues: Verify serial port and baud rate match your drone's configuration
No Telemetry: Check MAVLink heartbeat and ensure data streams are requested
WebSocket Errors: Ensure firewall allows WebSocket connections on configured port
GPS Not Showing: Wait for valid GPS fix (at least 3 satellites)


Contributing

Contributions are welcome! Please ensure code follows the existing style and includes appropriate documentation.


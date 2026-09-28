# DroneGuard Frontend Design Plan

## Subject Analysis
- **Product**: MAVLink telemetry dashboard for drone monitoring
- **Audience**: Drone operators, pilots, technicians
- **Primary Job**: Real-time visualization of critical flight data (attitude, GPS, IMU, RC channels, vehicle control)

## Current Design Assessment
The existing design uses a dark aerospace theme with:
- Glassmorphism panels
- Blue/cyan (#00d4ff) and green (#00ff88) accent colors
- Orbitron/Rajdhani/Share Tech Mono typography stack
- Standard grid-based panel layout
- Ambient particle background
- Circular attitude indicator

While functional, it follows common dashboard templating patterns that need revision per the frontend-design skill.

## Distinctive Design Plan

### Color Palette (Aviation-Inspired)
Inspired by vintage aircraft instrumentation and flight environments:
- `--bg-deepest: #0a0a0a` - Deep space black for infinite depth feeling
- `--bg-panel: rgba(15, 20, 35, 0.8)` - Navy glass with subtle transparency
- `--accent-primary: #ff6b35` - Vivid orange (critical flight data, like instrument needles)
- `--accent-secondary: #4cc9f0` - Bright cyan (navigation/secondary data)
- `--accent-success: #06d6a0` - Teal (positive indicators, GPS lock)
- `--accent-warning: #ff9f1c` - Amber (cautions, warnings)
- `--accent-danger: #d62828` - Red (alerts, emergencies)
- `--text-primary: #e0e0e0` - Soft white (primary text, reduced glare)
- `--text-secondary: #a0a0a0` - Medium gray (labels, secondary text)

### Typography (Technical but Distinctive)
Avoiding overused tech fonts for a more purposeful pairing:
- **Display**: `'JetBrains Mono', monospace` - Technical clarity with distinctive character
- **Body**: `'Inter', sans-serif` - Exceptional readability for interface elements
- **Utility**: `'Space Mono', monospace` - Specialized labels/data requiring monospace

### Layout Concept: Aircraft Instrument Panel
Organizing the dashboard like an actual cockpit rather than generic monitoring panels:

1. **Primary Flight Display (PFD) - Center Focus**
   - Large attitude indicator as central artificial horizon
   - Vertical flight tapes for altitude and airspeed on sides
   - Heading indicator at bottom
   - This creates the pilot's primary reference point

2. **Navigation Display - Right Side**
   - GPS/map visualization as secondary focal point
   - Waypoints, flight path, home position
   - Smaller than PFD but equally important for situational awareness

3. **Engine/RCD Telemetry - Left Side**
   - RC channel visualization as vertical bar graph
   - Battery voltage/current as tape indicators
   - Motor/ESC telemetry (if available)
   - Treated as engine monitoring in traditional aircraft

4. **Annunciator Panel - Top/Bottom**
   - Flight mode indications as prominent alert-style lights
   - Arm status as large, unmissable indicator
   - GPS fix status, satellite count
   - Warning/caution lights for system status

5. **Command Console - Persistent Access**
   - Maintained access to command center but less visually dominant

### Signature Element: Integrated Flight Tape System
The distinctive signature that embodies the brief:
- **Vertical Tape Indicators** for altitude and speed that move like traditional aircraft instruments
- **Connected to Attitude Indicator** - the tapes physically attach to the PFD creating a unified instrument
- **Dynamic Scaling** - tapes expand/contract based on current values for precision
- **Color Coding** - critical ranges highlighted with accent colors (red for limits, green for normal)
- **Tactile Feel** - subtle texture and shading to mimic real instrument bezels

This signature element is:
1. **Specific to Subject** - Directly derived from aviation instrumentation used in actual drones/aircraft
2. **Memorable** - Creates a unified focal point unlike scattered widgets
3. **Functional** - Provides immediate situational awareness through instrument-like presentation
4. **Not Templated** - Avoids standard gauges/widgets for purpose-built flight instruments

### Restraint & Discipline
- **Motion**: Limited to essential updates (tape movement, indicator changes) - no decorative animations
- **Decoration**: Zero non-functional elements - every visual element serves information display
- **Complexity**: Matches aviation precision - exact spacing, alignment, and readability-focused
- **Written Content**: Technical labels only, sentence case, active voice ("ARM"/"DISARM", not "Arm/Disarm")

## Implementation Approach
1. Start with structural HTML reorganization to match instrument panel layout
2. Implement color palette and typography system
3. Build the Primary Flight Display with attitude indicator + flight tapes
4. Create navigation and telemetry panels
5. Design annunciator/warning system
6. Refine interactions and responsiveness
7. Ensure accessibility (focus states, reduced motion support)

This plan avoids the three AI-generated design defaults:
1. ☐ Warm cream background (#F4F1EA) with terracotta accent
2. ☐ Near-black background with single bright accent (using multiple purposeful accents)
3. ☐ Broadsheet-style layout with hairline rules and dense columns (instrument panel organization)

Instead, it creates an aviation-specific visual language that serves the drone telemetry purpose while being distinctive and memorable.
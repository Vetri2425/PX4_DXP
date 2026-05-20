# T2: Trajectory & Pattern Planning from Mission Data

## What to Research

How does a marking mission (CAD file, waypoint list, shape definition) get converted into a real-time trajectory that the rover can follow? What is the pipeline from "drawing specification" to "setpoint stream"?

## Specific Questions

1. **Mission → Path: How does a shape become a drivable path?**
   - A DXF line: just two endpoints. How does that become a continuous (x,y,θ) trajectory?
   - A DXF arc/circle: defined by center + radius. How does that become waypoints or a curve?
   - A DXF polyline with arcs: how are line-arc transitions handled smoothly?
   - A sports field layout (rectangle + circles + arcs): how is it sequenced?

2. **Path → Trajectory: How does a path become a time-parameterized trajectory?**
   - What is the difference between a "path" (geometry only) and a "trajectory" (geometry + timing)?
   - How is speed profile generated? (acceleration, cruise, deceleration at corners)
   - How are transitions between segments handled? (line→arc, arc→line, arc→arc)
   - What about stop/start at segment boundaries (paint on/off)?

3. **CAD/DXF parsing:**
   - What DXF entities are relevant? (LINE, ARC, CIRCLE, LWPOLYLINE, POLYLINE, SPLINE, ELLIPSE)
   - How to extract geometry from DXF in Python? (ezdxf library?)
   - How to handle coordinate systems in DXF (local vs georeferenced)?
   - What about SVG parsing? Is it simpler than DXF?

4. **Arc discretization:**
   - Should arcs be converted to dense waypoints, or kept as arc parameters?
   - What resolution? (angular step, chord error, distance step)
   - For our ±2cm accuracy, how many points per meter of arc?

5. **Sequencing and optimization:**
   - How to order segments for minimum total distance / time?
   - How to handle "pen up" (spray off) transitions between disconnected segments?
   - Should the rover drive backward between segments or always turn around?

6. **Spray control integration:**
   - How to embed "spray on/off" into the trajectory?
   - Lead-in/lead-out distances for spray (valve actuation delay)
   - Speed changes at spray start/end points?

## Our Context

- Current approach: laptop generates dense waypoint files (Karney geodesic method), QGC `.waypoints` format
- This works but is inflexible — can't change mission on the fly
- Want: Jetson-side trajectory generation from a compact mission description
- Speed target: 0.3-0.4 m/s, accuracy target: ±2cm
- OFFBOARD mode: Jetson streams setpoints at 50Hz
- The rover is differential drive — minimum turning radius ~0.235m

## Deliverable

1. Pipeline diagram: Mission file → Path → Trajectory → Setpoints
2. Recommended approach for our rover (simple first, extensible later)
3. Python library recommendations (ezdxf, shapely, etc.)
4. Example: how a DXF line+arc becomes a 50Hz setpoint stream
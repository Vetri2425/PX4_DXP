# T1: Mission & File Formats for Commercial Marking Rovers

## What to Research

How do commercial road marking and sports field marking rovers receive their drawing instructions? What file formats, mission formats, and data pipelines are used in the industry?

## Specific Questions

1. **What file formats do commercial marking rovers accept?**
   - DXF (AutoCAD), DWG, SVG, GeoJSON, KML, custom binary?
   - Which is most common in the road marking industry?
   - Which is most common in sports field marking (soccer, tennis, athletics tracks)?

2. **How do existing products handle mission definition?**
   - TinyMobileRobots (TMR) / LineX: what format do they use?
   - Ditch Witch: how is striping data loaded?
   - Graco LineLazer with GPS: mission format?
   - Any other commercial marking robots (athletic fields, parking lots)?

3. **What data does a marking mission contain?**
   - Just waypoints (lat/lon sequences)?
   - Full geometry (lines, arcs, circles, bezier curves)?
   - Sprayer on/off triggers along the path?
   - Speed changes at specific points?
   - Multiple paint colors / line widths?

4. **How is georeferencing handled?**
   - Are shapes defined in local coordinates and then placed on a geolocation?
   - Or is everything in WGS84 from the start?
   - How do sports field layouts work (standard dimensions + anchor point)?

5. **What is the minimum viable mission format for our rover?**
   - What's the simplest format that supports lines + arcs + on/off triggers?
   - Can we start with a simple format and extend later?

## Our Context

- Our rover draws lines on ground using spray paint
- Need to support: straight lines, arcs, circles, squares, custom shapes
- Input could come from: CAD file, manual waypoint list, or QGC mission file
- Current waypoint format: QGC `.waypoints` (WGS84 lat/lon, NAV_WAYPOINT commands)
- Jetson runs ROS2 — mission data should be parseable in Python

## Deliverable

List of:
1. File formats used by commercial competitors (with examples if possible)
2. Recommended format(s) for our rover (short-term and long-term)
3. Minimum viable mission schema (what fields, what types)
4. Example mission file in the recommended format (straight line + arc + circle)
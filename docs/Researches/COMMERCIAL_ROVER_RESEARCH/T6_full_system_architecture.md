# T6: Full System Architecture — End-to-End Drawing Pipeline

## What to Research

What is the complete end-to-end pipeline from "user draws a shape in CAD" to "rover paints it on the ground"? How do all the components fit together? What runs where?

## Specific Questions

1. **Complete data flow:**
   - User creates drawing in CAD → exports file → uploads to rover → rover paints
   - What are ALL the intermediate steps and data transformations?
   - What format at each stage? (CAD → mission → path → trajectory → setpoints → motor commands)
   - Where does each transformation happen? (laptop, Jetson, PX4)

2. **What runs on laptop vs Jetson vs PX4?**
   - Laptop: CAD editing, mission generation, QGC monitoring
   - Jetson: trajectory generation, path following controller, spray control, logging
   - PX4: EKF2 fusion, motor mixing, safety/failsafe
   - Is this the right split? Should any of these move?

3. **What is the minimum viable pipeline?**
   - Phase 2 MVP: what is the simplest end-to-end that draws a line and an arc?
   - Can we start with: laptop generates waypoints → Jetson follows with PP → PX4 drives motors?
   - What can be deferred? (CAD parsing, automatic sequencing, MPC)

4. **How does spray control integrate?**
   - GPIO pin on Jetson? Or PWM channel on CubeOrangePlus?
   - How to sync spray on/off with position along trajectory?
   - Lead-in distance: spray starts N cm before the paint point (valve delay)
   - Lead-out distance: spray stops N cm after the end point (residual paint)
   - How to handle spray during turns? (paint off during spot-turns between segments)

5. **What is the feedback loop?**
   - Controller outputs setpoint → PX4 drives motors → rover moves → GNSS measures position → EKF2 fuses → publishes pose → controller reads pose → computes next setpoint
   - What is the total loop time?
   - Is 50Hz fast enough? What if we need 100Hz?

6. **Error recovery and safety:**
   - What happens if RTK fix is lost during marking?
   - What happens if a wheel slips?
   - What happens if NTRIP server goes down mid-mission?
   - Should the rover stop, pause, or continue with degraded accuracy?
   - How to resume a mission after an interruption?

7. **Commercial product comparison:**
   - TinyMobileRobots LineX: what is their full pipeline?
   - What sensors, what controller, what mission format?
   - How do they achieve ±5mm claimed accuracy?
   - What can we learn from their architecture?

8. **Scalability:**
   - Sports field: 1-2 hours of continuous marking
   - Road marking: full day operation
   - Battery management, paint management
   - Mission segmentation (break large jobs into manageable segments)

## Our Context

- **Current working components:**
  - MAVROS2 bridge: working
  - OFFBOARD mode: position + velocity setpoints working
  - NTRIP RTK: node healthy, server 502 (external issue)
  - EKF2: running on PX4, outputs position + heading
  - Spray system: NOT YET BUILT (GPIO relay on Jetson planned)

- **Current known gaps:**
  - P3 (reverse) not working in OFFBOARD
  - P4 (heading hold) not validated
  - No path following controller
  - No trajectory generator
  - No CAD/mission parser
  - No spray control
  - No mission management (start/pause/resume/stop)

## Deliverable

1. End-to-end data flow diagram (CAD → paint on ground)
2. Component placement diagram (laptop / Jetson / PX4 — what runs where)
3. Minimum viable pipeline definition (simplest working end-to-end)
4. Full pipeline definition (commercial-grade)
5. Gap analysis: what exists vs what's needed
6. Phase plan: MVP → commercial-grade (ordered by priority)
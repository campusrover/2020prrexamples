# Design notes

## Desired behavior

### constants

- WALLDIST = 0.25
- WALLSIDE = pi/2
- DANGDIST = 0.2

### Behaviors

- Approach: Drive towards the nearest obstacle until within 0.2 Meters
- Parallel: Turn until obstacle is at left of the robot
- Follow: Follow obstacle maintaining 0.2 meter distance until obstacle in other direction

### States

- INIT: Execute Approach while FREE -> ALIGN
- ALIGN: Rotate until PARALLEL -> FOLLOW
- FOLLOW: D


### Conditions

- FREE: No obstacle closer than WALLDIST
- PARALLEL: Nearest obstacle is at WALLDIST with bearing WALLSIDE
- TOOCLOSE: Nearest obstacle is at DANGDIST


## driver.py

- subscribes to topic /scan/filtered which gives:
  - nearest distance and bearing
  - Fwd, Left, Right, Rear distance to obstacles

- man

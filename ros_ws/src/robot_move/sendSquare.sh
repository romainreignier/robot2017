#!/bin/bash
rosservice call /follow_path "path:
  header:
    seq: 0
    stamp:
      secs: 0
      nsecs: 0
    frame_id: 'odom'
  poses:
  - x: 0.0
    y: 0.0
    theta: 0.0
  - x: 1.0
    y: 0.0
    theta: 0.0
  - x: 1.0
    y: 0.0
    theta: 1.5707963267948966
  - x: 1.0
    y: 1.0
    theta: 1.5707963267948966
  - x: 1.0
    y: 1.0
    theta: 3.141592653589793
  - x: 0.0
    y: 1.0
    theta: 3.141592653589793
  - x: 0.0
    y: 1.0
    theta: 4.71238898038469"

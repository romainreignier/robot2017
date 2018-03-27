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
  - x: 0.25
    y: 0.0
    theta: 0.0
  - x: 0.5
    y: 0.0
    theta: 0.0
  - x: 0.75
    y: 0.0
    theta: 0.0
  - x: 1.0
    y: 0.0
    theta: 0.0
  - x: 1.25
    y: 0.0
    theta: 0.0
  - x: 1.5
    y: 0.0
    theta: 0.0
  - x: 1.75
    y: 0.0
    theta: 0.0
  - x: 2.0
    y: 0
    theta: 0
  - x: 3.0
    y: 0
    theta: 0
  - x: 3.0
    y: -1.0
    theta: -1.57"

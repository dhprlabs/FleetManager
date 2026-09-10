#!/bin/bash

echo "Setting initial poses..."

# Robot 1
ros2 topic pub --once /robot1/initialpose \
geometry_msgs/msg/PoseWithCovarianceStamped \
"{
  header: {
    frame_id: map
  },
  pose: {
    pose: {
      position: {
        x: 0.0,
        y: 0.0,
        z: 0.0
      },
      orientation: {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0
      }
    },
    covariance: [
      0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0685
    ]
  }
}"

sleep 1

# Robot 2
ros2 topic pub --once /robot2/initialpose \
geometry_msgs/msg/PoseWithCovarianceStamped \
"{
  header: {
    frame_id: map
  },
  pose: {
    pose: {
      position: {
        x: 2.0,
        y: 0.0,
        z: 0.0
      },
      orientation: {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0
      }
    },
    covariance: [
      0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0685
    ]
  }
}"

sleep 1

# Robot 3
ros2 topic pub --once /robot3/initialpose \
geometry_msgs/msg/PoseWithCovarianceStamped \
"{
  header: {
    frame_id: map
  },
  pose: {
    pose: {
      position: {
        x: -2.0,
        y: 0.0,
        z: 0.0
      },
      orientation: {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0
      }
    },
    covariance: [
      0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0685
    ]
  }
}"

echo "Initial poses sent."

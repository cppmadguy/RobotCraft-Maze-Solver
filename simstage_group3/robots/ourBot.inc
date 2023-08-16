define laser ranger(
  sensor(
    range [ 0.0  5.6 ]
    fov 360
    samples 360
  )

  color "blue"
  size [ 0.03 0.03 0.03 ]

  block( points 4
     point[0] [0 0]
     point[1] [0 1]
     point[2] [1 1]
     point[3] [1 0]
     z [0 1]
  )
)

define ourBot position(

  odom_error [0.03 0.03 0.00 0.05]
  localization "odom"
  size [ 0.25 0.25 0.5 ]
  gui_nose 1
  drive "diff"
  laser(pose [ 0.125 0 -0.25 0 ])

  color "red"
  block(
     points 4
     point[0] [1 1]
     point[1] [1 -1]
     point[2] [-1 -1]
     point[3] [-1 1]
     z [0 1]
  )

  block(
    points 4
    point[0] [0.85 -1]
    point[1] [0.85 -1.3]
    point[2] [-0.85 -1.3]
    point[3] [-0.85 -1]
    z [0 0.8]
  )

  block(
    points 4
    point[0] [0.85 1]
    point[1] [0.85 1.3]
    point[2] [-0.85 1.3]
    point[3] [-0.85 1]
    z [0 0.8]
  )

  block(
    points 4
    point[0] [-1 0.1]
    point[1] [-1.1 0.1]
    point[2] [-1.1 -0.1]
    point[3] [-1 -0.1]
    z [0.5 1]
  )

  block(
    points 4
    point[0] [-1.1 0.1]
    point[1] [-1.2 0.1]
    point[2] [-1.2 -0.1]
    point[3] [-1.1 -0.1]
    z [1 2]
  )
)
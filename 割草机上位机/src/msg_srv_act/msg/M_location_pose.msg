#pose is [x, y, yaw, pitch, v]
float64[5] location_pose
#location state    0:rtk imu odo fusion positioning   1:imu odo fusion positioning
uint8 location_state
# covariance is 5*5  [[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0]]
float64[25] covariance_pose

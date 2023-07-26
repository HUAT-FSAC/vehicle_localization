# gps_imu_fusion

## depenencies

- common_msgs (HUAT_ASENSING.h)

## Todo

- update cmakelist file: add exec

- 2 exec files, using launch file to initiate

## ESKF Requirements:

- acc(m/s^2): x,y,z (including gravity)
- gps: lat(deg), lon(deg), alt(m), vN,vE,vD(m/s) (North, East, Down)
- gyro(deg/s): x,y,z
- gps_time?
  the timestamp that its data updates
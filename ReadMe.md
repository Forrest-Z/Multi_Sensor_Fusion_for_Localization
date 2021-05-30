# 多传感器融合定位

融合IMU、VO、Lidar scan matching的定位系统

# bug

1. localize:imu的坐标系需要转换到成odom下
   
   阅读了定位部分的源码，发现可以在imu的回调函数中把imu转到雷达坐标系,或者模仿loam里面的实现，把激光雷达转到imu中估计，再转回自己坐标系
2. seg:由于相机rgb模块不是正中间，因此不能按照xy四分区方法，后面好要根据tf来确定分区方法
3. localize: hdl里程计查询部分有bug


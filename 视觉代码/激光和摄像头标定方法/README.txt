设置六个点 如图1.jpg
1.激光雷达获取角点信息，距离和角度（距离单位mm，角度单位°）
2.
--roslaunch usb_cam usb_cam_test.luanch 打开摄像头节点
--python save_picture.py 保存照片
--python mouse_point.py  获取摄像头坐标点
3.python calibration.py  标定获取单应矩阵
4.
--roslaunch usb_cam usb_cam-test.launch打开摄像头
--roslaunch ydlidar_ros_driver X2.launch打开激光雷达
--python laseroncamera.py 检测融合是否正常
--rviz观看话题

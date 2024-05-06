shell文件用于启动多个ROS节点:

<--启动方法为./xxx.sh-->

1.mapping.sh 键盘控制建图
2.exploring.sh 探索式建图
3.calibration.sh 相机标定
4.navigation.sh 导航
5.navi_face.sh 导航+人脸检测

Kazam录屏转码：
ffmpeg -y -i Kazam_screencast_00001.mp4  -c:v libx264 -c:a aac -strict experimental -tune fastdecode -pix_fmt yuv420p -b:a 192k -ar 48000 2.mp4 


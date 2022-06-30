# Air-Hockey-Robot
使用OpenCV3 + Arduino mega2560，可与人对战的桌上冰球机器人

展示视频：https://www.bilibili.com/video/BV1ax411d7me?share_source=copy_web

-------------------------------------------------------------------------------------------------------------------------

参考项目：https://github.com/JJulio/AHRobot.git

原项目是国外一个大神做的，相关教程也很详细，链接里文档、视频、源码都有，他的球桌甚至都是自己做的，我也试着用三合板做了个球桌，做的有些大了，而且xy平台用的是H-bot结构，这种结构xy轴是联动的，x轴越长，在快速运动时会产生一个使轴旋转的力，导致高速运动时会卡住。

<div align=left><img width=600 src="image\三合板球桌1.jpg">

接下来干脆直接tb买个球桌，长这个样子，81*42cm，比较小，接下来就是将所有机械部分组装上去。

<div align=left><img width=400 src="image\新球桌.jpg">

一些连接件也要自己设计了，用SolidWorks2016设计出以下三维图，黑色的为3D打印件。xy轴是分开控制的，x轴单独由一个电机控制，y轴左右两边各一个电机，这两个电机运动完全同步。

<div align=left><img width=800 src="image\三维图.jpg">

<div align=left><img width=800 src="image\整体图片.jpg">

<div align=left><img width=800 src="image\摄像头截图.jpg">

下位机控制采用的arduino mega2560+ramps1.4扩展板，和diy 3d打印机常用的主控一样。

下位机Arduino代码使用的VS2013编译，Arduino官方IDE实在是太难用。Visual Studio安装一个Arduino插件后也可以编译上传Arduino代码。下位机功能类似于GRBL固件，控制击球器点对点移动。与GRBL不同的是，使用GRBL固件时，向下位机发送A点坐标，电机向A点运动，运动期间再发送B点坐标，电机要到达A点之后才会向B点运动。但冰球机器人运动轨迹是上位机来预测的，终点在不断变化，如果还按照GRBL固件来的话会走很多多余路线，也就不能及时抵挡冰球。因此冰球机器人是接收到新的坐标B后便放弃A坐标，立刻向B坐标运动。

上位机图像处理的策略是：通过HSV颜色阈值在画面中寻找冰球和机器人击球器坐标，通过帧差判断出冰球的运动速度和方向，当冰球朝着机器人运动时根据冰球的轨迹预测撞击点坐标（其中包括有反弹和无反弹的情况），将预测的终点坐标发送给下位机，下位机运动到终点视具体情况进行防御或主动进攻。

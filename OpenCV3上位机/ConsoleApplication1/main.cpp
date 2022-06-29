#include "iostream"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp" 
#include "opencv2/core/core.hpp"

#include "serialport.h"

using namespace cv;
using namespace std;

#define RESOLUTION_X 640 //设定摄像头参数，单位为像素
#define RESOLUTION_Y 480 //设定摄像头参数，单位为像素
#define FPS 60 //摄像头帧率

#define CONSTRAIN(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt))) //限制参数范围
#define PUCK_LEFT_CONFINE 51 //冰球在摄像头视野中最小x坐标（单位：像素）
#define PUCK_RIGHT_CONFINE 423 //冰球在摄像头视野中最小y坐标（单位：像素）
#define DEFENSE_POSITION 20 //防御线y坐标（单位：像素）
#define ATTACK_POSITION 120 //进攻线y坐标（单位：像素）

//图像变量
Mat frame, imgHSV;
Mat imgThresh, imgThresh2;

//保存串口向下位机发送的指令
unsigned char coordResult[20] = { "\0" };

//寻找冰球和机器人变量
int RobotCoordX = 0;//机器人实时x坐标
int RobotCoordY = 0;//机器人实时y坐标
int puckCoordX = 0;//冰球实时x坐标
int puckCoordY = 0;//冰球实时y坐标
int puckOldCoordX = 0;//冰球上次x坐标
int puckOldCoordY = 0;//冰球上次x坐标
int puckSpeedX = 0;//冰球在x轴上的实时速度
int puckSpeedY = 0;//冰球在y轴上的实时速度
int puckOldSpeedX = 0;//冰球在x轴上的上次速度
int puckOldSpeedY = 0;//冰球在y轴上的上次速度
int puckSpeedXAverage = 0;//冰球在x轴平均速度
int puckSpeedYAverage = 0;//冰球在y轴平均速度
int predict_x = 0;//预测冰球进入到机器人防御线时的x坐标
int predict_y = 0;//预测冰球进入到机器人防御线时的y坐标
int predict_x_old = 0;//保存上次预测的冰球到达机器人防御线时的x坐标
int predict_x_attack = 0;//预测冰球运动到机器人进攻线时的x坐标
int predict_status = 0;// -1：噪声 0：冰球移动缓慢或者正向人移动 1：冰球直线运动到目标 2：冰球与侧边发生了一次碰撞

//读取配置文件变量
int camera_cap;
int serial_port;
int minH,maxH,minS,maxS,minV,maxV;
int RminH,RmaxH,RminS,RmaxS,RminV,RmaxV;
bool log_output;
bool video_output;

//主动进攻模式变量
long attack_time = 0;
int attack_pos_x = 0;
int attack_pos_y =0;
int attack_predict_x = 0;
int attack_predict_y = 0;
int attack_status = 0;

//关于时间的预测
int predict_time = 0;//预测冰球到达机器人防御线的时间，单位ms
int predict_time_attack = 0;//预测冰球到达机器人进攻线的时间，单位ms

//与计算帧率有关变量
double firstTime = 0;//记录首次时间
double realTime = 0;//记录实时时间
double realTime_old = 0;//记录上次实时时间
int run_counter = 0;//记录程序运行的次数

//显示FPS
double fps;
char str[10];  // 用于存放帧率的字符串
string fpsString("FPS:");

//用于计算机器人策略
unsigned char robot_status = 0;//保存机器人应执行的策略

//感兴趣区域
Rect Hockey(40, 20, 600, 440);//寻找冰球感兴趣区域
//Rect Robot(0, 20, 60, 440);//寻找机器人感兴趣区域
Rect Robot(0, 20, 160, 440);//寻找机器人感兴趣区域

//声明串口对象
CSerialPort mySerialPort;

void serialPortInit(UINT com)
{
	if (!mySerialPort.InitPort(com))//打开串口
	{
		std::cout << "initPort fail !" << std::endl;
	}
	else
	{
		std::cout << "initPort success !" << std::endl;
	}

	if (!mySerialPort.OpenListenThread())//打开监听线程
	{
		std::cout << "OpenListenThread fail !" << std::endl;
	}
	else
	{
		std::cout << "OpenListenThread success !" << std::endl;
	}
}

void cameraProcess(float time)
{
	int vectorX;//冰球在两帧图片上x方向位移差
	int vectorY;//冰球在两帧图片上y方向位移差
	double slope;//轨迹斜率

	int bounce_x;//冰球与边界撞击点x坐标
	int bounce_y;//冰球与边界撞击点y坐标

	vectorX = puckCoordX - puckOldCoordX;//位移差，交换xy坐标后
	vectorY = puckCoordY - puckOldCoordY;//位移差，交换xy坐标后

	puckOldSpeedX = puckSpeedX;//将实时速度赋给上次速度
	puckOldSpeedY = puckSpeedY;//将实时速度赋给上次速度

	//更新速度值
	puckSpeedX = vectorX * 100L / time;//每秒30帧
	puckSpeedY = vectorY * 100L / time;//每秒30帧

	//if ((puckSpeedX < -500) || (puckSpeedX > 500) || (puckSpeedY < -500) || (puckSpeedY > 500))//***********************************************************
	//{
	//	//cout << "********noise********" << endl;
	//	predict_status = -1;
	//	predict_x_old = -1;
	//	return;
	//}

	//if (predict_status == -1)// Noise on last reading?  上次阅读的噪音？
	//{
	//	puckSpeedXAverage = puckSpeedX;
	//	puckSpeedYAverage = puckSpeedY;
	//}
	//else
	//{
	//	// if there are low accelerations (similar speeds on readings) we apply an average filtering with the previous value...
	//	//如果有较低的加速度（类似的读数），我们应用一个与先前值的平均过滤。
	//	if (abs(puckSpeedX - puckOldSpeedX) < 50)//***************************************************************************************************
	//		puckSpeedXAverage = (puckSpeedX + puckOldSpeedX) >> 1;
	//	else
	//		puckSpeedXAverage = puckSpeedX;

	//	if (abs(puckSpeedY - puckOldSpeedY) < 50)//***************************************************************************************************
	//		puckSpeedYAverage = (puckSpeedY + puckOldSpeedY) >> 1;
	//	else
	//		puckSpeedYAverage = puckSpeedY;
	//}

		puckSpeedXAverage = puckSpeedX;
		puckSpeedYAverage = puckSpeedY;

	predict_x_attack = -1;//

	//冰球来了
	if (puckSpeedYAverage < -20)//********************************************************************************************************************
	{
		predict_status = 1;

		if (vectorX == 0) slope = 999999;
		else slope = (float)vectorY / (float)vectorX;

		predict_y = DEFENSE_POSITION;//冰球最终目标点的y坐标
		predict_x = (predict_y - puckCoordY) / slope + puckCoordX;//冰球最终目标点的x坐标

		predict_x_attack = (ATTACK_POSITION - puckCoordY) / slope + puckCoordX;//冰球运动到进攻线时的x坐标

		if ((predict_x < PUCK_LEFT_CONFINE) || (predict_x > PUCK_RIGHT_CONFINE))//冰球与侧边发生碰撞
		{
			predict_status = 2;
			//predict_bounce = 1;
			//predict_bounce_status = 1;

			if (predict_x < PUCK_LEFT_CONFINE)//判断与那个侧边发生碰撞
				bounce_x = PUCK_LEFT_CONFINE;
			else
				bounce_x = PUCK_RIGHT_CONFINE;

			bounce_y = (bounce_x - puckCoordX)*slope + puckCoordY;//计算撞击点y坐标
			predict_time = (bounce_y - puckCoordY) * 100L / puckSpeedY;  // time until bouce

			slope = -slope;
			predict_y = DEFENSE_POSITION;
			predict_x = (predict_y - bounce_y) / slope + bounce_x;

			if ((predict_x < PUCK_LEFT_CONFINE) || (predict_x > PUCK_RIGHT_CONFINE)) // New bounce with side wall?
			{
				//我们什么也不做。有了两次反弹，目标的风险很小。
				predict_x_old = -1;
				predict_status = 0;
			}
			else
			{
				////只有一次反弹……
				////如果球速改变了很多这就意味着冰球已经触到了一边
				//if (abs(puckSpeedY - puckOldSpeedY) > 40)//**************************************************************************************************
				//{
				//	// We dont make a new prediction...  不做一个新的预测。
				//	//predict_x_old = -1;
				//}
				//else
				//{
					 //average of the results (some noise filtering)  平均结果（一些噪声滤波）
					if (predict_x_old != -1)
						predict_x = (predict_x_old + predict_x) >> 1;
					predict_x_old = predict_x;

					//我们引入一个因子（120而不是100）来模拟反弹（速度损失20%）
					predict_time = predict_time + (predict_y - puckCoordY) * 130L / puckSpeedY;  // in ms
				//}
			}
			cv::line(frame, Point(puckCoordY, puckCoordX), Point(bounce_y, bounce_x), Scalar(255, 255, 0), 2);
			cv::line(frame, Point(bounce_y, bounce_x), Point(predict_y, predict_x), Scalar(255, 0, 0), 2);
		}
		else  //冰球没有撞到侧边，直接过来
		{
			//if (predict_bounce_status == 1)
			//{
			//	// We dont predict nothing new...
			//	predict_bounce_status = 0;
			//}
			//else
			//{
				// average of the results (some noise filtering)  结果的平均值（一些噪声滤波）
				if (predict_x_old != -1)
					predict_x = (predict_x_old + predict_x) >> 1;
				predict_x_old = predict_x;

				predict_time = (predict_y - puckCoordY) * 100L / puckSpeedY;  // in ms
				predict_time_attack = (ATTACK_POSITION - puckCoordY) * 100L / puckSpeedY; // in ms

				cv::line(frame, Point(puckCoordY, puckCoordX), Point(predict_y, predict_x), Scalar(255, 0, 0), 2);//x轴和y轴对调
			//}
		}
	}//冰球来了
	else //冰球运动缓慢或向另一边移动
	{
		predict_x_old = -1;
		predict_status = 0;
		//predict_bounce = 0;
		//predict_bounce_status = 0;
	}
}

// Return the predicted position of the puck in predict_time miliseconds
int predictPuckXPosition(int predict_time)
{
	return (puckCoordX + (long)puckSpeedXAverage*predict_time / 100L);
}

// Return the predicted position of the puck in predict_time miliseconds
int predictPuckYPosition(int predict_time)
{
	return (puckCoordY + (long)puckSpeedYAverage*predict_time / 100L);
}

void sendMessage()
{
	coordResult[0] = 'm'; coordResult[1] = 'm';//起始位
	coordResult[2] = '\0'; coordResult[3] = '\0';//时间戳

	coordResult[4] = (predict_x >> 8) & 0xFF; coordResult[5] = predict_x & 0xFF;
	coordResult[6] = (predict_y >> 8) & 0xFF; coordResult[7] = predict_y & 0xFF;

	coordResult[8] = 0x00; coordResult[9] = robot_status;

	//coordResult[10] = 0x00; coordResult[11] = 0x00;//机器人x坐标
	//coordResult[12] = 0x00; coordResult[13] = 0x00;//机器人y坐标
	coordResult[10] = (RobotCoordX >> 8) & 0xFF; coordResult[11] = RobotCoordX & 0xFF;//机器人x坐标
	coordResult[12] = (RobotCoordY >> 8) & 0xFF; coordResult[13] = RobotCoordY & 0xFF;//机器人y坐标

	mySerialPort.WriteData(coordResult, 14);//串口发送坐标
}

void trackObjectPuck()//找冰球
{
	vector<vector<Point>> contours;
	CvPoint puckCenter = (0, 0);//计算冰球中心坐标

	imgThresh(Hockey).copyTo(imgThresh);//划定寻找冰球的感兴趣区域
	rectangle(frame, Hockey, Scalar(255, 0, 255), 1); //将感兴趣区域框出来

	findContours(imgThresh, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);//找到符合hsv阈值的区域

	if (contours.empty())//如果没有符合hsv阈值的区域，退出
		return;

	for (int index = 0; index<contours.size(); ++index)
	{
		float area = fabs(contourArea(contours[index]));
		if (area>1000 && area < 1600)//冰球面积为1400-1410像素
		{
			//double perimeter = arcLength(contours[index],true);
			//double roundness = (perimeter*perimeter) / (3.14*4*area);
			//if (roundness < 1.8)//近似于圆形（加上此函数冰球一旦运动起来处理速度不够）
			//{
				int num = contours.at(index).size();
				for (int i = 0; i < num; i++)
				{
					puckCenter.x += contours.at(index).at(i).x;
					puckCenter.y += contours.at(index).at(i).y;
				}
				puckCenter.x /= num;
				puckCenter.y /= num;

				puckOldCoordX = puckCoordX;
				puckOldCoordY = puckCoordY;

				//此处将xy坐标交换，将摄像头坐标系转换为机器人坐标系，此函数以后都使用机器人坐标系，所以有关绘图函数（circle(),line()）都是将xy坐标对调过的。
				puckCoordX = puckCenter.y + Hockey.y;
				puckCoordY = puckCenter.x + Hockey.x;

				cv::circle(frame, Point(puckCoordY, puckCoordX), 22, Scalar(255, 0, 0), 2);//用蓝色的圆圈出冰球
				cv::line(frame, Point(puckCoordY, puckCoordX), Point(puckOldCoordY, puckOldCoordX), Scalar(255, 255, 0), 2);//画出冰球已经运动了的轨迹

				//cout << "puck in pixel: " << puckCoordX << ',' << puckCoordY << endl;

				break;
			//}
		}
	}
}

void trackObjectRobot()//找机器人
{
	vector<vector<Point>> contours_R;
	CvPoint robotCenter = (0, 0);//计算机器人中心坐标

	imgThresh2(Robot).copyTo(imgThresh2);//划定寻找机器人的感兴趣区域
	rectangle(frame, Robot, Scalar(0, 255, 255), 1); //将感兴趣区域框出来

	findContours(imgThresh2, contours_R, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);//找出符合hsv阈值的区域

	if (contours_R.empty())//如果没有符合hsv阈值的区域，退出
		return;

	//找到面积最大的区域
	float maxArea = 0;
	int maxAreaIdx = 0;
	for (int index = 0; index < contours_R.size(); ++index)
	{
		float area = fabs(contourArea(contours_R[index]));
		if (area > maxArea)
		{
			maxArea = area;
			maxAreaIdx = index;
		}
	}

	if (maxArea<350 || maxArea>700)//限制面积，防止噪声和冰球影响
		return;

	//cout << "maxArea: " << maxArea << endl;

	int num = contours_R.at(maxAreaIdx).size();
	for (int i = 0; i < num; i++)
	{
		robotCenter.x += contours_R.at(maxAreaIdx).at(i).x;
		robotCenter.y += contours_R.at(maxAreaIdx).at(i).y;
	}

	robotCenter.x /= num;
	robotCenter.y /= num;

	RobotCoordY = robotCenter.x + Robot.x;
	RobotCoordX = robotCenter.y + Robot.y;
	cv::circle(frame, Point(RobotCoordY, RobotCoordX), 22, Scalar(0, 0, 100), 2);//用暗红色的圆圈出机器人

	//int robotCoordX = (RobotCoordX - 53) / 1.1242;
	//int robotCoordY = (RobotCoordY - 13) / 1.15385 + 20;

	//cout << "robot in mm: " << robotCoordX << "," << robotCoordY << endl;
	//cout << "robot in pixel: " << RobotCoordX << "," << RobotCoordY << endl;
}

/*
根据冰球具体情况给机器人传达相应的策略
	
*/
void newDataStrategy()
{
	robot_status = 0;//将机器人状态设为回到初始位置

	if (predict_status == 1)//冰球直冲过来，
	{
		if (predict_x > 151 && predict_x < 323 && predict_time_attack < 300)//predict_x单位为像素，范围51-423
		{
			robot_status = 2;//被动进攻模式
		}
		else if (predict_x > 151 && predict_x < 323)
		{
			robot_status = 1;//防御模式
		}
		else
		{
			CONSTRAIN(predict_x, 111, 363);//限制机器人移动范围
			robot_status = 1;//防御模式
		}
		attack_time = 0;
	}

	if (predict_status == 2)//冰球碰撞到了墙壁
	{
		CONSTRAIN(predict_x, 101, 373);//限制机器人移动范围
		robot_status = 1;  //防御模式
		attack_time = 0;
	}

	if (predict_status == 0 && puckCoordY < 180/* && abs(puckSpeedYAverage)<10*/)//冰球在机器人区域缓慢移动，主动进攻
	{
		if (attack_time == 0)
		{
			attack_predict_x = predictPuckXPosition(400);//预测400ms后冰球的x坐标，单位：像素
			attack_predict_y = predictPuckYPosition(400);//预测400ms后冰球的y坐标，单位：像素
			if ((attack_predict_x > 101) && (attack_predict_x < 373) && (attack_predict_y > 20) && (attack_predict_y < 180))//在一定区域内
			{
				attack_time = realTime * 1000 + 400;  // Prepare an attack in 500ms
				attack_pos_x = attack_predict_x;  // predict_x
				attack_pos_y = attack_predict_y;  // predict_y

				predict_x = attack_pos_x;//将机器人位置设置到冰球后方
				predict_y = attack_pos_y - 40;
				CONSTRAIN(predict_y, 20, 165);

				attack_status = 1;//已经在后方，准备好进攻了
			}
			else//冰球位置不合适
			{
				attack_time = 0;// Continue...
				attack_status = 0;
				// And go to defense position
				predict_x = 161;
				predict_y = DEFENSE_POSITION;
			}
		}
		else
		{
			if (attack_status == 1)//已经在后方，准备好进攻了
			{
				if ((attack_time - realTime * 1000) < 200)//准备了200ms，剩余200ms进攻
				{
					// Attack movement
					predict_x = predictPuckXPosition(200);
					predict_y = predictPuckYPosition(200) + 80;
					CONSTRAIN(predict_y, 20, 165);

					//Serial.print("ATTACK:");
					//Serial.print(com_pos_x);
					//Serial.print(",");
					//Serial.println(com_pos_y-80);

					attack_status = 2; //进攻完成      
				}
				else//可以进攻了，但是不是现在
				{
					predict_x = attack_pos_x;
					predict_y = attack_pos_y - 40;//到准备进攻区域
					CONSTRAIN(predict_y, 20, 165);
				}
			}
			if (attack_status == 2)//进攻完成了
			{
				if (realTime > (attack_time + 150))  // Attack move is done? => Reset to defense position
				{
					attack_time = 0;
					robot_status = 0;
					attack_status = 0;
				}
			}
		}
		robot_status = 3;//主动进攻模式
	}

}
//void newDataStrategy()
//{
//	// predict_status == 1 => 冰球直接向机器人移动
//	// predict_status == 2 => 冰球向机器人移动过程中发生了碰撞
//	// predict_status == 3 => 冰球在我们的场上移动缓慢，进攻？
//
//	robot_status = 0;//将机器人状态设为回到初始位置
//
//	if ((predict_status == 1) /*&& (predict_time<350)*/)//冰球直冲过来，
//	{
//		if (predict_bounce == 1)//冰球发生了碰撞
//		{
//			if (puckSpeedYAverage > -90)//冰球移动的比较慢
//				robot_status = 2;  //防御+进攻模式
//			else	//冰球移动的比较快，防御模式
//			{
//				CONSTRAIN(predict_x, 101, 373);//限制机器人移动范围，因为冰球速度太快，机器人可能赶不上
//				robot_status = 4;//防御模式（待优化）
//			}
//		}
//		else//冰球没有发生碰撞，直线抵达机器人区域
//		{
//			if (predict_y > 137 && predict_y < 337)//冰球直冲球门，防御+进攻
//				robot_status = 2; //防御+进攻模式
//			else//冰球比较偏，防御模式
//				robot_status = 1; //防御模式
//		}
//	}
//
//	if ((predict_status == 2)/* && (predict_time < 350)*/)//冰球碰撞到了墙壁
//	{
//		robot_status = 1;  //防御模式
//		CONSTRAIN(predict_x, 101, 373);//限制机器人移动范围，因为冰球速度太快，机器人可能赶不上
//	}
//
//	//if ((predict_status == 0) && (puckCoordY < 350) && (abs(puckSpeedY) < 20))//如果冰球在机器人领域缓慢移动，我们就可以开始进攻了。
//	//	robot_status = 3; //进攻模式
//
//	if (robot_status == 0)
//		cout << "回到初始位置" << endl;
//	else if (robot_status == 1)
//		cout << "防御模式" << endl;
//	else if (robot_status == 2)
//		cout << "防御+进攻模式" << endl;
//}

int configRead()
{
	FILE *f;
	char aux_str[255];
	int aux_int;
	if ((f = fopen("config.txt", "rt")) == NULL)
	{
		printf("未找到配置文件\"config.h\"\n");
		return 1;
	}
	printf("读取\"config.h\"配置文件...\n\n");
	while (!feof(f))
	{
		strcpy(aux_str, "empty");
		if (fscanf(f, "%s", aux_str))
		{
			if (strcmp(aux_str, "empty") != 0)
			{
				if (strcmp(aux_str, "#") == 0)
				{
				}
				else if (strcmp(aux_str, "CAMERA") == 0)
				{
					fscanf(f, "%d", &aux_int);
					camera_cap = aux_int;
					printf("CAMERA:%d\n", camera_cap);
				}
				else if (strcmp(aux_str, "SERIALPORT") == 0)
				{
					fscanf(f, "%d", &aux_int);
					serial_port = aux_int;
					printf("SERIALPORT:%d\n\n", serial_port);
				}
				else if (strcmp(aux_str, "PUCKMINH") == 0)
				{
					fscanf(f, "%d", &aux_int);
					minH = aux_int;
					printf("PUCKMINH:%d\n", minH);
				}
				else if (strcmp(aux_str, "PUCKMAXH") == 0)
				{
					fscanf(f, "%d", &aux_int);
					maxH = aux_int;
					printf("PUCKMAXH:%d\n", maxH);
				}
				else if (strcmp(aux_str, "PUCKMINS") == 0)
				{
					fscanf(f, "%d", &aux_int);
					minS = aux_int;
					printf("PUCKMINS:%d\n", minS);
				}
				else if (strcmp(aux_str, "PUCKMAXS") == 0)
				{
					fscanf(f, "%d", &aux_int);
					maxS = aux_int;
					printf("PUCKMAXS:%d\n", maxS);
				}
				else if (strcmp(aux_str, "PUCKMINV") == 0)
				{
					fscanf(f, "%d", &aux_int);
					minV = aux_int;
					printf("PUCKMINV:%d\n", minV);
				}
				else if (strcmp(aux_str, "PUCKMAXV") == 0)
				{
					fscanf(f, "%d", &aux_int);
					maxV = aux_int;
					printf("PUCKMAXV:%d\n\n", maxV);
				}
				else if (strcmp(aux_str, "ROBOTMINH") == 0)
				{
					fscanf(f, "%d", &aux_int);
					RminH = aux_int;
					printf("ROBOTMINH:%d\n", RminH);
				}
				else if (strcmp(aux_str, "ROBOTMAXH") == 0)
				{
					fscanf(f, "%d", &aux_int);
					RmaxH = aux_int;
					printf("ROBOTMAXH:%d\n", RmaxH);
				}
				else if (strcmp(aux_str, "ROBOTMINS") == 0)
				{
					fscanf(f, "%d", &aux_int);
					RminS = aux_int;
					printf("ROBOTMINS:%d\n", RminS);
				}
				else if (strcmp(aux_str, "ROBOTMAXS") == 0)
				{
					fscanf(f, "%d", &aux_int);
					RmaxS = aux_int;
					printf("ROBOTMAXS:%d\n", RmaxS);
				}
				else if (strcmp(aux_str, "ROBOTMINV") == 0)
				{
					fscanf(f, "%d", &aux_int);
					RminV = aux_int;
					printf("ROBOTMINV:%d\n", RminV);
				}
				else if (strcmp(aux_str, "ROBOTMAXV") == 0)
				{
					fscanf(f, "%d", &aux_int);
					RmaxV = aux_int;
					printf("ROBOTMAXV:%d\n\n", RmaxV);
				}
				else if (strcmp(aux_str, "VIDEOOUTPUT") == 0)
				{
					fscanf(f, "%s", aux_str);
					if (strcmp(aux_str, "YES") == 0)
						video_output = true;
					else
						video_output = false;
					printf("VIDEOOUTPUT:%s\n\n", aux_str);
				}
				else{}
			}
		}
	}
	return 0;
}

int main()
{
	if (configRead())//读取配置文件
		return -1;

	VideoCapture capture(camera_cap);//打开摄像头
	serialPortInit(serial_port);//打开串口

	cout << "open the camera:" << camera_cap << endl;
	cout << "open the serial port:" << serial_port << endl;

	capture.set(CV_CAP_PROP_FRAME_WIDTH, RESOLUTION_X);//设置分辨率x
	capture.set(CV_CAP_PROP_FRAME_HEIGHT, RESOLUTION_Y);//设置分辨率y
	capture.set(CV_CAP_PROP_FPS, FPS);//设定帧率，好像没什么用

	namedWindow("冰球机器人", 1);//创建大小不可改变的串口，名称为"AIR HOCKEY"

	firstTime = (double)cv::getTickCount();//记录首次时间
	//记录首次时间
	while (true)
	{
		realTime = ((double)cv::getTickCount() - firstTime) / cv::getTickFrequency();//计算开机时间
		run_counter++;//记录循环次数

		capture >> frame;//获取一帧图像
		GaussianBlur(frame, imgHSV, Size(5, 5), 0, 0);//高斯滤波
		//medianBlur(imgHSV, imgHSV, 7);//中值滤波，未加入，速度太慢
		cvtColor(imgHSV, imgHSV, CV_BGR2HSV);//RGB转HSV

		inRange(imgHSV, Scalar(minH, minS, minV), Scalar(maxH, maxS, maxV), imgThresh);//冰球HSV阈值
		inRange(imgHSV, Scalar(RminH, RminS, RminV), Scalar(RmaxH, RmaxS, RmaxV), imgThresh2);//机器人HSV阈值

		//应用过滤膨胀操作进行滤波

		trackObjectPuck();//寻找冰球
		trackObjectRobot();//寻找机器人

		cameraProcess(33.33);//预测冰球轨迹//fnffnhajjfheufjkkjjjssjhfdhujkgfeuf

		newDataStrategy();//给机器人一个策略fffefu

		//printf("%.2f: %d,%d\n", realTime, puckCoordX,puckCoordY);
		sendMessage();//串口发送数据
		//cout << coordResult << endl;//控制台输出坐标
		//COUT <<CORRNJJA
		/*******************************************计算帧率**********************************************/
		if (run_counter % 10 == 0)//十次刷新一次
		{
			fpsString = "FPS:";
			//t = ((double)cv::getTickCount() - realTime_old) / cv::getTickFrequency();//计算完成以上操作耗费的时间
			fps = 1.0 * 10 / (realTime - realTime_old);//fps

			realTime_old = realTime;

			std::sprintf(str, "%.2f", fps);      // 帧率保留两位小数
			fpsString += str;                    // 在"FPS:"后加入帧率数值字符串
		}
		cv::putText(frame, fpsString, cv::Point(5, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0));//将fps值打印在图像上
		/*************************************************************************************************/

		cv::imshow("冰球机器人", frame);//显示图像

		int c = cv::waitKey(1);
		if ((char)c == 27)//按下“ESC”键退出
			break;
	}
	return 0;
}
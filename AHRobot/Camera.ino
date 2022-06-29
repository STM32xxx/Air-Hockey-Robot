// AHR AIR HOCKEY ROBOT PROJECT

// Camera serial packet read and Puck trayectory prediction
// Robot position detection and missing steps check control

uint16_t extractParamInt(uint8_t pos){
  union{
    unsigned char Buff[2];
    uint16_t d;
  }
  u;

  u.Buff[0] = (unsigned char)SBuffer[pos];
  u.Buff[1] = (unsigned char)SBuffer[pos+1];
  return(u.d); 
}

// Read Vision System Packets over Serial Port
void packetRead()
{
  unsigned char i;
  if (Serial.available() > 0) {
    //Serial.println("P");
    // We rotate the Buffer (we could implement a ring buffer in future)
    for (i=11;i>0;i--){
      SBuffer[i] = SBuffer[i-1];
    }
    SBuffer[0] = Serial.read();
    //Serial.print(S1Buffer[0]);
    // We look for a  message start like "mm" to sync packets
    if ((SBuffer[0] == 'm')&&(SBuffer[1] == 'm'))
    {
      if (readStatus == 0)
      {
        readStatus=1;
        readCounter=12;
      }
      else
      {
        Serial.println("S ERR");
        readStatus=1;
        readCounter=12;
      }
      return;
    }
    else if (readStatus==1)
    {
      readCounter--;   // Until we complete the packet
      if (readCounter<=0)   // packet complete!!
      {
        // Extract parameters
        cam_timestamp = extractParamInt(10);
        puckPixX = extractParamInt(8);
        puckPixY = extractParamInt(6);
        puckSize = extractParamInt(4);
        robotPixX = extractParamInt(2);
        robotPixY = extractParamInt(0);  
        readStatus = 0;
        newPacket = 1;
      }
    }
  }
}

// Return the predicted position of the puck in predict_time miliseconds
int predictPuckXPosition(int predict_time)
{
  return (puckCoordX + (long)puckSpeedXAverage*predict_time/100L);
}

// Return the predicted position of the puck in predict_time miliseconds
int predictPuckYPosition(int predict_time)
{
  return (puckCoordY + (long)puckSpeedYAverage*predict_time/100L);
}

// Initialization routine
void cameraProcessInit()
{
  predict_x_old = -1;
}

//功能:检测步进器中缺失的步骤
//当机器人被停在一个已知位置（防御位置）时，我们比较了在摄像机中看到的机器人的位置。
void missingStepsDetection(int posX, int posY)
{
	int robot_position_x_mm;
	int robot_position_y_mm;

	robotCoordX = (posX - 53) / 1.1242;//将像素单位转为mm
	robotCoordY = (posY - 13) / 1.15385 + 20;//将像素单位转为mm

	// If we are stopped and we have a good robot detection (camera)
	if ((speed_x == 0)&&(speed_y == 0)&&(robotCoordX != 0))
	{
		robot_position_x_mm = position_x / X_AXIS_STEPS_PER_UNIT;//计算出机器人x坐标（mm）
		robot_position_y_mm = position_y / Y_AXIS_STEPS_PER_UNIT;//计算出机器人y坐标（mm）
		//判断是否在防御线的中心
		if ((robot_position_x_mm  > (ROBOT_CENTER_X-40))&&(robot_position_x_mm  < (ROBOT_CENTER_X+40))&&(robot_position_y_mm  >= ROBOT_MIN_Y)&&(robot_position_y_mm  < (ROBOT_DEFENSE_POSITION+30)))
		{
			robotCoordSamples++;//累加变量
			robotCoordXAverage += robotCoordX;
			robotCoordYAverage += robotCoordY;
			//当我们收集10个样本时我们会进行修正
			if (robotCoordSamples == 10)//收集了10次
			{
				#if CORRECT_MISSING_STEPS_X
					robotCoordXAverage = robotCoordXAverage / robotCoordSamples;//x坐标平均值（mm）
					robotMissingStepsErrorX = myAbs(robot_position_x_mm - robotCoordXAverage);//计算缺失的位移（mm）
					if (robotMissingStepsErrorX > MISSING_STEPS_MAX_ERROR_X)//误差大于一定程度
					{
						position_x = robotCoordXAverage * X_AXIS_STEPS_PER_UNIT;
					}
				#endif

				#if CORRECT_MISSING_STEPS_Y
					robotCoordYAverage = robotCoordYAverage / robotCoordSamples;//x坐标平均值（mm）
					//robot_position_y_mm += ROBOT_POSITION_CAMERA_CORRECTION_Y;   // correction because camera point of view and robot mark
					robotMissingStepsErrorY = myAbs(robot_position_y_mm - robotCoordYAverage);
					if (robotMissingStepsErrorY > MISSING_STEPS_MAX_ERROR_Y) 
					{
						//Serial.print("Y missing:");
						position_y = ((robotCoordYAverage - 13) / 1.15385 + 20) * Y_AXIS_STEPS_PER_UNIT;
						//Serial.print("MSY ");
						//Serial.println(robotMissingStepsErrorY); 
					}
				#endif
			}    
		}
		else
		{
			robotCoordSamples = 0;
			robotCoordXAverage = 0;
			robotCoordYAverage = 0;
			robotMissingStepsErrorX = 0;
			robotMissingStepsErrorY = 0;
		}
	}
	else
	{
		robotCoordSamples = 0;
		robotCoordXAverage = 0;
		robotCoordYAverage = 0;
		robotMissingStepsErrorX = 0;
		robotMissingStepsErrorY = 0;
	}
}



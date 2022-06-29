// AHR AIR HOCKEY ROBOT PROJECT

// Each time a new data packet from camera is reveived this function is called
void newDataStrategy()
{
	// predict_status == 1 => Puck is moving to our field directly
	// predict_status == 2 => Puck is moving to our field with a bounce
	// predict_status == 3 => Puck is in our field moving slowly, attack?
    
	// Default
	robot_status = 0;   // Going to initial position (defense)
    
	if ((predict_status==1)&&(predict_time<350))
	{
		// WE  come from a bounce?
		if (predict_bounce == 1)
		{
			if ((puckSpeedYAverage)>-150)
			// puck is moving slowly...
			robot_status = 2;  // Defense+Attack
			else
			{
				if (predict_x < 200)
				predict_x = 200;
				if (predict_x > 400)
				predict_x = 400;
				robot_status = 4;
			}
		}
		else
		{
			if ((predict_x > (ROBOT_MIN_X+35))&&(predict_x < (ROBOT_MAX_X-35))) 
			robot_status = 2; //2  //Defense+attack mode
			else
			robot_status = 1; //1
		}
	}
    
	// Prediction with side bound
	if ((predict_status==2)&&(predict_time<350))
	{
		robot_status = 1;  //1   // Defense mode
		// We limit the movement in this phase
		if (predict_x < 200)
		predict_x = 200;
		if (predict_x > 400)
		predict_x = 400;
	}
   
	// If the puck is moving slowly in the robot field we could start an attack
	if ((predict_status==0)&&(puckCoordY < ROBOT_CENTER_Y)&&(myAbs(puckSpeedY)<50))
	robot_status = 3; //3
}

/*
	机器人策略
	robot_status 0：回到初始位置
				 1：防御模式（只在x轴上移动）
				 2：进攻+防御模式
				 3：进攻模式
				 4：防御模式（待优化）
		   default：回到初始位置
*/
void robotStrategy()
{
	switch(robot_status)
	{
		case 0://回到初始位置
			com_pos_y = ROBOT_DEFENSE_POSITION;
			com_pos_x = ROBOT_CENTER_X;
			setSpeedS(MAX_SPEED_X, MAX_SPEED_Y);
			setPosition(com_pos_x,com_pos_y);
			attack_time = 0;
			break;
		case 1://防御模式，只在x轴上移动
			com_pos_y = ROBOT_DEFENSE_POSITION;
			com_pos_x = predict_x;//predict_x，预测轨迹终点的x坐标
			setSpeedS(MAX_SPEED_X, MAX_SPEED_Y);
			setPosition(com_pos_x,com_pos_y);
			attack_time = 0;
			break;
		case 2://进攻模式1
			com_pos_y = ROBOT_DEFENSE_ATTACK_POSITION + 20;
			//com_pos_x = predict_x_attack;//预测冰球到达进攻线时的x坐标
			com_pos_x = predict_x;
			setSpeedS(MAX_SPEED_X, MAX_SPEED_Y);
			setPosition(com_pos_x,com_pos_y);
			break;

		case 3:
			com_pos_y = predict_y;
			com_pos_x = predict_x;
			setSpeedS(MAX_SPEED_X, MAX_SPEED_Y);
			setPosition(com_pos_x, com_pos_y);
			break;


		//case 3://进攻模式
		//	if (attack_time == 0)
		//	{
		//		attack_predict_x = predictPuckXPosition(500);//预测500ms后冰球的x坐标
		//		attack_predict_y = predictPuckYPosition(500);//预测500ms后冰球的y坐标
		//		if ((attack_predict_x > 120)&&(attack_predict_x < 480)&&(attack_predict_y >180)&&(attack_predict_y<320))//在一定区域内
		//		{
		//			attack_time = millis() + 500;  // Prepare an attack in 500ms
		//			attack_pos_x = attack_predict_x;  // predict_x
		//			attack_pos_y = attack_predict_y;  // predict_y
		//			//Serial.print("AM:");
		//			//Serial.print(attack_pos_x);
		//			//Serial.print(",");
		//			//Serial.println(attack_pos_y);

		//			com_pos_x = attack_pos_x;//将机器人位置设置到冰球后方
		//			com_pos_y = attack_pos_y-120;
		//			setSpeedS(com_speed_x/2,com_speed_y/2);
		//			setPosition(com_pos_x,com_pos_y);
		//			attack_status = 1;//已经在后方，准备好进攻了
		//		}
		//		else//冰球位置不合适
		//		{
		//			attack_time = 0;// Continue...
		//			attack_status = 0;
		//			// And go to defense position
		//			com_pos_y = ROBOT_DEFENSE_POSITION;
		//			com_pos_x = ROBOT_CENTER_X;  //center X axis
		//			setSpeedS(com_speed_x/2,com_speed_y/2);
		//			setPosition(com_pos_x,com_pos_y);
		//		}
		//	}
		//	else
		//	{
		//		if (attack_status == 1)//已经在后方，准备好进攻了
		//		{
		//			if ((attack_time-millis())<200)//准备了300ms，剩余200ms进攻
		//			{
		//				// Attack movement
		//				com_pos_x = predictPuckXPosition(200); 
		//				com_pos_y = predictPuckYPosition(200) + 80;
		//				setSpeedS(com_speed_x,com_speed_y);
		//				setPosition(com_pos_x,com_pos_y);
  //      
		//				//Serial.print("ATTACK:");
		//				//Serial.print(com_pos_x);
		//				//Serial.print(",");
		//				//Serial.println(com_pos_y-80);
  //              
		//				attack_status = 2; //进攻完成      
		//			}
		//			else//可以进攻了，但是不是现在
		//			{
		//				com_pos_x = attack_pos_x;
		//				com_pos_y = attack_pos_y-120;//到准备进攻区域
		//				setSpeedS(com_speed_x/2,com_speed_y/2);
		//				setPosition(com_pos_x,com_pos_y);
		//			}
		//		}
		//		if (attack_status==2)//进攻完成了
		//		{
		//			if (millis()>(attack_time+80))  // Attack move is done? => Reset to defense position
		//			{
		//				//Serial.print("RESET");
		//				attack_time = 0;
		//				robot_status = 0;
		//				attack_status = 0;
		//			}
		//		}
		//	}
		//	break;


		case 4://******待优化******
			//冰球来自一个弹跳
			//现在只做防御（我们可以在将来改进它）
			//防御模式（只在防御线上移动X轴）
			com_pos_y = ROBOT_DEFENSE_POSITION;
			com_pos_x = predict_x;//predict_x，预测轨迹终点的x坐标
			setSpeedS(MAX_SPEED_X, MAX_SPEED_Y);
			setPosition(com_pos_x,com_pos_y);
			attack_time = 0;
			break;
		default://失败，回到初始位置
			com_pos_y = ROBOT_DEFENSE_POSITION;
			com_pos_x = ROBOT_CENTER_X;
			setSpeedS(MAX_SPEED_X, MAX_SPEED_Y);
			setPosition(com_pos_x,com_pos_y);
			attack_time = 0;
	}
}

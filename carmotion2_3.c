#include <time.h>
#include <stdio.h>
#include <math.h>
#include "carmotion.h"
#include "pid.h"
#include "agv_motor.h"
#include "agv_peripheral.h"
#include "log.h"
#include "wg_net_lib.h"

//#define TEST_MESSAGE_MOVING				//测试信息打印控制
//#define TEST_MESSAGE_FILTER

/* pid 控制环变量声明 */
extern struct PID_DATA Posi_Loop;
extern int left_v, right_v;

/* 错误信息全局变量声明 */
extern unsigned char Error_Type_Pi[6];
extern int agvstate;
extern int taskstate;


/**
* assignment 判断小车是否停止
* parameters
* method
* return 0：没有停止，1：小车停止
*/
int judge_car_stop()
{
	int right_actual_speed;
	int left_actual_speed;
	right_actual_speed = AGV_Get_Motor_Actual_Speed(MOTOR_RIGHT);
	left_actual_speed = -AGV_Get_Motor_Actual_Speed(MOTOR_LEFT);
	if (right_actual_speed <= 10 || left_actual_speed <= 10)			/* 判断是否进入了停止状态 */
		return 1;
	else
		return 0;
}

/**
* assignment 设置小车状态并发送
* parameters index:需要设置的状态
*/
void set_agvstate(int index)
{
	if (agvstate != index)
	{
		agvstate = index;
		sendAgvState(index);
	}
}
/**
* assignment 获得位置误差大小
* parameters
* method 160-centre_point
*/
int get_position_error(int centre_point)
{
	return (160 - centre_point);
}

/**
* assignment 获取误差方向
* parameters
* method
* return -1:误差减小，1：误差增大
*/
int get_error_dir(int centre_point)
{
	static int position_error_last = 0;
	static int error_dir = 1;
	int position_error_now;
	position_error_now = get_position_error(centre_point);
	if (position_error_now*(position_error_now - position_error_last) > 0)		/* 得到误差增长方向，1：误差增大，-1：误差减小 */
		error_dir = 1;
	if (position_error_now*(position_error_now - position_error_last) < 0)
		error_dir = -1;
	position_error_last = position_error_now;	/*更新上次误差值*/
	return error_dir;
}

/**
* assignment 预瞄点信息前期处理
* parameters *centre_point 、 *k_line
* method
*/
void imfor_process(int* centre_point, double* k_line)
{
	if (*centre_point >= 150 && *centre_point <= 170)		/* 设置调节死区 */
	{
		*centre_point = 160;
	}
	if (abs(*centre_point - 160) >= 30)	/* 距离误差大于30时 */
	{
		if ((*k_line > 0 && *centre_point > 160) || (*k_line < 0 && *centre_point < 160))			/* 误差反向时,斜率误差不产生影响 */
			*k_line = 0;
		if (*k_line > 0.10 && *centre_point < 160)		/* 大距离误差下，仍有正向相同误差，增大斜率影响 */
			*k_line = (*k_line) * 2;
		if (*k_line < -0.10 && *centre_point > 160)
			*k_line = (*k_line) * 2;
	}

	if (*k_line >= 1)		/* 预防斜率爆炸 */
		*k_line = 1;
	if (*k_line <= -1)
		*k_line = -1;
}

/**
* assignment 直道控制
* parameters position_error：位置误差大小
* parameters error_dir:误差增长方向
* method 修改系数kp来实现
*/
void straight_path_moving(int position_error, int error_dir)
{
	if (error_dir == 1)
		pidloop_set_kp(&Posi_Loop, 590);
	if (error_dir == -1 && abs(position_error) <= 60)
		pidloop_set_kp(&Posi_Loop, -210);
	pidloop_set_ki(&Posi_Loop, 15);
	//Posi_Loop.ErrorIntegration = 0;
}


/**
* assignment 换道控制
* parameters
* method
*/
void change_path_moving(int position_error, double k_line)
{
	if ((k_line > 0.4 && position_error > 0) || (k_line < -0.4 && position_error < 0))
		pidloop_set_kp(&Posi_Loop, 650);
	else
		pidloop_set_kp(&Posi_Loop, 400 + abs(position_error) / 2);
	pidloop_set_ki(&Posi_Loop, 20);
}


/**
* assignment 弯道控制
* parameters
* method
*/
void curve_path_moving(int position_error)
{
	pidloop_set_ki(&Posi_Loop, 10);		/* 弯道积分作用 */
	pidloop_set_kp(&Posi_Loop, 510 + abs(position_error) / 2);
}

/**
* assignment 根据误差对积分进行处理
*/
void error_integration_process(int position_error, int error_dir)
{
	if (error_dir == -1)
	{
		Posi_Loop.ErrorIntegration -= position_error;
		if (abs(position_error) <= 120)
			Posi_Loop.ErrorIntegration = Posi_Loop.ErrorIntegration*0.8;
		if (abs(position_error) <= 15)
			Posi_Loop.ErrorIntegration = 0;
	}
	if ((Posi_Loop.ErrorIntegration*position_error) < 0)
		Posi_Loop.ErrorIntegration = 0;
	printf("\n>>>>>>   error integration is:%d\n", Posi_Loop.ErrorIntegration);
}


/**
* assignment 判断是否产生振荡
* parameters centrepoint
* method 连续20帧数据的跨幅大小
* return 0:非振荡状态 1：振荡状态
*/
int oscillating_judge(int centrepoint)
{
	static int point_array[20];
	static int judge_result = 0;
	int max = 0;
	int min = 320;
	int i = 0;
	for (i = 18; i >= 0; --i)					/* 更新数组序列 */
	{
		point_array[i + 1] = point_array[i];
	}
	point_array[0] = centrepoint;
	for (i = 0; i < 20; ++i)
	{
		if (point_array[i] <= min)
			min = point_array[i];
		if (point_array[i] >= max)
			max = point_array[i];
	}
	if ((max - min) >= 160)
		judge_result = 1;
	if ((max - min) <= 100)
		judge_result = 0;
	return judge_result;
}

/**
* assignment 弯直道判断
* parameters k_line:曲线斜率
* method 设立阈值以及按照斜率连续性来判断弯直道
* return 0:直道 1：弯道
*/
int curve_straight_judge(double k_line)
{
	static int straight_count = 0;
	static int curve_count = 0;
	if (k_line > 0.15 || k_line < -0.10)
	{
		straight_count = 0;		/* 遇到弯道将计数清零 */
		curve_count += 1;
	}
	else
	{
		curve_count = 0;
		straight_count += 1;
	}
	if (curve_count >= 3)
		return 1;
	else if (straight_count >= 10)
		return 0;
	else
		return 0;				/* 默认为直道 */
}

/**
* assignment 上限函数
* parameters var
* parameters uplimit
*/
void uplimit_fun(int* var, int uplimit)
{
	if (*var > uplimit)
		*var = uplimit;
}

/**
* assignment 下限函数
*/
void lowlimit_fun(int* var, int lowlimit)
{
	if (*var < lowlimit)
		*var = lowlimit;
}

/**
* assignment 获取速度轮差
*/
int getspeed_delta(int centre_point, double k_line)
{
	int speed_delta = 0;
	set_pid_targ_act(&Posi_Loop, 160, centre_point);		/* 位置环控制得到速度应该有的增量,160：中心点位置 */
	speed_delta += pid_realize(&Posi_Loop);
	if (abs(k_line) >= 0.4)									/* 增加斜率影响，作为辅助手段 */
		speed_delta += k_line * 20;
	uplimit_fun(&speed_delta, 130);							/* 界定轮差界限 */
	lowlimit_fun(&speed_delta, -130);
	return speed_delta;
}

/**
* assignment 加速允许判断
* parameters position_error
* method 连续多帧小误差许可
* return 0:不允许加速；1：允许加速
*/
int judge_speed_up(int position_error)
{
	static int label = 0;
	if (abs(position_error) <= 35)
		label += 1;
	else
		label = 0;
	if (label >= 10)
		return 1;
	else
		return 0;
}

/**
* assignment 基本线速度设置
* parameters position_error:位置误差
* parameters setspeed:服务器下设与雷达影响后的线速度
* method
* return 基本线速度
*/
int setspeed_slider(int position_error, double k_line, int setspeed)
{
	int temp = setspeed;		/* 临时记录修正后的下设速度值 */
	int carstop = 0;			/* 判断小车是否停止 */
	static int last_setspeed = 0;
	static int last_position_error = 0;
	static double last_k_line = 0;
	carstop = judge_car_stop();
	if (carstop == 1 && last_setspeed >= 200)
		last_setspeed = 0;
	if (carstop == 0)
	{
		temp = setspeed - (abs(position_error) / 160.0 + abs(k_line))*(setspeed / 12.0*5.0);
		temp = temp - (abs(position_error - last_position_error) / 160.0 + abs(k_line - last_k_line))*(setspeed / 2.0);
		lowlimit_fun(&temp, setspeed / 2.0);
		lowlimit_fun(&temp, 200);
	}
	if (temp - last_setspeed >= 10)
	{
		if (carstop == 1)
			temp = last_setspeed + 10;
		else if (judge_speed_up(position_error))
			temp = last_setspeed + 10;
		else
			temp = last_setspeed;
	}
	if (temp - last_setspeed <= -35)
		temp = last_setspeed - 35;

	last_position_error = position_error;
	last_k_line = k_line;
	last_setspeed = temp;
	return temp;
}
/**
* assignment 下设小车速度
* parameters left_speed,right_speed
*/
void set_car_speed(int left_speed, int right_speed)
{
	AGV_Set_Motor_Speed(MOTOR_LEFT, left_speed);		/* 设定电机速度 */
	AGV_Set_Motor_Speed(MOTOR_RIGHT, right_speed);
}

/**
* assignment 下设速度的最小值不能低于一定数值
*/
void setspeed_lowlimit_process(int* left_set, int* right_set)
{
	if (judge_car_stop())
		return;
	else
	{
		int temp_speed_delta = 0;		/* 两轮轮差值 */
		temp_speed_delta = abs(*left_set - *right_set);
		if (*left_set <= 150 && *right_set >= 150)
		{
			*left_set = 150;
			*right_set = 150 + temp_speed_delta;
			return;
		}
		if (*right_set <= 150 && *left_set >= 150)
		{
			*right_set = 150;
			*left_set = 150 + temp_speed_delta;
		}
	}
}

/**
* assignment 巡线行进电机速度设定
* parameters centre_point: 图像120行处直线位置，图像为240*320
* parameters k_line:	直线斜率
* parameters setspeed: 服务端下设的速度值
* method 位置pid控制环
* return 无
*/
void moving(int centre_point, double k_line, int setspeed, int recspeed)
{
	set_agvstate(3);	/* 设置小车状态为3 */
	int speed_delta;
	int position_error;
	int error_dir;
	double temp;
	int left_speed, right_speed;
	int straight_curve = 0;
	int oscillating_flag = 0;

	straight_curve = curve_straight_judge(k_line);			/* 获取曲直线判断结果 */
	oscillating_flag = oscillating_judge(centre_point);		/* 获取振荡判断结果 */
	temp = (double)setspeed;
	position_error = get_position_error(centre_point);		/* 获取误差大小 */
	error_dir = get_error_dir(centre_point);				/* 获取误差增长方向 */
	imfor_process(&centre_point, &k_line);					/* 预瞄点信息前期处理 */
	if (recspeed == 4 || (recspeed == 2 && straight_curve == 0 && abs(position_error) <= 30) || oscillating_flag == 1)				/* 直线行驶规则 */
		straight_path_moving(position_error, error_dir);
	else if (recspeed == 2)
		curve_path_moving(position_error);
	else
		change_path_moving(position_error, k_line);
	error_integration_process(position_error, error_dir);	/* 误差积分处理 */
	speed_delta = getspeed_delta(centre_point, k_line);		/* 获取速度轮差 */
	setspeed = setspeed_slider(position_error, k_line, setspeed);	/* 根据误差修正速度 */
	speed_delta = 12.0 / (11.0 + (temp / setspeed))*speed_delta;		/* 根据修正后的速度相应地减少调整量 */

	left_speed = setspeed - speed_delta;
	right_speed = setspeed + speed_delta;
	setspeed_lowlimit_process(&left_speed, &right_speed);
	lowlimit_fun(&left_speed, 0);
	lowlimit_fun(&right_speed, 0);
	left_v = left_speed;
	right_v = right_speed;
	set_car_speed(left_speed, right_speed);
}

/**
* assignment 判断小车跟线情况
* parameters centre_point
* method 连续多帧判定
* retrun 0：丢线；1：正常巡线；2：重新巡线
*/
int judge_line_state(int centre_point)
{
	static int line_state = 1;				/* 丢线标志位 0：丢线；1：正常巡线；2：重新巡线 */
	static int losing_count = 0;			/* 丢线计数位 */
	static int return_count = 0;			/* 重巡线计数位 */
	if (centre_point == -1)					/* 判断图像中是否有线，-1：表示无线 */
		return_count = 0;					/* 无线则将归线计数清零 */
	else
		losing_count = 0;					/* 有线则将丢线计数归零 */
	if (line_state == 2 && centre_point != -1)	/* 重新巡线后，下一帧亦非-1，则认为正常巡线 */
		line_state = 1;
	if (line_state != 0 && centre_point == -1)	/* 运行中，判断为巡线状态，图中无线，则丢线计数++ */
		losing_count += 1;
	if (line_state == 0 && centre_point != -1)	/* 运行中，判断为丢线状态，图中有线，则归线计数++ */
		return_count += 1;
	if (losing_count >= 5)					/* 丢线计数>=5,则认为丢线 */
	{
		losing_count = 0;
		line_state = 0;
	}
	if (return_count >= 5)					/* 归线计数>=5,则认为重新巡线 */
	{
		return_count = 0;
		line_state = 2;
	}
	return line_state;
}

/**
* assignment 幅度限制
*/
void range_control(int* centre_point, double* k_line)
{
	if (*centre_point >= 320)
		*centre_point = 320;
	if (*centre_point <= 0)
		*centre_point = 0;
	if (*k_line >= 1)
		*k_line = 1;
	if (*k_line <= -1)
		*k_line = -1;
}


/**
* assignment 对目标点和斜率值进行滤波处理
* parameters *centre_point：目标点
* parameters *k_line:斜率
* method 限幅滤波
* return 无
*/
void filter_processing(int* centre_point, double* k_line)
{
#ifdef TEST_MESSAGE_FILTER			/* 测试时用于创建和打开文件 */
	static int count = 0;			/* 第一次进入时，需要创建当次文件 */
	FILE *fp;
	if (count == 0)
	{
		while ((fp = fopen("/home/pi/carmotion_filter.txt", "wt")) == NULL)
		{
			printf("\n>>>>>>>>>>> create file \"carmotion_filter\" filed   >>>>>>>\n");
		}
		count += 1;
	}
	else
	{
		while ((fp = fopen("/home/pi/carmotion_filter.txt", "at")) == NULL)
		{
			printf("\n>>>>>>>>>>> open file \"carmotion_filter\" filed   >>>>>>>\n");
		}
	}
#endif
	static int last_point = 160;
	static double last_kline = 0;
	int line_state = 1;
	line_state = judge_line_state(*centre_point);
	if (line_state != 0 && *centre_point == -1)			/* 未丢线，而图中无线，保持上一值行进 */
	{
		*centre_point = last_point;
		*k_line = last_kline;
	}
	if (line_state == 0)								/* 丢线处理 */
	{
		if (last_point <= 80)
			*centre_point = last_point - 5;
		if (last_point >= 240)
			*centre_point = last_point + 5;
		*k_line = 0;									/* 丢线时，斜率影响无效 */
	}
	if (line_state == 2 || judge_car_stop() == 1)		/* 重新巡线或者小车停止，跳过限幅滤波*/
	{
		if (*centre_point != -1)
		{
			last_point = *centre_point;
			last_kline = *k_line;
		}
		else
		{
			last_point = 160;
			last_kline = 0;
		}
	}
	if (abs(*centre_point - last_point) >= 120)		/* 限幅滤波 */
	{
		*centre_point = last_point;
		*k_line = last_kline;
	}
	range_control(centre_point, k_line);
	last_point = *centre_point;
	last_kline = *k_line;
#ifdef TEST_MESSAGE_FILTER
	fclose(fp);
#endif
}

/**
* assignment 导出sever端下设的速度
* parameters recspeed:sever端下设的状态
* method 无
* return 导出的速度
*/
int base_speed(int recspeed)
{
	switch (recspeed)
	{
	case 0:						/* 顶框速度 */
		return 200;
		break;
	case 1:						/* 慢速换道 */
		return 500;
		break;
	case 2:						/* 慢速过弯 */
		return 600;
		break;
	case 4:						/* 快速直行 */
		return 900;
		break;
	default:
		return 0;
		break;
	}
}

/**
* assignment button状态影响
* parameters motion_flag:按钮影响motion_flag
* method 无
* return 无
*/
void button_effect(int* motion_flag)
{
	static int button_state = 0, last_button_state = 0;
	static int press_count = 3;
	button_state = Agv_Peripheral_Get_Startbutton_State();
	if (button_state != last_button_state)
	{
		press_count += 1;
		last_button_state = button_state;
	}
	if (press_count % 2 == 0)
	{
		printf("stop by press stop button");
		*motion_flag = 1;
		Error_Type_Pi[3] = 1;
		Agv_Peripheral_Set_LED_State(0);
	}
	else
	{
		Error_Type_Pi[3] = 0;
		Agv_Peripheral_Set_LED_State(1);
	}
}

/**
* assignment 防撞条影响
* parameters motionflag:防撞条影响motion_flag
* method 无
* return 无
*/
void bumper_effect(int* motion_flag)
{
	if (Agv_Peripheral_Get_Bumper_State())
	{
		*motion_flag = 13;
		printf("stop by agv_bump!");
		Error_Type_Pi[2] = 1;
	}
	else
	{
		Error_Type_Pi[2] = 0;
	}
}

/**
* assignment AGV停止
* parameters left_v:左轮速度
* parameters right_v
* parameters time_ms:延迟多少毫秒
*/
void stop_agv_ms(int time_ms)
{
	int left_v, right_v;
	left_v = -AGV_Get_Motor_Actual_Speed(MOTOR_LEFT);	/* 获取当前实际速度，左轮需要反向 */
	right_v = AGV_Get_Motor_Actual_Speed(MOTOR_RIGHT);

	while (left_v > 0 || right_v > 0)
	{
		if (left_v > 20)
			left_v = left_v - 20;
		else
			left_v = 0;
		if (right_v > 20)
			right_v = right_v - 20;
		else
			right_v = 0;
		AGV_Set_Motor_Speed(MOTOR_LEFT, left_v);
		AGV_Set_Motor_Speed(MOTOR_RIGHT, right_v);
		usleep(time_ms * 1000);
	}
}

/**
* assignment 打开声光报警
*/
void agv_alarm_on()
{
	Agv_Peripheral_Set_Bright_Alarm(1);
	Agv_Peripheral_Set_Voice_Alarm(1);
}

/**
* assignment 熄灭声光报警
*/
void agv_alarm_off()
{
	Agv_Peripheral_Set_Bright_Alarm(0);
	Agv_Peripheral_Set_Voice_Alarm(0);
}

/**
* assignment 停车处理
* parameters motionflag:状态标志，据此进行各种处理
* method 无
* return 无
*/
void agv_stop_processing(int* motionflag)
{
	if (*motionflag == 9 || *motionflag == 1 || *motionflag == 13)
	{
		set_agvstate(4);
		if (*motionflag != 1)
		{
			agv_alarm_on();
		}
		stop_agv_ms(5);
		sleep(1);
	}
	if (*motionflag == 5)
	{
		set_agvstate(4);
		if (*motionflag != 1)
		{
			agv_alarm_on();
		}
		stop_agv_ms(5);
		usleep(50 * 1000);
	}
	if (*motionflag == 17 || *motionflag == 21)
	{
		stop_agv_ms(5);
	}
}

/**
* assignment 急停按钮拍下影响
* parameters motionflag
* method 无
* return 无
*/
void safe_button_effect(int* motionflag)
{
	if (Agv_Peripheral_Get_stop_State())	/*急停按钮被拍下*/
	{
		*motionflag = 21;
	}
}


/**
* assignment 激光雷达影响
* parameters motionflag:
* parameters setspeed:下设速度
* method 距离判断
* return 无
*/
void radar_effect(int* motionflag, int* setspeed)
{
	switch (agv_pidmotor_get_out())			/* 激光雷达判断 */
	{
	case 0:
		break;

	case 1:
		*motionflag = 9;
		Error_Type_Pi[0] = 1;
		break;
	case 2:
		if (*setspeed >= 350)
			*setspeed = 350;
		Error_Type_Pi[0] = 0;
		break;
	case 3:
		*motionflag = 9;
		Error_Type_Pi[0] = 1;
		break;
	case 4:
		if (*setspeed >= 500)
			*setspeed = 500;
		Error_Type_Pi[0] = 0;
		break;
	case 5:
		*motionflag = 9;
		Error_Type_Pi[0] = 1;
		break;
	case 6:
		if (*setspeed >= 350)
			*setspeed = 350;
		Error_Type_Pi[0] = 0;
		break;
	case 7:
		*motionflag = 9;
		Error_Type_Pi[0] = 1;
		break;
	default:
		break;
	}
}

/**
* assignment AGV丢线措施
* parameters 全局变量Posi_Loop
* method 一轮旋转，一轮停止，利用误差方向确定变化值
* return 无
*/
void losing_processing()
{
	if (Posi_Loop.Error_K_0 < 0)
	{
		AGV_Set_Motor_Speed(MOTOR_LEFT, 200);
		AGV_Set_Motor_Speed(MOTOR_RIGHT, 0);
	}
	if (Posi_Loop.Error_K_0 > 0)
	{
		AGV_Set_Motor_Speed(MOTOR_LEFT, 0);
		AGV_Set_Motor_Speed(MOTOR_RIGHT, 200);
	}
}


/**
* assignment AGV行进策略
* parameter centre_point: 图像90行处直线位置，图像为240*320
* parameter k_line:	直线斜率
* parameter motionflag: 小车状态
* parameter recspeed: 获得的速度
* method 无
* return 无
*/
void carmotion(int centre_point, double k_line, int* motionflag, int recspeed)
{
	int setspeed;			/* 服务端下设的小车速度值 */
	filter_processing(&centre_point, &k_line);		/* 滤波器处理数据 */
	setspeed = base_speed(recspeed);				/* 根据recspeed得到下设速度值 */
	button_effect(motionflag);						/* 按钮状态影响:motionflag==1 */
	bumper_effect(motionflag);						/* 防撞条状态影响:motionflag==13 */
	safe_button_effect(motionflag);					/* 急停按钮被按下:motionflag==21 */
	radar_effect(motionflag, &setspeed);			/* 激光雷达距离影响:motionflag==9 */
	switch ((*motionflag) % 4)
	{
	case 1:		/* 车停止 */
		agv_stop_processing(motionflag);
		break;
	case 2:		/* 正常行走 */
		agv_alarm_off();			/* 熄灭报警 */
		moving(centre_point, k_line, setspeed, recspeed);
		break;
	case 3:		/* 慢速运行 */
		agv_alarm_off();
		moving(centre_point, k_line, setspeed, recspeed);
		break;
	case 0:		/* 丢线重寻 */
		losing_processing();
		break;
	default:
		break;
	}
}
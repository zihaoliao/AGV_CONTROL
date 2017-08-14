#ifndef _CARMOTION_H_
#define	_CARMOTION_H_

/**
* assignment 判断小车是否停止
* parameters
* method
* return 0：没有停止，1：小车停止
*/
int judge_car_stop();

/**
* assignment 设置小车状态并发送
* parameters index:需要设置的状态
*/
void set_agvstate(int index);

/**
* assignment 获得位置误差大小
* parameters
* method 160-centre_point
*/
int get_position_error(int centre_point);

/**
* assignment 获取误差方向
* parameters
* method
* return -1:误差减小，1：误差增大
*/
int get_error_dir(int centre_point);

/**
* assignment 预瞄点信息前期处理
* parameters *centre_point 、 *k_line
* method
*/
void imfor_process(int* centre_point, double* k_line);

/**
* assignment 直道控制
* parameters position_error：位置误差大小
* parameters error_dir:误差增长方向
* method 修改系数kp来实现
*/
void straight_path_moving(int position_error, int error_dir);

/**
* assignment 换道控制
* parameters
* method
*/
void change_path_moving(int position_error, double k_line);

/**
* assignment 弯道控制
* parameters
* method
*/
void curve_path_moving(int position_error);

/**
* assignment 根据误差对积分进行处理
*/
void error_integration_process(int position_error, int error_dir);

/**
* assignment 判断是否产生振荡
* parameters centrepoint
* method 连续20帧数据的跨幅大小
* return 0:非振荡状态 1：振荡状态
*/
int oscillating_judge(int centrepoint);

/**
* assignment 弯直道判断
* parameters k_line:曲线斜率
* method 设立阈值以及按照斜率连续性来判断弯直道
* return 0:直道 1：弯道
*/
int curve_straight_judge(double k_line);

/**
* assignment 上限函数
* parameters var
* parameters uplimit
*/
void uplimit_fun(int* var, int uplimit);

/**
* assignment 下限函数
*/
void lowlimit_fun(int* var, int lowlimit);

/**
* assignment 获取速度轮差
*/
int getspeed_delta(int centre_point, double k_line);

/**
* assignment 加速允许判断
* parameters position_error
* method 连续多帧小误差许可
* return 0:不允许加速；1：允许加速
*/
int judge_speed_up(int position_error);

/**
* assignment 基本线速度设置
* parameters position_error:位置误差
* parameters setspeed:服务器下设与雷达影响后的线速度
* method
* return 基本线速度
*/
int setspeed_slider(int position_error, double k_line, int setspeed);

/**
* assignment 下设小车速度
* parameters left_v,right_v
*/
void set_car_speed(int left_v, int right_v);


/**
* assignment 下设速度的最小值不能低于一定数值
*/
void setspeed_lowlimit_process(int* left_set,int* right_set);



/**
* assignment 巡线行进电机速度设定
* parameters centre_point: 图像120行处直线位置，图像为240*320
* parameters k_line:	直线斜率
* parameters setspeed: 服务端下设的速度值
* method 位置pid控制环
* return 无
*/
void moving(int centre_point, double k_line, int setspeed, int recspeed);

/**
* assignment 判断小车跟线情况
* parameters centre_point
* method 连续多帧判定
* retrun 0：丢线；1：正常巡线；2：重新巡线
*/
int judge_line_state(int centre_point);

/**
* assignment 幅度限制
*/
void range_control(int* centre_point, double* k_line);

/**
* assignment 对目标点和斜率值进行滤波处理
* parameters *centre_point：目标点
* parameters *k_line:斜率
* method 限幅滤波
* return 无
*/
void filter_processing(int* centre_point, double* k_line);

/**
* assignment 导出sever端下设的速度
* parameters recspeed:sever端下设的状态
* method 无
* return 导出的速度
*/
int base_speed(int recspeed);

/**
* assignment button状态影响
* parameters motion_flag:按钮影响motion_flag
* method 无
* return 无
*/
void button_effect(int* motion_flag);

/**
* assignment 防撞条影响
* parameters motionflag:防撞条影响motion_flag
* method 无
* return 无
*/
void bumper_effect(int* motion_flag);

/**
* assignment AGV停止
* parameters left_v:左轮速度
* parameters right_v
* parameters time_ms:延迟多少毫秒
*/
void stop_agv_ms(int time_ms);

/**
* assignment 打开声光报警
*/
void agv_alarm_on();

/**
* assignment 熄灭声光报警
*/
void agv_alarm_off();

/**
* assignment 停车处理
* parameters motionflag:状态标志，据此进行各种处理
* method 无
* return 无
*/
void agv_stop_processing(int* motionflag);

/**
* assignment 急停按钮拍下影响
* parameters motionflag
* method 无
* return 无
*/
void safe_button_effect(int* motionflag);

/**
* assignment 激光雷达影响
* parameters motionflag:
* parameters setspeed:下设速度
* method 距离判断
* return 无
*/
void radar_effect(int* motionflag, int* setspeed);

/**
* assignment AGV丢线措施
* parameters 全局变量Posi_Loop
* method 一轮旋转，一轮停止，利用误差方向确定变化值
* return 无
*/
void losing_processing();

/**
* assignment AGV行进策略
* parameter centre_point: 图像90行处直线位置，图像为240*320
* parameter k_line:	直线斜率
* parameter motionflag: 小车状态
* parameter recspeed: 获得的速度
* method 无
* return 无
*/
void carmotion(int centre_point, double k_line, int* motionflag, int recspeed);


#endif
#include<cmath>
#include<iostream>
#include<aris.hpp>
#include"kinematics.h"
#include"plan.h"

using namespace std;

extern double file_current_leg[18];
extern double file_current_body[16];
extern double temp_body_pos[16];
extern double temp_leg_pos[18];
extern double relate_body_pos[16];
extern double PI;

static double current_body_in_ground2[16] = { 0 };
static double current_leg_in_ground2[18] = { 0 };

//-------------------------------------------------------梯形曲线----------------------------------------------------//

//生成梯形曲线0->1
//输入：时间，每毫秒计数一次
//输出：当前时刻s的值
auto TCurve::getTCurve(int count)->double
{
	//double ta = p.ta_;
	//double a = p.a_;
	//double v = p.v_;
	//double T_c = p.Tc_;
	int t = count + 1;
	double s = 0;

	if (2 * ta_ == Tc_)   //三角形曲线
	{
		if (t < ta_ * 1000)
		{
			s = 0.5 * a_ * t * t / 1000.0 / 1000.0;
		}
		else
		{
			s = 0.5 * a_ * ta_ * ta_ + 0.5 * (t / 1000.0 - ta_) * (2 * v_ - a_ * (t / 1000.0 - ta_));
		}
	}
	else    //梯形曲线
	{
		if (t < ta_ * 1000)
		{
			s = 0.5 * a_ * t * t / 1000.0 / 1000.0;
		}
		else if (t >= ta_ * 1000 && t < (Tc_ * 1000 - ta_ * 1000))
		{
			s = v_ * t / 1000 - v_ * v_ / 2.0 / a_;
		}
		else
		{
			s = (2 * a_ * v_ * Tc_ - 2 * v_ * v_ - a_ * a_ * (t / 1000.0 - Tc_) * (t / 1000.0 - Tc_)) / (2 * a_);
		}
	}
	//std::cout << s << std::endl;
	return s;
}

//计算梯形曲线的参数，由成员函数初始化，对应输入参数由构造函数初始化
auto TCurve::getCurveParam()->void
{
	if (v_ * v_ / a_ <= 1) //梯形曲线
	{
		this->Tc_ = (a_ + v_ * v_) / v_ / a_;
		this->a_ = a_;
		this->v_ = v_;
	}
	else
	{
		//按速度计算，此时给定的加速度不起作用
		this->Tc_ = 2.0 / v_;  //三角形曲线
		this->a_ = v_ * v_;
		this->v_ = v_;
	}
	this->ta_ = v_ / a_;
}


//-----------------------------梯形曲线2，传入a,v,s-------------------------------------------//
//这个有个问题需要注意，就是同样的a，v但输入的距离不同，时间周期会不一样，使用的时候注意一下，选择适当的梯形曲线
auto TCurve2::getTCurve(int count)->double
{
	int t = count ; //单位为秒
	double s = 0;

	if (2 * ta_ == Tc_)   //三角形曲线
	{
		if (t < ta_*1000)
		{
			s = 0.5 * a_ * t * t / 1000 / 1000;
		}
		else
		{
			s = 0.5 * a_ * ta_ * ta_ + 0.5 * (t / 1000.0 - ta_) * (2 * v_ - a_ * (t / 1000.0 - ta_));
		}
	}
	else    //梯形曲线
	{
		if (t < ta_ * 1000)
		{
			s = 0.5 * a_ * t * t / 1000.0 / 1000.0;
		}
		else if (t >= ta_ * 1000 && t < (Tc_ * 1000 - ta_ * 1000))
		{
			s = v_ * t / 1000 - v_ * v_ / 2.0 / a_;
		}
		else
		{
			s = (2 * a_ * v_ * Tc_ - 2 * v_ * v_ - a_ * a_ * (t / 1000.0 - Tc_) * (t / 1000.0 - Tc_)) / (2 * a_);
		}
	}
	return s;
}

auto TCurve2::getCurveParam()->void
{
	if (v_ * v_ / a_ <= d_) //梯形曲线
	{
		this->Tc_ = (d_ * a_ + v_ * v_) / v_ / a_;
		this->a_ = a_;
		this->v_ = v_;
	}
	else
	{
		//按速度计算，此时给定的加速度不起作用
		this->Tc_ = 2.0 * d_ / v_;  //三角形曲线
		this->a_ = v_ * v_ / d_;
		this->v_ = v_;
	}
	this->ta_ = v_ / a_;
}


//---------------------------------------------------------椭圆轨迹---------------------------------------------------//

//生成椭圆轨迹，在Tc时间内  x方向0->a/2;y方向0->b->0;z方向0->c/2。对应输入参数由构造函数初始化。
//参数构造为椭圆的长轴和短轴，所以这个地方xz会除2
auto EllipseTrajectory::getEllipseTrajectory(int count)->void
{
	x_ = a_ * (1 + cos(PI - PI * s_.getTCurve(count))) / 2.0;
	y_ = b_ * sin(PI - PI * s_.getTCurve(count));
	z_ = c_ * (1 + cos(PI - PI * s_.getTCurve(count))) / 2.0;
	//std::cout << y << std::endl;
}

//-------------------------------------------------直线轨迹  （身体） ------------------------------------------------//
auto BodyMoveTrajectory::getStraightTraj(int count)->void
{
	x_ = a_ * s_.getTCurve(count);
	y_ = b_ * s_.getTCurve(count);
	z_ = c_ * s_.getTCurve(count);
}


//-------------------------------------------------直线轨迹  （腿） ------------------------------------------------//
auto StraightTrajectory::getStraightTraj(int count)->void
{
	x_ = a_ * s_.getTCurve(count);
	y_ = b_ * s_.getTCurve(count);
	z_ = c_ * s_.getTCurve(count);
}

//-------------------------------------------------圆周轨迹  （腿） ------------------------------------------------//
auto CircleTrajectory::getCircleTraj(int count)->void
{
	this->get_s().getCurveParam();
	int T = this->get_s().getTc();
	double theta = n_*2 * PI * this->get_s().getTCurve(count);
	//这些都是过程量（变化量）
	if (dir_ > 0) {//逆时针
		x_ = r_ * (cos(theta) - 1);
	}
	else {
		x_ = r_ * (1 - cos(theta));
	}
	
	y_ = 0;
	z_ = r_ * sin(theta);
}



//-----------------------------------------------------身体旋转角度轨迹----------------------------------------------//

//生成身体绕xyz三轴旋转的角度轨迹0->theta  角度为弧度
auto BodyPose::getBodyRotationTrajectory(int count)->void
{
	pitch_ = pitch_angle_z_ * PI * b_r_s_.getTCurve(count) / 180.0;  //pitch  转化为弧度 并按照梯形曲线生成
	roll_ = roll_angle_x_ * PI * b_r_s_.getTCurve(count) / 180.0;
	yaw_ = yaw_angle_y_ * PI * b_r_s_.getTCurve(count) / 180.0;

	//std::cout << b_r_s_.getTCurve << std::endl;
	//std::cout << pitch_ << std::endl;
}



//---------------------------------------------------规划脚---------------------------------------------//

///足尖在笛卡尔坐标系下的规划。每执行完一次梯形曲线记录一次数据
///count=e_2 是0 到 Tc 循环  
///判断当前在走哪一步,腿走一步e1加1

//三角步态
//当前脚的位置 = 上一步脚的位置 + 脚位置增量
//#注意：目前只适用于平地行走
//e_1用来记录当前走到第几步，共需要走n步，count是一次迈腿的计时，故不能把总的时钟传入，要处理
auto planLegTripod(int e_1, int n, double* current_leg, int count, EllipseTrajectory* Ellipse)->void
{
	if (count == 0)//初始化脚的位置
	{
		for (int i = 0; i < 18; ++i)
		{
			current_leg[i] = foot_position_start_point[i];
		}
	}

	Ellipse->getEllipseTrajectory(count);

	if (e_1 % 2 == 0)  //偶数135迈腿，246停
	{
		if (e_1 == 0)   //加速段
		{
			//规划leg1
			current_leg[0] = foot_position_start_point[0] + Ellipse->get_x() / 2;
			current_leg[1] = foot_position_start_point[1] + Ellipse->get_y();
			current_leg[2] = foot_position_start_point[2] + Ellipse->get_z() / 2;
			//规划leg3
			current_leg[6] = foot_position_start_point[6] + Ellipse->get_x() / 2;
			current_leg[7] = foot_position_start_point[7] + Ellipse->get_y();
			current_leg[8] = foot_position_start_point[8] + Ellipse->get_z() / 2;
			//leg5
			current_leg[12] = foot_position_start_point[12] + Ellipse->get_x() / 2;
			current_leg[13] = foot_position_start_point[13] + Ellipse->get_y();
			current_leg[14] = foot_position_start_point[14] + Ellipse->get_z() / 2;
		}
		else
		{
			//规划leg1
			current_leg[0] = foot_position_start_point[0] + Ellipse->get_x();
			current_leg[1] = foot_position_start_point[1] + Ellipse->get_y();
			current_leg[2] = foot_position_start_point[2] + Ellipse->get_z();


			//规划leg3
			current_leg[6] = foot_position_start_point[6] + Ellipse->get_x();
			current_leg[7] = foot_position_start_point[7] + Ellipse->get_y();
			current_leg[8] = foot_position_start_point[8] + Ellipse->get_z();
			//leg5
			current_leg[12] = foot_position_start_point[12] + Ellipse->get_x();
			current_leg[13] = foot_position_start_point[13] + Ellipse->get_y();
			current_leg[14] = foot_position_start_point[14] + Ellipse->get_z();
		}
	}
	else if (e_1 % 2 == 1)  //奇数24迈腿13停
	{
		if (e_1 == (2 * n - 1))//减速段
		{
			//规划leg2
			current_leg[3] = foot_position_start_point[3] + Ellipse->get_x() / 2;
			current_leg[4] = foot_position_start_point[4] + Ellipse->get_y();
			current_leg[5] = foot_position_start_point[5] + Ellipse->get_z() / 2;
			//规划leg4
			current_leg[9] = foot_position_start_point[9] + Ellipse->get_x() / 2;
			current_leg[10] = foot_position_start_point[10] + Ellipse->get_y();
			current_leg[11] = foot_position_start_point[11] + Ellipse->get_z() / 2;
			//leg6
			current_leg[15] = foot_position_start_point[15] + Ellipse->get_x() / 2;
			current_leg[16] = foot_position_start_point[16] + Ellipse->get_y();
			current_leg[17] = foot_position_start_point[17] + Ellipse->get_z() / 2;
		}
		else
		{
			//规划leg2
			current_leg[3] = foot_position_start_point[3] + Ellipse->get_x();
			current_leg[4] = foot_position_start_point[4] + Ellipse->get_y();
			current_leg[5] = foot_position_start_point[5] + Ellipse->get_z();
			//规划leg4
			current_leg[9] = foot_position_start_point[9] + Ellipse->get_x();
			current_leg[10] = foot_position_start_point[10] + Ellipse->get_y();
			current_leg[11] = foot_position_start_point[11] + Ellipse->get_z();
			//leg6
			current_leg[15] = foot_position_start_point[15] + Ellipse->get_x();
			current_leg[16] = foot_position_start_point[16] + Ellipse->get_y();
			current_leg[17] = foot_position_start_point[17] + Ellipse->get_z();
		}
	}

	if (count + 1 == std::floor(Ellipse->get_s().getTc() * 1000)) //floor 函数 向下取整
	{
		for (int i = 0; i < 18; ++i)
		{
			foot_position_start_point[i] = current_leg[i];
		}
	}
}

//三角步态下原地旋转
//规划脚竖直上下抬起。然后乘旋转矩阵？？
//#注意：目前只适用于平地
auto planLegTripodTurn(int e_1, double* current_leg, int count, EllipseTrajectory* Ellipse, BodyPose* body_pose_param)->void
{

	//count 是0 到 Tc 循环  //判断当前在走哪一步,腿走一步e1加1
	
	if (count == 0)//初始化脚的位置
	{
		for (int i = 0; i < 18; ++i)
		{
			current_leg[i] = foot_position_start_point[i];
		}
	}
	Ellipse->getEllipseTrajectory(count);

	double temp_xyz_in_ground[18] = { 0 };
	static double yaw = 0;
	//每个梯形曲线开始时读取之前的值
	if (count == 0)
	{
		yaw = 0;
	}

	body_pose_param->getBodyRotationTrajectory(count); //得到随count变化的rpy
	yaw = body_pose_param->getCurrentYaw(); //得到当前时刻的yaw

	double R_y[16] = {
						 std::cos(yaw), 0, std::sin(yaw), 0,
								0, 1,        0, 0,
						-std::sin(yaw), 0, std::cos(yaw), 0,
								0, 0,        0, 1
	};

	//按正常规划，脚上下抬起
	if (e_1 % 2 == 0)  //偶数135迈腿，246停
	{
		//规划leg1
		temp_xyz_in_ground[0] = foot_position_start_point[0] + Ellipse->get_x();
		temp_xyz_in_ground[1] = foot_position_start_point[1] + Ellipse->get_y();
		temp_xyz_in_ground[2] = foot_position_start_point[2] + Ellipse->get_z();


		//规划leg3
		temp_xyz_in_ground[6] = foot_position_start_point[6] + Ellipse->get_x();
		temp_xyz_in_ground[7] = foot_position_start_point[7] + Ellipse->get_y();
		temp_xyz_in_ground[8] = foot_position_start_point[8] + Ellipse->get_z();

		//规划leg5
		temp_xyz_in_ground[12] = foot_position_start_point[12] + Ellipse->get_x();
		temp_xyz_in_ground[13] = foot_position_start_point[13] + Ellipse->get_y();
		temp_xyz_in_ground[14] = foot_position_start_point[14] + Ellipse->get_z();

		aris::dynamic::s_pp2pp(R_y, temp_xyz_in_ground + 0 * 3, current_leg + 0 * 3);
		aris::dynamic::s_pp2pp(R_y, temp_xyz_in_ground + 2 * 3, current_leg + 2 * 3);
		aris::dynamic::s_pp2pp(R_y, temp_xyz_in_ground + 4 * 3, current_leg + 4 * 3);


	}
	else if (e_1 % 2 == 1)  //奇数246迈腿135停
	{
		//规划leg2
		temp_xyz_in_ground[3] = foot_position_start_point[3] + Ellipse->get_x();
		temp_xyz_in_ground[4] = foot_position_start_point[4] + Ellipse->get_y();
		temp_xyz_in_ground[5] = foot_position_start_point[5] + Ellipse->get_z();
		//规划leg4
		temp_xyz_in_ground[9] = foot_position_start_point[9] + Ellipse->get_x();
		temp_xyz_in_ground[10] = foot_position_start_point[10] + Ellipse->get_y();
		temp_xyz_in_ground[11] = foot_position_start_point[11] + Ellipse->get_z();
		//规划leg6
		temp_xyz_in_ground[15] = foot_position_start_point[15] + Ellipse->get_x();
		temp_xyz_in_ground[16] = foot_position_start_point[16] + Ellipse->get_y();
		temp_xyz_in_ground[17] = foot_position_start_point[17] + Ellipse->get_z();

		aris::dynamic::s_pp2pp(R_y, temp_xyz_in_ground + 1 * 3, current_leg + 1 * 3);
		aris::dynamic::s_pp2pp(R_y, temp_xyz_in_ground + 3 * 3, current_leg + 3 * 3);
		aris::dynamic::s_pp2pp(R_y, temp_xyz_in_ground + 5 * 3, current_leg + 5 * 3);
	}


	//每完成一个梯形曲线后记录一次脚的位置
	if (count + 1 == floor(Ellipse->get_s().getTc() * 1000))
	{
		for (int i = 0; i < 18; ++i)
		{
			foot_position_start_point[i] = current_leg[i];
		}
	}
}

//四足步态
//当前脚的位置 = 上一步脚的位置 + 脚位置增量
//#注意：目前只适用于平地行走
auto planLegTetrapod(int e_1, int n, double* current_leg, int count, EllipseTrajectory* Ellipse)->void
{
	if (count == 0)//初始化脚的位置
	{
		for (int i = 0; i < 18; ++i)
		{
			current_leg[i] = foot_position_start_point[i];
		}
	}
	Ellipse->getEllipseTrajectory(count);


	//14  25  36  
	if (e_1  % 3 == 0)  //迈14腿
	{
		if (e_1 == 0)   //加速段
		{
			//规划leg1
			current_leg[0] = foot_position_start_point[0] + Ellipse->get_x() / 2;
			current_leg[1] = foot_position_start_point[1] + Ellipse->get_y();
			current_leg[2] = foot_position_start_point[2] + Ellipse->get_z() / 2;

			//规划leg4
			current_leg[9] = foot_position_start_point[9] + Ellipse->get_x() / 2;
			current_leg[10] = foot_position_start_point[10] + Ellipse->get_y();
			current_leg[11] = foot_position_start_point[11] + Ellipse->get_z() / 2;
		}
		else if (e_1 == 3 * n - 3) //减速段
		{
			//规划leg1  
			current_leg[0] = foot_position_start_point[0] + Ellipse->get_x() / 2;
			current_leg[1] = foot_position_start_point[1] + Ellipse->get_y();
			current_leg[2] = foot_position_start_point[2] + Ellipse->get_z() / 2;

			//规划leg4
			current_leg[9] = foot_position_start_point[9] + Ellipse->get_x() / 2;
			current_leg[10] = foot_position_start_point[10] + Ellipse->get_y();
			current_leg[11] = foot_position_start_point[11] + Ellipse->get_z() / 2;
		}
		else  //匀速段
		{
			//规划leg1 
			current_leg[0] = foot_position_start_point[0] + Ellipse->get_x();
			current_leg[1] = foot_position_start_point[1] + Ellipse->get_y();
			current_leg[2] = foot_position_start_point[2] + Ellipse->get_z();

			//规划leg4 
			current_leg[9] = foot_position_start_point[9] + Ellipse->get_x();
			current_leg[10] = foot_position_start_point[10] + Ellipse->get_y();
			current_leg[11] = foot_position_start_point[11] + Ellipse->get_z();
		}
	}
	else if (e_1  % 3 == 1)  //迈25腿
	{
		if (e_1 == 1)//加速段
		{
			//规划leg2
			current_leg[3] = foot_position_start_point[3] + Ellipse->get_x() / 2;
			current_leg[4] = foot_position_start_point[4] + Ellipse->get_y();
			current_leg[5] = foot_position_start_point[5] + Ellipse->get_z() / 2;
			//规划leg5
			current_leg[12] = foot_position_start_point[12] + Ellipse->get_x() / 2;
			current_leg[13] = foot_position_start_point[13] + Ellipse->get_y();
			current_leg[14] = foot_position_start_point[14] + Ellipse->get_z() / 2;
		}
		else if (e_1 == 3 * n - 2)  //减速段
		{
			//规划leg2
			current_leg[3] = foot_position_start_point[3] + Ellipse->get_x() / 2;
			current_leg[4] = foot_position_start_point[4] + Ellipse->get_y();
			current_leg[5] = foot_position_start_point[5] + Ellipse->get_z() / 2;
			//规划leg5
			current_leg[12] = foot_position_start_point[12] + Ellipse->get_x() / 2;
			current_leg[13] = foot_position_start_point[13] + Ellipse->get_y();
			current_leg[14] = foot_position_start_point[14] + Ellipse->get_z() / 2;
		}
		else
		{
			//规划leg2
			current_leg[3] = foot_position_start_point[3] + Ellipse->get_x();
			current_leg[4] = foot_position_start_point[4] + Ellipse->get_y();
			current_leg[5] = foot_position_start_point[5] + Ellipse->get_z();
			//规划leg5
			current_leg[12] = foot_position_start_point[12] + Ellipse->get_x();
			current_leg[13] = foot_position_start_point[13] + Ellipse->get_y();
			current_leg[14] = foot_position_start_point[14] + Ellipse->get_z();
		}
	}
	else if (e_1  % 3 == 2)  //迈36腿
	{
		if (e_1 == 2)//加速段
		{
			//规划leg3
			current_leg[6] = foot_position_start_point[6] + Ellipse->get_x() / 2;
			current_leg[7] = foot_position_start_point[7] + Ellipse->get_y();
			current_leg[8] = foot_position_start_point[8] + Ellipse->get_z() / 2;

			//规划leg6
			current_leg[15] = foot_position_start_point[15] + Ellipse->get_x() / 2;
			current_leg[16] = foot_position_start_point[16] + Ellipse->get_y();
			current_leg[17] = foot_position_start_point[17] + Ellipse->get_z() / 2;
		}
		else if (e_1 == 3 * n - 1) //减速段
		{
			//规划leg3
			current_leg[6] = foot_position_start_point[6] + Ellipse->get_x() / 2;
			current_leg[7] = foot_position_start_point[7] + Ellipse->get_y();
			current_leg[8] = foot_position_start_point[8] + Ellipse->get_z() / 2;

			//规划leg6
			current_leg[15] = foot_position_start_point[15] + Ellipse->get_x() / 2;
			current_leg[16] = foot_position_start_point[16] + Ellipse->get_y();
			current_leg[17] = foot_position_start_point[17] + Ellipse->get_z() / 2;
		}
		else  //匀速段
		{
			//规划leg3
			current_leg[6] = foot_position_start_point[6] + Ellipse->get_x();
			current_leg[7] = foot_position_start_point[7] + Ellipse->get_y();
			current_leg[8] = foot_position_start_point[8] + Ellipse->get_z();

			//规划leg6
			current_leg[15] = foot_position_start_point[15] + Ellipse->get_x();
			current_leg[16] = foot_position_start_point[16] + Ellipse->get_y();
			current_leg[17] = foot_position_start_point[17] + Ellipse->get_z();
		}
	}


	if (count == floor(Ellipse->get_s().getTc() * 1000) - 1)
	{
		for (int i = 0; i < 18; ++i)
		{
			foot_position_start_point[i] = current_leg[i];
		}
	}
}


//------------------------------------------------规划身体----------------------------------------------//

//本函数用于规划六足机器人在三角步态下身体的位置轨迹，不加旋转（姿态变换）
//当前身体的位置 = 上一步身体的位置 + 身体位置增量
//每当结束一次命令是，身体的位置更新一次
//#注意：目前只适用于平地行走
auto planBodyTransformTripod(int e_1, int n, double* current_body, int count, EllipseTrajectory* Ellipse)->void
{
	int per_step_count = Ellipse->get_s().getTc() * 1000;
	if (count == 0) //初始化身体位姿
	{
		for (int i = 0; i < 16; ++i)
		{
			current_body[i] = body_position_start_point[i];
		}
	}

	if (e_1 == 0)   //加速段
	{
		//规划身体
		current_body[3] = body_position_start_point[3] + Ellipse->get_a() * count * count / (4.0 * per_step_count * per_step_count);
		current_body[7] = body_position_start_point[7];
		current_body[11] = body_position_start_point[11] + Ellipse->get_c() * count * count / (4.0 * per_step_count * per_step_count);
	}
	else if (e_1 == (2 * n - 1))//减速段
	{

		//规划身体
		int t = (2 * n - 1) * per_step_count + per_step_count;
		current_body[3] = body_position_start_point[3] + +0 - Ellipse->get_a() * (count - t) * (count - t) / (4.0 * per_step_count * per_step_count) + Ellipse->get_a() * n - Ellipse->get_a() / 2.0;//n * a
		current_body[7] = body_position_start_point[7];
		current_body[11] = body_position_start_point[11] + 0 - Ellipse->get_c() * (count - t) * (count - t) / (4.0 * per_step_count * per_step_count) + Ellipse->get_c() * n - Ellipse->get_c() / 2.0;
	}
	else //匀速段
	{
		//规划身体
		current_body[3] = body_position_start_point[3] + Ellipse->get_a() / 4.0 + Ellipse->get_a() * (count - per_step_count) / per_step_count / 2;//速度为75mm/s  每秒计数per_step_count
		current_body[7] = body_position_start_point[7];
		current_body[11] = body_position_start_point[11] + Ellipse->get_c() / 4.0 + Ellipse->get_c() * (count - per_step_count) / per_step_count / 2;
	}
	//整个身体运动为梯形曲线
	if (count + 1 >= 2 * n * per_step_count)
	{
		for (int i = 0; i < 16; ++i)
		{
			body_position_start_point[i] = current_body[i];
		}
	}
}


//规划身体用于腿不动，只移动身体
auto planBodyTransformOnlyBodyMove(double* current_body, int count, BodyMoveTrajectory* bodyCurve)->void
{
	bodyCurve->getStraightTraj(count);

	current_body[3] = temp_body_pos[3] + bodyCurve->get_x();
	current_body[7] = temp_body_pos[7] + bodyCurve->get_y();
	current_body[11] = temp_body_pos[11] + bodyCurve->get_z();



}

//规划腿直线运动，身体不动
//current_leg 是腿末端的三个坐标xyz
auto planLegTransformOnlyLegMove(double* current_leg, int leg_num, int count, StraightTrajectory* legCurve)->void
{
	legCurve->getStraightTraj(count);
	//leg_num是第几条腿

	current_leg[leg_num * 3 + 0] = temp_leg_pos[leg_num * 3 + 0] + legCurve->get_x();
	current_leg[leg_num * 3 + 1] = temp_leg_pos[leg_num * 3 + 1] + legCurve->get_y();
	current_leg[leg_num * 3 + 2] = temp_leg_pos[leg_num * 3 + 2] + legCurve->get_z();


}


//规划腿圆周运动，身体不动
//current_leg 是腿末端的三个坐标xyz,这个相当于重载，根据参数类型不同
auto planLegTransformOnlyLegMove(double* current_leg, int leg_num, int count, CircleTrajectory* legCurve)->void
{
	legCurve->getCircleTraj(count);
	//leg_num是第几条腿

	current_leg[leg_num * 3 + 0] = temp_leg_pos[leg_num * 3 + 0] + legCurve->get_x();
	current_leg[leg_num * 3 + 1] = temp_leg_pos[leg_num * 3 + 1] + legCurve->get_y();
	current_leg[leg_num * 3 + 2] = temp_leg_pos[leg_num * 3 + 2] + legCurve->get_z();


}




//本函数用于规划六足机器人原地旋转
//每一个梯形曲线转过给定角度的一半，和tripod步态对应
//count 是 0 -> count
auto planBodyTurn(int count, double* current_body, BodyPose* body_pose_param)->void
{
	double roll = 0;
	double yaw = 0;
	double pitch = 0;
	


	if (count == 1) //有用，不能删，否则算不出角度
	{
		for (int i = 0; i < 16; ++i)
		{
			current_body[i] = body_position_start_point[i];
		}

	}
	
	body_pose_param->getBodyRotationTrajectory(count); //由body设置的角度参数得到弧度制的旋转角（随时间）

	yaw = body_pose_param->getCurrentYaw() / 2;
	roll = body_pose_param->getCurrentRoll() / 2;
	pitch = body_pose_param->getCurrentPitch() / 2;

	double R_y[16] = {
						 std::cos(yaw), 0, std::sin(yaw), 0,
								0, 1,        0, 0,
						-std::sin(yaw), 0, std::cos(yaw), 0,
								0, 0,        0, 1
	};

	double R_r[16] = {
					 std::cos(roll),	-std::sin(roll),	0,					0,
					 std::sin(roll),	std::cos(roll),		0,					0,
					 0,					0,					1,					0,
					 0,					0,					0,					1
	};

	double R_p[16] = {
					 1,					0,					0,					0,
					 0,					std::cos(pitch),	-std::sin(pitch),	0,
					 0,					std::sin(pitch),	std::cos(pitch),	0,
					 0,					0,					0,					1
	};

	double tempy[16] = { 0 };
	double tempr[16] = { 0 };
	double tempp[16] = { 0 };

	aris::dynamic::s_pm_dot_pm(temp_body_pos, R_y, tempy); //矩阵相乘。tempy得到的是旋转后的身体位姿
	aris::dynamic::s_pm_dot_pm(tempy, R_r, tempr);
	aris::dynamic::s_pm_dot_pm(tempr, R_p, tempp);
	std::copy(tempp, tempp + 16, current_body); //copy（拷贝内容的首地址，尾地址，拷贝目的地的首地址） 得到current_body




	//结束时保存变化之后的值
	if (count+1  == std::floor(body_pose_param->getTcurve().getTc() * 1000)) //std::floor 向下取整数
	{

		for (int i = 0; i < 16; i++)
		{
			temp_body_pos[i] = current_body[i];
		}
		//std::cout << "aaaaaaaa" << std::endl;
		//std::cout << "count = " << count << std::endl;
		//coutMatrix16(current_body);

	}
}

//本函数用于规划六足机器人在四足步态下身体的位置轨迹，不加旋转（姿态变换）
//当前身体的位置 = 上一步身体的位置 + 身体位置增量
//每当结束一次命令是，身体的位置更新一次
//#注意：目前只适用于平地行走
auto planBodyTransformTetrapod(int e_1, int n, double* current_body, int count, EllipseTrajectory* Ellipse)->void
{
	int per_step_count = Ellipse->get_s().getTc() * 1000;
	if (count == 0) //初始化身体位姿
	{
		for (int i = 0; i < 16; ++i)
		{
			current_body[i] = body_position_start_point[i];
		}
	}

	if (e_1 <= 2)   //加速段
	{
		//规划身体
		current_body[3] = body_position_start_point[3] + Ellipse->get_a() * count * count / (18.0 * per_step_count * per_step_count);
		current_body[7] = body_position_start_point[7];
		current_body[11] = body_position_start_point[11] + Ellipse->get_c() * count * count / (18.0 * per_step_count * per_step_count);
	}
	else if (e_1 >= (3 * n - 3))//减速段
	{

		//规划身体
		int t = 3 * n * per_step_count;
		current_body[3] = body_position_start_point[3] - Ellipse->get_a() * (count - t) * (count - t) / (18.0 * per_step_count * per_step_count) + Ellipse->get_a() * (n - 1);//n * a
		current_body[7] = body_position_start_point[7];
		current_body[11] = body_position_start_point[11] - Ellipse->get_c() * (count - t) * (count - t) / (18.0 * per_step_count * per_step_count) + Ellipse->get_c() * (n - 1);
	}
	else //匀速段
	{
		//规划身体
		current_body[3] = body_position_start_point[3] + Ellipse->get_a() / 2.0 + Ellipse->get_a() * (count -3* per_step_count) / per_step_count / 3;//速度为75mm/s  每秒计数per_step_count
		current_body[7] = body_position_start_point[7];
		current_body[11] = body_position_start_point[11] + Ellipse->get_c() / 2.0 + Ellipse->get_c() * (count -3* per_step_count) / per_step_count / 3;
	}
	//整个身体运动为梯形曲线
	if (count + 1 >= 3 * n * per_step_count)
	{
		for (int i = 0; i < 16; ++i)
		{
			body_position_start_point[i] = current_body[i];
		}
	}
}

//--------------------------------------------------步态规划---------------------------------------------------------//

//机器人行走三角步态，包括原地踏步、前进、后退、左移、右移。
//其中步长步高和步数可由用户输入。走一步的时间（或行走快慢）可由用户输入梯形曲线的速度和加速度确定
//#注意：行走最大速度和加速度还没测试
auto tripodPlan(int n, int count, EllipseTrajectory* Ellipse, double* input)->int
{

	int per_step_count = Ellipse->get_s().getTc() * 1000;

	static double current_leg_in_ground[18] = { 0 };
	static double current_body_in_ground[16] = { 0 };
	//判断行走状态
	int e_1 = count / per_step_count;  //判断当前在走哪一步,腿走一步,e1加1
	int e_2 = count % per_step_count;  //在一次迈腿周期0->Tc 的count


	//规划腿
	//e_1是判断当前走到第几步，e_2是当前Tc内的count
	//身体和腿的Tc不是一个Tc
	planLegTripod(e_1, n, current_leg_in_ground, e_2, Ellipse);
	//规划身体
	planBodyTransformTripod(e_1, n, current_body_in_ground, count, Ellipse);

	//模型测试使用
	for (int i = 0; i < 18; ++i)
	{

		file_current_leg[i] = current_leg_in_ground[i];
	}
	for (int i = 0; i < 16; ++i)
	{
		file_current_body[i] = current_body_in_ground[i];
	}
	inverseLeg(current_leg_in_ground, current_body_in_ground, input);

	return 2 * n * per_step_count - count - 1;
}



//身体平移规划
//身体一步到达某个位置，不用几步
auto bodyStaightMovePlan(int count, BodyMoveTrajectory* bodyCurve, double* input)->int
{
	int per_step_count = bodyCurve->get_s().getTc() * 1000;


	

	//腿不动，只用规划身体
	//规划身体
	planBodyTransformOnlyBodyMove(current_body_in_ground2, count, bodyCurve);
	aris::dynamic::s_vc(16, body_position_start_point + 0, file_current_body + 0);

	file_current_body[3] = current_body_in_ground2[3];
	file_current_body[7] = current_body_in_ground2[7];
	file_current_body[11] = current_body_in_ground2[11];


	int tempret = per_step_count - count - 1;


	if (tempret == 0) {
		//更新参考坐标值
		temp_body_pos[3] = temp_body_pos[3] + bodyCurve->get_a();
		temp_body_pos[7] = temp_body_pos[7] + bodyCurve->get_b();
		temp_body_pos[11] = temp_body_pos[11] + bodyCurve->get_c();


	}

	//模型测试使用
	return tempret;

}



//把一条腿一步移动到某个位置
auto legStaightMovePlan(int count, int leg_num,  StraightTrajectory* legCurve, double* input)->int
{

	int per_step_count = legCurve->get_s().getTc() * 1000;

	//规划腿
	planLegTransformOnlyLegMove(current_leg_in_ground2,leg_num, count, legCurve); //current_leg是记录在一个梯形曲线周期内的变化值，temp_leg是起始值，故每个周期结束后，要更新temp_leg
	//把变化的腿的坐标传入file_current_leg
	
	//给特殊腿赋变化值
	file_current_leg[leg_num * 3 + 0] = current_leg_in_ground2[leg_num * 3 + 0];
	file_current_leg[leg_num * 3 + 1] = current_leg_in_ground2[leg_num * 3 + 1];
	file_current_leg[leg_num * 3 + 2] = current_leg_in_ground2[leg_num * 3 + 2];


	int tempret = per_step_count - count - 1;


	if (tempret == 0) {
		//更新参考坐标值
		temp_leg_pos[leg_num * 3 + 0] = temp_leg_pos[leg_num * 3 + 0] + legCurve->get_a();
		temp_leg_pos[leg_num * 3 + 1] = temp_leg_pos[leg_num * 3 + 1] + legCurve->get_b();
		temp_leg_pos[leg_num * 3 + 2] = temp_leg_pos[leg_num * 3 + 2] + legCurve->get_c();



	}

	//模型测试使用
	return tempret;

}

//指定某条腿在原本高度进行平面运动
auto legCircleMovePlan(int count, int leg_num, CircleTrajectory* legCurve, double* input)->int
{

	int per_step_count = legCurve->get_s().getTc() * 1000;

	//规划腿
	planLegTransformOnlyLegMove(current_leg_in_ground2, leg_num, count, legCurve); //current_leg是记录在一个梯形曲线周期内的变化值，temp_leg是起始值，故每个周期结束后，要更新temp_leg
	//把变化的腿的坐标传入file_current_leg

	//给特殊腿赋变化值
	file_current_leg[leg_num * 3 + 0] = current_leg_in_ground2[leg_num * 3 + 0];
	file_current_leg[leg_num * 3 + 1] = current_leg_in_ground2[leg_num * 3 + 1];
	file_current_leg[leg_num * 3 + 2] = current_leg_in_ground2[leg_num * 3 + 2];


	int tempret = per_step_count - count - 1;


	if (tempret == 0) {
		//貌似无需更新参考坐标值，因为圆周运动会回到原点


	}

	//模型测试使用
	return tempret;

}




//机器人行走四足步态，包括原地踏步、前进、后退、左移、右移。
//其中步长步高和步数可由用户输入。走一步的时间（或行走快慢）可由用户输入梯形曲线的速度和加速度确定
//#注意：行走最大速度和加速度还没测试
auto tetrapodPlan(int n, int count, EllipseTrajectory* Ellipse, double* input)->int
{

	int per_step_count = Ellipse->get_s().getTc() * 1000;

	static double current_leg_in_ground[18] = { 0 };
	static double current_body_in_ground[16] = { 0 };
	//判断行走状态
	int e_1 = count / per_step_count;  //判断当前在走哪一步,腿走一步,e1加1
	int e_2 = count % per_step_count;  //0->Tc count


	//规划腿
	planLegTetrapod(e_1, n, current_leg_in_ground, e_2, Ellipse);
	//规划身体
	planBodyTransformTetrapod(e_1, n, current_body_in_ground, count, Ellipse);

	//模型测试使用
	for (int i = 0; i < 18; ++i)
	{
		file_current_leg[i] = current_leg_in_ground[i];
	}
	for (int i = 0; i < 16; ++i)
	{
		file_current_body[i] = current_body_in_ground[i];
	}
	inverseLeg(current_leg_in_ground, current_body_in_ground, input);

	return 3 * n * per_step_count - count - 1;
}


//对角步态下原地旋转
auto turnPlanTripod(int n, int count, EllipseTrajectory* Ellipse, BodyPose* body_pose_param, double* input)->int
{

	int per_step_count = Ellipse->get_s().getTc() * 1000;

	static double current_leg_in_ground[18] = { 0 };
	static double current_body_in_ground[16] = { 0 };

	//判断行走状态
	int e_1 = count / per_step_count;  //判断当前在走哪一步,腿走一步,e1加1
	int e_2 = count % per_step_count;  //0->Tc count

	//规划腿
	planLegTripodTurn(e_1, current_leg_in_ground, e_2, Ellipse, body_pose_param);
	//规划身体
	planBodyTurn(e_2, current_body_in_ground, body_pose_param);


	//模型测试使用
	for (int i = 0; i < 18; ++i)
	{
		file_current_leg[i] = current_leg_in_ground[i];
	}
	for (int i = 0; i < 16; ++i)
	{
		file_current_body[i] = current_body_in_ground[i];
	}
	//模型测试使用

	inverseLeg(current_leg_in_ground, current_body_in_ground, input);
	

	return  per_step_count * n * 2 - count - 1;
}























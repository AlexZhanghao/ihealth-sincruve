#pragma once
#ifndef SINCURVE_H
#define SINCURVE_H
#include"control_card.h"
//#include"boundarydetection.h"
#include<vector>

using namespace std;

struct Teach {
	vector<double>Target_Pos[2];//存储位置的数组
	vector<double>Target_Vel[2];//存储速度的数组
	vector<double>Interpolation_Data[2];//存储插值后的数组
	//std::vector<double>Voltage_Data[4];//存储电压值、命令速度和反馈速度
	double Time;//运动时间 s
};

class sincurve {
public:
	vector<double> sincurvepos[2];
	vector<double> sincruvevel[2];
};

class passivemove {
public:
	passivemove();
	~passivemove();

	//进行sin曲线运动
	void BeginSincruveMove();
	//进行直接点到点运动
	void BeginPtpMove();
	void StopMove();
	void MoveStart();
	void StartSampling();
	void FeedbackTraceExport();
	void InterpolationExport();

public:
	//打开线程
	bool stopthread ;
	//开始运动
	bool beginMove;



private:
	double PHermite(double foretime[2],//已知时间点
		double forepos[2],//已知位置点
		double forevel[2],//已知速度点
		double t);////所求时间点

private:
	double Her_Teach_Time[2][2];//插值运动位置的时间
	double  Her_Teach_Vel[2][2];//插值运动的始末速度
	double  Her_Teach_Pos[2][2];//插值运动的始末位置							
	I32 opt = 0x5000;

	sincurve curve;
	Teach movesample;
};

#endif
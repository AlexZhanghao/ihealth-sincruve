#include"sincurve.h"
#include<process.h>
#include<Windows.h>
#include<iostream>
#include <fstream>

#define TIMER_SLEEP   0.1

const bool MotorOn = 1;
const bool MotorOff = 0;
const bool ClutchOn = 0;
const bool ClutchOff = 1;
const int OutputChannels = 24;
const int InputChannels = 24;
const int ShoulderAxisId = 0;
const int ElbowAxisId = 1;
const double Unit_Convert = 0.009;
int loop_count;//计数器
int Target_count;//插值计数
passivemove::passivemove() {
	stopthread = true;
	beginMove = false;

	double s_vel[101] = { -1.099377,-1.098292,-1.096123,-1.092872,-1.088543,-1.083140,-1.076667,-1.069133,-1.060543,-1.050906,-1.040232,-1.028532,-1.015817,-1.002099,-0.987392,-0.971711,-0.955071,-0.937488,-0.918980,-0.899566,-0.879263,-0.858093,-0.836076,-0.813233,-0.789589,-0.765165,-0.739985,-0.714076,-0.687462,-0.660169,-0.632225,-0.603657,-0.574494,-0.544763,-0.514495,-0.483718,-0.452465,-0.420765,-0.388650,-0.356151,-0.323301,-0.290132,-0.256676,-0.222967,-0.189038,-0.154923,-0.120654,-0.086267,-0.051794,-0.017270,0.017270,0.051794,0.086267,0.120654,0.154923,0.189038,0.222967,0.256676,0.290132,0.323301,0.356151,0.388650,0.420765,0.452465,0.483718,0.514495,0.544763,0.574494,0.603657,0.632225,0.660169,0.687462,0.714076,0.739985,0.765165,0.789589,0.813233,0.836076,0.858093,0.879263,0.899566,0.918980,0.937488,0.955071,0.971711,0.987392,1.002099,1.015817,1.028532,1.040232,1.050906,1.060543,1.069133,1.076667,1.083140,1.088543,1.092872,1.096123,1.098292,1.099377,1.099377 };
	double e_vel[101] = { -1.413484,-1.412089,-1.409301,-1.405121,-1.399555,-1.392608,-1.384287,-1.374599,-1.363555,-1.351165,-1.337442,-1.322398,-1.306050,-1.288413,-1.269504,-1.249343,-1.227948,-1.205342,-1.181546,-1.156584,-1.130481,-1.103262,-1.074954,-1.045586,-1.015185,-0.983783,-0.951410,-0.918098,-0.883880,-0.848789,-0.812861,-0.776131,-0.738635,-0.700409,-0.661493,-0.621924,-0.581741,-0.540984,-0.499693,-0.457909,-0.415673,-0.373026,-0.330012,-0.286672,-0.243049,-0.199186,-0.155127,-0.110914,-0.066592,-0.022205,0.022205,0.066592,0.110914,0.155127,0.199186,0.243049,0.286672,0.330012,0.373026,0.415673,0.457909,0.499693,0.540984,0.581741,0.621924,0.661493,0.700409,0.738635,0.776131,0.812861,0.848789,0.883880,0.918098,0.951410,0.983783,1.015185,1.045586,1.074954,1.103262,1.130481,1.156584,1.181546,1.205342,1.227948,1.249343,1.269504,1.288413,1.306050,1.322398,1.337442,1.351165,1.363555,1.374599,1.384287,1.392608,1.399555,1.405121,1.409301,1.412089,1.413484, };
	double s_pos[101] = { 0.000000,-1.099377,-2.197668,-3.293791,-4.386663,-5.475206,-6.558346,-7.635013,-8.704146,-9.764689,-10.815595,-11.855827,-12.884359,-13.900176,-14.902275,-15.889667,-16.861379,-17.816450,-18.753938,-19.672918,-20.572484,-21.451747,-22.309840,-23.145915,-23.959149,-24.748737,-25.513902,-26.253887,-26.967963,-27.655425,-28.315595,-28.947820,-29.551477,-30.125971,-30.670734,-31.185228,-31.668947,-32.121412,-32.542177,-32.930827,-33.286978,-33.610279,-33.900411,-34.157087,-34.380054,-34.569092,-34.724015,-34.844669,-34.930935,-34.982730,-35.000000,-34.982730,-34.930935,-34.844669,-34.724015,-34.569092,-34.380054,-34.157087,-33.900411,-33.610279,-33.286978,-32.930827,-32.542177,-32.121412,-31.668947,-31.185228,-30.670734,-30.125971,-29.551477,-28.947820,-28.315595,-27.655425,-26.967963,-26.253887,-25.513902,-24.748737,-23.959149,-23.145915,-22.309840,-21.451747,-20.572484,-19.672918,-18.753938,-17.816450,-16.861379,-15.889667,-14.902275,-13.900176,-12.884359,-11.855827,-10.815595,-9.764689,-8.704146,-7.635013,-6.558346,-5.475206,-4.386663,-3.293791,-2.197668,-1.099377,0 };
	double e_pos[101] = { 0.000000,-1.413484,-2.825573,-4.234874,-5.639996,-7.039551,-8.432159,-9.816446,-11.191045,-12.554600,-13.905765,-15.243206,-16.565605,-17.871655,-19.160068,-20.429572,-21.678915,-22.906864,-24.112206,-25.293752,-26.450336,-27.580817,-28.684080,-29.759034,-30.804620,-31.819805,-32.803588,-33.754998,-34.673096,-35.556976,-36.405765,-37.218626,-37.994757,-38.733391,-39.433801,-40.095294,-40.717217,-41.298958,-41.839942,-42.339635,-42.797543,-43.213216,-43.586242,-43.916254,-44.202926,-44.445975,-44.645162,-44.800288,-44.911203,-44.977795,-45.000000,-44.977795,-44.911203,-44.800288,-44.645162,-44.445975,-44.202926,-43.916254,-43.586242,-43.213216,-42.797543,-42.339635,-41.839942,-41.298958,-40.717217,-40.095294,-39.433801,-38.733391,-37.994757,-37.218626,-36.405765,-35.556976,-34.673096,-33.754998,-32.803588,-31.819805,-30.804620,-29.759034,-28.684080,-27.580817,-26.450336,-25.293752,-24.112206,-22.906864,-21.678915,-20.429572,-19.160068,-17.871655,-16.565605,-15.243206,-13.905765,-12.554600,-11.191045,-9.816446,-8.432159,-7.039551,-5.639996,-4.234874,-2.825573,-1.413484,0 };

	for (int i = 0; i < 101; ++i) {
		curve.sincruvevel[0].push_back(s_vel[i]);
	}
	for (int i = 0; i < 101; ++i) {
		curve.sincruvevel[1].push_back(e_vel[i]);
	}
	for (int i = 0; i < 101; ++i) {
		curve.sincurvepos[0].push_back(s_pos[i]);
	}
	for (int i = 0; i < 101; ++i) {
		curve.sincurvepos[1].push_back(e_pos[i]);
	}
}

passivemove::~passivemove() {

}

unsigned int __stdcall ThreadFun(PVOID pParam) {
	passivemove *pasvCtrl = (passivemove*)pParam;
	UINT oldTickCount, newTickCount;
	oldTickCount = GetTickCount();
	while (TRUE) {
		//延时 TIMER_SLEEP s
		while (TRUE) {
			newTickCount = GetTickCount();
			if (newTickCount - oldTickCount >= TIMER_SLEEP * 1000) {
				oldTickCount = newTickCount;
				break;
			}

			else
				SwitchToThread();
		}
		if (pasvCtrl->stopthread) {
			break;
		}

		//是否开始运动
		if (pasvCtrl->beginMove) {
			pasvCtrl->MoveStart();
			pasvCtrl->StartSampling();
		}
			
		loop_count++;

	}

	pasvCtrl->FeedbackTraceExport();
	pasvCtrl->InterpolationExport();

	cout << "Thread end!";
	return 0;
}

void passivemove::BeginSincruveMove() {
	//初始化
	ControlCard::GetInstance().Initial();
	cout << "Initial ok!" << endl;

	//复位
	ControlCard::GetInstance().ResetPosition();
	cout << "Reset position ok!" << endl;

	//打开电机，离合器
	ControlCard::GetInstance().SetMotor(MotorOn);
	ControlCard::GetInstance().SetClutch(ClutchOn);
	cout << "motor and clutch on!" << endl;

	//打开线程
	stopthread = false;
	//开始运动
	beginMove = true;
	//计数器置零
	loop_count = 0;
	Target_count = 0;

	HANDLE handle;
	handle = (HANDLE)_beginthreadex(NULL, 0, ThreadFun, this, 0, NULL);
}

void passivemove::StopMove() {
	//关闭电机
	ControlCard::GetInstance().SetMotor(MotorOff);
	//关闭离合器
	ControlCard::GetInstance().SetClutch(ClutchOff);
	//关闭线程
	stopthread = true;
	beginMove = false;
	cout << "motion stop!" << endl;

	//复位
	ControlCard::GetInstance().ResetPosition();
	cout << "Reset position ok!" << endl;
}

double passivemove::PHermite(double foretime[2], double forepos[2], double forevel[2], double t)
{
	double Houtput = 0;
	double a[2] = { 0 };
	double b[2] = { 0 };
	a[0] = (1 - 2 * (t - foretime[0]) / (foretime[0] - foretime[1]))*pow((t - foretime[1]) / (foretime[0] - foretime[1]), 2);
	a[1] = (1 - 2 * (t - foretime[1]) / (foretime[1] - foretime[0]))*pow((t - foretime[0]) / (foretime[1] - foretime[0]), 2);
	b[0] = (t - foretime[0])*pow((t - foretime[1]) / (foretime[0] - foretime[1]), 2);
	b[1] = (t - foretime[1])*pow((t - foretime[0]) / (foretime[1] - foretime[0]), 2);
	Houtput = a[0] * forepos[0] + a[1] * forepos[1] + b[0] * forevel[0] + b[1] * forevel[1];
	return Houtput;
}

void passivemove::MoveStart() {
	double Teach_Time = loop_count*0.1;
	I32 Axis[2] = { ShoulderAxisId, ElbowAxisId };

	//每过一秒就更新插值区间
	if (loop_count % 10 == 0) {
		if (Target_count < curve.sincruvevel[0].size() - 1) {
			for (int i = 0; i<2; i++) {
				//更新插值时间范围
				Her_Teach_Time[i][0] = Teach_Time;
				Her_Teach_Time[i][1] = Teach_Time + 1;
				//更新插值位置范围
				Her_Teach_Pos[i][0] = curve.sincurvepos[i].at(Target_count);
				Her_Teach_Pos[i][1] = curve.sincurvepos[i].at(Target_count + 1);
				//更新插值速度范围
				Her_Teach_Vel[i][0] = curve.sincruvevel[i].at(Target_count);
				Her_Teach_Vel[i][1] = curve.sincruvevel[i].at(Target_count + 1);
			}
			Target_count++;
		}
		else {
			//运动完成，停止运动
			StopMove();
		}
	}

	//在插值区间内，相当于每100ms就运动到一个新的插值点。这里的插值有问题，连续发了两次命令
	for (int j = 0; j < 2; j++) {
		double Teach_Cmd = PHermite(Her_Teach_Time[j],
			Her_Teach_Pos[j],
			Her_Teach_Vel[j],
			Teach_Time);
		//APS_absolute_move(Axis[j], Teach_Cmd / Unit_Convert, 15 / Unit_Convert);
		APS_ptp_v(Axis[j], opt, Teach_Cmd / Unit_Convert, 15 / Unit_Convert, NULL);
		movesample.Interpolation_Data[j].push_back(Teach_Cmd);
	}
}

void passivemove::StartSampling() {
	//取角度
	double joint_angle[2]{ 0 };
	//double joint_vel[2]{ 0 };
	ControlCard::GetInstance().GetEncoderData(joint_angle);
	//ControlCard::GetInstance().GetJointVelocity(joint_vel);
	for (int i = 0; i < 2; i++) {
		movesample.Target_Pos[i].push_back(joint_angle[i]);
		//movesample.Target_Vel[i].push_back(joint_vel[i]);
	}
}

void passivemove::FeedbackTraceExport() {
	ofstream dataFile3;
	dataFile3.open("positiontracedata.txt", ofstream::app);
	dataFile3 << "shoulder_practical_trace" << "   " << "elbow_practical_trace" << endl;
	for (int i = 0; i < movesample.Target_Pos[0].size(); ++i) {
		dataFile3 << movesample.Target_Pos[0][i] << "        " << movesample.Target_Pos[1][i] << endl;
	}
	dataFile3.close();
}

void passivemove::InterpolationExport() {
	ofstream dataFile1;
	dataFile1.open("interpolation.txt", ofstream::app);
	dataFile1 << "interpolation_sholuder" << "   " << "interpolation_elbow" << endl;
	for (int i = 0; i < movesample.Interpolation_Data[0].size(); ++i) {
		dataFile1 << movesample.Interpolation_Data[0][i] << "        " << movesample.Interpolation_Data[1][i] << endl;
	}
	dataFile1.close();
}

void passivemove::BeginPtpMove() {
	//初始化
	ControlCard::GetInstance().Initial();
	cout << "Initial ok!" << endl;

	//复位
	ControlCard::GetInstance().ResetPosition();
	cout << "Reset position ok!" << endl;

	//打开电机，离合器
	ControlCard::GetInstance().SetMotor(MotorOn);
	ControlCard::GetInstance().SetClutch(ClutchOn);
	cout << "motor and clutch on!" << endl;

	//开始运动
	int axis_id = 0;
	double trip = 100;
	APS_absolute_move(axis_id , trip / Unit_Convert, 5 / Unit_Convert);
}
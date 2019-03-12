#pragma once
#ifndef SINCURVE_H
#define SINCURVE_H
#include"control_card.h"
//#include"boundarydetection.h"
#include<vector>

using namespace std;

struct Teach {
	vector<double>Target_Pos[2];//�洢λ�õ�����
	vector<double>Target_Vel[2];//�洢�ٶȵ�����
	vector<double>Interpolation_Data[2];//�洢��ֵ�������
	//std::vector<double>Voltage_Data[4];//�洢��ѹֵ�������ٶȺͷ����ٶ�
	double Time;//�˶�ʱ�� s
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

	//����sin�����˶�
	void BeginSincruveMove();
	//����ֱ�ӵ㵽���˶�
	void BeginPtpMove();
	void StopMove();
	void MoveStart();
	void StartSampling();
	void FeedbackTraceExport();
	void InterpolationExport();

public:
	//���߳�
	bool stopthread ;
	//��ʼ�˶�
	bool beginMove;



private:
	double PHermite(double foretime[2],//��֪ʱ���
		double forepos[2],//��֪λ�õ�
		double forevel[2],//��֪�ٶȵ�
		double t);////����ʱ���

private:
	double Her_Teach_Time[2][2];//��ֵ�˶�λ�õ�ʱ��
	double  Her_Teach_Vel[2][2];//��ֵ�˶���ʼĩ�ٶ�
	double  Her_Teach_Pos[2][2];//��ֵ�˶���ʼĩλ��							
	I32 opt = 0x5000;

	sincurve curve;
	Teach movesample;
};

#endif
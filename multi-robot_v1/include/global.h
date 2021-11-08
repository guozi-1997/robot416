#ifndef _GLOABLE_H
#define _GLOABLE_H

#include <disp_manager.h>
#include <video_manager.h>
#include <convert_manager.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <config.h>
#include <render.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include "MyMatrix.h"
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <math.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <input_manager.h>

// #define dis_50 67.0 //75  //1=67.0 2=65.0   3=65   4=66

#define a11 0.3683
#define a12 0.9848
#define a13 1.1477

#define a21 0.1409
#define a22 0.2431
#define a23 2.0380

#define a31 0.0090
#define a32 0.1257
#define a33 3.4491

#define a41 0.0038
#define a42 0.0601
#define a43 4.9448
#define ROBOT_WIDTH 40 //单位厘米

#define delta_l 20 //步长设置为20cm

#define ROLLWINDOW_R 1000.0

#define TrackNodeNum 20
//#define cir_x  370
//#define cir_y  300
int cir_x;
int cir_y;

int dlta_d;
int dlta_a;

int get_x;
int get_y;
int get_tht;

int differentAngle[2];
//char ekfRecvBuf[3][300];
#define cell(x, y, d) (4 * (x)-3200 * (y) + cir_x * 4 + 3200 * cir_y + d)
#define DAY 2 // 1白天  0 黑夜荧光灯照明 2 自适应属于硬件配置addr[ii*3200+jj*4+0]
#define big_cell(x, y, d) (4 * (x) + 4096 * (y) + d)

struct get_point
{
	unsigned int r;
	unsigned int g;
	unsigned int b;
} get_point1, get_point2, get_point3, get_point4;

/* struct ekf
{
	int i;

	MATRIX *H;
	MATRIX *X;	//后验估计（最优修正值，最终要的就是这个）
	MATRIX *X_; //先验估计
	MATRIX *P;
	MATRIX *P_;
	MATRIX *K;
	MATRIX *K_;
	MATRIX *Z; //观测值
	MATRIX *K__;
	MATRIX *Q;
	MATRIX *R;

	REAL tht; //存本机与目标的电子罗盘航向角角度差？？？
	REAL old_tht1;
	REAL tht2; //存本机与目标的电子罗盘航向角角度差？？？

	int tht3;
	int tht4; //目标机器人航位推算的角度（目标的偏转角）
	int tht5;

	int tht6;
	int tht7;
	int tht8; //本机航位推算的角度（本机的偏转角）

	int x1;
	int x2; //本机航位推算的x值
	int x3;
	int y1;
	int y2; //本机航位推算的y值
	int y3;
	int distance; //本机航位推算的移动距离（本机的前进距离）

	int x4;
	int x5; //目标机器人航位推算的x值
	int x6;
	int y4;
	int y5; //目标机器人航位推算的y值
	int y6;
	int distance1; //目标机器人航位推算的的移动距离（目标机器人的前进距离）

	int count;
} ekf111[4]; */
struct ekf
{
	int i;
	Matrix A;
	Matrix H;
	Matrix X;  //后验估计（最优修正值，最终要的就是这个）
	Matrix X_; //先验估计
	Matrix P;
	Matrix P_;
	Matrix K;
	Matrix K_;
	Matrix Z; //观测值
	Matrix K__;
	Matrix Q;
	Matrix R;
	double Avalue[4];
	double Hvalue[4];
	double Xvalue[2];
	double X_value[2];
	double oldXvalue[2];
	double Zvalue[2];
	double Rvalue[4];
	double Qvalue[4];
	double tht; //存本机与目标的电子罗盘航向角角度差？？？
	double old_tht1;
	double tht2; //存本机与目标的电子罗盘航向角角度差？？？

	int tht3;
	int tht4; //目标机器人航位推算的角度（目标的偏转角）
	int tht5;

	int tht6;
	int tht7;
	int tht8; //本机航位推算的角度（本机的偏转角）

	int x1;
	int x2; //本机航位推算的x值
	int x3;
	int y1;
	int y2; //本机航位推算的y值
	int y3;
	int distance; //本机航位推算的移动距离（本机的前进距离）

	int x4;
	int x5; //目标机器人航位推算的x值
	int x6;
	int y4;
	int y5; //目标机器人航位推算的y值
	int y6;
	int distance1; //目标机器人航位推算的的移动距离（目标机器人的前进距离）

	int count;
} ekf[4];

struct robot
{
	unsigned int rad;	//radius半径（表示观测机器人所成全景图像中，与目标机器人之间的像素个数）
	int angle;			//本机全景图像观测到目标机器人的观测角度
	unsigned int count; //全景图像中的特征（像素）点个数
	int x;
	int y;
	int distance; //本机全景图像观测到目标机器人的观测距离
	float dis_diff;
	int number;
	int dlta_a;	  //目标机器人（发过来的）角速度
	int dlta_d;	  //目标机器人（发过来的）线速度
	int Ture_Tht; //存本机与目标的电子罗盘航向角角度差？？？
} robot_temp[7], robot_end[7], robot_send[7], robot_rece[7], robot_ekf[7], robot_ekf2[7], robot_self[7];
struct ekf_send
{
	int dlta_a; //
	int dlta_d; //
	int distance[10];
	int angle[10];
	int bef_distance[10];
	int bef_angle[10];
} ekf_send[10];
T_VideoDevice tVideoDevice_mid; //
PT_VideoBuf ptVideoBufCur_mid;	//
T_VideoBuf tVideoBuf_mid;		//
T_VideoBuf tFrameBuf_mid;		//

T_VideoDevice tVideoDevice_now; //
PT_VideoBuf ptVideoBufCur_now;	//
T_VideoBuf tVideoBuf_now;		//
T_VideoBuf tFrameBuf_now;		//

int Ture_Tht;	 //电子罗盘航向角，磁北方向始（沿顺时针）到当前方向止的夹角（当罗盘水平旋转的时候，航向角在0—360度之间变化）
int First_Angle; //记录本机器人初始的航向角角度
int first_flag;	 //为配合 First_Angle 设置的标志位
int file_flag;
float CurLidarDist;
int CurLidarQuality;

int CurLidarDistPix2;
float CurLidarAng2;

int AimCount;
int CurLidarDistPix[1024];
int CurLidarAng[1024];		//lidar每个采样点的角度值
int CurLidarDistance[1024]; //lidar每个采样点的距离值

int TrueAimCount;
int TrueCurLidarAng[2000];
int TrueCurLidarDistPix[2000]; //仅用于绘制图像

int TrueAimCntDis;
int TrueCurLidarAng2[1024];
int TrueCurLidarDistance[1024];

int FilterCount;
int FilterCount2;
int FilterLidarAng[2000];
int FilterLidarDistPix[2000];
int FilterLidarAng2[2000];
int FilterLidarDistance[2000];
int showFiltercount;
int showFilterLidarDistPix[2000];
int showFilterLidarAng[2000];

int LidarEdgeCnt;
int LidarEdgeDistPix[1024];
int LidarEdgeAngle[1024];
int LidarEdgeDistance[1024];

int OkPathCnt;
int OkPathDistPix[10];
int OkPathAngle[10];
int OkPathDistance[10];

int LidarOkPathMidCnt;
int LidarOkPathMidAng[10];
int LidarOkPathMidPix[10];
int LidarOkPathMidDistance[10];

int PicEdgePixCnt;
int PicEdgePix[1024];
int PicEdgeAngle[1024];

int PathPlanningCnt;
int NumPathPlanningCnt;
int PathPlanningAng[5][100][5];
int PathPlanningPix[5][100][5];
int PathPlanningDistance[10][5];

int RRTPlanningCnt;
int RRTPlanningAng[256];
int RRTPlanningPix[256];

float PathWidth;

int isOkPath;
int SafeFlag;
int RandRito;
int ROLLWINDOW_RITO;

int HaveDoubleObsPath;

int startflag;

int fd_rand;

int Next_UnSafe_Flag;

//int Layer;
int RRT_Ok_Flag;
int Arrived_ChildAim_Flag;
int isArrivedDirect_Flag;
int SrandNum[5];

int Lidar_Circle_Cnt;
int Lidar_PerCircle_Ok_Flag;

int Safe_Dis_Sub;
int Track_Flag;

int timer_cnt;

typedef struct Cordinate_PerLayer
{
	int Layer;
	int Cordinate_X[10][256];
	int Cordinate_Y[10][256];
	int Cordinate_Ang[10][256];
} T_CorPerLayer;

T_CorPerLayer Cord_PerLayer;

typedef struct Ok_PathPlanning_Point
{
	int Layer_Num;
	int evaluation[1024]; //评估函数的评估值
	int Ok_Angle[1024];
	int Ok_Pix[1024];
	int Ok_Distance[1024];
	int Ok_Cnt_PerLayer;
	int Ok_PerLayer_Angle[1024][4];
} T_OkPath;

typedef struct Best_OkPath_Point
{
	int Best_Layer_Num;
	int Best_Ok_Angle[10];
	int Best_Ok_Pix[10];
	int Best_Ok_Distance[10];
	int Best_Ok_PerLayer_Ang[10][4];
	int Best_Ok_Cnt;
	int showBest_Ok_PerLayer_Ang[10][4];
	int showBest_Ok_Cnt;
} T_Best_OkPath;
struct timer
{
	double PreMs;
	double NowMs;
	int start_flag;
	int time_flag;
} get_time, ekf_gettime, ekf_gettime2;

/* 将 客户端地址结构 和 服务器连接套接字描述符 绑定在一起，一一对应 */
typedef struct net_receive
{
	int iSocketClient; //服务器连接套接字描述符

	unsigned char *ip; //客户端地址结构IP
} t_net_rec, *pt_net_rec;
#endif /* _gloable_H */

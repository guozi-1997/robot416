

#ifndef _WAY_H
#define _WAY_H

#include <global.h>
#include <stdio.h>
#include <stdlib.h>

int show_y[1000];
int show_x[1000];
int showedge_x[1000];
int showedge_y[1000];

int FinalEdgePicCnt;
int FinalEdgePix[1024];
int FinalEdgeAngle[1024];

void min_cir(unsigned char *addr);
int rgb(unsigned char *addr, int y, int x);
void scan(unsigned char *addr);

//void srand(int n);
//-----------------link------------------------
struct info
{
	int data;
	int connected_flag;
	int start_connect_flag;
	int iSocketClient;
	char send_ip[14];
	unsigned int dis_ang[20];
	struct info *pNext;
};

typedef struct Reachable_Path_Pre
{
	int Reach_Angle[128];
	int Reach_Distance[128];
	int Reach_Cnt;
} T_Reach;

typedef struct Reachable_Path_Filter
{
	int ReachFilter_Angle[128];
	int ReachFilter_Distance[128];
	int ReachFilter_Cnt;
} T_ReachFilter;

typedef struct Lidar_Data_Cnt
{
	int Lidar_Data_Dis[2000];
	int Lidar_Data_Ang[2000];
	int Lidar_Data_Pix[2000];
	int Lidar_Data_Cnt;
} T_Lidar_Cnt;

typedef struct info node, *PNODE;
Point2D PointOnBezierOneControl(Point2D *cp, float t);
void ComputeOneBezier(Point2D *cp, int numberOfPoints, Point2D *curve);
Point2D PointOnCubicBezier(Point2D *cp, float t);
void ComputeDoubleBezier(Point2D *cp, int numberOfPoints, Point2D *curve);
PNODE addBack(PNODE phead, int data, char *ip, int fd); //尾部插入
PNODE addFront(PNODE phead, int data);					//头部插入
PNODE findFirst(PNODE phead, int data);					//检索数据
PNODE findIp(PNODE phead, char *ip);					//检索IP
PNODE deleteFirst(PNODE phead, int data);				//删除数据

PNODE deleteNode(PNODE phead, int data, PNODE *ptemp); //删除数据

PNODE insertNode(PNODE phead, int finddata, int data); //插入数据
int getNum(PNODE phead);							   //返回链表的个数
void showAll(PNODE phead);							   //显显示全部

#endif /* _WAY_H */

#ifndef HEADER_MYMATRIX_H_
#define HEADER_MYMATRIX_H_
#include "common.h"
/*
 * 问题记录：
 *　* 存在内存滥用的现象，因为每次创建矩阵都会申请一块新内存，并没有free掉
 *  * 慎用创建矩阵这个函数，在调用之后要想办法free掉
 *　* LU分解求逆不太熟
 */
#define visual_Dis_204 100
#define visual_Ang_204 60
#define pi 3.141592

typedef struct MNode *PtrToMNode;
typedef struct MNode
{
	int row;	   //行
	int column;	   //列
	double **data; //等价于 double *data[]、double data[][]
};

struct Robot_msg
{
	int angle;
	int distance;
};

struct My_msg
{
	char flag;					   //"n"/"y"
	char local_ip[15];			   // local_ip
	int Ture_Tht;				   // 陀螺仪偏转角度
	int dlta_d;					   // global.distanc,
	int dlta_a;					   // global.angl,
	int get_x;					   // robot_ekf[0].x,
	int get_y;					   // robot_ekf[0].y,
	int get_tht;				   // robot_ekf[0].angle,
	struct Robot_msg robot_msg[3]; // robot_send[1].distance、robot_send[1].angle、robot_send[2].distance、robot_send[2].angle
} mymsg_send, mymsg_recv, ekfRecvBuf[3];

typedef PtrToMNode Matrix;
// 释放申请的矩阵空间
void Free_Matrix(Matrix mat);
//　创建矩阵，并将所有元素初始化为０
Matrix Create_Matrix(int row, int column);
// 初始化矩阵(将所有元素初始化为０)
void Init_Matrix(Matrix mat);
// 给矩阵每个元素赋值
void SetData_Matrix(Matrix mat, double data[]);
//　打印矩阵
void Show_Matrix(Matrix mat, char *s);
//　矩阵加减法
Matrix AddorSub_Matrix(Matrix mat_1, Matrix mat_2, int flag);
//　矩阵转置
Matrix Trans_Matrix(Matrix mat);
// 矩阵乘法
Matrix Mult_Matrix(Matrix mat_1, Matrix mat_2);
//　创建单位矩阵
Matrix eye(int n);
//　取出矩阵某行某列的元素
double PickInMat(Matrix mat, int r, int c);
// 向量叉乘
Matrix Cross(Matrix a, Matrix b);

// 矩阵求逆，利用初等行变换求逆
Matrix EleTransInv_Matrix(Matrix mat);
// 矩阵第n行与第m行互换
void Swap_row(Matrix mat, int n, int m);
/*
 * 求矩阵行列式的值，通过行变换（列主元消去法）将矩阵变换成上三角矩阵
 * 注意行变换会改变行列式的符号
 */
double Det_Matrix(Matrix mat);
//　对一个矩阵进行复制
Matrix Copy_Matrix(Matrix mat);
//  产生一个以miu为均值，sigma为均方差的高斯分布的随机变量，结果需要使用上面的宏来判断是否为-inf
double gaussrand(double miu, double sigma);
//从start往后取出len个字符
char *substr(char src[], int start, int len);
//比较排序
//void sortLinker(int count);
void assignMent(struct My_msg src, int count, int a, char *local_ip);
//这个函数用于寻找虚拟机器人在本机局部坐标系下的距离和角度
void math_Dis_Ang_204(int a);
void InitEkf(int a, STACKS *S, ERROR_ID errorID);
//秒级定时器
void seconds_sleep(unsigned seconds);
//毫秒级别定时器
void milliseconds_sleep(unsigned long mSec);
//微妙级别定时器
void microseconds_sleep(unsigned long uSec);

#endif /* HEADER_MYMATRIX_H_ */

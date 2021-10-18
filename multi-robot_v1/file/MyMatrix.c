#include "MyMatrix.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "global.h"
#include <errno.h>
#include <sys/time.h>

void Init_Matrix(Matrix mat)
{
	int i, j;
	for (i = 0; i < mat->row; i++)
	{
		for (j = 0; j < mat->column; j++)
		{
			mat->data[i][j] = 0;
		}
	}
}

// 释放申请的矩阵空间
void Free_Matrix(Matrix mat)
{
	int i;
	for (i = 0; i < mat->row; i++)
		free(mat->data[i]); // 释放行指针
	free(mat->data);		// 释放头指针
	free(mat);				// 释放结构体指针
}

Matrix Create_Matrix(int row, int col)
{
	Matrix mat;
	mat = (Matrix)malloc(sizeof(struct MNode)); //　分配结构体指针
	if (row <= 0 || col <= 0)
	{
		printf("ERROR, in creat_Matrix the row or col <= 0\n");
		exit(1);
	}
	if (row > 0 && col > 0)
	{
		mat->row = row;
		mat->column = col;
		mat->data = (double **)malloc(row * sizeof(double *)); // 分配头指针
		if (mat->data == NULL)
		{
			printf("ERROR, in creat_Matrix the mat->data == NULL\n");
			exit(1);
		}
		int i;
		for (i = 0; i < row; i++)
		{
			*(mat->data + i) = (double *)malloc(col * sizeof(double)); //　分配每行的指针
			if (mat->data[i] == NULL)
			{
				printf("ERROR, in create_Matrix the mat->data[i] == NULL\n");
				exit(1);
			}
		}
		Init_Matrix(mat);
	}
	return mat;
}

void Show_Matrix(Matrix mat, char *s)
{
	int i, j;
	printf("%s\n", s);
	for (i = 0; i < mat->row; i++)
	{
		for (j = 0; j < mat->column; j++)
			printf("%.6f\t", mat->data[i][j]);
		printf("\n");
	}
	printf("\n");
}

void SetData_Matrix(Matrix mat, double data[])
{
	int i, j;
	for (i = 0; i < mat->row; i++)
	{
		for (j = 0; j < mat->column; j++)
		{
			mat->data[i][j] = data[i * mat->column + j];
		}
	}
}

//flag = 0, add; flag = 1, sub
Matrix AddorSub_Matrix(Matrix mat_1, Matrix mat_2, int flag)
{
	Matrix rst_mat;
	if (mat_1->column != mat_2->column)
	{
		printf("ERROR in AddorSub, column !=\n");
		exit(1);
	}
	if (mat_1->row != mat_2->row)
	{
		printf("ERROR in AddorSub, row !=\n");
		exit(1);
	}
	int i, j;
	rst_mat = Create_Matrix(mat_1->row, mat_1->column);
	for (i = 0; i < mat_1->row; i++)
	{
		for (j = 0; j < mat_1->column; j++)
			rst_mat->data[i][j] = mat_1->data[i][j] + pow(-1, flag) * mat_2->data[i][j];
	}
	return rst_mat;
}

//转置
Matrix Trans_Matrix(Matrix mat) //fuck you！！！！！！！
{
	Matrix mat_;
	int i, j;
	mat_ = Create_Matrix(mat->column, mat->row);
	for (i = 0; i < mat->column; i++)
	{
		for (j = 0; j < mat->row; j++)
			mat_->data[i][j] = mat->data[j][i];
	}

	return mat_;
}

Matrix Mult_Matrix(Matrix mat_1, Matrix mat_2)
{
	Matrix rst_mat;
	int i, j, m;
	if (mat_1->column != mat_2->row)
	{
		printf("ERROR in Mult_Matrix, column != row\n");
		exit(1);
	}
	else
	{
		rst_mat = Create_Matrix(mat_1->row, mat_2->column);
		for (i = 0; i < mat_1->row; i++)
		{
			for (j = 0; j < mat_2->column; j++)
			{
				for (m = 0; m < mat_1->column; m++)
					rst_mat->data[i][j] += mat_1->data[i][m] * mat_2->data[m][j];
			}
		}
	}
	return rst_mat;
}

Matrix eye(int n) //单位矩阵
{
	Matrix E;
	int i, j;
	if (n <= 0)
	{
		printf("ERROR in eye\n");
		exit(1);
	}
	E = Create_Matrix(n, n);
	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++)
		{
			if (i == j)
				E->data[i][j] = 1;
			else
				E->data[i][j] = 0;
		}
	}
	return E;
}

double PickInMat(Matrix mat, int r, int c)
{
	double rst;
	rst = mat->data[r - 1][c - 1];
	return rst;
}

// 3×3向量叉乘
Matrix Cross(Matrix a, Matrix b)
{
	Matrix C;
	C = Create_Matrix(a->row, a->column);
	double ax, ay, az;
	double bx, by, bz;
	double c[a->row];
	ax = PickInMat(a, 1, a->column);
	ay = PickInMat(a, 2, a->column);
	az = PickInMat(a, 3, a->column);
	bx = PickInMat(b, 1, b->column);
	by = PickInMat(b, 2, b->column);
	bz = PickInMat(b, 3, b->column);
	c[0] = ay * bz - az * by;
	c[1] = az * bx - ax * bz;
	c[2] = ax * by - ay * bx;
	SetData_Matrix(C, c);
	return C;
}

// 矩阵第n行与第m行互换
void Swap_row(Matrix mat, int n, int m)
{
	int i;
	double temp;
	for (i = 0; i < mat->column; i++)
	{
		temp = mat->data[n - 1][i];
		mat->data[n - 1][i] = mat->data[m - 1][i];
		mat->data[m - 1][i] = temp;
	}
}

// 矩阵求逆，利用初等行变换求逆
Matrix EleTransInv_Matrix(Matrix mat)
{
	if (mat->column != mat->row)
	{
		printf("ERROR in inverse, the row != the column\n");
		exit(1);
	}
	if (Det_Matrix(mat) == 0)
	{
		printf("The Matrix is not invertible\n");
		exit(1);
	}
	int n = mat->row;
	// 为防止改变原矩阵，此处进行复制处理
	Matrix mat_ = Copy_Matrix(mat);
	//　创建单位矩阵
	Matrix inv_eye = eye(n);
	double e, a_max;
	int i, j, k, t, cnt;
	for (k = 0; k < n - 1; k++)
	{
		a_max = fabs(mat_->data[k][k]);
		cnt = k;
		// 选主元
		for (i = k; i < n; i++)
		{
			if (fabs(mat_->data[i][k]) > a_max)
			{
				a_max = fabs(mat_->data[i][k]);
				cnt = i;
			}
		}
		//　换行，原矩阵换行的同时，单位矩阵也换行
		double temp, temp_e;
		if (cnt != k)
		{
			for (j = 0; j < n; j++)
			{
				temp = mat_->data[k][j];
				mat_->data[k][j] = mat_->data[cnt][j];
				mat_->data[cnt][j] = temp;
				temp_e = inv_eye->data[k][j];
				inv_eye->data[k][j] = inv_eye->data[cnt][j];
				inv_eye->data[cnt][j] = temp_e;
			}
		}
		// 消元
		for (i = k + 1; i < n; i++)
		{
			e = mat_->data[i][k] / mat_->data[k][k];
			for (j = 0; j < n; j++)
			{
				mat_->data[i][j] = mat_->data[i][j] - e * mat_->data[k][j];
				inv_eye->data[i][j] = inv_eye->data[i][j] - e * inv_eye->data[k][j];
			}
		}
	}
	// 将矩阵每行的行首元素化为１
	for (i = 0; i < n; i++)
	{
		e = 1.0 / mat_->data[i][i];
		for (j = 0; j < n; j++)
		{
			mat_->data[i][j] = mat_->data[i][j] * e;
			inv_eye->data[i][j] = inv_eye->data[i][j] * e;
		}
	}
	// 从最后一排开始消元，把增广矩阵左边的矩阵化为单位矩阵
	for (i = n - 1; i > 0; i--)
	{
		for (j = i - 1; j >= 0; j--)
		{
			e = mat_->data[j][i] / mat_->data[i][i];
			for (t = 0; t < n; t++)
			{
				mat_->data[j][t] = mat_->data[j][t] - e * mat_->data[i][t];
				inv_eye->data[j][t] = inv_eye->data[j][t] - e * inv_eye->data[i][t];
			}
		}
	}
	Free_Matrix(mat_);
	return inv_eye;
}
/*
 * 通过行变换（列主元消去法）将矩阵变换成上三角矩阵
 * 注意行变换会改变行列式的符号
 */
double Det_Matrix(Matrix mat)
{
	int i, j, k;
	double det = 1.0;
	if (mat->row != mat->column)
	{
		printf("ERROR in Det_Matrix, the row != the column\n");
		exit(1);
	}
	if (mat->row == 1 && mat->column == 1)
		return mat->data[0][0];
	// 为防止改变原矩阵，此处进行复制处理
	Matrix mat_ = Copy_Matrix(mat);
	int n = mat_->row, s = 0, cnt;
	double L, a_max;
	for (k = 0; k < n - 1; k++)
	{
		cnt = k;
		a_max = fabs(mat_->data[k][k]);
		for (i = k; i < n; i++)
		{
			if (fabs(mat_->data[i][k]) > a_max)
			{
				a_max = fabs(mat_->data[i][k]);
				cnt = i; // 确定下标i
			}
		}
		//　换行
		double temp;
		if (cnt != k)
		{
			s++; // 换行次数
			for (j = 0; j < n; j++)
			{
				temp = mat_->data[cnt][j];
				mat_->data[cnt][j] = mat_->data[k][j];
				mat_->data[k][j] = temp;
			}
		}
		// 消元计算
		for (i = k + 1; i < n; i++)
		{
			L = mat_->data[i][k] / mat_->data[k][k];
			for (j = k + 1; j < n; j++)
				mat_->data[i][j] = mat_->data[i][j] - L * mat_->data[k][j];
		}
	}
	if (s % 2 == 0)
		s = 1;
	else
		s = -1;
	for (i = 0; i < n; i++)
		det *= mat_->data[i][i];
	det *= s;
	Free_Matrix(mat_); //释放掉复制矩阵的内存
	return det;
}

//　伴随矩阵
// Matrix Adj_Matrix(Matrix mat)
// {
// 	Matrix adj;

// 	return adj;
// }

//　对一个矩阵进行复制
Matrix Copy_Matrix(Matrix mat)
{
	int i, j;
	Matrix copy_mat = Create_Matrix(mat->row, mat->column);
	for (i = 0; i < mat->row; i++)
	{
		for (j = 0; j < mat->column; j++)
			copy_mat->data[i][j] = mat->data[i][j];
	}
	return copy_mat;
}
#define ISINF(a) ((a) <= DBL_MAX && (a) >= -DBL_MAX)

//产生一个以miu为均值，sigma为均方差的高斯分布的随机变量，结果需要使用上面的宏来判断是否为-inf
double gaussrand(double miu, double sigma)
{
	double U1, U2;
	double Z, X;

	//产生两个服从U(0,1)分布的随机变量
	U1 = (double)rand() / RAND_MAX;
	U2 = (double)rand() / RAND_MAX;

	//这里的log是自然对数
	Z = sqrt(-2 * log(U2)) * cos(2 * 3.1415926 * U1);

	X = miu + sigma * Z;

	return X;
}
char *substr(char src[], int start, int len)
{
	int i;
	static char dst[10];
	for (i = 0; i < len; i++)
	{
		dst[i] = src[start + i]; //从第start+i个元素开始向数组内赋值
	}
	dst[i] = '\0';
	return dst;
}
//本函数作用：找出 视觉系统的目标物 和 共享网络中信息 的对应关系
//对不起我实在编不出三个以上机器人的识别了,这个凑合着用吧
/* 理论上来说是可以简化的 */
void assignMent(struct My_msg src, int count, int a, char *local_ip)
{
	int i, j, k;
	for (k = 1; k < count; k++)
	{

		ekf_send[a].distance[k] = src.robot_msg[k].distance;
		ekf_send[a].angle[k] = src.robot_msg[k].angle;
	}

	for (i = 1; i < count; i++)
	{
		//-(double)Ture_Tht / 100
		for (j = 1; j < count; j++)
		{
			robot_rece[a].number = abs(robot_ekf[i].angle / 10 - Ture_Tht / 100 - ekf_send[a].angle[j] / 10 + src.Ture_Tht / 100);
			if (robot_rece[a].number >= 360)
			{
				robot_rece[a].number = robot_rece[a].number - 360;
			}
			//printf("robot_rece[a].number = %d\n", robot_rece[a].number);
			printf("角度=%d 观测角=%d 电子罗盘1=%d 传回角=%d 电子罗盘2=%d \n", robot_rece[a].number, robot_ekf[i].angle / 10, -Ture_Tht / 100, -ekf_send[a].angle[j] / 10, src.Ture_Tht / 100);
			if (abs(robot_rece[a].number - 180) <= 20)
			{
				printf("距离差=%d 观测距离=%d 传回距离=%d\n", abs(robot_ekf[i].distance - ekf_send[a].distance[j]), robot_ekf[i].distance, ekf_send[a].distance[j]);
				if (abs(robot_ekf[i].distance - ekf_send[a].distance[j]) <= 40)
				{
					//printf("角度=%d 观测角=%d 电子罗盘1=%d 传回角=%d 电子罗盘2=%d \n", robot_rece[a].number, robot_ekf[i].angle / 10, -Ture_Tht / 100, -ekf_send[a].angle[j] / 10, atoi(substr(src, 14, 6)) / 100);
					//printf("距离差=%d 观测距离=%d 传回距离=%d\n", abs(robot_ekf[i].distance - ekf_send[a].distance[j]), robot_ekf[i].distance, ekf_send[a].distance[j]);
					printf("success\n");
					ekf[a].tht = 180. - (360. - ekf_send[a].angle[j] * 0.1 + robot_ekf[i].angle * 0.1);
					if (ekf[a].tht > 180)
					{
						ekf[a].tht = 360. - ekf[a].tht;
					}
					if (ekf[a].tht < -180.)
					{
						ekf[a].tht = 360. + ekf[a].tht;
					}
					//printf("ekf[a].tht = %lf\n", ekf[a].tht);
					if (atoi(substr(local_ip, 12, 1)) == 1)
					{
						switch (atoi(substr(src.local_ip, 12, 1)))
						{
						case 4:
							robot_ekf2[1].number = atoi(substr(src.local_ip, 12, 1));
							robot_ekf2[1].angle = robot_ekf[i].angle;
							robot_ekf2[1].distance = robot_ekf[i].distance;
							robot_ekf2[1].dlta_d = src.dlta_d;
							robot_ekf2[1].dlta_a = src.dlta_a;
							robot_ekf2[1].Ture_Tht = ekf[a].tht;
							robot_send[1].angle = src.get_tht;
							robot_send[1].x = src.get_x;
							robot_send[1].y = src.get_y;
							robot_end[1].number = 1;
							break;
						case 6:
							robot_ekf2[2].number = atoi(substr(src.local_ip, 12, 1));
							robot_ekf2[2].angle = robot_ekf[i].angle;
							robot_ekf2[2].distance = robot_ekf[i].distance;
							robot_ekf2[2].dlta_d = src.dlta_d;
							robot_ekf2[2].dlta_a = src.dlta_a;
							robot_ekf2[2].Ture_Tht = ekf[a].tht;
							robot_send[2].angle = src.get_tht;
							robot_send[2].x = src.get_x;
							robot_send[2].y = src.get_y;
							robot_end[2].number = 1;
							break;
						default:
							break;
						}
					}
					if (atoi(substr(local_ip, 12, 1)) == 4)
					{
						switch (atoi(substr(src.local_ip, 12, 1)))
						{

						case 1:
							robot_ekf2[1].number = atoi(substr(src.local_ip, 12, 1));
							robot_ekf2[1].angle = robot_ekf[i].angle;
							robot_ekf2[1].distance = robot_ekf[i].distance;
							robot_ekf2[1].dlta_d = src.dlta_d;
							robot_ekf2[1].dlta_a = src.dlta_a;
							robot_ekf2[1].Ture_Tht = ekf[a].tht;
							robot_send[1].angle = src.get_tht;
							robot_send[1].x = src.get_x;
							robot_send[1].y = src.get_y;
							robot_end[1].number = 1;
							break;
						case 6:
							robot_ekf2[2].number = atoi(substr(src.local_ip, 12, 1));
							robot_ekf2[2].angle = robot_ekf[i].angle;
							robot_ekf2[2].distance = robot_ekf[i].distance;
							robot_ekf2[2].dlta_d = src.dlta_d;
							robot_ekf2[2].dlta_a = src.dlta_a;
							robot_ekf2[2].Ture_Tht = ekf[a].tht;
							robot_send[2].angle = src.get_tht;
							robot_send[2].x = src.get_x;
							robot_send[2].y = src.get_y;
							robot_end[2].number = 1;
							break;
						default:
							break;
						}
					}
					if (atoi(substr(local_ip, 12, 1)) == 6)
					{
						switch (atoi(substr(src.local_ip, 12, 1)))
						{
						case 1:
							robot_ekf2[1].number = atoi(substr(src.local_ip, 12, 1));
							robot_ekf2[1].angle = robot_ekf[i].angle;
							robot_ekf2[1].distance = robot_ekf[i].distance;
							robot_ekf2[1].dlta_d = src.dlta_d;
							robot_ekf2[1].dlta_a = src.dlta_a;
							robot_ekf2[1].Ture_Tht = ekf[a].tht;
							robot_send[1].angle = src.get_tht;
							robot_send[1].x = src.get_x;
							robot_send[1].y = src.get_y;
							robot_end[1].number = 1;
							break;
						case 4:
							robot_ekf2[2].number = atoi(substr(src.local_ip, 12, 1));
							robot_ekf2[2].angle = robot_ekf[i].angle;
							robot_ekf2[2].distance = robot_ekf[i].distance;
							robot_ekf2[2].dlta_d = src.dlta_d;
							robot_ekf2[2].dlta_a = src.dlta_a;
							robot_ekf2[2].Ture_Tht = ekf[a].tht;
							robot_send[2].angle = src.get_tht;
							robot_send[2].x = src.get_x;
							robot_send[2].y = src.get_y;
							robot_end[2].number = 1;
							break;
						default:
							break;
						}
					}
				}
			}
		}
	}
}

void math_Dis_Ang_204(int a)
{
	//向左转激光雷达角度会增加
	if (a == 4)
	{
		float dis = 0;
		float tht = 0;
		int to_Visual_angle;
		/* ******to_Visual_angle：本机到leader再到目标点形成的角度************ */
		to_Visual_angle = abs(90 * 10 - visual_Ang_204 * 10 - robot_ekf2[1].angle - robot_ekf2[1].Ture_Tht * 10);
		dis = pow(visual_Dis_204, 2) + pow(robot_ekf2[1].distance, 2) -
			  2 * visual_Dis_204 * robot_ekf2[1].distance * cos(to_Visual_angle * pi / 1800);
		dis = sqrt(dis);
		tht = (pow(dis, 2) + pow(visual_Dis_204, 2) - visual_Dis_204 * visual_Dis_204) /
			  (2 * dis * robot_ekf2[1].distance);
		if (tht > 1)
			tht = 1;
		else if (tht < -1)
			tht = -1;
		tht = acos(tht) * 180 / pi;
		/**************本机观测leader所得到的角度********************/
		//printf("robot_ekf2[1].angle / 10 = %d\n", robot_ekf2[1].angle / 10);
		/*********leader到本机再到目标点所形成的夹角******************/
		//printf("tht =%f\n", tht);
		/******本机与leader之间的角度差，本机左转时会增大，leader左转时会减小********* */
		//printf("robot_ekf2[1].Ture_Tht = %d\n", robot_ekf2[1].Ture_Tht);
		tht = abs(90 - robot_ekf2[1].angle / 10) + tht - robot_ekf2[1].Ture_Tht;
		printf("dis = %f,tht = %f\n", dis, tht);
	}
}

void InitEkf(int a)
{
	/*初始化，赋予初值*/
	ekf[a].A = Create_Matrix(2, 2);
	ekf[a].H = Create_Matrix(2, 2);
	ekf[a].X = Create_Matrix(2, 1);
	ekf[a].X_ = Create_Matrix(2, 1);
	ekf[a].P = Create_Matrix(2, 2);
	ekf[a].P_ = Create_Matrix(2, 2);
	ekf[a].K = Create_Matrix(2, 2);
	ekf[a].K_ = Create_Matrix(2, 2);
	ekf[a].K__ = Create_Matrix(2, 2);
	ekf[a].Z = Create_Matrix(2, 1);
	ekf[a].Q = Create_Matrix(2, 2);
	ekf[a].R = Create_Matrix(2, 2);
	ekf[a].P = eye(2);
	ekf[a].Avalue[0] = 1;
	ekf[a].Avalue[1] = 0;
	ekf[a].Avalue[2] = 0;
	ekf[a].Avalue[3] = 1;
	ekf[a].Hvalue[0] = 1;
	ekf[a].Hvalue[1] = 0;
	ekf[a].Hvalue[2] = 0;
	ekf[a].Hvalue[3] = 1;
	ekf[a].Rvalue[0] = 0.1;
	ekf[a].Rvalue[3] = 0.1;
	ekf[a].Qvalue[0] = 0.1;
	ekf[a].Qvalue[3] = 0.1;
	SetData_Matrix(ekf[a].A, ekf[a].Avalue);
	SetData_Matrix(ekf[a].H, ekf[a].Hvalue);
	SetData_Matrix(ekf[a].X, ekf[a].Xvalue);
	SetData_Matrix(ekf[a].R, ekf[a].Rvalue);
	SetData_Matrix(ekf[a].Q, ekf[a].Qvalue);
	printf("InitEkf ready\n");
}

//秒级定时器
void seconds_sleep(unsigned seconds)
{
	struct timeval tv;
	tv.tv_sec = seconds;
	tv.tv_usec = 0;
	int err;
	do
	{
		err = select(0, NULL, NULL, NULL, &tv);
	} while (err < 0 && errno == EINTR);
}
//毫秒级别定时器
void milliseconds_sleep(unsigned long mSec)
{
	struct timeval tv;
	tv.tv_sec = mSec / 1000;
	tv.tv_usec = (mSec % 1000) * 1000;
	int err;
	do
	{
		err = select(0, NULL, NULL, NULL, &tv);
	} while (err < 0 && errno == EINTR);
}
//微妙级别定时器
void microseconds_sleep(unsigned long uSec)
{
	struct timeval tv;
	tv.tv_sec = uSec / 1000000;
	tv.tv_usec = uSec % 1000000;
	int err;
	do
	{
		err = select(0, NULL, NULL, NULL, &tv);
	} while (err < 0 && errno == EINTR);
}
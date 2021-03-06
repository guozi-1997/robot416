#include <way.h>
#include <show.h>
#include "io.h"
#include <myfile.h>
#include "MyMatrix.h"
#include "matrix.h"

int ekf_ts = 1000000;	 //你可以通过ekf_ts决定滤波频率
int ControlFun_ts = 200; //毫秒，你可以通过ControlFun_ts确定控制频率
struct _globals
{
	int order_flag; //令牌
	int start_flag;
	pthread_mutex_t db;		  //配合条件变量的互斥锁
	pthread_cond_t db_update; //条件变量，用于操作robot_temp[]、robot_end[]、...
	unsigned char *buf;
	int size;
	int distanc;
	int angl;
	int outcnt;
	int Ture_Tht;
} global;

typedef union
{
	float fdata;
	unsigned char ldata[4];
} FloatLongType;

char mymsg_send_[100];
char mymsg_recv_[100];

//------lidar global var-----------------

int ctrl_c_pressed = 0;
int count = 0;
int READ_FLAG = 0;
int ReadByte = 0; //接收 read 函数返回值，即读到的字节数
int pernum = 0;
int ReadHead = 0; //是否已接收到起始应答报文标志位（0未收到，1已接收到）
int TotalReadByte = 0;
int LidarStopFlag = 0;
static char buf[BUFFER_SIZE] = {0}; //一个采样点 数据应答报文

//------lidar global var------------------
//--------------定义环形链表操作的锁------------------
pthread_mutex_t db_link;
pthread_mutex_t ekf_lock;
pthread_cond_t ekf_lock1;
//pthread_cond_t  db_update;

//-----------------------------------
//struct timeval tv_glo;
//struct timeval tv_glo_pre;

PNODE phead = NULL;
PNODE phead_rec = NULL;

char all_ip[7][14] = {"192.168.0.201",
					  "192.168.0.202",
					  "192.168.0.203",
					  "192.168.0.204",
					  "192.168.0.205",
					  "192.168.0.206",
					  "192.168.0.207"};
//pthread_mutex_t g_tLidar = PTHREAD_MUTEX_INITIALIZER; //此互斥锁用于雷达，现在已废弃（被 rwlock_lidar 代替）
pthread_mutex_t g_tMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t g_tConVar = PTHREAD_COND_INITIALIZER;

static pthread_rwlock_t rwlock; //此读写锁用于变量dlta_d和dlta_a

static pthread_rwlock_t rwlock_lidar; //此读写锁用于雷达
T_VideoBuf tConvertBuf_now;
T_VideoBuf tConvertBuf_mid;

/* ******无线通信相关线程***** */
void *socket_send(void *povid);
void *socket_receive(void *povid);
/****************************/

/* 卡尔曼滤波相关线程 */
void *ekf_algorithm(void *arg);
void *ekf_multi(void *arg);
void *ekf_algorithm2(void *arg);
void *ekf_algorithm(void *arg);
/*************************/

/*Lidar数据处理相关线程*/
void *Thread_Read(void *arg);
void *Thread_Handle(void *arg);
/*****************************/

//----------------------
void *VideoTreadFunction(void *povid)
{
	chose_xy();
	int iError;
	T_VideoDevice tVideoDevice; //创建一个video设备
	PT_VideoConvert ptVideoConvert;
	int iPixelFormatOfVideo;
	int iPixelFormatOfDisp;

	PT_VideoBuf ptVideoBufCur; //存 tConvertBuf 的地址
	PT_VideoBuf ptVideoBufCur_old;
	T_VideoBuf tVideoBuf;	//从内核缓冲区中读取图像
	T_VideoBuf tConvertBuf; //转换格式后的图像
	T_VideoBuf tZoomBuf;
	T_VideoBuf tFrameBuf; //

	int iLcdWidth;
	int iLcdHeigt;
	int iLcdBpp;

	struct timeval tv;

	//-------------------------------------
	//---------------------显示屏初始化------------------------------------
	DisplayInit();
	SelectAndInitDefaultDispDev("fb");
	GetDispResolution(&iLcdWidth, &iLcdHeigt, &iLcdBpp);

	GetVideoBufForDisplay(&tFrameBuf);			 //tFrameBuf 传出参数
	iPixelFormatOfDisp = tFrameBuf.iPixelFormat; //确定显示屏的像素格式：RGB565（即确定像素深度）
	//----------------------摄像头初始化--------------------------------
	VideoInit();														//注册一个v4l2设备实例，设备信息存到全局变量 g_ptVideoOprHead（指向结构体 g_tV4l2VideoOpr）中
	iError = VideoDeviceInit("/dev/video4", &tVideoDevice);				//即执行函数 V4l2InitDevice（"/dev/video4", &tVideoDevice），其中tVideoDevice是传出参数
	iPixelFormatOfVideo = tVideoDevice.ptOPr->GetFormat(&tVideoDevice); //即执行函数 V4l2GetFormat（&tVideoDevice）
	//iPixelFormatOfVideo 返回为宏值 V4L2_PIX_FMT_MJPEG

	VideoConvertInit(); //Mjpeg2RgbInit()，将转换时要使用的函数信息存到全局变量 g_ptVideoConvertHead（指向结构体 g_tMjpeg2RgbConvert）中
	ptVideoConvert = GetVideoConvertForFormats(iPixelFormatOfVideo, iPixelFormatOfDisp);
	//ptVideoConvert 返回为全局变量结构体 g_tMjpeg2RgbConvert 的地址
	if (NULL == ptVideoConvert)
	{
		DBG_PRINTF("can not support this format convert\n");
	}

	//启动摄像头
	iError = tVideoDevice.ptOPr->StartDevice(&tVideoDevice); //即执行函数 V4l2StartDevice(&tVideoDevice)
	if (iError)
	{
	}

	memset(&tVideoBuf, 0, sizeof(tVideoBuf));
	memset(&tConvertBuf, 0, sizeof(tConvertBuf));
	tConvertBuf.iPixelFormat = iPixelFormatOfDisp; //RGB565
	tConvertBuf.tPixelDatas.iBpp = iLcdBpp;		   //深度为16
	memset(&tZoomBuf, 0, sizeof(tZoomBuf));

	//------------------------主循环部分-----------------------
	while (1)
	{
		// gettimeofday(&tv,NULL);
		// DBG_PRINTF("time1 is %f \n", tv.tv_sec+tv.tv_usec/1000000.);

		//-----------从（内核）缓冲区中得到（一帧）图像，存到 tVideoBuf 中---------------------
		iError = tVideoDevice.ptOPr->GetFrame(&tVideoDevice, &tVideoBuf); //即执行函数 V4l2GetFrameForStreaming(&tVideoDevice, &tVideoBuf)，将缓冲区从队列中取出来

		//ptVideoBufCur = &tVideoBuf;

		//--------------采集图像格式与显示格式不同，要作转换，将转换后的（一帧）图像存到 tConvertBuf 中----------------

		iError = ptVideoConvert->Convert(&tVideoBuf, &tConvertBuf); //即执行函数 Mjpeg2RgbConvert(&tVideoBuf, &tConvertBuf)
		ptVideoBufCur = &tConvertBuf;

		tVideoDevice.ptOPr->PutFrame(&tVideoDevice, &tVideoBuf); //即执行函数 V4l2PutFrameForStreaming(&tVideoDevice, &tVideoBuf)，将刚刚处理完的缓冲重新放入队列尾，这样可以循环采集
		//------------------图像处理部分-------------------------------
		// / gettimeofday(&tv,NULL);
		//		DBG_PRINTF("time2 is %f \n", tv.tv_sec+tv.tv_usec/1000000.);

		//根据此RGB565全景图像，（遍历像素点）寻找特征像素点（黑黄黑），结合 K-means 聚类算法，观测出其他目标机器人的相对距离和相对角度
		scan(tConvertBuf.tPixelDatas.aucPixelDatas); //消耗51毫秒
													 //find_dir(tConvertBuf.tPixelDatas.aucPixelDatas);
													 //gettimeofday(&tv,NULL);
													 // DBG_PRINTF("time3 is %f \n", tv.tv_sec+tv.tv_usec/1000000.);

		pthread_mutex_lock(&g_tMutex);

		//-----------------------------------------------
		//tFrameBuf_mid=tFrameBuf; //
		// tConvertBuf_mid=tConvertBuf;

		tConvertBuf_mid.tPixelDatas.iWidth = tConvertBuf.tPixelDatas.iWidth;
		tConvertBuf_mid.tPixelDatas.iHeight = tConvertBuf.tPixelDatas.iHeight;
		tConvertBuf_mid.tPixelDatas.iBpp = tConvertBuf.tPixelDatas.iBpp;
		tConvertBuf_mid.tPixelDatas.iLineBytes = tConvertBuf.tPixelDatas.iLineBytes;
		tConvertBuf_mid.tPixelDatas.iTotalBytes = tConvertBuf.tPixelDatas.iTotalBytes;
		if (!tConvertBuf_mid.tPixelDatas.aucPixelDatas) //仅第一次循环进此if语句
		{
			tConvertBuf_mid.tPixelDatas.aucPixelDatas = malloc(tConvertBuf.tPixelDatas.iTotalBytes);
		}
		//----------拷贝函数耗时2毫秒-----------
		memcpy(tConvertBuf_mid.tPixelDatas.aucPixelDatas, tConvertBuf.tPixelDatas.aucPixelDatas, 800 * 600 * 4); //拷贝两帧图像数据 2*800*600*2

		//-----------------------------------------------
		/* 解锁 */
		pthread_mutex_unlock(&g_tMutex);
		/*  唤醒主线程 */
		pthread_cond_signal(&g_tConVar);
	}
}

//=================================================
//===
//===
//==========================================
int can_fd;
int file_fd;
void ControlFun(void)
{
	//---path following values---采用标准单位-------//
	int get_x;
	int get_y;
	int get_tht;
	//上述三个变量与全局变量分开
	static float w = 0.0; //路径参数
	float w_der = 0.0;	  //路径参数导数
	float error_s = 0.0;  //前向跟踪误差
	float error_e = 0.0;  //横向跟踪误差
	static float vc = 0.1;
	float w_omiga = 0.0;

	float x0 = 0, y0 = 0;
	float x1 = 0, y1 = 0;
	float x2 = 0, y2 = 0;
	float x3 = 0, y3 = 0;

	float x_p = 0.0, y_p = 0.0;				  //期望路径
	float x_p_der = 0.0, y_p_der = 0.0;		  //期望路径一阶导
	float x_p_derder = 0.0, y_p_derder = 0.0; //期望路径二阶导

	float thta_p = 0.0; //thta_p PF2IF的旋转角度
	float thta_p_der = 0.0;

	float thta_e = 0.0;

	float x_measure = 0.0, y_measure = 0.0; //机器人测量状态，由航位推算转换而来
	float thta_measure = 0.0;

	float ke = 0.3;			  //bezier ke=2;
	float ks = 1.8, kw = 0.1; //参数选取满足1.44kw>=vc*ke
	static float path_radiu = 0.5;

	int value_flag = 0;
	int path_follow_flag = 0;
	int error_thta = 0;
	float curve_radio = 0.0;
	float a_rad_max = 0.8;
	static int bezier_cnt = 0;
	//----路径跟踪测试------//
	//------将脉冲值转换为角速度、线速度标准值-----//
	//signal(SIGALRM, ControlFun);
	//这两个值一直都是0

	while (1)
	{
		milliseconds_sleep(ControlFun_ts); //这个定时器使用信号量做的，不需要考虑线程抢占问题
		//printf("%d\n", RRT_Ok_Flag);
		/************如果RRT路径规划好了之后，就赋值三个点****************/
		if (isArrivedDirect_Flag)
		{
			printf("%d\n", __LINE__);
			path_follow_flag = 1;
			value_flag = 1;
			isArrivedDirect_Flag = 0;
		}
		else
			value_flag = 0;
		if (w >= 1.0)
		{
			w = 0.;
			value_flag = 0;

			frame_send.data[5] = 1; // 1底层坐标和角度清零
			nbytes = write(can_fd, &frame_send, sizeof(frame_send));

			path_follow_flag = 0;
			dlta_a = 0;
			dlta_d = 0;
			Arrived_ChildAim_Flag = 1;
		}
		if (value_flag)
		{
			/**********注意这里X代表横向，Y代表纵向*********************/
			x1 = Cord_PerLayer.Cordinate_X[0][0] / 1000;
			y1 = Cord_PerLayer.Cordinate_Y[0][0] / 1000;
			x2 = Cord_PerLayer.Cordinate_X[1][0] / 1000;
			y2 = Cord_PerLayer.Cordinate_Y[1][0] / 1000;
			x3 = Cord_PerLayer.Cordinate_X[2][0] / 1000;
			y3 = Cord_PerLayer.Cordinate_Y[2][0] / 1000;
			//path_follow_flag = 1;
			printf("x1 = %f,y1 = %f\n", x1, y1);
			printf("x1 = %f,y1 = %f\n", x2, y2);
			printf("x1 = %f,y1 = %f\n", x3, y3);
			value_flag = 0;
			w = 0.; //在旧的路径走完之前，w只会执行一次等于0.
		}

		//printf("%d\n", __LINE__);
		timer_cnt++;

		//if(w<1.0){
		/********向底部发出指令********/
		pthread_rwlock_rdlock(&rwlock);
		frame_send.data[1] = (int)dlta_d >> 8;
		frame_send.data[2] = (int)dlta_d;
		frame_send.data[3] = (int)dlta_a >> 8;
		frame_send.data[4] = (int)dlta_a;
		frame_send.data[5] = 0;
		nbytes = write(can_fd, &frame_send, sizeof(frame_send)); //通过CAN控制stm32电机，令机器人移动
		pthread_rwlock_unlock(&rwlock);							 //解锁，如果其他的线程需要对速度进行改变，则需要锁

		//******************************************//

		addr_len = sizeof(addr);
		len = 0;

		nbytes = recvfrom(s, &frame_rev, sizeof(struct can_frame), 0, (struct sockaddr *)&addr, &len);

		ifr.ifr_ifindex = addr.can_ifindex;
		ioctl(s, SIOCGIFNAME, &ifr);

		get_y = (-1) * (short)(frame_rev.data[0] * 256 + frame_rev.data[1]); //代表y偏移量前进为-
		get_x = (-1) * (short)(frame_rev.data[2] * 256 + frame_rev.data[3]); //代表x偏移量,左为正
		get_tht = (short)(frame_rev.data[4] * 256 + frame_rev.data[5]);		 //右转为正
																			 //}
		//======将航位推算结果转换成测量值======//
		x_measure = -1.0 * (double)get_x / 1000.0;
		y_measure = -1.0 * (double)get_y / 1000.0; //get_x、get_y单位mm/s，get_thta单位0.1°/s
		thta_measure = (double)get_tht * 3.14159 / 1800.0;
		//============================================//
		/* 	if (w > 1.0)
	{
		w = w - 1.0;
		bezier_cnt = 1;
	} */
		if (path_follow_flag)
		{

			if (timer_cnt < 251 && w < 1.0)
			//	if(timer_cnt<201 )
			{

				if (thta_measure < (-0.001))
				{
					thta_measure = -thta_measure;
				}
				else if (thta_measure > 0.001)
				{
					thta_measure = -thta_measure + 2 * pi;
				}
				else
				{
					thta_measure = thta_measure;
				}
				thta_measure = thta_measure + 1 * pi / 2;
				//***************以下算法实现部分**********************************//
				//=====步骤1 :设定vc,计算期望路径一阶导，二阶导==//
				//vc=0.1;				//v=vc=200mm/s

				//**********圆轨迹**********************//
				/*
		x_p = path_radiu*cos(w) - path_radiu;
		y_p = path_radiu*sin(w);

		x_p_der = -path_radiu*sin(w);
		y_p_der = path_radiu*cos(w);
		
		x_p_derder = -path_radiu*cos(w);
		y_p_derder = -path_radiu*sin(w);
		*/

				//**********圆轨迹**********************//

				//**********直线轨迹y = x  第一象限**********************//
				/* 就是来跑直线的 */
				/* ***********一次贝塞尔曲线*************************** */
				/* 		x_p = (1 - w) * x0 + w * x1;
		y_p = (1 - w) * y0 + w * y1;

		x_p_der = -x0 + x1;
		y_p_der = -y0 + y1;

		x_p_derder = 0;
		y_p_derder = 0; */
				//********** 优化三阶贝塞尔曲线轨迹**********************//
				x_p = pow((1 - w), 3) * x0 + 3 * pow((1 - w), 2) * w * x1 +
					  3 * (1 - w) * w * w * x2 + pow(w, 3) * x3;
				y_p = pow((1 - w), 3) * y0 + 3 * pow((1 - w), 2) * w * y1 +
					  3 * (1 - w) * w * w * y2 + pow(w, 3) * y3;
				x_p_der = 3 * pow((1 - w), 2) * x0 + 3 * (1 - w) * (1 - 3 * w) * x1 +
						  3 * (2 - 3 * w) * w * x2 + 3 * pow(w, 2) * x3;
				y_p_der = 3 * pow((1 - w), 2) * y0 + 3 * (1 - w) * (1 - 3 * w) * y1 +
						  3 * (2 - 3 * w) * w * y2 + 3 * pow(w, 2) * y3;
				x_p_derder = 6 * (1 - w) * x0 - 6 * (2 - 3 * w) * x1 +
							 6 * (1 - 3 * w) * x2 + 6 * w * x3;
				y_p_derder = 6 * (1 - w) * y0 - 6 * (2 - 3 * w) * y1 +
							 6 * (1 - 3 * w) * y2 + 6 * w * y3;

				//********计算不同路径参数对应的曲率、规划速度========//
				curve_radio = fabs((x_p_der * y_p_derder - y_p_der * x_p_derder) / (pow(x_p_der * x_p_der + y_p_der * y_p_der, 1.5)));
				//vc=pow(a_rad_max/curve_radio,0.5);
				vc = 0.1;
				//printf("vc=%f\n",vc);
				//file_write_xy(file_fd,timer_cnt , (int)(vc*1000));
				if (vc > 0.4)
					vc = 0.4;

				//**********三阶贝塞尔曲线轨迹**********************//

				//=====步骤2:求thta_p,前向、横向跟踪误差===//
				thta_p = atan2(y_p_der, x_p_der);

				if (thta_p < (-0.001))
				{
					thta_p = thta_p + 2 * pi;
				}
				else
				{
					thta_p = thta_p;
				}
				/**********error_s前向跟踪误差，error_e横向跟踪误差************/
				error_s = (x_measure - x_p) * cos(thta_p) + (y_measure - y_p) * sin(thta_p);
				error_e = -1 * (x_measure - x_p) * sin(thta_p) + (y_measure - y_p) * cos(thta_p);

				thta_e = thta_measure - thta_p;

				//*******贝塞尔曲线加的***************//

				if (thta_e > 2 * pi || thta_measure > 2 * pi)
				{
					thta_measure = thta_measure - 2 * pi;
				}

				if (thta_e > 0.001)
				{
					thta_e = thta_e - 2 * pi; //防止出现thta_p由360->0跳变时，thta_measure滞后，导致角速度跳变成不可控值而使系统不可控
											  //*******贝塞尔曲线删除的***************//
											  //thta_measure=thta_measure - 6.2832;
				}

				//=====步骤3:求w导数，thta_p导数,角速度dlta_a==//
				w_der = (vc * cos(thta_e) + ks * error_s) / (sqrt(x_p_der * x_p_der + y_p_der * y_p_der));
				thta_p_der = w_der * (y_p_derder * x_p_der - y_p_der * x_p_derder) / (x_p_der * x_p_der + y_p_der * y_p_der);
				w_omiga = kw * (thta_p + atan2(-error_e, ke) - thta_measure) + thta_p_der + ke * (thta_p_der * error_s - vc * sin(thta_measure - thta_p)) / (error_e * error_e + ke * ke);

				//=====步骤4:更新路径参数w===============//
				w = w + w_der;

				//***************以上算法实现部分**********************************//

				//====系统平台中的一些说明======//
				//===实际控制，线速度单位mm/s，角速度单位1/10°/s==//
				//===dlta_d>0为前进，get_y<0,    dlta_d<0为后退，get_y>0    ======//
				//===dlta_a>0时候左转get_x>0，get_thta由0->-1800->1800->0  ======//
				//===dlta_a<0时候右转get_x<0，get_thta由0->1800->-1800->0  ======//

				dlta_d = vc * 1000;
				dlta_a = 10 * w_omiga * 57.296; //180/3.14;

				if (dlta_a > 400)
				{
					dlta_a = 400;
				}
				if (dlta_a < -400)
				{
					dlta_a = -400;
				}
				file_write_xy(file_fd, (int)(w * 100), dlta_a);

				printf("w_der=%f\n", w_der * 57.296);
				printf("error_s=%f cm\n", error_s * 100);
				printf("error_e=%f cm\n", error_e * 100);
				printf("w_der=%f,w=%f\n", w_der, w);
				printf("dlta_a=%d\n", dlta_a);
				printf("dlta_d=%d\n", dlta_d);
				printf("x_measure=%f cm\n", x_measure * 100);
				printf("y_measure=%f cm\n", y_measure * 100);
				printf("thta_measure=%f \n", thta_measure * 57.296);
				printf("thta_p=%f  \n", thta_p * 57.296);

				//error_thta=(int)(atan2(-error_e,ke)*10000.0);
				//error_thta=(int)(error_e*10000.0);
				//printf("thta_error=%d \n",error_thta);
				//file_write_xy(file_fd, (int)(w*100), (int)(error_e*1000));
				//file_write_xy(file_fd, (int)(w*100), (int)(thta_e*573));
				//file_write_xy(file_fd, -get_x, -get_y);
				printf("\n");
			}
			else
			{
				dlta_d = 0;
				dlta_a = 0;
			}
		}
	}
}

int main(int argc, char **argv)
{
	struct timeval tv1;
	int i = 0;
	int choice_x = 0;
	int choice_y = 0;
	int count = 0;
	int differ = 0;
	int counter = 0;
	int res;
	float dis_50;
	//-----------io---------------
	int usart_fd = 0;
	//int can_fd=0;
	//int file_fd=0;
	unsigned char get_data[9];
	int read_num = 0;
	float sin_count = 0.001;
	FloatLongType fl;
	int lidar_stop = 0;
	int reset_flag = 0;
	int resetok_flag = 0;
	//----------------------------
	T_InputEvent tInputEvent;  //触摸屏相关
	T_VideoBuf tFrameBuf_main; //
	pthread_t tTreadID1;
	pthread_t tTreadID2;
	pthread_t tTreadID3;
	pthread_t tTreadID4;
	pthread_t tTreadID5;
	//------------lidar-----------------//
	pthread_t thrd_id1, thrd_id2;
	pthread_attr_t attr; //设置thrd_id1线程的属性
	struct sched_param param;

	res = pthread_attr_init(&attr);
	if (res != 0)
	{
		printf("Create attr failed!\n");
		exit(res);
	}

	pthread_attr_setschedpolicy(&attr, SCHED_RR); //设置线程的调度策略，SCHED_RR是设置一个时间片

	param.sched_priority = 99;
	res = pthread_attr_setschedparam(&attr, &param); //设置线程的调度参数,优先级设为99
	if (res != 0)
	{
		printf("Setting attr failed!\n");
		exit(res);
	}
	//------------lidar-----------------//

	signal(SIGPIPE, SIG_IGN);					//客户端半关闭，服务器端返回消息的时候就会收到内核给的SIGPIPE信号，忽略它
	pthread_cond_init(&global.db_update, NULL); /* 初始化 global.db_update(条件变量) 成员，用于操作robot_temp[]、robot_end[]、... */
	pthread_mutex_init(&global.db, NULL);		/* 初始化 global.db(配合条件变量的互斥锁) 成员 */
	pthread_mutex_init(&db_link, NULL);			//--用于操作链表 的互斥锁
	pthread_rwlock_init(&rwlock, NULL);			//初始化读写锁--此读写锁用于变量dlta_d和dlta_a
	pthread_rwlock_init(&rwlock_lidar, NULL);	//初始化雷达的读写锁
	global.order_flag = 0;
	/* 创建子线程 */
	/* 	res = pthread_create(&thrd_id1, &attr, Thread_Read, NULL); //------------lidar-----------------//
	if (res != 0)
	{
		printf("Create thread failed!");
		exit(res);
	}

	res = pthread_create(&thrd_id2, NULL, Thread_Handle, NULL); //------------lidar-----------------//
	if (res != 0)
	{
		printf("Create thread failed!");
		exit(res);
	} */

	pthread_create(&tTreadID3, NULL, socket_receive, NULL); //创建服务器（监听）线程
	pthread_create(&tTreadID2, NULL, socket_send, NULL);	//创建客户端线程
	milliseconds_sleep(100);								//主线程休眠100ms //主动让出cpu
	//先放在后面，先让出通信所需的资源

	pthread_create(&tTreadID1, NULL, VideoTreadFunction, NULL);

	//----------------------------
	InputInit();		   //触摸屏相关
	AllInputDevicesInit(); //触摸屏相关

	hzk_init(); //字库初始化，显示屏相关
	//----------------------------

	usart_fd = usart_init(); //串口用于电子罗盘
	can_fd = can_init();	 //CAN用于控制stm32pwm电机

	//close(file_fd);

	ask_data[0] = 0x01;
	ask_data[1] = 0x03;
	ask_data[2] = 0x00;
	ask_data[3] = 0x05;
	ask_data[4] = 0x00;
	ask_data[5] = 0x02;
	ask_data[6] = 0xD4;
	ask_data[7] = 0x0A; //陀螺仪Z轴数据申请

	set_data[0] = 0x01;
	set_data[1] = 0x06;
	set_data[2] = 0x00;
	set_data[3] = 0x13;
	set_data[4] = 0x00;
	//set_data[5]字节设置0x00代表绝对零点，0x01代表相对零点
	/*这里的绝对零点指的是robot第一次开机所朝向的角度，如果不reboot，会保持*/
	set_data[5] = 0x00;
	set_data[6] = 0xB9;
	set_data[7] = 0xCF; //相对零点设定
	//---------------生成一个CAN报文---------------//
	frame_send.can_id = 0x80cc0000;
	frame_send.data[0] = 0xfb;
	frame_send.data[1] = 0; //存放 (int)dlta_d 的第1字节
	frame_send.data[2] = 0; //存放 (int)dlta_d 的第0字节
	frame_send.data[3] = 0; //存放 (int)dlta_a 的第1字节
	frame_send.data[4] = 0; //存放 (int)dlta_a 的第0字节
	frame_send.data[5] = 0;
	frame_send.data[6] = 00;
	frame_send.data[7] = 00;
	frame_send.can_dlc = 8;

	//----------------------------
	GetVideoBufForDisplay(&tFrameBuf_main); //tFrameBuf 传出参数，存显示屏相关格式和数据信息

	DBG_PRINTF("x %d\n", tFrameBuf_main.tPixelDatas.iWidth);  //800
	DBG_PRINTF("y %d\n", tFrameBuf_main.tPixelDatas.iHeight); //600
															  //----------------------------

	//===系统定时器======//

	/* 	struct itimerval tick;

	signal(SIGALRM, ControlFun);
	memset(&tick, 0, sizeof(tick));

	//Timeout to run first time
	tick.it_value.tv_sec = 0;
	tick.it_value.tv_usec = 200000;

	//After first, the Interval time for clock
	tick.it_interval.tv_sec = 0;
	tick.it_interval.tv_usec = 200000;

	if (setitimer(ITIMER_REAL, &tick, NULL) < 0)
		printf("Set timer failed!\n"); */
	/* 	res = pthread_create(&tTreadID5, NULL, ControlFun, NULL);
	if (res != 0)
	{
		printf("Create thread failed!");
		exit(res);
	} */
	pthread_create(&tTreadID4, NULL, ekf_multi, NULL);
	/*所有关于卡尔曼滤波的线程都从上面这行开始，如果你想终止卡尔曼的所有程序，请将上一行屏蔽，这对主程序不会有影响*/
	char *local_ip = NULL;
	local_ip = GetLocalIp();
	if (atoi(substr(local_ip, 12, 1)) == 1) //若 local_ip='192.168.0.201' ,进此 if 语句
	{
		//201机器人观测时，与目标机器人实际距离为 50 cm时，像素点个数为 82
		dis_50 = 82.0;
	}
	else if (atoi(substr(local_ip, 12, 1)) == 4) //若 local_ip='192.168.0.204' ,进此 if 语句
	{
		//204机器人观测时，与目标机器人实际距离为 50 cm时，像素点个数为 72
		dis_50 = 72.0;
	}
	else if (atoi(substr(local_ip, 12, 1)) == 6) //若 local_ip='192.168.0.206' ,进此 if 语句
	{
		//206机器人观测时，与目标机器人实际距离为 50 cm时，像素点个数为 79
		dis_50 = 79.0;
	}
	if (!reset_flag)
	{
		frame_send.data[5] = 1; // 1底层坐标和角度清零
		nbytes = write(can_fd, &frame_send, sizeof(frame_send));
		reset_flag = 1;
		resetok_flag = 1;
		resetok_flag++;
	}
	write(usart_fd, set_data, sizeof(ask_data));
	while (1)
	{
		//---------------触摸屏---------------------
		if (0 == GetInputEvent(&tInputEvent))
		{
			DBG_PRINTF("x is %d\n", tInputEvent.x);
			DBG_PRINTF("y is %d\n", tInputEvent.y);
		}

		if (tInputEvent.x > 0 && tInputEvent.y > 0)
		{
			choice_x = tInputEvent.x;
			choice_y = tInputEvent.y;
		}
		//---------------------------------------------
		//--工程运行开始复位底层航位推算值---

		//--------------------------------------------------
		frame_send.data[5] = 0;
		nbytes = write(can_fd, &frame_send, sizeof(frame_send));
		addr_len = sizeof(addr);
		len = 0;
		/*接收 CAN 数据，注意第六个参数len是传入传出参数*/
		nbytes = recvfrom(s, &frame_rev, sizeof(struct can_frame), 0, (struct sockaddr *)&addr, &len);
		ifr.ifr_ifindex = addr.can_ifindex;
		ioctl(s, SIOCGIFNAME, &ifr);
		/* n*256 即 n<<8 */
		get_x = (short)(frame_rev.data[0] * 256 + frame_rev.data[1]);
		get_y = (short)(frame_rev.data[2] * 256 + frame_rev.data[3]);
		get_tht = (short)(frame_rev.data[4] * 256 + frame_rev.data[5]); //旋转角度θ
		/* 经底层航位推算（底层运动系统航位推算处理周期为 20ms），得到本机器人当前位姿 */
		/* 在给定机器人初始位置的情况下，通过航位推算可以得到机器人下一时刻的相对于初始位置的坐标和朝向角 */

		//---------------陀螺仪--------------------
		write(usart_fd, ask_data, sizeof(ask_data));
		read_num = read(usart_fd, get_data, sizeof(get_data));
		fl.ldata[3] = get_data[3];
		fl.ldata[2] = get_data[4];
		fl.ldata[1] = get_data[5];
		fl.ldata[0] = get_data[6]; //如果不理解为什么这么写可以去了解一下大端字节序和小端字节序
		gyroValue = fl.fdata;	   //共用体的巧妙使用
		mymsg_send.Ture_Tht = fl.fdata * 1000;
		Ture_Tht = mymsg_send.Ture_Tht;
		//printf("gyroValue = %f\n", gyroValue);
		//----------------电子罗盘-----------------------

		//----------------风速仪-----------------------

		//---------------无线网卡-----------------

		//-----------------综合显示部分------------------------------------
		pthread_mutex_lock(&g_tMutex);
		/* 休眠 */
		pthread_cond_wait(&g_tConVar, &g_tMutex);
		/* 被唤醒后,返回数据 */
		tConvertBuf_now = tConvertBuf_mid;
		pthread_mutex_unlock(&g_tMutex);
		//此时，线程 VideoTreadFunction 已循环完一次，即已调用 scan 函数， robot_temp[i].rad 和 robot_temp[i].angle 的值已被更新
		//------------------------------------------------------------------

		if (atoi(substr(local_ip, 12, 1)) == 1) //若 local_ip='192.168.0.201' ,进此 if 语句
		{
			for (i = 1; i <= 2; i++) //本机为201，则只需存 201与204关系 和 201与206关系 即可，故循环两次
			{
				//robot_temp[i].rad 的值会在线程 VideoTreadFunction 中调用 scan 函数进行更新
				robot_temp[i].dis_diff = robot_temp[i].rad - dis_50;
				//DBG_PRINTF("robot_temp[i].dis_diff %f\n", robot_temp[i].dis_diff);
				if (robot_temp[i].dis_diff < 75)
					robot_temp[i].distance = 0.0622 * pow(1.3137 * robot_temp[i].dis_diff, 1.4960) + 50;
				//robot_temp[i].distance 即本机器人观测到目标机器人的观测距离（cm）
				if (robot_temp[i].dis_diff >= 75 && robot_temp[i].dis_diff < 122)
					robot_temp[i].distance = 0.2567 * pow(0.2504 * robot_temp[i].dis_diff, 1.8704) + 50;
				if (robot_temp[i].dis_diff >= 122 && robot_temp[i].dis_diff < 139)
					robot_temp[i].distance = 0.0870 * pow(0.1347 * robot_temp[i].dis_diff, 2.6708) + 50;
				if (robot_temp[i].dis_diff >= 139 && robot_temp[i].dis_diff < 150)
					robot_temp[i].distance = 0.0936 * pow(0.0559 * robot_temp[i].dis_diff, 3.7819) + 50;
				if (robot_temp[i].dis_diff >= 150)
					robot_temp[i].distance = 0.0387 * pow(0.0431 * robot_temp[i].dis_diff, 4.7811) + 50;
				//DBG_PRINTF("robot_temp[i].distance %d\n",robot_temp[i].distance);
				//if(robot_temp[i].rad==0){robot_temp[i].distance=0;}
				//file_write_xy(file_fd, robot_ekf2[i].distance, robot_ekf2[i].angle);
			}
		}
		if (atoi(substr(local_ip, 12, 1)) == 4) //若 local_ip='192.168.0.204' ,进此 if 语句
		{
			for (i = 1; i <= 2; i++)
			{
				robot_temp[i].dis_diff = robot_temp[i].rad - dis_50;
				//DBG_PRINTF("robot_temp[i].dis_diff %d\n", robot_temp[i].rad);
				if (robot_temp[i].dis_diff < 75)
					robot_temp[i].distance = 0.2151 * pow(0.7600 * robot_temp[i].dis_diff, 1.3456) + 50;
				if (robot_temp[i].dis_diff >= 75 && robot_temp[i].dis_diff < 139)
					robot_temp[i].distance = 0.2351 * pow(0.2062 * robot_temp[i].dis_diff, 1.9648) + 50;
				if (robot_temp[i].dis_diff >= 139 && robot_temp[i].dis_diff < 155)
					robot_temp[i].distance = 0.0890 * pow(0.0801 * robot_temp[i].dis_diff, 3.1497) + 50;
				if (robot_temp[i].dis_diff >= 155 && robot_temp[i].dis_diff < 163)
					robot_temp[i].distance = 0.0507 * pow(0.0486 * robot_temp[i].dis_diff, 4.2175) + 50;
				if (robot_temp[i].dis_diff >= 163)
					robot_temp[i].distance = 0.0354 * pow(0.0377 * robot_temp[i].dis_diff, 5.0001) + 45;
				//DBG_PRINTF("robot_temp[i].distance %d\n", robot_temp[i].distance);
				//if(robot_temp[i].rad==0){robot_temp[i].distance=0;}
				//file_write_xy(file_fd, robot_ekf2[i].distance, robot_ekf2[i].angle);
			}
		}
		if (atoi(substr(local_ip, 12, 1)) == 6) //若 local_ip='192.168.0.206' ,进此 if 语句
		{
			for (i = 1; i <= 2; i++)
			{
				robot_temp[i].dis_diff = robot_temp[i].rad - dis_50;
				//DBG_PRINTF("robot_temp[i].dis_diff %f\n", robot_temp[i].dis_diff);
				if (robot_temp[i].dis_diff < 74)
					robot_temp[i].distance = 0.4485 * pow(0.4410 * robot_temp[i].dis_diff, 1.3532) + 50;
				if (robot_temp[i].dis_diff >= 74 && robot_temp[i].dis_diff < 139)
					robot_temp[i].distance = 0.2030 * pow(0.2064 * robot_temp[i].dis_diff, 2.0403) + 50;
				if (robot_temp[i].dis_diff >= 139 && robot_temp[i].dis_diff < 152)
					robot_temp[i].distance = 0.0292 * pow(0.0938 * robot_temp[i].dis_diff, 3.4354) + 50;
				if (robot_temp[i].dis_diff >= 152)
					robot_temp[i].distance = 0.0534 * pow(0.0464 * robot_temp[i].dis_diff, 4.3684) + 50;
				//DBG_PRINTF("robot_temp[i].distance %d\n",robot_temp[i].distance);
				//if(robot_temp[i].rad==0){robot_temp[i].distance=0;}
				//file_write_xy(file_fd, robot_ekf2[i].distance, robot_ekf2[i].angle);
			}
		}

		//---------------显示屏显示----------------------
		//ps:point_get和show_some是显示屏部分函数，由于ide的问题，可能会导致段错误，故暂时屏蔽
		//point_get(tInputEvent.x, tInputEvent.y, &tConvertBuf_now.tPixelDatas, &tFrameBuf_main.tPixelDatas);
		/*********这个函数用于寻找去往目标点的路径，RRT。并且将路径画了出来************/
		min_cir(tConvertBuf_now.tPixelDatas.aucPixelDatas);

		PicMerge(0, 0, &tConvertBuf_now.tPixelDatas, &tFrameBuf_main.tPixelDatas); //---x--y---图像经缩放整合到显示屏整个屏幕上

		//show_some(&tFrameBuf_main.tPixelDatas); //显示屏显示文字内容

		FlushPixelDatasToDev(&tFrameBuf_main.tPixelDatas); //向整个屏幕绘画（即把图像拷贝到显存里）
		//---------------保存处理好的图片-----------------
		/*	FILE *fp = fopen("frame1.jpg", "wb+");
			if (fp < 0)
			{
				DBG_PRINTF("open frame data file failed\n");
				return -1;
			}
			fwrite(ptVideoDevice->pucVideBuf[tV4l2Buf.index], 1, tV4l2Buf.length, fp);
			fclose(fp);
		*/
		//---------------触摸屏的触摸点清零------------
		tInputEvent.x = 0;
		tInputEvent.y = 0;
		//------------------------------------------------

		//------------------------互斥量用于收发线程---------
		pthread_mutex_lock(&global.db);
		//---------数据拷贝区域----------------------
		for (i = 1; i <= 6; i++) //由上文知，只有 i 取 1、2 时，才有意义
		{
			if (robot_temp[i].distance == 0) // i 取 3、4、5、6 时，进if
			{
				continue;
			}
			robot_end[i].distance = robot_temp[i].distance;
			robot_end[i].angle = robot_temp[i].angle;
			//robot_temp[i].angle 的值会在线程 VideoTreadFunction 中调用 scan 函数进行更新
		}
		/* printf("robot_end[1].distance = %d\n", robot_end[1].distance);
		printf("robot_end[1].angle = %d\n", robot_end[1].angle);
		printf("robot_end[2].distance = %d\n", robot_end[2].distance);
		printf("robot_end[2].angle = %d\n", robot_end[2].angle); */

		global.distanc = dlta_d;
		global.angl = dlta_a;
		robot_ekf[0].x = get_y;
		robot_ekf[0].y = get_x;
		robot_ekf[0].angle = get_tht; //经航位推算得到机器人的旋转角度

		mymsg_send.dlta_d = dlta_d;	  //填充 mymsg_send 结构体信息
		mymsg_send.dlta_a = dlta_a;	  //填充 mymsg_send 结构体信息
		mymsg_send.get_x = get_y;	  //填充 mymsg_send 结构体信息
		mymsg_send.get_y = get_x;	  //填充 mymsg_send 结构体信息
		mymsg_send.get_tht = get_tht; //填充 mymsg_send 结构体信息

		pthread_cond_broadcast(&global.db_update); // 发出一个数据更新的信号，通知发送通道来取数据
		pthread_mutex_unlock(&global.db);		   // 原子操作结束
	}
}

void *Thread_Read(void *arg)
{
	int fd = 0;
	int Node = 0; //数据应答报文的长度（字节）
	int wrs = 0;

	//int pi = 0;
	//int wrhealth = 0, readhel = 0;

	//char bufhelth[] = {0xA5, 0x52};
	char bufscan[] = {0xA5, 0x20}; //命令：开始进入扫描采样状态
	char bufstop[] = {0xA5, 0x25};
	char bufreset[] = {0xA5, 0x40};

	//char BufferHel[BUFFER_SIZE] = {0};
	char Buffer[BUFFER_SIZE] = {0};

	clock_t start, finish;
	double duration; //读取一个采样点的时间

	fd = open(DEV_FD, O_RDWR);
	if (fd == -1)
	{
		perror("serialport error\n");
	}
	else
	{
		printf("open %d succesfully!\n", fd);
	}
	set_lidaropt(fd, 115200, 8, 'N', 1); //初始化lidar所用串口

	//signal(SIGINT, ctrlc);	//监听ctrl + c 是否执行
	/* 健康码有严重的问题，禁止执行，直接执行scan命令！ */
	/* wrhealth = write(fd, bufhelth, sizeof(bufhelth));
	printf("wrh=%d\n", wrhealth);
	if ((readhel = read(fd, BufferHel, BUFFER_SIZE)) > 0)
	{
		printf("readhel length=%d\n", readhel);
		for (pi = 0; pi < readhel; pi++)
		{
			printf("%02x  ", BufferHel[pi]);
		}
		printf("\n");
	}
	if (BufferHel[readhel - 3] == 0)
	{ */

	wrs = write(fd, bufstop, sizeof(bufstop));
	usleep(100000);

	while (1)
	{
		int i = 0;
		memset(Buffer, 0, BUFFER_SIZE);

		if (!ReadHead)
		{
			wrs = write(fd, bufscan, sizeof(bufscan)); //发送请求报文（2字节）
			ReadByte = read(fd, Buffer, BUFFER_SIZE);  //读串口 头 ，接收 起始应答报文
		}
		if (ReadByte == 7) //起始应答报文（7字节） //数据应答报文（5字节）
		{
			for (i = 0; i < ReadByte; i++)
			{
				printf("%02x  ", Buffer[i]); //输出所读数据（A5 5A 05 00 00 40 81）
			}
			printf("\n");
			Node = Buffer[2]; //0x05,表示后续的数据应答报文的长度为 5 字节
			printf("Node=%d\n", Node);
			ReadHead = 1;
		}
		if (ReadHead)
		{
			fd_set rd; //读操作文件描述符集
			while (1)
			{
				memset(Buffer, 0, BUFFER_SIZE);
				ReadByte = 0;
				FD_ZERO(&rd);	 //清空 rd
				FD_SET(fd, &rd); //将 fd（要监听的文件描述符） 添加到 rd（读操作文件描述符集） 里面
				while (FD_ISSET(fd, &rd))
				{
					if (select(fd + 1, &rd, NULL, NULL, NULL) < 0) //阻塞监听文件描述符，直到有文件描述符就绪（可读）
					{
						perror("select error\n");
					}
					else
					{
						while ((ReadByte = read(fd, Buffer, sizeof(Buffer))) > 0) //读串口 头 ，接收 数据应答报文
						{
							start = clock();

							pthread_rwlock_rdlock(&rwlock_lidar); //获取读锁
							//pthread_mutex_lock(&g_tLidar);
							memcpy(buf, Buffer, ReadByte);
							memset(Buffer, 0, BUFFER_SIZE);
							//TotalReadByte += ReadByte;
							//printf("ReadByte length=%d \n", ReadByte);
							finish = clock();
							duration = (double)(finish - start) / CLOCKS_PER_SEC; //CLOCKS_PER_SEC表示一秒钟内CPU运行的时钟周期数
							//	printf( "%f seconds\n", duration );
							//pthread_mutex_unlock(&g_tLidar);
							pthread_rwlock_unlock(&rwlock_lidar); //释放读写锁
						}
					}
				}
			}
		}
	}
	//}

	close(fd);
	return 0;
}

void *Thread_Handle(void *arg)
{
	printf("this is thread handler 2!..............................................\n");

	int i = 0;
	int j = 0;
	int m = 0;

	int recvPos = 0;	 //数据应答报文 的 字节偏移量
	int NodeNum = 0;	 //采样点的个数
	int CountCircle = 0; //lidar圈数
	float CurLidarDistPre = 0;
	float CurLidarAng2pre = 0;
	//int file_fd = 0;

	clock_t start, finish;
	double duration;

	char NodeBuffer[BUF_NODEDATA] = {0}; //一个采样点 的 数据应答报文
	char tempbuf[BUF_NODEDATA] = {0};
	char FinalBuffer[BUFFER_SIZE] = {0}; //旋转一圈360个采样点的 数据应答报文

	int nbytes;
	int can_fd = 0;

	AimCount = 0;
	memset(CurLidarAng, 0, 1024);
	memset(CurLidarDistPix, 0, 1024);
	//file_fd = thread_file_init();
	//LidarStopFlag=0;
	while (1)
	{
		pthread_rwlock_wrlock(&rwlock_lidar); //获取写锁
		//pthread_mutex_lock(&g_tLidar);
		//printf("......................................thread 2 ReadByte Length = %d\n",ReadByte);
		start = clock();
		/*******************应答数据起始数据位判断*******************/
		for (i = 0; i < ReadByte; i++) //ReadByte = 5
		{
			char currentByte = buf[i];
			switch (recvPos)
			{
			case 0: // 期待这个字节的同步位和反向
			{
				int tmp = (currentByte >> 1);  //相当于校验S^,用于数据应答报文起始字节
				if ((tmp ^ currentByte) & 0x1) //tmp ^ currentByte必须是1
				{
				}
				else
				{
					continue; //如果不是数据应答起始，就重找
				}
			}
			break;

			case 1: //期望最低位为1
			{
				if (currentByte & CHECK_FLAG) //用C作为校验位
				{
				}
				else
				{
					recvPos = 0;
					continue;
				}
			}
			break;
			}
			NodeBuffer[recvPos++] = currentByte; /* 例： x=6；y=x++ ——> y=6,x=7	*/
			if (recvPos == BUF_NODEDATA)
			{
				for (j = 0; j < recvPos; j++)
				{
					FinalBuffer[j + NodeNum * BUF_NODEDATA] = NodeBuffer[j];
				}
				NodeNum++;
				memset(NodeBuffer, 0, BUF_NODEDATA);
				recvPos = 0;
			}
			if (NodeNum == PERCIRCLE_DATA)
			{
				/******************根据角度大小排序****************/ //（冒泡排序）
				for (i = 0; i < (PERCIRCLE_DATA - 1) * BUF_NODEDATA; i = i + 5)
				{
					for (j = (i + 5); j < PERCIRCLE_DATA * BUF_NODEDATA; j = j + 5)
					{
						if (((FinalBuffer[i + 2] << 8) | (FinalBuffer[i + 1])) > ((FinalBuffer[j + 2] << 8) | (FinalBuffer[j + 1]))) // > 升序
						{
							for (m = 0; m < 5; m++)
							{
								tempbuf[m] = FinalBuffer[i + m];
								FinalBuffer[i + m] = FinalBuffer[j + m];
								FinalBuffer[j + m] = tempbuf[m];
							}
							memset(tempbuf, 0, sizeof(tempbuf));
						}
					}
				}
				/************************************************************/
				/************打印采集到的数据***********/
				for (j = 0; j < PERCIRCLE_DATA * BUF_NODEDATA; j = j + 5)
				{ //**************激光雷达****正向0-180度*****//
					if (((((((FinalBuffer[j + 2] << 8) | (FinalBuffer[j + 1])) >> 1) / 64.0) < 90.0) || (((((FinalBuffer[j + 2] << 8) | (FinalBuffer[j + 1])) >> 1) / 64.0) > 270.0))
						/*********雷达不要有超过360的标定。只选取距离为（100，4000）的障碍物************/
						&& (((((FinalBuffer[j + 2] << 8) | (FinalBuffer[j + 1])) >> 1) / 64.0) < 360.0) && ((((FinalBuffer[j + 4] << 8) | (FinalBuffer[j + 3])) / 4.0) > 100.0) && ((((FinalBuffer[j + 4] << 8) | (FinalBuffer[j + 3])) / 4.0) < 4000.0) && (FinalBuffer[j] >> 2) > 5)
					{
						/* 角度值&&距离值 */
						CurLidarAng2 = (((FinalBuffer[j + 2] << 8) | (FinalBuffer[j + 1])) >> 1) / 64.0;
						CurLidarAng2 = CurLidarAng2 * 10;
						CurLidarDist = ((FinalBuffer[j + 4] << 8) | (FinalBuffer[j + 3])) / 4.0 + 130;
						//CurLidarDist=pow(pow(130,2)+pow(CurLidarDist,2)+2*130*CurLidarDist*cos(CurLidarDist*3.14 / 1800),0.5);
						//CurLidarDist=pow(pow(130,2)+pow(CurLidarDistPre,2)+2*130*CurLidarDistPre*cos(CurLidarAng2pre*3.14 / 1800),0.5);
						//CurLidarAng2=acos((CurLidarDistPre*cos(CurLidarAng2pre*3.14 / 1800)+130)/CurLidarDist);
						//CurLidarAng2=acos((pow(CurLidarDist,2)+pow(130,2)-pow(CurLidarDistPre,2))/(2*130*CurLidarDist));
						//CurLidarAng2=CurLidarAng2 *1800 / 3.14;
						CurLidarQuality = FinalBuffer[j] >> 2;
						//printf("CurLidarDist=%f \n", CurLidarDist);

						if ((CurLidarDist > 150) && (CurLidarDist < (618 + 300)))
						{
							CurLidarDistPix2 = 17.6132 * pow(0.0099 * ((CurLidarDist - 430)), 0.8712) + 41;
							//printf("di 1 ge...\n");
						}
						if ((CurLidarDist > (618 + 300)) && (CurLidarDist < (995 + 300)))
						{
							CurLidarDistPix2 = 9.9210 * pow(0.0918 * ((CurLidarDist - 430)), 0.5322) + 41;
							//printf("di 2 ge...\n");
						}
						if ((CurLidarDist > (995 + 300)) && (CurLidarDist < (2019 + 300)))
						{
							CurLidarDistPix2 = 2.7994 * pow(2.4136 * ((CurLidarDist - 430)), 0.4712) + 41;
							//printf("di 3 ge...\n");
						}
						if (CurLidarDist > (2019 + 300))
						{
							CurLidarDistPix2 = 4.0607 * pow(4.0504 * ((CurLidarDist - 430)), 0.4028) + 41;
							//printf("di 4 ge...\n");
						}

						CurLidarAng[AimCount] = (int)CurLidarAng2;
						CurLidarDistPix[AimCount] = CurLidarDistPix2;
						CurLidarDistance[AimCount] = CurLidarDist;
						//file_write_xy(file_fd, CurLidarAng[AimCount], CurLidarDistance[AimCount]);
						//printf("CurLidarDistance=%d \n", CurLidarDistance[AimCount]);
						AimCount++;
						if ((((FinalBuffer[j + 4] << 8) | (FinalBuffer[j + 3])) / 4.0) < 500.0 && ((((FinalBuffer[j + 2] << 8) | (FinalBuffer[j + 1])) >> 1) / 64.0) < 15.0)
						//&&(((((FinalBuffer[j+2]<<8) | (FinalBuffer[j+1]))>>1)/64.0) < 4.0 || ((((FinalBuffer[j+2]<<8) | (FinalBuffer[j+1]))>>1)/64.0) > 5.0))
						//	&&  ((FinalBuffer[j]>>2)>20))
						//	&&((((((FinalBuffer[j+2]<<8) | (FinalBuffer[j+1]))>>1)/64.0) < 10.0)
						//	||(((((FinalBuffer[j+2]<<8) | (FinalBuffer[j+1]))>>1)/64.0) > 350.0)))
						{
							LidarStopFlag = 1;
							/* 	printf("theta: %03.2f Dist: %08.2f  Q: %d \n",
								   (((FinalBuffer[j + 2] << 8) | (FinalBuffer[j + 1])) >> 1) / 64.0,
								   ((FinalBuffer[j + 4] << 8) | (FinalBuffer[j + 3])) / 4.0 + 130,
								   FinalBuffer[j] >> 2); */
						}
						else if ((((FinalBuffer[j + 4] << 8) | (FinalBuffer[j + 3])) / 4.0) > 500.0 && ((((FinalBuffer[j + 2] << 8) | (FinalBuffer[j + 1])) >> 1) / 64.0) < 15.0 && j == (PERCIRCLE_DATA - 1) * BUF_NODEDATA)
						{
							LidarStopFlag = 0;
						}
					}
				}
				CountCircle++;
				NodeNum = 0;
				memset(FinalBuffer, 0, BUFFER_SIZE);
				/* 				memset(CurLidarAng, 0, 1024);
				memset(CurLidarDistPix, 0, 1024);
				memset(CurLidarDistance, 0, 1024); */
				//AimCount = 0;
				//printf("\n");
			}
		}
		//printf("CountCircle *************************************  = %d\n",CountCircle);

		finish = clock();
		duration = (double)(finish - start) / CLOCKS_PER_SEC;
		//printf( "%f seconds\n", duration );
		pthread_rwlock_unlock(&rwlock_lidar);
		//pthread_mutex_unlock(&g_tLidar);

		usleep(10000); //主动休眠10ms，令 Thread_Read 线程有机会获取读锁
	}
}

/*            客户端             */
void *socket_send(void *povid)
{
	PNODE phead_send = NULL;

	int once = 1;
	int temp = 0;
	struct timeval tv_limt;
	int time_limt = 0;
	int len = 0;
	int i = 0, j = 1;
	int ReceLen = 0;
	int get_Num = 0;
	// signal(SIGCHLD,SIG_IGN);
	signal(SIGPIPE, SIG_IGN);

	//send_server_init();
	tSocketServerAddr.sin_family = AF_INET;
	tSocketServerAddr.sin_port = htons(SERVER_PORT); /* host to net, short */

	char buf_n[100];
	char buf_y[100];
	unsigned char *local_ip;
	char *last_ip;
	int last_data;

	signal(SIGPIPE, SIG_IGN);
	//--------------得到本地ip -----------------
	local_ip = GetLocalIp();
	strcpy(mymsg_send.local_ip, local_ip); //填充 mymsg_send 结构体信息
	printf("local ip is %s\n", local_ip);
	//----------------------------------------------------------------

	//------------根据自己的ip 等级设置超时时间--------------
	if (strcmp(local_ip, all_ip[0]) == 0) //192.168.0.201
		time_limt = 1000;
	if (strcmp(local_ip, all_ip[1]) == 0)
		time_limt = 3000;
	if (strcmp(local_ip, all_ip[2]) == 0)
		time_limt = 7000;
	if (strcmp(local_ip, all_ip[3]) == 0) //192.168.0.204
		time_limt = 15000;
	if (strcmp(local_ip, all_ip[4]) == 0)
		time_limt = 31000;
	if (strcmp(local_ip, all_ip[5]) == 0) //192.168.0.206
		time_limt = 63000;

	//--------测试------------
	//printf("this number is %d\n",strcmp(all_ip[1],all_ip[0]));
	//---------------------开机加入通信环中------
	sleep(1); //延时一到二秒是必须的

	//注：目前三台机器人，故网络中只有 192.168.0.201、204、206 是有效IP地址，即i=0、3、5时，connect_to()才会成功连接返回1
	for (i = 0; i < 7; i++)
	{
		//gettimeofday(&tv_glo_pre, NULL);
		rec_content.connet_flag[i] = connect_to(&rec_content.iSocketClient[i], all_ip[i]);

		//gettimeofday(&tv_glo, NULL);
		//DBG_PRINTF("add time is %f \n", (tv_glo.tv_sec - tv_glo_pre.tv_sec) + (tv_glo.tv_usec / 1000000. - tv_glo_pre.tv_usec / 1000000.));
		if (rec_content.connet_flag[i] == 1) //连接成功加入环形链表中
		{
			pthread_mutex_lock(&db_link);
			if (NULL == findIp(phead_rec, all_ip[i]))
				phead = addBack(phead, i + 1, all_ip[i], rec_content.iSocketClient[i]);
			printf("send fd is %s %d\n", all_ip[i], rec_content.iSocketClient[i]);
			pthread_mutex_unlock(&db_link);
		}
	}
	showAll(phead);

	//---------------------------------------------
	/*if(strcmp(local_ip,all_ip[0])==0) 
	{
		global.order_flag=1;//第一号先得到令牌
		for(i=1;i<7;i++)
			{
				rec_content.connet_flag[i]=connect_to(&rec_content.iSocketClient[i],all_ip[i]);
			}
		}
	*/
	printf("..................................................\n");

	//--------------------------------------------------------------------------
	while (1)
	{

		//pthread_mutex_lock( &global.db );
		pthread_cond_wait(&global.db_update, &global.db);
		for (i = 1; i < 3; i++) // i取1、2
		{

			robot_send[i].distance = robot_end[i].distance;
			robot_send[i].angle = robot_end[i].angle;

			mymsg_send.robot_msg[i].distance = robot_end[i].distance; //填充 mymsg_send 结构体信息
			mymsg_send.robot_msg[i].angle = robot_end[i].angle;		  //填充 mymsg_send 结构体信息
		}
		pthread_mutex_unlock(&global.db);
		//--------------------------------------------------------------

		if (once == 1)
		{
			once = 0;
			gettimeofday(&tv_limt, NULL);
			get_time.PreMs = tv_limt.tv_sec * 1000 + tv_limt.tv_usec / 1000.;
		}
		gettimeofday(&tv_limt, NULL);
		get_time.NowMs = tv_limt.tv_sec * 1000 + tv_limt.tv_usec / 1000.;
		if ((get_time.NowMs > get_time.PreMs + time_limt) && get_Num >= 2) //（超时处理）超时等待，将自动生成令牌
		{
			global.order_flag = 1;
			printf("this is out_time then have own order!\n");
		}
		//------------------------------------------------------
		pthread_mutex_lock(&db_link);
		get_Num = getNum(phead_rec);
		//printf("phead_rec number is %d\n",get_Num);
		//11 showAll(phead_rec);

		for (i = 0; i < get_Num; i++)
		{

			if (phead_rec->start_connect_flag == 0)
			{
				phead_rec->start_connect_flag = 1;
				phead_rec->connected_flag = connect_to(&temp, phead_rec->send_ip);
				if (phead_rec->connected_flag == 1)
				{
					phead = addBack(phead, phead_rec->send_ip[12] - 48, phead_rec->send_ip, temp);
					//11  printf("rece send fd is %s %d",phead_rec->send_ip,temp);
				}
			}
			phead_rec = phead_rec->pNext;
		}
		//11 showAll(phead);

		get_Num = getNum(phead);
		robot_end[1].count = get_Num;
		//printf("phead number is %d\n",get_Num);
		phead_send = phead;
		pthread_mutex_unlock(&db_link);
		//---------------------------------------------------------
		// printf("send num is %d\n",get_Num);
		if (global.order_flag == 1 && get_Num >= 2)
		{
			global.order_flag = 0;
			//// sprintf(buf_n, "%s%s%6d%4d%4d%6d%6d%6d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d", "n", local_ip, Ture_Tht, global.distanc, global.angl, robot_ekf[0].x, robot_ekf[0].y, robot_ekf[0].angle, robot_send[1].distance, robot_send[1].angle, robot_send[2].distance, robot_send[2].angle, robot_send[3].distance, robot_send[3].angle, robot_send[4].distance, robot_send[4].angle, robot_send[5].distance, robot_send[5].angle, robot_send[6].distance, robot_send[6].angle);
			//// sprintf(buf_y, "%s%s%6d%4d%4d%6d%6d%6d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d", "y", local_ip, Ture_Tht, global.distanc, global.angl, robot_ekf[0].x, robot_ekf[0].y, robot_ekf[0].angle, robot_send[1].distance, robot_send[1].angle, robot_send[2].distance, robot_send[2].angle, robot_send[3].distance, robot_send[3].angle, robot_send[4].distance, robot_send[4].angle, robot_send[5].distance, robot_send[5].angle, robot_send[6].distance, robot_send[6].angle);
			//自己IP		//自己电子罗盘朝向角（航向角）				//自己航位推算值									  //其它机器人在本机坐标系下的观测距离和观测角度	   //其它机器人在本机坐标系下的观测距离和观测角度
			//	printf("buf_n atoi(substr(src, 14, 6)) = %d\n", atoi(substr(buf_n, 14, 6)));
			//	printf("buf_n atoi(substr(src, 14, 6)) = %d\n", atoi(substr(buf_n, 20, 4)));
			//	printf("buf_n atoi(substr(src, 14, 6)) = %d\n", atoi(substr(buf_n, 14, 6)) / 100);
			//	sprintf(buf_n, "%s%s%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d", "n", local_ip, global.distanc, global.angl, robot_send[1].distance, robot_send[1].angle, robot_send[2].distance, robot_send[2].angle, robot_send[3].distance, robot_send[3].angle, robot_send[4].distance, robot_send[4].angle, robot_send[5].distance, robot_send[5].angle, robot_send[6].distance, robot_send[6].angle);
			//	sprintf(buf_y, "%s%s%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d", "y", local_ip, global.distanc, global.angl, robot_send[1].distance, robot_send[1].angle, robot_send[2].distance, robot_send[2].angle, robot_send[3].distance, robot_send[3].angle, robot_send[4].distance, robot_send[4].angle, robot_send[5].distance, robot_send[5].angle, robot_send[6].distance, robot_send[6].angle);
			//	printf("global.distanc=%d", global.distanc);
			//--------------------链表排序---------------

			phead_send = sortLink(phead_send);
			//showAll(phead_send);

			//-------------------发送数据----------------
			phead_send = findIp(phead_send, local_ip);
			phead_send = phead_send->pNext->pNext;

			for (i = 0; i < get_Num; i++)
			{

				//gettimeofday(&tv_glo_pre, NULL);

				if (i < get_Num - 1)
				{
					mymsg_send.flag = 'n'; //填充 mymsg_send 结构体信息
				}
				else
				{
					mymsg_send.flag = 'y'; //填充 mymsg_send 结构体信息
				}
				memcpy(mymsg_send_, &mymsg_send, sizeof(mymsg_send)); //结构体转字符串
				ReceLen = send(phead_send->iSocketClient, mymsg_send_, sizeof(mymsg_send_), 0);
				if (ReceLen <= 0)
				{
					phead_send = deleteFirst(phead_send, phead_send->data);
					//pthread_mutex_lock(&db_link);
					//phead=deleteFirst(phead,phead_send->data);
					//pthread_mutex_unlock(&db_link);
				}

				phead_send = phead_send->pNext;
			}
			//11 printf("phead_send number is %d\n",getNum(phead_send));
			//11 showAll(phead_send);
			pthread_mutex_lock(&db_link);
			phead = phead_send;
			pthread_mutex_unlock(&db_link);

			//--------------------更新初始时间--------
			gettimeofday(&tv_limt, NULL);
			get_time.PreMs = tv_limt.tv_sec + tv_limt.tv_usec / 1000000;
		}
	}
}
//===========================================
//---------服务器连接-------------接受线程-----------------------
//============================================
void *client_thread(void *arg)
{
	int i = 0, j = 0;
	int k = 0;
	int l = 1;
	int iRecvLen;
	int SendLen = 0;
	char buf[80];
	unsigned char ucRecvBuf[1000];
	int iClientNum = -1;
	char *local_ip = NULL;
	pt_net_rec p_net_rec = (t_net_rec *)malloc(sizeof(t_net_rec) + 20 * sizeof(char));
	memcpy(p_net_rec, arg, sizeof(t_net_rec) + 20 * sizeof(char));
	printf("now connect ip is %s\n", p_net_rec->ip); //客户端IP
	local_ip = GetLocalIp();

	//----------------------对于符合访问条件的加入链表中-------

	pthread_mutex_lock(&db_link);

	if (strcmp(local_ip, p_net_rec->ip) != 0) //剔除掉本机客户端
		if (NULL == findIp(phead, p_net_rec->ip) && p_net_rec->ip[10] != 49)
		{
			phead_rec = addBack_rec(phead_rec, p_net_rec->ip[12] - 48, p_net_rec->ip);
		}

	pthread_mutex_unlock(&db_link);

	showAll(phead_rec);

	//-------------------------------------------------------------
	while (1)
	{
		//printf("this is %d\n",p_net_rec->iSocketClient);
		//printf("count is %d\n",iClientNum++);
		//printf("Get connect from client %d : %s\n",  iClientNum, p_net_rec->ip);

		//// iRecvLen = recv(p_net_rec->iSocketClient, ucRecvBuf, 999, 0);

		iRecvLen = recv(p_net_rec->iSocketClient, mymsg_recv_, 100, 0);
		memcpy(&mymsg_recv, mymsg_recv_, sizeof(mymsg_recv)); //把接收到的信息转换成结构体
															  /*printf("flag: %c\n", mymsg_recv.flag);
		printf("local_ip: %s\n", mymsg_recv.local_ip);
		printf("Ture_Tht: %d\n", mymsg_recv.Ture_Tht);
		printf("dlta_d: %d\n", mymsg_recv.dlta_d);
		printf("dlta_a: %d\n", mymsg_recv.dlta_a);
		printf("get_x: %d\n", mymsg_recv.get_x);
		printf("get_y: %d\n", mymsg_recv.get_y);
		printf("get_tht: %d\n", mymsg_recv.get_tht);
		printf("1_distance: %d\n", mymsg_recv.robot_msg[1].distance);
		printf("1_angle: %d\n", mymsg_recv.robot_msg[1].angle);
		printf("2_distance: %d\n", mymsg_recv.robot_msg[2].distance);
		printf("2_angle: %d\n", mymsg_recv.robot_msg[2].angle); */

		//printf("Get Msg From Client %d\n", iRecvLen);
		//printf("receive is %d %d %d \n",ucRecvBuf[0],ucRecvBuf[1],ucRecvBuf[2]);
		//send(p_net_rec->iSocketClient, "123\n", strlen("123\n"), 0);

		if (iRecvLen <= 0)
		{
			close(p_net_rec->iSocketClient);
			return -1;
		}
		else
		{
			if (atoi(substr(mymsg_recv.local_ip, 12, 1)) != atoi(substr(local_ip, 12, 1))) //非本机客户端，进此if语句
			{
				switch (atoi(substr(mymsg_recv.local_ip, 12, 1)))
				{
				case 1:
					memcpy(&ekfRecvBuf[1], &mymsg_recv, sizeof(mymsg_recv));
					//	assignment(ekfRecvBuf[1], robot_end[1].count, 1, local_ip);
					break;
				case 4:
					if (atoi(substr(local_ip, 12, 1)) == 1)
					{
						memcpy(&ekfRecvBuf[1], &mymsg_recv, sizeof(mymsg_recv));
						//	assignment(ekfRecvBuf[1], robot_end[1].count, 1, local_ip);
					}
					else
					{
						memcpy(&ekfRecvBuf[2], &mymsg_recv, sizeof(mymsg_recv));
						//	assignment(ekfRecvBuf[2], robot_end[1].count, 2, local_ip);
					}
					break;
				case 6:
					memcpy(&ekfRecvBuf[2], &mymsg_recv, sizeof(mymsg_recv));
					//assignment(ekfRecvBuf[2], robot_end[1].count, 2, local_ip);
					break;
				default:
					break;
				}
			}
			if (mymsg_recv.flag == 'y' || mymsg_recv.flag == 'n')
			{
				if (mymsg_recv.flag == 'y')
				{
					//printf("------------------------I get a order-------form %s------\n", substr(mymsg_recv.local_ip, 12, 1));// 测试令牌环网络是否正常
					global.order_flag = 1;
				}
			}
			//-------------------------加入链表------------------------------------------
		}
	}
}

//------服务器监听---------------------------
void *socket_receive(void *povid)
{
	int iRet;
	int opt = 1;
	pthread_t client;
	int listenfd;	   //监听套接字描述符
	int iSocketClient; //服务器连接套接字描述符
	struct sockaddr_in tSocketServerAddr;
	struct sockaddr_in tSocketClientAddr;
	int iAddrLen;

	int iClientNum = -1;
	signal(SIGPIPE, SIG_IGN);
	listenfd = socket(AF_INET, SOCK_STREAM, 0);
	if (-1 == listenfd)
	{
		printf("socket error!\n");
		return -1;
	}
	//-------------------------------------------------
	//设置监听描述符为 端口复用，防止因端口处于 TIME_WAIT 状态(2MSL)使服务器无法启动
	if (setsockopt(listenfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0)
	{
		perror("setsockopt(SO_REUSEADDR) failed");
		exit(EXIT_FAILURE);
	}

	//-------------------------------------------------
	tSocketServerAddr.sin_family = AF_INET;
	tSocketServerAddr.sin_port = htons(SERVER_PORT); /* host to net, short */
	tSocketServerAddr.sin_addr.s_addr = INADDR_ANY;	 /* 自动指定本机可用的IP */
	memset(tSocketServerAddr.sin_zero, 0, 8);
	iRet = bind(listenfd, (const struct sockaddr *)&tSocketServerAddr, sizeof(struct sockaddr));
	if (-1 == iRet)
	{
		printf("bind error!\n");
		return -1;
	}

	iRet = listen(listenfd, BACKLOG);
	printf("listen ok\n");
	if (-1 == iRet)
	{
		printf("listen error!\n");
		return -1;
	}

	while (1)
	{
		pt_net_rec p_net_rec = (pt_net_rec)malloc(sizeof(t_net_rec) + 20 * sizeof(char)); //堆区开辟内存

		iAddrLen = sizeof(struct sockaddr);
		printf("start accept \n");
		iSocketClient = accept(listenfd, (struct sockaddr *)&tSocketClientAddr, &iAddrLen);
		printf("accept ok \n");
		if (-1 != iSocketClient)
		{
			/* 将 客户端地址结构 和 服务器连接套接字描述符 绑定在一起，一一对应 */
			p_net_rec->ip = inet_ntoa(tSocketClientAddr.sin_addr); //客户端地址结构IP
			p_net_rec->iSocketClient = iSocketClient;			   //服务器连接套接字描述符
			//	iClientNum++;

			printf("befor Get connect from client %d : %s\n", iClientNum, inet_ntoa(tSocketClientAddr.sin_addr));

			pthread_create(&client, NULL, &client_thread, p_net_rec);

			//	pthread_detach(client);	//设置线程分离
			//  pthread_join(client,&tret); // 等待线程结束,回收资源
			//  printf("thread exit code %ld \n",(long)tret);
		}
	}

	close(listenfd);
	return 0;
}

/*
	这个线程主要是对采集的数据进行滤波，获取机器人之间相对准确的距离值和角度值
	采用的是卡尔曼滤波
*/

void *ekf_algorithm(void *arg)
{
	int a = *(int *)arg;
	int k = 0;
	ERROR_ID errorID = _ERROR_NO_ERROR;
	REAL trace;
	STACKS S;
	init_stack(&S);
	InitEkf(a, &S, errorID);
	char *local_ip = NULL;
	//file_fd = file_init(); //打开记录文件 ./record/x.txt
	local_ip = GetLocalIp();
	int i;
	struct timeval ekf_timeval;

	while (1)
	{
		gettimeofday(&ekf_timeval, NULL);
		ekf_gettime.start_flag = ekf_timeval.tv_sec * 1000000 + ekf_timeval.tv_usec;
		//pthread_cond_wait(&global.db_update, &global.db);
		//pthread_rwlock_rdlock(&rwlock);
		for (i = 1; i < 3; i++) //i取1，2
		{

			robot_ekf[i].distance = robot_end[i].distance;
			robot_ekf[i].angle = robot_end[i].angle;
			//printf("angle1 = %d\n", robot_ekf[i].angle);
		}
		//pthread_rwlock_unlock(&rwlock);
		//pthread_mutex_unlock(&global.db);

		assignMent(ekfRecvBuf[a], robot_end[1].count, a, local_ip);
		//                         本机观测目标的观测距离			//本机观测目标的观测角度
		ekf[a].Zvalue[0] = (double)robot_ekf2[a].distance * cos(robot_ekf2[a].angle * pi / 1800.0);
		ekf[a].Zvalue[1] = (double)robot_ekf2[a].distance * sin(robot_ekf2[a].angle * pi / 1800.0);
		ekf[a].Zvalue[2] = robot_ekf2[a].Ture_Tht; //两个机器人之间角度差
		//ekf[a].Zvalue[2] = (double)robot_ekf2[a].angle;

		ekf[a].Z->p = ekf[a].Zvalue;

		//printf("%d\n", __LINE__);
		if (ekf[a].Xvalue[0] >= -0.00001 && ekf[a].Xvalue[0] <= 0.00001 &&
			ekf[a].Xvalue[1] >= -0.00001 && ekf[a].Xvalue[1] <= 0.00001)
		{
			printf("set value for X\n");
			ekf[a].Xvalue[0] = ekf[a].Zvalue[0];
			ekf[a].Xvalue[1] = ekf[a].Zvalue[1];
			ekf[a].Xvalue[2] = ekf[a].Zvalue[2];
			ekf[a].X->p = ekf[a].Xvalue; //赋初值
			ekf[a].gyro2 = Ture_Tht;
			ekf[a].targetGyro2 = robot_send[a].gyro;
		}
		/**************************陀螺仪数据*******************************/
		//向左转偏转角为正
		ekf[a].gyro1 = Ture_Tht;
		ekf[a].gyro = ekf[a].gyro1 - ekf[a].gyro2;
		ekf[a].gyro2 = ekf[a].gyro1;
		printf("本机的陀螺仪偏转角为 %d\n", ekf[a].gyro);

		ekf[a].targetGyro1 = robot_send[a].gyro;
		ekf[a].targetGyro = ekf[a].targetGyro1 - ekf[a].targetGyro2;
		ekf[a].targetGyro2 = ekf[a].targetGyro1;
		printf("目标机器人的陀螺仪偏转角为 %d\n", ekf[a].targetGyro);
		//------------------------------------------------------------------

		/**************************里程计数据*******************************/
		//向左转偏转角为正

		ekf[a].tht6 = robot_ekf[0].angle;
		ekf[a].tht8 = -(ekf[a].tht6 - ekf[a].tht7);
		ekf[a].tht7 = ekf[a].tht6;
		//printf("本机的里程计偏转角为 %d\n", ekf[a].tht8);

		ekf[a].x1 = robot_ekf[0].x;
		ekf[a].x2 = ekf[a].x1 - ekf[a].x3;
		ekf[a].x3 = ekf[a].x1;

		ekf[a].y1 = robot_ekf[0].y;
		ekf[a].y2 = ekf[a].y1 - ekf[a].y3;
		ekf[a].y3 = ekf[a].y1;
		ekf[a].distance = (int)sqrt(pow(ekf[a].x2, 2) + pow(ekf[a].y2, 2));
		//printf("本机的里程计前进距离为 %d\n", ekf[a].distance);

		ekf[a].tht3 = robot_send[a].angle;
		ekf[a].tht4 = -(ekf[a].tht3 - ekf[a].tht5);
		ekf[a].tht5 = ekf[a].tht3;
		//printf("%d号机器人的里程计偏转角为%d\n", robot_ekf2[a].number, ekf[a].tht4);

		ekf[a].x4 = robot_send[a].x;
		ekf[a].x5 = ekf[a].x4 - ekf[a].x6;
		ekf[a].x6 = ekf[a].x4;
		ekf[a].y4 = robot_send[a].y;
		ekf[a].y5 = ekf[a].y4 - ekf[a].y6;
		ekf[a].y6 = ekf[a].y4;
		ekf[a].distance1 = (int)sqrt(pow(ekf[a].x5, 2) + pow(ekf[a].y5, 2));
		//printf("%d号机器人的里程计前进距离为 %d\n", robot_ekf2[a].number, ekf[a].distance);
		//-----------------------------------------------------------------------

		//printf("%d号机器人运行的距离为%d\n", robot_ekf2[a].number, ekf[a].distance1);
		ekf[a].X_value[0] = cos((ekf[a].tht8 + 900.) * pi / 1800) * (ekf[a].Xvalue[0] - ekf[a].distance * 0.1 * cos((ekf[a].tht8 + 900.) * pi / 1800)) +
							sin((ekf[a].tht8 + 900.) * pi / 1800) * (ekf[a].Xvalue[1] - ekf[a].distance * 0.1 * sin((ekf[a].tht8 + 900.) * pi / 1800));
		ekf[a].X_value[1] = -sin((ekf[a].tht8 + 900.) * pi / 1800) * (ekf[a].Xvalue[0] - ekf[a].distance * 0.1 * cos((ekf[a].tht8 + 900.) * pi / 1800)) +
							cos((ekf[a].tht8 + 900.) * pi / 1800) * (ekf[a].Xvalue[1] - ekf[a].distance * 0.1 * sin((ekf[a].tht8 + 900.) * pi / 1800));
		ekf[a].X_value[0] += ekf[a].distance1 * 0.1 * cos((900. - (ekf[a].Xvalue[2] + (ekf[a].tht8 - ekf[a].tht4) * 0.1)) * pi / 1800);
		ekf[a].X_value[1] += ekf[a].distance1 * 0.1 * sin((900. - (ekf[a].Xvalue[2] + (ekf[a].tht8 - ekf[a].tht4) * 0.1)) * pi / 1800);
		ekf[a].X_value[2] = ekf[a].Xvalue[2] + (ekf[a].tht8 - ekf[a].tht4) * 0.1;
		//本次的先验角度值计算= 上次最优估计角度  加  本机器人      加 目标机器人之间新增加的角度
		//printf("ekf[a].distance = %d,ekf[a].tht8 = %d\n", ekf[a].distance, ekf[a].tht8);

		/**************测试**********************/
		printf("本机偏转角 =%d ,", ekf[a].tht8);
		printf("Z_x=%f ,Z_y= %f\n", ekf[a].Zvalue[0], ekf[a].Zvalue[1]);
		printf("X x=%f ,X y= %f\n", ekf[a].Xvalue[0], ekf[a].Xvalue[1]);
		printf("robot[%d]X_X=%f, X_y=%f, X_3=%f\n", robot_ekf2[a].number,
			   ekf[a].X_value[1], ekf[a].X_value[2]);
		//————————————————————————————————————————————————————————————

		ekf[a].X_->p = ekf[a].X_value; //先验估计值

		ekf[a].Fvalue[0] = cos((ekf[a].tht8 + 900.) * pi / 1800);
		ekf[a].Fvalue[1] = sin((ekf[a].tht8 + 900.) * pi / 1800);
		ekf[a].Fvalue[2] = -ekf[a].distance1 * 0.1 * sin((900. - ekf[a].Xvalue[2]) * pi / 1800);
		ekf[a].Fvalue[3] = -sin((ekf[a].tht8 + 900.) * pi / 1800);
		ekf[a].Fvalue[4] = cos((ekf[a].tht8 + 900.) * pi / 1800);
		ekf[a].Fvalue[5] = ekf[a].distance1 * 0.1 * cos((900. - ekf[a].Xvalue[2]) * pi / 1800);
		ekf[a].Fvalue[6] = 0;
		ekf[a].Fvalue[7] = 0;
		ekf[a].Fvalue[8] = 1;
		ekf[a].F->p = ekf[a].Fvalue;

		//printf("与%d号机器人之间的角度为 %f\n", robot_ekf2[a].number, ekf[a].Xvalue[2]);
		/******************************************************/

		errorID = matrix_transpose(ekf[a].F, ekf[a].F_);
		errorID = matrix_multiplication(ekf[a].F, ekf[a].P_, ekf[a].F);
		errorID = matrix_multiplication(ekf[a].F, ekf[a].F_, ekf[a].P_);
		errorID = matrix_add(ekf[a].P_, ekf[a].Q, ekf[a].P_); //协方差矩阵

		errorID = matrix_multiplication(ekf[a].H, ekf[a].X_, ekf[a].preZ); //观测预测值

		errorID = matrix_multiplication(ekf[a].P_, ekf[a].H, ekf[a].K);
		errorID = matrix_multiplication(ekf[a].H, ekf[a].P_, ekf[a].K_);
		errorID = matrix_multiplication(ekf[a].K_, ekf[a].H, ekf[a].K_);
		errorID = matrix_add(ekf[a].K_, ekf[a].R, ekf[a].K_);
		errorID = matrix_inverse(ekf[a].K_, ekf[a].K_);
		errorID = matrix_multiplication(ekf[a].K, ekf[a].K_, ekf[a].K); //协方差矩阵K

		errorID = matrix_subtraction(ekf[a].Z, ekf[a].preZ, ekf[a].Z);
		errorID = matrix_multiplication(ekf[a].K, ekf[a].Z, ekf[a].Z);
		errorID = matrix_add(ekf[a].preZ, ekf[a].Z, ekf[a].X);

		errorID = matrix_multiplication(ekf[a].Eye, ekf[a].K, ekf[a].P);
		errorID = matrix_multiplication(ekf[a].P, ekf[a].P_, ekf[a].P);
		//printf("%d\n", errorID);
		printf("\n");
		gettimeofday(&ekf_timeval, NULL);
		ekf_gettime.time_flag = ekf_timeval.tv_sec * 1000000 + ekf_timeval.tv_usec;
		ekf_gettime.PreMs = ekf_gettime.time_flag - ekf_gettime.start_flag;
		if (ekf_gettime.PreMs >= ekf_ts)
		{
			microseconds_sleep(1);
		}
		else
		{
			microseconds_sleep(ekf_ts - ekf_gettime.PreMs); //微调
		}
	}
}

int robotRuning()
{
	int a = 0;
	seconds_sleep(2);
	int file_fd = 0;
	while (1)
	{
		for (a = 1; a < 3; a++)
		{
			if (robot_ekf[a].number) //标志位（在线程 ekf_algorithm 中会被置位）
			{
				//目标机器人的编号       //目标机器人在本机坐标系下的x值的后验估计（最优估计值）				  //目标机器人在本机坐标系下的x值的先验估计								  //目标机器人在本机坐标系下的x值的观测值
				file_write_ekf(file_fd, robot_ekf2[a].number, (int)PickInMat(ekf[a].X, 1, 1), (int)PickInMat(ekf[a].X, 2, 1), (int)PickInMat(ekf[a].X_, 1, 1), (int)PickInMat(ekf[a].X_, 2, 1), (int)PickInMat(ekf[a].Z, 1, 1), (int)PickInMat(ekf[a].Z, 2, 1));
				//目标机器人在本机坐标系下的y值的后验估计（最优估计值）					//目标机器人在本机坐标系下的y值的先验估计								//目标机器人在本机坐标系下的y值的观测值
				robot_ekf[a].number = 0; //这个变量不需要再次加锁，因为他的时间与ekf错开
			}
			else
				file_write_xy(file_fd, a, 0);
		}
		milliseconds_sleep(1000);
	}
	return 0;
}
int robotRuning1()
{
	/* float sin_count = 0.001;
	int time_count = 0;
	while (1)
	{
		for (time_count = 0; time_count < 100; time_count++)
		{
			//dlta_d = abs(100);
			//dlta_a = 100 * sin(sin_count) / pow((1 + cos(sin_count) * cos(sin_count)), 1.5);
			//sin_count = sin_count + 0.02;
			//dlta_a = 100;
			dlta_d = 100;
			pthread_rwlock_rdlock(&rwlock);
			frame_send.data[1] = (int)dlta_d >> 8;
			frame_send.data[2] = (int)dlta_d;
			frame_send.data[3] = (int)dlta_a >> 8;
			frame_send.data[4] = (int)dlta_a;
			frame_send.data[5] = 0;
			nbytes = write(can_fd, &frame_send, sizeof(frame_send)); //通过CAN控制stm32电机，令机器人移动
			pthread_rwlock_unlock(&rwlock);							 //解锁，如果其他的线程需要对速度进行改变，则需要锁
			milliseconds_sleep(100);
		}
		//dlta_a = 0;
		dlta_d = 0;
		pthread_rwlock_rdlock(&rwlock);
		frame_send.data[1] = (int)dlta_d >> 8;
		frame_send.data[2] = (int)dlta_d;
		frame_send.data[3] = (int)dlta_a >> 8;
		frame_send.data[4] = (int)dlta_a;
		frame_send.data[5] = 0;
		nbytes = write(can_fd, &frame_send, sizeof(frame_send)); //通过CAN控制stm32电机，令机器人移动
		pthread_rwlock_unlock(&rwlock);							 //解锁，如果其他的线程需要对速度进行改变，则需要锁
		milliseconds_sleep(100);
		for (time_count = 0; time_count < 100; time_count++)
		{

			//	dlta_d = -100;
			//	dlta_a = -100 * sin(sin_count) / pow((1 + cos(sin_count) * cos(sin_count)), 1.5);
			//	sin_count = sin_count - 0.02;
			//dlta_a = -100;
			dlta_d = -100;
			pthread_rwlock_rdlock(&rwlock);
			frame_send.data[1] = (int)dlta_d >> 8;
			frame_send.data[2] = (int)dlta_d;
			frame_send.data[3] = (int)dlta_a >> 8;
			frame_send.data[4] = (int)dlta_a;
			frame_send.data[5] = 0;
			nbytes = write(can_fd, &frame_send, sizeof(frame_send)); //通过CAN控制stm32电机，令机器人移动
			pthread_rwlock_unlock(&rwlock);							 //解锁，如果其他的线程需要对速度进行改变，则需要锁

			milliseconds_sleep(100);
		}
	} */
	//dlta_a = 100;
	dlta_d = 100;

	pthread_rwlock_rdlock(&rwlock);
	frame_send.data[1] = (int)dlta_d >> 8;
	frame_send.data[2] = (int)dlta_d;
	frame_send.data[3] = (int)dlta_a >> 8;
	frame_send.data[4] = (int)dlta_a;
	frame_send.data[5] = 0;
	nbytes = write(can_fd, &frame_send, sizeof(frame_send)); //通过CAN控制stm32电机，令机器人移动
	pthread_rwlock_unlock(&rwlock);
	seconds_sleep(5);
	dlta_a = 0;
	dlta_d = 0;

	pthread_rwlock_rdlock(&rwlock);
	frame_send.data[1] = (int)dlta_d >> 8;
	frame_send.data[2] = (int)dlta_d;
	frame_send.data[3] = (int)dlta_a >> 8;
	frame_send.data[4] = (int)dlta_a;
	frame_send.data[5] = 0;
	nbytes = write(can_fd, &frame_send, sizeof(frame_send)); //通过CAN控制stm32电机，令机器人移动
	pthread_rwlock_unlock(&rwlock);
	return 0;
}

void *ekf_multi(void *arg)
{
	//dlta_a = 100;
	char *local_ip = NULL;
	local_ip = GetLocalIp();
	sleep(20);

	// pthread_t robotRun1;
	// pthread_create(&robotRun1, NULL, robotRuning1, NULL); //该线程用于控制机器人的运动
	//}
	//pthread_t robotRun;
	//	pthread_create(&robotRun, NULL, robotRuning1, NULL); //该线程用于记录
	if (robot_end[1].count == 1)
	{
		printf("no robot connect\n,%d", robot_end[1].count);
	}
	if (robot_end[1].count == 2)
	{
		int i = 1;
		pthread_t tTreadEkf;
		pthread_create(&tTreadEkf, NULL, ekf_algorithm, &i);
		pthread_join(tTreadEkf, NULL);
	}
	if (robot_end[1].count == 3)
	{
		int i = 1;
		int j = 2;
		pthread_t tTreadEkf;
		pthread_t tTreadEkf2;
		pthread_create(&tTreadEkf, NULL, ekf_algorithm, &i);
		milliseconds_sleep(500);
		pthread_create(&tTreadEkf2, NULL, ekf_algorithm, &j);
		pthread_join(tTreadEkf, NULL);
		pthread_join(tTreadEkf2, NULL);
	}
	//pthread_join(robotRun, NULL);
	return arg;
}
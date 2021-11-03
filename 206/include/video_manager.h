
#ifndef _VIDEO_MANAGER_H
#define _VIDEO_MANAGER_H

#include <config.h>
#include <pic_operation.h>
#include <linux/videodev2.h>
#include <pthread.h>


#define NB_BUFFER 2

struct VideoDevice;
struct VideoOpr;
typedef struct VideoDevice T_VideoDevice, *PT_VideoDevice;
typedef struct VideoOpr T_VideoOpr, *PT_VideoOpr;

//video设备结构体
struct VideoDevice {
    int iFd;             //摄像头文件描述符
    int iPixelFormat;    //支持的设备格式（V4L2_PIX_FMT_MJPEG）
    int iWidth;          //摄像头输出指定格式（写死800）
    int iHeight;         //摄像头输出指定格式（写死600）

    int iVideoBufCnt;    //帧缓冲数量（设置为2）
    int iVideoBufMaxLen; //一个帧缓冲区（即内核空间的视频缓冲区）多大： iWidth*iHeight=800*600（字节）
    int iVideoBufCurIndex; //当前操作的 帧缓冲区（即内核空间的视频缓冲区） 的索引编号   （0或1）
    unsigned char *pucVideBuf[NB_BUFFER]; //映射区（内核空间的视频缓冲区）的内存起始地址，保存起来供应用程序访问。【因为申请了两块内核缓冲区（帧缓冲数量为2），故此处存两个起始地址】

    /* 函数 */
    PT_VideoOpr ptOPr;  //此成员指向全局变量结构体 g_tV4l2VideoOpr（即把全局变量结构体 g_tV4l2VideoOpr 的地址赋给了此成员）
};

//存储摄像头读取到的数据和格式
typedef struct VideoBuf {
    T_PixelDatas tPixelDatas;   /* 图片的象素数据 */
    int iPixelFormat;           /* 格式：V4L2_PIX_FMT_MJPEG（未转换） 或 V4L2_PIX_FMT_RGB565（转换后） */
}T_VideoBuf, *PT_VideoBuf;

//操作函数结构体,用于操作摄像头（这个操作函数结构体放在上面那个VideoDevice结构体中，作为成员）
struct VideoOpr {
    char *name;
	
    int (*InitDevice)(char *strDevName, PT_VideoDevice ptVideoDevice);
    int (*ExitDevice)(PT_VideoDevice ptVideoDevice);
    int (*GetFrame)(PT_VideoDevice ptVideoDevice, PT_VideoBuf ptVideoBuf);
    int (*GetFormat)(PT_VideoDevice ptVideoDevice);
    int (*PutFrame)(PT_VideoDevice ptVideoDevice, PT_VideoBuf ptVideoBuf);
    int (*StartDevice)(PT_VideoDevice ptVideoDevice);
    int (*StopDevice)(PT_VideoDevice ptVideoDevice);
    struct VideoOpr *ptNext;
};


int VideoDeviceInit(char *strDevName, PT_VideoDevice ptVideoDevice);
int V4l2Init(void);
int RegisterVideoOpr(PT_VideoOpr ptVideoOpr);
int VideoInit(void);

#endif /* _VIDEO_MANAGER_H */


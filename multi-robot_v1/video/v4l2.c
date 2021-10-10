#include <config.h>
#include <video_manager.h>
#include <disp_manager.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <string.h>
#include <unistd.h>

static int g_aiSupportedFormats[] = {V4L2_PIX_FMT_YUYV, V4L2_PIX_FMT_MJPEG, V4L2_PIX_FMT_RGB565};

static int V4l2GetFrameForReadWrite(PT_VideoDevice ptVideoDevice, PT_VideoBuf ptVideoBuf);
static int V4l2PutFrameForReadWrite(PT_VideoDevice ptVideoDevice, PT_VideoBuf ptVideoBuf);
static T_VideoOpr g_tV4l2VideoOpr;

static int isSupportThisFormat(int iPixelFormat)
{
    int i;
    for (i = 0; i < sizeof(g_aiSupportedFormats)/sizeof(g_aiSupportedFormats[0]); i++)
    {
        if (g_aiSupportedFormats[i] == iPixelFormat)
            return 1;
    }
    return 0;
}

/* 参考 luvcview */

/* open
 * VIDIOC_QUERYCAP 确定它是否视频捕捉设备,支持哪种接口(streaming/read,write)
 * VIDIOC_ENUM_FMT 查询支持哪种格式
 * VIDIOC_S_FMT    设置摄像头使用哪种格式
 * VIDIOC_REQBUFS  申请buffer
 对于 streaming:
 * VIDIOC_QUERYBUF 确定每一个buffer的信息 并且 mmap
 * VIDIOC_QBUF     放入队列
 * VIDIOC_STREAMON 启动设备
 * poll            等待有数据
 * VIDIOC_DQBUF    从队列中取出
 * 处理....
 * VIDIOC_QBUF     放入队列
 * ....
 对于read,write:
    read
    处理....
    read
 * VIDIOC_STREAMOFF 停止设备
 *
 */

//形参 ptVideoDevice：传出参数
static int V4l2InitDevice(char *strDevName, PT_VideoDevice ptVideoDevice)
{
    int i;
    int iFd;
    int iError;
    struct v4l2_capability tV4l2Cap;
	struct v4l2_fmtdesc tFmtDesc;
    struct v4l2_format  tV4l2Fmt;
    struct v4l2_requestbuffers tV4l2ReqBuffs;
    struct v4l2_buffer tV4l2Buf;

    int iLcdWidth;
    int iLcdHeigt;
    int iLcdBpp;

    iFd = open(strDevName, O_RDWR); //打开设备节点
    if (iFd < 0)
    {
        DBG_PRINTF("can not open %s\n", strDevName);
        return -1;
    }
    ptVideoDevice->iFd = iFd;

    /* 调用ioctl函数，通过驱动查询设备类型,传出到结构体tV4l2Cap中，并通过结构体成员capabilities确定,
       是否支持V4L2_CAP_VIDEO_CAPTURE即视频捕捉设备，是否支持V4L2_CAP_STREAMING和V4L2_CAP_READWRITE接口的数据读取方式 */
    iError = ioctl(iFd, VIDIOC_QUERYCAP, &tV4l2Cap);
    memset(&tV4l2Cap, 0, sizeof(struct v4l2_capability));
    iError = ioctl(iFd, VIDIOC_QUERYCAP, &tV4l2Cap);
    if (iError) {
    	DBG_PRINTF("Error opening device %s: unable to query device.\n", strDevName);
    	goto err_exit;
    }

    if (!(tV4l2Cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    {
    	DBG_PRINTF("%s is not a video capture device\n", strDevName);
        goto err_exit;
    }

	if (tV4l2Cap.capabilities & V4L2_CAP_STREAMING) {
	    DBG_PRINTF("%s supports streaming i/o\n", strDevName);
	}
    
	if (tV4l2Cap.capabilities & V4L2_CAP_READWRITE) {
	    DBG_PRINTF("%s supports read i/o\n", strDevName);
	}

	memset(&tFmtDesc, 0, sizeof(tFmtDesc));
	tFmtDesc.index = 0;
	tFmtDesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    /* 调用ioctl函数，通过驱动以遍历的方式查看当前驱动支持的所有设备格式，并与当前可支持的所有格式相比较，
       如果发现其中有可支持的格式，则可说明此驱动支持应用中所需的格式。 */
	while ((iError = ioctl(iFd, VIDIOC_ENUM_FMT, &tFmtDesc)) == 0) {
        if (isSupportThisFormat(tFmtDesc.pixelformat))
        {
            ptVideoDevice->iPixelFormat = tFmtDesc.pixelformat;
            break;
        }
		tFmtDesc.index++;
	}

    if (!ptVideoDevice->iPixelFormat)
    {
    	DBG_PRINTF("can not support the format of this device\n");
        goto err_exit;        
    }

    
    /* set format in */
    GetDispResolution(&iLcdWidth, &iLcdHeigt, &iLcdBpp); //获得所使用的显示设备（显示屏）的分辨率和BPP
    iLcdWidth=800;iLcdHeigt=600; //写死
    memset(&tV4l2Fmt, 0, sizeof(struct v4l2_format));
    tV4l2Fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    tV4l2Fmt.fmt.pix.pixelformat = ptVideoDevice->iPixelFormat;
    tV4l2Fmt.fmt.pix.width       = iLcdWidth;
    tV4l2Fmt.fmt.pix.height      = iLcdHeigt;
    tV4l2Fmt.fmt.pix.field       = V4L2_FIELD_ANY;

    /* 调用ioctl函数，通过驱动设置摄像头输出指定的格式,这个格式中使用的格式为上面函数得到的可支持的格式，
       使用的分辨率为LCD显示设备的分辨率，
       注意：如果驱动程序发现某些参数无法支持（如分辨率），它会调整这些参数并且返回应用程序，这些实际设置的分辨率的返回值也将它保存起来。 */
    iError = ioctl(iFd, VIDIOC_S_FMT, &tV4l2Fmt);
    if (iError) 
    {
    	DBG_PRINTF("Unable to set format\n");
        goto err_exit;        
    }
    ptVideoDevice->iWidth  = tV4l2Fmt.fmt.pix.width;
    ptVideoDevice->iHeight = tV4l2Fmt.fmt.pix.height;

    /* request buffers */
    memset(&tV4l2ReqBuffs, 0, sizeof(struct v4l2_requestbuffers));
    /*分配内核缓冲区,包含帧缓冲区的数量*/
    tV4l2ReqBuffs.count = NB_BUFFER;                  //帧缓冲数量
    tV4l2ReqBuffs.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; //视频捕获缓冲区类型
    tV4l2ReqBuffs.memory = V4L2_MEMORY_MMAP;          //内存映射方式
    /* 调用ioctl函数,来请求分配缓冲区 */
    iError = ioctl(iFd, VIDIOC_REQBUFS, &tV4l2ReqBuffs);
    if (iError) 
    {
    	DBG_PRINTF("Unable to allocate buffers.\n");
        goto err_exit;        
    }
    
    ptVideoDevice->iVideoBufCnt = tV4l2ReqBuffs.count;

    /* 如果是streaming接口方式读取数据，则接下来必须进行mmap方式，应用层才能读取数据，
       且映射时针对每一个具体的 buffer，根据实际申请的缓冲区映射，并将每个映射后的地址保存起来。
       （如果我们是read,write调用，则不必mmap就可直接读取数据） */
    if (tV4l2Cap.capabilities & V4L2_CAP_STREAMING) //使用 stream I/O，进此if语句
    {
        /* map the buffers */
        for (i = 0; i < ptVideoDevice->iVideoBufCnt; i++) //i取0、1
        {
        	memset(&tV4l2Buf, 0, sizeof(struct v4l2_buffer));
        	tV4l2Buf.index = i; //内核空间的视频缓冲区（内核缓冲区）的索引编号
        	tV4l2Buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        	tV4l2Buf.memory = V4L2_MEMORY_MMAP;
            /* 获取内核空间的视频缓冲区（内核缓冲区）信息 */
            iError = ioctl(iFd, VIDIOC_QUERYBUF, &tV4l2Buf);
        	if (iError) 
            {
        	    DBG_PRINTF("Unable to query buffer.\n");
        	    goto err_exit;
        	}

            ptVideoDevice->iVideoBufMaxLen = tV4l2Buf.length; //800*600（字节）
            /* 然后调用函数mmap把内核空间缓冲区地址映射到用户空间，这样应用程序才能够访问位于内核空间的视频缓冲区 
               mmap函数映射成功则返回映射区的内存起始地址 */
        	ptVideoDevice->pucVideBuf[i] = mmap(0 /* start anywhere */ ,
        			  tV4l2Buf.length, PROT_READ, MAP_SHARED, iFd,
        			  tV4l2Buf.m.offset);

        	if (ptVideoDevice->pucVideBuf[i] == MAP_FAILED) 
            {
        	    DBG_PRINTF("Unable to map buffer\n");
        	    goto err_exit;
        	}
        }        

        /* Queue the buffers. */
        for (i = 0; i < ptVideoDevice->iVideoBufCnt; i++) //i取0、1
        {
        	memset(&tV4l2Buf, 0, sizeof(struct v4l2_buffer));
        	tV4l2Buf.index = i;
        	tV4l2Buf.type  = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        	tV4l2Buf.memory = V4L2_MEMORY_MMAP;
            /* 将内核空间的视频缓冲区（内核缓冲区）放入队列
               for循环两次，往队列中放入两块 */
        	iError = ioctl(iFd, VIDIOC_QBUF, &tV4l2Buf);
        	if (iError)
            {
        	    DBG_PRINTF("Unable to queue buffer.\n");
        	    goto err_exit;
        	}
        }
        
    }
    else if (tV4l2Cap.capabilities & V4L2_CAP_READWRITE)
    {
        g_tV4l2VideoOpr.GetFrame = V4l2GetFrameForReadWrite;
        g_tV4l2VideoOpr.PutFrame = V4l2PutFrameForReadWrite;
        
        /* read(fd, buf, size) */
        ptVideoDevice->iVideoBufCnt  = 1;
        /* 在这个程序所能支持的格式里, 一个象素最多只需要4字节 */
        ptVideoDevice->iVideoBufMaxLen = ptVideoDevice->iWidth * ptVideoDevice->iHeight * 4;
        ptVideoDevice->pucVideBuf[0] = malloc(ptVideoDevice->iVideoBufMaxLen);
    }

    ptVideoDevice->ptOPr = &g_tV4l2VideoOpr;
    return 0;
    
err_exit:    
    close(iFd);
    return -1;    
}

static int V4l2ExitDevice(PT_VideoDevice ptVideoDevice)
{
    int i;
    for (i = 0; i < ptVideoDevice->iVideoBufCnt; i++)
    {
        if (ptVideoDevice->pucVideBuf[i])
        {
            munmap(ptVideoDevice->pucVideBuf[i], ptVideoDevice->iVideoBufMaxLen);
            ptVideoDevice->pucVideBuf[i] = NULL;
        }
    }
        
    close(ptVideoDevice->iFd);
    return 0;
}
    
static int V4l2GetFrameForStreaming(PT_VideoDevice ptVideoDevice, PT_VideoBuf ptVideoBuf)
{
    struct pollfd tFds[1];
    int iRet;
    struct v4l2_buffer tV4l2Buf;

    /* 思路：
       poll()//通过摄像头文件节点来监测设备数据。
       有数据后，将数据放入缓冲区，并调用ioctl(VIDIOC_DQBUF)将缓冲区从队列中取出来 
    */

    /* poll */
    tFds[0].fd     = ptVideoDevice->iFd; //摄像头文件描述符
    tFds[0].events = POLLIN;
    iRet = poll(tFds, 1, -1); //第二个参数：用来指定第一个参数数组元素个数。第三个参数：指定等待的毫秒数（-1表示永远等待，直至事件发生）
    if (iRet <= 0)
    {
        DBG_PRINTF("poll error!\n");
        return -1;
    }
    
    /* VIDIOC_DQBUF */
    memset(&tV4l2Buf, 0, sizeof(struct v4l2_buffer));
    tV4l2Buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    tV4l2Buf.memory = V4L2_MEMORY_MMAP;
    iRet = ioctl(ptVideoDevice->iFd, VIDIOC_DQBUF, &tV4l2Buf); //将缓冲区从队列中取出来
    if (iRet < 0) 
    {
    	DBG_PRINTF("Unable to dequeue buffer.\n");
    	return -1;
    }
    ptVideoDevice->iVideoBufCurIndex = tV4l2Buf.index;

    ptVideoBuf->iPixelFormat        = ptVideoDevice->iPixelFormat; //V4L2_PIX_FMT_MJPEG
    ptVideoBuf->tPixelDatas.iWidth  = ptVideoDevice->iWidth; //800
    ptVideoBuf->tPixelDatas.iHeight = ptVideoDevice->iHeight; //600
    ptVideoBuf->tPixelDatas.iBpp    = (ptVideoDevice->iPixelFormat == V4L2_PIX_FMT_YUYV) ? 16 : \
                                        (ptVideoDevice->iPixelFormat == V4L2_PIX_FMT_MJPEG) ? 0 :  \
                                        (ptVideoDevice->iPixelFormat == V4L2_PIX_FMT_RGB565) ? 16 :  \
                                        0; //0
    ptVideoBuf->tPixelDatas.iLineBytes    = ptVideoDevice->iWidth * ptVideoBuf->tPixelDatas.iBpp / 8; //800 * 0/8 = 0
    ptVideoBuf->tPixelDatas.iTotalBytes   = tV4l2Buf.bytesused;
    ptVideoBuf->tPixelDatas.aucPixelDatas = ptVideoDevice->pucVideBuf[tV4l2Buf.index];  
/*
 FILE *fp = fopen("frame1.jpg", "wb+");
        if (fp < 0)
        {
           DBG_PRINTF("open frame data file failed\n");
            return -1;
        }

        fwrite(ptVideoDevice->pucVideBuf[tV4l2Buf.index], 1, tV4l2Buf.length, fp);
        fclose(fp);
  */
    return 0;
}


static int V4l2PutFrameForStreaming(PT_VideoDevice ptVideoDevice, PT_VideoBuf ptVideoBuf)
{
    /* VIDIOC_QBUF */
    struct v4l2_buffer tV4l2Buf;
    int iError;
    
	memset(&tV4l2Buf, 0, sizeof(struct v4l2_buffer));
	tV4l2Buf.index  = ptVideoDevice->iVideoBufCurIndex;
	tV4l2Buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	tV4l2Buf.memory = V4L2_MEMORY_MMAP;
	iError = ioctl(ptVideoDevice->iFd, VIDIOC_QBUF, &tV4l2Buf);
	if (iError) 
    {
	    DBG_PRINTF("Unable to queue buffer.\n");
	    return -1;
	}
    return 0;
}

static int V4l2GetFrameForReadWrite(PT_VideoDevice ptVideoDevice, PT_VideoBuf ptVideoBuf)
{
    int iRet;

    iRet = read(ptVideoDevice->iFd, ptVideoDevice->pucVideBuf[0], ptVideoDevice->iVideoBufMaxLen);
    if (iRet <= 0)
    {
        return -1;
    }
    
    ptVideoBuf->iPixelFormat        = ptVideoDevice->iPixelFormat;
    ptVideoBuf->tPixelDatas.iWidth  = ptVideoDevice->iWidth;
    ptVideoBuf->tPixelDatas.iHeight = ptVideoDevice->iHeight;
    ptVideoBuf->tPixelDatas.iBpp    = (ptVideoDevice->iPixelFormat == V4L2_PIX_FMT_YUYV) ? 16 : \
                                        (ptVideoDevice->iPixelFormat == V4L2_PIX_FMT_MJPEG) ? 0 :  \
                                        (ptVideoDevice->iPixelFormat == V4L2_PIX_FMT_RGB565)? 16 : \
                                          0;
    ptVideoBuf->tPixelDatas.iLineBytes    = ptVideoDevice->iWidth * ptVideoBuf->tPixelDatas.iBpp / 8;
    ptVideoBuf->tPixelDatas.iTotalBytes   = iRet;
    ptVideoBuf->tPixelDatas.aucPixelDatas = ptVideoDevice->pucVideBuf[0];    
    
    return 0;
}


static int V4l2PutFrameForReadWrite(PT_VideoDevice ptVideoDevice, PT_VideoBuf ptVideoBuf)
{
    return 0;
}

static int V4l2StartDevice(PT_VideoDevice ptVideoDevice)
{
    int iType = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    int iError;

    iError = ioctl(ptVideoDevice->iFd, VIDIOC_STREAMON, &iType); //启动摄像头
    if (iError) 
    {
    	DBG_PRINTF("Unable to start capture.\n");
    	return -1;
    }
    return 0;
}
    
static int V4l2StopDevice(PT_VideoDevice ptVideoDevice)
{
    int iType = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    int iError;

    iError = ioctl(ptVideoDevice->iFd, VIDIOC_STREAMOFF, &iType);
    if (iError) 
    {
    	DBG_PRINTF("Unable to stop capture.\n");
    	return -1;
    }
    return 0;
}

static int V4l2GetFormat(PT_VideoDevice ptVideoDevice)
{
    return ptVideoDevice->iPixelFormat;
}


/* 构造一个VideoOpr结构体 */
static T_VideoOpr g_tV4l2VideoOpr = {
    .name        = "v4l2",
    .InitDevice  = V4l2InitDevice,
    .ExitDevice  = V4l2ExitDevice,
    .GetFormat   = V4l2GetFormat,
    .GetFrame    = V4l2GetFrameForStreaming,
    .PutFrame    = V4l2PutFrameForStreaming,
    .StartDevice = V4l2StartDevice,
    .StopDevice  = V4l2StopDevice,
};

/* 注册这个结构体 */
int V4l2Init(void)
{
    return RegisterVideoOpr(&g_tV4l2VideoOpr);
}


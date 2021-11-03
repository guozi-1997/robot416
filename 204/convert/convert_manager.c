
#include <config.h>
#include <convert_manager.h>
#include <string.h>

static PT_VideoConvert g_ptVideoConvertHead = NULL;

/**********************************************************************
 * 函数名称： RegisterVideoConvert
 * 功能描述： 注册"字体模块", 所谓字体模块就是取出字符位图的方法
 * 输入参数： ptVideoConvert - 一个结构体,内含"取出字符位图"的操作函数
 * 输出参数： 无
 * 返 回 值： 0 - 成功, 其他值 - 失败
 * 修改日期        版本号     修改人	      修改内容
 * -----------------------------------------------
 * 2013/02/08	     V1.0	  韦东山	      创建
 ***********************************************************************/
int RegisterVideoConvert(PT_VideoConvert ptVideoConvert)
{
	PT_VideoConvert ptTmp;

	if (!g_ptVideoConvertHead) //只有一个摄像头设备
	{
		g_ptVideoConvertHead   = ptVideoConvert;
		ptVideoConvert->ptNext = NULL;
	}
	else //两个及以上摄像头设备
	{
		ptTmp = g_ptVideoConvertHead;
		while (ptTmp->ptNext)
		{
			ptTmp = ptTmp->ptNext;
		}
		ptTmp->ptNext     = ptVideoConvert;
		ptVideoConvert->ptNext = NULL;
	}

	return 0;
}


/**********************************************************************
 * 函数名称： ShowVideoConvert
 * 功能描述： 显示本程序能支持的"字体模块"
 * 输入参数： 无
 * 输出参数： 无
 * 返 回 值： 无
 * 修改日期        版本号     修改人	      修改内容
 * -----------------------------------------------
 * 2013/02/08	     V1.0	  韦东山	      创建
 ***********************************************************************/
void ShowVideoConvert(void)
{
	int i = 0;
	PT_VideoConvert ptTmp = g_ptVideoConvertHead;

	while (ptTmp)
	{
		printf("%02d %s\n", i++, ptTmp->name);
		ptTmp = ptTmp->ptNext;
	}
}

/**********************************************************************
 * 函数名称： GetVideoConvert
 * 功能描述： 根据名字取出指定的"字体模块"
 * 输入参数： pcName - 名字
 * 输出参数： 无
 * 返 回 值： NULL   - 失败,没有指定的模块, 
 *            非NULL - 字体模块的PT_VideoConvert结构体指针
 * 修改日期        版本号     修改人	      修改内容
 * -----------------------------------------------
 * 2013/02/08	     V1.0	  韦东山	      创建
 ***********************************************************************/
PT_VideoConvert GetVideoConvert(char *pcName)
{
	PT_VideoConvert ptTmp = g_ptVideoConvertHead;
	
	while (ptTmp)
	{
		if (strcmp(ptTmp->name, pcName) == 0)
		{
			return ptTmp;
		}
		ptTmp = ptTmp->ptNext;
	}
	return NULL;
}

PT_VideoConvert GetVideoConvertForFormats(int iPixelFormatIn, int iPixelFormatOut)
{
	PT_VideoConvert ptTmp = g_ptVideoConvertHead;
	
	while (ptTmp)
	{
        if (ptTmp->isSupport(iPixelFormatIn, iPixelFormatOut)) //即执行函数 isSupportMjpeg2Rgb(iPixelFormatIn, iPixelFormatOut)
        {
            return ptTmp;
        }
		ptTmp = ptTmp->ptNext;
	}
	return NULL;
}


/**********************************************************************
 * 函数名称： FontsInit
 * 功能描述： 调用各个字体模块的初始化函数
 * 输入参数： 无
 * 输出参数： 无
 * 返 回 值： 0 - 成功, 其他值 - 失败
 * 修改日期        版本号     修改人	      修改内容
 * -----------------------------------------------
 * 2013/02/08	     V1.0	  韦东山	      创建
 ***********************************************************************/
int VideoConvertInit(void)
{
	int iError;

    iError = Yuv2RgbInit();
    iError |= Mjpeg2RgbInit();
    iError |= Rgb2RgbInit();

	return iError;
}






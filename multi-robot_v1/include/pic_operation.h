
#ifndef _PIC_OPERATION_H
#define _PIC_OPERATION_H

/* 图片的象素数据 */
typedef struct PixelDatas
{
	int iWidth;					  /* 宽度: 一行有多少个象素 */
	int iHeight;				  /* 高度: 一列有多少个象素 */
	int iBpp;					  /* 深度：一个象素用多少位来表示 */
	/* 注：每种格式下像素的大小：对应YUV格式，像素位宽为16位；对于MJPEG格式，没有像素位宽而言；对于RGB565格式，像素位宽为16位 */
	int iLineBytes;				  /* 一行数据有多少字节 */
	int iTotalBytes;			  /* 所有字节数 */
	unsigned char *aucPixelDatas; /* 象素数据存储的地方 （存两帧图像数据）*/
} T_PixelDatas, *PT_PixelDatas;

#endif /* _PIC_OPERATION_H */

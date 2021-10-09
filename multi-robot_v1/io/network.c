#include "io.h"

void send_server_init()
{

	tSocketServerAddr.sin_family = AF_INET;
	tSocketServerAddr.sin_port = htons(SERVER_PORT); /* host to net, short */
}

/* 
*  形参：fd是传出参数(客户端socket连接描述符)，dis是传入参数(欲连接服务器的IP) 
*  返回值：连接（服务器）成功，返回1；否则返回0
*/
int connect_to(int *fd, unsigned char *dis)
{

	int iRet;
	unsigned char ucSendBuf[1000];
	int iSendLen;

	*fd = socket(AF_INET, SOCK_STREAM, 0);
	
	//将一个字符串IP地址转换为一个网络字节序IP地址，成功返回非0
	if (0 == inet_aton(dis, &tSocketServerAddr.sin_addr))
	{
		printf("invalid server_ip\n");
	}
	memset(tSocketServerAddr.sin_zero, 0, 8);
	iRet = connect(*fd, (const struct sockaddr *)&tSocketServerAddr, sizeof(struct sockaddr));
	//注：目前三台机器人，故网络中只有 192.168.0.201、204、206 是有效IP地址
	if (-1 == iRet)
	{
		printf("connect error for  %s!\n", dis);
		return 0;
	}
	else
	{
		printf("connect success for  %s!\n", dis);
		return 1;
	}
}

char *GetLocalIp(void)
{
	int MAXINTERFACES = 16;
	char *ip = NULL;
	int fd, intrface, retn = 0;
	struct ifreq buf[MAXINTERFACES];	//interface request
	struct ifconf ifc;	//interface config

	if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) >= 0)
	{
		ifc.ifc_len = sizeof(buf);
		ifc.ifc_buf = (caddr_t)buf;
		if (!ioctl(fd, SIOCGIFCONF, (char *)&ifc))	//通过 SIOCGIFCONF 操作获取系统中所有的网络接口
		{
			intrface = ifc.ifc_len / sizeof(struct ifreq);

			while (intrface-- > 0)
			{
				if (!(ioctl(fd, SIOCGIFADDR, (char *)&buf[intrface])))	//通过 SIOCGIFADDR 操作获取指定网络接口的IPv4地址
				{
					ip = (inet_ntoa(((struct sockaddr_in *)(&buf[intrface].ifr_addr))->sin_addr));
					break;
				}
			}
		}
		close(fd);
		return ip;
	}
}

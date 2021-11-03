#include <io.h>

int can_init(void)
{
	system("ifconfig  can0 down");
	system("ip link set can0 type can bitrate 250000");
	system("ifconfig  can0 up");

	 /*建立 SocketCAN 套接字，设置为原始套接字，原始CAN协议 */
	s = socket(PF_CAN,SOCK_RAW,CAN_RAW);
	
	/*以下是对CAN接口进行初始化，如设置CAN接口名，即当我们用ifconfig命令时显示的名字 */
	strcpy((char *)(ifr.ifr_name),"can0");
	ioctl(s,SIOCGIFINDEX,&ifr);	//指定 can0 设备
			   
	 /*设置CAN协议 */
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	/*将套接字与 can0 绑定*/
	bind(s,(struct sockaddr*)&addr,sizeof(addr));

  return s;

}


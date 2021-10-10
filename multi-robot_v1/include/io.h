#ifndef _IO_H
#define _IO_H

#include <termios.h>
#include <global.h>

#include <sys/socket.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <signal.h>

//----得到ip 地址所需的头文件--------
#include <sys/ioctl.h>
#include <net/if.h>
#include <arpa/inet.h>

#define SERVER_PORT 8888
#define BACKLOG 10
//--------------------------RPLIDAR---------------------------
#define BUFFER_SIZE 4096
#define BUF_NODEDATA 5
#define PERCIRCLE_DATA 360
#define DEV_FD "/dev/ttyUSB0"
#define NEW_CIRCLE 0x01
#define CHECK_FLAG 0x01
#define min(a, b) (((a) < (b)) ? (a) : (b))

//--------------------------CAN---------------------------
#ifndef PF_CAN
#define PF_CAN 29
#endif

#ifndef AF_CAN
#define AF_CAN PF_CAN
#endif

int s;
unsigned long nbytes;
struct sockaddr_can addr;
struct ifreq ifr;
struct can_frame frame_send;

//--------get_can-----------
struct can_frame frame_rev;   //接收CAN报文
int addr_len;
socklen_t len;

int get_x;
int get_y;
int get_tht;

int dlta_d;
int dlta_a;

//----------------------------usart---------------------------------------
unsigned char ask_data[5];
unsigned char get_data[8];

//-------------------------network-------------------------------
struct receive_content
{
   int iSocketClient[10];  //（本机作为客户端）存客户端socket连接描述符
   int connet_flag[10];    //（本机作为客户端）存是否与服务器成功连接标志位
   int start_connect_flag[10];
   int connect_fd[10];
} rec_content;

struct sockaddr_in tSocketServerAddr;  //本机作为客户端时，存放欲连接服务器的地址结构
int set_lidaropt(int fd, int nSpeed, int nBits, char nEvent, int nStop);
//------------------------------------------------------------
#endif /* _IO_H */

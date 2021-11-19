#include <io.h>

/*
要点提示:
1. float和unsigned long具有相同的数据结构长度
2. union据类型里的数据存放在相同的物理空间
*/

/* 电子罗盘串口初始化 */
int set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio, oldtio;
    if (tcgetattr(fd, &oldtio) != 0)
    {
        perror("SetupSerial 1");
        return -1;
    }
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;
    /*数据位选择*/
    switch (nBits)
    {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    }
    /*设置奇偶校验位*/
    switch (nEvent)
    {
    case 'O':
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP);
        break;
    case 'E':
        newtio.c_iflag |= (INPCK | ISTRIP);
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        break;
    case 'N':
        newtio.c_cflag &= ~PARENB;
        break;
    }
    /*设置波特率*/
    switch (nSpeed)
    {
    case 2400:
        cfsetispeed(&newtio, B2400);
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    case 460800:
        cfsetispeed(&newtio, B460800);
        cfsetospeed(&newtio, B460800);
        break;
    case 921600:
        printf("B921600\n");
        cfsetispeed(&newtio, B921600);
        cfsetospeed(&newtio, B921600);
        break;
    default:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    }
    if (nStop == 1)
        newtio.c_cflag &= ~CSTOPB;
    else if (nStop == 2)
        newtio.c_cflag |= CSTOPB;
    newtio.c_cc[VTIME] = 150;
    newtio.c_cc[VMIN] = 0;
    tcflush(fd, TCIFLUSH);
    if ((tcsetattr(fd, TCSANOW, &newtio)) != 0)
    {
        perror("com set error");
        return -1;
    }
    //	printf("set done!\n\r");
    return 0;
}
/* set_lidaropt初始化 */
int set_lidaropt(int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio, oldtio;   // termios 的数据结构中包含了终端特性的完整描述
    if (tcgetattr(fd, &oldtio) != 0) // tcgetattr 函数用于获取终端属性
    {
        perror("Setup Serial 1");
        return -1;
    }
    bzero(&newtio, sizeof(newtio));
    //使能串口接收
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;

    //设置串口数据位
    switch (nBits)
    {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    }

    //设置奇偶校验位
    switch (nEvent)
    {
    case 'O':
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP);
        break;
    case 'E':
        newtio.c_iflag |= (INPCK | ISTRIP);
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        break;
    case 'N':
        newtio.c_cflag &= ~PARENB;
        break;
    }

    //设置串口波特率
    switch (nSpeed)
    {
    case 2400:
        cfsetispeed(&newtio, B2400); // cfsetispeed 函数用于设置输入速率
        cfsetospeed(&newtio, B2400); // cfsetospeed 函数用于设置输出速率
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    case 460800:
        cfsetispeed(&newtio, B460800);
        cfsetospeed(&newtio, B460800);
        break;
    default:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    }
    //设置停止位
    if (nStop == 1)
        newtio.c_cflag &= ~CSTOPB;
    else if (nStop == 2)
        newtio.c_cflag |= CSTOPB;

    newtio.c_iflag &= ~(IXON | IXOFF | IXANY); // no sw flow control
    // raw input mode
    newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    // raw output mode
    newtio.c_oflag &= ~OPOST;

    newtio.c_cc[VTIME] = 150; //重要
    newtio.c_cc[VMIN] = 0;    //返回的最小值  重要
    tcflush(fd, TCIFLUSH);    // tcflush 函数用于丢弃队列中尚未传送或接收的数据

    if ((tcsetattr(fd, TCSANOW, &newtio)) != 0) // tcsetattr 函数用于设置终端属性
    {
        perror("com set error");
        return -1;
    }

    int dtr_bit = TIOCM_DTR;

    ioctl(fd, TIOCMBIC, &dtr_bit);

    return 0;
}

//=====================================================================
//根据电子罗盘的数据手册来看，此处是 BCD码 转 十进制
int hextoshi(unsigned char hex1, unsigned char hex2, unsigned char hex3)
{
    int temp = 0, temp1 = 0, temp2 = 0, temp3 = 0;
    //-------------------------------------------------------------------
    if (hex2 < 0x10)
    {
        temp2 = hex2;
    }
    if (hex2 >= 0x10)
    {
        temp2 = hex2 - (hex2 * 6) / 16; //BCD码转十进制码的算法
    }
    //-------------------------------------------------------------------
    if (hex3 < 0x10)
    {
        temp3 = hex3;
    }
    if (hex3 >= 0x10)
    {
        temp3 = hex3 - (hex3 * 6) / 16;
    }
    if (hex1 < 0x10) //表示正角度
    {
        temp1 = hex1;
        temp = temp1 * 10000 + temp2 * 100 + temp3;
    }

    if (hex1 >= 0x10) //表示负角度
    {
        temp1 = hex1 & 0x0f;
        temp = -(temp1 * 10000 + temp2 * 100 + temp3);
    }

    //-------------------------------------------------------------------
    return temp;
}

//-----------------180-180-----------------------------------
int angle(int a)
{
    if (a > 18000)
        a = a - 36000;
    if (a < -18000)
        a = a + 36000;
    if (a == 18000 || a == -18000)
        a = 18000;
    return a;
}

int usart_init(void)
{

    int fd, read_num = 0;         // flag = 0;
    char *uart4 = "/dev/ttySAC1"; //CON4
    if ((fd = open(uart4, O_RDWR)) < 0)
    {
        printf("cannot open usart\n");
        return -1;
    }
    /* 波特率为 9600，数据位数为 8 位，无校验位，1位停止位     //即电子罗盘支持的通信协议格式*/
    set_opt(fd, 9600, 8, 'N', 1);

    return fd;
}

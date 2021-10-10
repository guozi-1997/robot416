#include <myfile.h>
#include "io.h"

int file_init(void)
{
	int fd = 0;
	int i = 0;
	char choose_ip[7][14] = {"192.168.0.201",
							 "192.168.0.202",
							 "192.168.0.203",
							 "192.168.0.204",
							 "192.168.0.205",
							 "192.168.0.206",
							 "192.168.0.207"};
	unsigned char files[7][15] = {"./record/1.txt",
								  "./record/2.txt",
								  "./record/3.txt",
								  "./record/4.txt",
								  "./record/5.txt",
								  "./record/6.txt",
								  "./record/7.txt"};
	unsigned char *choseip = NULL;
	unsigned char *filefd = NULL;

	char rmcmd[20]; //存放删除rm命令
	char chmodcmd[20]; //存放修改权限chmod命令
	strcpy(rmcmd, "rm ");
	strcpy(chmodcmd, "chmod 777 ");

	choseip = GetLocalIp();
	for (i = 0; i < 7; i++)
	{
		if (strcmp(choseip, choose_ip[i]) == 0) //相同返回0
		{

			filefd = files[i];
			strcat(rmcmd, filefd); //strcat函数：把src所框架的字符串追加到dest所框架的字符串结尾
			strcat(chmodcmd, filefd);
			break;
		}
	}

	// system("rm ./1/1.txt");
	system(rmcmd); //先删除该文件
	creat(filefd, S_IRUSR | S_IWUSR); //再新创建该文件，并赋予用户读、写权限
	//  system("chmod 777 ./1/1.txt");
	system(chmodcmd); //再修改文件权限为777
	fd = open(filefd, O_RDWR); //最后打开该文件，返回文件描述符

	return fd;
}

int thread_file_init(void)
{
	int fd = 0;
	int i = 0;
	char choose_ip[7][14] = {"192.168.0.201",
							 "192.168.0.202",
							 "192.168.0.203",
							 "192.168.0.204",
							 "192.168.0.205",
							 "192.168.0.206",
							 "192.168.0.207"};
	unsigned char files[7][15] = {"./record/11.txt",
								  "./record/22.txt",
								  "./record/33.txt",
								  "./record/44.txt",
								  "./record/55.txt",
								  "./record/66.txt",
								  "./record/77.txt"};
	unsigned char *choseip = NULL;
	unsigned char *filefd = NULL;

	char rmcmd[20];
	char chmodcmd[20];
	strcpy(rmcmd, "rm ");
	strcpy(chmodcmd, "chmod 777 ");

	choseip = GetLocalIp();
	for (i = 0; i < 7; i++)
	{
		if (strcmp(choseip, choose_ip[i]) == 0)
		{

			filefd = files[i];
			strcat(rmcmd, filefd);
			strcat(chmodcmd, filefd);
			break;
		}
	}

	// system("rm ./1/1.txt");
	system(rmcmd);
	creat(filefd, S_IRUSR | S_IWUSR);
	//  system("chmod 777 ./1/1.txt");
	system(chmodcmd);
	fd = open(filefd, O_RDWR);

	return fd;
}

void file_write_xy(int fd, int x, int y)
{
	char buf[20];
	sprintf(buf, "%d%s%d", x, " ", y);
	char space[] = {0x20};
	char enter[] = {0x0d, 0x0a};

	write(fd, buf, strlen(buf));
	write(fd, enter, sizeof(enter));
}
void file_write_xyz(int fd, int x, int y, int z)
{
	char buf[20];
	sprintf(buf, "%d%s%d%s%d", x, " ", y, " ", z);
	char space[] = {0x20};
	char enter[] = {0x0d, 0x0a};

	write(fd, buf, strlen(buf));
	write(fd, enter, sizeof(enter));
}

void file_write_ekf(int fd, int a, int b, int c, int d, int e, int f, int g)
{
	char buf[80];
	sprintf(buf, "%d%s%d%s%d%s%d%s%d%s%d%s%d", a, " ", b, " ", c, " ", d, " ", e, " ", f, " ", g);
	char space[] = {0x20};
	char enter[] = {0x0d, 0x0a};

	write(fd, buf, strlen(buf));
	write(fd, enter, sizeof(enter));
}
void file_write(int fd, int time, int x, int y, int tht, int tht1)
{

	char buf[40];
	sprintf(buf, "%d%s%d%s%d%s%d%s%d", time, " ", x, " ", y, " ", tht, " ", tht1);

	char space[] = {0x20};
	char enter[] = {0x0d, 0x0a};

	write(fd, buf, strlen(buf));
	write(fd, enter, sizeof(enter));

	/*char buf[]="123";
 
 char buf1[]={49,50};
 
 char space[]={0x20};
 
 char enter[]={0x0d,0x0a};

 write(fd,buf,strlen(buf));write(fd,space,sizeof(space));write(fd,buf1,sizeof(buf1));

write(fd,enter,sizeof(enter));

write(fd,buf,strlen(buf));write(fd,space,sizeof(space));write(fd,buf1,sizeof(buf1));
write(fd,enter,sizeof(enter));
*/
}

unsigned long get_file_size(const char *path)
{
	unsigned long filesize = -1;
	struct stat statbuff;
	if (stat(path, &statbuff) < 0)
	{
		return filesize;
	}
	else
	{
		filesize = statbuff.st_size;
	}
	return filesize;
}

void chose_xy(void)
{
	unsigned char *choseip = NULL;
	int i = 0;
	char choose_ip[7][14] = {"192.168.0.201",
							 "192.168.0.202",
							 "192.168.0.203",
							 "192.168.0.204",
							 "192.168.0.205",
							 "192.168.0.206",
							 "192.168.0.207"};

	int choose_xy[7][2] = {{412, 289},
						   {367, 300}, //因为装置设置不好，需要略微修改
						   {368, 293},
						   {420, 299},
						   {420, 300},
						   {420, 300},
						   {420, 300}};
	choseip = GetLocalIp();

	for (i = 0; i < 7; i++)
	{

		if (strcmp(choseip, choose_ip[i]) == 0)
		{
			cir_x = choose_xy[i][0];
			cir_y = choose_xy[i][1];
			break;
		}
	}
}

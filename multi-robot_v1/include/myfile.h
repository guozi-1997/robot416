#ifndef _MYFILE_H
#define _MYFILE_H

#include <global.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <sys/time.h>
#include <signal.h>
void file_write_ekf(int fd, int a, int b, int c, int d, int e, int f, int g);
void file_write_xy(int fd, int x, int y);
#endif /* _MYFILE_H */

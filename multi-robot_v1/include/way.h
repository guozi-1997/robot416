#ifndef _WAY_H
#define _WAY_H

#include <global.h>
#include <stdio.h>  
#include <stdlib.h>  
   

 int show_y[1000];
 int show_x[1000];

 void min_cir(unsigned char *addr);
 int rgb(unsigned char *addr,int y,int x);
 void scan(unsigned char *addr);
 //-----------------link------------------------
 struct info  
 {	
	 int data;	
	 int connected_flag;
	 int start_connect_flag;
	 int iSocketClient;
	 char send_ip[14];
	 unsigned int dis_ang[20];
	 struct info *pNext;
 };  
 
 typedef struct info node, *PNODE;	
   
 PNODE addBack(PNODE phead, int data,char *ip,int fd);	 //尾部插入  
 PNODE addFront(PNODE phead, int data);  //头部插入  
 PNODE findFirst(PNODE phead, int data); //检索数据  
 PNODE findIp(PNODE phead, char *ip); //检索IP  
 PNODE deleteFirst(PNODE phead, int data);	 //删除数据  
   
 PNODE deleteNode(PNODE phead, int data,PNODE *ptemp);	 //删除数据  
   
 PNODE insertNode(PNODE phead, int finddata, int data);  //插入数据  
 int getNum(PNODE phead);	 //返回链表的个数  
 void showAll(PNODE phead);  //显显示全部  


#endif /* _WAY_H */


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
   
 PNODE addBack(PNODE phead, int data,char *ip,int fd);	 //β������  
 PNODE addFront(PNODE phead, int data);  //ͷ������  
 PNODE findFirst(PNODE phead, int data); //��������  
 PNODE findIp(PNODE phead, char *ip); //����IP  
 PNODE deleteFirst(PNODE phead, int data);	 //ɾ������  
   
 PNODE deleteNode(PNODE phead, int data,PNODE *ptemp);	 //ɾ������  
   
 PNODE insertNode(PNODE phead, int finddata, int data);  //��������  
 int getNum(PNODE phead);	 //��������ĸ���  
 void showAll(PNODE phead);  //����ʾȫ��  


#endif /* _WAY_H */


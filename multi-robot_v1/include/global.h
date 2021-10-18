#ifndef _GLOABLE_H
#define _GLOABLE_H

#include <disp_manager.h>
#include <video_manager.h>
#include <convert_manager.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <config.h>
#include <render.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include "MyMatrix.h"
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <math.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <input_manager.h>

// #define dis_50 67.0 //75  //1=67.0 2=65.0   3=65   4=66

// #define a11 0.3683
// #define a12 0.9848
// #define a13 1.1477

// #define a21 0.1409
// #define a22 0.2431
// #define a23 2.0380

// #define a31 0.0090
// #define a32 0.1257
// #define a33 3.4491

// #define a41 0.0038
// #define a42 0.0601
// #define a43 4.9448

//#define cir_x  370
//#define cir_y  300


#define ROBOT_WIDTH	40    //闁告娲戠紞鍛村储濡壈鍎�

#define  delta_l      		20	//婵縿鍎甸弳杈╂媼閸撗呮瀭濞戞搫鎷�20cm

#define ROLLWINDOW_R	1000.0

#define	 TrackNodeNum	20
int dlta_d ;		  //缂佹儳娼￠埀顒傚枎鐎规娊鏁嶇仦绛嬫搐 dlta_d = 100 閻炴稏鍔庨妵锟�100mm/s 闁靛棙鍔栫敮鍫曞礆閼稿灚绨氶柛锝冨妺濮瑰宕滃鍫㈢闁告艾閰ｉ埀顑藉亾闁挎稑鐗婇婊堝极閺夊灝顤呴弶鈺傚喕缁辨繄鎷归悢鍛婃闁告艾閰ｉ埀顑藉亾闁挎稑顦埀顒婃嫹
int dlta_a ;		  //閻熸瑦甯￠埀顒傚枎鐎规娊鏁嶇仦绛嬫搐 dlta_a = 100 閻炴稏鍔庨妵锟�100rad/s闁靛棙鍔栫敮鍫曞礆閼稿灚绨氶柛锝冨妺濮瑰顔忛敃浣圭ギ闁告瑥鐤囧ù鍡涙晬閸噥鍔€闁轰焦婢樻稊蹇旀姜椤掑﹦绀夐悹鎰枑閺嗙喖宕ｇ€圭姵绁柨娑橆槶閳ь剨鎷�

int get_x;
int get_y;
int get_tht;
int cir_x;
int cir_y;
#define cell(x, y, d) (4 * (x)-3200 * (y) + cir_x * 4 + 3200 * cir_y + d)
#define DAY 2 // 1闁谎嗘閵囷拷  0 濮掓稒鍨甸¨渚€鎳¤閸樻粓鎮橀婊冨季闁哄嫸鎷� 2 闁煎浜埀顒€鍊哥花鑼沪閻愭壆鑹剧痪顓у厸濞嗐垽鏌婂鍥╂瀭addr[ii*3200+jj*4+0]
#define big_cell(x, y, d) (4 * (x) + 4096 * (y) + d)

struct get_point
{
	unsigned int r;
	unsigned int g;
	unsigned int b;
} get_point1, get_point2, get_point3, get_point4;

struct ekf
{
	int i;
	Matrix A;
	Matrix H;
	Matrix X;  //闁告艾閰ｉ悰娆愬閹峰矈鍚€闁挎稑鐗婂〒鑸靛濡鍙忔慨婵撶到閳ь剛銆嬬槐婵嬪嫉閳ь剛绱掗崼锝庢矗闁汇劌瀚銊╁及椤栨繄绠瑰☉鎿冧悍缁憋拷
	Matrix X_; //闁稿繐鐗撻悰娆愬閹峰矈鍚€
	Matrix P;
	Matrix P_;
	Matrix K;
	Matrix K_;
	Matrix Z; //閻熸瑥鍊圭粊鎾磹閿燂拷
	Matrix K__;
	Matrix Q;
	Matrix R;
	double Avalue[4];
	double Hvalue[4];
	double Xvalue[2];
	double X_value[2];
	double oldXvalue[2];
	double Zvalue[2];
	double Rvalue[4];
	double Qvalue[4];
	double tht; //閻庢稒蓱濠€浼村嫉鏉炴壆鐟㈤柣鈺婂枟閻栵綁鎯冮崟顓熸毄閻庢稒鍔楃紞蹇涙儎濡灝鐒婚柛姘灱椤鎲撮幒鎴濐唺鐎瑰壊鍣槐鐢告晬閻曞倻鍚�
	double old_tht1;
	double tht2; //閻庢稒蓱濠€浼村嫉鏉炴壆鐟㈤柣鈺婂枟閻栵綁鎯冮崟顓熸毄閻庢稒鍔楃紞蹇涙儎濡灝鐒婚柛姘灱椤鎲撮幒鎴濐唺鐎瑰壊鍣槐鐢告晬閻曞倻鍚�

	int tht3;
	int tht4; //闁烩晩鍠楅悥锝夊嫉閸濆嫭鐝ゅù婊堢細閸╁懏鎷呭鍡楄吂缂佺姵顨堝▓鎴犳喆閹烘垵顔婇柨娑樼墢濞蹭即寮介崶鈺傜暠闁稿绻楀ù鍡欐喆閹虹偟绀�
	int tht5;

	int tht6;
	int tht7;
	int tht8; //闁哄牜鍓氬┃鈧柤鍓蹭簷缂嶅懘骞掗妸褏鏆柣銊ュ椤鎯旈敂鑲╃闁哄牜鍓氬┃鈧柣銊ュ娴滃憡娼鍐炬健闁挎冻鎷�

	int x1;
	int x2; //闁哄牜鍓氬┃鈧柤鍓蹭簷缂嶅懘骞掗妸褏鏆柣銊ュⅰ闁稿⿵鎷�
	int x3;
	int y1;
	int y2; //闁哄牜鍓氬┃鈧柤鍓蹭簷缂嶅懘骞掗妸褏鏆柣銊ュⅱ闁稿⿵鎷�
	int y3;
	int distance; //闁哄牜鍓氬┃鈧柤鍓蹭簷缂嶅懘骞掗妸褏鏆柣銊ュ浜涢柛鏂诲姀缁愭稓绮嬫导娆戠闁哄牜鍓氬┃鈧柣銊ュ婢х姵娼诲☉婊呯崺缂佸倷绱槐锟�

	int x4;
	int x5; //闁烩晩鍠楅悥锝夊嫉閸濆嫭鐝ゅù婊堢細閸╁懏鎷呭鍡楄吂缂佺姵顨堝▓鎲嶉柛濠忔嫹
	int x6;
	int y4;
	int y5; //闁烩晩鍠楅悥锝夊嫉閸濆嫭鐝ゅù婊堢細閸╁懏鎷呭鍡楄吂缂佺姵顨堝▓鎲忛柛濠忔嫹
	int y6;
	int distance1; //闁烩晩鍠楅悥锝夊嫉閸濆嫭鐝ゅù婊堢細閸╁懏鎷呭鍡楄吂缂佺姵顨堝▓鎴︽儍閸曨厜鈺呭礉閵娿劎鐛╃紒鍌欑串缁辨瑩鎯勯鐣屽灱闁哄牆鎼▍鎺撶閾忚鐣遍柛鎾崇Х缁绘鎹勫┑鍫€查柨娑虫嫹

	int count;
} ekf[4];

struct robot
{
	unsigned int rad;	//radius闁告锕ょ欢鐐烘晬閸絻鈧啰绮堟ウ娆炬綆婵炴潙顑嗗┃鈧柛锝冨妺濮瑰骞嶉埀顒勫箣閹邦剙寮块柡鍜佸灠濞存﹢宕撹箛搴ゅ幀闁挎稑濂旂粭宀勬儎椤旂晫鍨奸柡鍫濇惈濞呮帗绂嶆潪鎵吅闂傚倸顕▓鎴﹀磽韫囨洜顦卞☉鎿冧簼閺嗙喖鏁嶉敓锟�
	int angle;			//闁哄牜鍓氬┃鈧柛蹇嬪妽濞呮瑩宕堕幆褍鍓奸悷娆忓€圭粊鎾礆閹殿喗绐楅柡宥呮处濠р偓闁革絻鍔嬪Ч澶愭儍閸曨噮娼庢繛鏉戭儓椤鎯旈敓锟�
	unsigned int count; //闁稿繈鍔嶅▍娆撳炊閹冨壖濞戞搩鍘惧▓鎴︽偋閻熸壆绐欓柨娑樼墕閸庢氨妲愰悪鍛闁绘劙鈧盯鍤嬮柡渚婃嫹
	int x;
	int y;
	int distance; //闁哄牜鍓氬┃鈧柛蹇嬪妽濞呮瑩宕堕幆褍鍓奸悷娆忓€圭粊鎾礆閹殿喗绐楅柡宥呮处濠р偓闁革絻鍔嬪Ч澶愭儍閸曨噮娼庢繛鏉戭儓缁愭稓绮嬮敓锟�
	float dis_diff;
	int number;
	int dlta_a;	  //闁烩晩鍠楅悥锝夊嫉閸濆嫭鐝ゅù婊呭皑缁辨瑩宕ｉ幋锝囩畺闁哄鍎冲▓鎴︽晬婢跺寒娼￠梺顐ゅ枎鐎癸拷
	int dlta_d;	  //闁烩晩鍠楅悥锝夊嫉閸濆嫭鐝ゅù婊呭皑缁辨瑩宕ｉ幋锝囩畺闁哄鍎冲▓鎴︽晬婢跺苯娈犻梺顐ゅ枎鐎癸拷
	int Ture_Tht; //閻庢稒蓱濠€浼村嫉鏉炴壆鐟㈤柣鈺婂枟閻栵綁鎯冮崟顓熸毄閻庢稒鍔楃紞蹇涙儎濡灝鐒婚柛姘灱椤鎲撮幒鎴濐唺鐎瑰壊鍣槐鐢告晬閻曞倻鍚�
} robot_temp[7], robot_end[7], robot_send[7], robot_rece[7], robot_ekf[7], robot_ekf2[7], robot_self[7];
struct ekf_send
{
	int dlta_a; //
	int dlta_d; //
	int distance[10];
	int angle[10];
	int bef_distance[10];
	int bef_angle[10];
} ekf_send[10];
T_VideoDevice tVideoDevice_mid; //
PT_VideoBuf ptVideoBufCur_mid;	//
T_VideoBuf tVideoBuf_mid;		//
T_VideoBuf tFrameBuf_mid;		//

T_VideoDevice tVideoDevice_now; //
PT_VideoBuf ptVideoBufCur_now;	//
T_VideoBuf tVideoBuf_now;		//
T_VideoBuf tFrameBuf_now;		//

int Ture_Tht;	 //闁汇垽娼ч悺娆戠磾濡ゅ啯纾搁柤鍓蹭簻閹粎鎲撮幒鐐电缁惧彞绀佺€垫娊寮悷鐗堝€诲┑顔碱儜缁辨瑥鈻介崸妞尖偓搴ㄥ籍閸洘瀚涢柨娑橆槸閸╁矁銇愰幘鍐差枀闁哄倻鎳撻幃婊冾潰閵忋垺鐣卞鍓侇攰椤鏁嶉崼婵堢Ъ缂傚啯顨堝ú蹇擃潩閺夋垿鎸柡鍐儓濞村棝鎯冮崟顒侇槯闁稿﹥鐟辩槐婵嬫嚋椤忓嫭鍊婚悷娆愬笒濠€锟�0闁炽儻鎷�360閹艰揪缂氱粻锝夋⒒閺夋垵缍侀柛鏍ㄧ壄缁憋拷
int First_Angle; //閻犱焦婢樼紞宥夊嫉椤掍焦绨氶柛锝冨妺濮瑰宕氬┑鍡╂綏闁汇劌瀚崺鍛村触閹达綆娼￠悷娆愬笒鐎癸拷
int first_flag;	 //濞戞挻妞介崢銈夊触閿燂拷 First_Angle 閻犱礁澧介悿鍡涙儍閸曨剛鍨奸煫鍥ㄣ仦缂嶏拷
int file_flag;
float CurLidarDist;
int CurLidarQuality;

int CurLidarDistPix2;
float CurLidarAng2;

int AimCount;
int CurLidarDistPix[1024];
int CurLidarAng[1024];		//lidar婵絽绻嬮柌婊堟煂閸ャ劎澹夐柣鎰贡濞堟垹鎲撮幒鎴濐唺闁稿⿵鎷�
int CurLidarDistance[1024]; //lidar婵絽绻嬮柌婊堟煂閸ャ劎澹夐柣鎰贡濞堟垹鎹勫┑鍫€查柛濠忔嫹

int TrueAimCount;
int TrueCurLidarAng[2000];
int TrueCurLidarDistPix[2000];	//闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔诲Ν娴兼瑦瀚归柨鐔告灮閹峰嘲娴橀柨鐔告灮閹凤拷

int TrueAimCntDis;
int TrueCurLidarAngle[2000];
int TrueCurLidarDistance[2000];

int FilterCount;
int FilterCount2;
int FilterLidarAng[2000];
int FilterLidarDistPix[2000];
int FilterLidarAng2[2000];
int FilterLidarDistance[2000];

int LidarEdgeCnt;
int LidarEdgeDistPix[1024];
int LidarEdgeAngle[1024];
int LidarEdgeDistance[1024];

int OkPathCnt;
int OkPathDistPix[10];
int OkPathAngle[10];
int OkPathDistance[10];

int LidarOkPathMidCnt;
int LidarOkPathMidAng[10];
int LidarOkPathMidPix[10];
int LidarOkPathMidDistance[10];

int PicEdgePixCnt;
int PicEdgePix[1024];
int PicEdgeAngle[1024];

int PathPlanningCnt;
int NumPathPlanningCnt;
int PathPlanningAng[5][100][5];
int PathPlanningPix[5][100][5];
int PathPlanningDistance[10][5];

int RRTPlanningCnt;
int RRTPlanningAng[256];
int RRTPlanningPix[256];

float PathWidth;

int isOkPath;
int SafeFlag;
int RandRito;
int ROLLWINDOW_RITO;

int HaveDoubleObsPath;

int startflag;

int fd_rand;

int Next_UnSafe_Flag;

//int Layer;
int RRT_Ok_Flag;
int Arrived_ChildAim_Flag;
int isArrivedDirect_Flag;
int SrandNum[5];

int Lidar_Circle_Cnt;
int Lidar_PerCircle_Ok_Flag;

int Safe_Dis_Sub;
int Track_Flag;

int timer_cnt;

typedef struct Cordinate_PerLayer
{
	int Layer;
	int Cordinate_X[10][256];
	int Cordinate_Y[10][256];
	int Cordinate_Ang[10][256];
}T_CorPerLayer;

T_CorPerLayer   Cord_PerLayer;

typedef struct Ok_PathPlanning_Point
{
	int Layer_Num;
	int Ok_Angle[1024];
	int Ok_Pix[1024];
	int Ok_Distance[1024];
	int Ok_Cnt_PerLayer;
	int Ok_PerLayer_Angle[1024][4];
}T_OkPath;

typedef struct Best_OkPath_Point
{
	int Best_Layer_Num;
	int Best_Ok_Angle[10];
	int Best_Ok_Pix[10];
	int Best_Ok_Distance[10];
	int Best_Ok_PerLayer_Ang[10][4];
	int Best_Ok_Cnt;
}T_Best_OkPath;

struct timer
{
	double PreMs;
	double NowMs;
	int start_flag;
	int time_flag;
} get_time, ekf_gettime, ekf_gettime2;



/* 閻忓骏鎷� 閻庡箍鍨洪崺娑氱博椤栨碍鍕鹃柛褉鍋撶紓浣规尰閻庯拷 闁告粣鎷� 闁哄牆绉存慨鐔煎闯閵娿劎绠鹃柟鎭掑劚椤ㄦ粓骞掗妷銉ф憻闁硅绻楅崼顏嗙箔閿燂拷 缂備焦鍨甸悾楣冨捶閵娿倗顏遍悹褍鍤栫槐婵囩▔閳ь剚绋夐埀顒傗偓鐢垫嚀缁拷 */
typedef struct net_receive
{
	int iSocketClient; //闁哄牆绉存慨鐔煎闯閵娿劎绠鹃柟鎭掑劚椤ㄦ粓骞掗妷銉ф憻闁硅绻楅崼顏嗙箔閿燂拷
	unsigned char *ip; //閻庡箍鍨洪崺娑氱博椤栨碍鍕鹃柛褉鍋撶紓浣规尰閻庣枠P
} t_net_rec, *pt_net_rec;

 typedef struct
{	
	int x;
	int y;
} Point2D; 

Point2D	no_control_point[2],one_control_point[3],double_control_points[4];
Point2D	no_control_r[TrackNodeNum],one_control_r[TrackNodeNum],double_control_r[TrackNodeNum];
#endif /* _gloable_H */


#include <way.h>
#define min(a, b) ((a > b) ? (b) : (a))
#define max(a, b) ((a > b) ? (a) : (b))

#define branch_num 3     //搜索树分支数
#define Layer 4          //静态搜索树层数为定值
#define ObsDeltaAng 150  //障碍物与搜索路径角度差阈值
#define ExpandDis 200    //膨胀距离mm
#define Step 40          //步长,单位pix
#define StepDistance 300 //步长，单位mm
#define Display_Flag 1   //0显示所有搜索路径，1显示规划后的可行路径
float dis_50 = 67.0;
int randangle[6][100][10] = {0};
int childrandcnt = 0;
int pix[10] = {0};
int stepdis[10] = {0};
int planning_cnt = 0;
RandRito = 1;
int time_srand = 0;
int release_num[10] = {0};
int Node_PerNum = 3; //所在层的节点数

int Circle_num = 0; //产生随机数的循环次数

int obs_search_cnt = 0;
int tmpminpix = 0;
int tmpmaxpix = 0;

int tmpmindistance = 0;
int tmpmaxdistance = 0;

int tmpminangsub = 0;

int tmpminang_2 = 0;
int tmpminpix_2 = 0;

T_OkPath OkPath;
T_Best_OkPath Best_OkPath;

struct Distance
{

    int tmp4_angle;
    int tmp4_distance;
    int evaluation;
    int tmp_angle_layer4[4];
    int dis_tmp4_i_x[4];
    int dis_tmp4_i_y[4];

    int dis_tmp4_j_x[4];
    int dis_tmp4_j_y[4];

    int dis_tmp_i_4[4];
    int dis_tmp_j_4[4];
    float finalaim_x;
    float finalaim_y;
    float finalaim;
    int BA[2];
    int AC[2];
    int BC[2];

    int BA_x[2];
    int AC_x[2];
    int BC_x[2];

    int BA_y[2];
    int AC_y[2];
    int BC_y[2];
    float angle_A[2];

    int cost_i;
    int cost_j;
} dis;

int goalListLeader(struct Distance *dis, int i, int j, int p)
{
    /********一些评估函数用的上的预处理数据************/
    dis->finalaim_x = robot_temp[1].distance * cos(robot_temp[1].angle * 3.14 / 1800);
    dis->finalaim_y = robot_temp[1].distance * sin(robot_temp[1].angle * 3.14 / 1800);

    dis->dis_tmp4_i_x[0] = StepDistance * cos(OkPath.Ok_PerLayer_Angle[i][0] * 3.14 / 1800);
    dis->dis_tmp4_i_y[0] = StepDistance * sin(OkPath.Ok_PerLayer_Angle[i][0] * 3.14 / 1800);

    dis->dis_tmp4_i_x[1] = dis->dis_tmp4_i_x[0] + StepDistance * cos(OkPath.Ok_PerLayer_Angle[i][1] * 3.14 / 1800);
    dis->dis_tmp4_i_y[1] = dis->dis_tmp4_i_y[0] + StepDistance * sin(OkPath.Ok_PerLayer_Angle[i][1] * 3.14 / 1800);

    dis->dis_tmp4_i_x[2] = dis->dis_tmp4_i_x[1] + StepDistance * cos(OkPath.Ok_PerLayer_Angle[i][2] * 3.14 / 1800);
    dis->dis_tmp4_i_y[2] = dis->dis_tmp4_i_y[1] + StepDistance * sin(OkPath.Ok_PerLayer_Angle[i][2] * 3.14 / 1800);

    dis->dis_tmp4_i_x[3] = dis->dis_tmp4_i_x[2] + StepDistance * cos(OkPath.Ok_PerLayer_Angle[i][3] * 3.14 / 1800);
    dis->dis_tmp4_i_y[3] = dis->dis_tmp4_i_y[2] + StepDistance * sin(OkPath.Ok_PerLayer_Angle[i][3] * 3.14 / 1800);

    dis->dis_tmp_i_4[0] = (dis->finalaim_x - dis->dis_tmp4_i_x[0]) * (dis->finalaim_x - dis->dis_tmp4_i_x[0]) + (dis->finalaim_y - dis->dis_tmp4_i_y[0]) * (dis->finalaim_y - dis->dis_tmp4_i_y[0]);
    dis->dis_tmp_i_4[1] = (dis->finalaim_x - dis->dis_tmp4_i_x[1]) * (dis->finalaim_x - dis->dis_tmp4_i_x[1]) + (dis->finalaim_y - dis->dis_tmp4_i_y[1]) * (dis->finalaim_y - dis->dis_tmp4_i_y[1]);
    dis->dis_tmp_i_4[2] = (dis->finalaim_x - dis->dis_tmp4_i_x[2]) * (dis->finalaim_x - dis->dis_tmp4_i_x[2]) + (dis->finalaim_y - dis->dis_tmp4_i_y[2]) * (dis->finalaim_y - dis->dis_tmp4_i_y[2]);
    dis->dis_tmp_i_4[3] = (dis->finalaim_x - dis->dis_tmp4_i_x[3]) * (dis->finalaim_x - dis->dis_tmp4_i_x[3]) + (dis->finalaim_y - dis->dis_tmp4_i_y[3]) * (dis->finalaim_y - dis->dis_tmp4_i_y[3]);

    dis->dis_tmp4_j_x[0] = StepDistance * cos(OkPath.Ok_PerLayer_Angle[j][0] * 3.14 / 1800);
    dis->dis_tmp4_j_y[0] = StepDistance * sin(OkPath.Ok_PerLayer_Angle[j][0] * 3.14 / 1800);

    dis->dis_tmp4_j_x[1] = dis->dis_tmp4_j_x[0] + StepDistance * cos(OkPath.Ok_PerLayer_Angle[j][1] * 3.14 / 1800);
    dis->dis_tmp4_j_y[1] = dis->dis_tmp4_j_y[0] + StepDistance * sin(OkPath.Ok_PerLayer_Angle[j][1] * 3.14 / 1800);

    dis->dis_tmp4_j_x[2] = dis->dis_tmp4_i_x[1] + StepDistance * cos(OkPath.Ok_PerLayer_Angle[j][2] * 3.14 / 1800);
    dis->dis_tmp4_j_y[2] = dis->dis_tmp4_i_y[1] + StepDistance * sin(OkPath.Ok_PerLayer_Angle[j][2] * 3.14 / 1800);

    dis->dis_tmp4_j_x[3] = dis->dis_tmp4_j_x[2] + StepDistance * cos(OkPath.Ok_PerLayer_Angle[j][3] * 3.14 / 1800);
    dis->dis_tmp4_j_y[3] = dis->dis_tmp4_j_y[2] + StepDistance * sin(OkPath.Ok_PerLayer_Angle[j][3] * 3.14 / 1800);

    dis->dis_tmp_j_4[0] = (dis->finalaim_x - dis->dis_tmp4_j_x[0]) * (dis->finalaim_x - dis->dis_tmp4_j_x[0]) + (dis->finalaim_y - dis->dis_tmp4_j_y[0]) * (dis->finalaim_y - dis->dis_tmp4_j_y[0]);
    dis->dis_tmp_j_4[1] = (dis->finalaim_x - dis->dis_tmp4_j_x[1]) * (dis->finalaim_x - dis->dis_tmp4_j_x[1]) + (dis->finalaim_y - dis->dis_tmp4_j_y[1]) * (dis->finalaim_y - dis->dis_tmp4_j_y[1]);
    dis->dis_tmp_j_4[2] = (dis->finalaim_x - dis->dis_tmp4_j_x[2]) * (dis->finalaim_x - dis->dis_tmp4_j_x[2]) + (dis->finalaim_y - dis->dis_tmp4_j_y[2]) * (dis->finalaim_y - dis->dis_tmp4_j_y[2]);
    dis->dis_tmp_j_4[3] = (dis->finalaim_x - dis->dis_tmp4_j_x[3]) * (dis->finalaim_x - dis->dis_tmp4_j_x[3]) + (dis->finalaim_y - dis->dis_tmp4_j_y[3]) * (dis->finalaim_y - dis->dis_tmp4_j_y[3]);
    /**************************************************************/

    dis->BC_x[0] = dis->dis_tmp4_i_x[3] - dis->dis_tmp4_i_x[2];
    dis->BC_y[0] = dis->dis_tmp4_i_y[3] - dis->dis_tmp4_i_y[2];
    dis->BC[0] = dis->BC_x[0] * dis->BC_x[0] + dis->BC_y[0] * dis->BC_y[0];
    dis->BC[0] = (int)pow(dis->BC[0], 0.5);

    dis->BA[0] = (int)pow(dis->dis_tmp_i_4[2], 0.5);
    dis->AC[0] = (int)pow(dis->dis_tmp_i_4[3], 0.5);
    dis->angle_A[0] = dis->dis_tmp_i_4[2] + dis->dis_tmp_i_4[3] - dis->BC[0] * dis->BC[0];
    dis->angle_A[0] = dis->angle_A[0] / (2 * dis->BA[0] * dis->AC[0]);

    dis->BC_x[1] = dis->dis_tmp4_j_x[3] - dis->dis_tmp4_j_x[2];
    dis->BC_y[1] = dis->dis_tmp4_j_y[3] - dis->dis_tmp4_j_y[2];
    dis->BC[1] = dis->BC_x[1] * dis->BC_x[1] + dis->BC_y[1] * dis->BC_y[1];
    dis->BC[1] = (int)pow(dis->BC[1], 0.5);

    dis->BA[1] = (int)pow(dis->dis_tmp_j_4[2], 0.5);
    dis->AC[1] = (int)pow(dis->dis_tmp_j_4[3], 0.5);
    dis->angle_A[1] = dis->dis_tmp_j_4[2] + dis->dis_tmp_j_4[3] - dis->BC[1] * dis->BC[1];
    dis->angle_A[1] = dis->angle_A[1] / (2 * dis->BA[1] * dis->AC[1]);
    //printf("dis->angle_A[0] = %f\n", dis->angle_A[0]);

    /*******heading评估函数**********/
    dis->angle_A[0] = acos(dis->angle_A[0]) * 180 / pi;
    dis->angle_A[1] = acos(dis->angle_A[1]) * 180 / pi;
    //heading评估函数第一次评分
    OkPath.evaluation[j] = -dis->angle_A[1] * 5;
    OkPath.evaluation[i] = -dis->angle_A[0] * 5;
    /**************************************************************/

    /********gaollist代价函数************/
    dis->cost_i = pow(dis->dis_tmp_i_4[0], 0.5);
    dis->cost_j = pow(dis->dis_tmp_j_4[0], 0.5);
    //gaollist评估函数第二次评分
    //只有距离目标距离值小于2m的时候才会开启goallist函数
    dis->finalaim = pow((dis->finalaim_x * dis->finalaim_x + dis->finalaim_y * dis->finalaim_y), 0.5);
    if (dis->finalaim <= 200)
    {
        OkPath.evaluation[i] = OkPath.evaluation[i] - dis->cost_i / 10 * 5;
        OkPath.evaluation[j] = OkPath.evaluation[j] - dis->cost_j / 10 * 5;
    }
    /**************************************************************/

    if (OkPath.evaluation[i] < OkPath.evaluation[j])
    {
        dis->tmp4_distance = OkPath.Ok_Distance[i];
        OkPath.Ok_Distance[i] = OkPath.Ok_Distance[j];
        OkPath.Ok_Distance[j] = dis->tmp4_distance;

        dis->tmp4_angle = OkPath.Ok_Angle[i];
        OkPath.Ok_Angle[i] = OkPath.Ok_Angle[j];
        OkPath.Ok_Angle[j] = dis->tmp4_angle;

        dis->evaluation = OkPath.evaluation[i];
        OkPath.evaluation[i] = OkPath.evaluation[j];
        OkPath.evaluation[j] = dis->evaluation;

        for (p = 0; p < 4; p++)
        {
            dis->tmp_angle_layer4[p] = OkPath.Ok_PerLayer_Angle[i][p];
            OkPath.Ok_PerLayer_Angle[i][p] = OkPath.Ok_PerLayer_Angle[j][p];
            OkPath.Ok_PerLayer_Angle[j][p] = dis->tmp_angle_layer4[p];
        }
    }

    return 0;
}

int get_r(int rad)
{

    if (rad < 20)
    {
        return rad / 4 + 5;
    }
    if (rad >= 20 && rad < 30)
    {
        return rad / 4 + 4;
    }
    if (rad >= 30 && rad < 40)
    {
        return rad / 4 + 3;
    }
    if (rad >= 40 && rad < 50)
    {
        return rad / 4 + 2;
    }
    if (rad >= 50 && rad < 60)
    {
        return rad / 4 + 1;
    }
    if (rad >= 60 && rad < 70)
    {
        return rad / 4 + 0;
    }
    if (rad >= 70 && rad < 80)
    {
        return rad / 4 - 1;
    }
    if (rad >= 80 && rad < 90)
    {
        return rad / 4 - 2;
    }
    if (rad >= 90 && rad < 100)
    {
        return rad / 4 - 3;
    }
    if (rad >= 100)
    {
        return rad / 4 - 4;
    }
}
//=========================================================================
void min_cir(unsigned char *addr)
{
    int y = 0;
    int x = 0;
    float angle = 0;
    int r = 0;
    int d = 0;
    int d_x = 0;
    int d_y = 0;
    int m = 0, n = 0;
    int i = 0;
    int j = 0;
    int k = 0;
    int l = 0;
    int p = 0;
    int s = 0, q = 0;
    float theta = 0.0;

    //--------------------画直角坐标系-------------------------------
    x = 0;
    for (y = -275; y < 275; y++)
    {
        addr[cell(x, y, 0)] = 255;
        addr[cell(x, y, 1)] = 255;
        addr[cell(x, y, 2)] = 255;
    }

    y = 0;
    for (x = -275; x < 275; x++)
    {
        addr[cell(x, y, 0)] = 255;
        addr[cell(x, y, 1)] = 255;
        addr[cell(x, y, 2)] = 255;
    }
    //这里的距离值已经被处理，与图像相接轨
    //----------判断lidar采集到的障碍物点是否属实(滤波)-------
    if (AimCount > 50)
    {
        for (i = 0; i < AimCount; i++)
        {
            //r = CurLidarDistPix[i];
            if ((i > 0) && (i < AimCount - 1) && (abs(CurLidarDistPix[i] - CurLidarDistPix[i - 1]) < 2 || abs(CurLidarDistPix[i + 1] - CurLidarDistPix[i]) < 2))
            {
                if (CurLidarDistPix[i] > 40)
                {
                    TrueCurLidarDistPix[TrueAimCount] = CurLidarDistPix[i];
                    TrueCurLidarAng[TrueAimCount] = CurLidarAng[i];
                    //printf("TrueCurLidarAng[%d]=%d\n",TrueAimCount,TrueCurLidarAng[TrueAimCount]);
                    //printf("TrueCurLidarDistPix[%d]=%d\n",TrueAimCount,TrueCurLidarDistPix[TrueAimCount]);
                    TrueAimCount++;
                }
                //printf("TrueAimCount = %d\n", TrueAimCount);
            }

            if ((i > 0) && (i < AimCount - 1) && (abs(CurLidarDistance[i] - CurLidarDistance[i - 1]) < 5 && abs(CurLidarDistance[i + 1] - CurLidarDistance[i]) < 5))
            {
                TrueCurLidarDistance[TrueAimCntDis] = CurLidarDistance[i];
                TrueCurLidarAng2[TrueAimCntDis] = CurLidarAng[i];
                TrueAimCntDis++;
                if (CurLidarDistance[i] < 350)
                {
                    //	printf("CurLidarDistance[%d]=%d\n",i-1,CurLidarDistance[i-1]);
                    //	printf("CurLidarDistance[%d]=%d\n",i,CurLidarDistance[i]);
                    //	printf("CurLidarDistance[%d]=%d\n",i+1,CurLidarDistance[i+1]);
                }
            }
        }
        //	printf("TrueAimCntDis=%d,TrueAimCount=%d\n",TrueAimCntDis,TrueAimCount);
    }

    //--------------------滤波后的lidar障碍物显示-----------------------------------

    //-----雷达坐标转换到图像坐标,雷达坐标的零度角位于机器人正向y轴上-------
    for (i = 0; i < TrueAimCount; i++)
    {
        TrueCurLidarAng[i] = (3600 + 900 - TrueCurLidarAng[i]) % 3600;
    }

    for (i = 0; i < TrueAimCntDis; i++)
    {
        TrueCurLidarAng2[i] = (3600 + 900 - TrueCurLidarAng2[i]) % 3600;
    }

    //--------------冒泡法排序------------------------------------------
    //----------将扫描到的障碍物转化到机器人显示坐标下--
    //----------按照角度递增排序(机器人头朝向90度)------------
    int tmpangle = 0;
    int tmppix = 0;
    int tmpdis = 0;
    for (i = 0; i < TrueAimCount - 1; i++)
    {
        for (j = (i + 1); j < TrueAimCount; j++)
        {
            if (TrueCurLidarAng[i] > TrueCurLidarAng[j])
            {
                tmpangle = TrueCurLidarAng[i];
                TrueCurLidarAng[i] = TrueCurLidarAng[j];
                TrueCurLidarAng[j] = tmpangle;

                tmppix = TrueCurLidarDistPix[i];
                TrueCurLidarDistPix[i] = TrueCurLidarDistPix[j];
                TrueCurLidarDistPix[j] = tmppix;

                /*	tmpdis=TrueCurLidarDistance[i];
				TrueCurLidarDistance[i]=TrueCurLidarDistance[j];
				TrueCurLidarDistance[j]=tmpdis;	
			*/
            }
        }
    }

    for (i = 0; i < TrueAimCntDis - 1; i++)
    {
        for (j = (i + 1); j < TrueAimCntDis; j++)
        {
            if (TrueCurLidarAng2[i] > TrueCurLidarAng2[j])
            {
                tmpangle = TrueCurLidarAng2[i];
                TrueCurLidarAng2[i] = TrueCurLidarAng2[j];
                TrueCurLidarAng2[j] = tmpangle;

                tmpdis = TrueCurLidarDistance[i];
                TrueCurLidarDistance[i] = TrueCurLidarDistance[j];
                TrueCurLidarDistance[j] = tmpdis;
            }
        }
    }

    //-------将采集到的障碍物数据滤波，去除重复数据----

    for (i = 0; i < TrueAimCount - 1; i++)
    {
        if (TrueCurLidarAng[i] != TrueCurLidarAng[i + 1])
        {
            FilterLidarAng[FilterCount] = TrueCurLidarAng[i];
            FilterLidarDistPix[FilterCount] = TrueCurLidarDistPix[i];
            //	FilterLidarDistance[FilterCount]=TrueCurLidarDistance[i];
            FilterCount++;
        }
    }
    for (i = 0; i < TrueAimCntDis - 1; i++)
    {
        if (TrueCurLidarAng2[i] != TrueCurLidarAng2[i + 1])
        {
            FilterLidarAng2[FilterCount2] = TrueCurLidarAng2[i];
            FilterLidarDistance[FilterCount2] = TrueCurLidarDistance[i];
            FilterCount2++;
        }
    }

    //----提取距离机器人最近的障碍物像素和角度----------
    if (FilterCount)
    {
        tmpminpix = FilterLidarDistPix[0];
        tmpmaxpix = FilterLidarDistPix[0];

        tmpminangsub = abs(FilterLidarAng[0] - robot_temp[1].angle);

        for (i = 0; i < FilterCount; i++)
        {
            tmpminpix = min(tmpminpix, FilterLidarDistPix[i]);
            tmpmaxpix = max(tmpmaxpix, FilterLidarDistPix[i]);
            tmpminangsub = min(tmpminangsub, abs(FilterLidarAng[i] - robot_temp[1].angle));
        }
    }

    //----------显示按图像坐标排序后的障碍物-----------------

    if (FilterCount != 0)
    {
        showFiltercount = 0;
        memset(showFilterLidarDistPix, 0, 1024);
        memset(showFilterLidarAng, 0, 1024);
        for (i = 0; i < FilterCount; i++)
        {
            showFilterLidarAng[showFiltercount] = FilterLidarAng[i];
            showFilterLidarDistPix[showFiltercount] = FilterLidarDistPix[i];
            showFiltercount++;
        }
        // printf("showFiltercount = %d,FilterCount =%d\n", showFiltercount, FilterCount);
    }

    for (i = 0; i < showFiltercount; i++)
    {
        r = showFilterLidarDistPix[i];
        if (r < 100)
        {
            for (angle = showFilterLidarAng[i] * 3.14 / 1800 - 0.05; angle < showFilterLidarAng[i] * 3.14 / 1800 + 0.05; angle = angle + 0.05)
            {

                y = r * sin(angle);
                x = r * cos(angle);
                addr[cell(x, y, 0)] = 0;
                addr[cell(x, y, 1)] = 255;
                addr[cell(x, y, 2)] = 255;
            }
        }
        else
        {
            for (angle = showFilterLidarAng[i] * 3.14 / 1800 - 0.05; angle < showFilterLidarAng[i] * 3.14 / 1800 + 0.05; angle = angle + 0.01)
            {

                y = r * sin(angle);
                x = r * cos(angle);
                addr[cell(x, y, 0)] = 0;
                addr[cell(x, y, 1)] = 255;
                addr[cell(x, y, 2)] = 255;
            }
        }
        /*         for (d = 0; d < 4; d++)
        {
            if (showFilterLidarAng[i] < 900)
            {
                y = r * sin(showFilterLidarAng[i] * 3.14 / 1800) - 2 + d;
                x = r * cos(showFilterLidarAng[i] * 3.14 / 1800) - 2 + d;
            }
            if (showFilterLidarAng[i] > 900)
            {
                y = r * sin(showFilterLidarAng[i] * 3.14 / 1800) + 1 + d;
                x = r * cos(showFilterLidarAng[i] * 3.14 / 1800) + 5 * sin(showFilterLidarAng[i] * 3.14 / 1800);
            }
            addr[cell(x, y, 0)] = 255;
            addr[cell(x, y, 1)] = 0;
            addr[cell(x, y, 2)] = 255;
        } */
        //printf("FilterLidarAng[%d]=%d\n",i,FilterLidarAng[i]);
        //printf("FilterLidarDistPix[%d]=%d\n",i,FilterLidarDistPix[i]);
        //printf("\n");
    }

    //------------------------------------------------------------------------
    //--------将筛选后的障碍物的像素信息转换为距离信息-------------
    for (i = 0; i < FilterCount; i++)
    {
        //单位cm转换成mm
        if (FilterLidarDistPix[i] - dis_50 < 75)
            FilterLidarDistance[i] = 10 * (a11 * pow(a12 * (FilterLidarDistPix[i] - dis_50), a13) + 50);

        if (FilterLidarDistPix[i] - dis_50 >= 75 && FilterLidarDistPix[i] - dis_50 < 146)
            FilterLidarDistance[i] = 10 * (a21 * pow(a22 * (FilterLidarDistPix[i] - dis_50), a23) + 50);

        if (FilterLidarDistPix[i] - dis_50 >= 146 && FilterLidarDistPix[i] - dis_50 < 164)
            FilterLidarDistance[i] = 10 * (a31 * pow(a32 * (FilterLidarDistPix[i] - dis_50), a33) + 50);

        if (FilterLidarDistPix[i] - dis_50 >= 164)
            FilterLidarDistance[i] = 10 * (a41 * pow(a42 * (FilterLidarDistPix[i] - dis_50), a43) + 50);
        //printf("FilterLidarDistance[%d]=%d\n",i,FilterLidarDistance[i]);
    }
    if (FilterCount2)
    {
        tmpmindistance = FilterLidarDistance[0];
        tmpmaxdistance = FilterLidarDistance[0];

        for (i = 0; i < FilterCount2; i++)
        {
            tmpmindistance = min(tmpmindistance, FilterLidarDistance[i]);
            tmpmaxdistance = max(tmpmaxdistance, FilterLidarDistance[i]);
        }
    }
    //--------------用lidar数据初步判断障碍物的边界--------------------------

    for (i = 0; i < FilterCount - 1; i++)
    {
        if (FilterLidarAng[i + 1] - FilterLidarAng[i] > 100)
        {
            LidarEdgeAngle[LidarEdgeCnt] = FilterLidarAng[i];
            LidarEdgeDistPix[LidarEdgeCnt] = FilterLidarDistPix[i];
            LidarEdgeCnt++;

            LidarEdgeAngle[LidarEdgeCnt] = FilterLidarAng[i + 1];
            LidarEdgeDistPix[LidarEdgeCnt] = FilterLidarDistPix[i + 1];
            LidarEdgeCnt++;
        }
    }

    for (i = 0; i < LidarEdgeCnt; i++)
    {
        r = LidarEdgeDistPix[i];

        for (d = -8; d < 8; d++)
        {
            if (LidarEdgeAngle[i] < 900)
            {
                y = r * sin(LidarEdgeAngle[i] * 3.14 / 1800) + 1 + 5 * cos(LidarEdgeAngle[i] * 3.14 / 1800) + d;
                x = r * cos(LidarEdgeAngle[i] * 3.14 / 1800) - 5;
            }
            if (LidarEdgeAngle[i] > 900)
            {
                y = r * sin(LidarEdgeAngle[i] * 3.14 / 1800) + 1 + d;
                x = r * cos(LidarEdgeAngle[i] * 3.14 / 1800) + 5 * sin(LidarEdgeAngle[i] * 3.14 / 1800);
            }

            //addr[cell(x,y,0)]=0;
            //addr[cell(x,y,1)]=0;
            //addr[cell(x,y,2)]=255;
        }

        //printf("LidarEdgeCnt=%d\n",LidarEdgeCnt);
        //printf("LidarEdgeAngle[%d]=%d\n",i,LidarEdgeAngle[i]);
        //printf("LidarEdgeDistPix[%d]=%d\n",i,LidarEdgeDistPix[i]);
        //printf("\n");
    }
    //-------------根据获取的边界信息提取可行路径-----------

    //---像素信息转换为距离信息，筛选可行路径----
    for (i = 0; i < LidarEdgeCnt; i++)
    {

        if (LidarEdgeDistPix[i] - dis_50 < 75)
            LidarEdgeDistance[i] = a11 * pow(a12 * (LidarEdgeDistPix[i] - dis_50), a13) + 50;

        if (LidarEdgeDistPix[i] - dis_50 >= 75 && LidarEdgeDistPix[i] - dis_50 < 146)
            LidarEdgeDistance[i] = a21 * pow(a22 * (LidarEdgeDistPix[i] - dis_50), a23) + 50;

        if (LidarEdgeDistPix[i] - dis_50 >= 146 && LidarEdgeDistPix[i] - dis_50 < 164)
            LidarEdgeDistance[i] = a31 * pow(a32 * (LidarEdgeDistPix[i] - dis_50), a33) + 50;

        if (LidarEdgeDistPix[i] - dis_50 >= 164)
            LidarEdgeDistance[i] = a41 * pow(a42 * (LidarEdgeDistPix[i] - dis_50), a43) + 50;
        //	printf("LidarEdgeDistance[%d]=%d\n",i,LidarEdgeDistance[i]);//厘米
    }
    //---判断是否可行-------
    OkPathCnt = 0;
    memset(OkPathDistPix, 0, 10);
    memset(OkPathAngle, 0, 10);
    memset(OkPathDistance, 0, 10);

    PathWidth = 0.0;

    //-----------------------第二层筛选判断是否为可行路径------
    if (LidarEdgeCnt > 1)
    {
        for (i = 0; i < LidarEdgeCnt - 1; i++)
        {
            if (i % 2 == 0)
            {
                theta = abs(LidarEdgeAngle[i] - LidarEdgeAngle[i + 1]) * 3.14 / 1800;
                /*******如果障碍物之间的空隙超过40像素值那就说明这个区间可以让机器人通过********************/
                if (LidarEdgeDistance[i] * LidarEdgeDistance[i] + LidarEdgeDistance[i + 1] * LidarEdgeDistance[i + 1] - 2 * LidarEdgeDistance[i] * LidarEdgeDistance[i + 1] * cos(theta) > 1600.0)
                {
                    OkPathAngle[OkPathCnt] = LidarEdgeAngle[i];
                    OkPathDistPix[OkPathCnt] = LidarEdgeDistPix[i];
                    OkPathDistance[OkPathCnt] = LidarEdgeDistance[i];
                    OkPathCnt++;

                    OkPathAngle[OkPathCnt] = LidarEdgeAngle[i + 1];
                    OkPathDistPix[OkPathCnt] = LidarEdgeDistPix[i + 1];
                    OkPathDistance[OkPathCnt] = LidarEdgeDistance[i + 1];
                    OkPathCnt++;

                    PathWidth = LidarEdgeDistance[i] * LidarEdgeDistance[i] + LidarEdgeDistance[i + 1] * LidarEdgeDistance[i + 1] - 2 * LidarEdgeDistance[i] * LidarEdgeDistance[i + 1] * cos(theta);
                    PathWidth = pow(PathWidth, 0.5);

                    //	printf("PathWidth=%6.2f\n",PathWidth);
                    //	printf("path theta=%6.2f\n",theta*1800/3.14);
                    //	printf("path width=%6.2f\n",LidarEdgeDistance[i]*LidarEdgeDistance[i]+LidarEdgeDistance[i+1]*LidarEdgeDistance[i+1]-2*LidarEdgeDistance[i]*LidarEdgeDistance[i+1]*cos(theta));
                    //	printf("\n");
                }
                //printf("theta=%f\n",theta*1800/3.14);
                //printf("path width=%f\n",LidarEdgeDistance[i]*LidarEdgeDistance[i]+LidarEdgeDistance[i+1]*LidarEdgeDistance[i+1]-2*LidarEdgeDistance[i]*LidarEdgeDistance[i+1]*cos(theta));
            }
        }
    }
    //-----------------取筛选后可行路径的中点---------------------
    if (OkPathCnt)
    {
        for (i = 0; i < OkPathCnt - 1; i++)
        {
            if (i % 2 == 0)
            {
                LidarOkPathMidAng[LidarOkPathMidCnt] = (OkPathAngle[i] + OkPathAngle[i + 1]) / 2;
                LidarOkPathMidPix[LidarOkPathMidCnt] = (OkPathDistPix[i] + OkPathDistPix[i + 1]) / 2;
                LidarOkPathMidCnt++;
            }
            //printf("LidarOkPathMidAng[%d]=%d\n",i,LidarOkPathMidAng[i]);
            //printf("LidarOkPathMidPix[%d]=%d\n",i,LidarOkPathMidPix[i]);
            //printf("\n");
        }
    }

    //-----lidar扫描的可行路径中点按照与目标物的偏差排序------
    if (LidarOkPathMidCnt)
    {
        int tmplidarang = 0;
        int tmplidarpix = 0;
        if (LidarOkPathMidCnt > 1)
        {
            for (i = 0; i < LidarOkPathMidCnt - 1; i++)
            {
                for (j = i + 1; j < LidarOkPathMidCnt; j++)
                {
                    if (abs(LidarOkPathMidAng[i] - robot_temp[1].angle) > abs(LidarOkPathMidAng[j] - robot_temp[1].angle))
                    {
                        tmplidarang = LidarOkPathMidAng[i];
                        LidarOkPathMidAng[i] = LidarOkPathMidAng[j];
                        LidarOkPathMidAng[j] = tmplidarang;

                        tmplidarpix = LidarOkPathMidPix[i];
                        LidarOkPathMidPix[i] = LidarOkPathMidPix[j];
                        LidarOkPathMidPix[j] = tmplidarpix;
                    }
                }
            }
        }

        //	printf("LidarOkPathMidAng[0]=%d\n",LidarOkPathMidAng[0]);
        //	printf("LidarOkPathMidPix[0]=%d\n",LidarOkPathMidPix[0]);
        //	printf("\n");
    }

    //-------判断机器人到总目标的直线路径上有无障碍物---

    /*     if (FilterCount)
    {
        if (tmpminangsub > 50)
            isArrivedDirect_Flag = 1;
    } */

    //--------------------RRT产生一个随机路径----------------------
    //	srand((unsigned)time(NULL));
    //pix[]用于在屏幕上描绘rrt路径
    pix[0] = Step;
    stepdis[0] = StepDistance;
    //NumPathPlanningCnt=0;
    //PathPlanningAng[6][256][10]={0};
    //PathPlanningPix[6][256][10]={0};

    if (FilterCount && FilterCount2 /*&& (!isArrivedDirect_Flag)*/)
    {
        OkPath.Ok_Cnt_PerLayer = 0;
        Best_OkPath.Best_Ok_Cnt = 0;
        //srand((int)time(NULL));
        //srand(time_srand);
        if (!isOkPath)
        {
            for (k = 0; k < Layer; k++) //师兄写的是i，事实上i会不断变成0，无法使i递增，改为K
            {
                int tmp = 0;
                fd_rand = open("/dev/urandom", O_RDONLY); //  /dev/urandom设备可实现随机数产生
                printf("RandRito=%d\n", RandRito);
                PathPlanningCnt = 0;

                for (j = 0; j < Node_PerNum; j++)
                {
                    for (i = 0; i < branch_num; i++)
                    {
                        //randangle[childrandcnt][i]=rand()%1200+300;
                        read(fd_rand, randangle[childrandcnt][j] + i, sizeof(int));                      //二维数组a[i]表示 第i行首地址
                        randangle[childrandcnt][j][i] = abs(randangle[childrandcnt][j][i]) % 1200 + 300; //30-150度搜索
                                                                                                         //	printf("randangle[%d][%d]=%d\n",childrandcnt,i,randangle[childrandcnt][i]%1200+300);
                    }
                }

                for (j = 0; j < Node_PerNum; j++)
                {
                    PathPlanningCnt = 0;
                    for (i = 0; i < branch_num; i++)
                    {
                        PathPlanningAng[NumPathPlanningCnt][j][PathPlanningCnt] = randangle[childrandcnt][j][i];

                        PathPlanningCnt++;
                    }
                }

                // printf("pix[%d]=%d\n", planning_cnt, pix[planning_cnt]);
                childrandcnt++;
                NumPathPlanningCnt++;
                RandRito = RandRito * 2;
                printf("NumPathPlanningCnt = %d\n", NumPathPlanningCnt);
                tmp = pix[planning_cnt];
                planning_cnt++;
                pix[planning_cnt] = tmp + pix[0];

                isOkPath = 1;
                close(fd_rand);
            }

            if (NumPathPlanningCnt < Layer)
            {
                Node_PerNum = pow(branch_num, Circle_num + 1); //需要拓展的搜索节点
                printf("Node_PerNum=%d\n", Node_PerNum);
                isOkPath = 0;
                Circle_num++;
            }
        }

        /*
		for(m=0;m<Node_PerNum;m++)
		{
			for(i=0;i<branch_num;i++)
			{
				printf("PathPlanningAng[3][%d][%d]=%d\n",m,i,PathPlanningAng[3][m][i]);
			}
		}*/

        //	printf("\n");

        //printf("NumPathPlanningCnt=%d\n",NumPathPlanningCnt);
        //printf("isOkPath=%d\n",isOkPath);
        //printf("randangle=%d\n",randangle);

        //--------------------------------------------------------------------------//
        //-------------将步长的距离信息转换成像素信息-------------//
        //--------------------------------------------------------------------------//

        /*	if((CurLidarDist>(618+300)) && (CurLidarDist<(995+300)))
		{
			CurLidarDistPix2=9.9210*pow(0.0918*((CurLidarDist-430)),0.5322)+41;			
		}
		if((CurLidarDist>(995+300)) && (CurLidarDist<(2019+300)))
		{
			CurLidarDistPix2=2.7994*pow(2.4136*((CurLidarDist-430)),0.4712)+41;			
		}
		
*/

        //-------------判断RRT产生的随机路径是否可行-------------
        //---分层判断，如果可行继续拓展，否则停止拓展----

        //---第一层判断，遇到不可行分支，该分支------------
        //---------停止搜索，之后所有搜索树置0------------

#if 1
        //---第四层判断，遇到不可行分支，该分支------------
        //---------停止搜索，之后所有搜索树置0------------
        if (NumPathPlanningCnt > 3)
        {
            int tmpangle3 = 0;
            int tmpdistance3 = 0;
            float tmp_dx3 = 0.0;
            float tmp_dy3 = 0.0;

            int tmp_path4_ang[4] = {0};
            int tmp_path4_new_ang[4] = {0};

            float tmp_path4_dist_x[4] = {0.0};

            float tmp_path4_dist_y[4] = {0.0};
            float tmp_path4_dist[4] = {0.0};

            for (m = 0; m < branch_num * branch_num * branch_num; m++)
            {
                for (i = 0; i < branch_num; i++) //第四层的搜索树
                {
                    //---每一层的角度---------
                    //printf("m = %d,\n",m);
                    /* 
                        其中，m = Node_PerNum，这是由Node_PerNum决定的,Node_PerNum的值范围是1-27
                        假设Node_PerNum = 9;那么m从0-8有值，9-26的部分对应的角度值和距离值就是0；
                        则i = 3(0,1,2) m = 9;
                        从第四层一共有i*m个节点     第四层一共由27个节点
                        第三层一共有i*m/3个节点     第三层一共有9个节点
                        第二层一共有i*m/9个节点     第二层一共有3个节点
                        第一层一共有i个节点         第一层一共有3个节点 （第一层永远只有三个节点）
                        
                     */
                    tmp_path4_ang[0] = PathPlanningAng[0][0][i];
                    tmp_path4_ang[1] = PathPlanningAng[1][m / (branch_num * branch_num)][i];
                    tmp_path4_ang[2] = PathPlanningAng[2][m / branch_num][i];
                    tmp_path4_ang[3] = PathPlanningAng[3][m][i];

                    tmp_path4_dist_x[0] = StepDistance * cos(tmp_path4_ang[0] * 3.14 / 1800);
                    tmp_path4_dist_y[0] = StepDistance * sin(tmp_path4_ang[0] * 3.14 / 1800);
                    tmp_path4_dist[0] = tmp_path4_dist_x[0] * tmp_path4_dist_x[0] + tmp_path4_dist_y[0] * tmp_path4_dist_y[0];
                    tmp_path4_dist[0] = pow(tmp_path4_dist[0], 0.5);

                    tmp_path4_dist_x[1] = tmp_path4_dist_x[0] + StepDistance * cos(tmp_path4_ang[1] * 3.14 / 1800);
                    tmp_path4_dist_y[1] = tmp_path4_dist_y[0] + StepDistance * sin(tmp_path4_ang[1] * 3.14 / 1800);
                    tmp_path4_dist[1] = tmp_path4_dist_x[1] * tmp_path4_dist_x[1] + tmp_path4_dist_y[1] * tmp_path4_dist_y[1];
                    tmp_path4_dist[1] = pow(tmp_path4_dist[1], 0.5);

                    tmp_path4_dist_x[2] = tmp_path4_dist_x[1] + StepDistance * cos(tmp_path4_ang[2] * 3.14 / 1800);
                    tmp_path4_dist_y[2] = tmp_path4_dist_y[1] + StepDistance * sin(tmp_path4_ang[2] * 3.14 / 1800);
                    tmp_path4_dist[2] = tmp_path4_dist_x[2] * tmp_path4_dist_x[2] + tmp_path4_dist_y[2] * tmp_path4_dist_y[2];
                    tmp_path4_dist[2] = pow(tmp_path4_dist[2], 0.5);

                    tmp_path4_dist_x[3] = tmp_path4_dist_x[2] + StepDistance * cos(tmp_path4_ang[3] * 3.14 / 1800);
                    tmp_path4_dist_y[3] = tmp_path4_dist_y[2] + StepDistance * sin(tmp_path4_ang[3] * 3.14 / 1800);
                    tmp_path4_dist[3] = tmp_path4_dist_x[3] * tmp_path4_dist_x[3] + tmp_path4_dist_y[3] * tmp_path4_dist_y[3];
                    tmp_path4_dist[3] = pow(tmp_path4_dist[3], 0.5);

                    for (n = 0; n < 4; n++)
                    {
                        //------每个步长终点相对于起点的位姿--------
                        tmp_path4_new_ang[n] = acos(tmp_path4_dist_x[n] / tmp_path4_dist[n]) * 1800 / 3.14;
                    }
                    //tmp_dx3代表x的总长度，tmp_dy3代表y的总长度
                    tmp_dx3 = StepDistance * (cos(PathPlanningAng[0][0][i] * 3.14 / 1800) + cos(PathPlanningAng[1][m / (branch_num * branch_num)][i] * 3.14 / 1800) + cos((PathPlanningAng[2][m / branch_num][i]) * 3.14 / 1800) + cos((PathPlanningAng[3][m][i]) * 3.14 / 1800));
                    tmp_dy3 = StepDistance * (sin(PathPlanningAng[0][0][i] * 3.14 / 1800) + sin(PathPlanningAng[1][m / (branch_num * branch_num)][i] * 3.14 / 1800) + sin((PathPlanningAng[2][m / branch_num][i]) * 3.14 / 1800) + sin((PathPlanningAng[3][m][i]) * 3.14 / 1800));

                    //tmp_dy3=pix[0]*sin(PathPlanningAng[0][0][i]*3.14/1800)+pix[0]*sin(PathPlanningAng[1][m/(branch_num*branch_num)][i]*3.14/1800)+pix[0]*sin(PathPlanningAng[2][m/branch_num][i]*3.14/1800)+pix[0]*sin(PathPlanningAng[3][m][i]*3.14/1800);
                    //tmp_dx3=pix[0]*cos(PathPlanningAng[0][0][i]*3.14/1800)+pix[0]*cos(PathPlanningAng[1][m/(branch_num*branch_num)][i]*3.14/1800)+pix[0]*cos(PathPlanningAng[2][m/branch_num][i]*3.14/1800)+pix[0]*cos(PathPlanningAng[3][m][i]*3.14/1800);
                    //tmpangle3是原点到第四节点的角度
                    tmpangle3 = atan(tmp_dy3 / tmp_dx3) * 1800 / 3.14;
                    if (tmpangle3 < 0)
                    {
                        tmpangle3 = tmpangle3 + 1800;
                    }
                    //	printf("tmp_dy3=%6.2f\n",tmp_dy3);
                    //	printf("tmp_dx3=%6.2f\n",tmp_dx3);
                    //斜边长
                    tmpdistance3 = tmp_dy3 * tmp_dy3 + tmp_dx3 * tmp_dx3;
                    tmpdistance3 = pow(tmpdistance3, 0.5);

                    //	printf("tmpangle3=%d\n",tmpangle3);
                    //	printf("tmppix3=%d\n",tmppix3);
                    //	printf("\n");

                    //--------障碍物膨胀处理------------------------------------
                    //----保证每一个子段都不能碰到障碍物--------------
                    for (j = 0; j < FilterCount2; j++)
                    {
                        //------tmp_path4_new_ang[0]每个步长终点相对于起点的位姿--------
                        /* ********如果发现障碍物与机器人方向不超过15度且距离小于20厘米，就放弃这条rrt路径*************************** */
                        if (abs(FilterLidarAng2[j] - tmp_path4_new_ang[0]) < ObsDeltaAng) //先判断障碍物角度
                        {
                            if (FilterLidarDistance[j] < tmp_path4_dist[0] + ExpandDis)
                            {
                                PathPlanningAng[0][0][i] = 0;
                            }
                        }
                        if (abs(FilterLidarAng2[j] - tmp_path4_new_ang[1]) < ObsDeltaAng) //先判断障碍物角度
                        {
                            if (FilterLidarDistance[j] < tmp_path4_dist[1] + ExpandDis)
                            {
                                PathPlanningAng[1][m / (branch_num * branch_num)][i] = 0;
                            }
                        }
                        if (abs(FilterLidarAng2[j] - tmp_path4_new_ang[2]) < ObsDeltaAng) //先判断障碍物角度
                        {
                            if (FilterLidarDistance[j] < tmp_path4_dist[2] + ExpandDis)
                            {
                                PathPlanningAng[2][m / branch_num][i] = 0;
                            }
                        }
                        if (abs(FilterLidarAng2[j] - tmp_path4_new_ang[3]) < ObsDeltaAng) //先判断障碍物角度
                        {
                            if (FilterLidarDistance[j] < tmp_path4_dist[3] + ExpandDis)
                            {
                                PathPlanningAng[3][m][i] = 0;
                            }
                        }
                    }

                    //	if(PathPlanningAng[0][0][i] && PathPlanningAng[3][m][i] && 4==Layer)
                    //角度不为0 表示满足要求的
                    if (PathPlanningAng[3][m][i] && PathPlanningAng[2][m / branch_num][i] && PathPlanningAng[1][m / (branch_num * branch_num)][i] && PathPlanningAng[0][0][i] && 4 == Layer) //	if(PathPlanningAng[3][m][i] && 4==Layer)
                    {
                        OkPath.Layer_Num = 4;
                        OkPath.Ok_Angle[OkPath.Ok_Cnt_PerLayer] = tmpangle3;
                        //OkPath.Ok_Pix[OkPath.Ok_Cnt_PerLayer]=tmppix3;
                        OkPath.Ok_Distance[OkPath.Ok_Cnt_PerLayer] = tmpdistance3;
                        for (p = 0; p < 4; p++)
                        {
                            OkPath.Ok_PerLayer_Angle[OkPath.Ok_Cnt_PerLayer][p] = tmp_path4_ang[p];
                        }
                        //memcpy(OkPath.Ok_PerLayer_Angle,tmp_path4_ang,sizeof(tmp_path4_ang));
                        //printf("tmppix3=%d\n",tmppix3);
                        //printf("OkPath.Ok_Pix=%d\n",OkPath.Ok_Pix[OkPath.Ok_Cnt_PerLayer]);
                        OkPath.Ok_Cnt_PerLayer++;
                    }
                    //	printf("\n");
                }
            }
            if (4 == Layer && !RRT_Ok_Flag)
            {
                // ------将4层搜索树可行路径与目标角度差值排序--------

                // ------将4层搜索树可行路径与目标距离差值排序--------
                //-------------代价函数的计算-------------------------------------
                /* 评估函数中代价函数 */
                for (i = 0; i < OkPath.Ok_Cnt_PerLayer - 1; i++)
                {
                    for (j = i + 1; j < OkPath.Ok_Cnt_PerLayer; j++)
                    {
                        if (goalListLeader(&dis, i, j, p) != 0)
                        {
                            printf("goalListLeader error\n");
                        }
                    }
                }
                Best_OkPath.Best_Layer_Num = 4;
                for (i = 0; i < OkPath.Ok_Cnt_PerLayer; i++)
                {
                    //----路径选择条件:需要优化--------
                    //if(OkPath.Ok_Pix[i]>tmpminpix  && (((OkPath.Ok_Angle[i]>OkPathAngle[0])&&(OkPath.Ok_Angle[i]<OkPathAngle[1])) || ((OkPath.Ok_Angle[i]>OkPathAngle[2])&&(OkPath.Ok_Angle[i]<OkPathAngle[3]))))
                    //if(OkPath.Ok_Distance[i]>(tmpmaxdistance+tmpmindistance)/2  && ((OkPath.Ok_Angle[i]>OkPathAngle[0])&&(OkPath.Ok_Angle[i]<OkPathAngle[1])))
                    /********如果规划的RRT斜边长大于最近的障碍物距离*************/
                    if (OkPath.Ok_Distance[i] > tmpmindistance)
                    {
                        Best_OkPath.Best_Ok_Angle[Best_OkPath.Best_Ok_Cnt] = OkPath.Ok_Angle[i];
                        //Best_OkPath.Best_Ok_Pix[Best_OkPath.Best_Ok_Cnt] = OkPath.Ok_Pix[i];
                        Best_OkPath.Best_Ok_Distance[Best_OkPath.Best_Ok_Cnt] = OkPath.Ok_Distance[i];
                        for (p = 0; p < 4; p++)
                        {
                            Best_OkPath.Best_Ok_PerLayer_Ang[Best_OkPath.Best_Ok_Cnt][p] = OkPath.Ok_PerLayer_Angle[i][p];
                        }
                        Best_OkPath.Best_Ok_Cnt++;
                        // i = OkPath.Ok_Cnt_PerLayer - 1;
                    }
                    // printf("i = %d\n", i);
                }

                for (i = 0; i < Best_OkPath.Best_Ok_Cnt; i++)
                {
                    for (j = 0; j < 4; j++)
                    {
                        Cord_PerLayer.Cordinate_Ang[j][0] = Best_OkPath.Best_Ok_PerLayer_Ang[i][j];
                        printf("Cordinate_Ang[%d][0]=%d\n", j, Cord_PerLayer.Cordinate_Ang[j][0]);
                    }
                    //printf("Best_Ok_PerLayer_Ang[%d][%d]=%d\n",i,j,Best_OkPath.Best_Ok_PerLayer_Ang[i][j]);

                    printf("\n");
                }
                /***********stepdis[0] = 300*******************/
                Cord_PerLayer.Cordinate_X[0][0] = stepdis[0] * cos(Cord_PerLayer.Cordinate_Ang[0][0] * 3.14 / 1800);
                Cord_PerLayer.Cordinate_Y[0][0] = stepdis[0] * sin(Cord_PerLayer.Cordinate_Ang[0][0] * 3.14 / 1800);

                Cord_PerLayer.Cordinate_X[1][0] = stepdis[0] * (cos(Cord_PerLayer.Cordinate_Ang[0][0] * 3.14 / 1800) + cos(Cord_PerLayer.Cordinate_Ang[1][0] * 3.14 / 1800));
                Cord_PerLayer.Cordinate_Y[1][0] = stepdis[0] * (sin(Cord_PerLayer.Cordinate_Ang[0][0] * 3.14 / 1800) + sin(Cord_PerLayer.Cordinate_Ang[1][0] * 3.14 / 1800));

                Cord_PerLayer.Cordinate_X[2][0] = stepdis[0] * (cos(Cord_PerLayer.Cordinate_Ang[0][0] * 3.14 / 1800) + cos(Cord_PerLayer.Cordinate_Ang[1][0] * 3.14 / 1800) + cos(Cord_PerLayer.Cordinate_Ang[2][0] * 3.14 / 1800));
                Cord_PerLayer.Cordinate_Y[2][0] = stepdis[0] * (sin(Cord_PerLayer.Cordinate_Ang[0][0] * 3.14 / 1800) + sin(Cord_PerLayer.Cordinate_Ang[1][0] * 3.14 / 1800) + sin(Cord_PerLayer.Cordinate_Ang[2][0] * 3.14 / 1800));

                Cord_PerLayer.Cordinate_X[3][0] = stepdis[0] * (cos(Cord_PerLayer.Cordinate_Ang[0][0] * 3.14 / 1800) + cos(Cord_PerLayer.Cordinate_Ang[1][0] * 3.14 / 1800) + cos(Cord_PerLayer.Cordinate_Ang[2][0] * 3.14 / 1800) + cos(Cord_PerLayer.Cordinate_Ang[3][0] * 3.14 / 1800));
                Cord_PerLayer.Cordinate_Y[3][0] = stepdis[0] * (sin(Cord_PerLayer.Cordinate_Ang[0][0] * 3.14 / 1800) + sin(Cord_PerLayer.Cordinate_Ang[1][0] * 3.14 / 1800) + sin(Cord_PerLayer.Cordinate_Ang[2][0] * 3.14 / 1800) + sin(Cord_PerLayer.Cordinate_Ang[3][0] * 3.14 / 1800));
                /*                 printf("Cord_PerLayer.Cordinate_X[0][0] = %f\n", Cord_PerLayer.Cordinate_X[0][0]);
                printf("Cord_PerLayer.Cordinate_Y[0][0] = %f\n", Cord_PerLayer.Cordinate_Y[0][0]);
                printf("Cord_PerLayer.Cordinate_X[1][0] = %f\n", Cord_PerLayer.Cordinate_X[1][0]);
                printf("Cord_PerLayer.Cordinate_X[1][0] = %f\n", Cord_PerLayer.Cordinate_Y[1][0]); */
                //--------赋值用于机器人路径------------------------------
                //Cord_PerLayer.Cordinate_Ang[0][0]=Best_OkPath.Best_Ok_PerLayer_Ang[Best_OkPath.Best_Ok_Cnt][0];

                //------判断规划好的路径上是否出现新的障碍物----
                //--若出现新的障碍，重新规划路径---------------------
                /*	for(i=0;i<FilterCount;i++)	
				{
					//r=FilterLidarDistPix[i];
					if(abs(Best_OkPath.Best_Ok_Angle[0]-FilterLidarAng[i])<20 && Best_OkPath.Best_Ok_Pix[0]>FilterLidarDistPix[i])	
					{
						printf("need planning path again........\n");

						Circle_num=0;
						childrandcnt=0;				
						planning_cnt=0;
						RandRito=1;
						
						NumPathPlanningCnt=0;
						memset(PathPlanningAng,0,sizeof(int)*6*100*10);
						memset(PathPlanningPix,0,sizeof(int)*6*100*10);
						isOkPath=0;						
					}
				}
				
			}*/

                //-----判断机器人开始运动前是否有可行的RRT路径---
                //------没有就重新规划-----------------------------------------
                printf("Best_OkPath.Best_Ok_Cnt = %d\n", Best_OkPath.Best_Ok_Cnt);
                if (!Best_OkPath.Best_Ok_Cnt)
                {
                    RRT_Ok_Flag = 0;

                    Circle_num = 0;
                    childrandcnt = 0;
                    planning_cnt = 0;
                    RandRito = 1;

                    NumPathPlanningCnt = 0;
                    memset(PathPlanningAng, 0, sizeof(int) * 6 * 100 * 10);
                    memset(PathPlanningPix, 0, sizeof(int) * 6 * 100 * 10);
                    isOkPath = 0;
                }
                if (Best_OkPath.Best_Ok_Cnt)
                {
                    RRT_Ok_Flag = 1;
                    isArrivedDirect_Flag = 1;
                }
            }
        }
        //printf("RRT_Ok_Flag=%d\n",RRT_Ok_Flag);
        if (Arrived_ChildAim_Flag)
        {
            RRT_Ok_Flag = 0;

            Circle_num = 0;
            childrandcnt = 0;
            planning_cnt = 0;
            RandRito = 1;

            NumPathPlanningCnt = 0;
            memset(PathPlanningAng, 0, sizeof(int) * 6 * 100 * 10);
            memset(PathPlanningPix, 0, sizeof(int) * 6 * 100 * 10);
            isOkPath = 0;
            Arrived_ChildAim_Flag = 0;
        }

        //---第五层判断，遇到不可行分支，该分支------------
        //---------停止搜索，之后所有搜索树置0------------

        if (NumPathPlanningCnt > 4)
        {
            int tmpangle4 = 0;
            int tmppix4 = 0;
            float tmp_dx4 = 0.0;
            float tmp_dy4 = 0.0;

            for (m = 0; m < branch_num * branch_num * branch_num * branch_num; m++)
            {
                for (i = 0; i < branch_num; i++) //第四层的搜索树
                {
                    tmp_dy4 = pix[0] * sin(PathPlanningAng[0][0][i] * 3.14 / 1800) + pix[0] * sin(PathPlanningAng[1][m / (branch_num * branch_num * branch_num)][i] * 3.14 / 1800) + pix[0] * sin(PathPlanningAng[2][m / branch_num * branch_num][i] * 3.14 / 1800) + pix[0] * sin(PathPlanningAng[3][m / branch_num][i] * 3.14 / 1800) + pix[0] * sin(PathPlanningAng[4][m][i] * 3.14 / 1800);
                    tmp_dx4 = pix[0] * cos(PathPlanningAng[0][0][i] * 3.14 / 1800) + pix[0] * cos(PathPlanningAng[1][m / (branch_num * branch_num * branch_num)][i] * 3.14 / 1800) + pix[0] * cos(PathPlanningAng[2][m / branch_num * branch_num][i] * 3.14 / 1800) + pix[0] * cos(PathPlanningAng[3][m / branch_num][i] * 3.14 / 1800) + pix[0] * cos(PathPlanningAng[4][m][i] * 3.14 / 1800);
                    tmpangle4 = atan(tmp_dy4 / tmp_dx4) * 1800 / 3.14;
                    if (tmpangle4 < 0)
                    {
                        tmpangle4 = tmpangle4 + 1800;
                    }
                    //	printf("tmp_dy3=%6.2f\n",tmp_dy3);
                    //	printf("tmp_dx3=%6.2f\n",tmp_dx3);

                    tmppix4 = tmp_dy4 * tmp_dy4 + tmp_dx4 * tmp_dx4;
                    tmppix4 = pow(tmppix4, 0.5);

                    //	printf("tmpangle3=%d\n",tmpangle3);
                    //	printf("tmppix3=%d\n",tmppix3);
                    //	printf("\n");
                    for (j = 0; j < FilterCount; j++)
                    {
                        if (abs(FilterLidarAng[j] - tmpangle4) < ObsDeltaAng) //先判断障碍物角度
                        {
                            if (FilterLidarDistPix[j] < tmppix4)
                            {
                                for (l = 4; l < NumPathPlanningCnt; l++)
                                {
                                    PathPlanningAng[l][m][i] = 0;
                                }
                                obs_search_cnt++;
                            }
                        }
                    }
                    if (PathPlanningAng[4][m][i] && 5 == Layer) //角度不为0 表示满足要求的
                    {
                        OkPath.Layer_Num = 4;
                        OkPath.Ok_Angle[OkPath.Ok_Cnt_PerLayer] = tmpangle4;
                        OkPath.Ok_Pix[OkPath.Ok_Cnt_PerLayer] = tmppix4;
                        OkPath.Ok_Cnt_PerLayer++;
                    }
                }
            }
            if (5 == Layer)
            {
                int tmp5_angle = 0;
                int tmp5_pix = 0;

                // ------将5层搜索树可行路径与目标角度差值排序--------
                for (i = 0; i < OkPath.Ok_Cnt_PerLayer - 1; i++)
                {
                    for (j = i + 1; j < OkPath.Ok_Cnt_PerLayer; j++)
                    {
                        if (abs(OkPath.Ok_Angle[i] - robot_temp[1].angle) > abs(OkPath.Ok_Angle[j] - robot_temp[1].angle))
                        {
                            tmp5_angle = OkPath.Ok_Angle[i];
                            OkPath.Ok_Angle[i] = OkPath.Ok_Angle[j];
                            OkPath.Ok_Angle[j] = tmp5_angle;

                            tmp5_pix = OkPath.Ok_Pix[i];
                            OkPath.Ok_Pix[i] = OkPath.Ok_Pix[j];
                            OkPath.Ok_Pix[j] = tmp5_pix;
                        }
                    }
                }
                //	printf("robot_temp[1].angle=%d\n",robot_temp[1].angle);
                //	printf("OkPath.Ok_Angle[0]=%d\n",OkPath.Ok_Angle[0]);
                //	printf("OkPath.Ok_Pix[0]=%d\n",OkPath.Ok_Pix[0]);
                //	printf("\n");

                Best_OkPath.Best_Layer_Num = 5;
                for (i = 0; i < OkPath.Ok_Cnt_PerLayer; i++)
                {
                    if (OkPath.Ok_Pix[i] > tmpminpix)
                    {
                        Best_OkPath.Best_Ok_Angle[Best_OkPath.Best_Ok_Cnt] = OkPath.Ok_Angle[i];
                        Best_OkPath.Best_Ok_Pix[Best_OkPath.Best_Ok_Cnt] = OkPath.Ok_Pix[i];
                        Best_OkPath.Best_Ok_Cnt++;
                        i = OkPath.Ok_Cnt_PerLayer - 1;
                    }
                }
                printf("Best_OkPath.Best_Ok_Pix=%d\n", Best_OkPath.Best_Ok_Pix[0]);
                printf("tmpminpix=%d\n", tmpminpix);
                //printf("Best_OkPath.Best_Ok_Cnt=%d\n",Best_OkPath.Best_Ok_Cnt);
                //--------规划层数为5时画出满足的RRT路径-------------------------------
                for (i = 0; i < Best_OkPath.Best_Ok_Cnt; i++)
                {
                    r = Best_OkPath.Best_Ok_Pix[i];
                    for (j = 0; j < r; j++)
                    {
                        y = j * sin(Best_OkPath.Best_Ok_Angle[i] * 3.14 / 1800);
                        x = j * cos(Best_OkPath.Best_Ok_Angle[i] * 3.14 / 1800);

                        addr[cell(x, y, 0)] = 0;
                        addr[cell(x, y, 1)] = 0;
                        addr[cell(x, y, 2)] = 255;

                        //	printf("j=%d\n,y=%d,x=%d,r=%d",j,y,x,r);
                        //	printf("Best_OkPath.Best_Ok_Cnt=%d\n",Best_OkPath.Best_Ok_Cnt);
                        //	printf("...................\n");
                    }
                }
            }
        }
#endif

        //	printf("obs_search_cnt=%d\n",obs_search_cnt);
        obs_search_cnt = 0;

        //-------------------显示RRT随机路径-----------------------------
        if (0 == Display_Flag)
        {
            switch (NumPathPlanningCnt)
            {
            case 5:
                p = branch_num * branch_num * branch_num * branch_num;
                q = branch_num * branch_num * branch_num;
                l = branch_num * branch_num;
                for (m = 0; m < p; m++)
                {
                    for (i = 0; i < branch_num; i++)
                    {
                        for (j = pix[3]; j < pix[4]; j++)
                        {
                            y = (j - pix[3]) * sin((PathPlanningAng[4][m][i]) * 3.14 / 1800) + pix[0] * sin(PathPlanningAng[3][m / branch_num][i] * 3.14 / 1800) + pix[0] * sin(PathPlanningAng[2][m / l][i] * 3.14 / 1800) + pix[0] * sin(PathPlanningAng[1][m / q][i] * 3.14 / 1800) + pix[0] * sin(PathPlanningAng[0][0][i] * 3.14 / 1800);
                            x = (j - pix[3]) * cos((PathPlanningAng[4][m][i]) * 3.14 / 1800) + pix[0] * cos(PathPlanningAng[3][m / branch_num][i] * 3.14 / 1800) + pix[0] * cos(PathPlanningAng[2][m / l][i] * 3.14 / 1800) + pix[0] * cos(PathPlanningAng[1][m / q][i] * 3.14 / 1800) + pix[0] * cos(PathPlanningAng[0][0][i] * 3.14 / 1800);

                            addr[cell(x, y, 0)] = 255;
                            addr[cell(x, y, 1)] = 0;
                            addr[cell(x, y, 2)] = 0;
                        }
                    }
                }
            case 4:
                for (m = 0; m < branch_num * branch_num * branch_num; m++)
                {
                    for (i = 0; i < branch_num; i++)
                    {
                        Cord_PerLayer.Cordinate_X[3][i] = stepdis[0] * (cos(PathPlanningAng[0][0][i] * 3.14 / 1800) + cos(PathPlanningAng[1][m / (branch_num * branch_num)][i] * 3.14 / 1800) + cos((PathPlanningAng[2][m / branch_num][i]) * 3.14 / 1800) + cos((PathPlanningAng[3][m][i]) * 3.14 / 1800));
                        Cord_PerLayer.Cordinate_Y[3][i] = stepdis[0] * (sin(PathPlanningAng[0][0][i] * 3.14 / 1800) + sin(PathPlanningAng[1][m / (branch_num * branch_num)][i] * 3.14 / 1800) + sin((PathPlanningAng[2][m / branch_num][i]) * 3.14 / 1800) + sin((PathPlanningAng[3][m][i]) * 3.14 / 1800));
                        Cord_PerLayer.Cordinate_Ang[3][i] = PathPlanningAng[3][m][i];

                        for (j = pix[2]; j < pix[3]; j++)
                        {
                            y = (j - pix[2]) * sin((PathPlanningAng[3][m][i]) * 3.14 / 1800) + pix[0] * sin(PathPlanningAng[2][m / branch_num][i] * 3.14 / 1800) + pix[0] * sin(PathPlanningAng[1][m / (branch_num * branch_num)][i] * 3.14 / 1800) + pix[0] * sin(PathPlanningAng[0][0][i] * 3.14 / 1800);
                            x = (j - pix[2]) * cos((PathPlanningAng[3][m][i]) * 3.14 / 1800) + pix[0] * cos(PathPlanningAng[2][m / branch_num][i] * 3.14 / 1800) + pix[0] * cos(PathPlanningAng[1][m / (branch_num * branch_num)][i] * 3.14 / 1800) + pix[0] * cos(PathPlanningAng[0][0][i] * 3.14 / 1800);

                            addr[cell(x, y, 0)] = 255;
                            addr[cell(x, y, 1)] = 255;
                            addr[cell(x, y, 2)] = 0;
                        }
                    }
                }

            case 3:
                for (m = 0; m < branch_num * branch_num; m++)
                {
                    for (i = 0; i < branch_num; i++) //选取根节点
                    {
                        Cord_PerLayer.Cordinate_X[2][i] = stepdis[0] * (cos(PathPlanningAng[0][0][i] * 3.14 / 1800) + cos((PathPlanningAng[1][m / branch_num][i]) * 3.14 / 1800) + cos((PathPlanningAng[2][m][i]) * 3.14 / 1800));
                        Cord_PerLayer.Cordinate_Y[2][i] = stepdis[0] * (sin(PathPlanningAng[0][0][i] * 3.14 / 1800) + sin(PathPlanningAng[1][m / branch_num][i] * 3.14 / 1800) + sin((PathPlanningAng[2][m][i]) * 3.14 / 1800));
                        Cord_PerLayer.Cordinate_Ang[2][i] = PathPlanningAng[2][m][i];

                        for (j = pix[1]; j < pix[2]; j++) //绘制直线
                        {
                            y = (j - pix[1]) * sin((PathPlanningAng[2][m][i]) * 3.14 / 1800) + pix[0] * sin(PathPlanningAng[1][m / branch_num][i] * 3.14 / 1800) + pix[0] * sin(PathPlanningAng[0][0][i] * 3.14 / 1800);
                            x = (j - pix[1]) * cos((PathPlanningAng[2][m][i]) * 3.14 / 1800) + pix[0] * cos(PathPlanningAng[1][m / branch_num][i] * 3.14 / 1800) + pix[0] * cos(PathPlanningAng[0][0][i] * 3.14 / 1800);

                            addr[cell(x, y, 0)] = 255;
                            addr[cell(x, y, 1)] = 0;
                            addr[cell(x, y, 2)] = 0;
                        }
                    }
                }
                //break;
            case 2:
                for (m = 0; m < branch_num * 1; m++)
                {
                    for (i = 0; i < branch_num; i++) //第二层
                    {
                        Cord_PerLayer.Cordinate_X[1][i] = stepdis[0] * (cos(PathPlanningAng[0][0][i] * 3.14 / 1800) + cos((PathPlanningAng[1][m][i]) * 3.14 / 1800));
                        Cord_PerLayer.Cordinate_Y[1][i] = stepdis[0] * (sin(PathPlanningAng[0][0][i] * 3.14 / 1800) + sin(PathPlanningAng[1][m][i] * 3.14 / 1800));
                        Cord_PerLayer.Cordinate_Ang[1][i] = PathPlanningAng[1][m][i];

                        for (j = 0; j < pix[1]; j++)
                        {
                            y = (j)*sin((PathPlanningAng[1][m][i]) * 3.14 / 1800) + pix[0] * sin(PathPlanningAng[0][0][i] * 3.14 / 1800);
                            x = (j)*cos((PathPlanningAng[1][m][i]) * 3.14 / 1800) + pix[0] * cos(PathPlanningAng[0][0][i] * 3.14 / 1800);

                            addr[cell(x, y, 0)] = 0;
                            addr[cell(x, y, 1)] = 255;
                            addr[cell(x, y, 2)] = 0;
                        }
                    }
                }
                //break;
            case 1:

                for (i = 0; i < branch_num; i++)
                {

                    if ((stepdis[0] > 150) && (stepdis[0] < (618 + 300)))
                    {
                        pix[0] = 17.6132 * pow(0.0099 * ((stepdis[0] - 300)), 0.8712) + 41;
                    }

                    Cord_PerLayer.Cordinate_X[0][i] = stepdis[0] * cos(PathPlanningAng[0][0][i] * 3.14 / 1800);
                    Cord_PerLayer.Cordinate_Y[0][i] = stepdis[0] * sin(PathPlanningAng[0][0][i] * 3.14 / 1800);
                    Cord_PerLayer.Cordinate_Ang[0][i] = PathPlanningAng[0][0][i];

                    //	printf("Cord_PerLayer.Cordinate_X[0][%d]=%d\n",i,Cord_PerLayer.Cordinate_X[0][i]);
                    //	printf("Cord_PerLayer.Cordinate_Y[0][%d]=%d\n",i,Cord_PerLayer.Cordinate_Y[0][i]);
                    //	printf("\n");

                    for (j = 0; j < pix[0]; j++)
                    {
                        y = j * sin(PathPlanningAng[0][0][i] * 3.14 / 1800);
                        x = j * cos(PathPlanningAng[0][0][i] * 3.14 / 1800);

                        addr[cell(x, y, 0)] = 255;
                        addr[cell(x, y, 1)] = 0;
                        addr[cell(x, y, 2)] = 0;

                        //printf("pix[0]=%d\n", pix[0]);
                    }
                }
            case 0:
                break;
            }
        }

        /*	if(NumPathPlanningCnt<Layer)
		{				
			Node_PerNum=pow(branch_num,Circle_num+1);	//需要拓展的搜索节点
			printf("Node_PerNum=%d\n",Node_PerNum);
			isOkPath=0;	
			Circle_num++;
		}	
		*/
    }
    //--------规划层数为4时画出满足的RRT分段折线路径-------------------------------
#if 1

    if (Best_OkPath.Best_Ok_Cnt != 0)
    { //防止数据在循环中被清0，使用中间变量来承接rrt的数据，只有当新的数据来临的时候，中间变量才会更新
        Best_OkPath.showBest_Ok_Cnt = 0;
        memset(Best_OkPath.showBest_Ok_PerLayer_Ang, 0, sizeof(Best_OkPath.showBest_Ok_PerLayer_Ang));
        for (i = 0; i < Best_OkPath.Best_Ok_Cnt; i++)
        {
            for (j = 0; j < 4; j++)
            {
                Best_OkPath.showBest_Ok_PerLayer_Ang[i][j] = Best_OkPath.Best_Ok_PerLayer_Ang[i][j];
            }
            Best_OkPath.showBest_Ok_Cnt++;
        }
    }
    //使用中间变量来表达rrt路径
    for (i = 0; i < Best_OkPath.showBest_Ok_Cnt; i++)
    {
        for (j = 0; j < pix[0]; j++)
        {
            y = j * sin(Best_OkPath.showBest_Ok_PerLayer_Ang[i][0] * 3.14 / 1800);
            x = j * cos(Best_OkPath.showBest_Ok_PerLayer_Ang[i][0] * 3.14 / 1800);

            addr[cell(x, y, 0)] = 255;
            addr[cell(x, y, 1)] = 0;
            addr[cell(x, y, 2)] = 0;
        }
        for (j = 0; j < pix[0]; j++)
        {
            y = j * sin(Best_OkPath.showBest_Ok_PerLayer_Ang[i][1] * 3.14 / 1800) + pix[0] * sin(Best_OkPath.showBest_Ok_PerLayer_Ang[i][0] * 3.14 / 1800);
            x = j * cos(Best_OkPath.showBest_Ok_PerLayer_Ang[i][1] * 3.14 / 1800) + pix[0] * cos(Best_OkPath.showBest_Ok_PerLayer_Ang[i][0] * 3.14 / 1800);

            addr[cell(x, y, 0)] = 0;
            addr[cell(x, y, 1)] = 255;
            addr[cell(x, y, 2)] = 0;
        }
        for (j = 0; j < pix[0]; j++)
        {
            y = j * sin(Best_OkPath.showBest_Ok_PerLayer_Ang[i][2] * 3.14 / 1800) + pix[0] * (sin(Best_OkPath.showBest_Ok_PerLayer_Ang[i][1] * 3.14 / 1800) + sin(Best_OkPath.showBest_Ok_PerLayer_Ang[i][0] * 3.14 / 1800));
            x = j * cos(Best_OkPath.showBest_Ok_PerLayer_Ang[i][2] * 3.14 / 1800) + pix[0] * (cos(Best_OkPath.showBest_Ok_PerLayer_Ang[i][1] * 3.14 / 1800) + cos(Best_OkPath.showBest_Ok_PerLayer_Ang[i][0] * 3.14 / 1800));

            addr[cell(x, y, 0)] = 255;
            addr[cell(x, y, 1)] = 0;
            addr[cell(x, y, 2)] = 0;
        }
        for (j = 0; j < pix[0]; j++)
        {
            y = j * sin(Best_OkPath.showBest_Ok_PerLayer_Ang[i][3] * 3.14 / 1800) + pix[0] * (sin(Best_OkPath.showBest_Ok_PerLayer_Ang[i][2] * 3.14 / 1800) + sin(Best_OkPath.showBest_Ok_PerLayer_Ang[i][1] * 3.14 / 1800) + sin(Best_OkPath.showBest_Ok_PerLayer_Ang[i][0] * 3.14 / 1800));
            x = j * cos(Best_OkPath.showBest_Ok_PerLayer_Ang[i][3] * 3.14 / 1800) + pix[0] * (cos(Best_OkPath.showBest_Ok_PerLayer_Ang[i][2] * 3.14 / 1800) + cos(Best_OkPath.showBest_Ok_PerLayer_Ang[i][1] * 3.14 / 1800) + cos(Best_OkPath.showBest_Ok_PerLayer_Ang[i][0] * 3.14 / 1800));

            addr[cell(x, y, 0)] = 0;
            addr[cell(x, y, 1)] = 255;
            addr[cell(x, y, 2)] = 0;
        }
    }
#endif

    //--------显示可行路径的边界---------------------------------
    for (i = 0; i < OkPathCnt; i++)
    {
        r = OkPathDistPix[i];

        for (d = -8; d < 8; d++)
        {
            if (OkPathAngle[i] < 900)
            {
                y = r * sin(OkPathAngle[i] * 3.14 / 1800) + 1 + 5 * cos(OkPathAngle[i] * 3.14 / 1800) + d;
                x = r * cos(OkPathAngle[i] * 3.14 / 1800) - 5;
            }
            if (OkPathAngle[i] > 900)
            {
                y = r * sin(OkPathAngle[i] * 3.14 / 1800) + 1 + d;
                x = r * cos(OkPathAngle[i] * 3.14 / 1800) + 5 * sin(OkPathAngle[i] * 3.14 / 1800);
            }

            if (!startflag)
            {
                addr[cell(x, y, 0)] = 0;
                addr[cell(x, y, 1)] = 0;
                addr[cell(x, y, 2)] = 0;
            }
        }

        //	printf("OkPathCnt=%d\n",OkPathCnt/2);
        //	printf("OkPathAngle[%d]=%d\n",i,OkPathAngle[i]);
        //	printf("OkPathDistPix[%d]=%d\n",i,OkPathDistPix[i]);
        //	printf("\n");
    }

    //----------雷达扫到的近似边界点-------------------------------

    //PicEdgePixCnt=AheadCount;
    //memcpy(PicEdgePix,CurAheadPix,sizeof(CurAheadPix));
    //memcpy(PicEdgeAngle,CurAheadAngle,sizeof(CurAheadAngle));

    /*	if(AheadCount>1)
	{
		//printf("AheadCount=%d\n",AheadCount);	
		for(i=0;i<AheadCount;i++)
		{	
		
				tmp2=((3600+900-CurAheadAngle[i]) % 3600);
				printf("tmp2=%d\n",tmp2);

			for(d=-3;d<3;d++)
			{									
				r=CurAheadPix[i];
				y=r*sin(tmp2*3.14 / 1800)+d;
				x=r*cos(tmp2* 3.14  / 1800);
				addr[cell(x,y,0)]=255;
				addr[cell(x,y,1)]=0;
				addr[cell(x,y,2)]=0;
			}	
			//printf("CurAheadAngle=%d\n",CurAheadAngle[i]);
			//printf("CurAheadPix=%d\n",CurAheadPix[i]);
		}
	}
	*/
    //-------------------------------------------------------------------------

    AimCount = 0;
    memset(CurLidarAng, 0, 1024);
    memset(CurLidarDistPix, 0, 1024);

    TrueAimCount = 0;
    memset(TrueCurLidarDistPix, 0, 1024);
    memset(TrueCurLidarAng, 0, 1024);

    TrueAimCntDis = 0;
    memset(TrueCurLidarDistance, 0, 1024);
    memset(TrueCurLidarAng2, 0, 1024);

    FilterCount = 0;
    memset(FilterLidarDistPix, 0, 1024);
    memset(FilterLidarAng, 0, 1024);
    FilterCount2 = 0;
    memset(FilterLidarDistance, 0, 1024);
    memset(FilterLidarAng2, 0, 1024);

    LidarEdgeCnt = 0;
    memset(LidarEdgeDistPix, 0, 1024);
    memset(LidarEdgeDistance, 0, 1024);
    memset(LidarEdgeAngle, 0, 1024);

    LidarOkPathMidCnt = 0;
    memset(LidarOkPathMidAng, 0, 10);
    memset(LidarOkPathMidPix, 0, 10);

    //NumPathPlanningCnt=0;
    //memset(PathPlanningAng,0,sizeof(int)*6*256*10);
    //memset(PathPlanningPix,0,sizeof(int)*6*256*10);

    //---------------------------------------------------------------------------

    /*angle=0.01745*30;//0.5235
	for(r=0;r<275;r++)
	{
		y=r*sin(angle);
		x=r*cos(angle);
		addr[cell(x,y,0)]=255;
		addr[cell(x,y,1)]=255;
		addr[cell(x,y,2)]=255;
	}

	angle=0.01745*4+1.5708;//0.0698
	for(r=0;r<275;r++)
	{
		y=r*sin(angle);
		x=r*cos(angle);
		addr[cell(x,y,0)]=255;
		addr[cell(x,y,1)]=255;
		addr[cell(x,y,2)]=255;
	}
	*/
    //------------------------------------------------------------------------

    //------------------------屏幕上画圆-----------------------------------
    for (angle = 0; angle < 6.283; angle = angle + 0.1)
    {
        r = 20;
        y = r * sin(angle);
        x = r * cos(angle);
        addr[cell(x, y, 0)] = 255;
        addr[cell(x, y, 1)] = 255;
        addr[cell(x, y, 2)] = 255;

        r = 40;
        y = r * sin(angle);
        x = r * cos(angle);
        addr[cell(x, y, 0)] = 0;
        addr[cell(x, y, 1)] = 255;
        addr[cell(x, y, 2)] = 0;
    }

    for (angle = 0; angle < 6.283; angle = angle + 0.01)
    {
        r = 100;
        y = r * sin(angle);
        x = r * cos(angle);
        addr[cell(x, y, 0)] = 0;
        addr[cell(x, y, 1)] = 255;
        addr[cell(x, y, 2)] = 255;

        r = 125;
        y = r * sin(angle);
        x = r * cos(angle);
        addr[cell(x, y, 0)] = 255;
        addr[cell(x, y, 1)] = 255;
        addr[cell(x, y, 2)] = 255;

        r = 150;
        y = r * sin(angle);
        x = r * cos(angle);
        addr[cell(x, y, 0)] = 0;
        addr[cell(x, y, 1)] = 255;
        addr[cell(x, y, 2)] = 255;

        r = 175;
        y = r * sin(angle);
        x = r * cos(angle);
        addr[cell(x, y, 0)] = 255;
        addr[cell(x, y, 1)] = 255;
        addr[cell(x, y, 2)] = 255;

        r = 200;
        y = r * sin(angle);
        x = r * cos(angle);
        addr[cell(x, y, 0)] = 0;
        addr[cell(x, y, 1)] = 255;
        addr[cell(x, y, 2)] = 255;

        r = 225;
        y = r * sin(angle);
        x = r * cos(angle);
        addr[cell(x, y, 0)] = 255;
        addr[cell(x, y, 1)] = 255;
        addr[cell(x, y, 2)] = 255;

        r = 250;
        y = r * sin(angle);
        x = r * cos(angle);
        addr[cell(x, y, 0)] = 0;
        addr[cell(x, y, 1)] = 255;
        addr[cell(x, y, 2)] = 255;

        r = 275;
        y = r * sin(angle);
        x = r * cos(angle);
        addr[cell(x, y, 0)] = 0;   //b
        addr[cell(x, y, 1)] = 255; //g
        addr[cell(x, y, 2)] = 0;   //r
    }
    //---------------------------------------------------------------------

    //-------------画目标物----------------------------------------

    for (d = 0; d < 2; d++)

        for (angle = 0; angle < 6.283; angle = angle + 0.05)
        {
            r = get_r(robot_temp[1].count) + d;
            y = r * sin(angle) + robot_temp[1].rad * sin((float)(robot_temp[1].angle / 572.95));
            x = r * cos(angle) + robot_temp[1].rad * cos((float)(robot_temp[1].angle / 572.95));
            addr[cell(x, y, 0)] = d * 100;
            addr[cell(x, y, 1)] = 0;
            addr[cell(x, y, 2)] = 255;

            //--------------------------------------
            r = get_r(robot_temp[2].count) + d;
            y = r * sin(angle) + robot_temp[2].rad * sin((float)(robot_temp[2].angle / 572.95));
            x = r * cos(angle) + robot_temp[2].rad * cos((float)(robot_temp[2].angle / 572.95));
            addr[cell(x, y, 0)] = d * 100;
            addr[cell(x, y, 1)] = 0;
            addr[cell(x, y, 2)] = 255;

            //--------------------------------------
            //当实验室搭载了更多机器人的时候，可以开启下列代码
            /*             r = get_r(robot_temp[3].count) + d;
            y = r * sin(angle) + robot_temp[3].rad * sin((float)(robot_temp[3].angle / 572.95));
            x = r * cos(angle) + robot_temp[3].rad * cos((float)(robot_temp[3].angle / 572.95));
            addr[cell(x, y, 0)] = d * 100;
            addr[cell(x, y, 1)] = 0;
            addr[cell(x, y, 2)] = 255;

            //--------------------------------------

            r = get_r(robot_temp[4].count) + d;
            y = r * sin(angle) + robot_temp[4].rad * sin((float)(robot_temp[4].angle / 572.95));
            x = r * cos(angle) + robot_temp[4].rad * cos((float)(robot_temp[4].angle / 572.95));
            addr[cell(x, y, 0)] = d * 100;
            addr[cell(x, y, 1)] = 0;
            addr[cell(x, y, 2)] = 255;

            //--------------------------------------

            r = get_r(robot_temp[5].count) + d;
            y = r * sin(angle) + robot_temp[5].rad * sin((float)(robot_temp[5].angle / 572.95));
            x = r * cos(angle) + robot_temp[5].rad * cos((float)(robot_temp[5].angle / 572.95));
            addr[cell(x, y, 0)] = d * 100;
            addr[cell(x, y, 1)] = 0;
            addr[cell(x, y, 2)] = 255;

            r = get_r(robot_temp[6].count) + d;
            y = r * sin(angle) + robot_temp[6].rad * sin((float)(robot_temp[6].angle / 572.95));
            x = r * cos(angle) + robot_temp[6].rad * cos((float)(robot_temp[6].angle / 572.95));
            addr[cell(x, y, 0)] = d * 100;
            addr[cell(x, y, 1)] = 0;
            addr[cell(x, y, 2)] = 255; */
        }

    //-----------------模拟雷达扫描画面---------------------------------------
    /*
	static float scan=0;

	for(angle=0;angle<=6.283+0.005;angle=angle+0.005)
	{
		for(r=0;r<275;r=r+1)
		{
			y=(r*sin(angle));
			x=(r*cos(angle));
			//----------雷达扫描线-------------
			if(abs(scan*100-angle*100)<3)
			{
				addr[cell(x,y,0)]=0;
				addr[cell(x,y,1)]=0;
				addr[cell(x,y,2)]=255;
			}
		}
	}	 
	scan=scan+0.4;

	if(scan>=6.28)
		scan=0;

*/
    //-------------------------------------------------------------------------
}

//=========================================================================
int rgb(unsigned char *addr, int y, int x)
{
    //================================================================
    if (DAY == 1)
    {
        //----------------------------------------------------------------------------
        if (addr[cell(x, y, 2)] > addr[cell(x, y, 1)] + 40 && addr[cell(x, y, 2)] > addr[cell(x, y, 0)] + 35) //red
        {
            return 1;
        }
        //----------------------------------------------------------------------------
        if (addr[cell(x, y, 1)] > addr[cell(x, y, 2)] + 30 && addr[cell(x, y, 1)] > addr[cell(x, y, 0)] + 30) //green
        {
            return 2;
        }
        //----------------------------------------------------------------------------
        if (addr[cell(x, y, 0)] >= addr[cell(x, y, 2)] + 40 && addr[cell(x, y, 1)] > addr[cell(x, y, 2)] + 30) //blue
        {
            return 3;
        }
        //----------------------------------------------------------------------------
        if (addr[cell(x, y, 1)] >= addr[cell(x, y, 0)] + 20 && addr[cell(x, y, 2)] > addr[cell(x, y, 0)] + 20 && addr[cell(x, y, 1)] > 60 && addr[cell(x, y, 2)] > 60 && addr[cell(x, y, 0)] < 190) //yellow
        {
            return 4;
        }

        //----------------------------------------------------------------------------
        if (addr[cell(x, y, 0)] < 50 && addr[cell(x, y, 1)] < 50 && addr[cell(x, y, 2)] < 60) //black
        {
            return 5;
        }
    }
    //=======================================================================
    if (DAY == 0)
    {
        //----------------------------------------------------------------------------
        if (addr[cell(x, y, 2)] > addr[cell(x, y, 1)] + 40 &&
            addr[cell(x, y, 2)] > addr[cell(x, y, 0)] + 35) //red
        {
            return 1;
        }
        //----------------------------------------------------------------------------
        if (addr[cell(x, y, 1)] > addr[cell(x, y, 2)] + 30 &&
            addr[cell(x, y, 1)] > addr[cell(x, y, 0)] + 30) //green
        {
            return 2;
        }
        //----------------------------------------------------------------------------
        if (addr[cell(x, y, 0)] >= addr[cell(x, y, 2)] + 40 &&
            addr[cell(x, y, 1)] > addr[cell(x, y, 2)] + 30) //blue
        {
            return 3;
        }
        //----------------------------------------------------------------------------
        if (addr[cell(x, y, 1)] >= addr[cell(x, y, 0)] + 30 && addr[cell(x, y, 2)] > addr[cell(x, y, 0)] + 40 && addr[cell(x, y, 1)] > 65 && addr[cell(x, y, 2)] > 75 && addr[cell(x, y, 0)] < 70) //yellow
        {
            return 4;
        }

        //----------------------------------------------------------------------------
        if (addr[cell(x, y, 0)] < 35 && addr[cell(x, y, 1)] < 40 && addr[cell(x, y, 2)] < 50) //black
        {
            return 5;
        }
    }

    //=======================================================================
    if (DAY == 2)
    {
        //----------------------------------------------------------------------------
        if (addr[cell(x, y, 2)] > addr[cell(x, y, 1)] + 40 &&
            addr[cell(x, y, 2)] > addr[cell(x, y, 0)] + 35) //red
        {
            return 1;
        }
        //----------------------------------------------------------------------------
        if (addr[cell(x, y, 1)] > addr[cell(x, y, 2)] + 30 &&
            addr[cell(x, y, 1)] > addr[cell(x, y, 0)] + 30) //green
        {
            return 2;
        }
        //----------------------------------------------------------------------------
        if (addr[cell(x, y, 0)] >= addr[cell(x, y, 2)] + 40 &&
            addr[cell(x, y, 1)] > addr[cell(x, y, 2)] + 30) //blue
        {
            return 3;
        }
        //----------------------------------------------------------------------------

        if (addr[cell(x, y, 1)] >= addr[cell(x, y, 0)] + 15 &&
            addr[cell(x, y, 2)] > addr[cell(x, y, 0)] + 20 && addr[cell(x, y, 2)] >= 110) //yellow
        {
            return 4;
        }

        //----------------------------------------------------------------------------
        if (addr[cell(x, y, 0)] < 90 && addr[cell(x, y, 1)] < 90 && addr[cell(x, y, 2)] < 120) //black
        {
            return 5;
        }
    }

    //=================================================================
    return 0;

} //========================================================================

void find_dir(unsigned char *addr)
{
    int y = 0;
    int x = 0;
    int i = 0;
    float angle = 0;
    int r = 25;
    int cir_line[70];
    int dot_count = 0;
    int dot_sum = 0;

    int then_count = 0;
    int then_sum = 0;
    for (angle = 0.7; angle <= 2.44; angle = angle + 0.05)
    {

        y = (r * sin(angle));
        x = (r * cos(angle));
        // addr[cell(x,y,0)]=0;
        addr[cell(x, y, 1)] = 0;
        addr[cell(x, y, 2)] = 0;
        // printf(" %d",addr[cell(x,y,0)]);
        dot_count++;
        cir_line[dot_count] = addr[cell(x, y, 0)];
        dot_sum += cir_line[dot_count];
    }
    printf("\n");
    //---------------------------------------------------

    cir_line[0] = dot_count;
    dot_sum = dot_sum / cir_line[0];
    printf("all is %d avg is %d \n", cir_line[0], dot_sum);
    for (i = 1; i < dot_count; i++)
    {
        if ((dot_sum - cir_line[i]) > 10)
        {
            printf(" %d   %d\n", cir_line[i], i);

            then_count++;
            then_sum += i;
        }
    }
    if (then_count >= 1)
        then_sum = then_sum / then_count;

    printf("tis then %d avg is %d \n", then_count, then_sum);
}

void scan(unsigned char *addr)
{
    int i = 0;
    //-----------------for循环体变量----------------
    int y = 0;
    int x = 0;

    float angle = 0;
    int r = 40;
    //-----------------------------------------------------
    int y_j = 0, b_j1 = 0, b_j2 = 0;
    int y_r = 0, b_r1 = 0, b_r2 = 0;
    //-------------------------------------------
    int rad_array[1000] = {0};
    int pixel_flag[1000] = {0};
    float angle_array[1000] = {0};
    int pixel_j = 0;
    //-----------------------------------------------------------------------------
    int r_value = 0;

    //----------------------------------------------
    unsigned char tmp1 = 0, tmp2 = 0, tmp3 = 0;
    int ii = 0;
    int jj = 0;

    for (ii = -299; ii <= 299; ii++)
    {
        for (jj = -399; jj <= -1; jj++)
        {
            //互换 addr[4 * jj - 3200 * ii + 961600] 和 addr[-4 * jj - 3200 * ii + 961600] 的值
            tmp1 = addr[4 * jj - 3200 * ii + 961600];
            addr[4 * jj - 3200 * ii + 961600] = addr[-4 * jj - 3200 * ii + 961600];
            addr[-4 * jj - 3200 * ii + 961600] = tmp1;

            //互换 addr[4 * jj - 3200 * ii + 961601] 和 addr[-4 * jj - 3200 * ii + 961601] 的值
            tmp2 = addr[4 * jj - 3200 * ii + 961601];
            addr[4 * jj - 3200 * ii + 961601] = addr[-4 * jj - 3200 * ii + 961601];
            addr[-4 * jj - 3200 * ii + 961601] = tmp2;

            //互换 addr[4 * jj - 3200 * ii + 961602] 和 addr[-4 * jj - 3200 * ii + 961602] 的值
            tmp3 = addr[4 * jj - 3200 * ii + 961602];
            addr[4 * jj - 3200 * ii + 961602] = addr[-4 * jj - 3200 * ii + 961602];
            addr[-4 * jj - 3200 * ii + 961602] = tmp3;
        }
    }
    //---------------------------------------------

    for (angle = 0; angle <= 6.283 + 0.005; angle = angle + 0.005)
    {

        for (r = 40; r < 275; r = r + 1)
        {
            y = (r * sin(angle));
            x = (r * cos(angle));
            //-------------------------------------------------
            y_j = 0;
            b_j1 = 0;
            b_j2 = 0;
            y_r = 0;
            b_r1 = 0;
            b_r2 = 0;

            //-----------------------------------------------------------------

            if (4 == rgb(addr, (r * sin(angle)), (r * cos(angle)))) //rgb函数返回值是4，表示黄色
            {
                if (r >= 50)
                {
                    r_value = 30;
                }
                if (r >= 100)
                {
                    r_value = 25;
                }
                if (r >= 150)
                {
                    r_value = 20;
                }

                if (r >= 230)
                {
                    r_value = 10;
                }

                for (i = 1; i < r_value; i = i + 1)
                {
                    if (((4 == rgb(addr, (r + i) * sin(angle), (r + i) * cos(angle))) || (4 == rgb(addr, (r + i + 1) * sin(angle), (r + i + 1) * cos(angle)))) && ((4 == rgb(addr, (r - i) * sin(angle), (r - i) * cos(angle))) || (4 == rgb(addr, (r - i - 1) * sin(angle), (r - i - 1) * cos(angle)))))
                    {
                        y_j++;
                        y_r = i;
                    }
                    if (((5 == rgb(addr, (r + i) * sin(angle), (r + i) * cos(angle))) || (5 == rgb(addr, (r + i + 1) * sin(angle), (r + i + 1) * cos(angle)))) && ((5 == rgb(addr, (r - i) * sin(angle), (r - i) * cos(angle))) || (5 == rgb(addr, (r - i - 1) * sin(angle), (r - i - 1) * cos(angle)))))
                    {
                        b_j1++;
                        b_r1 = i;
                        break;
                    }
                }

                if (y_j >= 0 && b_r1 > y_r && b_r1 - y_r <= 2)
                {

                    rad_array[pixel_j] = r;
                    pixel_flag[pixel_j] = 1;
                    angle_array[pixel_j] = angle;

                    show_x[pixel_j] = x;
                    show_y[pixel_j] = y;

                    pixel_j++;
                }
            }

            //--------------------------------------------
        }

        //----------------------------------------------------
    }

    //-----------------------------------------------------------------------------

    robot_temp[0].count = pixel_j; //存储
    //---------------------------------------------------------------------
    int start_j = 0;
    int j = 0;
    int pixel_rad[10] = {0};     // 用于存储各个组别的像素点
    float pixel_angle[10] = {0}; // 用于存储各个组别的像素点

    float pixel_angle_zero[10] = {0}; // 用于存储零点附近的点
    int pixel_rad_zero[10] = {0};
    int pixel_array_zeroj[10] = {0};

    int pixel_array_j[10] = {0}; // 用于存储各个组别的像素点个数

    int Dif_rad = 0;
    int Dif_angle = 0;

    float angle_old = 0;
    float angle_zero = 0;
    int z = 0;

    if (pixel_j >= 10)
    {
        for (j = 0; j < 10; j++)
        {
            //-------------------------------------------------------------------------------
            for (i = 0; i < pixel_j; i++) // 找出还没有归类的像素点
            {
                if (pixel_flag[i] == 1)
                {
                    start_j = i;
                    //pixel_flag[i]=0;
                    angle_old = angle_array[i];
                    angle_zero = angle_array[i];
                    break;
                }
            }

            //------------------------------不同的半径取不同的限制值---------------------------------------------
            if (rad_array[start_j] < 100)
            {
                Dif_rad = 25;
                Dif_angle = 18;
            }
            if (rad_array[start_j] >= 100 && rad_array[start_j] < 125)
            {
                Dif_rad = 20;
                Dif_angle = 17;
            }
            if (rad_array[start_j] >= 125 && rad_array[start_j] < 150)
            {
                Dif_rad = 18;
                Dif_angle = 16;
            }
            if (rad_array[start_j] >= 150 && rad_array[start_j] < 175)
            {
                Dif_rad = 12;
                Dif_angle = 15;
            }
            if (rad_array[start_j] >= 175 && rad_array[start_j] < 200)
            {
                Dif_rad = 11;
                Dif_angle = 14;
            }
            if (rad_array[start_j] >= 200 && rad_array[start_j] < 225)
            {
                Dif_rad = 10;
                Dif_angle = 9;
            }
            if (rad_array[start_j] >= 225 && rad_array[start_j] < 250)
            {
                Dif_rad = 6;
                Dif_angle = 4;
            }
            if (rad_array[start_j] >= 250 && rad_array[start_j] < 275)
            {
                Dif_rad = 5;
                Dif_angle = 3;
            }

            //----------------------------------------------------------------------------

            //----------------------------------------------------------------------------
            for (i = 0; i < pixel_j; i++) // 进行每个周期的遍历
            {
                //-------------------------处理零点特殊点------------------------------------
                if (angle_zero * 100 <= 2)
                {
                    angle_zero = 6.283;
                    for (z = pixel_j - 1; z > 0; z--)
                    {
                        if (pixel_flag[z] == 1 && abs(rad_array[start_j] - rad_array[z]) < Dif_rad)

                        {
                            if (abs(angle_zero * 100 - angle_array[z] * 100) < Dif_angle)
                            {
                                pixel_rad_zero[j] += rad_array[z];
                                pixel_angle_zero[j] += angle_array[z];
                                pixel_array_zeroj[j]++;
                                pixel_flag[z] = 0;
                                angle_zero = angle_array[z];
                            }
                            else
                            {
                                break;
                            }
                        }
                    }
                }
                //-------------------------------------------------------------

                if (pixel_flag[i] == 1 && abs(rad_array[start_j] - rad_array[i]) < Dif_rad) //
                {

                    if (abs(angle_old * 100 - angle_array[i] * 100) < Dif_angle) //10=0.01 还有在临界点上还需要考虑
                    {
                        pixel_rad[j] += rad_array[i];
                        pixel_angle[j] += angle_array[i];
                        pixel_array_j[j]++;
                        pixel_flag[i] = 0;
                        angle_old = angle_array[i];
                    }
                    else
                    {
                        break;
                    }
                }
            }

            //--------------------------------------------------------------------------------
            if (pixel_array_j[j] + pixel_array_zeroj[j] >= 10)
            {
                pixel_rad[j] = pixel_rad[j] / pixel_array_j[j];
                pixel_angle[j] = pixel_angle[j] / pixel_array_j[j];
            }

            else
            {
                pixel_rad[j] = 0;
                pixel_angle[j] = 0;
                pixel_array_j[j] = 0;
            }
        }
    }

    //-----------------------第二层辨别-------------------------------------------
    //int lim_rad=0;
    //int lim_angle=0;
    int lim_count = 0;
    int pixel_rad_tmp[10] = {0};     // 用于存储各个组别的像素点
    float pixel_angle_tmp[10] = {0}; // 用于存储各个组别的像素点
    int pixel_array_j_tmp[10] = {0};
    int tmp_count = 0;

    for (j = 0; j < 10; j++)
    {
        lim_count = 10;

        if (pixel_rad[j] >= 40 && pixel_rad[j] < 100 /*&&(pixel_angle[j]*57.295>10&&pixel_angle[j]*57.295<350)*/)
        {
            lim_count = 40;
        } //50

        if (pixel_rad[j] >= 100 && pixel_rad[j] < 125 /*&&(pixel_angle[j]*57.295>9&&pixel_angle[j]*57.295<351)*/)
        {
            lim_count = 35;
        } //40

        if (pixel_rad[j] >= 125 && pixel_rad[j] < 150 /*&&(pixel_angle[j]*57.295>8&&pixel_angle[j]*57.295<352)*/)
        {
            lim_count = 30;
        } //35

        if (pixel_rad[j] >= 150 && pixel_rad[j] < 175 /*&&(pixel_angle[j]*57.295>7&&pixel_angle[j]*57.295<353)*/)
        {
            lim_count = 25;
        } //30

        if (pixel_rad[j] >= 175 && pixel_rad[j] < 200 /*&&(pixel_angle[j]*57.295>6&&pixel_angle[j]*57.295<354)*/)
        {
            lim_count = 20;
        } //25

        if (pixel_rad[j] >= 200 && pixel_rad[j] < 225 /*&&(pixel_angle[j]*57.295>5&&pixel_angle[j]*57.295<355)*/)
        {
            lim_count = 6;
        } //20

        if (pixel_rad[j] >= 225 && pixel_rad[j] < 250 /*&&(pixel_angle[j]*57.295>4&&pixel_angle[j]*57.295<356)*/)
        {
            lim_count = 5;
        } //15

        if (pixel_rad[j] >= 250 && pixel_rad[j] < 275 /*&&(pixel_angle[j]*57.295>3&&pixel_angle[j]*57.295<357)*/)
        {
            lim_count = 4;
        } //10

        if ((pixel_array_j[j] + pixel_array_zeroj[j]) > lim_count)
        {
            pixel_rad_tmp[tmp_count] = pixel_rad[j];
            pixel_angle_tmp[tmp_count] = pixel_angle[j];
            pixel_array_j_tmp[tmp_count++] = pixel_array_j[j];
        }
    }
    if (tmp_count >= 1)
    {
        robot_temp[0].number = tmp_count;
    }
    else
    {
        robot_temp[0].number = 0;
    }

    float angle_tmp = 0;
    for (i = 0; i < 9; i++)
    {
        if (pixel_array_zeroj[i] == 0)
        {
            robot_temp[i + 1].rad = pixel_rad_tmp[i];
            robot_temp[i + 1].angle = pixel_angle_tmp[i] * 572.95;
            robot_temp[i + 1].count = pixel_array_j_tmp[i];
        }
        else
        {
            angle_tmp = (pixel_angle_tmp[i] * 572.95 * pixel_array_j_tmp[i] + pixel_angle_zero[i] * 572.95 - 3600 * pixel_array_zeroj[i]) / (pixel_array_zeroj[i] + pixel_array_j_tmp[i]);

            robot_temp[i + 1].rad = (pixel_rad_tmp[i] * pixel_array_j_tmp[i] + pixel_rad_zero[i]) / (pixel_array_zeroj[i] + pixel_array_j_tmp[i]);
            robot_temp[i + 1].angle = angle_tmp >= 0 ? angle_tmp : 3600 + angle_tmp;

            // (pixel_angle_tmp[i]*572.95+(pixel_angle_zero[i]/pixel_array_zeroj[i])*572.95)>3600 ?
            //((pixel_angle_tmp[i]*572.95+(pixel_angle_zero[i]/pixel_array_zeroj[i])*572.95)-3600)/2 :
            //((pixel_angle_tmp[i]*572.95+(pixel_angle_zero[i]/pixel_array_zeroj[i])*572.95)+3600)/2;

            robot_temp[i + 1].count = pixel_array_j_tmp[i] + pixel_array_zeroj[i];
        }
    }

    //------------------------处理特殊情况------------------------------------------

    //-------------------------------------------------------------------------
}
//==========================---link---=====================
PNODE addBack(PNODE phead, int data, char *ip, int fd) //尾部插入
{
    node *pnew = malloc(sizeof(struct info)); //分配内存
    pnew->data = data;
    pnew->connected_flag = 1;
    pnew->iSocketClient = fd;
    pnew->start_connect_flag = 1;

    memcpy(pnew->send_ip, ip, 14);
    printf("add link is %s\n", pnew->send_ip);

    if (phead == NULL)
    {
        phead = pnew;
        phead->pNext = pnew;
    }
    else
    {
        PNODE p = phead;
        while (p->pNext != phead) //循环到尾部 尾插法
        {
            p = p->pNext;
        }
        p->pNext = pnew;
        pnew->pNext = phead; //头尾相连
    }

    return phead;
}

PNODE addBack_rec(PNODE phead, int data, char *ip) //尾部插入
{
    node *pnew = malloc(sizeof(struct info)); //分配内存
    pnew->data = data;
    pnew->connected_flag = 0;
    pnew->iSocketClient = 111;
    pnew->start_connect_flag = 0;

    memcpy(pnew->send_ip, ip, 14);
    printf("add link is %s\n", pnew->send_ip);

    if (phead == NULL)
    {
        phead = pnew;
        phead->pNext = pnew;
    }
    else
    {
        PNODE p = phead;
        while (p->pNext != phead) //循环到尾部
        {
            p = p->pNext;
        }
        p->pNext = pnew;
        pnew->pNext = phead; //头尾相连
    }
    //showAll(phead);
    return phead;
}

//-----------------链表冒泡排序---------------
PNODE sortLink(PNODE phead)
{
    int i = 0, j = 0, num = 0, temp_data = 0;
    ;
    PNODE p_tmp;
    p_tmp = phead;
    char ip_tmp[14];

    int connected_flag_tmp = 0;
    int iSocketClient_tmp = 0;
    int start_connect_flag_tmp = 0;
    num = getNum(phead);
    for (j = 0; j < num - 1; j++)
    {
        p_tmp = phead;
        for (i = 0; i < num - 1 - j; i++)
        {
            if (p_tmp->data > p_tmp->pNext->data)
            {
                //--------------------------------------------------------------------
                temp_data = p_tmp->pNext->data;
                memcpy(ip_tmp, p_tmp->pNext->send_ip, 14);
                connected_flag_tmp = p_tmp->pNext->connected_flag;
                iSocketClient_tmp = p_tmp->pNext->iSocketClient;
                start_connect_flag_tmp = p_tmp->pNext->start_connect_flag;
                //--------------------------------------------------------------------
                p_tmp->pNext->data = p_tmp->data;
                memcpy(p_tmp->pNext->send_ip, p_tmp->send_ip, 14);
                p_tmp->pNext->connected_flag = p_tmp->connected_flag;
                p_tmp->pNext->iSocketClient = p_tmp->iSocketClient;
                p_tmp->pNext->start_connect_flag = p_tmp->start_connect_flag;
                //---------------------------------------------------------------------------
                p_tmp->data = temp_data;
                memcpy(p_tmp->send_ip, ip_tmp, 14);
                p_tmp->connected_flag = connected_flag_tmp;
                p_tmp->iSocketClient = iSocketClient_tmp;
                p_tmp->start_connect_flag = start_connect_flag_tmp;
                //------------------------------------------------------------------
            }
            p_tmp = p_tmp->pNext;
        }
    }

    return phead;
}

//------------------------------------------------------
PNODE addFront(PNODE phead, int data) //头部插入
{
    node *pnew = malloc(sizeof(struct info)); //分配内存
    pnew->data = data;

    if (phead == NULL)
    {
        phead = pnew;
        phead->pNext = pnew;
    }
    else
    {
        PNODE p = phead;
        while (p->pNext != phead) //循环到尾部
        {
            p = p->pNext;
        }
        p->pNext = pnew;
        pnew->pNext = phead; //头尾相连
        phead = pnew;
    }

    return phead;
}
void showAll(PNODE phead) //显显示全部
{
    if (phead == NULL)
    {
        printf("链表为空！\n");
        return;
    }
    else if (phead->pNext == phead)
    {
        printf("%d %d %d %d %s %p %p\n", phead->data, phead->start_connect_flag,
               phead->connected_flag, phead->iSocketClient,
               phead->send_ip, phead, phead->pNext); //只有一个节点
    }
    else
    {
        PNODE p = phead;
        while (p->pNext != phead)
        {
            printf("%d %d %d %d %s %p %p\n", p->data, p->start_connect_flag,
                   p->connected_flag, p->iSocketClient, p->send_ip, p, p->pNext);
            p = p->pNext;
        }
        printf("%d %d %d %d %s %p %p\n", p->data, p->start_connect_flag,
               p->connected_flag, p->iSocketClient, p->send_ip, p, p->pNext); //最后一个节点。
    }
}

PNODE findFirst(PNODE phead, int data) //检索数据
{
    if (phead == NULL)
    {
        return NULL;
    }
    else if (phead->pNext == phead) //如果头节点是要查询的数据
    {
        return phead;
    }
    else
    {
        PNODE p = phead;
        while (p->pNext != phead)
        {
            if (p->data == data)
            {
                return p; //如果找到返回。
            }
            else
            {
                p = p->pNext; //找不到继续前进
            }
        }
        if (p->data == data)
            return p;
        return NULL;
    }
}

PNODE findIp(PNODE phead, char *ip)
{
    //memcpy(pnew->send_ip, ip, 14);
    //if(strcmp(local_ip,all_ip[0])==0)

    if (phead == NULL) //无节点
    {
        return NULL;
    }
    else if (phead->pNext == phead) //只有一个节点（如果头节点是要查询的数据）
    {
        //return phead;
        if (strcmp(phead->send_ip, ip) == 0)
        {
            return phead; //如果找到返回。
        }
        else
        {
            return NULL;
        }
    }
    else //两个及以上节点
    {
        PNODE p = phead;
        while (p->pNext != phead)
        {
            //if (p->data == data)
            if (strcmp(p->send_ip, ip) == 0)

            {
                return p; //如果找到返回。
            }
            else
            {
                p = p->pNext; //找不到继续前进
            }
        }
        //if (p->data == data)
        if (strcmp(p->send_ip, ip) == 0)
            return p;
        return NULL;
    }
}

PNODE deleteFirst(PNODE phead, int data) //删除数据
{
    //先判断要删除的数据是否存在。
    PNODE p = findFirst(phead, data);
    if (p == NULL)
    {
        printf("没有检索到数据！\n");
        return phead;
    }
    //删除需要使用双指针
    PNODE p1, p2;
    p1 = phead;
    p2 = NULL;
    //上面的判断要使用，否则最后一个不好判断
    while (p1->pNext != phead)
    {
        if (p1->data == data)
        {
            break;
        }
        else
        {
            p2 = p1;
            p1 = p1->pNext; //循环下个节点
        }
    }

    if (p1 != phead)
    {
        p2->pNext = p1->pNext;
        free(p1);
        p1 = NULL;
    }
    else
    {
        node *p = phead;
        while (p->pNext != phead)
        {
            p = p->pNext;
        }

        phead = phead->pNext; //改变头节点
        free(p1);             //释放p1
        p->pNext = phead;
    }

    //
    //  if (p1->pNext == phead)
    //  {
    //      if (p1->data == data)
    //      {
    //          printf("删除的数据是:%d %p\n", p1->data, p1);
    //          free(p1);
    //          phead = NULL;
    //          return phead;
    //      }
    //  }
    //  else
    //  {
    //      while (p1->pNext != phead)
    //          {
    //              if (p1->data == data)
    //              {
    //                  break;//找到后跳出循环
    //              }
    //              else
    //              {
    //                  p2 = p1;
    //                  p1 = p1->pNext;
    //              }
    //          }
    //      /* 这儿最后一个数据没有访问到，所以先要用查询，查询一遍 */
    //  }
    //  if (p1 == phead)    //如果是第一个
    //  {
    //      PNODE p = phead;
    //      while (p->pNext != phead)
    //      {
    //          p = p->pNext;    //循环到尾部
    //      }
    //      phead = phead->pNext;
    //      p->pNext = phead;
    //      printf("删除的数据是:%d %p\n", p1->data, p1);
    //      free(p1);
    //      p1 = NULL;
    //  }
    //  else
    //  {
    //      p2->pNext = p1->pNext;
    //      printf("删除的数据是:%d %p\n", p1->data, p1);
    //      free(p1);
    //  }
    return phead;
}

PNODE deleteNode(PNODE phead, int data, PNODE *ptemp)
{
    //先判断要删除的数据是否存在。
    PNODE p = findFirst(phead, data);
    if (p == NULL)
    {
        printf("没有检索到数据！\n");
        return phead;
    }
    //删除需要使用双指针
    PNODE p1, p2;
    p1 = phead;
    p2 = NULL;
    //上面的判断要使用，否则最后一个不好判断
    while (p1->pNext != phead)
    {
        if (p1->data == data)
        {
            break;
        }
        else
        {
            p2 = p1;
            p1 = p1->pNext; //循环下个节点
        }
    }

    if (p1 != phead)
    {
        p2->pNext = p1->pNext;

        *ptemp = p1->pNext; //指向下个节点

        free(p1);
        p1 = NULL;
    }
    else
    {
        node *p = phead;
        while (p->pNext != phead)
        {
            p = p->pNext;
        }

        phead = phead->pNext; //改变头节点

        *ptemp = p1->pNext; //指向下个节点

        free(p1); //释放p1
        p1 = NULL;
        p->pNext = phead;
    }
    return phead;
}

PNODE insertNode(PNODE phead, int finddata, int data) //插入数据
{
    PNODE pnew = malloc(sizeof(node));
    pnew->data = data;

    PNODE p = findFirst(phead, finddata);
    if (p == NULL)
    {
        printf("没有找到要插入数据的标记！\n");
    }
    else
    {
        PNODE p1, p2;
        p1 = phead;
        p2 = NULL;
        while (p1->pNext != phead)
        {
            if (p1->data == finddata)
            {
                break;
            }
            else
            {
                p2 = p1;
                p1 = p1->pNext;
            }
        }
        if (p1 == phead)
        {

            PNODE p = phead;
            while (p->pNext != phead)
            {
                p = p->pNext;
            }
            //插在前面
            p->pNext = pnew;
            pnew->pNext = phead;
            phead = pnew;

            //插在后面
            /*  pnew->pNext = phead->pNext; 
            phead->pNext = pnew;*/
        }
        else
        {
            //插在前面
            p2->pNext = pnew;
            pnew->pNext = p1;
            //插在后面
            /*pnew->pNext = p1->pNext; 
            p1->pNext = pnew;*/
        }
    }

    return phead;
}

//---------------------------------------------------------
int getNum(PNODE phead) //返回链表的个数
{
    if (phead == NULL)
    {
        return 0;
    }
    else if (phead->pNext == phead)
    {
        return 1;
    }
    else
    {
        int sum = 0;
        PNODE p = phead;
        while (p->pNext != phead)
        {
            sum++;
            p = p->pNext;
        }
        sum++;
        return sum;
    }
}

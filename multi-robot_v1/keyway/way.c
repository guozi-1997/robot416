
#include <way.h>

#define min(a, b) ((a > b) ? (b) : (a))
#define max(a, b) ((a > b) ? (a) : (b))

#define branch_num 3     //鎼滅储鏍戝垎鏀暟
#define Layer 4          //闈欐€佹悳绱㈡爲灞傛暟涓哄畾鍊�
#define ObsDeltaAng 150  //闅滅鐗╀笌鎼滅储璺緞瑙掑害宸槇鍊�
#define ExpandDis 200    //鑶ㄨ儉璺濈mm
#define Step 40          //姝ラ暱,鍗曚綅pix
#define StepDistance 300 //姝ラ暱锛屽崟浣峬m
#define Display_Flag 1   //0鏄剧ず鎵€鏈夋悳绱㈣矾寰勶紝1鏄剧ず瑙勫垝鍚庣殑鍙璺緞
#define SafeDistance 400

int randangle[6][100][10] = {0};
int childrandcnt = 0;
int pix[10] = {0};
int stepdis[10] = {0};
int planning_cnt = 0;
RandRito = 1;
int time_srand = 0;
int release_num[10] = {0};
int Node_PerNum = 1; //鎵€鍦ㄥ眰鐨勮妭鐐规暟
int Circle_num = 0;  //浜х敓闅忔満鏁扮殑寰幆娆℃暟

int obs_search_cnt = 0;

int tmpmindistance = 0;
int tmpmaxdistance = 0;

int tmpminangle_f = 0;
int tmpmaxangle_f = 0;

int tmpminang_2 = 0;
int tmpminpix_2 = 0;

int ROLLWINDOW_PIX = 0;

int Tmp_Cnt[10] = {0};
int Tmp_Cilcle_Cnt[10] = {0};

int tmp_maxrad = 0;

int tmp_maxangle = 0, tmp_minangle = 0, tmp_mindis = 0;
int tmp_minangle_dis = 0;
int tmp_maxangle_dis = 0;
int SigleObs_MaxAngle = 0;
int SigleObs_MinAngle = 0;
int SigleObs_MaxAngle_Dis = 0;
int SigleObs_MinAngle_Dis = 0;
int SigleObs_MinDis = 0;

T_OkPath OkPath;
T_Best_OkPath Best_OkPath;
T_Reach Reach_Path;
T_ReachFilter ReachFilter_Path;
T_Lidar_Cnt Lidar_Data;
ROLLWINDOW_RITO = 1;
int calibrate_cnt = 0;
int HaveNoObs = 0;

int Lidar_Ok_Flag = 0;

//one control point
Point2D PointOnBezierOneControl(Point2D *cp, float t)
{
    float ax, bx;
    float ay, by;
    float tSquared;
    Point2D result;

    ax = cp[1].x - cp[0].x;
    ax = cp[0].x + t * ax;

    bx = cp[2].x - cp[1].x;
    bx = cp[1].x + t * bx;

    ay = cp[1].y - cp[0].y;
    ay = cp[0].y + t * ay;

    by = cp[2].y - cp[1].y;
    by = cp[1].y + t * by;

    tSquared = t * t;

    result.x = ax + t * (bx - ax);
    result.y = ay + t * (by - ay);

    return result;
}

void ComputeOneBezier(Point2D *cp, int numberOfPoints, Point2D *curve)
{
    float dt;
    int i;
    dt = 1.0 / (numberOfPoints - 1);
    for (i = 0; i < numberOfPoints; i++)
        curve[i] = PointOnBezierOneControl(cp, i * dt);
}

//two control points 涓や釜鎺у埗鐐�
Point2D PointOnCubicBezier(Point2D *cp, float t)
{
    float ax, bx, cx;
    float ay, by, cy;
    float tSquared, tCubed;
    Point2D result;
    /* 鐠侊紕鐣绘径姘躲€嶅蹇曢兇閺侊拷 */
    cx = 3.0 * (cp[1].x - cp[0].x);
    bx = 3.0 * (cp[2].x - cp[1].x) - cx;
    ax = cp[3].x - cp[0].x - cx - bx;
    cy = 3.0 * (cp[1].y - cp[0].y);
    by = 3.0 * (cp[2].y - cp[1].y) - cy;
    ay = cp[3].y - cp[0].y - cy - by;
    /* 鐠侊紕鐣籺娴ｅ秶鐤嗛惃鍕仯閸婏拷 */
    tSquared = t * t;
    tCubed = tSquared * t;
    result.x = (ax * tCubed) + (bx * tSquared) + (cx * t) + cp[0].x;
    result.y = (ay * tCubed) + (by * tSquared) + (cy * t) + cp[0].y;
    return result;
}
/* ComputeBezier 娴犮儲甯堕崚鍓佸仯 cp 閹碘偓娴溠呮晸閻ㄥ嫭娲哥痪璺ㄥ仯閿涘苯锝為崗锟� Point2D 缂佹挻鐎弫鎵矋閵嗭拷 
鐠嬪啰鏁ら弬鐟扮箑妞よ鍨庨柊宥堝喕婢剁喓娈戠粚娲？娴犮儰绶垫潏鎾冲毉閿涳拷<sizeof(Point2D) numberOfPoints> */
void ComputeDoubleBezier(Point2D *cp, int numberOfPoints, Point2D *curve)
{
    float dt;
    int i;
    dt = 1.0 / (numberOfPoints - 1);
    for (i = 0; i < numberOfPoints; i++)
        curve[i] = PointOnCubicBezier(cp, i * dt);
}

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
        if (addr[cell(x, y, 1)] >= addr[cell(x, y, 0)] + 40 && addr[cell(x, y, 2)] > addr[cell(x, y, 0)] + 50 && addr[cell(x, y, 1)] > 110 && addr[cell(x, y, 2)] > 120 && addr[cell(x, y, 0)] < 110) //yellow
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
    if (DAY == 2) //鏆傚畾閲囩敤璇ユ柟寮忓垽鏂壂鎻忕殑鏄笉鏄粍鑹�
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
        if (addr[cell(x, y, 1)] >= addr[cell(x, y, 0)] + 30 &&
            addr[cell(x, y, 2)] > addr[cell(x, y, 0)] + 30 && addr[cell(x, y, 2)] >= 115) //yellow	0 1 2 鍒嗗埆bgr
        {
            return 4;
        }

        //----------------------------------------------------------------------------
        if (addr[cell(x, y, 0)] < 90 && addr[cell(x, y, 1)] < 90 && addr[cell(x, y, 2)] < 120) //black
        {
            return 5;
        }

        //---------------------娣诲姞鐨�--------------------------
        if (addr[cell(x, y, 0)] - addr[cell(x - 1, y, 0)] > 20 || addr[cell(x, y, 1)] - addr[cell(x - 1, y, 1)] > 20 || addr[cell(x, y, 2)] - addr[cell(x - 1, y, 2)] > 20)
        {
            return 6;
        }
    }

    //=================================================================
    return 0;

} //========================================================================

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

    return 0;
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
    int l = 0;
    int p = 0;
    int s = 0, q = 0;
    float theta = 0.0;
    float tmp_rollwindow_r = 0.0;

    //--------------------鐢荤洿瑙掑潗鏍囩郴-------------------------------
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

    //----------鍒ゆ柇lidar閲囬泦鍒扮殑闅滅鐗╃偣鏄惁灞炲疄(婊ゆ尝)-------
    //-------杩愯鍒板瓙鐩爣鐐硅繃绋嬩腑妫€娴嬫鍓嶆柟鏄惁鏈夐殰纰�--------

    if (AimCount > 0)
    {

        for (i = 0; i < AimCount; i++)
        {
            if (abs((3600 + 900 - CurLidarAng[i]) % 3600 - 900) < 10 && CurLidarDistance[i] < (SafeDistance - Safe_Dis_Sub) && CurLidarDistance[i] != 258)
            {
                Next_UnSafe_Flag = 1; //涓嶅畨鍏�
                printf("CurLidarAng=%d\n", (3600 + 900 - CurLidarAng[i]) % 3600);
                printf("CurLidarDistance=%d\n", CurLidarDistance[i]);
                printf("\n");
            }
        }
        //printf("Next_UnSafe_Flag=%d\n",Next_UnSafe_Flag);
        if (Lidar_Circle_Cnt < 7)
        {
            for (i = 0; i < AimCount; i++)
            {
                Lidar_Data.Lidar_Data_Dis[Lidar_Data.Lidar_Data_Cnt] = CurLidarDistance[i];
                Lidar_Data.Lidar_Data_Ang[Lidar_Data.Lidar_Data_Cnt] = CurLidarAng[i];
                Lidar_Data.Lidar_Data_Pix[Lidar_Data.Lidar_Data_Cnt] = CurLidarDistPix[i];
                Lidar_Data.Lidar_Data_Cnt++;
            }
        }

        //	printf("AimCount=%d\n",AimCount);
        //	printf("Lidar_Circle_Cnt=%d\n",Lidar_Circle_Cnt);
        if (Lidar_Circle_Cnt > 5)
        {
            Lidar_Ok_Flag = 1;
            //	printf("Lidar_Data.Lidar_Data_Cnt=%d\n",Lidar_Data.Lidar_Data_Cnt);
            //	memset(&Lidar_Data,0,sizeof(Lidar_Data));
            Lidar_Circle_Cnt = 0;
        }
    }
    if (Lidar_Ok_Flag)
    {
        for (i = 0; i < Lidar_Data.Lidar_Data_Cnt; i++)
        {
            r = Lidar_Data.Lidar_Data_Pix[i];
            if ((i > 0) && (i < Lidar_Data.Lidar_Data_Cnt - 1) && (abs(Lidar_Data.Lidar_Data_Pix[i] - Lidar_Data.Lidar_Data_Pix[i - 1]) < 2 || abs(Lidar_Data.Lidar_Data_Pix[i + 1] - Lidar_Data.Lidar_Data_Pix[i]) < 2))
            {
                if (Lidar_Data.Lidar_Data_Pix[i] > 40)
                {
                    TrueCurLidarDistPix[TrueAimCount] = Lidar_Data.Lidar_Data_Pix[i];
                    TrueCurLidarAng[TrueAimCount] = Lidar_Data.Lidar_Data_Ang[i];
                    TrueAimCount++;
                }
            }

            if ((i > 0) && (Lidar_Data.Lidar_Data_Dis[i] != 258) && (i < Lidar_Data.Lidar_Data_Cnt - 1) && (abs(Lidar_Data.Lidar_Data_Dis[i] - Lidar_Data.Lidar_Data_Dis[i - 1]) < 50 || abs(Lidar_Data.Lidar_Data_Dis[i + 1] - Lidar_Data.Lidar_Data_Dis[i]) < 50))
            {
                if (Lidar_Data.Lidar_Data_Dis[i] > 250)
                {
                    TrueCurLidarDistance[TrueAimCntDis] = Lidar_Data.Lidar_Data_Dis[i];
                    TrueCurLidarAngle[TrueAimCntDis] = Lidar_Data.Lidar_Data_Ang[i];
                    TrueAimCntDis++;
                }
            }
        }
        //	printf("TrueAimCntDis=%d,TrueAimCount=%d\n",TrueAimCntDis,TrueAimCount);
        //	printf("\n");

        //--------------------婊ゆ尝鍚庣殑lidar闅滅鐗╂樉绀�-----------------------------------

        //-----闆疯揪鍧愭爣杞崲鍒板浘鍍忓潗鏍�-------
        for (i = 0; i < TrueAimCount; i++)
        {
            TrueCurLidarAng[i] = (3600 + 900 - TrueCurLidarAng[i]) % 3600;
        }

        for (i = 0; i < TrueAimCntDis; i++)
        {
            TrueCurLidarAngle[i] = (3600 + 900 - TrueCurLidarAngle[i]) % 3600;
        }

        //--------------鍐掓场娉曟帓搴�------------------------------------------
        //----------灏嗘壂鎻忓埌鐨勯殰纰嶇墿杞寲鍒版満鍣ㄤ汉鏄剧ず鍧愭爣涓�--
        //----------鎸夌収瑙掑害閫掑鎺掑簭(鏈哄櫒浜哄ご鏈濆悜90搴�)------------
        int tmpangle = 0;
        int tmppix = 0;
        int tmpdis = 0;
        for (i = 0; i < TrueAimCount - 1; i++)
        {
            for (j = (i + 1); j < TrueAimCount; j++)
            {
                if (TrueCurLidarAng[i] >= TrueCurLidarAng[j])
                {
                    tmpangle = TrueCurLidarAng[i];
                    TrueCurLidarAng[i] = TrueCurLidarAng[j];
                    TrueCurLidarAng[j] = tmpangle;

                    tmppix = TrueCurLidarDistPix[i];
                    TrueCurLidarDistPix[i] = TrueCurLidarDistPix[j];
                    TrueCurLidarDistPix[j] = tmppix;
                }
            }
        }

        for (i = 0; i < TrueAimCntDis - 1; i++)
        {
            for (j = (i + 1); j < TrueAimCntDis; j++)
            {
                if (TrueCurLidarAngle[i] >= TrueCurLidarAngle[j])
                {
                    tmpangle = TrueCurLidarAngle[i];
                    TrueCurLidarAngle[i] = TrueCurLidarAngle[j];
                    TrueCurLidarAngle[j] = tmpangle;

                    tmpdis = TrueCurLidarDistance[i];
                    TrueCurLidarDistance[i] = TrueCurLidarDistance[j];
                    TrueCurLidarDistance[j] = tmpdis;
                }
            }
        }

        //-------灏嗛噰闆嗗埌鐨勯殰纰嶇墿鏁版嵁婊ゆ尝锛屽幓闄ら噸澶嶆暟鎹�----

        for (i = 0; i < TrueAimCount - 1; i++)
        {
            if (abs(TrueCurLidarAng[i] - TrueCurLidarAng[i + 1]) < 1)
            {
                FilterLidarAng[FilterCount] = TrueCurLidarAng[i];
                FilterLidarDistPix[FilterCount] = TrueCurLidarDistPix[i];
                FilterCount++;
            }
        }

        for (j = 0; j < TrueAimCntDis - 1; j++)
        {
            if (TrueCurLidarAngle[j] != TrueCurLidarAngle[j + 1])
            {
                FilterLidarAng2[FilterCount2] = TrueCurLidarAngle[j];
                FilterLidarDistance[FilterCount2] = TrueCurLidarDistance[j];
                FilterCount2++;
            }
        }
        //printf("TrueAimCount=%d,TrueAimCntDis=%d\n",TrueAimCount,TrueAimCntDis);
        //printf("FilterCount=%d,FilterCount2=%d\n",FilterCount,FilterCount2);
        //printf("\n");

        //----鎻愬彇璺濈鏈哄櫒浜烘渶杩戠殑闅滅鐗╄窛绂诲拰瑙掑害----------

        if (!tmpminangle_f)
        {
            tmpminangle_f = FilterLidarAng2[0];
        }
        else
        {
            tmpminangle_f = min(tmpminangle_f, FilterLidarAng2[0]);
        }

        if (!tmpmaxangle_f)
        {
            tmpmaxangle_f = FilterLidarAng2[0];
        }
        else
        {
            tmpminangle_f = max(tmpminangle_f, FilterLidarAng2[0]);
        }
        tmpmindistance = FilterLidarDistance[0];
        tmpmaxdistance = FilterLidarDistance[0];

        for (i = 0; i < FilterCount2; i++)
        {
            tmpmindistance = min(tmpmindistance, FilterLidarDistance[i]);
            tmpmaxdistance = max(tmpmaxdistance, FilterLidarDistance[i]);

            tmpminangle_f = min(tmpminangle_f, FilterLidarAng2[i]);
            tmpmaxangle_f = max(tmpmaxangle_f, FilterLidarAng2[i]);
        }

        //	printf("tmpminangle=%d\n",tmpminangle_f);
        //	printf("tmpmaxangle=%d\n",tmpmaxangle_f);

        //	printf("Lidar_Circle_Cnt=%d\n",Lidar_Circle_Cnt);
        //	printf("tmpminpix=%d\n",tmpminpix);
        //	printf("tmpmaxpix=%d\n",tmpmaxpix);
        //	printf("\n");

        //----------鏄剧ず鎸夊浘鍍忓潗鏍囨帓搴忓悗鐨勯殰纰嶇墿-----------------

        for (i = 0; i < FilterCount; i++)
        {
            r = FilterLidarDistPix[i];

            for (d = 0; d < 4; d++)
            {
                if (FilterLidarAng[i] < 900)
                {
                    y = r * sin(FilterLidarAng[i] * 3.14 / 1800) + 1 + 5 * cos(FilterLidarAng[i] * 3.14 / 1800) + d;
                    x = r * cos(FilterLidarAng[i] * 3.14 / 1800) - 5;
                }
                if (FilterLidarAng[i] > 900)
                {
                    y = r * sin(FilterLidarAng[i] * 3.14 / 1800) + 1 + d;
                    x = r * cos(FilterLidarAng[i] * 3.14 / 1800) + 5 * sin(FilterLidarAng[i] * 3.14 / 1800);
                }

                addr[cell(x, y, 0)] = 0;
                addr[cell(x, y, 1)] = 255;
                addr[cell(x, y, 2)] = 0;
            }
        }

        //------------------------------------------------------------------------

        //-------鍒ゆ柇鏈哄櫒浜哄埌鎬荤洰鏍囩殑鐩寸嚎璺緞涓婃湁鏃犻殰纰嶇墿---

        //*********************************************************************//
        //--------------鏀硅繘鐨凴RT绠楁硶瀹炵幇鍔犲叆婊氬姩绐楀彛----------//
        //*********************************************************************//

        if (!isArrivedDirect_Flag)
        {
            if (FilterCount2)
                HaveNoObs = 0;
            if (!FilterCount2)
                HaveNoObs = 1;
        }
        //printf("HaveNoObs=%d\n",HaveNoObs);

        //---鐩存帴璺戝悜鐩爣鐨勬潯浠�:
        //----1.绐楀彛鍖哄煙鍐呮棤闅滅鐗�2.绐楀彛鍐呮湁闅滅锛�
        //--------涓旈殰纰嶇墿鍒嗗竷涓嶅湪鏈哄櫒浜轰笌鐩爣杩炵嚎鐨勫畨鍏ㄩ€氶亾鍐�---
        if (HaveNoObs && (!isArrivedDirect_Flag))
        {
            SrandNum[0] = robot_temp[1].angle;
            printf("SrandNum[0]=%d\n", SrandNum[0]);
            printf("No Obs Path00000000000000000000000000000000000000000000000\n");

            double_control_points[0].x = 0;
            double_control_points[0].y = 0;

            double_control_points[1].x = 0;
            double_control_points[1].y = ROLLWINDOW_R * sin(SrandNum[0] * 3.14 / 1800) / 2;

            double_control_points[2].x = ROLLWINDOW_R * cos(SrandNum[0] * 3.14 / 1800) / 2;
            double_control_points[2].y = ROLLWINDOW_R * sin(SrandNum[0] * 3.14 / 1800) / 2;

            double_control_points[3].x = ROLLWINDOW_R * cos(SrandNum[0] * 3.14 / 1800);
            double_control_points[3].y = ROLLWINDOW_R * sin(SrandNum[0] * 3.14 / 1800);

            ComputeDoubleBezier(double_control_points, TrackNodeNum, double_control_r);
            Track_Flag = 1;
            for (i = 0; i < 4; i++)
            {
                printf("x=%d,y=%d\n", double_control_points[i].x, double_control_points[i].y);
            }
            printf("\n");
        }

        for (i = 0; i < ROLLWINDOW_PIX; i++)
        {
            y = i * sin((SrandNum[0] + get_tht) * 3.14 / 1800);
            x = i * cos((SrandNum[0] + get_tht) * 3.14 / 1800);
            addr[cell(x, y, 0)] = 0;
            addr[cell(x, y, 1)] = 0;
            addr[cell(x, y, 2)] = 255;
        }

        //--HaveNoObs=1琛ㄧず鏃犻殰纰�----

        int tmp_pow = 0;
        int tmp_dis = 0;

        if (FilterCount && FilterCount2 && (!isArrivedDirect_Flag) && !HaveNoObs)
        {
            //Reach_Path.Reach_Cnt=0;
            if (isOkPath == 10)
            {
                calibrate_cnt = 0;
                for (i = 0; i < 5; i++)
                {
                    fd_rand = open("/dev/urandom", O_RDONLY); //  /dev/urandom璁惧鍙疄鐜伴殢鏈烘暟浜х敓

                    read(fd_rand, SrandNum + i, sizeof(int));    //浜岀淮鏁扮粍a[i]琛ㄧず 绗琲琛岄鍦板潃
                    SrandNum[i] = abs(SrandNum[i]) % 1600 + 100; //10-170搴︽悳绱�

                    RandRito = RandRito * 2;
                    isOkPath = 1;
                    close(fd_rand);
                }
            }

            //--- 鐢ㄤ簬涓や晶闅滅鐗╁瓨鍦ㄦ椂閫夊彇閫氶亾-------
            for (i = 0; i < FilterCount2 - 1; i++)
            {
                tmp_pow = pow((FilterLidarDistance[i] * FilterLidarDistance[i] + FilterLidarDistance[i + 1] * FilterLidarDistance[i + 1] - 2 * FilterLidarDistance[i] * FilterLidarDistance[i + 1] * cos((FilterLidarAng2[i] - FilterLidarAng2[i + 1]) * 3.14 / 1800)), 0.5);

                //------瀛樺湪涓や晶闅滅鐗╀箣闂寸殑闂撮殧瀹硅鏈哄櫒浜洪€氳繃---------
                if ((abs(FilterLidarAng2[i] - FilterLidarAng2[i + 1]) > 200) && tmp_pow > 400)
                {
                    Reach_Path.Reach_Angle[Reach_Path.Reach_Cnt] = FilterLidarAng2[i];
                    Reach_Path.Reach_Distance[Reach_Path.Reach_Cnt] = FilterLidarDistance[i];
                    Reach_Path.Reach_Cnt++;

                    Reach_Path.Reach_Angle[Reach_Path.Reach_Cnt] = FilterLidarAng2[i + 1];
                    Reach_Path.Reach_Distance[Reach_Path.Reach_Cnt] = FilterLidarDistance[i + 1];
                    Reach_Path.Reach_Cnt++;
                    //	printf("have double OBS   reachable path========================\n");
                    //	printf("tmp_pow=%d\n",tmp_pow);
                    //	printf("FilterLidarAng2[%d]=%d,FilterLidarAng2[%d]=%d,FilterLidarDistance[%d]=%d,FilterLidarDistance[%d]=%d\n",i,FilterLidarAng2[i],i+1,FilterLidarAng2[i+1],i,FilterLidarDistance[i],i+1,FilterLidarDistance[i+1]);
                }
            }

            //--鍒ゆ柇鏈€灏忚搴﹁缃垵濮嬪€�------
            //------璁板綍瑙掑害鍙栨渶鍊兼椂鍊欏搴旂殑闅滅鐗╄窛绂�----

            if (!tmp_minangle)
            {
                tmp_minangle = FilterLidarAng2[0];
                tmp_minangle_dis = FilterLidarDistance[0];
            }
            else
            {
                tmp_minangle = min(tmp_minangle, FilterLidarAng2[0]);
                tmp_minangle_dis = min(tmp_minangle_dis, FilterLidarDistance[0]);
            }

            if (!tmp_mindis)
            {
                tmp_mindis = FilterLidarDistance[0];
            }
            else
            {
                tmp_mindis = min(tmp_mindis, FilterLidarDistance[0]);
            }

            int tmp_anglesub[100] = {0};
            int tmp_distancesub[100] = {0};
            int tmp_anglesub_cnt = 0;
            for (i = 0; i < FilterCount2 - 1; i++)
            {
                if (abs(FilterLidarAng2[i] - FilterLidarAng2[i + 1]) > 200)
                {
                    tmp_anglesub[tmp_anglesub_cnt] = FilterLidarAng2[i];
                    tmp_distancesub[tmp_anglesub_cnt] = FilterLidarDistance[i];
                    tmp_anglesub_cnt++;

                    tmp_anglesub[tmp_anglesub_cnt] = FilterLidarAng2[i + 1];
                    tmp_distancesub[tmp_anglesub_cnt] = FilterLidarDistance[i + 1];
                    tmp_anglesub_cnt++;
                }
            }

            for (i = 0; i < FilterCount2 - 1; i++)
            {
                tmp_maxrad = max(tmp_maxrad, abs(FilterLidarAng2[i] - FilterLidarAng2[i + 1]));
            }
            for (i = 0; i < FilterCount2; i++)
            {
                tmp_maxangle = max(tmp_maxangle, FilterLidarAng2[i]);
                tmp_minangle = min(tmp_minangle, FilterLidarAng2[i]);

                tmp_mindis = min(tmp_mindis, FilterLidarDistance[i]);
            }
            //----鑾峰彇鏈€鍊艰搴﹀搴旂殑璺濈-----
            for (i = 0; i < FilterCount2; i++)
            {
                if (tmp_minangle == FilterLidarAng2[i])
                {
                    tmp_minangle_dis = FilterLidarDistance[i];
                }
                if (tmp_maxangle == FilterLidarAng2[i])
                {
                    tmp_maxangle_dis = FilterLidarDistance[i];
                }
            }
            //	printf("tmpminangle=%d,tmpmaxangle=%d\n",tmp_minangle,tmp_maxangle);
            //	printf("tmp_maxrad=%d\n",tmp_maxrad);
            //	printf("tmp_mindis=%d\n",tmp_mindis);
            //	printf("tmp_minangle_dis=%d\n",tmp_minangle_dis);
            //	printf("tmp_maxangle_dis=%d\n",tmp_maxangle_dis);
            //	printf("\n");

            //-----鍗曚晶闅滅鐗╁垎涓洪殰纰嶇墿瑕嗙洊--------
            //---瑙嗚澶т簬150掳鍜屽皬浜�150掳鐨勬儏鍐�----
            if (tmp_maxrad < 200)
            {
                HaveDoubleObsPath = 2; //=2琛ㄧず鍗曚晶闅滅
                SigleObs_MaxAngle = tmp_maxangle;
                SigleObs_MinAngle = tmp_minangle;
                SigleObs_MinDis = tmp_mindis;
                SigleObs_MinAngle_Dis = tmp_minangle_dis;
                SigleObs_MaxAngle_Dis = tmp_maxangle_dis;

                if (2 == HaveDoubleObsPath)
                {
                    if (tmp_maxangle - tmp_minangle > 1500) //琛ㄧず杩炵画鍗曚晶闅滅瑙嗚瑕嗙洊杩囧ぇ
                    {
                        HaveDoubleObsPath = 3; //鐗规畩鐜
                    }
                }
            }
            else
            {
                HaveDoubleObsPath = 1; //=1琛ㄧず鏈変袱渚ч殰纰�
                SigleObs_MaxAngle = 0;
                SigleObs_MinAngle = 0;
                SigleObs_MinDis = 0;
                SigleObs_MinAngle_Dis = 0;
                SigleObs_MaxAngle_Dis = 0;
            }

            tmp_maxangle = 0;
            tmp_minangle = FilterLidarAng2[0];
            tmp_mindis = 0;
            tmp_minangle_dis = FilterLidarDistance[0];
            tmp_maxangle_dis = 0;

            //	printf("SigleObs_MaxAngle=%d\n",SigleObs_MaxAngle);
            //	printf("SigleObs_MinAngle=%d\n",SigleObs_MinAngle);
            //	printf("\n");

            //----鍒ゆ柇绛涢€夊嚭鏉ョ殑鏃犻殰纰嶅尯闂存槸鍚﹀悎鐞�--------------//
            /*	if(Reach_Path.Reach_Cnt)
		{			
			for(i=0;i<Reach_Path.Reach_Cnt-1;i=i+2)
			{
				for(j=0;j<FilterCount2;j++)
				{
					if(abs(FilterLidarAng2[j]-(Reach_Path.Reach_Angle[i]+Reach_Path.Reach_Angle[i+1])/2)<100)
					{
						Reach_Path.Reach_Angle[i]=0;
						Reach_Path.Reach_Angle[i+1]=0;					
					}
				}
			}
		}
		//----鍒ゆ柇婊′竴瀹氬湀鏁帮紝鍑忓皬璇樊-------
		if(Reach_Path.Reach_Cnt )
		{
			for(i=0;i<Reach_Path.Reach_Cnt-1;i=i+2)
			{
				if(Reach_Path.Reach_Angle[i])
				{
					ReachFilter_Path.ReachFilter_Angle[ReachFilter_Path.ReachFilter_Cnt]=Reach_Path.Reach_Angle[i];
					ReachFilter_Path.ReachFilter_Cnt++;
					ReachFilter_Path.ReachFilter_Angle[ReachFilter_Path.ReachFilter_Cnt]=Reach_Path.Reach_Angle[i+1];					
					ReachFilter_Path.ReachFilter_Cnt++;
				//	printf("Lidar_Circle_Cnt=%d\n",Lidar_Circle_Cnt);
				//	printf("Reach_Angle[%d]=%d,Reach_Angle[%d]=%d\n",i,Reach_Path.Reach_Angle[i],i+1,Reach_Path.Reach_Angle[i+1]);					
				}
			}

			//Lidar_Circle_Cnt=0;
			//memset(Tmp_Cilcle_Cnt,0,sizeof(Tmp_Cilcle_Cnt));
			Reach_Path.Reach_Cnt=0;
		}
		int tmp_filter1=0;
		int tmp_filter2=0;
		if(ReachFilter_Path.ReachFilter_Cnt)
		{
			for(i=0;i<ReachFilter_Path.ReachFilter_Cnt-1;i=i+2)
			{
				tmp_filter1+=ReachFilter_Path.ReachFilter_Angle[i];
				tmp_filter2+=ReachFilter_Path.ReachFilter_Angle[i+1];
			}
			tmp_filter1=tmp_filter1/(ReachFilter_Path.ReachFilter_Cnt/2);
			tmp_filter2=tmp_filter2/(ReachFilter_Path.ReachFilter_Cnt/2);
			printf("tmp_filter1=%d,tmp_filter2=%d\n",tmp_filter1,tmp_filter2);
			SrandNum[0]=(tmp_filter1+tmp_filter2)/2;
		
			printf("SrandNum=%d\n",SrandNum[0]);
			printf("have double OBS   reachable path========================\n");

			//-----鎺掑簭-------
			int tmp1_s=0;
			for(i=0;i<FilterCount2-1;i++)
			{
				for(j=i+1;j<FilterCount2;j++)
				{
					if(abs(FilterLidarAng2[i]-SrandNum[0])>abs(FilterLidarAng2[j]-SrandNum[0]))
					{
						tmp1_s=FilterLidarAng2[i];
						FilterLidarAng2[i]=FilterLidarAng2[j];
						FilterLidarAng2[j]=tmp1_s;
					}
				}				
			}			
			if(abs(FilterLidarAng2[0]-SrandNum[0])>200)
			{
				isOkPath=1;
				HaveDoubleObsPath=1;
			//	isArrivedDirect_Flag=1;
			//	printf("FilterLidarAng2[0]-SrandNum[0]=%d\n",abs(FilterLidarAng2[0]-SrandNum[0]));
			}
			
			ReachFilter_Path.ReachFilter_Cnt=0;
			
		}
*/
            if (1 == HaveDoubleObsPath)
            {
                int tmp_left_ang = 0;
                int tmp_right_ang = 0;
                int tmp_left_dis = 0;
                int tmp_right_dis = 0;
                printf("have double OBS   reachable path========================\n");
                printf("tmp_maxrad=%d\n", tmp_maxrad);
                //----------鎵惧嚭鏈€澶у彲琛屽叆鍙�----------
                for (i = 0; i < FilterCount2 - 1; i++)
                {
                    if (tmp_maxrad == abs(FilterLidarAng2[i + 1] - FilterLidarAng2[i]))
                    {
                        tmp_left_ang = FilterLidarAng2[i];
                        tmp_left_dis = FilterLidarDistance[i];
                        tmp_right_ang = FilterLidarAng2[i + 1];
                        tmp_right_dis = FilterLidarDistance[i + 1];
                    }
                }

                if (abs(tmp_left_ang - robot_temp[1].angle) < abs(tmp_right_ang - robot_temp[1].angle))
                {
                    SrandNum[0] = tmp_left_ang + asin(200 / (float)tmp_left_dis) * 1800 / 3.14;
                }
                else
                {
                    SrandNum[0] = tmp_right_ang - asin(200 / (float)tmp_right_dis) * 1800 / 3.14;
                }
                printf("tmp_left_ang=%d,tmp_right_ang=%d\n", tmp_left_ang, tmp_right_ang);
                printf("tmp_left_dis=%d,tmp_right_dis=%d\n", tmp_left_dis, tmp_right_dis);
                printf("SrandNum[0]=%d\n", SrandNum[0]);
            }

            //---濡傛灉鍙湁鍗曚晶闅滅鐗╀笖瑙嗛噹瑕嗙洊灏忥紝鐩爣瀵煎悜鎬ч€夊彇-------
            if (2 == HaveDoubleObsPath)
            {
                //-----淇濊瘉鐩爣鍦ㄦ満鍣ㄤ汉鍓嶆柟--鍗�0-180掳----------
                //---鍒ゆ柇瑙掑害鐨勭洰鐨勬槸涓轰簡鍒ゆ柇璺濈锛屾墍浠�---
                //-------闇€瑕佽浆鎹负璺濈----淇敼--------------------
                int a = 0, b = 0;
                if (SigleObs_MinDis > 250)
                {
                    if (abs(SigleObs_MaxAngle - robot_temp[1].angle) > abs(SigleObs_MinAngle - robot_temp[1].angle))
                    {
                        SrandNum[0] = SigleObs_MinAngle - asin(200 / (float)SigleObs_MinAngle_Dis) * 1800 / 3.14;
                        if (SrandNum[0] < 0)
                            SrandNum[0] = 0;
                        a = asin(200 / (float)SigleObs_MinAngle_Dis) * 1800 / 3.14;
                    }
                    else
                    {
                        SrandNum[0] = SigleObs_MaxAngle + asin(200 / (float)SigleObs_MaxAngle_Dis) * 1800 / 3.14;
                        if (SrandNum[0] > 1800)
                            SrandNum[0] = 1800;
                        b = asin(200 / (float)SigleObs_MaxAngle_Dis) * 1800 / 3.14;
                        double_control_points[0].x = 0;
                        double_control_points[0].y = 0;

                        double_control_points[1].x = 0;
                        double_control_points[1].y = ROLLWINDOW_R * sin(SrandNum[0] * 3.14 / 1800) / 2;

                        double_control_points[2].x = ROLLWINDOW_R * cos(SrandNum[0] * 3.14 / 1800);
                        double_control_points[2].y = ROLLWINDOW_R * sin(SrandNum[0] * 3.14 / 1800) / 2;

                        double_control_points[3].x = ROLLWINDOW_R * cos(SrandNum[0] * 3.14 / 1800);
                        double_control_points[3].y = ROLLWINDOW_R * sin(SrandNum[0] * 3.14 / 1800);

                        ComputeDoubleBezier(double_control_points, TrackNodeNum, double_control_r);
                        //Track_Flag=1;
                    }

                    printf("a=%d,b=%d\n", a, b);
                    printf("SigleObs_MaxAngle_Dis=%d\n", SigleObs_MaxAngle_Dis);
                    printf("SigleObs_MinAngle_Dis=%d\n", SigleObs_MinAngle_Dis);
                    printf("SigleObs_MaxAngle=%d\n", SigleObs_MaxAngle);
                    printf("SigleObs_MinAngle=%d\n", SigleObs_MinAngle);
                    printf("have single obs   reachable path-----------------------------\n");
                    printf("SrandNum[0]=%d\n", SrandNum[0]);
                    for (i = 0; i < 4; i++)
                    {
                        printf("x=%d,y=%d\n", double_control_points[i].x, double_control_points[i].y);
                    }
                    printf("\n");
                }
            }

            //---鍗曚晶闅滅鐗╀笖瑙嗛噹瑕嗙洊澶�-----------------------------
            if (3 == HaveDoubleObsPath)
            {
                if (abs(tmpminangle_f - 0) >= abs(1800 - tmpmaxangle_f))
                    SrandNum[0] = 10;
                else
                    SrandNum[0] = 1800;
                printf("tmpminangle_f=%d,tmpmaxangle_f=%d\n", tmpminangle_f, tmpmaxangle_f);
                printf("te shu huan jing!+++++++++++++++++++++++++++++++++++\n");
            }
        }

        //---鏈哄櫒浜鸿兘澶熻繍琛岀殑鍓嶆彁鏄湁鍓嶈繘鏂瑰悜---------------
        //--涓旀柟鍚戞槸鐢辩獥鍙ｅ唴鏃犻殰纰嶇墿鎴栬€呮湁涓€渚ч殰纰嶇墿----
        //-----鎴栬€呬袱渚ч殰纰嶇墿鎵€閫夊嚭鏉ョ殑璺緞--------------------
        if (SrandNum[0] && (HaveDoubleObsPath || HaveNoObs))
        {
            isArrivedDirect_Flag = 1;
            HaveDoubleObsPath = 0;
        }

        tmp_maxrad = 0;
        tmpminangle_f = 0;
        tmpmaxangle_f = 0;
        memset(&Lidar_Data, 0, sizeof(Lidar_Data));
        Lidar_Ok_Flag = 0;
    }

    if (Lidar_Circle_Cnt > 0)
    {
        AimCount = 0;
        memset(CurLidarAng, 0, sizeof(CurLidarAng));
        memset(CurLidarDistPix, 0, sizeof(CurLidarDistPix));

        TrueAimCount = 0;
        memset(TrueCurLidarDistPix, 0, sizeof(TrueCurLidarDistPix));
        memset(TrueCurLidarAng, 0, sizeof(TrueCurLidarAng));

        TrueAimCntDis = 0;
        memset(TrueCurLidarDistance, 0, sizeof(TrueCurLidarDistance));
        memset(TrueCurLidarAngle, 0, sizeof(TrueCurLidarAngle));

        FilterCount = 0;
        memset(FilterLidarDistPix, 0, sizeof(FilterLidarDistPix));
        memset(FilterLidarAng, 0, sizeof(FilterLidarAng));
        FilterCount2 = 0;
        memset(FilterLidarDistance, 0, sizeof(FilterLidarDistance));
        memset(FilterLidarAng2, 0, sizeof(FilterLidarAng2));
    }

    //printf("isArrivedDirect_Flag=%d\n",isArrivedDirect_Flag);

    //--------------------RRT浜х敓涓€涓殢鏈鸿矾寰�----------------------

    //--------鏄剧ず鍙璺緞鐨勮竟鐣�---------------------------------

    //----------闆疯揪鎵埌鐨勮繎浼艰竟鐣岀偣-------------------------------

    //-------------------------------------------------------------------------

    LidarEdgeCnt = 0;
    memset(LidarEdgeDistPix, 0, 1024);
    memset(LidarEdgeDistance, 0, 1024);
    memset(LidarEdgeAngle, 0, 1024);

    LidarOkPathMidCnt = 0;
    memset(LidarOkPathMidAng, 0, 10);
    memset(LidarOkPathMidPix, 0, 10);

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

    //------------------------灞忓箷涓婄敾鍦�-----------------------------------
    //-----鐢荤獥鍙�-------//

    tmp_rollwindow_r = ROLLWINDOW_R / ROLLWINDOW_RITO + 130;
    if ((tmp_rollwindow_r > 150) && (tmp_rollwindow_r < (618 + 300)))
    {
        ROLLWINDOW_PIX = 17.6132 * pow(0.0099 * ((tmp_rollwindow_r - 430)), 0.8712) + 41;
        //printf("di 1 ge...\n");
    }
    if ((tmp_rollwindow_r > (618 + 300)) && (tmp_rollwindow_r < (995 + 300)))
    {
        ROLLWINDOW_PIX = 9.9210 * pow(0.0918 * ((tmp_rollwindow_r - 430)), 0.5322) + 41;
        //printf("di 2 ge...\n");
    }
    if ((tmp_rollwindow_r > (995 + 300)) && (tmp_rollwindow_r < (2019 + 300)))
    {
        ROLLWINDOW_PIX = 2.7994 * pow(2.4136 * ((tmp_rollwindow_r - 430)), 0.4712) + 41;
        //printf("di 3 ge...\n");
    }
    if (tmp_rollwindow_r > (2019 + 300))
    {
        ROLLWINDOW_PIX = 4.0607 * pow(4.0504 * ((tmp_rollwindow_r - 430)), 0.4028) + 41;
        //printf("di 4 ge...\n");
    }
    //printf("tmp_rollwindow_r=%6.2f,ROLLWINDOW_PIX=%d\n",tmp_rollwindow_r,ROLLWINDOW_PIX);

    for (angle = 0; angle < 6.283; angle = angle + 0.02)
    {
        r = ROLLWINDOW_PIX;
        y = r * sin(angle);
        x = r * cos(angle);
        addr[cell(x, y, 0)] = 0;
        addr[cell(x, y, 1)] = 255;
        addr[cell(x, y, 2)] = 0;
    }

    /*for(angle=0;angle<6.283;angle=angle+0.1)
	{  
		r=20;
		y=r*sin(angle);
		x=r*cos(angle);
		addr[cell(x,y,0)]=255;
		addr[cell(x,y,1)]=255;
		addr[cell(x,y,2)]=255;

		r=40;
		y=r*sin(angle);
		x=r*cos(angle);
		addr[cell(x,y,0)]=0;
		addr[cell(x,y,1)]=255;
		addr[cell(x,y,2)]=0;
	}
	
	for(angle=0;angle<6.283;angle=angle+0.01)
	{
		r=100;
		y=r*sin(angle);
		x=r*cos(angle);
		addr[cell(x,y,0)]=0;
		addr[cell(x,y,1)]=255;
		addr[cell(x,y,2)]=255;


		r=125;
		y=r*sin(angle);
		x=r*cos(angle);
		addr[cell(x,y,0)]=255;
		addr[cell(x,y,1)]=255;
		addr[cell(x,y,2)]=255;

		r=150;
		y=r*sin(angle);
		x=r*cos(angle);
		addr[cell(x,y,0)]=0;
		addr[cell(x,y,1)]=255;
		addr[cell(x,y,2)]=255;

		r=175;
		y=r*sin(angle);
		x=r*cos(angle);
		addr[cell(x,y,0)]=255;
		addr[cell(x,y,1)]=255;
		addr[cell(x,y,2)]=255;

		r=200;
		y=r*sin(angle);
		x=r*cos(angle);
		addr[cell(x,y,0)]=0;
		addr[cell(x,y,1)]=255;
		addr[cell(x,y,2)]=255;

		r=225;
		y=r*sin(angle);
		x=r*cos(angle);
		addr[cell(x,y,0)]=255;
		addr[cell(x,y,1)]=255;
		addr[cell(x,y,2)]=255;

		r=250;
		y=r*sin(angle);
		x=r*cos(angle);
		addr[cell(x,y,0)]=0;
		addr[cell(x,y,1)]=255;
		addr[cell(x,y,2)]=255;


		r=275;
		y=r*sin(angle);
		x=r*cos(angle);
		addr[cell(x,y,0)]=0;	//b
		addr[cell(x,y,1)]=255;	//g
		addr[cell(x,y,2)]=0;	//r
				
	}
	*/
    //---------------------------------------------------------------------

    //-------------鐢荤洰鏍囩墿----------------------------------------

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

            r = get_r(robot_temp[3].count) + d;
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
            addr[cell(x, y, 2)] = 255;
        }

    //-----------------妯℃嫙闆疯揪鎵弿鐢婚潰---------------------------------------
    /*
	static float scan=0;

	for(angle=0;angle<=6.283+0.005;angle=angle+0.005)
	{
		for(r=0;r<275;r=r+1)
		{
			y=(r*sin(angle));
			x=(r*cos(angle));
			//----------闆疯揪鎵弿绾�-------------
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

void scan(unsigned char *addr) // *addr璞＄礌鏁版嵁瀛樺偍鐨勫湴鏂�
{
    int i = 0;
    //-----------------for寰幆浣撳彉閲�----------------
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

    //--------------------瑙嗚鍧愭爣瀵圭О鏃嬭浆x->y 锛泍->x---------------
    for (ii = -299; ii <= 299; ii++)
    {
        for (jj = -399; jj <= -1; jj++)
        {
            tmp1 = addr[4 * jj - 3200 * ii + 961600];
            addr[4 * jj - 3200 * ii + 961600] = addr[-4 * jj - 3200 * ii + 961600];
            addr[-4 * jj - 3200 * ii + 961600] = tmp1;

            tmp2 = addr[4 * jj - 3200 * ii + 961601];
            addr[4 * jj - 3200 * ii + 961601] = addr[-4 * jj - 3200 * ii + 961601];
            addr[-4 * jj - 3200 * ii + 961601] = tmp2;

            tmp3 = addr[4 * jj - 3200 * ii + 961602];
            addr[4 * jj - 3200 * ii + 961602] = addr[-4 * jj - 3200 * ii + 961602];
            addr[-4 * jj - 3200 * ii + 961602] = tmp3;
        }
    }
    //-------------------------------------------------------------------------

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

            if (4 == rgb(addr, (r * sin(angle)), (r * cos(angle)))) //濡傛灉妫€娴嬩紶鍏ョ殑鍍忕礌鏁版嵁鏄粍鑹�
            {                                                       //rgb(unsigned char *addr,int y,int x)
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
                    //閲囬泦鍒伴粍鑹插紑濮嬪欢浼稿鏋滆繕鏄粍鑹插氨缁х画鍒ゆ柇
                    if (((4 == rgb(addr, (r + i) * sin(angle), (r + i) * cos(angle))) || (4 == rgb(addr, (r + i + 1) * sin(angle), (r + i + 1) * cos(angle)))) && ((4 == rgb(addr, (r - i) * sin(angle), (r - i) * cos(angle))) || (4 == rgb(addr, (r - i - 1) * sin(angle), (r - i - 1) * cos(angle)))))
                    {
                        y_j++;
                        y_r = i;
                    }
                    //绛夊埌涓婁笅涓洪粦鑹诧紝鍋滄
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

    robot_temp[0].count = pixel_j; //瀛樺偍鎬荤洰鏍囨暟
    //---------------------------------------------------------------------
    int start_j = 0;
    int j = 0;
    int pixel_rad[10] = {0};     // 鐢ㄤ簬瀛樺偍鍚勪釜缁勫埆鐨勫儚绱犵偣
    float pixel_angle[10] = {0}; // 鐢ㄤ簬瀛樺偍鍚勪釜缁勫埆鐨勫儚绱犵偣

    float pixel_angle_zero[10] = {0}; // 鐢ㄤ簬瀛樺偍闆剁偣闄勮繎鐨勭偣
    int pixel_rad_zero[10] = {0};
    int pixel_array_zeroj[10] = {0};

    int pixel_array_j[10] = {0}; // 鐢ㄤ簬瀛樺偍鍚勪釜缁勫埆鐨勫儚绱犵偣涓暟

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
            for (i = 0; i < pixel_j; i++) // 鎵惧嚭杩樻病鏈夊綊绫荤殑鍍忕礌鐐�
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

            //---------------------涓嶅悓鐨勫崐寰勫彇涓嶅悓鐨勯檺鍒跺€�---------------------------------------------
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
            for (i = 0; i < pixel_j; i++) // 杩涜姣忎釜鍛ㄦ湡鐨勯亶鍘�
            {
                //-------------------------澶勭悊闆剁偣鐗规畩鐐�------------------------------------
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

                    if (abs(angle_old * 100 - angle_array[i] * 100) < Dif_angle) //10=0.01 杩樻湁鍦ㄤ复鐣岀偣涓婅繕闇€瑕佽€冭檻
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

    //-----------------------绗簩灞傝鲸鍒�-------------------------------------------
    //int lim_rad=0;
    //int lim_angle=0;
    int lim_count = 0;
    int pixel_rad_tmp[10] = {0};     // 鐢ㄤ簬瀛樺偍鍚勪釜缁勫埆鐨勫儚绱犵偣
    float pixel_angle_tmp[10] = {0}; // 鐢ㄤ簬瀛樺偍鍚勪釜缁勫埆鐨勫儚绱犵偣
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

    //------------------------澶勭悊鐗规畩鎯呭喌------------------------------------------

    //-------------------------闅滅鐗╄竟鐣岀簿纭彁鍙�-------------------------------
    int edge = 0;
    int edge_i = 0;
    int shift_pix = 0;
    int e_x = 0;
    int e_y = 0;
    int e_r = 0;
    for (edge = 0; edge < PicEdgePixCnt; edge++)
    {
        e_r = PicEdgePix[edge];
        e_y = e_r * sin(((3600 + 900 - PicEdgeAngle[edge]) % 3600) * 3.14 / 1800);
        e_x = e_r * cos(((3600 + 900 - PicEdgeAngle[edge]) % 3600) * 3.14 / 1800);

        /*	if(((3600+900-PicEdgeAngle[edge]) % 3600)<3590 &&  ((3600+900-PicEdgeAngle[edge]) % 3600) > 10)
		{
			SafeFlag=1;
		}
		else
		{
			SafeFlag=0;
		}
	*/
        for (edge_i = 0; edge_i < 10; edge_i++)
        {
            if (6 == rgb(addr, e_y, e_x))
            {
                showedge_x[FinalEdgePicCnt] = e_x;
                showedge_y[FinalEdgePicCnt] = e_y;
                FinalEdgePicCnt++;
            }
            e_x--;
        }
    }
}

//==========================---link---=====================
PNODE addBack(PNODE phead, int data, char *ip, int fd) //灏鹃儴鎻掑叆
{
    node *pnew = malloc(sizeof(struct info)); //鍒嗛厤鍐呭瓨
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
        while (p->pNext != phead) //寰幆鍒板熬閮�
        {
            p = p->pNext;
        }
        p->pNext = pnew;
        pnew->pNext = phead; //澶村熬鐩歌繛
    }
    return phead;
}

PNODE addBack_rec(PNODE phead, int data, char *ip) //灏鹃儴鎻掑叆
{
    node *pnew = malloc(sizeof(struct info)); //鍒嗛厤鍐呭瓨
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
        while (p->pNext != phead) //寰幆鍒板熬閮�
        {
            p = p->pNext;
        }
        p->pNext = pnew;
        pnew->pNext = phead; //澶村熬鐩歌繛
    }
    return phead;
}

//-----------------閾捐〃鍐掓场鎺掑簭---------------
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
PNODE addFront(PNODE phead, int data) //澶撮儴鎻掑叆
{
    node *pnew = malloc(sizeof(struct info)); //鍒嗛厤鍐呭瓨
    pnew->data = data;

    if (phead == NULL)
    {
        phead = pnew;
        phead->pNext = pnew;
    }
    else
    {
        PNODE p = phead;
        while (p->pNext != phead) //寰幆鍒板熬閮�
        {
            p = p->pNext;
        }
        p->pNext = pnew;
        pnew->pNext = phead; //澶村熬鐩歌繛
        phead = pnew;
    }

    return phead;
}

void showAll(PNODE phead) //鏄炬樉绀哄叏閮�
{
    if (phead == NULL)
    {
        printf("閾捐〃涓虹┖锛乗n");
        return;
    }
    else if (phead->pNext == phead)
    {
        printf("%d %d %d %d %s %p %p\n", phead->data, phead->start_connect_flag,
               phead->connected_flag, phead->iSocketClient,
               phead->send_ip, phead, phead->pNext); //鍙湁涓€涓妭鐐�
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
               p->connected_flag, p->iSocketClient, p->send_ip, p, p->pNext); //鏈€鍚庝竴涓妭鐐广€�
    }
}

PNODE findFirst(PNODE phead, int data) //妫€绱㈡暟鎹�
{
    if (phead == NULL)
    {
        return NULL;
    }
    else if (phead->pNext == phead) //濡傛灉澶磋妭鐐规槸瑕佹煡璇㈢殑鏁版嵁
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
                return p; //濡傛灉鎵惧埌杩斿洖銆�
            }
            else
            {
                p = p->pNext; //鎵句笉鍒扮户缁墠杩�
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

    if (phead == NULL)
    {
        return NULL;
    }
    else if (phead->pNext == phead) //濡傛灉澶磋妭鐐规槸瑕佹煡璇㈢殑鏁版嵁
    {
        //return phead;
        if (strcmp(phead->send_ip, ip) == 0)
        {
            return phead; //濡傛灉鎵惧埌杩斿洖銆�
        }
        else
        {
            return NULL;
        }
    }
    else
    {
        PNODE p = phead;
        while (p->pNext != phead)
        {
            //if (p->data == data)
            if (strcmp(p->send_ip, ip) == 0)

            {
                return p; //濡傛灉鎵惧埌杩斿洖銆�
            }
            else
            {
                p = p->pNext; //鎵句笉鍒扮户缁墠杩�
            }
        }
        //if (p->data == data)
        if (strcmp(p->send_ip, ip) == 0)
            return p;
        return NULL;
    }
}

PNODE deleteFirst(PNODE phead, int data) //鍒犻櫎鏁版嵁
{
    //鍏堝垽鏂鍒犻櫎鐨勬暟鎹槸鍚﹀瓨鍦ㄣ€�
    PNODE p = findFirst(phead, data);
    if (p == NULL)
    {
        printf("娌℃湁妫€绱㈠埌鏁版嵁锛乗n");
        return phead;
    }
    //鍒犻櫎闇€瑕佷娇鐢ㄥ弻鎸囬拡
    PNODE p1, p2;
    p1 = phead;
    p2 = NULL;
    //涓婇潰鐨勫垽鏂浣跨敤锛屽惁鍒欐渶鍚庝竴涓笉濂藉垽鏂�
    while (p1->pNext != phead)
    {
        if (p1->data == data)
        {
            break;
        }
        else
        {
            p2 = p1;
            p1 = p1->pNext; //寰幆涓嬩釜鑺傜偣
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

        phead = phead->pNext; //鏀瑰彉澶磋妭鐐�
        free(p1);             //閲婃斁p1
        p->pNext = phead;
    }

    //
    //  if (p1->pNext == phead)
    //  {
    //      if (p1->data == data)
    //      {
    //          printf("鍒犻櫎鐨勬暟鎹槸:%d %p\n", p1->data, p1);
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
    //                  break;//鎵惧埌鍚庤烦鍑哄惊鐜�
    //              }
    //              else
    //              {
    //                  p2 = p1;
    //                  p1 = p1->pNext;
    //              }
    //          }
    //      /* 杩欏効鏈€鍚庝竴涓暟鎹病鏈夎闂埌锛屾墍浠ュ厛瑕佺敤鏌ヨ锛屾煡璇竴閬� */
    //  }
    //  if (p1 == phead)    //濡傛灉鏄涓€涓�
    //  {
    //      PNODE p = phead;
    //      while (p->pNext != phead)
    //      {
    //          p = p->pNext;    //寰幆鍒板熬閮�
    //      }
    //      phead = phead->pNext;
    //      p->pNext = phead;
    //      printf("鍒犻櫎鐨勬暟鎹槸:%d %p\n", p1->data, p1);
    //      free(p1);
    //      p1 = NULL;
    //  }
    //  else
    //  {
    //      p2->pNext = p1->pNext;
    //      printf("鍒犻櫎鐨勬暟鎹槸:%d %p\n", p1->data, p1);
    //      free(p1);
    //  }
    return phead;
}

PNODE deleteNode(PNODE phead, int data, PNODE *ptemp)
{
    //鍏堝垽鏂鍒犻櫎鐨勬暟鎹槸鍚﹀瓨鍦ㄣ€�
    PNODE p = findFirst(phead, data);
    if (p == NULL)
    {
        printf("娌℃湁妫€绱㈠埌鏁版嵁锛乗n");
        return phead;
    }
    //鍒犻櫎闇€瑕佷娇鐢ㄥ弻鎸囬拡
    PNODE p1, p2;
    p1 = phead;
    p2 = NULL;
    //涓婇潰鐨勫垽鏂浣跨敤锛屽惁鍒欐渶鍚庝竴涓笉濂藉垽鏂�
    while (p1->pNext != phead)
    {
        if (p1->data == data)
        {
            break;
        }
        else
        {
            p2 = p1;
            p1 = p1->pNext; //寰幆涓嬩釜鑺傜偣
        }
    }

    if (p1 != phead)
    {
        p2->pNext = p1->pNext;

        *ptemp = p1->pNext; //鎸囧悜涓嬩釜鑺傜偣

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

        phead = phead->pNext; //鏀瑰彉澶磋妭鐐�

        *ptemp = p1->pNext; //鎸囧悜涓嬩釜鑺傜偣

        free(p1); //閲婃斁p1
        p1 = NULL;
        p->pNext = phead;
    }
    return phead;
}

PNODE insertNode(PNODE phead, int finddata, int data) //鎻掑叆鏁版嵁
{
    PNODE pnew = malloc(sizeof(node));
    pnew->data = data;

    PNODE p = findFirst(phead, finddata);
    if (p == NULL)
    {
        printf("娌℃湁鎵惧埌瑕佹彃鍏ユ暟鎹殑鏍囪锛乗n");
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
            //鎻掑湪鍓嶉潰
            p->pNext = pnew;
            pnew->pNext = phead;
            phead = pnew;

            //鎻掑湪鍚庨潰
            /*  pnew->pNext = phead->pNext; 
			phead->pNext = pnew;*/
        }
        else
        {
            //鎻掑湪鍓嶉潰
            p2->pNext = pnew;
            pnew->pNext = p1;
            //鎻掑湪鍚庨潰
            /*pnew->pNext = p1->pNext; 
			p1->pNext = pnew;*/
        }
    }

    return phead;
}

//---------------------------------------------------------
int getNum(PNODE phead) //杩斿洖閾捐〃鐨勪釜鏁�
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

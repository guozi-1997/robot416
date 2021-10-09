
#include <way.h>

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
    int i = 0, d = 0;
    //-----------------------------------------------------------------------
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

    /*angle=0.01745*30;//0.5235
	for(r=0;r<275;r++)
		{
		y=r*sin(angle);x=r*cos(angle);
		addr[cell(x,y,0)]=255;addr[cell(x,y,1)]=255;addr[cell(x,y,2)]=255;
	    }

	angle=0.01745*4+1.5708;//0.0698
	for(r=0;r<275;r++)
		{
		y=r*sin(angle);x=r*cos(angle);
		addr[cell(x,y,0)]=255;addr[cell(x,y,1)]=255;addr[cell(x,y,2)]=255;
	    }
	    */
    //------------------------------------------------------------------------
    for (angle = 3.141; angle < 6.283; angle = angle + 0.1)
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
        addr[cell(x, y, 0)] = 0;
        addr[cell(x, y, 1)] = 255;
        addr[cell(x, y, 2)] = 0;
    }
    //-------------画目标物-----------------------

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

    //--------------------------------------------------------------
    /*static	float scan=0;

for(angle=0;angle<=6.283+0.005;angle=angle+0.005)
	 {

	for(r=0;r<275;r=r+1)
		{

         y=(r*sin(angle));
          x=(r*cos(angle));
 //----------雷达扫描线-------------
		if(abs(scan*100-angle*100)<3)
 
		 {
 
		//	addr[cell(x,y,0)]=0;
 
		  addr[cell(x,y,1)]=255;
 
		 // addr[cell(x,y,2)]=0;
 
		 }
	  }
     }	 
	 scan=scan+0.4;
	   
   if(scan>=6.28)scan=0;
//-------------------------------------------------------------------------
  */
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

    if (phead == NULL)  //无节点
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
    else    //两个及以上节点
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

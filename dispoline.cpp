#include "dispoline.h"
#include <iostream>
#include <fstream>

DispoLine::DispoLine()
{

}
DispoLine::DispoLine(std::vector<double>InputRpoints)
{
    Rpoints=InputRpoints;
}
//将极坐标点转换为直角坐标下的坐标点
std::vector<std::vector<double>> DispoLine::TransPolToCar(std::vector<double>Rpoints)
{
    std::vector<std::vector<double>> retPointsXY;

    for(int i=0;i<Rpoints.size();i++)
    {
        std::vector<double> temp;
        double tempx=Rpoints[i]*std::cos(i*ANGLERESOLUTION/Rpoints.size()-ANGLE_HALF);
        //std::cout<<Rpoints[i]<<" "<<"X:"<<tempx<<" ";
        double tempy=Rpoints[i]*std::sin(i*ANGLERESOLUTION/Rpoints.size()-ANGLE_HALF);
        //std::cout<<"Y:"<<tempy<<std::endl;
        //std::cout<<cos(i*ANGLERESOLUTION)<<" "<<sin(i*ANGLERESOLUTION)<<std::endl;
        temp.push_back(tempx);
        temp.push_back(tempy);
        retPointsXY.push_back(temp);
    }

    return retPointsXY;

}

//计算两点之间的斜率与截距
SlopAndInter DispoLine::CalSlopAndInte(std::vector<std::vector<double>> pointXY,int index1,int index2)
{
    SlopAndInter retSandI;

    if(std::abs(pointXY[index1][0]-pointXY[index2][0])<EPSI)
    {
        retSandI.slope=1/EPSI;
        //截距
        retSandI.intercept=/*INFNUM*/(pointXY[index1][0]+pointXY[index2][0])/2;
    }
    retSandI.slope=(pointXY[index1][1]-pointXY[index2][1])/
            (pointXY[index1][0]-pointXY[index2][0]);
    retSandI.intercept=pointXY[index1][1]-retSandI.slope*pointXY[index1][0];
    return retSandI;
}

MaxDisAndInd DispoLine::MaxDistOfPoints(SlopAndInter SloAndInte,std::vector<std::vector<double>> pointsXY,int firstIndex,int lastIndex)
{
    MaxDisAndInd retMaxDisAndInt;
    double a=SloAndInte.slope;
    double b=SloAndInte.intercept;
    double maxDis=0;
    for(int i=firstIndex;i<lastIndex;i++)
    {
        double disTemp=std::abs(a*pointsXY[i][0]+b-pointsXY[i][1])/std::sqrt(1+a*a);
        if(maxDis<disTemp)
        {
            maxDis=disTemp;
            retMaxDisAndInt.index=i;
            retMaxDisAndInt.maxDis=maxDis;
        }

    }
    return retMaxDisAndInt;
}
//提取其中的满足条件的转折点
std::vector<int >DispoLine::pointsSave(std::vector<double> Rpoints)
{
    std::vector<int>  retIndex;

    /*************** 1 极坐标点转为直角坐标系坐标点 ******************/
    std::vector<std::vector<double>> retPointsXY=TransPolToCar(Rpoints);

    int first=0;
    int finally=Rpoints.size()-1;
    std::stack<int > memS;
    //第一个参数点的存储
    retIndex.push_back(first);
    //最后一个参数点需要存储
    memS.push(finally);

    bool BREAK=true;
    do
    {
        SlopAndInter retSandI=CalSlopAndInte(retPointsXY,first,finally);

        MaxDisAndInd retMaxDisAndInt=MaxDistOfPoints(retSandI,retPointsXY,first,finally);

        //满足近似直线或者点数较少两种情况下的处理
        if(retMaxDisAndInt.maxDis<DISOFPOINTTOLINE||(finally-first)<NPOINTS)
        {
            //满足条件的情况下，存储满足条件的索引点
            retIndex.push_back(memS.top());
            //满足条件下，需要调整first参数
            first=memS.top();
            //弹出栈顶的索引元素
            memS.pop();
            //设定终端索引的指针
            //跳出循环的结束语句
            if(memS.empty())
            {
                BREAK=false;
                break;
            }
            finally=memS.top();
        }
        //不满足上述的情况下，也就是选择的点是其中的一个转折点的情况下的处理
        else
        {
            //首先存储这个转折点
            memS.push(retMaxDisAndInt.index);
            //更改重点finally这个参数，但是first参数不需要改变
            finally=retMaxDisAndInt.index;
            //std::cout<<"finally: "<<finally<<std::endl;
        }

    }while(BREAK);


    return retIndex;
}



//实现对提取得到的点进行最小二乘拟合
std::vector<FeatureOfFitLine>DispoLine::PlotLine(std::vector<std::vector<double>> PointsXY,
std::vector<int>retIndex)
{
   //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //定义返回值的变量
    std::vector<FeatureOfFitLine> retFeatureOfLine;
    //定义存储的中间变量
    FeatureOfFitLine temp;
    //实现对点云数据的拟合
    for(int i=0;i<retIndex.size()-1;i++)
    {
        /*
        这么做的原因是需要剔除这种情况下的角点
        参见代码解释文档
        */
        int first=retIndex[i]+1;
        int last=retIndex[i+1]-1;
        //判断两个点之间的数量是否满足要求
        if(std::abs(last-first)<NPOINTS)
        {
            continue;
        }
        //当直线的点云数量满足要求的时候
        else
        {
            //判断直线是否垂直
            double maxNum=PointsXY[first][0];
            double minNum=PointsXY[first][0];
            double sumX=0;
            for(int i=first;i<last;i++)
            {
                //关键是观察X轴上面点之间的最大和最小点之间的差距
                if(PointsXY[i][0]>maxNum)
                    maxNum=PointsXY[i][0];
                if(PointsXY[i][0]<minNum)
                    minNum=PointsXY[i][0];
                sumX+=PointsXY[i][0];
            }
            //如果直线垂直的情况下，那么应该是特殊的处理方式
            //这里面我们暂时使用一种比较愚蠢的处理方式
            if(std::abs(maxNum-minNum)<FW_VALUE)
            {
                /*
                 * 当直线垂直于 x 轴的时候，为了精确性，
                 * 我们将 x 当做 y 处理；
                 * 将 y 当做 x 处理。
                */
                double sum_xi_yi=0;
                double sum_x2=0;
                double sum_xi=0;
                double sum_yi=0;
                int n=last-first;

                //将x坐标转为y坐标
                double ytemp_x_to_y=0;
                //将y坐标转为y坐标
                double xtemp_y_to_x=0;

                for (int i=first;i<last;i++)
                {
                    ytemp_x_to_y=PointsXY[i][0];
                    xtemp_y_to_x=PointsXY[i][1];

                    sum_xi+=xtemp_y_to_x;
                    sum_yi+=ytemp_x_to_y;

                    sum_xi_yi+=xtemp_y_to_x*ytemp_x_to_y;
                    sum_x2+=xtemp_y_to_x*xtemp_y_to_x;
                }
                double at=( sum_xi_yi-n*(sum_xi/n)*(sum_yi/n) )/
                        ( sum_x2-n*(sum_xi/n)*(sum_xi/n) );

                double bt=(sum_yi/n)-at*(sum_xi/n);

                double a=1.0/at;
                double b=-bt/at;


                //对直线特征进行记录
                temp.slope=a;
                temp.intercept=b;

                temp.rou=std::abs(b)/std::sqrt(1+a*a);
                temp.fi=std::atan2(b,-a*b);

                temp.beginPoint.y=PointsXY[first][1];
                temp.beginPoint.x=(temp.beginPoint.y-b)/a;

                temp.endPoint.y=PointsXY[last][1];
                temp.endPoint.x=(temp.endPoint.y-b)/a;

                temp.mediumPoint.x=sum_xi/n;
                temp.mediumPoint.y=sum_yi/n;
                temp.lengthLine=std::sqrt(
                            (temp.beginPoint.x-temp.endPoint.x) * (temp.beginPoint.x-temp.endPoint.x) +
                            (temp.beginPoint.y-temp.endPoint.y) * (temp.beginPoint.y-temp.endPoint.y)
                            );

                temp.begin_index=first;
                temp.end_index=last;

                retFeatureOfLine.push_back(temp);

            }
            //如果并非是垂直，是一种常见的倾斜直线的处理方式
            else
            {
                double sum_xi_yi=0;
                double sum_x2=0;
                double sum_xi=0;
                double sum_yi=0;
                int n=last-first;
                for (int i=first;i<last;i++)
                {
                    sum_xi+=PointsXY[i][0];
                    sum_yi+=PointsXY[i][1];

                    sum_xi_yi+=PointsXY[i][0]*PointsXY[i][1];
                    sum_x2+=PointsXY[i][0]*PointsXY[i][0];
                }
                double a=( sum_xi_yi-n*(sum_xi/n)*(sum_yi/n) )/
                        ( sum_x2-n*(sum_xi/n)*(sum_xi/n) );

                double b=(sum_yi/n)-a*(sum_xi/n);

                //对直线特征进行记录
                temp.slope=a;
                temp.intercept=b;

                temp.rou=std::abs(b)/std::sqrt(1+a*a);
                temp.fi=std::atan2(b,-a*b);

                temp.beginPoint.x=PointsXY[first][0];
                temp.beginPoint.y=a*temp.beginPoint.x+b;

                temp.endPoint.x=PointsXY[last][0];
                temp.endPoint.y=a*PointsXY[last][0]+b;

                temp.mediumPoint.x=sum_xi/n;
                temp.mediumPoint.y=sum_yi/n;
                temp.lengthLine=std::sqrt(
                            (temp.beginPoint.x-temp.endPoint.x) * (temp.beginPoint.x-temp.endPoint.x) +
                            (temp.beginPoint.y-temp.endPoint.y) * (temp.beginPoint.y-temp.endPoint.y)
                            );

                temp.begin_index=first;
                temp.end_index=last;

                retFeatureOfLine.push_back(temp);
            }
        }
    }
    return retFeatureOfLine;
}






//实现对直角坐标系下的直线转为极坐标系下的直线特征描述
/*
 * 通过对处理得到的返回值的数据含义：
 * 第一列表示是极径
 * 第二列表示是极角
*/
std::vector<std::vector<double>>DispoLine::rou_fi_trans(std::vector<FeatureOfFitLine> Cartesian_line)
{
    std::vector<std::vector<double>> ret_rou_fi;
    //定义遍历器
    std::vector<FeatureOfFitLine>::iterator it;

    for (it=Cartesian_line.begin();it!=Cartesian_line.end();it++)
    {
        std::vector<double> temp;
        //垂直 x 轴的情况下的霍夫变换
        if(std::abs(it->slope-SLOPE_VALUE)<0.2)
        {
            temp.push_back(std::abs(it->intercept));
            //也就是通过垂直的位置，确定位置
            if(it->intercept<0)
                temp.push_back(0.0);
            else
                temp.push_back(PI);
            ret_rou_fi.push_back(temp);
        }
        //其他情况下的霍夫变换
        else
        {
            std::vector<double> temp;
            double rou_temp=std::abs(it->intercept)/std::sqrt(it->slope*it->slope+1);
            temp.push_back(rou_temp);
            double fi_temp=std::atan2(it->intercept,-it->slope*it->intercept);
            temp.push_back(fi_temp);

            ret_rou_fi.push_back(temp);
        }
    }
    return ret_rou_fi;

}



//直线之间的融合，弥补分割算法的不足以及激光雷达的局限性
/*
保证相近的直线是一条直线，弥补因为IEPF算法的局限性
*/
std::vector<FeatureOfFitLine> DispoLine::LineMerge(std::vector<FeatureOfFitLine> un_merged_line_feature)
{   
    //std::list<FeatureOfFitLine> retLineFeature;
    FeatureOfFitLine temp;
    //定义迭代器进行直线的融合过程
    std::vector<FeatureOfFitLine>::iterator it;
    std::vector<FeatureOfFitLine>::iterator it_j;

    for(it=un_merged_line_feature.begin();it<(un_merged_line_feature.end()-1);it++)
    {
        for(it_j=it+1;it_j<un_merged_line_feature.end();)
        {
            if(
            std::abs(it->rou-it_j->rou)<DISTANCE_ROU_TOLERANCE&&
            std::abs(it->fi-it_j->fi)<ANGLE_FI_TOLERANCE&&
            std::abs(std::sqrt((it->mediumPoint.x-it_j->mediumPoint.x)*(it->mediumPoint.x-it_j->mediumPoint.x)
                    +(it->mediumPoint.y-it_j->mediumPoint.y)*(it->mediumPoint.y-it_j->mediumPoint.y))
                    -((it->lengthLine+it_j->lengthLine)/2))<DISTANCE_OF_SIMILAR_LINE 
                )
                {
                    double mid_x1=-it->slope*it->intercept/(1+it->slope*it->slope);
                    double mid_y1=it->intercept/(1+it->slope*it->slope);

                    double mid_xj1=-it_j->slope*it_j->intercept/(1+it_j->slope*it_j->slope);
                    double mid_yj1=it_j->intercept/(1+it_j->slope*it_j->slope);

                    double mid_x=(mid_x1+mid_xj1)/2;
                    double mid_y=(mid_y1+mid_yj1)/2;

                    temp.fi=std::atan2(mid_y,mid_x);
                    temp.rou=(it->rou+it_j->rou)/2;

                    temp.slope=-1.0/std::tan(temp.fi);
                    temp.intercept=mid_y-temp.slope*mid_x;

                    

                    if(it->begin_index < it_j->begin_index)
                    {
                        temp.begin_index=it->begin_index;
                        temp.end_index=it_j->end_index;

                        temp.beginPoint=it->beginPoint;
                        temp.endPoint=it_j->endPoint;

                        
                    }
                    else
                    {
                        temp.begin_index=it_j->begin_index;
                        temp.end_index=it->begin_index;

                        temp.beginPoint=it_j->beginPoint;
                        temp.endPoint=it->endPoint;

                    }
                    temp.mediumPoint.x=(temp.beginPoint.x+temp.endPoint.x)/2;
                    temp.mediumPoint.y=(temp.beginPoint.y+temp.endPoint.y)/2;

                    temp.lengthLine=std::sqrt(
                        (temp.beginPoint.x-temp.endPoint.x)*(temp.beginPoint.x-temp.endPoint.x)+
                        (temp.beginPoint.y-temp.endPoint.y)*(temp.beginPoint.y-temp.endPoint.y)
                    );
                    *it=temp;
                    it_j=un_merged_line_feature.erase(it_j);
                    
                }
                else
                {
                    it_j++;
                }
        }
        
    }
    //并未处理完成 2019-11-10
    /*
    还需要处理比较短的直线，
    进行剔除操作
    */
    //时间：2019年12月20日，增加功能，对较短的直线进行删除操作
    //时间：2019年12月20日，增加功能，对较远的直线进行删除操作
    std::vector<FeatureOfFitLine> get_long_lines;
    for(int i=0;i<un_merged_line_feature.size();i++)
    {
        if(
            un_merged_line_feature[i].lengthLine> LINE_LENGTH &&
            un_merged_line_feature[i].rou < LINE_DISTANCE &&
            //这部分是对线段中点距离的约束要求
            std::sqrt(
                un_merged_line_feature[i].mediumPoint.x*un_merged_line_feature[i].mediumPoint.x+
                un_merged_line_feature[i].mediumPoint.y*un_merged_line_feature[i].mediumPoint.y
            )<LINE_MID_DISTANCE
            )
        {
            get_long_lines.push_back(un_merged_line_feature[i]);
        }
    }

    /*
    直线特征的存储,存储在txt文本中，用于绑架问题
    */
    ADD_I++;
    if(true==MAP_OBSERVE_SETTING && ADD_I>100 && OBSERVE_SETTING==false)
    {
        //满足这个条件的话，存储直线特征到txt中
        std::ofstream outfile("OBSERVE.txt");//ios::app表示在原文件末尾追加
	    if(!outfile){
		    std::cout << "Open the file failure...\n";
	    //exit(0);
	    }	    
	    //outfile<<"地图"<<"    "<<"预测观测"<<"    "<<"观测"<<std::endl;
        for(int i=0;i<get_long_lines.size();i++)
        {
            outfile<<get_long_lines[i].rou<<std::endl;
            outfile<<get_long_lines[i].fi<<std::endl;
            outfile<<get_long_lines[i].lengthLine<<std::endl;

        }
        OBSERVE_SETTING=true;
        outfile.close();
    }

    return get_long_lines;
}
    










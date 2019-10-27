#ifndef DATATYPE_H
#define DATATYPE_H
#include<Eigen/Dense>
#include<Eigen/Core>

/*
 * 时间：2019-10-22 AM10:56
 * 地点：华中科技大学 机械学院 C202
 * 事件：对数据类型的定义，包括常量的定义，结构体的定义
*/

//常量圆周率PI的定义
const static double PI=3.141592654;
//常量激光雷达分辨率的定义
const static double ANGLERESOLUTION=240*PI/180;
//定义误差的最小极限值
const static double EPSI=1E-6;
//定义最大的数值
const static double INFNUM=1E8;

//计算点到直线距离的时候，设置近似直线的最小阈值，单位是 /mm
const static double DISOFPOINTTOLINE=20;
//形成直线所需要的最少的直线数量
const static int NPOINTS=20;

//Eigen::VectorXf<double,Dynamic> RPoints;
//实现对最大距离以及该点的下标输出
struct MaxDisAndInd
{
    double maxDis;
    int index;
};
//实现斜率和截距的输出
struct SlopAndInter
{
    double slope;
    double intercept;
};

//实现对最小二乘中得到的直线记录，构建结构体
//当直线的斜率的绝对值过大的时候，需要进行纠正设定，
/*
 * 步骤一：
 * 设定直线垂直x轴的时候，设定x轴的方差
 * 步骤二：
 * 设定直线垂直时候的斜率值，使用常数描述
*/
const static double FW_VALUE=20;
const static double SLOPE_VALUE=1E5;

struct StructPoint
{
    double x;
    double y;
};
struct FeatureOfFitLine
{
    /*
     * 包含信息：
     * 拟合直线斜率，
     * 截距，
     * 起点，终点，中点
     * 长度；
    */
    double slope;
    double intercept;
    StructPoint beginPoint;
    StructPoint endPoint;
    StructPoint mediumPoint;
    double lengthLine;

};















#endif // DATATYPE_H

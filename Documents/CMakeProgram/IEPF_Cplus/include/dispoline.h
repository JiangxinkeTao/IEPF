#ifndef DISPOLINE_H
#define DISPOLINE_H

#include<vector>
#include<stack>
#include<list>
#include<string>
#include<algorithm>
#include<Eigen/Dense>
#include<Eigen/Core>

#include"datatype.h"

class DispoLine
{
private:
    //点云数据的定义
    std::vector<double> Rpoints;

public:
    DispoLine();
    //实现对点云数据的输入
    DispoLine(std::vector<double>InputRpoints);
    //将极坐标点转换为直角坐标下的坐标点
    std::vector<std::vector<double>> TransPolToCar(std::vector<double>Rpoints);
    //计算两点之间的斜率与截距
    SlopAndInter CalSlopAndInte(std::vector<std::vector<double>> pointXY,int index1,int index2);
    //寻找距离选定的直线的最大距离的点
    MaxDisAndInd MaxDistOfPoints(SlopAndInter SloAndInte,std::vector<std::vector<double>> pointsXY,int firstIndex,int lastIndex);
    //提取其中的满足条件的转折点
    std::vector<int > pointsSave(std::vector<double> Rpoints);
    //实现对选择的拐点最小二乘拟合，并且记录拟合得到的直线信息
    std::list<FeatureOfFitLine> PlotLine(std::vector<std::vector<double>> PointsXY,std::vector<int>retIndex);
};

#endif // DISPOLINE_H

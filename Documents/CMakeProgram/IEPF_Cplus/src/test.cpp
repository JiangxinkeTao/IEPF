//#include <QCoreApplication>
#include <iostream>
#include <fstream>
#include <istream>
#include <string>

#include <dispoline.h>

int main(int argc, char *argv[])
{
    //std::cout<<std::cos(PI)<<std::endl;
    std::string line;
    std::vector<double> InputPoints;

    std::ifstream myfile("//home//tomas//Documents//QtProgram//QtProgram//points.txt");
    if(!myfile.is_open())
    {
        std::cout<<"Filed to Open TXT"<<std::endl;

    }
    while(std::getline(myfile,line))
    {
        std::cout<<std::stof(line)<<std::endl;
        InputPoints.push_back(std::stof(line));

    }

    DispoLine P(InputPoints);

    std::vector<int > retP=P.pointsSave(InputPoints);

    std::vector<std::vector<double>> PointXY=P.TransPolToCar(InputPoints);
    std::list<FeatureOfFitLine>retFeatrue=P.PlotLine(PointXY,retP);


    std::cout<<"OVER !!!";





    //QCoreApplication a(argc, argv);

    return 0;
}



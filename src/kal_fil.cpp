#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "ros/console.h"
#include <bits/stdc++.h>
#include "kalman/ekfilter.hpp"
#include <cmath>

class cPlaneEKF:public Kalman::EKFilter<double,1,false,true,false>{
public:
        cPlaneEKF();

protected:
        void makeBaseA();
        void makeBaseH();
        void makeBaseV();
        void makeBaseR();
        void makeBaseW();
        void makeBaseQ();

        void makeA();
        void makeProcess();
        void makeMeasure();

        double Period;
};

typedef cPlaneEKF::Vector Vector;
typedef cPlaneEKF::Matrix Matrix;


Vector z(3);
Vector u(0);
double a,b,c,d;

cPlaneEKF::cPlaneEKF() 
{
        setDim(8,0,3,5,5);
        Period = 0.02;
}

void cPlaneEKF::makeBaseA()
{       
        A(1,1) = 0.0;
        A(1,4) = 0.0;
        A(1,5) = 0.0;
        A(1,6) = 0.0;
        A(1,7) = 0.0;
        A(1,8) = 0.0;

        A(2,1) = 0.0;
        A(2,2) = 1.0;
        A(2,4) = 0.0;
        A(2,5) = 0.0;
        A(2,6) = 0.0;
        A(2,7) = 0.0;
        A(2,8) = 0.0;

        A(3,1) = 0.0;
        A(3,2) = 0.0;
        A(3,3) = 1.0;
        A(3,4) = 0.0;
        A(3,5) = 0.0;
        A(3,6) = 0.0;
        A(3,7) = 0.0;
        A(3,8) = 0.0;

        A(4,1) = 0.0;
        A(4,2) = 0.0;
        A(4,3) = 0.0;
        A(4,4) = 1.0;
        A(4,7) = 0.0;
        A(4,8) = 0.0;

        A(5,1) = 0.0;
        A(5,2) = 0.0;
        A(5,3) = 0.0;
        A(5,4) = 0.0;
        A(5,5) = 1.0;
        A(5,7) = 0.0;
        A(5,8) = 0.0;

        A(6,1) = 0.0;
        A(6,2) = 0.0;
        A(6,3) = 0.0;
        A(6,4) = 0.0;
        A(6,5) = 0.0;
        A(6,6) = 1.0;
        A(6,7) = 0.0;
        A(6,8) = 0.0;

        A(7,1) = 0.0;
        A(7,2) = 0.0;
        A(7,3) = 0.0;
        A(7,4) = 0.0;
        A(7,5) = 0.0;
        A(7,6) = 1.0;
        A(7,7) = 0.0;
        
        A(8,1) = 0.0;
        A(8,2) = 0.0;
        A(8,3) = 0.0;
        A(8,4) = 0.0;
        A(8,5) = 0.0;
        A(8,6) = 0.0;
        A(8,7) = 0.0;
        A(8,8) = 1.0;
}

void cPlaneEKF::makeA()
{
    A(1,2)=Period;
    A(1,3)=Period*Period/2;
    A(2,3)=Period;
    A(4,5)=Period;
    A(4,6)=Period*Period/2;
    A(5,6)=Period;
    A(7,8)=Period;
}

void cPlaneEKF::makeBaseW()
{
        W(1,1) = 0.0;
        W(1,2) = 0.0;
        W(1,3) = 0.0;
        W(2,1) = 0.0;
        W(2,2) = 0.0;
        W(2,3) = 0.0;
        W(3,1) = 1.0;
        W(3,2) = 0.0;
        W(3,3) = 0.0;
        W(4,1) = 0.0;
        W(4,2) = 0.0;
        W(4,3) = 0.0;
        W(5,1) = 0.0;
        W(5,2) = 0.0;
        W(5,3) = 0.0;
        W(6,1) = 0.0;
        W(6,2) = 1.0;
        W(6,3) = 0.0;
        W(7,1) = 0.0;
        W(7,2) = 0.0;
        W(7,3) = 0.0;
        W(8,1) = 0.0;
        W(8,2) = 0.0;
        W(8,3) = 1.0;
}

void cPlaneEKF::makeBaseQ()
{
        Q(1,1) = 0.01*0.01;
        Q(1,2) = 0.0;
        Q(1,3) = 0.0;
        Q(2,1) = 0.0;
        Q(2,2) = 0.01*0.01;
        Q(2,3) = 0.0;
        Q(3,1) = 0.0;
        Q(3,3) = 0.01*0.01;
        Q(3,2) = 0.0;
}

void cPlaneEKF::makeBaseH()
{

        H(1,1) = 1.0;
        H(1,2) = 0.0;
        H(1,3) = 0.0;
        H(1,4) = 0.0;
        H(1,5) = 0.0;
        H(1,6) = 0.0;
        H(1,7) = 0.0;
        H(1,8) = 0.0;

        H(2,1) = 0.0;
        H(2,2) = 0.0;
        H(2,3) = 1.0;
        H(2,4) = 0.0;
        H(2,5) = 0.0;
        H(2,6) = 0.0;
        H(2,7) = 0.0;
        H(2,8) = 0.0;

        H(3,1) = 0.0;
        H(3,2) = 0.0;
        H(3,3) = 0.0;
        H(3,4) = 1.0;
        H(3,5) = 0.0;
        H(3,6) = 0.0;
        H(3,7) = 0.0;
        H(3,8) = 0.0;

        H(4,1) = 0.0;
        H(4,2) = 0.0;
        H(4,3) = 0.0;
        H(4,4) = 0.0;
        H(4,5) = 0.0;
        H(4,6) = 1.0;
        H(4,7) = 0.0;
        H(4,8) = 0.0;

        H(5,1) = 0.0;
        H(5,2) = 0.0;
        H(5,3) = 0.0;
        H(5,4) = 0.0;
        H(5,5) = 0.0;
        H(5,6) = 0.0;
        H(5,7) = 1.0;
        H(5,8) = 0.0;        

       
}

void cPlaneEKF::makeBaseV()
{
        V(1,1) = 1.0;
        V(1,2) = 0.0;
        V(1,3) = 0.0;
        V(1,4) = 0.0;
        V(1,5) = 0.0;
        V(2,1) = 0.0;
        V(2,2) = 1.0;
        V(2,3) = 0.0;
        V(2,4) = 0.0;
        V(2,5) = 0.0;
        V(3,1) = 0.0;
        V(3,2) = 0.0;
        V(3,3) = 1.0;
        V(3,4) = 0.0;
        V(3,5) = 0.0;
        V(4,1) = 0.0;
        V(4,2) = 0.0;
        V(4,3) = 0.0;
        V(4,4) = 1.0;
        V(4,5) = 0.0;
        V(5,1) = 0.0;
        V(5,2) = 0.0;
        V(5,3) = 0.0;
        V(5,4) = 0.0;
        V(5,5) = 1.0;
  
}

void cPlaneEKF::makeBaseR()
{       
        R(1,1) = 0.01*0.01;
        R(2,2) = 0.01*0.01;
        R(3,3) = 0.01*0.01;
        R(4,4) = 0.01*0.01;
        R(5,5) = 0.01*0.01;
}

void cPlaneEKF::makeProcess()
{
        Vector x_(x.size());
        x_(1)= x(1) + Period*x(2) + 0.5*Period*Period*x(3);
        x_(2)= x(2) + Period*x(3);
        x_(3)= x(3);
        x_(4)= x(4) + Period*x(5) + 0.5*Period*Period*x(6);
        x_(5)= x(5) + Period*x(6);
        x_(6)= x(6);
        x_(7)= x(7) + Period*x(8);
        x_(8)= x(8);
        std::cout<<x(4)<<","<<x(7)<<","<<x(8)<<"\n";
        x.swap(x_);
        
        
}

void cPlaneEKF::makeMeasure()
{
        z(1)=x(1);
        z(2)=x(3);
        z(3)=x(4);
        z(4)=x(6);
        z(5)=x(7);
}

cPlaneEKF filter;

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{       
        z(1)=msg->pose.pose.position.x;
        z(3)=msg->pose.pose.position.y;
        filter.step(u,z);
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)

{       a=msg->orientation.x;
        b=msg->orientation.y;
        c=msg->orientation.z;
        d=msg->orientation.w;
        z(5)=(atan2(a*d - b*c,a*a + b*b -c*c -d*d))*180/3.14;
        z(2)=msg->linear_acceleration.x;
        z(4)=msg->linear_acceleration.y;
}

int main(int argc, char **argv)
{       
      
        ros::init(argc, argv, "listener");
        ros::NodeHandle n;
        
       
        /*const double P1[8][8]={{1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
                        {0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0},
                        {0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0},
                        {0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0},
                        {0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0},
                        {0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0},
                        {0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0},
                        {0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0}};

        //const double *P=P1;*/
        
        Matrix P0(8,8);
        for (int i=1;i<9;i++)
                for(int j=1;j<9;j++)
                {
                        if(i==j)
                        P0(i,j)=1.0;
                        else
                        P0(i,j)=0.0;
                }

        double x[8];
        x[0]=0.0;
        x[1]=0.0;
        x[2]=0.0;
        x[3]=0.0;
        x[4]=0.0;
        x[5]=0.0;
        x[6]=0.0;
        x[7]=0.0;

        Vector y;
        y.assign(8,x);

        
        filter.init(y,P0);
       
        ros::Subscriber sub1 = n.subscribe("/imu/data", 1000, imuCallback);
        ros::Subscriber sub2 = n.subscribe("/vo", 1000, poseCallback);

        
        ros::spin();
        return 0;

}

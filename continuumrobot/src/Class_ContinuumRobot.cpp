/**
 * @file Class_ContinuumRobot.cpp
 * @brief Unit:SI-m,s,kg
 * 
 * @author Zhang Xu (tjuzx2016@126.com)
 * @date 2022-01-03
 * 
 * @copyright Copyright (c) 2022 Key Laboratory of Mechanism Theory and Equipment Design of the Ministry of Education, School of Mechanical Engineering, Tianjin University, China
 */

#include <ros/ros.h>
#include "ECanVci.h"
#include "continuumrobot/Class_ContinuumRobot.h"
#include "continuumrobot/Class_Motor.h"

namespace NS_ContinuumRobot {
    //public
    ContinuumRobot::ContinuumRobot(){
    }

    ContinuumRobot::~ContinuumRobot(){
        Motor.CloseMotors();
    }



    int ContinuumRobot::InitRobot(){
        FILE *InitTxt = NULL;
        DWORD dwRel;
        //get length of DrivingWire
        if((InitTxt = fopen(FileAddress,"r")) == NULL){
            InitTxt = fopen(FileAddress,"w");  
            fprintf(InitTxt,"%lf,%lf,%lf",Length_Backbone,Length_Backbone,Length_Backbone);
            fclose(InitTxt);
            Length_DrivingWire(0)=Length_DrivingWire(1)=Length_DrivingWire(2)=Length_Backbone;
        }
        else{
            fscanf(InitTxt,"%lf,%lf,%lf",&Length_DrivingWire(0),&Length_DrivingWire(1),&Length_DrivingWire(2));
            ROS_INFO_STREAM("[test]ld = "<<Length_DrivingWire(0)<<" "<<Length_DrivingWire(1)<<" "<<Length_DrivingWire(2));
            fclose(InitTxt);
        }
        //get length of DrivingWire at startup robot 
        Length_DrivingWire__Startup = Length_DrivingWire;

        //init motor:init motors->enable motors->set speed mode
        dwRel = Motor.InitMotors();
        //ROS_INFO_STREAM("test dwr = "<<dwRel);
        if(dwRel != STATUS_OK){
            return dwRel;
        }
        dwRel = Motor.EnableMotors();
        if(dwRel != STATUS_OK){
            return dwRel;
        }
        Motor.SetSpeedMode();

        //reset robot
        ResetRobot();
        return 1;
    }

    int ContinuumRobot::ClearMotorPosition(){
        int dwRel;
        dwRel = Motor.ClearPosition();
        if(dwRel == 0){
            ROS_ERROR_STREAM("[Continuum Robot] Failed to clear position of motors");
            return 0;
        }
        ROS_INFO_STREAM("[Continuum Robot] Clear position of motors successfully");
        return 1;
    }

    int ContinuumRobot::ToConfiguration(double Configuration_Desired[2],int T,int F){
        MatrixXd Velocity_Path;
        int k =0;
        ros::Rate F_Control(F);
        Vector3d Velocity;

        Velocity_Path = TrajectoryGeneration(Configuration_Desired[0],Configuration_Desired[1],T,F);
        Vector2d C,dC;
        DWORD dwRel;
        while(k<T*F){
            ROS_INFO_STREAM("[Continuum Robot]k = "<<k);
            ResetRobot();
            if (k==0){
                C << Configuration(0),Configuration(1);
            }
            else{
                dC<< Velocity_Path(0,k)/F,Velocity_Path(1,k)/F;
                C = C + dC;
            }
            Velocity = JacobianLC*(Velocity_Path.col(k) + 5*(C-Configuration));
            ROS_INFO_STREAM("[test]Velocity_L = "<<Velocity(0)<<" "<<Velocity(1)<<" "<<Velocity(2));

            dwRel = SetVelocity(Velocity);
            if(dwRel==0){
                break;
                ROS_ERROR_STREAM("[Continuum Robot] Failed to set velocity,stop motion");
                return 0;
            }

            k++;
            F_Control.sleep();            
        }

        ROS_INFO_STREAM("[ContinuumRobot] To Configuration:"<<Configuration_Desired[0]<<" "<<Configuration_Desired[1]<<" successfull");
        return 1;
    }

    int ContinuumRobot::ToLength_DrivingWire(double Length_DrivingWire_Desired[3],int T,int F){
        MatrixXd Velocity_Path;
        int k =0;
        ros::Rate F_Control(F);
        Vector3d Velocity;

        Velocity_Path = TrajectoryGeneration(Length_DrivingWire_Desired[0],Length_DrivingWire_Desired[1],Length_DrivingWire_Desired[2],T,F);
        Vector3d L,dL;
        DWORD dwRel;
        while(k<T*F){
            ROS_INFO_STREAM("[Continuum Robot]k = "<<k);
            ResetRobot();
            if (k==0){
                L << Length_DrivingWire(0),Length_DrivingWire(1),Length_DrivingWire(2);
            }
            else{
                dL<< Velocity_Path(0,k)/F,Velocity_Path(1,k)/F,Velocity_Path(2,k)/F;
                L = L + dL;
            }
            Velocity = (Velocity_Path.col(k) + 5*(L-Length_DrivingWire));
            ROS_INFO_STREAM("[test]Velocity_L = "<<Velocity(0)<<" "<<Velocity(1)<<" "<<Velocity(2));

            dwRel = SetVelocity(Velocity);
            if(dwRel==0){
                ROS_ERROR_STREAM("[Continuum Robot] Failed to set velocity,stop motion");
                return 0;
            }

            k++;
            F_Control.sleep();            
        }
        ROS_INFO_STREAM("[ContinuumRobot] To Length_DrivingWire:"<<Length_DrivingWire_Desired[0]<<" "<<Length_DrivingWire_Desired[1]<<" "<<Length_DrivingWire_Desired[2]<<" successfull");
        return 1;
    }


    //private
    void ContinuumRobot::ResetRobot(){
        //Reset position in three space
        ResetLength_DrivingWire();
        ResetConfiguration();
        ResetX();

        //Reset jacobian matrix
        ResetJacobianCX();
        ResetJacobianLC();
        ResetJacobianLX();

        //write data to txt
        FILE *WriteTXT = NULL;
        WriteTXT = fopen(FileAddress,"w+");
        fprintf(WriteTXT,"%lf,%lf,%lf",Length_DrivingWire(0),Length_DrivingWire(1),Length_DrivingWire(2));
        fclose(WriteTXT);
    }

    void ContinuumRobot::ResetLength_DrivingWire(){
        double* R_Motor;
        R_Motor = Motor.GetPosition();
        if(R_Motor)
        for(int i=0;i<3;i++){
            Length_DrivingWire(i) = Length_DrivingWire__Startup(i) + R_Motor[i]*ScrewLead;
        }
        ROS_INFO_STREAM("[test] ld = "<<Length_DrivingWire(0)<<" "<<Length_DrivingWire(1)<<" "<<Length_DrivingWire(2));

    }

    void ContinuumRobot::ResetConfiguration(){        
        if (Length_DrivingWire(0)==Length_DrivingWire(1)&&Length_DrivingWire(1)==Length_DrivingWire(2)){
            Configuration(0) = 0;//theta = 0
            Configuration(1) = 0;//Psi = 0         
        }
        else{
            Configuration(1) = atan2((Length_DrivingWire(2)-Length_Backbone)-(Length_DrivingWire(1)-Length_Backbone)*cos(Beta),-(Length_DrivingWire(1)-Length_Backbone)*sin(Beta));
            Configuration(0) = (Length_DrivingWire(0)-Length_Backbone)/Radius/cos(Configuration(1));
        }
        ROS_INFO_STREAM("[test]config = "<<Configuration(0)<<" "<<Configuration(1));
    }

    void ContinuumRobot::ResetX(){
        if(Configuration(0)==0){
            X = Vector3d(0,0,Length_Backbone);            
        }
        else{
            X(0) = (Length_Backbone*cos(Configuration(1)))/Configuration(0) - (Length_Backbone*cos(Configuration(1))*cos(Configuration(0)))/Configuration(0);
            X(1) = (Length_Backbone*sin(Configuration(1)))/Configuration(0) - (Length_Backbone*sin(Configuration(1))*cos(Configuration(0)))/Configuration(0);
            X(2) = Length_Backbone*sin(Configuration(0))/Configuration(0);
        }
    }

    void ContinuumRobot::ResetJacobianCX(){
        Matrix<double,3,2> JacobianXC;
        double theta = Configuration(0);
        double delta = Configuration(1);
        double L = Length_Backbone;
        double dx1dtheta,dx1ddelta,dy1dtheta,dy1ddelta,dz1dtheta,dz1ddelta;
        if(theta == 0){//Singular posture
            dx1dtheta = (L*cos(delta))/2;
            dx1ddelta = 0;
            dy1dtheta = (L*sin(delta))/2;
            dy1ddelta = 0;
            dz1dtheta = 0;
            dz1ddelta = 0;
        }
        else{
            dx1dtheta = (L*cos(delta)*cos(theta))/pow(theta,2) - (L*cos(delta))/pow(theta,2) + (L*cos(delta)*sin(theta))/theta;
            dx1ddelta = (L*sin(delta)*cos(theta))/theta - (L*sin(delta))/theta;
            dy1dtheta = (L*sin(delta)*cos(theta))/pow(theta,2) - (L*sin(delta))/pow(theta,2) + (L*sin(delta)*sin(theta))/theta;
            dy1ddelta = (L*cos(delta))/theta - (L*cos(delta)*cos(theta))/theta;
            dz1dtheta = (L*cos(theta))/theta - (L*sin(theta))/pow(theta,2);
            dz1ddelta = 0;
        }

        JacobianXC<< dx1dtheta,dx1ddelta,
                     dy1dtheta,dy1ddelta,
                     dz1dtheta,dz1ddelta;     
        JacobianCX = pinv(JacobianXC);   
    }

    void ContinuumRobot::ResetJacobianLC(){
        double theta = Configuration(0);
        double delta = Configuration(1);
        double R = Radius;
        JacobianLC<<          R*cos(delta),          -R*theta*sin(delta),
                       R*cos(Beta + delta),   -R*theta*sin(Beta + delta),
                     R*cos(2*Beta + delta), -R*theta*sin(2*Beta + delta);   
    }

    void ContinuumRobot::ResetJacobianLX(){
        ResetJacobianCX();
        ResetJacobianLC();
        JacobianLX = JacobianLC*JacobianCX;
    }

    MatrixXd ContinuumRobot::pinv(MatrixXd J){	
        MatrixXd S;
        MatrixXd invJ;
        JacobiSVD<MatrixXd> svd(J, ComputeThinU | ComputeThinV);
        int a=svd.singularValues().rows();
        int b=svd.nonzeroSingularValues();
        S.resize(a,a);
        S<<MatrixXd::Zero(a,a);
        int i=0;
        while(i<a)
        {
            if (i<b)
            {
                S(i,i)=1/(svd.singularValues()(i));
                i++;
            }
            else
            {
                S(i,i)=0;
                i++;
            }
        }		
        invJ=svd.matrixV()*S*svd.matrixU().transpose();
        return invJ;
    }

    int ContinuumRobot::SetVelocity(Vector3d Velocity){
        DWORD dwRel;
        double Speed_Motor[3];
        for(int i=0;i<3;i++){
            Speed_Motor[i] = Velocity(i)/ScrewLead*60;//rpm->m/s
        }
        dwRel = Motor.SetSpeed(Speed_Motor);
        if(dwRel == 0){
            ROS_ERROR_STREAM("[Continuum Robot] Set veolocity fail");
            return 0;
        }
        return 1;
    }

        MatrixXd  ContinuumRobot::TrajectoryGeneration(double Configuration_Desired_0,double Configuration_Desired_1,int T,int F){
        double dTheta = Configuration_Desired_0 - Configuration(0);
        double dPsi = Configuration_Desired_1 - Configuration(1);
        double A = dTheta/T;
        double B = dPsi/T;
        double Omega = 2*M_PI/T;
        ArrayXd t=ArrayXd::LinSpaced(T*F,0,T);
        ArrayXd a=ArrayXd::LinSpaced(T*F,A,A);
        ArrayXd b=ArrayXd::LinSpaced(T*F,B,B);
        ArrayXd traa=(Omega*t).cos()*(-A)+a;
        ArrayXd trab=(Omega*t).cos()*(-B)+b;
        MatrixXd Tra(2,T*F);
        Tra.row(0)=traa.matrix().transpose();
        Tra.row(1)=trab.matrix().transpose();
        return Tra;
    }
    MatrixXd  ContinuumRobot::TrajectoryGeneration(double Length_DrivingWire_Desired_0,double Length_DrivingWire_Desired_1,double Length_DrivingWire_Desired_2,int T,int F){
        double dL1 = Length_DrivingWire_Desired_0 - Length_DrivingWire(0);
        double dL2 = Length_DrivingWire_Desired_1 - Length_DrivingWire(1);
        double dL3 = Length_DrivingWire_Desired_2 - Length_DrivingWire(2);
        double A = dL1/T;
        double B = dL2/T;
        double C = dL3/T;
        double Omega = 2*M_PI/T;
        ArrayXd t=ArrayXd::LinSpaced(T*F,0,T);
        ArrayXd a=ArrayXd::LinSpaced(T*F,A,A);
        ArrayXd b=ArrayXd::LinSpaced(T*F,B,B);
        ArrayXd c=ArrayXd::LinSpaced(T*F,C,C);
        ArrayXd traa=(Omega*t).cos()*(-A)+a;
        ArrayXd trab=(Omega*t).cos()*(-B)+b;
        ArrayXd trac=(Omega*t).cos()*(-C)+c;
        MatrixXd Tra(3,T*F);
        Tra.row(0)=traa.matrix().transpose();
        Tra.row(1)=trab.matrix().transpose();
        Tra.row(2)=trac.matrix().transpose();
        return Tra;
    }

}
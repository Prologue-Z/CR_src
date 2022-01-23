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
    ContinuumRobot::ContinuumRobot(ros::NodeHandle nh_)
    : nh(nh_) {
    }

    ContinuumRobot::~ContinuumRobot(){
        Motor.CloseMotors();
        ResetDoc_Length();
    }


    int ContinuumRobot::InitRobot(){
        FILE *InitTxt = NULL;
        DWORD dwRel;
        //get length of DrivingWire
        if((InitTxt = fopen(Add_Record,"rt")) == NULL){
            InitTxt = fopen(Add_Record,"w");  
            fprintf(InitTxt,"%lf,%lf,%lf",Length_Backbone,Length_Backbone,Length_Backbone);
            fclose(InitTxt);
            Length_DrivingWire(0)=Length_DrivingWire(1)=Length_DrivingWire(2)=Length_Backbone;
        }
        else{
            fscanf(InitTxt,"%lf,%lf,%lf",&Length_DrivingWire(0),&Length_DrivingWire(1),&Length_DrivingWire(2));
            fclose(InitTxt);
        }
        //get length of DrivingWire at startup robot 
        Length_DrivingWire__Startup = Length_DrivingWire;
        ROS_INFO_STREAM("[Continuum Robot]Length_DrivingWire__Startup = "<<Length_DrivingWire(0)<<" "<<Length_DrivingWire(1)<<" "<<Length_DrivingWire(2));

        //init motor:init motors->enable motors->set speed mode
        dwRel = Motor.InitMotors();
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

    int ContinuumRobot::ToConfiguration(double Configuration_Desired[2],int T,int F,bool IsWrite){
        int flag = 1;
        double* Length_DrivingWire_Desired = ConfigurationToLength_DrivingWire(Configuration_Desired);
        flag = ToLength_DrivingWire(Length_DrivingWire_Desired,T,F,IsWrite);
        if(flag == 0){
            ROS_ERROR_STREAM("[Continuum Robot] Fail to arrive desired Length_DrivingWire");
            return 0;
        }

        ROS_INFO_STREAM("[Continuum Robot] To Configuration:"<<Configuration_Desired[0]<<" "<<Configuration_Desired[1]<<" successfull");
        return 1;
    }

    int ContinuumRobot::ToLength_DrivingWire(double Length_DrivingWire_Desired[3],int T,int F,bool IsWrite){
        double Kp = 5;
        double Ki = 2;
        double Kd = 0.0001;

        int k =0;
        ros::Rate F_Control(F);
        DWORD dwRel;

        MatrixXd Velocity_Path;
        Vector3d Velocity;          
        Vector3d L_Desired,L_Delta;
        Vector3d Error;
        Vector3d Error_Last;
        Vector3d Errot_Sum;

        Velocity_Path = TrajectoryGeneration(Length_DrivingWire_Desired[0],Length_DrivingWire_Desired[1],Length_DrivingWire_Desired[2],T,F);

        while(k<T*F){
            ROS_INFO_STREAM("[Continuum Robot]k = "<<k);
            ResetRobot();
            ros::spinOnce();
            if(IsWrite == 1){
                WriteTXT();
            }
            
            if (k==0){
                L_Desired = Length_DrivingWire;
                Error_Last << 0,0,0;
                Error << 0,0,0;
                Errot_Sum << 0,0,0;
            }
            else{
                L_Delta << Velocity_Path(0,k)/F,Velocity_Path(1,k)/F,Velocity_Path(2,k)/F;
                L_Desired = L_Desired + L_Delta;
                Error_Last = Error;
                Error = L_Desired - Length_DrivingWire;
                Errot_Sum += Error/F;
            }
            
            Velocity = Velocity_Path.col(k) + Kp*Error + Ki*(Errot_Sum) + Kd*(Error-Error_Last)*F;

            dwRel = SetVelocity(Velocity);
            if(dwRel==0){
                ROS_ERROR_STREAM("[Continuum Robot] Failed to set velocity,stop motion");
                return 0;
            }           

            k++;
            F_Control.sleep(); 
        }
        Velocity<<0,0,0;
        dwRel = SetVelocity(Velocity);
        ROS_INFO_STREAM("[ContinuumRobot] To Length_DrivingWire:"<<Length_DrivingWire_Desired[0]<<" "<<Length_DrivingWire_Desired[1]<<" "<<Length_DrivingWire_Desired[2]<<" successfull");
        return 1;
    }

    void ContinuumRobot::To0Position(){
        int T = 10;
        int F = 30;
        double L_D[3] = {0.3,0.3,0.3};
	    ToLength_DrivingWire(L_D,T,F,0);
    }

    void ContinuumRobot::DataCollection(int Num){
        Vector2d Configuration_Desired;
        double Configuration_Temp1[2];
        double Configuration_Temp2[2];

        int T = 10;
        int F = 30;
        double Configuration0_Rand;
        double Configuration1_Rand;

        ros::Subscriber Sub_Force = nh.subscribe("Topic_Force",1,&NS_ContinuumRobot::ContinuumRobot::Force_CallBack,this);
        
        while(Num>0){
            //get random Configuration_Desired            
            Configuration0_Rand = double(rand()%1000/1000)*PI/6 + PI/3;
            Configuration1_Rand = double(rand()%1000/1000)*2*PI/3 - PI/3;
            Configuration_Temp1[0] = Configuration0_Rand;
            Configuration_Temp1[1] = Configuration(1) + Configuration0_Rand;
            Configuration0_Rand = double(rand()%1000/1000)*PI/6;
            Configuration1_Rand = double(rand()%1000/1000)*2*PI/3 - PI/3;
            Configuration_Temp2[0] = Configuration0_Rand;            
            Configuration_Temp2[1] = Configuration(1) + Configuration0_Rand;

            ToConfiguration(Configuration_Temp1,T,F,1);
            ToConfiguration(Configuration_Temp2,T,F,1);
            Num = Num - 2*T*F;
        }
        To0Position();
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

    }

    void ContinuumRobot::ResetLength_DrivingWire(){
        double* R_Motor;
        R_Motor = Motor.GetPosition();
        if(R_Motor)
        for(int i=0;i<3;i++){
            Length_DrivingWire(i) = Length_DrivingWire__Startup(i) + R_Motor[i]*ScrewLead;
        }
        ROS_INFO_STREAM("[Continuum Robot] Length_DrivingWire = "<<Length_DrivingWire(0)<<" "<<Length_DrivingWire(1)<<" "<<Length_DrivingWire(2));

    }

    void ContinuumRobot::ResetConfiguration(){        
        if (Length_DrivingWire(0)==Length_DrivingWire(1)&&Length_DrivingWire(1)==Length_DrivingWire(2)){
            Configuration(0) = 0;//theta = 0
            Configuration(1) = 0;//Psi = 0         
        }
        else{
            Configuration(1) = atan2(cos(Beta)*(Length_Backbone-Length_DrivingWire(0))-(Length_Backbone-Length_DrivingWire(1)),(Length_Backbone-Length_DrivingWire(0))*sin(Beta));
            Configuration(0) = (Length_Backbone-Length_DrivingWire(0))/Radius/cos(Configuration(1));
        }
        //ROS_INFO_STREAM("[test]C = "<<Configuration(0)<<" "<<Configuration(1));
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


    double* ContinuumRobot::ConfigurationToLength_DrivingWire(double Configuration_Desired[2]){
        double PSI[3];
        double DELTA[3];
        double *Length_DrivingWire_Desired = new double[3];
        for(int i=0;i<3;i++){
            PSI[i] = Configuration_Desired[1] + i*Beta;
            DELTA[i] = Radius * cos(PSI[i]);
            Length_DrivingWire_Desired[i] = Length_Backbone - DELTA[i]*Configuration_Desired[0];
        }
        return Length_DrivingWire_Desired;
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

    void ContinuumRobot::ResetDoc_Length(){
        FILE *WriteTXT = NULL;
        WriteTXT = fopen(Add_Record,"w+");
        fprintf(WriteTXT,"%lf,%lf,%lf",Length_DrivingWire(0),Length_DrivingWire(1),Length_DrivingWire(2));
        fclose(WriteTXT);
    }

    void ContinuumRobot::WriteTXT(){
        FILE *WriteTXT = NULL;
        WriteTXT = fopen(Add_Data,"a+");
        ros::Time Time_Now = ros::Time::now();
        double Time_Double = Time_Now.toSec();
        fprintf(WriteTXT,"%lf %lf %lf %lf %lf %lf %lf\n",Time_Double,Length_DrivingWire(0),Length_DrivingWire(1),Length_DrivingWire(2),Force[0],Force[1],Force[2]);
        fclose(WriteTXT);
    }

    void ContinuumRobot::Force_CallBack(const msgs_continuumrobot::Msg_Force::ConstPtr& Forcemsg){
        ROS_INFO_STREAM("[Topic test]F = "<<Forcemsg->F1<<" "<<Forcemsg->F2<<" "<<Forcemsg->F3);
        Force[0] = Forcemsg->F1;
        Force[1] = Forcemsg->F2;
        Force[2] = Forcemsg->F3;
    }

}

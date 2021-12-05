/*
*  ContinuumRobot.cpp
*
*  Saved on:2021.11.11
*    Author:Xu Zhang
* Institute:Key Laboratory of Mechanism Theory and Equipment Design of the Ministry of Education, School of Mechanical Engineering, Tianjin University, China
*
* Note:Angles are expressed in radians.
*/

//ContinuumRobot
#include <ContinuumRobot_DataCollection/ContinuumRobot.h>

//Eigen3
#include <eigen3/Eigen/Dense>
using namespace Eigen;


namespace ContinuumRobot_NS {
    //public
    ContinuumRobot::ContinuumRobot(double L, double R)
    :Beta(PI*2/3) {
        Backbone_Length = L;
        Radius = R;
        DrivingWire_Length(0)=DrivingWire_Length(1)=DrivingWire_Length(2)=Backbone_Length;
    }

    void ContinuumRobot::InitRobot(){
        FILE *InitTxt=NULL;
        const char * FileAddress = "../doc/Length_Record.txt";
        
        if((InitTxt = fopen(FileAddress,"r")) == NULL){
            InitTxt = fopen(FileAddress,"w");  
            fprintf(InitTxt,"%lf,%lf,%lf",Backbone_Length,Backbone_Length,Backbone_Length);
            fclose(InitTxt);
            DrivingWire_Length(0)=DrivingWire_Length(1)=DrivingWire_Length(2)=Backbone_Length;
        }
        else{
            fscanf(InitTxt,"%lf,%lf,%lf",&DrivingWire_Length(0),&DrivingWire_Length(1),&DrivingWire_Length(2));
            fclose(InitTxt);
        }

        //Definition DrivingWire_Length_Init
        DrivingWire_Length_Init = DrivingWire_Length;
    }

    void ContinuumRobot::ResetRobot(double R[3]){
        //Postion transformation from R of motor(radians) to L of screw(m)
        double L[3];//Driving wire length nows
        for(int i=0;i<sizeof(R);i++){L[i] = DrivingWire_Length_Init(i) - R[i]/(2*PI)/ScrewLead;}//- -> screw direction

        //Reset position in three space
        ResetDrivingWire_Length(L);
        ResetConfiguration();   
        ResetX();

        //Reset jacobian matrix
        ResetJacobianLX();

        //Reset Boundry_Flag
        ResetBoundry_Flag();
    }

    signed short * ContinuumRobot::TransVelocity(Vector3d Driving_Velocity){
        signed short Speed_Reducer[3];
        for(int i=0;i<3;i++){Speed_Reducer[i] = (signed short)(Driving_Velocity(i)/ScrewLead);}
        return Speed_Reducer;
    }

    
    //private
    void ContinuumRobot::ResetDrivingWire_Length(double L[3]){
        for(int i=0;i++;i<sizeof(L))  {DrivingWire_Length(i) = L[i];}
    }
    
    void ContinuumRobot::ResetConfiguration(){        
        if (DrivingWire_Length(0)==DrivingWire_Length(1)&&DrivingWire_Length(1)==DrivingWire_Length(2)){
            Configuration(0) = 0;//theta = 0
            Configuration(1) = 0;//Psi = 0         
        }
        else{
            Configuration(1) = atan2((DrivingWire_Length(2)-Backbone_Length)-(DrivingWire_Length(1)-Backbone_Length)*cos(Beta),-(DrivingWire_Length(1)-Backbone_Length)*sin(Beta));
            Configuration(0) = (DrivingWire_Length(0)-Backbone_Length)/Radius/cos(Configuration(1));
        }
    }

    void ContinuumRobot::ResetX(){
        if(Configuration(0)==0){
            X = Vector3d(0,0,Backbone_Length);            
        }
        else{
            X(0) = (Backbone_Length*cos(Configuration(1)))/Configuration(0) - (Backbone_Length*cos(Configuration(1))*cos(Configuration(0)))/Configuration(0);
            X(1) = (Backbone_Length*sin(Configuration(1)))/Configuration(0) - (Backbone_Length*sin(Configuration(1))*cos(Configuration(0)))/Configuration(0);
            X(2) = Backbone_Length*sin(Configuration(0))/Configuration(0);
        }
    }

    void ContinuumRobot::ResetJacobianCX(){
        Matrix<double,3,2> JacobianXC;
        double theta = Configuration(0);
        double delta = Configuration(1);
        double L = Backbone_Length;
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

    void ContinuumRobot::ResetBoundry_Flag(){
        if(DrivingWire_Length.minCoeff()<=20){
            Boundary_Flag = true;
        }
        else{Boundary_Flag = false;}
    }

	MatrixXd ContinuumRobot::pinv(MatrixXd J)
		{	MatrixXd S;
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
		};
}
#include "GravityPose.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
int main(int argc,char**argv) {
  if(argc<3)return 2;
  std::ifstream list(argv[1]);std::ofstream out(argv[2]);std::string file;
  out<<"current,old,n,model,inliers,relative_t_x,relative_t_y,relative_t_z,qw,qx,qy,qz,yaw_deg,accepted,gravity_error_deg\n"<<std::setprecision(17);
  cv::setRNGSeed(0);cv::setNumThreads(1);
  while(list>>file) {
    std::ifstream in(file);int current,old,n;in>>current>>old>>n;
    Eigen::Vector3d p,po,pic;Eigen::Matrix3d R,Ro,Ric;
    auto vector=[&](Eigen::Vector3d &x){for(int i=0;i<3;++i)in>>x[i];};
    auto matrix=[&](Eigen::Matrix3d &x){for(int i=0;i<3;++i)for(int j=0;j<3;++j)in>>x(i,j);};
    vector(p);matrix(R);vector(po);matrix(Ro);vector(pic);matrix(Ric);
    std::vector<cv::Point3f> xyz;std::vector<cv::Point2f> uv;
    for(int i=0;i<n;++i) {double x,y,z,u,v;in>>x>>y>>z>>u>>v;xyz.emplace_back(x,y,z);uv.emplace_back(u,v);}
    if(!in)return 3;
    for(int model=0;model<2;++model) {
      Eigen::Matrix3d Re=R;Eigen::Vector3d pe=p;int count=0;
      if(model==0) {
        cv::Mat rv,tv,tmp,rr,K=cv::Mat::eye(3,3,CV_64F),D,inliers;
        Eigen::Matrix3d Rcw=(R*Ric).transpose();Eigen::Vector3d tcw=-Rcw*(p+R*pic);
        cv::eigen2cv(Rcw,tmp);cv::Rodrigues(tmp,rv);cv::eigen2cv(tcw,tv);
        cv::solvePnPRansac(xyz,uv,K,D,rv,tv,true,100,10.0/460.0,.99,inliers);
        cv::Rodrigues(rv,rr);Eigen::Matrix3d r;Eigen::Vector3d t;cv::cv2eigen(rr,r);cv::cv2eigen(tv,t);
        Eigen::Matrix3d Rwc=r.transpose();Re=Rwc*Ric.transpose();pe=-Rwc*t-Re*pic;count=inliers.rows;
      } else {std::vector<uchar> status;gravity_pose::estimate(xyz,uv,Ro,Ric,pic,Re,pe,status);count=std::count(status.begin(),status.end(),1);}
      auto yaw=[](const Eigen::Matrix3d &r){return std::atan2(r(1,0),r(0,0))*180/M_PI;};
      double relative_yaw=yaw(R)-yaw(Re);while(relative_yaw>180)relative_yaw-=360;while(relative_yaw<-180)relative_yaw+=360;
      Eigen::Vector3d t=Re.transpose()*(p-pe);Eigen::Matrix3d rel=Re.transpose()*R;Eigen::Quaterniond q(rel);
      double ge=std::acos(std::max(-1.,std::min(1.,(Ro.transpose()*Eigen::Vector3d::UnitZ()).dot(rel*(R.transpose()*Eigen::Vector3d::UnitZ())))))*180/M_PI;
      bool accepted=count>25 && std::abs(relative_yaw)<30 && t.norm()<20;
      out<<current<<','<<old<<','<<n<<','<<model<<','<<count<<','<<t.x()<<','<<t.y()<<','<<t.z()<<','<<q.w()<<','<<q.x()<<','<<q.y()<<','<<q.z()<<','<<relative_yaw<<','<<accepted<<','<<ge<<'\n';
    }
  }
}

#include "GravityPose.h"
#include <iostream>
#include <iomanip>
int main() {
  std::mt19937 rng(20261017);std::uniform_real_distribution<double> u(-1,1);std::normal_distribution<double> noise(0,.001);
  double max_t=0,max_r=0;int passed=0;
  for(int test=0;test<100;++test) {
    double yaw=u(rng)*M_PI;
    Eigen::Matrix3d ref=gravity_pose::rz(u(rng)*M_PI)*Eigen::AngleAxisd(test%2?1.569: .7*u(rng),Eigen::Vector3d::UnitY()).toRotationMatrix()*Eigen::AngleAxisd(u(rng),Eigen::Vector3d::UnitX()).toRotationMatrix();
    Eigen::Matrix3d Ric=Eigen::AngleAxisd(.8,Eigen::Vector3d(1,2,3).normalized()).toRotationMatrix();
    Eigen::Vector3d pic(.02,-.01,.03),p(4*u(rng),4*u(rng),u(rng));
    Eigen::Matrix3d R=gravity_pose::rz(yaw)*ref;Eigen::Vector3d center=p+R*pic;
    std::vector<cv::Point3f> xyz;std::vector<cv::Point2f> uv;
    bool noisy=test>=50;
    for(int i=0;i<100;++i) {
      Eigen::Vector3d pc(u(rng)*2,u(rng)*1.5,3+2*(1+u(rng)));
      Eigen::Vector3d pw=R*Ric*pc+center;xyz.emplace_back(pw.x(),pw.y(),pw.z());
      double x=pc.x()/pc.z(),y=pc.y()/pc.z();
      if(noisy) {x+=noise(rng);y+=noise(rng);if(i<30) {x=u(rng);y=u(rng);}}
      uv.emplace_back(x,y);
    }
    Eigen::Matrix3d estimated;Eigen::Vector3d position;std::vector<uchar> status;
    bool ok=gravity_pose::estimate(xyz,uv,ref,Ric,pic,estimated,position,status,test);
    double t=(position-p).norm(),a=Eigen::AngleAxisd(R.transpose()*estimated).angle();
    int n=std::count(status.begin(),status.end(),1);
    if(!ok || t>(noisy?.06:1e-4) || a>(noisy?.015:1e-4) || n<(noisy?65:100)) {
      std::cerr<<"FAIL "<<test<<" translation "<<t<<" rotation "<<a<<" inliers "<<n<<std::endl;return 1;
    }
    max_t=std::max(max_t,t);max_r=std::max(max_r,a);++passed;
  }
  // Parallel rays must return no unstable hypothesis.
  std::vector<Eigen::Vector3d> x={{0,0,3},{0,0,4},{0,0,5}},b(3,Eigen::Vector3d(0,0,1));
  if(!gravity_pose::hypotheses(x,b,Eigen::Matrix3d::Identity(),{0,1,2}).empty())return 2;
  std::cout<<std::setprecision(17)<<"{\"passed\":"<<passed<<",\"max_translation_error_m\":"<<max_t<<",\"max_rotation_error_rad\":"<<max_r<<",\"parallel_ray_rejected\":true}"<<std::endl;
}

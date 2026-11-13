#pragma once
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <opencv2/core.hpp>
#include <random>
#include <vector>
#include <algorithm>
#include <complex>

namespace gravity_pose {
using Eigen::Matrix3d; using Eigen::Vector3d;
inline Matrix3d rz(double a) { return Eigen::AngleAxisd(a,Vector3d::UnitZ()).toRotationMatrix(); }
struct Pose { double yaw=0; Vector3d center=Vector3d::Zero(); };
// For fixed yaw, eliminate camera center analytically from orthogonal ray distances.
// The remaining cost is ||u cos(yaw)+v sin(yaw)+w||^2; its stationary points
// are roots of a quartic in tan(yaw/2). All real roots are evaluated.
inline std::vector<Pose> hypotheses(const std::vector<Vector3d>& xyz,
 const std::vector<Vector3d>& bearings,const Matrix3d& R0,const std::vector<int>& ids) {
  Matrix3d A=Matrix3d::Zero(); Vector3d ba=Vector3d::Zero(),bb=ba,bz=ba;
  std::vector<Matrix3d> projectors;
  for(int i:ids) {
    Vector3d d=(R0*bearings[i]).normalized(); Matrix3d M=Matrix3d::Identity()-d*d.transpose();
    projectors.push_back(M);A+=M;
    ba+=M*Vector3d(xyz[i].x(),xyz[i].y(),0);
    bb+=M*Vector3d(xyz[i].y(),-xyz[i].x(),0);
    bz+=M*Vector3d(0,0,xyz[i].z());
  }
  Eigen::SelfAdjointEigenSolver<Matrix3d> spectrum(A);
  if(spectrum.eigenvalues()[0]<1e-9*spectrum.eigenvalues()[2]) return {};
  Vector3d ca=A.ldlt().solve(ba),cb=A.ldlt().solve(bb),cz=A.ldlt().solve(bz);
  double uu=0,vv=0,uv=0,uw=0,vw=0; size_t j=0;
  for(int i:ids) {
    Matrix3d M=projectors[j++];
    Vector3d u=M*(Vector3d(xyz[i].x(),xyz[i].y(),0)-ca);
    Vector3d v=M*(Vector3d(xyz[i].y(),-xyz[i].x(),0)-cb);
    Vector3d w=M*(Vector3d(0,0,xyz[i].z())-cz);
    uu+=u.squaredNorm();vv+=v.squaredNorm();uv+=u.dot(v);uw+=u.dot(w);vw+=v.dot(w);
  }
  std::vector<double> coeff={2*uv+2*vw,4*(vv-uu)-4*uw,-12*uv,-4*(vv-uu)-4*uw,2*uv-2*vw};
  double scale=0;for(double x:coeff) scale=std::max(scale,std::abs(x));
  if(scale<1e-15) return {};
  for(double &x:coeff)x/=scale;
  int degree=4;while(degree>0 && std::abs(coeff[degree])<1e-12)--degree;
  std::vector<double> angles={0,M_PI};
  if(degree>0) {
    Eigen::MatrixXd companion=Eigen::MatrixXd::Zero(degree,degree);
    for(int i=1;i<degree;++i) companion(i,i-1)=1;
    for(int i=0;i<degree;++i) companion(i,degree-1)=-coeff[i]/coeff[degree];
    Eigen::EigenSolver<Eigen::MatrixXd> roots(companion,false);
    for(int i=0;i<degree;++i) {
      auto x=roots.eigenvalues()[i];
      if(std::abs(x.imag())<1e-7*(1+std::abs(x.real())))angles.push_back(2*std::atan(x.real()));
    }
  }
  std::vector<Pose> result;
  for(double a:angles) { Pose p;p.yaw=a;p.center=rz(a)*(ca*std::cos(a)+cb*std::sin(a)+cz);if(p.center.allFinite())result.push_back(p); }
  return result;
}
struct Reprojection {
  Vector3d point; Eigen::Vector2d observation; Matrix3d R0;
  template<class T> bool operator()(const T* const p,T* residual) const {
    T c=cos(p[0]),s=sin(p[0]);
    Eigen::Matrix<T,3,1> delta=point.cast<T>()-Eigen::Matrix<T,3,1>(p[1],p[2],p[3]);
    Eigen::Matrix<T,3,1> rotated(c*delta.x()+s*delta.y(),-s*delta.x()+c*delta.y(),delta.z());
    Eigen::Matrix<T,3,1> pc=R0.transpose().cast<T>()*rotated;
    if(pc.z()<=T(1e-8)) return false;
    residual[0]=pc.x()/pc.z()-T(observation.x());residual[1]=pc.y()/pc.z()-T(observation.y());return true;
  }
};
inline std::vector<int> score(const Pose& p,const std::vector<Vector3d>& xyz,
 const std::vector<Eigen::Vector2d>& uv,const Matrix3d& R0,double threshold,double &error) {
  Matrix3d Rcw=(rz(p.yaw)*R0).transpose();std::vector<int> inliers;error=0;
  for(size_t i=0;i<xyz.size();++i) {
    Vector3d pc=Rcw*(xyz[i]-p.center);if(pc.z()<=1e-8)continue;
    double e=(pc.head<2>()/pc.z()-uv[i]).squaredNorm();
    if(e<=threshold*threshold) { inliers.push_back(i);error+=e; }
  }
  return inliers;
}
inline bool estimate(const std::vector<cv::Point3f>& points,const std::vector<cv::Point2f>& observations,
 const Matrix3d& imu_reference,const Matrix3d& Ric,const Vector3d& pic,
 Matrix3d& Rgi,Vector3d& pgi,std::vector<uchar>& status,unsigned seed=0) {
  status.assign(points.size(),0);if(points.size()<3 || points.size()!=observations.size())return false;
  std::vector<Vector3d> xyz,bearings;std::vector<Eigen::Vector2d> uv;
  // Local centering improves the polynomial's numerical conditioning without pose priors.
  Vector3d origin=Vector3d::Zero();for(auto &p:points)origin+=Vector3d(p.x,p.y,p.z);origin/=points.size();
  for(size_t i=0;i<points.size();++i) { xyz.emplace_back(Vector3d(points[i].x,points[i].y,points[i].z)-origin);uv.emplace_back(observations[i].x,observations[i].y);bearings.emplace_back(uv.back().x(),uv.back().y(),1); }
  Matrix3d R0=imu_reference*Ric; const double threshold=10.0/460.0;
  std::mt19937 random(seed);std::uniform_int_distribution<int> pick(0,points.size()-1);
  Pose best;std::vector<int> best_ids;double best_error=1e300;
  for(int iteration=0;iteration<100;++iteration) {
    std::vector<int> ids;while(ids.size()<3) {int i=pick(random);if(std::find(ids.begin(),ids.end(),i)==ids.end())ids.push_back(i);}
    for(const auto &p:hypotheses(xyz,bearings,R0,ids)) {
      double error;auto inliers=score(p,xyz,uv,R0,threshold,error);
      if(inliers.size()>best_ids.size() || (inliers.size()==best_ids.size() && error<best_error)) {best=p;best_ids=inliers;best_error=error;}
    }
  }
  if(best_ids.size()<3)return false;
  for(int round=0;round<2;++round) {
    double parameters[4]={best.yaw,best.center.x(),best.center.y(),best.center.z()};
    ceres::Problem problem;
    for(int i:best_ids)problem.AddResidualBlock(new ceres::AutoDiffCostFunction<Reprojection,2,4>(new Reprojection{xyz[i],uv[i],R0}),new ceres::HuberLoss(threshold),parameters);
    ceres::Solver::Options options;options.linear_solver_type=ceres::DENSE_QR;options.max_num_iterations=30;options.num_threads=1;
    ceres::Solver::Summary summary;ceres::Solve(options,&problem,&summary);
    if(!summary.IsSolutionUsable())break;
    Pose candidate;candidate.yaw=parameters[0];candidate.center=Vector3d(parameters[1],parameters[2],parameters[3]);
    double error;auto inliers=score(candidate,xyz,uv,R0,threshold,error);
    if(inliers.size()<3)break;
    best=candidate;best_ids=inliers;
  }
  Rgi=rz(best.yaw)*imu_reference;pgi=best.center+origin-Rgi*pic;
  for(int i:best_ids)status[i]=1;
  return Rgi.allFinite()&&pgi.allFinite();
}
}

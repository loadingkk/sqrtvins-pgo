#include "pose_graph.h"
#include "parameters.h"
#include "camodocal/camera_models/EquidistantCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <cstdlib>
camodocal::CameraPtr m_camera;
Eigen::Vector3d tic;
Eigen::Matrix3d qic;
ros::Publisher pub_match_img;
int VISUALIZATION_SHIFT_X=0,VISUALIZATION_SHIFT_Y=0,ROW=0,COL=0,DEBUG_IMAGE=0;
std::string BRIEF_PATTERN_FILE,POSE_GRAPH_SAVE_PATH,VINS_RESULT_PATH;
int main(int argc,char**argv) {
  ros::init(argc,argv,"packet_replay");
  if(argc<4) {
    std::cerr << "Usage: packet_replay PACKETS OUTPUT LOOP_FLAG\n";
    return 2;
  }
  std::string input=argv[1]; POSE_GRAPH_SAVE_PATH=std::string(argv[2])+"/";
  bool loops=std::stoi(argv[3])!=0;
  bool selfcheck=argc>4 && std::string(argv[4])=="selfcheck";
  const char* assets = std::getenv("SQRTVINS_PGO_ASSETS");
  const std::string asset_directory = assets ? assets : "/opt/ws/src/VINS-Fusion/support_files";
  BRIEF_PATTERN_FILE=asset_directory+"/brief_pattern.yml";
  if (!std::ifstream(BRIEF_PATTERN_FILE) || !std::ifstream(asset_directory+"/brief_k10L6.bin"))
    throw std::runtime_error("BRIEF assets are missing from "+asset_directory);
  VINS_RESULT_PATH=POSE_GRAPH_SAVE_PATH+"native_path.csv";
  cv::setNumThreads(1); cv::setRNGSeed(0);
  ros::NodeHandle nh;
  PoseGraph graph; graph.registerPub(nh);
  graph.loadVocabulary(asset_directory+"/brief_k10L6.bin");
  graph.setIMUFlag(true);
  std::vector<KeyFrame*> frames;
  std::ofstream stats(POSE_GRAPH_SAVE_PATH+"packets.csv");
  if (!stats) throw std::runtime_error("Cannot write packet diagnostics");
  stats<<"id,t_cam,t_imu,points,reprojection_px_rms,normalization_max\n"<<std::setprecision(17);
  for(int id=0;;++id) {
    std::ostringstream stem; stem<<input<<"/"<<std::setw(6)<<std::setfill('0')<<id;
    std::ifstream in(stem.str()+".txt"); if(!in) break;
    if(selfcheck && id%100!=50) continue;
    double t,dt,captured,qx,qy,qz,qw,k[8]; int fish;
    Eigen::Vector3d p; Eigen::Matrix3d R;
    in>>t>>dt>>captured>>p.x()>>p.y()>>p.z()>>qx>>qy>>qz>>qw;
    R=Eigen::Quaterniond(qw,qx,qy,qz).normalized().toRotationMatrix();
    in>>tic.x()>>tic.y()>>tic.z()>>qx>>qy>>qz>>qw;
    qic=Eigen::Quaterniond(qw,qx,qy,qz).normalized().toRotationMatrix();
    in>>fish>>COL>>ROW; for(double &v:k) in>>v;
    if(!in || (fish!=0 && fish!=1) || COL<=0 || ROW<=0 || k[0]<=0 || k[1]<=0 ||
       !std::isfinite(t+dt) || !p.allFinite() || !R.allFinite() || !tic.allFinite() || !qic.allFinite())
      throw std::runtime_error("Invalid packet header or unsupported camera model: "+stem.str());
    if(fish) m_camera.reset(new camodocal::EquidistantCamera("cam0",COL,ROW,k[4],k[5],k[6],k[7],k[0],k[1],k[2],k[3]));
    else m_camera.reset(new camodocal::PinholeCamera("cam0",COL,ROW,k[4],k[5],k[6],k[7],k[0],k[1],k[2],k[3]));
    std::vector<cv::Point3f> xyz; std::vector<cv::Point2f> uv,norm; std::vector<double> ids;
    double fid,x,y,z,u,v,un,vn; int n; double err=0,maxnorm=0;
    while(in>>std::ws && in.peek()!=std::char_traits<char>::eof()) {
      if (!(in>>fid>>x>>y>>z>>u>>v>>un>>vn>>n))
        throw std::runtime_error("Malformed feature row: "+stem.str());
      xyz.emplace_back(x,y,z); uv.emplace_back(u,v); norm.emplace_back(un,vn); ids.push_back(fid);
      Eigen::Vector3d pc=qic.transpose()*(R.transpose()*(Eigen::Vector3d(x,y,z)-p)-tic),bearing;
      Eigen::Vector2d projected; m_camera->spaceToPlane(pc,projected);
      err+=(projected-Eigen::Vector2d(u,v)).squaredNorm();
      m_camera->liftProjective(Eigen::Vector2d(u,v),bearing);
      maxnorm=std::max(maxnorm,(bearing.head<2>()/bearing.z()-Eigen::Vector2d(un,vn)).norm());
    }
    cv::Mat image=cv::imread(stem.str()+".png",cv::IMREAD_GRAYSCALE);
    if(image.empty() || image.cols!=COL || image.rows!=ROW) throw std::runtime_error("image mismatch");
    stats<<id<<','<<t<<','<<t+dt<<','<<xyz.size()<<','<<(xyz.empty()?0:std::sqrt(err/xyz.size()))<<','<<maxnorm<<std::endl;
    KeyFrame *kf=new KeyFrame(t+dt,id,p,R,image,xyz,uv,norm,ids,1);
    if(selfcheck) {
      bool accepted=kf->findConnection(kf);
      std::cout<<"SELF_CHECK "<<id<<" accepted "<<accepted<<" translation "<<kf->getLoopRelativeT().norm()<<std::endl;
      delete kf; continue;
    }
    graph.addKeyFrame(kf,loops); frames.push_back(kf);
    std::cout<<"FRAME "<<id<<" POINTS "<<xyz.size()<<" LOOP "<<kf->loop_index<<std::endl;
  }
  graph.stop();
  if (frames.empty()) throw std::runtime_error("No keyframes to optimize");
  std::ofstream out(POSE_GRAPH_SAVE_PATH+"poses.txt"); out<<std::setprecision(17);
  std::ofstream native(POSE_GRAPH_SAVE_PATH+"pose_graph.txt"); native<<std::setprecision(17);
  for(auto *kf:frames) {
    Eigen::Vector3d p; Eigen::Matrix3d R; kf->getPose(p,R); Eigen::Quaterniond q(R);
    out<<kf->time_stamp<<' '<<p.transpose()<<' '<<q.x()<<' '<<q.y()<<' '<<q.z()<<' '<<q.w()<<' '<<kf->loop_index<<'\n';
    Eigen::Quaterniond raw(kf->vio_R_w_i);
    native<<kf->index<<' '<<kf->time_stamp<<' '<<kf->vio_T_w_i.transpose()<<' '<<p.transpose()<<' '
          <<raw.w()<<' '<<raw.vec().transpose()<<' '<<q.w()<<' '<<q.vec().transpose()<<' '
          <<kf->loop_index<<' '<<kf->loop_info.transpose()<<' '<<kf->keypoints.size()<<'\n';
  }
  out.close(); native.close(); stats.close();
  if (!out || !native || !stats) throw std::runtime_error("Failed to write pose graph outputs");
  ros::shutdown(); return 0;
}

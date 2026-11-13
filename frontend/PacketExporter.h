#pragma once
#include "state/State.h"
#include "feat/Feature.h"
#include "feat/FeatureDatabase.h"
#include "feat/FeatureInitializer.h"
#include "cam/CamEqui.h"
#include "cam/CamRadtan.h"
#include "utils/sensor_data.h"
#include <opencv2/imgcodecs.hpp>
#include <boost/filesystem.hpp>
#include <thread>
#include <condition_variable>
#include <deque>
#include <set>
#include <iomanip>
#include <cstdlib>

namespace ov_srvins {
// Experiment-only observer: the worker owns every value it consumes.
class PacketExporter {
  using CameraPoses = std::unordered_map<size_t, std::unordered_map<double, ov_core::FeatureInitializer::ClonePose>>;
  struct Packet {
    double t, dt, captured;
    cv::Mat image;
    Eigen::Matrix<double,3,3> R, Ric;
    Eigen::Vector3d p, pic;
    Eigen::VectorXd intrinsics;
    bool fisheye;
    CameraPoses cameras;
    std::vector<std::shared_ptr<ov_core::Feature>> tracks;
  };
  std::string directory;
  ov_core::FeatureInitializerOptions options;
  std::map<double,cv::Mat> images;
  std::unordered_map<size_t,std::shared_ptr<ov_core::Feature>> history;
  std::shared_ptr<Packet> pending;
  double last = -1;
  std::mutex mutex;
  std::condition_variable ready;
  std::condition_variable space;
  std::deque<std::shared_ptr<Packet>> queue;
  bool stopped = false;
  size_t dropped = 0, written = 0, failed = 0;
  std::thread worker;
public:
  PacketExporter(ov_core::FeatureInitializerOptions opt) : options(opt) {
    const char *path = std::getenv("SQRTVINS_PACKET_DIR");
    if (!path || !*path) return;
    directory = path;
    boost::filesystem::create_directories(directory);
    worker = std::thread([this]{ run(); });
  }
  ~PacketExporter() { finish(); }
  void finish() {
    if (directory.empty() || !worker.joinable()) return;
    { std::lock_guard<std::mutex> lock(mutex); stopped = true; }
    ready.notify_one(); space.notify_all(); worker.join();
    std::ofstream out(directory + "/summary.json");
    out << "{\"written\":" << written << ",\"dropped\":" << dropped
        << ",\"failed\":" << failed << "}\n";
  }
  void remember(const ov_core::CameraData &msg, size_t max_clones) {
    if (directory.empty()) return;
    for (size_t i=0;i<msg.sensor_ids.size();++i)
      if (msg.sensor_ids[i]==0) images[msg.timestamp]=msg.images[i].clone();
    while (images.size()>max_clones+3) images.erase(images.begin());
  }
  void before(const std::shared_ptr<State> &state, const std::shared_ptr<ov_core::FeatureDatabase> &db) {
    pending.reset();
    if (directory.empty()) return;
    // The filter consumes tracks every update. Mirror native measurements before
    // consumption, retaining only the current clone window, independently of it.
    std::vector<double> valid;
    for(auto &clone:state->clones_IMU) valid.push_back(clone.first);
    for(auto &item:db->get_internal_data()) {
      ov_core::Feature copy;
      if(!db->get_feature_clone(item.first,copy)) continue;
      auto &saved=history[item.first];
      if(!saved) { saved=std::make_shared<ov_core::Feature>(); saved->featid=item.first; }
      for(auto &cam:copy.timestamps) {
        auto &times=saved->timestamps[cam.first];
        for(size_t i=0;i<cam.second.size();++i) {
          if(std::find(valid.begin(),valid.end(),cam.second[i])==valid.end()) continue;
          if(std::find(times.begin(),times.end(),cam.second[i])!=times.end()) continue;
          times.push_back(cam.second[i]);
          saved->uvs[cam.first].push_back(copy.uvs.at(cam.first)[i]);
          saved->uvs_norm[cam.first].push_back(copy.uvs_norm.at(cam.first)[i]);
        }
      }
    }
    for(auto it=history.begin();it!=history.end();) {
      it->second->clean_old_measurements(valid);
      size_t count=0; for(auto &cam:it->second->timestamps) count+=cam.second.size();
      if(count==0) it=history.erase(it); else ++it;
    }
    if (state->clones_IMU.size()<size_t(state->options.max_clone_size)) return;
    double t=state->clones_IMU.begin()->first;
    if (t-last<0.1-1e-4 || !images.count(t)) return;
    pending=std::make_shared<Packet>(); pending->t=t; pending->image=images.at(t);
    for (auto &item:history) {
      auto &f=item.second;
      if(!f->timestamps.count(0)) continue;
      auto &times=f->timestamps.at(0);
      if(std::find(times.begin(),times.end(),t)==times.end()) continue;
      pending->tracks.push_back(std::make_shared<ov_core::Feature>(*f));
    }
    last=t;
  }
  void after(const std::shared_ptr<State> &state) {
    if (!pending) return;
    auto packet=pending; pending.reset();
    packet->dt=state->calib_dt_CAMtoIMU->value()(0);
    packet->captured=state->timestamp;
    auto pose=state->clones_IMU.at(packet->t);
    packet->R=pose->Rot().transpose().cast<double>(); packet->p=pose->pos().cast<double>();
    auto ext=state->calib_IMUtoCAM.at(0);
    packet->Ric=ext->Rot().transpose().cast<double>();
    packet->pic=-packet->Ric*ext->pos().cast<double>();
    packet->intrinsics=state->cam_intrinsics.at(0)->value().cast<double>();
    const auto &camera_model = state->cam_intrinsics_cameras.at(0);
    packet->fisheye=bool(std::dynamic_pointer_cast<ov_core::CamEqui>(camera_model));
    if (!packet->fisheye && !std::dynamic_pointer_cast<ov_core::CamRadtan>(camera_model))
      throw std::runtime_error("PGO cam0 requires equidistant or radtan distortion");
    std::vector<double> valid;
    for (auto &clone:state->clones_IMU) {
      valid.push_back(clone.first);
      for (auto &camera:state->calib_IMUtoCAM) {
        Mat3 R=camera.second->Rot()*clone.second->Rot();
        Vec3 p=clone.second->pos()-R.transpose()*camera.second->pos();
        packet->cameras[camera.first].emplace(clone.first,ov_core::FeatureInitializer::ClonePose(R,p));
      }
    }
    for (auto &f:packet->tracks) {
      f->clean_old_measurements(valid);
      for (auto &cam:f->uvs)
        for (size_t i=0;i<cam.second.size();++i)
          f->uvs_norm.at(cam.first).at(i)=state->cam_intrinsics_cameras.at(cam.first)->undistort(cam.second[i]);
    }
    { std::unique_lock<std::mutex> lock(mutex);
      space.wait(lock, [this] { return stopped || queue.size() < 16; });
      if (stopped) throw std::runtime_error("PGO export stopped before enqueue");
      queue.push_back(packet);
    }
    ready.notify_one();
  }
private:
  void run() {
    for (;;) {
      std::shared_ptr<Packet> packet;
      { std::unique_lock<std::mutex> lock(mutex);
        ready.wait(lock,[this]{return stopped || !queue.empty();});
        if(queue.empty()) break;
        packet=queue.front(); queue.pop_front();
        space.notify_one();
      }
      try { save(*packet); ++written; }
      catch(const std::exception &e) { ++failed; std::cerr << "PACKET FAILURE " << e.what() << std::endl; }
    }
  }
  void save(Packet &p) {
    ov_core::FeatureInitializer initializer(options);
    std::ostringstream stem; stem << directory << "/" << std::setw(6) << std::setfill('0') << written;
    std::ofstream out(stem.str()+".txt"); out << std::setprecision(17);
    Eigen::Quaterniond q(p.R), qi(p.Ric);
    out << p.t << ' ' << p.dt << ' ' << p.captured << '\n'
        << p.p.transpose() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n'
        << p.pic.transpose() << ' ' << qi.x() << ' ' << qi.y() << ' ' << qi.z() << ' ' << qi.w() << '\n'
        << p.fisheye << ' ' << p.image.cols << ' ' << p.image.rows << ' ' << p.intrinsics.transpose() << '\n';
    for(auto &f:p.tracks) {
      if(!f->timestamps.count(0)) continue;
      auto &times=f->timestamps.at(0);
      auto it=std::find(times.begin(),times.end(),p.t);
      if(it==times.end()) continue;
      size_t index=it-times.begin(); std::set<double> distinct;
      size_t count=0;
      for(auto &cam:f->timestamps) { distinct.insert(cam.second.begin(),cam.second.end()); count+=cam.second.size(); }
      if(distinct.size()<2) continue;
      if(!initializer.single_triangulation(f,p.cameras)) continue;
      if(options.refine_features && !initializer.single_gaussnewton(f,p.cameras)) continue;
      if(!f->p_FinG.allFinite()) continue;
      out << f->featid << ' ' << f->p_FinG.transpose() << ' ' << f->uvs.at(0).at(index).transpose()
          << ' ' << f->uvs_norm.at(0).at(index).transpose() << ' ' << count << '\n';
    }
    out.close(); if(!out) throw std::runtime_error("packet write failed");
    if(!cv::imwrite(stem.str()+".png",p.image)) throw std::runtime_error("image write failed");
  }
};
}

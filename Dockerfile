FROM ros:noetic-ros-base-focal@sha256:72b8bc59035dc0a5b8e07aae28c16caa84192971d72d207c72ed734fb1d5e97d
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential ca-certificates git libboost-all-dev libceres-dev libeigen3-dev \
    libopencv-dev libopencv-contrib-dev python3-catkin-tools python3-dev python3-matplotlib \
    python3-yaml ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-tf \
    ros-noetic-cmake-modules ros-noetic-rosbag \
    && rm -rf /var/lib/apt/lists/*

ARG BUILD_JOBS=4
WORKDIR /opt/sqrtvins-pgo
COPY vendor/ /opt/ws/src/

COPY frontend /opt/overlay/frontend
COPY backend /opt/ws/src/VINS-Fusion/loop_fusion
COPY scripts/camera-models-opencv4.patch /opt/overlay/
RUN cp /opt/overlay/frontend/{VioManager.cpp,VioManager.h,PacketExporter.h} /opt/ws/src/sqrtVINS/ov_srvins/src/core/ \
    && cp /opt/overlay/frontend/ros1_serial_msckf.cpp /opt/ws/src/sqrtVINS/ov_srvins/src/ \
    && git -C /opt/ws/src/VINS-Fusion apply --ignore-space-change /opt/overlay/camera-models-opencv4.patch \
    && source /opt/ros/noetic/setup.bash \
    && cd /opt/ws \
    && catkin config --extend /opt/ros/noetic --cmake-args -DENABLE_ARUCO_TAGS=OFF \
    && catkin build -j"${BUILD_JOBS}" -p1 ov_srvins ov_eval loop_fusion

COPY scripts /opt/sqrtvins-pgo/scripts
COPY examples /opt/sqrtvins-pgo/examples
ENV OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
ENTRYPOINT ["/bin/bash", "/opt/sqrtvins-pgo/scripts/entrypoint.sh"]

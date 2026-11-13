# Third-party sources

| Component | Upstream revision | Local source and license |
|---|---|---|
| sqrtVINS | [30fafc89](https://github.com/rpng/sqrtVINS/tree/30fafc89d89cfb896d06fdaa68fbda8545e02e9f) | [Upstream source and license](vendor/sqrtVINS/LICENSE). Individual source notices are retained. The Docker build overlays `frontend/`. |
| VINS-Fusion | [be55a937](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/tree/be55a937a57436548ddfb1bd324bc1e9a9e828e0) | [Upstream GPLv3 license](vendor/VINS-Fusion/LICENCE). The vendored subset includes the camera models and BRIEF assets. The derived loop backend lives in `backend/`. |

The EuRoC calibration example derives from the sqrtVINS EuRoC profile. The bag is obtained directly from the dataset publisher. OpenCV, Ceres, Eigen and ROS dependencies are installed from the base image package sources.

Verify the vendored files from the project root with `(cd vendor && sha256sum -c sqrtVINS.sha256 VINS-Fusion.sha256)`.

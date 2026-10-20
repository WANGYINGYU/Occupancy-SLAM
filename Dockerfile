FROM ros:jazzy-ros-base-noble
ARG DEBIAN_FRONTEND=noninteractive
ARG BUILD_JOBS=2
ARG LIBIGL_VERSION=v2.5.0
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake ninja-build git ca-certificates libeigen3-dev libpcl-dev \
    python3-pip python3-venv python3-numpy python3-scipy python3-yaml \
    python3-pil python3-matplotlib python3-h5py \
    ros-jazzy-slam-toolbox ros-jazzy-rviz2 ros-jazzy-rmw-fastrtps-cpp \
    libgl1-mesa-dri libglx-mesa0 x11-utils xauth \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/occupancy-slam
COPY requirements*.txt ./
RUN python3 -m venv --system-site-packages /opt/occupancy-venv \
    && /opt/occupancy-venv/bin/pip install --no-cache-dir -r requirements-matlab.txt
ENV PATH="/opt/occupancy-venv/bin:/opt/occupancy-slam/build:${PATH}" \
    PYTHONDONTWRITEBYTECODE=1 \
    OCCUPANCY_RUNTIME_SETUP=/opt/ros/jazzy/setup.bash \
    QT_X11_NO_MITSHM=1
RUN git clone --depth 1 --branch "${LIBIGL_VERSION}" https://github.com/libigl/libigl.git third_party/libigl \
    && rm -rf third_party/libigl/.git
COPY . .
RUN source /opt/ros/jazzy/setup.bash \
    && cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
       -DBUILD_3D=ON -DBUILD_LEGACY=OFF -DBUILD_TESTING=ON \
    && cmake --build build --parallel "${BUILD_JOBS}" \
    && bash tools/build_rviz.sh
ENTRYPOINT ["/bin/bash", "/opt/occupancy-slam/docker/entrypoint.sh"]
CMD ["--help"]

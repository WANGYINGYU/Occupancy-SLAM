# Container internals

The public entry point is `../tools/docker.sh`; see [Docker usage](../docs/DOCKER.md).
The image contains its own source, C++ build, ROS2 Jazzy, slam_toolbox, RViz grayscale
plugin and Python dependencies. Host builds and `.runtime` are excluded.

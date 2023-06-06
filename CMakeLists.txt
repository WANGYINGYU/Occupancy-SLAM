cmake_minimum_required(VERSION 2.7)
project(Occupancy_SLAM)
set(CMAKE_CXX_STANDARD 17)
# Find Eigen library
find_package(Eigen3 REQUIRED)
# Find OpenMP library
find_package(OpenMP REQUIRED)
# Find OpenCV
find_package(OpenCV REQUIRED)
if(OPENMP_FOUND)
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS} -O3 -march=native")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -O3 -march=native -mavx -mfma -fopenmp-simd -fopenmp=libomp -DNDEBUG")
endif()
#find_package(LIBIGL REQUIRED)
set(LIBIGL_INCLUDE_DIR /Users/yingyuwang/Documents/libigl/include)
include_directories(${LIBIGL_INCLUDE_DIR})
# Add include directories for Eigen library
include_directories(${EIGEN3_INCLUDE_DIR})
add_compile_definitions(EIGEN_USE_THREADS)
add_executable(Occupancy_SLAM main.cpp MyStruct.cpp MyStruct.h SubFuncs.h SubFuncs.cpp FuncLeastSquares.cpp FuncConvertObs.cpp)
target_link_libraries(Occupancy_SLAM PUBLIC Eigen3::Eigen  OpenMP::OpenMP_CXX ${OpenCV_LIBS} ${LIBIGL_LIBRARIES} -fopenmp)

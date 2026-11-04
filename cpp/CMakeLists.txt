cmake_minimum_required(VERSION 3.16)
project(monocular_depth_slam_runners LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(ORB_SLAM3_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/ORB_SLAM3" CACHE PATH "Pinned ORB-SLAM3 checkout")
find_package(OpenCV 4.4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
add_executable(monocular_tum mono_tum_runner.cc)
target_include_directories(monocular_tum PRIVATE
    "${ORB_SLAM3_ROOT}" "${ORB_SLAM3_ROOT}/include" "${ORB_SLAM3_ROOT}/include/CameraModels"
    "${ORB_SLAM3_ROOT}/Thirdparty/Sophus" ${Pangolin_INCLUDE_DIRS})
target_link_libraries(monocular_tum PRIVATE
    "${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so" ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES} Eigen3::Eigen)
add_executable(da3_rgbd_tum da3_rgbd_tum_runner.cc)
target_include_directories(da3_rgbd_tum PRIVATE
    "${ORB_SLAM3_ROOT}" "${ORB_SLAM3_ROOT}/include" "${ORB_SLAM3_ROOT}/include/CameraModels"
    "${ORB_SLAM3_ROOT}/Thirdparty/Sophus" ${Pangolin_INCLUDE_DIRS})
target_link_libraries(da3_rgbd_tum PRIVATE
    "${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so" ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES} Eigen3::Eigen)
add_executable(control_rgbd_tum control_rgbd_tum_runner.cc)
target_include_directories(control_rgbd_tum PRIVATE
    "${ORB_SLAM3_ROOT}" "${ORB_SLAM3_ROOT}/include" "${ORB_SLAM3_ROOT}/include/CameraModels"
    "${ORB_SLAM3_ROOT}/Thirdparty/Sophus" ${Pangolin_INCLUDE_DIRS})
target_link_libraries(control_rgbd_tum PRIVATE
    "${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so" ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES} Eigen3::Eigen)

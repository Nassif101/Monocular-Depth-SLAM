// Project-owned headless replay harness; SLAM algorithms remain in pinned ORB-SLAM3.
#include <System.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

struct InputFrame {
    std::string stamp;
    double timestamp;
    std::string path;
};

int main(int argc, char** argv) {
    if(argc != 7) {
        std::cerr << "Usage: monocular_tum vocabulary settings sequence_root rgb_index output_dir seed\n";
        return 1;
    }
    try {
        std::ifstream input(argv[4]);
        if(!input) throw std::runtime_error("Cannot open RGB index");
        std::vector<InputFrame> frames;
        std::string line;
        while(std::getline(input, line)) {
            line = line.substr(0, line.find('#'));
            std::istringstream stream(line);
            InputFrame frame;
            if(!(stream >> frame.stamp)) continue;
            if(!(stream >> frame.path)) throw std::runtime_error("Malformed RGB index row");
            frame.timestamp = std::stod(frame.stamp);
            if(!std::isfinite(frame.timestamp) ||
               (!frames.empty() && frame.timestamp <= frames.back().timestamp))
                throw std::runtime_error("Invalid/unordered RGB timestamp");
            frames.push_back(frame);
        }
        if(frames.empty()) throw std::runtime_error("RGB index is empty");
        const int seed = std::stoi(argv[6]);
        cv::setNumThreads(1);
        cv::setRNGSeed(seed);
        std::srand(seed);
        const std::string output(argv[5]);
        std::ofstream tracking(output + "/tracking.csv");
        std::ofstream online(output + "/online_trajectory.tum");
        if(!tracking || !online) throw std::runtime_error("Cannot open run output files");
        tracking << "frame_index,timestamp,state,tracked_map_points,keypoints,decode_seconds,tracking_seconds\n";
        online << "# camera-to-world online poses; state OK/OK_KLT only; no final BA correction\n";
        ORB_SLAM3::System slam(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR,
                               false, 0, "tum_freiburg1_xyz");
        if(std::abs(slam.GetImageScale() - 1.f) > 1e-6)
            throw std::runtime_error("Baseline requires unscaled original RGB images");
        std::cout << "Processing " << frames.size() << " RGB-only frames; viewer disabled\n";
        for(size_t index = 0; index < frames.size(); ++index) {
            const auto& frame = frames[index];
            const auto decode_start = std::chrono::steady_clock::now();
            cv::Mat bgr = cv::imread(std::string(argv[3]) + "/" + frame.path, cv::IMREAD_COLOR);
            if(bgr.empty()) throw std::runtime_error("Missing RGB input: " + frame.path);
            if(bgr.cols != 640 || bgr.rows != 480)
                throw std::runtime_error("Baseline expects original 640x480 inputs");
            cv::Mat rgb;
            cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB); // Camera.RGB: 1
            const auto tracking_start = std::chrono::steady_clock::now();
            const auto tcw = slam.TrackMonocular(rgb, frame.timestamp);
            const auto tracking_end = std::chrono::steady_clock::now();
            const double decode = std::chrono::duration<double>(tracking_start-decode_start).count();
            const double elapsed = std::chrono::duration<double>(tracking_end-tracking_start).count();
            const int state = slam.GetTrackingState();
            const auto points = slam.GetTrackedMapPoints();
            const size_t tracked = std::count_if(points.begin(), points.end(),
                                                 [](ORB_SLAM3::MapPoint* point){return point != nullptr;});
            tracking << index << ',' << frame.stamp << ',' << state << ',' << tracked << ','
                     << slam.GetTrackedKeyPointsUn().size() << ',' << std::setprecision(9)
                     << decode << ',' << elapsed << '\n';
            if(state == 2 || state == 5) {
                const auto twc = tcw.inverse();
                const auto translation = twc.translation();
                const auto quaternion = twc.unit_quaternion();
                online << frame.stamp << ' ' << std::fixed << std::setprecision(9)
                       << translation.x() << ' ' << translation.y() << ' ' << translation.z()
                       << ' ' << quaternion.x() << ' ' << quaternion.y() << ' '
                       << quaternion.z() << ' ' << quaternion.w() << '\n';
            }
            if((index+1) % 100 == 0 || index+1 == frames.size()) {
                tracking.flush(); online.flush();
                std::cout << index+1 << '/' << frames.size() << " state=" << state << std::endl;
            }
            // Same paced, no-frame-dropping replay convention as upstream mono_tum.
            const double interval = index+1 < frames.size()
                ? frames[index+1].timestamp-frame.timestamp
                : (index ? frame.timestamp-frames[index-1].timestamp : 0.0);
            if(elapsed < interval)
                std::this_thread::sleep_for(std::chrono::duration<double>(interval-elapsed));
        }
        tracking.close(); online.close();
        slam.Shutdown(); // Tracked patch waits for workers before optimized export.
        const std::string diagnostic = output + "/keyframes_all_maps_diagnostic.tum";
        slam.SaveKeyFrameTrajectoryTUM(diagnostic);
        std::ifstream keyframes(diagnostic);
        size_t n_keyframes = 0;
        while(std::getline(keyframes, line)) if(!line.empty()) ++n_keyframes;
        if(!n_keyframes) {
            std::cerr << "No initialized keyframes; tracking log is retained\n";
            return 2;
        }
        // This upstream exporter supports monocular, selects the largest map,
        // and writes timestamp_ns tx ty tz qx qy qz qw (space-delimited).
        slam.SaveTrajectoryEuRoC(output + "/trajectory_orb_ns.txt");
        slam.SaveKeyFrameTrajectoryEuRoC(output + "/keyframes_orb_ns.txt");
        return 0;
    } catch(const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}

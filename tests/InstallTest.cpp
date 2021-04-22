/*  BSD 3-Clause License
 *
 *  Copyright (c) 2020, FriederPankratz <frieder.pankratz@gmail.com>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/
#include <iostream>

#include <traact/traact.h>
#include <traact/facade/DefaultFacade.h>
#include <traact/util/PerformanceMonitor.h>
#include <traact/component/spatial/util/Pose6DTestSource.h>
//#include <traact/component/spa
//#include "../../spatial_module/src/traact/component/spatial/util/Pose6DTestSource.h"

#include <fstream>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <traact/serialization/JsonGraphInstance.h>
#include <traact/component/facade/ApplicationSyncSink.h>

#include <boost/timer/timer.hpp>
#include <traact/util/Logging.h>
#include <signal.h>

bool running = true;
traact::facade::DefaultFacade myfacade;
void ctrlC(int i) {
    spdlog::info("User requested exit (Ctrl-C).");
    myfacade.stop();
}

void callback_func(traact::TimestampType ts, const Eigen::Affine3d & pose) {
    spdlog::info("callback_func");
}

std::string getIdxName(std::string name, int idx){
    return fmt::format("{0}_{1}", name, idx);
}

void addTracking(const traact::DefaultInstanceGraphPtr& pattern_graph_ptr, int idx, int ir2gray, const std::string& mkv_file, const std::string& result_file){
    using namespace traact::facade;
    using namespace traact;
    using namespace traact::dataflow;

    DefaultPatternInstancePtr
            source_pattern = pattern_graph_ptr->addPattern(getIdxName("source",idx),myfacade.instantiatePattern("KinectAzureSingleFilePlayer"));
    DefaultPatternInstancePtr
            undistort_pattern = pattern_graph_ptr->addPattern(getIdxName("undistorted",idx),myfacade.instantiatePattern("OpenCVUndistortImage"));
    DefaultPatternInstancePtr
            convert_pattern_6000 = pattern_graph_ptr->addPattern(getIdxName("convert_6000",idx), myfacade.instantiatePattern("OpenCvConvertImage"));
    DefaultPatternInstancePtr
            circle_tracking = pattern_graph_ptr->addPattern(getIdxName("circle_tracking",idx), myfacade.instantiatePattern("CircleTracking"));
    DefaultPatternInstancePtr
            estimate_pose = pattern_graph_ptr->addPattern(getIdxName("estimate_pose",idx), myfacade.instantiatePattern("EstimatePose"));
    DefaultPatternInstancePtr
            write_2dlist = pattern_graph_ptr->addPattern(getIdxName("write_2dList",idx), myfacade.instantiatePattern("FileRecorder_cereal_spatial:Position2DList"));

    pattern_graph_ptr->connect(getIdxName("source",idx), "output_ir", getIdxName("undistorted",idx), "input");
    pattern_graph_ptr->connect(getIdxName("source",idx), "output_calibration_ir", getIdxName("undistorted",idx), "input_calibration");
    pattern_graph_ptr->connect(getIdxName("undistorted",idx), "output", getIdxName("convert_6000",idx), "input");


    pattern_graph_ptr->connect(getIdxName("convert_6000",idx), "output", getIdxName("circle_tracking",idx), "input");

    pattern_graph_ptr->connect(getIdxName("circle_tracking",idx), "output", getIdxName("estimate_pose",idx), "input");
    pattern_graph_ptr->connect("source_target", "output", getIdxName("estimate_pose",idx), "input_model");
    pattern_graph_ptr->connect(getIdxName("undistorted",idx), "output_calibration", getIdxName("estimate_pose",idx), "input_calibration");

    pattern_graph_ptr->connect(getIdxName("estimate_pose",idx), "output_points", getIdxName("write_2dList",idx), "input");

    source_pattern->pattern_pointer.parameter["file"]["value"] = mkv_file;
    convert_pattern_6000->pattern_pointer.parameter["irToGray"]["value"] = ir2gray;
    estimate_pose->pattern_pointer.parameter["forceZFaceCamera"]["value"] = false;

    write_2dlist->pattern_pointer.parameter["file"]["value"] = result_file;

}
int mainBundle(int argc, char **argv) {
    using namespace traact::facade;
    using namespace traact;
    using namespace traact::dataflow;

    signal(SIGINT, &ctrlC);

    util::init_logging(spdlog::level::trace,false, "");

    DefaultInstanceGraphPtr pattern_graph_ptr = std::make_shared<DefaultInstanceGraph>("tracking");
    DefaultPatternInstancePtr
            source_target_pattern = pattern_graph_ptr->addPattern("source_target",myfacade.instantiatePattern("FileReader_cereal_spatial:Position3DList"));
    source_target_pattern->pattern_pointer.parameter["file"]["value"] = "/media/frieder/System/data/inm_ba/LTarget.json";

    std::vector<std::string> folders;
    ///media/frieder/System/data/inm_ba/recording_20210408_ARTCalib_Origin/cn01/k4a_capture.mkv
    folders.push_back("/media/frieder/System/data/inm_ba/recording_20210408_ARTCalib_Origin/cn01/");
    folders.push_back("/media/frieder/System/data/inm_ba/recording_20210408_ARTCalib_Origin/cn02/");
    folders.push_back("/media/frieder/System/data/inm_ba/recording_20210408_ARTCalib_Origin/cn03/");
    folders.push_back("/media/frieder/System/data/inm_ba/recording_20210408_ARTCalib_Origin/cn04/");
    folders.push_back("/media/frieder/System/data/inm_ba/recording_20210408_ARTCalib_Origin/cn05/");
    folders.push_back("/media/frieder/System/data/inm_ba/recording_20210408_ARTCalib_Origin/cn06/");


    for(int i=0;i<folders.size();++i){
        std::string folder = folders[i];
        addTracking(pattern_graph_ptr, i, 2000, fmt::format("{0}{1}", folder, "k4a_capture.mkv"), fmt::format("{0}{1}", folder, "LTarget_points.txt"));
    }


    buffer::TimeDomainManagerConfig td_config;
    td_config.time_domain = 0;
    td_config.ringbuffer_size = 10;
    td_config.master_source = "source_0";
    td_config.source_mode = SourceMode::WaitForBuffer;
    td_config.missing_source_event_mode = MissingSourceEventMode::WaitForEvent;
    td_config.max_offset = std::chrono::milliseconds(10);
    td_config.max_delay = std::chrono::milliseconds(100);
    td_config.measurement_delta = std::chrono::nanoseconds(33333333);

    pattern_graph_ptr->timedomain_configs[0] = td_config;





    myfacade.loadDataflow(pattern_graph_ptr);


    myfacade.start();
    while(running){
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    myfacade.stop();
    spdlog::info("exit program");


    return 0;
}
int main(int argc, char **argv) {

    boost::timer::cpu_timer cput;


    using namespace traact::facade;
    using namespace traact;
    using namespace traact::dataflow;

    signal(SIGINT, &ctrlC);

    util::init_logging(spdlog::level::trace,false, "");



    util::PerformanceMonitor monitor("tests",3);


//    DefaultInstanceGraphPtr pattern_graph_ptr = std::make_shared<DefaultInstanceGraph>("vicon_test");
//    DefaultPatternInstancePtr
//            source_pattern = pattern_graph_ptr->addPattern("source",myfacade.instantiatePattern("Vicon_PoseOutput"), true);
//    DefaultPatternInstancePtr
//            sink_pattern = pattern_graph_ptr->addPattern("sink", myfacade.instantiatePattern("Pose6DPrint"));
//    pattern_graph_ptr->connect("source", "output", "sink", "input");
//    source_pattern->pattern_pointer.parameter["server_name"]["value"] = "10.0.110.121";
//    source_pattern->pattern_pointer.parameter["subject"]["value"] = "TipTool_1";

    DefaultInstanceGraphPtr pattern_graph_ptr = std::make_shared<DefaultInstanceGraph>("tracking");
    DefaultPatternInstancePtr
            source_target_pattern = pattern_graph_ptr->addPattern("source_target",myfacade.instantiatePattern("FileReader_cereal_spatial:Position3DList"));

    DefaultPatternInstancePtr
            source_pattern = pattern_graph_ptr->addPattern("source",myfacade.instantiatePattern("KinectAzureSingleFilePlayer"));

    DefaultPatternInstancePtr
            undistort_pattern = pattern_graph_ptr->addPattern("undistorted",myfacade.instantiatePattern("OpenCVUndistortImage"));
    DefaultPatternInstancePtr
            convert_pattern_6000 = pattern_graph_ptr->addPattern("convert_6000", myfacade.instantiatePattern("OpenCvConvertImage"));
    DefaultPatternInstancePtr
            convert_pattern_2000 = pattern_graph_ptr->addPattern("convert_2000", myfacade.instantiatePattern("OpenCvConvertImage"));

    DefaultPatternInstancePtr
            circle_tracking = pattern_graph_ptr->addPattern("circle_tracking", myfacade.instantiatePattern("CircleTracking"));
    DefaultPatternInstancePtr
            estimate_pose = pattern_graph_ptr->addPattern("estimate_pose", myfacade.instantiatePattern("EstimatePose"));


//    DefaultPatternInstancePtr
//            sink_pattern = pattern_graph_ptr->addPattern("sink", myfacade.instantiatePattern("OpenCvWindow"));
    //DefaultPatternInstancePtr
    //        sink_raw_pattern = pattern_graph_ptr->addPattern("sink_raw", myfacade.instantiatePattern("OpenCvWindow"));
    //DefaultPatternInstancePtr
    //        print_tracking_pattern = pattern_graph_ptr->addPattern("print_tracking", myfacade.instantiatePattern("OpenCvPaint2DPositionList"));
        DefaultPatternInstancePtr
            sink_pattern = pattern_graph_ptr->addPattern("sink", myfacade.instantiatePattern("DebugWindow"));
    DefaultPatternInstancePtr
            print_pose = pattern_graph_ptr->addPattern("print_pose", myfacade.instantiatePattern("Pose6DPrint"));
    DefaultPatternInstancePtr
            write_pose = pattern_graph_ptr->addPattern("write_pose", myfacade.instantiatePattern("FileWriter_cereal_spatial:Pose6D"));
    DefaultPatternInstancePtr
            write_calib = pattern_graph_ptr->addPattern("write_calib", myfacade.instantiatePattern("FileWriter_cereal_vision:CameraCalibration"));



    DefaultPatternInstancePtr
            rotate_to_unity1 = pattern_graph_ptr->addPattern("rotate_to_unity1", myfacade.instantiatePattern("StaticPose"));
    DefaultPatternInstancePtr
            rotate_to_unity2 = pattern_graph_ptr->addPattern("rotate_to_unity2", myfacade.instantiatePattern("StaticPose"));
    DefaultPatternInstancePtr
            opencv_to_opengl = pattern_graph_ptr->addPattern("opencv_to_opengl", myfacade.instantiatePattern("StaticPose"));

    DefaultPatternInstancePtr
            mul_target_to_unity_rot = pattern_graph_ptr->addPattern("mul_target_to_unity_rot", myfacade.instantiatePattern("MultiplicationComponent"));
    DefaultPatternInstancePtr
            mul_target_to_unity = pattern_graph_ptr->addPattern("mul_target_to_unity", myfacade.instantiatePattern("MultiplicationComponent"));
    DefaultPatternInstancePtr
            mul_opencv_to_opengl = pattern_graph_ptr->addPattern("mul_opencv_to_opengl", myfacade.instantiatePattern("MultiplicationComponent"));

    //DefaultPatternInstancePtr
    //        write_2dlist = pattern_graph_ptr->addPattern("write_2dList", myfacade.instantiatePattern("FileRecorder_cereal_spatial:Position2DList"));
    //DefaultPatternInstancePtr
    //        play_2dlist = pattern_graph_ptr->addPattern("play_2dList", myfacade.instantiatePattern("FilePlayer_cereal_spatial:Position2DList"));

    pattern_graph_ptr->connect("source", "output_ir", "undistorted", "input");
    pattern_graph_ptr->connect("source", "output_calibration_ir", "undistorted", "input_calibration");
    pattern_graph_ptr->connect("undistorted", "output", "convert_6000", "input");
    //pattern_graph_ptr->connect("convert_6000", "output", "sink", "input");

    pattern_graph_ptr->connect("undistorted", "output", "convert_2000", "input");
    //pattern_graph_ptr->connect("convert_2000", "output", "sink_raw", "input");


    //pattern_graph_ptr->connect("undistorted", "output", "circle_tracking", "input_16Bit");
    pattern_graph_ptr->connect("convert_6000", "output", "circle_tracking", "input");
    //pattern_graph_ptr->connect("undistorted", "output", "circle_tracking", "input");
    pattern_graph_ptr->connect("circle_tracking", "output", "estimate_pose", "input");
    //pattern_graph_ptr->connect("play_2dList", "output", "estimate_pose", "input");
    pattern_graph_ptr->connect("source_target", "output", "estimate_pose", "input_model");
    pattern_graph_ptr->connect("undistorted", "output_calibration", "estimate_pose", "input_calibration");
    pattern_graph_ptr->connect("estimate_pose", "output", "print_pose", "input");

    pattern_graph_ptr->connect("convert_6000", "output", "sink", "input");
    //pattern_graph_ptr->connect("undistorted", "output", "sink_raw", "input");

    pattern_graph_ptr->connect("undistorted", "output_calibration", "sink", "input_intrinsics");
    pattern_graph_ptr->connect("estimate_pose", "output", "sink", "target_pose");
    pattern_graph_ptr->connect("circle_tracking", "output", "sink", "input_2d_tracking");
    //pattern_graph_ptr->connect("play_2dList", "output", "sink", "input_2d_tracking");



    pattern_graph_ptr->connect("rotate_to_unity1", "output", "mul_target_to_unity_rot", "input0");
    pattern_graph_ptr->connect("rotate_to_unity2", "output", "mul_target_to_unity_rot", "input1");

    pattern_graph_ptr->connect("mul_target_to_unity_rot", "output", "mul_target_to_unity", "input0");
    pattern_graph_ptr->connect("estimate_pose", "output", "mul_target_to_unity", "input1");

    pattern_graph_ptr->connect("mul_target_to_unity", "output", "mul_opencv_to_opengl", "input0");
    pattern_graph_ptr->connect("opencv_to_opengl", "output", "mul_opencv_to_opengl", "input1");

    pattern_graph_ptr->connect("mul_opencv_to_opengl", "output", "write_pose", "input");

    pattern_graph_ptr->connect("undistorted", "output_calibration", "write_calib", "input");



    //pattern_graph_ptr->connect("estimate_pose", "output_points", "write_2dList", "input");
    //pattern_graph_ptr->connect("circle_tracking", "output", "print_tracking", "input");



    source_pattern->pattern_pointer.parameter["file"]["value"] = "/media/frieder/System/data/inm_ba/recording_20210408_ARTCalib_Origin/cn01/k4a_capture.mkv";
    //source_pattern->pattern_pointer.parameter["file"]["value"] = "/home/frieder/data/ir05_withTarget/cn03/k4a_capture.mkv";
    //source_target_pattern->pattern_pointer.parameter["file"]["value"] = "/media/frieder/System/data/inm_ba/BoardTarget.json";
    source_target_pattern->pattern_pointer.parameter["file"]["value"] = "/media/frieder/System/data/inm_ba/LTarget.json";
    convert_pattern_6000->pattern_pointer.parameter["irToGray"]["value"] = 2000;
    convert_pattern_2000->pattern_pointer.parameter["irToGray"]["value"] = 1000;
    estimate_pose->pattern_pointer.parameter["forceZFaceCamera"]["value"] = true;
    //print_tracking_pattern->pattern_pointer.parameter["window"]["value"] = "sink_raw";

    write_pose->pattern_pointer.parameter["file"]["value"] = "/media/frieder/System/data/inm_ba/recording_20210408_ARTCalib_Origin/cn01/pose.json";
    write_calib->pattern_pointer.parameter["file"]["value"] = "/media/frieder/System/data/inm_ba/recording_20210408_ARTCalib_Origin/cn01/calib.json";
    //write_2dlist->pattern_pointer.parameter["file"]["value"] = "/media/frieder/System/data/inm_ba/recording_20210305_Calib_CN06/cn01/2dlist.json";
    //play_2dlist->pattern_pointer.parameter["file"]["value"] = "/media/frieder/System/data/inm_ba/recording_20210305_Calib_CN06/cn01/2dlist.json";

    opencv_to_opengl->pattern_pointer.parameter["rx"]["value"] = 1;
    opencv_to_opengl->pattern_pointer.parameter["ry"]["value"] = 0;
    opencv_to_opengl->pattern_pointer.parameter["rz"]["value"] = 0;
    opencv_to_opengl->pattern_pointer.parameter["rw"]["value"] = 0;

    rotate_to_unity1->pattern_pointer.parameter["rx"]["value"] = -0.707106781;
    rotate_to_unity1->pattern_pointer.parameter["ry"]["value"] = 0;
    rotate_to_unity1->pattern_pointer.parameter["rz"]["value"] = 0;
    rotate_to_unity1->pattern_pointer.parameter["rw"]["value"] = 0.707106781;

    rotate_to_unity2->pattern_pointer.parameter["rx"]["value"] = 0;
    rotate_to_unity2->pattern_pointer.parameter["ry"]["value"] = 0;
    rotate_to_unity2->pattern_pointer.parameter["rz"]["value"] = 0.707106781;
    rotate_to_unity2->pattern_pointer.parameter["rw"]["value"] = 0.707106781;

    buffer::TimeDomainManagerConfig td_config;
    td_config.time_domain = 0;
    td_config.ringbuffer_size = 10;
    td_config.master_source = "source";
    td_config.source_mode = SourceMode::WaitForBuffer;
    td_config.missing_source_event_mode = MissingSourceEventMode::WaitForEvent;
    td_config.max_offset = std::chrono::milliseconds(10);
    td_config.max_delay = std::chrono::milliseconds(100);
    td_config.measurement_delta = std::chrono::nanoseconds(33333333);

    pattern_graph_ptr->timedomain_configs[0] = td_config;



    std::string filename = pattern_graph_ptr->name + ".json";
    {
        nlohmann::json jsongraph;
        ns::to_json(jsongraph, *pattern_graph_ptr);

        std::ofstream myfile;
        myfile.open(filename);
        myfile << jsongraph.dump(4);
        myfile.close();

        //std::cout << jsongraph.dump(4) << std::endl;
    }

    myfacade.loadDataflow(filename);




    {
        MEASURE_TIME(monitor, 0, "network start")
        myfacade.start();
    }

    {
        MEASURE_TIME(monitor, 1, "wait for finish source")
        while(running){
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

    }

    {
        MEASURE_TIME(monitor, 2, "finish and wait for network Stop")
        myfacade.stop();
    }


    spdlog::info(monitor.toString());

    spdlog::info("exit program");

    spdlog::info("run the same dataflow again using: traactConsole {0}", filename);

    return 0;
}

int mainX6Playback(int argc, char **argv) {

    boost::timer::cpu_timer cput;


    using namespace traact::facade;
    using namespace traact;
    using namespace traact::dataflow;

    util::init_logging(spdlog::level::trace,false, "");

    DefaultFacade myfacade;

    util::PerformanceMonitor monitor("tests",3);



    DefaultInstanceGraphPtr pattern_graph_ptr = std::make_shared<DefaultInstanceGraph>("tracking");

    DefaultPatternInstancePtr
            source_pattern1 = pattern_graph_ptr->addPattern("source1",myfacade.instantiatePattern("KinectAzureSingleFilePlayer"));
    DefaultPatternInstancePtr
            sink_pattern1 = pattern_graph_ptr->addPattern("sink1", myfacade.instantiatePattern("OpenCvWindow"));

    pattern_graph_ptr->connect("source1", "output", "sink1", "input");
    source_pattern1->pattern_pointer.parameter["file"]["value"] = "/home/frieder/data/ir05_withTarget/cn01/k4a_capture.mkv";

    DefaultPatternInstancePtr
            source_pattern2 = pattern_graph_ptr->addPattern("source2",myfacade.instantiatePattern("KinectAzureSingleFilePlayer"));
    DefaultPatternInstancePtr
            sink_pattern2 = pattern_graph_ptr->addPattern("sink2", myfacade.instantiatePattern("OpenCvWindow"));

    pattern_graph_ptr->connect("source2", "output", "sink2", "input");
    source_pattern2->pattern_pointer.parameter["file"]["value"] = "/home/frieder/data/ir05_withTarget/cn02/k4a_capture.mkv";

    DefaultPatternInstancePtr
            source_pattern3 = pattern_graph_ptr->addPattern("source3",myfacade.instantiatePattern("KinectAzureSingleFilePlayer"));
    DefaultPatternInstancePtr
            sink_pattern3 = pattern_graph_ptr->addPattern("sink3", myfacade.instantiatePattern("OpenCvWindow"));

    pattern_graph_ptr->connect("source3", "output", "sink3", "input");
    source_pattern3->pattern_pointer.parameter["file"]["value"] = "/home/frieder/data/ir05_withTarget/cn03/k4a_capture.mkv";

    DefaultPatternInstancePtr
            source_pattern4 = pattern_graph_ptr->addPattern("source4",myfacade.instantiatePattern("KinectAzureSingleFilePlayer"));
    DefaultPatternInstancePtr
            sink_pattern4 = pattern_graph_ptr->addPattern("sink4", myfacade.instantiatePattern("OpenCvWindow"));

    pattern_graph_ptr->connect("source4", "output", "sink4", "input");
    source_pattern4->pattern_pointer.parameter["file"]["value"] = "/home/frieder/data/ir05_withTarget/cn04/k4a_capture.mkv";

    DefaultPatternInstancePtr
            source_pattern5 = pattern_graph_ptr->addPattern("source5",myfacade.instantiatePattern("KinectAzureSingleFilePlayer"));
    DefaultPatternInstancePtr
            sink_pattern5 = pattern_graph_ptr->addPattern("sink5", myfacade.instantiatePattern("OpenCvWindow"));

    pattern_graph_ptr->connect("source5", "output", "sink5", "input");
    source_pattern5->pattern_pointer.parameter["file"]["value"] = "/home/frieder/data/ir05_withTarget/cn05/k4a_capture.mkv";

    DefaultPatternInstancePtr
            source_pattern6 = pattern_graph_ptr->addPattern("source6",myfacade.instantiatePattern("KinectAzureSingleFilePlayer"));
    DefaultPatternInstancePtr
            sink_pattern6 = pattern_graph_ptr->addPattern("sink6", myfacade.instantiatePattern("OpenCvWindow"));

    pattern_graph_ptr->connect("source6", "output", "sink6", "input");
    source_pattern6->pattern_pointer.parameter["file"]["value"] = "/home/frieder/data/ir05_withTarget/cn06/k4a_capture.mkv";


    buffer::TimeDomainManagerConfig td_config;
    td_config.time_domain = 0;
    td_config.ringbuffer_size = 10;
    td_config.master_source = "source1";
    td_config.source_mode = SourceMode::WaitForBuffer;
    td_config.missing_source_event_mode = MissingSourceEventMode::WaitForEvent;
    td_config.max_offset = std::chrono::milliseconds(10);
    td_config.max_delay = std::chrono::milliseconds(100);
    td_config.measurement_delta = std::chrono::nanoseconds(33333333);

    pattern_graph_ptr->timedomain_configs[0] = td_config;



    std::string filename = pattern_graph_ptr->name + ".json";
    {
        nlohmann::json jsongraph;
        ns::to_json(jsongraph, *pattern_graph_ptr);

        std::ofstream myfile;
        myfile.open(filename);
        myfile << jsongraph.dump(4);
        myfile.close();

        std::cout << jsongraph.dump(4) << std::endl;
    }

    myfacade.loadDataflow(filename);




    {
        MEASURE_TIME(monitor, 0, "network start")
        myfacade.start();
    }

    {
        MEASURE_TIME(monitor, 1, "wait for finish source")
        while(running){
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

    }

    {
        MEASURE_TIME(monitor, 2, "finish and wait for network Stop")
        myfacade.stop();
    }


    spdlog::info(monitor.toString());

    spdlog::info("exit program");

    spdlog::info("run the same dataflow again using: traactConsole {0}", filename);

    return 0;
}
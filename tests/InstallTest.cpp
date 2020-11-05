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
#include <traact/facade/Facade.h>
#include <traact/util/PerformanceMonitor.h>
#include <traact/component/spatial/util/Pose6DTestSource.h>
//#include <traact/component/spa
//#include "../../spatial_module/src/traact/component/spatial/util/Pose6DTestSource.h"

#include <fstream>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <traact/serialization/JsonGraphInstance.h>
#include <traact/component/facade/ApplicationSyncSink.h>

void callback_func(traact::TimestampType ts, const Eigen::Affine3d & pose) {
    spdlog::info("callback_func");
}

int main(int argc, char **argv) {

  using namespace traact::facade;
  using namespace traact;
  using namespace traact::dataflow;
  //
  try {
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::trace);
    console_sink->set_pattern("[%^%l%$] %v");

  }
  catch (const spdlog::spdlog_ex &ex) {
    std::cout << "Log initialization failed: " << ex.what() << std::endl;
  }

  Facade myfacade;

  util::PerformanceMonitor monitor("tests",3);

  /*
  {
    DefaultInstanceGraphPtr pattern_graph_ptr = std::make_shared<DefaultInstanceGraph>("kinect_playback");

    // create instances of patterns
    DefaultPatternInstancePtr
        source_pattern = pattern_graph_ptr->addPattern("source", myfacade.instantiatePattern("KinectAzurePlayer"));
    DefaultPatternInstancePtr
        sink_pattern = pattern_graph_ptr->addPattern("sink", myfacade.instantiatePattern("OpenCvWindow"));

    // connect
    pattern_graph_ptr->connect("source", "output", "sink", "input");

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
  }*/

  // create dataflow configuraiton
  DefaultInstanceGraphPtr pattern_graph_ptr = std::make_shared<DefaultInstanceGraph>("test1");

  // create instances of patterns
  DefaultPatternInstancePtr
      source_pattern = pattern_graph_ptr->addPattern("source", myfacade.instantiatePattern("Pose6DTestSource"));
  DefaultPatternInstancePtr
      source2_pattern = pattern_graph_ptr->addPattern("source2", myfacade.instantiatePattern("Pose6DTestSource"));
  DefaultPatternInstancePtr
      sink_pattern = pattern_graph_ptr->addPattern("sink", myfacade.instantiatePattern("Pose6DPrint"));
  DefaultPatternInstancePtr
      mul_pattern = pattern_graph_ptr->addPattern("mul1", myfacade.instantiatePattern("MultiplicationComponent"));

    DefaultPatternInstancePtr
            sink_pattern_2 =
            pattern_graph_ptr->addPattern("sink2", myfacade.instantiatePattern("ApplicationSyncSink_Eigen::Affine3d"));

  // connect
  pattern_graph_ptr->connect("source", "output", "mul1", "input0");
  pattern_graph_ptr->connect("source2", "output", "mul1", "input1");
  pattern_graph_ptr->connect("mul1", "output", "sink", "input");
    pattern_graph_ptr->connect("mul1", "output", "sink2", "input");


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

  DefaultComponentPtr source = myfacade.getComponent("source");
  DefaultComponentPtr source2 = myfacade.getComponent("source2");

    DefaultComponentPtr sink = myfacade.getComponent("sink2");
    auto sink_ = std::dynamic_pointer_cast<traact::component::facade::ApplicationSyncSink<spatial::Pose6DHeader> >(sink);
    sink_->SetCallback(callback_func);


  {
    MEASURE_TIME(monitor, 0, "network start")
    myfacade.start();
  }

  {
    MEASURE_TIME(monitor, 1, "wait for finish source")

    std::shared_ptr<component::spatial::util::Pose6DTestSource>
        sourceTmp = std::dynamic_pointer_cast<component::spatial::util::Pose6DTestSource>(source);
    std::shared_ptr<component::spatial::util::Pose6DTestSource>
        source2Tmp = std::dynamic_pointer_cast<component::spatial::util::Pose6DTestSource>(source2);
    sourceTmp->waitForFinish();
    source2Tmp->waitForFinish();
  }

  {
    MEASURE_TIME(monitor, 2, "finish and wait for network stop")
    myfacade.stop();
  }

  spdlog::info(monitor.toString());

  spdlog::info("exit program");

  spdlog::info("run the same dataflow again using: traactConsole {0}", filename);

  return 0;
}
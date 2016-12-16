#ifdef DOXYGEN_SHOULD_SKIP_THIS
/**
   Main method for running the graspit planner on a given graspit world (or robot and object)
   Copyright (C) 2016 Jennifer Buehler
   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software Foundation,
   Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
*/
#endif  // DOXYGEN_SHOULD_SKIP_THIS

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <Eigen/Geometry>

#include <grasp_planning_graspit/GraspItSceneManagerHeadless.h>
#include <grasp_planning_graspit/LogBinding.h>
#include <grasp_planning_graspit/ContactGetter.h>

#include <string>
#include <stdio.h>
#include <execinfo.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <typeinfo>

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/parsers.hpp>


\

/**
 * Helper method to print the trace in case of a SIG event
 */
void print_trace(void)
{
    void *array[10];
    size_t size;
    char **strings;
    size_t i;

    size = backtrace(array, 10);
    strings = backtrace_symbols(array, size);

    printf("Obtained %zd stack frames.\n", size);

    for (i = 0; i < size; i++)
        printf("%s\n", strings[i]);

    free(strings);
}


void handler(int sig)
{
    print_trace();
    exit(1);
}

std::vector<double> quickGrasp(
    std::string& objectFilename, 
    std::string& robotFilename,
    Eigen::Vector3d& robPos, 
    const std::vector<double>& robRot)
{
    signal(SIGSEGV, handler);
    signal(SIGABRT, handler);
    PRINT_INIT_STD();

    PRINTMSG("Initializing GraspIt")
    std::string name = "ContactGetter"; 
    SHARED_PTR<GraspIt::GraspItSceneManager> graspitMgr(new GraspIt::GraspItSceneManagerHeadless());  
    SHARED_PTR<GraspIt::ContactGetter> cg(new GraspIt::ContactGetter(name, graspitMgr));
    
    GraspIt::EigenTransform robotTransform;
    GraspIt::EigenTransform objectTransform;
    // We want to keep the object at the absolute origin
    robotTransform.setIdentity();
    objectTransform.setIdentity();
    robotTransform.translate(robPos);
    // Have to do this because Eigen::Quaternion isn"t covered by pybind/eigen
      // Eigen::Quaterniond q(2, 0, 1, -3); 
    Eigen::Quaterniond robRotQ(robRot[0], robRot[1], robRot[2], robRot[3]);
    robotTransform.rotate(robRotQ);
    // robotTransform.rotate(robRot);
    
    std::string robotName="Robot1";
    std::string objectName="Object1";

    if (graspitMgr->loadRobot(robotFilename, robotName, robotTransform) != 0)
    {
        PRINTERROR("Could not load robot");
        
    }

    if (graspitMgr->loadObject(objectFilename, objectName, true, objectTransform))
    {
        PRINTERROR("Could not load object");
    }
    
    std::vector<double> dofs;// = cg->autoGrasp();
    // cg.reset();
    // graspitMgr->destroyCore();
    
    return dofs;
}

boost::program_options::options_description getOptions()
{
    // Declare the supported options.
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
    ("help", "produce help message")
    ("rob", boost::program_options::value<std::string>(), "filename for the robot file")
    ("obj", boost::program_options::value<std::string>(), "filename for the object file")
    ("rob_pos", boost::program_options::value<std::vector<float> >()->multitoken(), "Position of the object relative to the robot: Specify one x, y and z value.")
    ("rob_rot", boost::program_options::value<std::vector<float> >()->multitoken(), "A 4d quaternion to specify robot rotation.");
    return desc;
}

boost::program_options::variables_map loadParams(int argc, char ** argv)
{
    boost::program_options::options_description optDesc = getOptions();
    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::command_line_parser(argc, argv).options(optDesc).style(
        boost::program_options::command_line_style::unix_style ^ boost::program_options::command_line_style::allow_short).run(), vm);
    boost::program_options::notify(vm);
    return vm;
}

bool loadParams(int argc, char ** argv, 
  std::string& objectFilename, 
  std::string& robotFilename,
  Eigen::Vector3d robPos,
  std::vector<double> robRot)
{
  objectFilename.clear();
  robotFilename.clear();

  boost::program_options::variables_map vm;
  try
  {
    vm = loadParams(argc, argv);
  }
  catch (std::exception const& e)
  {
      PRINTERROR("Exception caught: " << e.what());
      return false;
  }
  catch (...)
  {
      PRINTERROR("Exception caught");
      return false;
  }
  boost::program_options::options_description desc = getOptions();
  if (vm.count("help"))
  {
      PRINTMSG(desc);
      return false;
  }
  // Validate user input
  if (!vm.count("obj"))
  {
    PRINTERROR("Must specify object file");
    PRINTMSG(desc);
    return false;
  } 
  if (!vm.count("rob"))
  {
    PRINTERROR("Must specify robot file");
    PRINTMSG(desc);
    return false;
  }
  if (!vm.count("rob_pos"))
  {
    PRINTERROR("Must specify robot position");
    PRINTMSG(desc);
    return false;
  } 
  if (!vm.count("rob_rot"))
  {
    PRINTERROR("Must specify robot rotation");
    PRINTMSG(desc);
    return false;
  }
  if (vm.count("rob") > 1)
  {
      PRINTERROR("You can only specify one robot at this stage.");
      PRINTMSG(desc);
      return false;
  }

  if (vm.count("obj") > 1)
  {
      PRINTERROR("You can only specify one object at this stage.");
      PRINTMSG(desc);
      return false;
  }
  // Assign valid user input
  
  if (vm.count("obj"))
  {
      objectFilename = vm["obj"].as<std::string>();
      PRINTMSG("Object file is " << objectFilename);
  }
  if (vm.count("rob"))
  {
      robotFilename = vm["rob"].as<std::string>();
      PRINTMSG("Robot file is " << robotFilename);
  }
  if (vm.count("rob_pos"))
  {
    std::vector<float> vals=vm["rob_pos"].as<std::vector<float> >();
    if (vals.size()!=3)
    {
        PRINTERROR("Must specify 3 values for --rob_pos: x, y and z (specified "<<vals.size()<<")");
        PRINTMSG(desc);
    }
    PRINTMSG("Using initial object pose "<<vals[0]<<", "<<vals[1]<<", "<<vals[2]);
    robPos=Eigen::Vector3d(vals[0],vals[1],vals[2]);
  }
  if (vm.count("rob_rot"))
  {
    std::vector<float> vals=vm["rob_rot"].as<std::vector<float> >();
    if (vals.size() != 4)
    {
      PRINTERROR("Must specify 4 values for --rob_rot: w, x, y, z (specified "<<vals.size()<<")");
      PRINTMSG(desc);
    }
    rob_rot = vals;
  }
  return true;
}

int main(int argc, char **argv)
{
  signal(SIGSEGV, handler);
  signal(SIGABRT, handler);

  PRINT_INIT_STD();

  std::string objectFilename;
  std::string robotFilename;
  Eigen::Vector3d robPos;
  std::vector<double> robRot;

  if (!loadParams(argc, argv, objectFilename, robotFilename, robPos, robRot))
  {
    PRINTERROR("Could not read arguments");
    return 1;
  }

  PRINTMSG("Performing quickGrasp...");
  std::vector<double> dofs = quickGrasp(objectFilename, robotFilename, robPos, robRot);
  PRINTMSG("The resulting dofs are");
  cout << dofs;
  return 0;

}

namespace py = pybind11;
PYBIND11_DECLARE_HOLDER_TYPE(T, SHARED_PTR<T>);
PYBIND11_PLUGIN(autograsp_planning) {
    py::module m("autograsp_planning", "Graspit!-quickGrasp plugin");

    m.def("quickGrasp", &quickGrasp, "Performs Graspit!-autograsp");
    m.def("main", &main, "Main call to quickGrasp");
    return m.ptr();
}



// /*
//  * This is a macro Boost.Python provides to signify a Python extension module.
//  */
// BOOST_PYTHON_MODULE(autograsp_planning) {
//     // An established convention for using boost.python.
//     using namespace boost::python;

//     // Expose the function.
//     def("quickGrasp", quickGrasp);
// }

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
// #include <grasp_planning_graspit/LogBinding.h>
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
    SHARED_PTR<GraspIt::GraspItSceneManager> graspitMgr(new GraspIt::GraspItSceneManagerHeadless());  
    SHARED_PTR<GraspIt::ContactGetter> cg(new GraspIt::ContactGetter("ContactGetter", graspitMgr));
    GraspIt::EigenTransform robotTransform;
    GraspIt::EigenTransform objectTransform;
    // We want to keep the object at the absolute origin
    robotTransform.setIdentity();
    objectTransform.setIdentity();
    robotTransform.translate(robPos);
    // Have to do this because Eigen::Quaternion isn't covered by pybind/eigen
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
    
    std::vector<double> dofs = cg->autoGrasp();
    graspitMgr.reset();
    cg.reset();
    return dofs;
}

namespace py = pybind11;
PYBIND11_DECLARE_HOLDER_TYPE(T, SHARED_PTR<T>);
PYBIND11_PLUGIN(autograsp_planning) {
    py::module m("autograsp_planning", "Graspit!-quickGrasp plugin");

    m.def("quickGrasp", &quickGrasp, "Performs Graspit!-autograsp");

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

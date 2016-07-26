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

#include <grasp_planning_graspit/GraspItSceneManagerHeadless.h>
#include <grasp_planning_graspit/LogBinding.h>
#include <grasp_planning_graspit/EigenGraspPlanner.h>
#include <grasp_planning_graspit/EigenGraspResult.h>
#include <grasp_planning_graspit/ContactGetter.h>

#include <string>
#include <set>
#include <stdio.h>
#include <execinfo.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <contact.h>

#include <EGPlanner/search.h>

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/parsers.hpp>


// if this is defined, the EigenGraspPlannerNoQt implementation
// is used. If not defined, EigenGraspPlannern implementation is used.
// #define USE_EIGENGRASP_NOQT

#ifdef USE_EIGENGRASP_NOQT
// This needs to be changed to remove graspit include for StateType
// #include <grasp_planning_graspit/EigenGraspPlannerNoQt.h>
#endif  // USE_EIGENGRASP_NOQT


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


boost::program_options::options_description getOptions()
{
    // Declare the supported options.
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
    ("help", "produce help message")
    ("dir", boost::program_options::value<std::string>(), "set output directory for resulting files")
    ("wld", boost::program_options::value<std::string>(), "filename for the world file")
    ("rob", boost::program_options::value<std::string>(), "filename for the robot file -- ALTERNATIVE to parameter wld!")
    ("obj", boost::program_options::value<std::string>(), "filename for the object file -- ALTERNATIVE to parameter wld!")
    ("iter", boost::program_options::value<int>(), "Maximum number of iterations for the planning algorithm")
    ("obj-pos", boost::program_options::value<std::vector<float> >()->multitoken(), "Position of the object relative to the robot: Specify one x, y and z value in quotes.")
    ("anneal", boost::program_options::value<std::vector<float> >()->multitoken(), "Annealing parameters to find suitable local minima. Specificy one for each of the following...")
    ("save-separate", "if this flag is set, robot and object files will be saved separately in addition to the normal result.")
    ("keep", boost::program_options::value<int>(), "Number of grasps stored, from best to worst.")
    ("save-prefix", boost::program_options::value<std::string>(), "Save prefix for output")
    ("autograsp", "Flag to finish grasp with autograsp");
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

bool loadParams(int argc, char ** argv, std::string& worldFilename, std::string& robotFilename,
                std::string& objectFilename, std::string& outputDirectory, bool& saveSeparate, Eigen::Vector3d& objPos, 
                std::vector<float> annealParams, AnnealingType t, int& maxIterations, int& keepMaxPlanningResults, 
                std::string& savePrefix, bool& autoGrasp)
{
    autoGrasp = false;
    saveSeparate = false;
    worldFilename.clear();
    robotFilename.clear();
    objectFilename.clear();
    outputDirectory.clear();

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
    // desc=getOptions();

    if (vm.count("help"))
    {
        PRINTMSG(desc);
        return false;
    }

    if (vm.count("dir") < 1)
    {
        PRINTERROR("Must specify an output directory");
        PRINTMSG(desc);
        return false;
    }

    if (vm.count("wld") && (vm.count("rob") || vm.count("obj")))
    {
        PRINTERROR("Cannot specify a world and a robot and/or object at the same time.");
        PRINTMSG(desc);
        return false;
    }

    if (!vm.count("wld") && !vm.count("rob"))
    {
        PRINTERROR("Have to specify either a robot or a world.");
        PRINTMSG(desc);
        return false;
    }

    if (vm.count("rob") != vm.count("obj"))
    {
        PRINTERROR("If you specify a robot, you also have to specify an object, and vice versa.");
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

    if (vm.count("obj") != vm.count("rob"))
    {
        PRINTERROR("If you specify a robot, you should also specify an object.");
        PRINTMSG(desc);
        return false;
    }

    if (vm.count("wld"))
    {
        worldFilename = vm["wld"].as<std::string>();
        PRINTMSG("World file is " << worldFilename);
    }
    if (vm.count("rob"))
    {
        robotFilename = vm["rob"].as<std::string>();
        PRINTMSG("Robot file is " << robotFilename);
    }
    if (vm.count("obj"))
    {
        objectFilename = vm["obj"].as<std::string>();
        PRINTMSG("Object file is " << objectFilename);
    }
    if (vm.count("dir"))
    {
        outputDirectory = vm["dir"].as<std::string>();
        PRINTMSG("Output dir is " << outputDirectory);
    }

    if (vm.count("iter"))
    {
        maxIterations = vm["iter"].as<int>();
        PRINTMSG("Number of iterations: " << maxIterations);
        if (maxIterations < 35000)
        {
            PRINTWARN("Planning is not working well with max iterations < 35000");
        }
    }


    if (vm.count("obj-pos"))
    {
         std::vector<float> vals=vm["obj-pos"].as<std::vector<float> >();
         if (vals.size()!=3)
         {
             PRINTERROR("Must specify 3 values for --obj-pos: x, y and z (specified "<<vals.size()<<")");
             PRINTMSG(desc);
         }
        PRINTMSG("Using initial object pose "<<vals[0]<<", "<<vals[1]<<", "<<vals[2]);
        objPos=Eigen::Vector3d(vals[0],vals[1],vals[2]);
    }

    if (vm.count("anneal"))
    {
         std::vector<float> vals=vm["anneal"].as<std::vector<float> >();
         if (val.size() !=3)
         {
            PRINTERROR("Must specify 8 values for --aneal: yc, hc, ydims, hdims, nbr_adj, err_adj, def_t0, def_k0");
            PRINTMSG(desc);
         }
         annealParams = vals;
         t = ANNEAL_CUSTOM;
    }
   
    if (vm.count("save-separate"))
    {
        saveSeparate=true;
    }

    if (vm.count("keep"))
    {
        keepMaxPlanningResults = vm["keep"].as<int>();
        if (keepMaxPlanningResults < 0)
        {
            PRINTERROR("Must save at least 1 grasp");
            PRINTMSG(desc);
        }

        PRINTMSG("Number of grasps kept: " << keepMaxPlanningResults);
    }

    if (vm.count("save-prefix"))
    {
        savePrefix = vm["save-prefix"].as<std::string>();
    }
    
    if (vm.count("autograsp"))
    {
        autoGrasp = true;
    }
    return true;
}

std::string vecToStr(std::vector<double> v)
{
    std::stringstream ss;
    for(size_t i = 0; i < v.size(); ++i)
    {
      if(i != 0)
        ss << ",";
      ss << v[i];
    }
    std::string s = ss.str();
    return s;
}


int main(int argc, char **argv)
{
    signal(SIGSEGV, handler);
    signal(SIGABRT, handler);

    PRINT_INIT_STD();

    std::string worldFilename;
    std::string robotFilename;
    std::string objectFilename;
    std::string outputDirectory;
    bool saveSeparate;
    bool autoGrasp;
    Eigen::Vector3d objPos;
    int maxPlanningSteps = 50000;
    int keepMaxPlanningResults = 3;
    std::string savePrefix = "world";
    std::vector<float> annealParams;
    AnnealingType annealPreset = ANNEAL_DEFAULT;

    if (!loadParams(argc, argv, worldFilename, robotFilename, objectFilename, outputDirectory, saveSeparate, objPos,
        annealParams, annealPreset, maxPlanningSteps, keepMaxPlanningResults, savePrefix, autoGrasp))
    {
        PRINTERROR("Could not read arguments");
        return 1;
    }

    std::vector<float> *aP = &annealParams;

    PRINTMSG("Creating planner");

    std::string name = "EigenGraspPlanner1";  // TODO make parameter
    SHARED_PTR<GraspIt::GraspItSceneManager> graspitMgr(new GraspIt::GraspItSceneManagerHeadless());
    SHARED_PTR<GraspIt::ContactGetter> cg(new GraspIt::ContactGetter("cg", graspitMgr));

#ifdef USE_EIGENGRASP_NOQT
    SHARED_PTR<GraspIt::EigenGraspPlannerNoQt> p(new GraspIt::EigenGraspPlannerNoQt(name, graspitMgr));
#else
    SHARED_PTR<GraspIt::EigenGraspPlanner> p(new GraspIt::EigenGraspPlanner(name, graspitMgr));
#endif

    // TODO parameterize:
    // Names for robot and object if not loaded from a world file.
    // If loaded from a world file, will be overwritten.
    std::string useRobotName="Robot1";
    std::string useObjectName="Object1";

    if (!worldFilename.empty())
    {
        PRINTMSG("Loading world");
        graspitMgr->loadWorld(worldFilename);
        std::vector<std::string> robs = graspitMgr->getRobotNames();       
        std::vector<std::string> objs = graspitMgr->getObjectNames(true);
        if (robs.empty())
        {
            PRINTERROR("No robots loaded");
            return 1;
        }
        if (objs.empty())
        {
            PRINTERROR("No graspable objects loaded");
            return 1;
        }
        if (robs.size()!=1)
        {
            PRINTERROR("Exactly 1 robot should have been loaded");
            return 1;
        }
        if (objs.size()!=1)
        {
            PRINTERROR("Exactly 1 graspable object should have been loaded");
            return 1;
        }
        useRobotName=robs.front();
        useObjectName=objs.front();
        PRINTMSG("Using robot "<<useRobotName<<" and object "<<useObjectName);
    }
    else
    {
        // TODO add an option to set the transforms.
        // For now, they're put in the origin. For the planning, this should not really matter...
        GraspIt::EigenTransform robotTransform;
        GraspIt::EigenTransform objectTransform;
        robotTransform.setIdentity();
        objectTransform.setIdentity();
        objectTransform.translate(objPos);
        // objectTransform.translate(Eigen::Vector3d(100,0,0));
        std::string robotName(useRobotName); 
        std::string objectName(useObjectName);
        if ((graspitMgr->loadRobot(robotFilename, robotName, robotTransform) != 0) ||
                (graspitMgr->loadObject(objectFilename, objectName, true, objectTransform)))
        {
            PRINTERROR("Could not load robot or object");
            return 1;
        }
    }
    
    
    bool createDir = true;
    bool saveIV = true;
    bool forceWrite = createDir;  // only enforce if creating dir is also allowed
    
    // in case one wants to view the initial world before planning, save it:
    graspitMgr->saveGraspItWorld(outputDirectory + "/" + savePrefix + "/startWorld.xml", createDir);
    graspitMgr->saveInventorWorld(outputDirectory + "/" + savePrefix + "/startWorld.iv", createDir);

    if (saveSeparate)
    {
        graspitMgr->saveRobotAsInventor(outputDirectory + "/"+ savePrefix +"/robotStartpose.iv", useRobotName, createDir, forceWrite);
        graspitMgr->saveObjectAsInventor(outputDirectory + "/"+ savePrefix + "/object.iv", useObjectName, createDir, forceWrite);
    }

    int repeatPlanning = 1;
    
    p->plan(maxPlanningSteps, repeatPlanning, keepMaxPlanningResults, autoGrasp, annealParams, annealPreset);

    PRINTMSG("Saving results...");

    bool saveWorld = true;

    std::string resultsWorldDirectory = outputDirectory + "/" + savePrefix;
    //std::string filenamePrefix = "grasp";
    p->saveResultsAsWorldFiles(resultsWorldDirectory, "grasp", saveWorld, saveIV, createDir, saveSeparate);

    std::vector<GraspIt::EigenGraspResult> allGrasps;
    p->getResults(allGrasps);

    std::string robotName(useRobotName);
    std::string objectName(useObjectName);
    // SHARED_PTR<GraspIt::ContactGetter> cg(new GraspIt::ContactGetter("cg", graspitMgr));
    std::list< Contact * > bodyContacts = cg->getGraspContacts();
//    GraspItAccessor * gc = new GraspIt::GraspItAccessor("gc", graspitMgr) ;

    // = GraspItAccessor("gc", graspitMgr);
    //Body *graspedBody = gc->getBody(useObjectName);
    //std::list< Contact * > bodyContacts = getContacts(graspedBody);

    std::ofstream dFile((outputDirectory + "/" + savePrefix + "/data.txt").c_str());
    //dFile.open(outputDirectory + "/" + savePrefix + "/data.txt");


    PRINTMSG("Writing Grasp results...");
    std::vector<GraspIt::EigenGraspResult>::iterator it;
    for (it = allGrasps.begin(); it != allGrasps.end(); ++it)
    {
        if (dFile.is_open())
        {
            int graspNumber = it - allGrasps.begin();
            dFile << "Object\t" << objectName;
            dFile << "Grasp\t" << graspNumber << "\n";
            std::vector<double> dofs = it->getGraspJointDOFs();
            dFile << "DOFS\t" << vecToStr(dofs) <<"\n";
            dFile << "energy\t" << it->getEnergy() << "\n";
            dFile << "epsilon\t" << it->qualityEpsilon() << "\n";
            dFile << "hand_transform\t" << it->getObjectToHandTransform() << "\n";

            dFile << "contacts_start\n";
            dFile << "contacts_size\t" << bodyContacts.size() << "\n";

            std::list<Contact *>::iterator itr;
            for(itr=bodyContacts.begin();itr != bodyContacts.end();++itr)
                {
                   std::vector<double> positions = cg->getContactPos(*itr);
                   dFile << "positions\t" << vecToStr(positions) << "\n";
                   std::vector<double> normals = cg->getContactNorm(*itr);
                   dFile << "normals\t" << vecToStr(positions) <<"\n";
                }
            dFile << "contacts_end";
        }
        else PRINTMSG("Unable to open data file");
    }
    if (dFile.is_open()) 
    {  
    dFile.close(); 
    }


    PRINTMSG("Quitting program.");
    return 1;
}
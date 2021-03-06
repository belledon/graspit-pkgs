//ContactGetter.cpp

#include <grasp_planning_graspit/ContactGetter.h>
#include <grasp_planning_graspit/PrintHelpers.h>
#include <grasp_planning_graspit/GraspItHelpers.h>
#include <EGPlanner/searchState.h>

#include <Inventor/Qt/SoQt.h>
#include <Inventor/actions/SoWriteAction.h>
#include <Inventor/actions/SoGetBoundingBoxAction.h>
#include <Inventor/sensors/SoIdleSensor.h>

#include <boost/filesystem.hpp>

using GraspIt::ContactGetter;
using GraspIt::Log;

ContactGetter::ContactGetter(const std::string& name, SHARED_PTR<GraspItSceneManager>& intr):
    GraspItAccessor(name, intr)

#ifdef USE_SEPARATE_SOSENSOR
    mIdleSensor(NULL)
#endif
    
{
    
    // statusThread = new THREAD_CONSTR(statusThreadLoop, this);
    if (!eventThreadRunsQt())
    {
        PRINTERROR("EigenGraspPlanner supports only GraspItSceneManager instances which run Qt.");
        throw std::string("EigenGraspPlanner supports only GraspItSceneManager instances which run Qt.");
    }

    addAsIdleListener();
}

ContactGetter::~ContactGetter()
{
    PRINTMSG("ContactGetter destructor");
    PRINTMSG("Removing idle listeners");
    removeFromIdleListeners();

#ifdef USE_SEPARATE_SOSENSOR
    // quit the idle sensor to avoid conflicts when Inventor
    // is shut down
    if (mIdleSensor)
    {
        if (mIdleSensor->isScheduled())
        {
            PRINTMSG("Unscheduling sensor");
            mIdleSensor->unschedule();
        }
        PRINTMSG("Deleting sensor");
        delete mIdleSensor;
        mIdleSensor = NULL;
    }
#endif
    PRINTMSG("Exit ContactGetter destructor");
//     if (statusThread)
//     {
//         statusThread->detach();
//         delete statusThread;
//         statusThread = NULL;
//     }
}

void ContactGetter::idleEventFromSceneManager()
{
//     if (statusThread)
//     {
//         statusThread->detach();
//         delete statusThread;
//         statusThread = NULL;
//     }
    
// }
// {
#ifdef USE_SEPARATE_SOSENSOR
    if (mIdleSensor)
    {
        if (mIdleSensor->isScheduled()) mIdleSensor->unschedule();
        delete mIdleSensor;
    }

    // start the idle sensor which is triggered regularly from within the inventor thread
    mIdleSensor = new SoIdleSensor(sensorCB, this);
    mIdleSensor->schedule();
#else
    scheduleForIdleEventUpdate();
    PRINTMSG("Stuck right before ivIdleCallback")
    // ivIdleCallback();
#endif
}

void ContactGetter::onSceneManagerShutdown()
{
    //     if (statusThread)
    // {
    //     statusThread->detach();
    //     delete statusThread;
    //     statusThread = NULL;
    // }
    PRINTMSG("SceneManagerShutdown for ContactGetter")
#ifdef USE_SEPARATE_SOSENSOR
    // quit the idle sensor to avoid conflicts when Inventor
    // is shut down
    if (mIdleSensor)
    {
        if (mIdleSensor->isScheduled())
        {
            mIdleSensor->unschedule();
        }
        delete mIdleSensor;
        mIdleSensor = NULL;
    }
#endif
}


#ifdef USE_SEPARATE_SOSENSOR
void ContactGetter::sensorCB(void *data, SoSensor *)
{
    // PRINTMSG(" ### sensorCB ###");
    ContactGetter* _this = dynamic_cast<ContactGetter*>(static_cast<ContactGetter*>(data));
    if (!_this)
    {
        PRINTERROR("Could not cast ContactGetter");
        return;
    }

    // _this->ivIdleCallback();

    // trigger another call of this method in the next event loop iteration
    _this->mIdleSensor->schedule();
}
#endif

bool ContactGetter::autoOpen(){
    PRINTMSG("Getting hand");
    Hand *h = getGraspItSceneManager()->getCurrentHand();
    GraspPlanningState gst = GraspPlanningState(h);
    GraspPlanningState *s;
    s = &gst;
    return s->getHand()->quickOpen();
}

std::vector<float> ContactGetter::autoGrasp(){
    PRINTMSG("Getting hand");
    Hand *h = getGraspItSceneManager()->getCurrentHand();
    PRINTMSG("Performing autograsp");
    GraspPlanningState gst = GraspPlanningState(h);
    GraspPlanningState *s;
    s = &gst;
    if (!s->getHand()->autoGrasp(false))
    {
        PRINTWARN("Could not correctly open hand with auto-grasp, the pre-grasp state may not be ideal.");
    }
   
    s->saveCurrentHandState();
    
    // if (!getGraspItSceneManager()->saveRobotBox("", "Robot", true, false))
    // Robot * nh = s->getHand();
    std::vector<float> bbox = getGraspItSceneManager()->getRobotBox(s->getHand(), s->getIVRoot());
    // if (!getGraspItSceneManager()->saveRobotBox(s->getHand(), s->getIVRoot()))
    // {
    //   PRINTMSG("Did something with bounding box");
    // }

    // const PostureState* handPosture = s->readPosture();
    // if (!handPosture)
    // {
    //     PRINTERROR("Posture is NULL!");
    // }

    // // std::string robotName="Robot1";
    // // Robot *r = getGraspItSceneManager()->getRobot(robotName);
    // // h = getGraspItSceneManager()->getCurrentHand();
    // PRINTMSG("Getting hand dofs");
    // const int numDOF = s->getHand()->getNumDOF();
    // double * _dofs = new double[numDOF];
    // handPosture->getHandDOF(_dofs);
    // std::vector<double> dofs;
    // for (int k = 0; k < numDOF; ++k)
    // {
    //     dofs.push_back(_dofs[k]);
    // }
    // // r->getDOFVals(dofs);
    PRINTMSG("Obtained hand bounding box");
    PRINTMSG("Cleaning up");
    // delete s;
    return bbox;
}

std::list< Contact* > ContactGetter::getGraspContacts(const std::string& robName, const std::string& objName)
{

    Robot *r = getGraspRobot(robName);
    GraspableBody *b = getGraspBody(objName);
    std::list< Contact * > c = getContacts(r, b);
    int numContacts = c.size();
    if (numContacts > 0)
    {
        char buffer [50];
        sprintf (buffer, "Found %d contacts for grasp", 
            numContacts);
        PRINTMSG(buffer);
    }
    else 
    {
        PRINTMSG("Did not find contacts for grasp");
    }
    return c;
}

std::vector<double> ContactGetter::getContactPos(Contact * c)
{
	position p = getContactLoc(c);
	return getXYZ(p);	
}

std::vector<double> ContactGetter::getContactNorm(Contact * c)
{
	vec3 v3 = c->getNormal();
	return getXYZ(v3);
}
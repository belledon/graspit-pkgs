//ContactGetter.cpp

#include <grasp_planning_graspit/ContactGetter.h>
#include <grasp_planning_graspit/LogBinding.h>
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

ContactGetter::ContactGetter(const std::string& name, const SHARED_PTR<GraspItSceneManager>& intr):
    GraspItAccessor(name, intr)

#ifdef USE_SEPARATE_SOSENSOR
    mIdleSensor(NULL)
#endif
    // planCommand(NONE)
{
    // mEnergyCalculator=new SearchEnergy();
    // mEnergyCalculator->setStatStream(&std::cout);

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
    removeFromIdleListeners();

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
    PRINTMSG("Exit ContactGetter destructor");
}

void ContactGetter::idleEventFromSceneManager()
{
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
#endif
}

void ContactGetter::onSceneManagerShutdown()
{
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

std::vector<double> ContactGetter::autoGrasp(){
    PRINTMSG("Getting hand");
    Hand *h = getGraspItSceneManager()->getCurrentHand();
    PRINTMSG("Performing autograsp");
    GraspPlanningState gst = GraspPlanningState(h);
    const GraspPlanningState *s;
    s = &gst;
    if (!s->getHand()->autoGrasp(false))
    {
        PRINTWARN("Could not correctly open hand with auto-grasp, the pre-grasp state may not be ideal.");
    }
    
    const PostureState* handPosture = s->readPosture();
    if (!handPosture)
    {
        PRINTERROR("Posture is NULL!");
    }

    // std::string robotName="Robot1";
    // Robot *r = getGraspItSceneManager()->getRobot(robotName);
    // h = getGraspItSceneManager()->getCurrentHand();
    PRINTMSG("Getting hand dofs");
    const int numDOF = s->getHand()->getNumDOF();
    double * _dofs = new double[numDOF];
    handPosture->getHandDOF(_dofs);
    std::vector<double> dofs;
    for (int k = 0; k < numDOF; ++k)
    {
        dofs.push_back(_dofs[k]);
    }
    // r->getDOFVals(dofs);
    PRINTMSG("Obtained hand dofs");
    return dofs;
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
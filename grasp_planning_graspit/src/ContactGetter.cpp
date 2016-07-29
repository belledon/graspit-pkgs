//ContactGetter.cpp

#include <grasp_planning_graspit/ContactGetter.h>
#include <grasp_planning_graspit/LogBinding.h>
#include <grasp_planning_graspit/PrintHelpers.h>
#include <grasp_planning_graspit/GraspItHelpers.h>


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

// std::list< Contact* > ContactGetter::getGraspContacts()
// {
// 	Hand * h = getRobotHand();
// 	Body * b = getGraspBody();
// 	return h->getContacts(b);
// }
std::list< Contact* > ContactGetter::getGraspContacts()
{
    Hand *h = getRobotHand();
    Grasp *g = getHandGrasp(h);
    int numContacts = g->getNumContacts();
    
    std::list< Contact* > cs;

    for (int it; it < numContacts; it ++)
    {   
        Contact * c = g->getContact(it);
        cs.push_back(c);
    }
    return cs;
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
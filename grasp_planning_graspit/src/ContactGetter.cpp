//ContactGetter.cpp

#include <grasp_planning_graspit/ContactGetter.h>

#include <Inventor/Qt/SoQt.h>
#include <Inventor/sensors/SoIdleSensor.h>

using GraspIt::ContactGetter

ContactGetter::ContactGetter(const std::string& name, 
							 const SHARED_PTR<GraspItSceneManager>& interface):
    GraspItAccessor(name, intr),
#ifdef USE_SEPARATE_SOSENSOR
    mIdleSensor(NULL),
#endif
    {
   
    if (!eventThreadRunsQt())
    {
        PRINTERROR("ContactGetter supports only GraspItSceneManager instances which run Qt.");
        throw std::string("ContactGetter supports only GraspItSceneManager instances which run Qt.");
    }

    addAsIdleListener();
}

ContactGetter::~ContactGetter()
{
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
    ivIdleCallback();
#endif
}

void ContactGetter::OnSceneManagerShutdown()
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

std::list< Contact* > ContactGetter::getGraspContacts()
{
	Hand * h = getRobotHand();
	Body * b = getGraspBody();
	return h->getContacts(b);
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
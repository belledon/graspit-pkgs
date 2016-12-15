//ContactGetter.h
//=================================
// include guard
#ifndef GRASP_PLANNING_GRASPIT_CONTACTGETTER_H
#define GRASP_PLANNING_GRASPIT_CONTACTGETTER_H

//=================================
// forward declared dependencies

//=================================
// included dependencies

// grasp_planning_graspit
#include <grasp_planning_graspit/GraspItSceneManager.h>
#include <grasp_planning_graspit/GraspItAccessor.h>
#include <grasp_planning_graspit/LogBinding.h>

// graspit
#include <robot.h>
#include <grasp.h>
#include <contact/contact.h>
#include <matvec3D.h>

// other
#include <list>
#include <string>
#include <vector>

#include <QObject>

class GraspPlanningState;
class GraspableBody;
class GraspItAccessor;


namespace GraspIt
{


	class ContactGetter:  public GraspItAccessor
	{

	public:

		ContactGetter(const std::string& name, const SHARED_PTR<GraspItSceneManager>& interface);
		virtual ~ContactGetter();

		std::list< Contact* > getGraspContacts(const std::string& robName, const std::string& objName);
		// std::list< Contact* > getGraspContacts();

		std::vector<double> getContactPos(Contact * c);
		std::vector<double> getContactNorm(Contact * c);

		std::vector<double> autoGrasp();

	protected:


    virtual void idleEventFromSceneManager();

    virtual void onSceneManagerShutdown();


	private:

		

		Robot * getGraspRobot(const std::string& robName)
		{
			if (tryLockWorld())
			{
				Robot *r = getRobot(robName);
				unlockWorld();
				PRINTMSG("The robot: " << r->getName().toLocal8Bit().constData()  << " was found");
				if (!r)
				{
					PRINTERROR("Robot" << robName << " could not be found!");
				}
				return r;
			}
			else{PRINTERROR("Could not lock the world to retrieve the robot");}
			
		}

		GraspableBody * getGraspBody(const std::string& objName)
		{
			if (tryLockWorld())
			{
				GraspableBody *b = getGraspableBody(objName);
				unlockWorld();
				PRINTMSG("The body: " << b->getName().toLocal8Bit().constData()  << " was found");
				if (!b)
				{
					PRINTERROR("Body" << objName << " could not be found!");
				}
				return b;
			}
			else{PRINTERROR("Could not lock the world to retrieve the body");}
		}


		Hand * getCurrentHand()
		{
			if (tryLockWorld())
			{
				Hand *h = getCurrentHand();
				unlockWorld();
				
				if (!h)
				{
					PRINTERROR("Current hand could not be found!");
				}
				return h;
			}
			else{PRINTERROR("Could not lock the world to retrieve the current hand");}
			
		}
		

		std::list< Contact * > getContacts(Robot* r, GraspableBody * b)
		{
			if (tryLockWorld())
			{
				std::list< Contact * > c = r->getContacts(b);
				unlockWorld();
				if (c.empty())
				{
					PRINTMSG("No contacts returned for robot " << r->getName().toLocal8Bit().constData()
					<< " and body " << b->getName().toLocal8Bit().constData())
				}
				return c;
			}
			else{PRINTERROR("Could not lock world during contact retrieval")}
		}


		position getContactLoc(Contact * c)
		{
			return c->getPosition();
		}

		std::vector<double> getXYZ(position p)
		{
			std::vector<double> v;
			v.push_back(p.x());
			v.push_back(p.y());
			v.push_back(p.z());
			return v;
		}

		std::vector<double> getXYZ(vec3 v3)
		{
			std::vector<double> v;
			v.push_back(v3.x());
			v.push_back(v3.y());
			v.push_back(v3.y());
		}

#ifdef USE_SEPARATE_SOSENSOR
    SoSensor *mIdleSensor;
#endif
};
} // namespace GraspIt
#endif // GRASP_PLANNING_GRASPIT_CONTACTGETTER_H_INCLUDED__
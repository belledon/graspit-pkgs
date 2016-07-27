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

// graspit
#include <robot.h>
#include <grasp.h>
#include <contact.h>
#include <matvec3D.h>

// other
#include <list>
#include <string>
#include <vector>


class GraspItAccessor;

namespace GraspIt
{


	class ContactGetter: public GraspItAccessor
	{
	public:

		ContactGetter(const std::string& name, const SHARED_PTR<GraspItSceneManager>& interface);
		virtual ~ContactGetter();

		// std::list< Contact* > getGraspContacts(const std::string& robName, const std::string& objName);
		std::list< Contact* > getGraspContacts();

		std::vector<double> getContactPos(Contact * c);
		std::vector<double> getContactNorm(Contact * c);

	protected:

    virtual void idleEventFromSceneManager();

    virtual void onSceneManagerShutdown();


	private:

		Hand * getRobotHand()
		{
			Hand * h = getCurrentHand();
			return h;
		}
		// Robot * getRobotHand(const std::string& name)
		// {
		// 	Robot * r = getRobot(name);
		// 	return r;
		// }

		Body * getGraspBody()
		{
			Body * b = getCurrentGraspableBody();
			return b;
		}
		Grasp * getHandGrasp(Hand * h)
		{
			Grasp * g = h->getGrasp();
			return g;
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

};
} // namespace GraspIt
#endif // GRASP_PLANNING_GRASPIT_CONTACTGETTER_H_INCLUDED__
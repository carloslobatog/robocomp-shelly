/*
 * Copyright 2017 pbustos <email>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "pathfinder.h"

<<<<<<< HEAD
using namespace Path;

PathFinder::PathFinder()
{}

PathFinder::~PathFinder()
{}
=======
using namespace robocomp::pathfinder;
>>>>>>> 4a123defec4e0344e337d4a02147d467ef77a033

///////////////////////////////////////////////////////////////////
///  Public Interface
//////////////////////////////////////////////////////////////////

void PathFinder::go(float x, float z, const ParameterMap &parameters)
{
	qDebug() << "GO...................";
};

///////////////////////////////////////////////////////////////////


void PathFinder::initialize( InnerModel *innerModel, const RoboCompAGMCommonBehavior::ParameterMap &params, const 		RoboCompCommonBehavior::ParameterList &localparams)
{
<<<<<<< HEAD
//////////////////////////////////////
	/// Initialize sampler of free space
	/////////////////////////////////////
	sampler.initialize(innerModel, localparams);

	///////////////////
	/// Planner
	///////////////////
	//plannerPRM.initialize(&sampler, params);  
	
	///////////////////
	/// Initializes the elastic band structure "road"
	///////////////////
	road.initialize(innerModel, localparams);

	///////////////////
	/// This object creates and maintains the road (elastic band) adapting it to the real world using a laser device
	///////////////////
	//elasticband.initialize( params);

	//////////////////////////////////////////////////////////////////////////
	/// Low level controller that drives the robot on the road
	/// by computing VAdv and VRot from the relative position wrt to the local road
	/////////////////////////////////////////////////////////////////////////////////
	//controller = new Controller(innerModel, laserData, params, 2 );

	qDebug() << __FUNCTION__ << "All objects initialized";
	
}

=======
	/// Initialize the elastic road
	road.initialize(innerModel, localparams);

	/// Initialize sampler of free space
	try
	{ sampler.initialize(innerModel, localparams); } catch(const std::exception &ex) { throw ex;	}

	/// Initialize the Planner
	//plannerPRM.initialize(&sampler, params);  
	pathplanner.initialize();
	
	/// Initialize the Projector
	//elasticband.initialize( params);

	/// Initialize the low level controller that drives the robot on the road	
	//controller = new Controller(innerModel, laserData, params, 2 );

	std::cout << __FUNCTION__ << "All objects initialized" << std::endl;
	
	//std::thread thread_planner(&PathPlanner::run, &pathplanner, std::bind(&PathFinder::getRoad, this), std::bind(&PathFinder::releaseRoad, this));
	//thread_planner.join();
}

Road& PathFinder::getRoad()
{
	mymutex.lock();
	std::cout << "mutex locked" << std::endl;
	return road;	
}

void PathFinder::releaseRoad()
{
	mymutex.unlock();
	std::cout << "mutex released" << std::endl;
}


//////////////////////////////////////////////////


>>>>>>> 4a123defec4e0344e337d4a02147d467ef77a033




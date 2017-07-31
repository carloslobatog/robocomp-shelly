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

using namespace robocomp::pathfinder;

PathFinder::PathFinder()
{}

PathFinder::~PathFinder()
{}

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
//////////////////////////////////////
	/// Initialize sampler of free space
	/////////////////////////////////////
	try
	{ sampler.initialize(innerModel, localparams); } catch(const std::exception &ex) { throw ex;	}

	////////////////////////////////////
	/// Initialize the Planner
	////////////////////////////////////
	//plannerPRM.initialize(&sampler, params);  
	
	////////////////////////////////////
	/// Initialize the elastic road
	////////////////////////////////////
	road.initialize(innerModel, localparams);

	/////////////////////////////////////
	/// Initialize the Projector
	////////////////////////////////////
	//elasticband.initialize( params);

	//////////////////////////////////////////////////////////////////////////
	/// Initialize the low level controller that drives the robot on the road
	/// by computing VAdv and VRot from the relative position wrt to the local road
	/////////////////////////////////////////////////////////////////////////////////
	//controller = new Controller(innerModel, laserData, params, 2 );

	std::cout << __FUNCTION__ << "All objects initialized" << std::endl;
	
}





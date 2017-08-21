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

#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <AGMCommonBehavior.h>
#include <CommonBehavior.h>
#include <thread>
<<<<<<< HEAD
#include "currenttarget.h"
#include "road.h"
#include "sampler.h"

using namespace std;

namespace Path
{
	typedef map<string, string> ParameterMap;		
	
	class PathFinder
	{
		public:
			PathFinder();
			~PathFinder();
			void initialize(InnerModel *innerModel, const RoboCompAGMCommonBehavior::ParameterMap &params, const RoboCompCommonBehavior::ParameterList &localparams);

	  /////////////////////////////
		/// Interface
		////////////////////////////
		void go(float x, float z, const ParameterMap &parameters = ParameterMap());
		
		///////////////////////////
		
		private:
	
			Road road;
			Sampler sampler;
			
			
	};

	
		
		
} //namespace
=======
#include <mutex>
#include <functional>
#include "currenttarget.h"
#include "road.h"
#include "sampler.h"
#include "pathplanner.h"

using namespace std;

namespace robocomp
{
	namespace pathfinder
	{
		typedef map<string, string> ParameterMap;		
		
		class PathFinder
		{
			public:
				PathFinder() = default;
				virtual ~PathFinder(){};
				void initialize(InnerModel *innerModel, const RoboCompAGMCommonBehavior::ParameterMap &params, const RoboCompCommonBehavior::ParameterList &localparams);
				void releaseRoad();
				Road& getRoad();
				
			/////////////////////////////
			/// Interface
			////////////////////////////
			void go(float x, float z, const ParameterMap &parameters = ParameterMap());
			
			///////////////////////////
			
			private:
		
				mutable std::mutex mymutex;
				Road road;
				Sampler sampler;
				PathPlanner pathplanner;
				
		};	
	} //path
} //robocomp
>>>>>>> 4a123defec4e0344e337d4a02147d467ef77a033

#endif // PATHFINDER_H

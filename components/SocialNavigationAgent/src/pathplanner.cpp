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

#include "pathplanner.h"
using namespace std::chrono_literals;

PathPlanner::PathPlanner()
{

}

PathPlanner::~PathPlanner()
{

}

void PathPlanner::initialize()
{

}

void PathPlanner::run(std::function<Road&()> getRoad, std::function<void()> releaseRoad)
{
	std::cout << "PathPlanner running" << std::endl;
	while(true)
	{
		Road &road = getRoad();
		std::cout << "Working with the road" << std::endl;
		std::this_thread::sleep_for(1s);
		releaseRoad();
	}
}

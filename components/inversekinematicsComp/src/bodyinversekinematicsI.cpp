/*
 *    Copyright (C) 2006-2010 by RoboLab - University of Extremadura
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "bodyinversekinematicsI.h"

BodyInverseKinematicsI::BodyInverseKinematicsI(GenericWorker *_worker, QObject *parent) : QObject(parent)
{
	worker = _worker;
	mutex = worker->mutex;       // Shared worker mutex
	// Component initialization...
}


BodyInverseKinematicsI::~BodyInverseKinematicsI()
{
	// Free component resources here
}

// Component functions, implementation
bool BodyInverseKinematicsI::setTarget(const string& bodyPart, const Pose6D& target, const Ice::Current&){
	return worker->setTarget(bodyPart,target);
}

void BodyInverseKinematicsI::setTargetPose6D(const string& bodyPart, const Pose6D& target, const WeightVector& weights, const Ice::Current&){
	worker->setTargetPose6D(bodyPart,target,weights);
}

void BodyInverseKinematicsI::pointAxisTowardsTarget(const string& bodyPart, const Pose6D& target, const string& axis, bool axisConstraint, float axisAngleConstraint, const Ice::Current&){
	worker->pointAxisTowardsTarget(bodyPart,target,axis,axisConstraint,axisAngleConstraint);
}

void BodyInverseKinematicsI::advanceAlongAxis(const string& bodyPart, const Axis& ax, float dist, const Ice::Current&){
	worker->advanceAlongAxis(bodyPart,ax,dist);
}

void BodyInverseKinematicsI::setFingers(float d, const Ice::Current&){
	worker->setFingers(d);
}



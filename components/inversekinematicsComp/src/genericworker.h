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
#ifndef GENERICWORKER_H
#define GENERICWORKER_H

// #include <ipp.h>
#include <QtGui>
#include <stdint.h>
#include <qlog/qlog.h>
#include <CommonBehavior.h>
#include <InnerModelManager.h>
#include <JointMotor.h>
#include <BodyInverseKinematics.h>

#define CHECK_PERIOD 5000
#define BASIC_PERIOD 100

typedef map <string,::IceProxy::Ice::Object*> MapPrx;

using namespace std;

/**
       \brief
       @author authorname
*/
using namespace RoboCompInnerModelManager;
using namespace RoboCompJointMotor;
using namespace RoboCompBodyInverseKinematics;

class GenericWorker : public QObject
{
Q_OBJECT
public:
	GenericWorker(MapPrx& mprx, QObject *parent = 0);
	virtual ~GenericWorker();
	virtual void killYourSelf();
	virtual void setPeriod(int p);
	
	virtual bool setParams(RoboCompCommonBehavior::ParameterList params) = 0;
	QMutex *mutex;                //Shared mutex with servant

	InnerModelManagerPrx innermodelmanager_proxy;
	JointMotorPrx jointmotor_proxy;
	virtual bool setTarget(const string& bodyPart, const Pose6D& target) = 0;
	virtual void  setTargetPose6D(const string& bodyPart, const Pose6D& target, const WeightVector& weights) = 0;
	virtual void  pointAxisTowardsTarget(const string& bodyPart, const Pose6D& target, const string& axis, bool axisConstraint, float axisAngleConstraint) = 0;
	virtual void  advanceAlongAxis(const string& bodyPart, const Axis& ax, float dist) = 0;
	virtual void  setFingers(float d) = 0;
protected:
	QTimer timer;
	int Period;
public slots:
	virtual void compute() = 0;
signals:
	void kill();
};

#endif
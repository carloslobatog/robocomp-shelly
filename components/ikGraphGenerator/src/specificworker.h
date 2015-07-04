/*
 *    Copyright (C) 2015 by YOUR NAME HERE
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

/**
       \brief
       @author authorname
*/

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>

#ifdef USE_QTGUI
	#include <osgviewer/osgview.h>
	#include <innermodel/innermodelviewer.h>
	#include <innermodeldraw.h>
#endif

#include <nabo/nabo.h>
#include <innermodeldraw.h>

#define MAX_ERROR_IK 5.

#include <djk.h>
#include <graph.h>

using namespace boost;


#define MIN(X,Y) (((X) < (Y)) ? (X) : (Y))
#define MAX(X,Y) (((X) > (Y)) ? (X) : (Y))



class WorkerThread : public QThread
{
Q_OBJECT
public:
	WorkerThread(void *data_)
	{
		data = data_;
	}

	void *data;

	void run();
};

class SpecificWorker : public GenericWorker
{
Q_OBJECT

public:
	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

public slots:
	void initFile();
	void initGenerate();
	void computeHard();
	void compute();

	void goIK();
	void goVIK();

private:
	void updateFrame(uint wait_usecs=0);
	bool goAndWait(int nodeId, MotorGoalPositionList &mpl, int &recursive);
	bool goAndWait(float x, float y, float z, int node, MotorGoalPositionList &mpl, int &recursive);
	void goAndWaitDirect(const MotorGoalPositionList &mpl);
	void updateInnerModel();


	std::pair<float, float> xrange, yrange, zrange;

	MotorGoalPositionList centerConfiguration;

	ConnectivityGraph *graph;
	WorkerThread *workerThread;

#ifdef USE_QTGUI
	OsgView *osgView;
	InnerModelViewer *innerViewer;
	InnerModel *innerVisual;
	InnerModel *innerModel;
#endif

	bool hasTarget;
	Pose6D target;
	WeightVector weights;



};

#endif


/*
 *    Copyright (C)2018 by YOUR NAME HERE
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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//       THE FOLLOWING IS JUST AN EXAMPLE

	try
	{
// 		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
// 		innermodel_path = par.value;
		
// 		innerModel = new InnerModel("/home/robocomp/robocomp/components/robocomp-araceli/etcSim/simulation.xml");
		innerModel = InnerModelMgr(std::make_shared<InnerModel>("/home/robocomp/robocomp/components/robocomp-araceli/etcSim/simulation.xml"));
	}
	catch(std::exception e) { qFatal("Error reading config params"); }

		// Fold arm

	innerModel->getNode<InnerModelJoint>("armX1")->setAngle(-1);
	innerModel->getNode<InnerModelJoint>("armX2")->setAngle(2.5);

	qDebug();
	qDebug() << __FILE__ << __FUNCTION__ << "Now creating viewer";
	
	
	viewer = new InnerViewer(innerModel, "Social Navigation");  //InnerViewer copies internally innerModel so it has to be resynchronized
	viewer->start();	

	timer.start(500);
	return true;
}

void SpecificWorker::compute()
{
	//Check thread-safe interface to change viewer's innermodel
	
	//Stop viewer and reload innermodel
	qDebug() << __FILE__ << __FUNCTION__ << "Reloading viewer";
	viewer->stop.store(true);
	while(viewer->stopped.load() != true);
	viewer->reloadInnerModel(innerModel);
	viewer->stop.store(false);
	

	
}




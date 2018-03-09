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
//		innerModel = InnerModelMgr(std::make_shared<InnerModel>("/home/robocomp/robocomp/components/robocomp-araceli/etcSim/simulation.xml"));
		innerModel = std::make_shared<InnerModel>("/home/robocomp/robocomp/components/robocomp-araceli/etcSim/simulation.xml");
	}
	catch(std::exception e) 
	{ qFatal("Error reading config params"); }

		// Fold arm

	innerModel->getNode<InnerModelJoint>("armX1")->setAngle(-1);
	innerModel->getNode<InnerModelJoint>("armX2")->setAngle(2.5);

	qDebug();
	qDebug() << __FILE__ << __FUNCTION__ << "Now creating viewer";
	
	
	viewer = new InnerViewer(innerModel, "Social Navigation");  //InnerViewer copies internally innerModel so it has to be resynchronized
	viewer->start();	
	
	std::future<void> th1, th2;

	th1 = std::async(std::launch::async, &SpecificWorker::changeInner, this);
	th2 = std::async(std::launch::async, &SpecificWorker::compute, this);
	
	th1.wait();
	th2.wait();
	
// 	connect(&timer2, SIGNAL(timeout()), this, SLOT(changeInner()));
// 	timer.start(3000);
// 	timer2.start(300);
	return true;
}

void SpecificWorker::compute()
{
	while(true)
	{
		qDebug() << __FILE__ << __FUNCTION__ << "Reloading viewer";
		viewer->reloadInnerModel(innerModel);
		std::this_thread::sleep_for(3000ms);
	}		
}

void SpecificWorker::changeInner()
{
	while(true)
	{
		qDebug() << __FILE__ << __FUNCTION__ << "Changing inner";
		std::random_device r;
		std::default_random_engine e1(r());
		std::uniform_int_distribution<int> unif_dist(-3000, 3000);
		QList<QString> keys = innerModel->getIDKeys();
		std::uniform_int_distribution<int> keys_dist(0, keys.size()-1);
		auto id = keys.at(keys_dist(e1));
		std::cout << __FILE__ << __FUNCTION__ << "id " << id.toStdString() << std::endl;
		viewer->ts_updateTransformValues(id, QVec::vec6(unif_dist(e1),0,unif_dist(e1),0, unif_dist(e1),0));
		std::this_thread::sleep_for(200ms);
	}	
}



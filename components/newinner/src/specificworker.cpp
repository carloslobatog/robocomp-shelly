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
	innermodel = std::make_shared<InnerModel>(InnerModel("/home/robocomp/robocomp/components/robocomp-araceli/etcSim/simulation.xml"));
	//innermodel->print("inner");
	
	
	/// Threads
	Traverser traverser;
	Writer writer;
    for (int i = 0; i < num_threadsR; ++i)
	{
		threadsR[i] = std::thread(&Traverser::run, traverser, innermodel);
    }
	for (int i = 0; i < num_threadsW; ++i)
	{
		threadsW[i] = std::thread(&Writer::run, writer, innermodel);
    }
	for (int i = 0; i < num_threadsR; ++i)
		threadsR[i].join();
	for (int i =
		0; i < num_threadsW; ++i)
		threadsW[i].join();
	
	
    qDebug() << __FILE__ << __FUNCTION__ << "All finished";
	exit(-1);
	//timer.start(Period);
	//compute();
	return true;
}

void SpecificWorker::compute()
{
}


// void SpecificWorker::traverse(InnerModelNode *node)
// {	
// 	qDebug() << "node:" << node->id;
// 	for (int i=0; i<node->children.size(); i++)
// 	{
// 		traverse(node->children[i]);
// 	}
// }
// 
// void SpecificWorker::traverseAndChange(InnerModelNode *node)
// {	
// 	QString a = node->id;
// 	node->id = "caca";
// 	qDebug() << "node:" << node->id;
// 	node->id = a;
// 	for (int i=0; i<node->children.size(); i++)
// 	{
// 		traverseAndChange(node->children[i]);
// 	}
// }


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
	
	int R = 15;
	int IDS = 0;
	int UP = 15;
	int TR = 15;
	
	std::thread threadsR[R], threadsIDS[IDS], threadsUpdate[UP], threadsTransform[TR];
	
	/// Threads
	Traverser traverser;
	WriterIDS writer;
	WriterTransforms writertransforms;
	WriterUpdates writerupdates;
	
    for (int i = 0; i < R; ++i)
	{
		threadsR[i] = std::thread(&Traverser::run, traverser, innermodel);
    }
	for (int i = 0; i < IDS; ++i)
	{
		threadsIDS[i] = std::thread(&WriterIDS::run, writer, innermodel);
    }
    for (int i = 0; i < TR; ++i)
	{
		threadsTransform[i] = std::thread(&WriterTransforms::run, writertransforms, innermodel);
    }
    for (int i = 0; i < UP; ++i)
	{
		threadsUpdate[i] = std::thread(&WriterUpdates::run, writerupdates, innermodel);
    }
    
    
	for (int i = 0; i < R; ++i)
		threadsR[i].join();
	for (int i =0; i < IDS; ++i)
		threadsIDS[i].join();
	for (int i =0; i < TR; ++i)
		threadsTransform[i].join();
	for (int i =0; i < UP; ++i)
		threadsUpdate[i].join();
	
    qDebug() << __FILE__ << __FUNCTION__ << "All finished";
	exit(-1);
	//timer.start(Period);
	//compute();
	return true;
}

void SpecificWorker::compute()
{
}




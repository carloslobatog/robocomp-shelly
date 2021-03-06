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
	innermodel = std::make_shared<InnerModel>("/home/robocomp/robocomp/components/robocomp-araceli/etcSim/simulation.xml");
	innermodel->print("tree");
	
	int R = 1;
	int IDS = 0;
	int UP = 10;
	int TR = 0;
	int DE = 0;
	
	std::future<void> threadsR[R], threadsIDS[IDS], threadsUpdate[UP], threadsTransform[TR], threadsDelete[DE];
	
	/// Threads
	Traverser traverser;
	WriterIDS writer;
	WriterTransforms writertransforms;
	WriterUpdates writerupdates;
	WriterDeletes writerdeletes;
	
    for (int i = 0; i < R; ++i)
	{
		threadsR[i] = std::async(std::launch::async, &Traverser::run, traverser, innermodel);
    }
	for (int i = 0; i < IDS; ++i)
	{
		threadsIDS[i] = std::async(std::launch::async, &WriterIDS::run, writer, innermodel);
    }
    for (int i = 0; i < TR; ++i)
	{
		threadsTransform[i] = std::async(std::launch::async, &WriterTransforms::run, writertransforms, innermodel);
    }
    for (int i = 0; i < UP; ++i)
	{
		threadsUpdate[i] = std::async(std::launch::async, &WriterUpdates::run, writerupdates, innermodel);
    }
    for (int i = 0; i < DE; ++i)
	{
		threadsDelete[i] = std::async(std::launch::async, &WriterDeletes::run, writerdeletes, innermodel);
    }
    
	for (int i = 0; i < R; ++i)
		threadsR[i].wait();
	for (int i =0; i < IDS; ++i)
		threadsIDS[i].wait();
	for (int i =0; i < TR; ++i)
		threadsTransform[i].wait();
	for (int i =0; i < UP; ++i)
		threadsUpdate[i].wait();
	for (int i =0; i < DE; ++i)
		threadsDelete[i].wait();
	
    qDebug() << __FILE__ << __FUNCTION__ << "All finished";
	exit(-1);
	//timer.start(Period);
	//compute();
	return true;
}

void SpecificWorker::compute()
{
	
}




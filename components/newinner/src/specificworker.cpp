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
	
	int NR = 5;
	int NW = 5;
	std::thread threadsR[NR], threadsIDS[NW], threadsUpdate[NW], threadsTransform[NW];
	
	/// Threads
	Traverser traverser;
	WriterIDS writer;
	WriterTransforms writertransforms;
	WriterUpdates writerupdates;
	
    for (int i = 0; i < NR; ++i)
	{
		threadsR[i] = std::thread(&Traverser::run, traverser, innermodel);
    }
	for (int i = 0; i < NW; ++i)
	{
		threadsIDS[i] = std::thread(&WriterIDS::run, writer, innermodel);
    }
    for (int i = 0; i < NW; ++i)
	{
		threadsTransform[i] = std::thread(&WriterTransforms::run, writertransforms, innermodel);
    }
    for (int i = 0; i < NW; ++i)
	{
		threadsUpdate[i] = std::thread(&WriterUpdates::run, writerupdates, innermodel);
    }
    
    
	for (int i = 0; i < NW; ++i)
		threadsR[i].join();
	for (int i =0; i < NW; ++i)
	{
		threadsIDS[i].join();
		threadsTransform[i].join();
		threadsUpdate[i].join();
	}
	
    qDebug() << __FILE__ << __FUNCTION__ << "All finished";
	exit(-1);
	//timer.start(Period);
	//compute();
	return true;
}

void SpecificWorker::compute()
{
}




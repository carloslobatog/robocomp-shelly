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

/**
       \brief
       @author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <thread>

class Traverser
{
	public:
		Traverser(){};
		void run(std::shared_ptr<InnerModel> inner)
		{
			while(true)
			{
				traverse(inner->getRoot());
				//std::this_thread::sleep_for(50ms);
			}
		}
		void traverse(InnerModelNode *node)
		{	
			qDebug() << "node:" << node->getId();
			for (int i=0; i<node->children.size(); i++)
			{
				traverse(node->children[i]);
			}
		}
};

class Writer
{
	public:
		Writer(){};
		void run(std::shared_ptr<InnerModel> inner)
		{
			while(true)
			{
				traverse(inner->getRoot());
				//std::this_thread::sleep_for(50ms);
			}
		}
		void traverse(InnerModelNode *node)
		{	
			//qDebug() << "node:" << node->id;
			QString a = node->getId();
			node->setId("cacafasdfasdfasdfasdfasdfasdf");
			//std::this_thread::sleep_for(20ms);
			node->setId(a);
			for (int i=0; i<node->children.size(); i++)
			{
				traverse(node->children[i]);
			}
		}
};

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);


public slots:
	void compute();

private:
	std::shared_ptr<InnerModel> innermodel;
	void traverse(InnerModelNode *node);
	void traverseAndChange(InnerModelNode *node);
	
	
	int num_threadsR = 10;
	int num_threadsW = 5;
	std::thread threadsR[10], threadsW[10];

};



#endif

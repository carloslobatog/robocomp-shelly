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
#include <random>

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
			//qDebug() << "node:" << node->getId();
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
				transforms(inner);
				//std::this_thread::sleep_for(50ms);
			}
		}
		void traverse(InnerModelNode *node)
		{	
			QString a = node->getId();
			node->setId("cacafasdfasdfasdfasdfasdfasdf");
			//std::this_thread::sleep_for(20ms);
			node->setId(a);
			for (int i=0; i<node->children.size(); i++)
				traverse(node->children[i]);
		}
		
		void transforms(std::shared_ptr<InnerModel> inner)
		{
			QList<QString> keys = inner->getIDKeys();
			std::random_device r;
			std::default_random_engine e1(r());
			std::uniform_int_distribution<int> uniform_dist(0, keys.size()-1);
			
			for(int i=0; i<100; i++)
			{
				QString dest = keys[uniform_dist(e1)];
				QString orig = keys[uniform_dist(e1)];
				if( dynamic_cast<InnerModelTransform *>(inner->getNode(orig)) != nullptr and dynamic_cast<InnerModelTransform *>(inner->getNode(dest)) != nullptr)
				{
					qDebug() << orig << dest;
					inner->transform(dest, QVec::vec3(3,4,5), orig);
				}
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

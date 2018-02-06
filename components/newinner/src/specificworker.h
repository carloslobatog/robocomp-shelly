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
#include <innermodel/innermodelnode.h>
#include <innermodel/innermodelplane.h>
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
				std::this_thread::sleep_for(1ms);
				qDebug() << "Reader:";
			}
		}
		void traverse(InnerModel::NodePtr node)
		{	
			QMat r = node->getRTS(); QVec t = node->getTrTS();
			qDebug() << "Reader:" << node->getId();
			//r.print("rot"); t.print("t");
			for (int i=0; i<node->children->size(); i++)
			{
				traverse(node->children->value(i));
			}
		}
};

class WriterIDS
{
	public:
		WriterIDS(){};
		void run(std::shared_ptr<InnerModel> inner)
		{
			while(true)
			{
				traverse(inner->getRoot());
				std::this_thread::sleep_for(1ms);
			}
		}
		void traverse(InnerModel::NodePtr node)
		{	
			QString a = node->getId();
			node->setId("cacafasdfasdfasdfasdfasdfasdf");
			//std::this_thread::sleep_for(20ms);
			node->setId(a);
			for (int i=0; i<node->children->size(); i++)
				traverse(node->children->value(i));
		}
};

class WriterTransforms
{
	public:
		WriterTransforms(){};
		void run(std::shared_ptr<InnerModel> inner)
		{
			while(true)
			{
				transforms(inner);
				std::this_thread::sleep_for(1ms);
			}
		}
		void transforms(std::shared_ptr<InnerModel> inner)
		{
			QList<QString> keys = inner->getIDKeys();
			std::random_device r;
			std::default_random_engine e1(r());
			std::uniform_int_distribution<int> uniform_dist(0, keys.size()-1);
			
			QString dest = keys[uniform_dist(e1)];
			QString orig = keys[uniform_dist(e1)];	
			
			try
			{ 
				QVec v = inner->transform(dest, QVec::vec6(3,4,5,0,0,0), orig); 
				qDebug() << "Transform: " << orig << dest << v;
			}
			catch(const InnerModelException &e){ std::cout << e.what() << std::endl; };
		}
};

class WriterUpdates
{
	public: 
		WriterUpdates(){};
		void run(std::shared_ptr<InnerModel> inner)
		{
			while(true)
			{
				updates(inner);
				std::this_thread::sleep_for(1ms);
			}
		}
		void updates(std::shared_ptr<InnerModel> inner)
		{
			QList<QString> keys = inner->getIDKeys();
			std::random_device r;
			std::default_random_engine e1(r());
			std::uniform_int_distribution<int> uniform_dist(0, keys.size()-1);
			std::uniform_int_distribution<int> vs(-1000, 1000);
			QString parent = keys[uniform_dist(e1)];
			QString id = keys[uniform_dist(e1)];
			//if(id == "root") return;
			qDebug() << "Updates:" << id;
			try{ inner->updateTransformValues(id, vs(e1), vs(e1), vs(e1), vs(e1), vs(e1), vs(e1), parent); }
			catch(const InnerModelException &e){ std::cout << e.what() << std::endl; };
			
			if( inner->getNode<InnerModelJoint>(id) != nullptr)
				inner->updateJointValue(id, vs(e1), false);
		}
};

class WriterDeletes
{
	public: 
		WriterDeletes(){};
		void run(std::shared_ptr<InnerModel> inner)
		{
			while(true)
			{
				//if(innerReload())
				//	inner = reloadInner();
				updates(inner);
				std::this_thread::sleep_for(1ms);
				
			}
		}
		void updates(std::shared_ptr<InnerModel> inner)
		{
			static int a=0;
			QList<QString> keys = inner->getIDKeys();
			std::random_device r;
			std::default_random_engine e1(r());
			std::uniform_int_distribution<int> uniform_dist(0, keys.size()-1);
			std::uniform_int_distribution<int> vs(-1000, 1000);
			
			QString id = keys[uniform_dist(e1)];
			if(id == "root") return;
			qDebug() << "Deletes:" << id;
			InnerModel::NodePtr p = inner->getNode<InnerModelNode>(id);
			QStringList l;
			
			inner->removeSubTree(p, &l);
			qDebug() << "LISTA BORRADA" <<  l;
			std::this_thread::sleep_for(1ms);
					
			id = keys[uniform_dist(e1)];
			if(id == "root") return;
			InnerModel::TransformPtr pp = inner->getNode<InnerModelTransform>(id);
			if(pp != nullptr) 
			{	
				//InnerModelPlane *paux = dynamic_cast<InnerModelPlane*>(p->copyNode(inner->hash, p->parent));
				//QStringList l;
				//inner->removeSubTree(p, &l);
				//InnerModelPlane *pn = inner->newPlane(paux->getId(), paux->parent, paux->getTexture(), paux->getWidth(), paux->getHeight(), paux->getDepth(), 
				//									  paux->getRepeat(), 0, 0, 0, 0, 0, 0, false)
				;
// 				QString name = "prueba" + QString::number(a++);
// 				qDebug() << "ADDED node" << name;
// 				InnerModelTransform* pn = inner->newTransform(name, "", pp, 0, 0, 0, 0, 0, 0, 0);
				
				//paux->parent->addChild(pn);
				//delete paux;
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

};



#endif

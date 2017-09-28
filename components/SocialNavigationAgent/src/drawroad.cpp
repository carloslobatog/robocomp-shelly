/*
 * Copyright 2016 pbustos <email>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#include "drawroad.h"

// To be safe, all these methods MUST NOT access innerModelViewer->InnerModel directly

void DrawRoad::draw(Road &road, InnerViewer *viewer,  std::shared_ptr<CurrentTarget> currenttarget)
{
	///////////////////////
	// Preconditions
	///////////////////////
	if (road.isEmpty())
		return;
	
	InnerViewer::guard gl(viewer->mutex);
	//qDebug() << "In drawroad" << viewer->innerModelViewer->innerModel->getNode<InnerModelJoint>("armX1")->getAngle();
 
	try	{ viewer->removeNode("road");} catch(const QString &s){	qDebug() << s; };
	try	{ viewer->addTransform_ignoreExisting("road","world");} catch(const QString &s){qDebug() << s; };
	
	try
	{
		///////////////////
		//Draw all points
		//////////////////
		for (int i = 1; i < road.size(); i++)
		{
			WayPoint &w = road[i];
			WayPoint &wAnt = road[i - 1];
			QLine2D l(wAnt.pos, w.pos);
			QLine2D lp = l.getPerpendicularLineThroughPoint(QVec::vec2(w.pos.x(), w.pos.z()));
			QVec normal = lp.getNormalForOSGLineDraw();  //3D vector
			QVec tangent = road.getTangentAtClosestPoint().getNormalForOSGLineDraw();    
			QString item = "p_" + QString::number(i);
			viewer->addTransform_ignoreExisting(item, "road", QVec::vec6(w.pos.x(), 10, w.pos.z(), 0, 0, 0));
		
			if (i == (int)road.getIndexOfCurrentPoint() + 1)
			{
				// tangent to road 
				viewer->drawLine(item + "_line", item, QVec::zeros(3), tangent, 600, 30, "#00FFFF"); 
			}
			if (w.isVisible)
				viewer->drawLine(item + "_point", item, QVec::zeros(3), normal, 500, 50, "#00FF00"); //TAKE WIDTH FROM PARAMS!!!
			else
				viewer->drawLine(item + "_point", item, QVec::zeros(3), normal, 500, 50, "#FF0000");  //Morado
				
		}
		if (currenttarget->hasRotation() == true)    //Draws an arrow indicating final desired orientation
		{
			float rot = currenttarget->getRotation().y();
			WayPoint &w = road.last();
			QLine2D l(w.pos, w.pos + QVec::vec3((T) (500 * sin(rot)), 0, (T) (500 * cos(rot))));
			QVec ln = l.getNormalForOSGLineDraw() + QVec::vec3(100,0,100);
			QString item = "p_" + QString::number(road.size() - 1);
			viewer->drawLine(item + "_line", item, QVec::zeros(3), ln, 600, 40, "#0044AA");
		}	
	}
	catch(const QString &s){qDebug() << s;}
}

void DrawRoad::drawmap(PathPlanner &pathplanner, InnerViewer *viewer, PathPlanner::FMap const& fmap )
{
	if(pathplanner.get_map_dirty_bit()==false)
		return;
	
	InnerViewer::guard gl(viewer->mutex);
	
	try	{ viewer->removeNode("IMV_fmap");} catch(const QString &s){	qDebug() << s; };
	try	{ viewer->addTransform_ignoreExisting("IMV_fmap","world");} catch(const QString &s){qDebug() << s; };
	
	try
	{
		uint i=0;
		// Draw all points
		for ( auto it = fmap.begin(); it != fmap.end(); ++it, i++ )
		{
			QString item = "IMV_fmap_point_" + QString::number(i);
			if(it->second.free)
				viewer->addPlane_ignoreExisting(item, "IMV_fmap", QVec::vec3(it->first.x, 20, it->first.z), QVec::vec3(1,0,0), "#00FF00", QVec::vec3(60,60,60));
			else
				viewer->addPlane_ignoreExisting(item, "IMV_fmap", QVec::vec3(it->first.x, 20, it->first.z), QVec::vec3(1,0,0), "#FF0000", QVec::vec3(60,60,60));		
			
			pathplanner.set_map_dirty_bit(false);
		}
	}
	catch(const QString &s)
	{		qDebug() << s;	}
}


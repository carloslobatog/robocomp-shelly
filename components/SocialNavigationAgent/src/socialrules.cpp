/*
 * Copyright 2017 <copyright holder> <email>
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

#include "socialrules.h"

SocialRules::SocialRules()
{
	worldModel = AGMModel::SPtr(new AGMModel());
	worldModel->name = "worldModel";
	innerModel = new InnerModel();

}

SocialRules::~SocialRules()
{

}
/**
* \brief Change the slider's value
*/
void SocialRules::changevalue(int value)
{
	prox=value;
	qDebug()<<"Proximity"<<prox;
}

/**
* \brief If the person is in the model it is added to a vector of persons wich is sent to the socialnavigationGaussian
* to model its personal space. 
* The function returns a sequence of polylines.
*/

SNGPolylineSeq SocialRules::gauss(bool draw)
{
	if (!quietperson.empty())
	{  
		sequence.clear();
		sequence = socialnavigationgaussian_proxy-> getPersonalSpace(quietperson, prox, draw);
	}
	return sequence;
}


SNGPolylineSeq SocialRules::PassOnRight(bool draw)
{

	if (!movperson.empty())
	{
		sequence2.clear();
		sequence2 = socialnavigationgaussian_proxy-> getPassOnRight(movperson, prox, draw);
	}	    
	return sequence2;

	
}


SNGPolylineSeq SocialRules::objectInteraction(bool d)
{
  
	RoboCompAGMWorldModel::World w = agmexecutive_proxy->getModel();
	structuralChange(w);
	
	objects.clear();
	
	int idx=0;
	while ((objectSymbolId = worldModel->getIdentifierByType("object_interaction", idx++)) != -1)
	{	
		
	  	AGMModelSymbol::SPtr objectP = worldModel->getParentByLink(objectSymbolId, "RT");
		AGMModelEdge &edgeRT  = worldModel->getEdgeByIdentifiers(objectP->identifier,objectSymbolId, "RT");
		object.x = str2float(edgeRT.attributes["tx"])/1000;
		object.z = str2float(edgeRT.attributes["tz"])/1000;
		object.angle=str2float(edgeRT.attributes["ry"]);
		object.space=str2float(worldModel->getSymbolByIdentifier(objectSymbolId)->getAttribute("interaction"));
	
		objects.push_back(object);
		
		qDebug()<<"Object"<<"Pose x"<<object.x<<"Pose z"<<object.z<<"Angle"<<object.angle<<"Space"<<object.space;
	}
	

	sequenceObj.clear();
	sequenceObj =socialnavigationgaussian_proxy->getObjectInteraction(totalperson,objects,d);

	return sequenceObj;
	
}




RoboCompTrajectoryRobot2D::PolyLineList SocialRules::ApplySocialRules(SNGPersonSeq tperson)
{
	totalperson=tperson;
	movperson.clear();
	quietperson.clear();
	//for each person check if the velocity is 0, if it is add to totalp, if not add to totalpmov
	
	for (auto p: totalperson)
	{
		if (p.vel>0)
			movperson.push_back(p);
		else
			quietperson.push_back(p);
		
	}
	
	
	RoboCompTrajectoryRobot2D::PolyLineList list;
	
	if (!quietperson.empty())
	{
		SNGPolylineSeq secuencia = gauss(false);
		for(auto s: secuencia)
		{
			RoboCompTrajectoryRobot2D::PolyLine poly;

			for(auto p: s)
			{
				RoboCompTrajectoryRobot2D::PointL punto = {p.x, p.z};
				poly.push_back(punto);
			}
			list.push_back(poly);
		}			    
	}
	
	if (!movperson.empty())
	{
		SNGPolylineSeq secuencia2 = PassOnRight(false);
		for(auto s: secuencia2)
		{
			RoboCompTrajectoryRobot2D::PolyLine poly;

			for(auto p: s)
			{
				RoboCompTrajectoryRobot2D::PointL punto = {p.x, p.z};
				poly.push_back(punto);

			}
			list.push_back(poly);
		}
		  
		}

	if (!objects.empty())	
	{  
		SNGPolylineSeq secuenciaobj = objectInteraction(false);
		for(auto s: secuenciaobj)
		{
			RoboCompTrajectoryRobot2D::PolyLine poly;

			for(auto p: s)
			{
				RoboCompTrajectoryRobot2D::PointL punto = {p.x, p.z};
				poly.push_back(punto);

			}
			list.push_back(poly);
		}
		
	} 
	

	return list;
}

void SocialRules::structuralChange(const World& modification)
{

	printf("pre <<structuralChange\n");
	//QMutexLocker l(mx);
	printf("<<structuralChange\n");

	AGMModelConverter::fromIceToInternal(modification, worldModel);
	//if (roomsPolygons.size()==0 and worldModel->numberOfSymbols()>0)
		//roomsPolygons = extractPolygonsFromModel(worldModel);

	if (innerModel) delete innerModel;
	innerModel = AGMInner::extractInnerModel(worldModel, "world", true);
	printf("structuralChange>>\n");
}

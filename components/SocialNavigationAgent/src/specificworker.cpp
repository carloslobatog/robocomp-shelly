 /*		SOCIAL NAVIGATION NUEVO
 *    Copyright (C) 2006-2010 by RoboLab - University of Extremadura
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
#include <../trajectoryrobot2d/src/innermodeldraw.h>
#include <math.h> 

#define PI 3.14159265

/**
* \brief Default constructor
*/

SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	
	active = false;
	active = false;	
	worldModel = AGMModel::SPtr(new AGMModel());
	worldModel->name = "worldModel";
	innerModel = nullptr;
	haveTarget = false;

// 	world = AGMModel::SPtr(new AGMModel());
	
	//Timed slot to read TrajectoryRobot2D state
	connect(&trajReader, SIGNAL(timeout()), &aE, SLOT(readTrajState()));

	connect(gaussiana,SIGNAL(clicked()),&sr, SLOT(gauss()));
	connect(por,SIGNAL(clicked()),&sr, SLOT(PassOnRight()));
	connect(objint,SIGNAL(clicked()),&sr, SLOT(objectInteraction()));
	
	connect(datos,SIGNAL(clicked()),this, SLOT(savedata()));
	//trajReader.start(1000);
	
	//SLIDER
	connect (proximidad,SIGNAL(valueChanged(int)),&sr,SLOT(changevalue(int)));
	//connect (proximidad,SIGNAL(sliderMoved()),this,SLOT(sliderM()));
	
	proximidad->QSlider::setMinimum (0);
	proximidad->QSlider::setMaximum (90);	
	proximidad->QSlider::setTracking (true);	
	proximidad->QSlider::setValue (50);
	
}		
/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	Period = 200;
	timer.start(Period);
	//this->params = params;
	
	try
	{
		RoboCompAGMWorldModel::World w = agmexecutive_proxy->getModel();
		structuralChange(w);
		}
		
	catch(...)
	{
		printf("SetParams: The executive is probably not running, waiting for first AGM model publication...");
	}
	if( innerModel == nullptr)
			qFatal("SetParams: InnerModel could not be read from Executive");
	
	pathfinder.initialize(innerModel, this->params, params);
	
	
	
	//Proxies for actionExecution
	aE.logger_proxy = logger_proxy;
	aE.agmexecutive_proxy = agmexecutive_proxy;
	aE.omnirobot_proxy = omnirobot_proxy;
	aE.trajectoryrobot2d_proxy = trajectoryrobot2d_proxy;
	//Proxies for SocialRules
	sr.socialnavigationgaussian_proxy=socialnavigationgaussian_proxy;
	sr.agmexecutive_proxy=agmexecutive_proxy;
	sr.mux=mutex;
	
	return true;
}

/**
* \brief Check if persons are included in the AGM. 
* Then, their pose is stored.
* Everytime a person has moved, its position is updated
*/
void SpecificWorker::compute( )
{
	QMutexLocker locker(mutex);

	bool sendChangesAGM = false;
	static bool first=true;
	
	if (first)
	{	
		qLog::getInstance()->setProxy("both", logger_proxy);
		rDebug2(("SocialnavigationAgent started"));
		changepos=true;
		
	}
	
	AGMModel::SPtr newM(new AGMModel(worldModel));
// 	if (worldModel->getIdentifierByType("robot") < 0)
// 	{ 
// 		try 
// 		{
// 			//qDebug()<<"Leo el mundo";
// 			agmexecutive_proxy->broadcastModel();		
// 			return;
// 		}
// 		catch(...)
// 		{	
// 			//printf("The executive is probably not running, waiting for first AGM model publication...");	}	
// 		}
// 	}

	//Check if the person is in the model
	
 	for (int i=0;i<pn.size();i++)
	{

		if (pn[i]==false)
		{	
			std::string type = "person" + std::to_string(i+1);
			std::string name = "fakeperson" + std::to_string(i+1);
			
			int idx=0;
			while ((personSymbolId = newM->getIdentifierByType(type, idx++)) != -1)
			{
				if (idx > 4) exit(0);
				if (newM->getSymbolByIdentifier(personSymbolId)->getAttribute("imName") == name)
				{
					pSymbolId[i]=personSymbolId;
					changepos=true;
					pn[i]=true;
					break;
				}	  
			}			
		}
	}

//If a person has moved its pose it is updated reading it from the AGM again.

	if (changepos)
	{
		totalpersons.clear();
		
		for (int ind=0;ind<pn.size();ind++)
		{
			if (pn[ind])
			{	
				
				
				AGMModelSymbol::SPtr personParent = newM->getParentByLink(pSymbolId[ind], "RT");
				AGMModelEdge &edgeRT = newM->getEdgeByIdentifiers(personParent->identifier, pSymbolId[ind], "RT");
				person.x = str2float(edgeRT.attributes["tx"])/1000;
				person.z = str2float(edgeRT.attributes["tz"])/1000;
				person.angle = str2float(edgeRT.attributes["ry"]);
	 			//person.vel=str2float(edgeRT.attributes["velocity"]);			
				person.vel=0;
				totalpersons.push_back(person);		
				qDebug() <<"PERSONA " <<ind+1  <<" Coordenada x"<< person.x << "Coordenada z"<< person.z << "Rotacion "<< person.angle/0.0175;			
				if (totalaux.empty())
				{
					//This must be changed. If the first human to be inserted is human2 it would be wrong
					//totalaux.push_back(person);
					totalaux[ind]=person;
					movperson=true;
				}
				else if  (movperson==false)
				{
					if ((totalaux[ind].x!=person.x)or(totalaux[ind].z!=person.z)or(totalaux[ind].angle!=person.angle))
						movperson = true;
			
					totalaux[ind]=person;  	  
					
				}
				
				/////////////////////checking if the person is looking at the robot /////////////////////////
				
				try
				{	
					qDebug()<<"------------------------------------------------";
					if (sr.checkHRI(person,ind+1,innerModel,newM) == true)
					{	
						qDebug()<<"SEND MODIFICATION PROPOSAL";
						sendChangesAGM = true;
						
					}
					else 
						qDebug()<<"NO HAY MODIFICACION";
				}
				catch(...)
				{
			
				}
			}
		}
		
		robotSymbolId = newM->getIdentifierByType("robot");
		AGMModelSymbol::SPtr robotparent = newM->getParentByLink(robotSymbolId, "RT");
		AGMModelEdge &edgeRTrobot  = newM->getEdgeByIdentifiers(robotparent->identifier, robotSymbolId, "RT");
			
		robot.x=str2float(edgeRTrobot.attributes["tx"])/1000;
		robot.z=str2float(edgeRTrobot.attributes["tz"])/1000;
		robot.angle=str2float(edgeRTrobot.attributes["ry"]);

		point.x=robot.x;
		point.z=robot.z;
		 
		if (poserobot.size()==0)
			poserobot.push_back(point);
	  
		else if ((poserobot[poserobot.size()-1].x!=point.x)or(poserobot[poserobot.size()-1].z!=point.z))		  
		{  
		  float  dist=sqrt((point.x - poserobot[poserobot.size()-1].x)*(point.x - poserobot[poserobot.size()-1].x)
				+(point.z - poserobot[poserobot.size()-1].z)*(point.z - poserobot[poserobot.size()-1].z));
		    
		  totaldist=totaldist + dist;
		  qDebug()<<"Distancia calculada"<<dist<<"Distancia total"<<totaldist;
		    
		  poserobot.push_back(point);  
		}		 	
		
		first = false;
		changepos=false;	
	}
		  
	if (movperson)
	{
	
		try
		{
			RoboCompTrajectoryRobot2D::PolyLineList list = sr.ApplySocialRules(totalpersons);
			trajectoryrobot2d_proxy->setHumanSpace(list);
		}
		
		catch( const Ice::Exception &e)
		{ 
//			std::cout << e << std::endl;
		}
		
	}	
	
	
	movperson=false;
	
	//qDebug()<<"Update actionEx";
	//aE.Update(action,params);
	
	if (sendChangesAGM)
	{	
		try
		{
			sendModificationProposal(newM,worldModel,"-");
		
		}
		catch(...){}
	}
}
	 	

void SpecificWorker::savedata()
{
	
	qDebug("Saving in robotpose.txt the robot's pose");
  	ofstream file("robotpose.txt", ofstream::out);
	for (auto p:poserobot)
	{
		file<< p.x << " " <<p.z<< endl;
	}
	file.close();
	
	qDebug("Saving in personpose.txt the human's poses");
  	ofstream file2("personpose.txt", ofstream::out);
	for (auto person:totalpersons)
	{
		file2<< person.x << " " <<person.z<<" "<<person.angle<< endl;
	}
	file2.close();	
	poserobot.clear();
	
	
	qDebug("Saving poly.txt la polilinea");
  	ofstream file3("poly.txt", ofstream::out);
	
	for (auto s:sequence)
	{
		for (auto p: s)
		{
			file3<< p.x << " " <<p.z<<" "<< endl;
		}
	}
	file3.close();	
	
	qDebug()<<"Saving in dist.txt the total distance"<<totaldist;
  	ofstream file4("dist.txt", ofstream::out);
	file4<< totaldist << endl;
	totaldist = 0;
	file4.close();

	/////Guardar cada polilinea por separado
	int i = 0;
	for (auto s:sequence)
	{
		QString name = QString("polyline")+QString::number(i,10)+QString(".txt");
		ofstream file5(name.toUtf8().constData(), ofstream::out);
		for (auto p: s)
		{
			file5<< p.x << " " <<p.z<<" "<< endl;
		}
		i++;
		file5.close();
	}

}

/**
* \brief The innerModel is extracted from the AGM and the polylines are inserted on it as a set of planes.
*/

void SpecificWorker::UpdateInnerModel(SNGPolylineSeq seq)
{
	QMutexLocker locker(mutex);
	qDebug() << __FUNCTION__ << "UpdadeInnerModel";

//EXTRACT INNERMODEL	
	innerModel = AGMInner::extractInnerModel(worldModel, "world", false);
		
//INSERT POLYLINES
	
	int count = 0;
	
	for (auto s:seq)
	{
		auto previousPoint = s[s.size()-1];	
		for (auto currentPoint:s)
		{
			QString name = QString("polyline_obs_")+QString::number(count,10);
			//qDebug() << __FUNCTION__ << "nombre"<<name;
			QVec ppoint = QVec::vec3(previousPoint.x*1000, 1000, previousPoint.z*1000);
			QVec cpoint = QVec::vec3(currentPoint.x*1000, 1000, currentPoint.z*1000);
			QVec center = (cpoint + ppoint).operator*(0.5);
			
			QVec normal = (cpoint-ppoint);
			float dist=normal.norm2();	
			float temp = normal(2);
			normal(2) = normal(0);
			normal(0) = -temp;
		
			if (innerModel->getNode(name))
			{
				try
				{
					innerModel->removeNode(name);
				}
				  
				catch(QString es){ qDebug() << "EXCEPCION" << es;}
			}
			
			InnerModelNode *parent = innerModel->getNode(QString("world"));			
			if (parent == NULL)
				printf("%s: parent does not exist\n", __FUNCTION__);
			else
			{			
				InnerModelPlane *plane;
				try
				{
					plane  = innerModel->newPlane(name, parent, QString("#FFFF00"), dist, 2000, 90, 1, normal(0), normal(1), normal(2), center(0), center(1), center(2), true);
					parent->addChild(plane); 
				}
				catch(QString es)
				{ 
					qDebug() << "EXCEPCION" << es;}
				}
			count++;
			previousPoint=currentPoint;
		}
	}
	
}	


// *****************************************************************************************++
// AGENT RELATED
// **********************************************************************************************

bool SpecificWorker::activateAgent(const ParameterMap& prs)
{
	bool activated = false;
	printf("<<activateAgent\n");
	if (setParametersAndPossibleActivation(prs, activated))
	{
			if (not activated)
			{
				printf("activateAgent 0 >>\n");
				return activate(p);
			}
	}
	else
	{
		printf("activateAgent 1 >>\n");
		return false;
	}
	printf("activateAgent 2 >>\n");
	return true;
}

bool SpecificWorker::deactivateAgent()
{
		return deactivate();
}

StateStruct SpecificWorker::getAgentState()
{
	StateStruct s;
	if (isActive())
	{
		s.state = Running;
	}
	else
	{
		s.state = Stopped;
	}
	s.info = p.action.name;
	return s;
}

ParameterMap SpecificWorker::getAgentParameters()
{
	return params;
}

bool SpecificWorker::setAgentParameters(const ParameterMap& prs)
{
	bool activated = false;
	return setParametersAndPossibleActivation(prs, activated);
}

void SpecificWorker::killAgent()
{
}

Ice::Int SpecificWorker::uptimeAgent()
{
	return 0;
}

bool SpecificWorker::reloadConfigAgent()
{
	return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////

void SpecificWorker::structuralChange(const RoboCompAGMWorldModel::World& modification)
{	qDebug()<<"StructuralChange";
	printf("pre <<structuralChange\n");
	QMutexLocker l(mutex);
	printf("<<structuralChange\n");

	AGMModelConverter::fromIceToInternal(modification, worldModel);
	
	//if (roomsPolygons.size()==0 and worldModel->numberOfSymbols()>0)
		//roomsPolygons = extractPolygonsFromModel(worldModel);

	if (innerModel) delete innerModel;
	
	innerModel = AGMInner::extractInnerModel(worldModel, "world", false);

	changepos=true;
	
	printf("structuralChange>>\n");
	
}
void SpecificWorker::symbolUpdated(const RoboCompAGMWorldModel::Node& modification)
{
  //qDebug()<<"symbolUpdated";
	
	QMutexLocker l(mutex);

	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
}

void SpecificWorker::symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence &modifications)
{
  	
  //qDebug()<<"symbolsUpdated";
	QMutexLocker l(mutex);

	for (auto modification : modifications)
		AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
}


void SpecificWorker::edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications)
{	//qDebug()<<"edgesUpdated";
	changepos=true;
	QMutexLocker lockIM(mutex);
	
	for (auto modification : modifications)
	{
	  
		AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
		AGMModelEdge dst;
		AGMModelConverter::fromIceToInternal(modification,dst);
		AGMInner::updateImNodeFromEdge(worldModel, dst, innerModel);
	}
}

/**
 * \brief ACTUALIZACION DEL ENLACE EN INNERMODEL
 */ 
void SpecificWorker::edgeUpdated(const RoboCompAGMWorldModel::Edge& modification)
{	
	changepos=true;
	
	//qDebug()<<"edgeUpdated";
	QMutexLocker lockIM(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
	AGMModelEdge dst;
	AGMModelConverter::fromIceToInternal(modification,dst);
	AGMInner::updateImNodeFromEdge(worldModel, dst, innerModel);
}


bool SpecificWorker::setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated)
{
	printf("<<< setParametersAndPossibleActivation\n");
	// We didn't reactivate the component
	reactivated = false;

	// Update parameters
	params.clear();
	for (ParameterMap::const_iterator it=prs.begin(); it!=prs.end(); it++)
	{
		printf("param:%s   value:%s\n", it->first.c_str(), it->second.value.c_str());
		params[it->first] = it->second;
	}
	printf("----\n");

	try
	{
		action = params["action"].value;
		std::transform(action.begin(), action.end(), action.begin(), ::tolower);

		if (action == "graspobject")
		{
			active = true;
		}
		else
		{
			active = true;
		}
	}
	catch (...)
	{
		printf("exception in setParametersAndPossibleActivation %d\n", __LINE__);
		return false;
	}

	// Check if we should reactivate the component
	if (active)
	{
		active = true;
		reactivated = true;
	}

	printf("setParametersAndPossibleActivation >>>\n");

	return true;
}


void SpecificWorker::sendModificationProposal(AGMModel::SPtr &newModel, AGMModel::SPtr &worldModel, string m)
{
	QMutexLocker locker(mutex);
	
	try
	{	
		AGMMisc::publishModification(newModel, agmexecutive_proxy, std::string( "SocialnavigationAgent")+m);
	}
	catch(const RoboCompAGMExecutive::Locked &e)
	{
	}
	catch(const RoboCompAGMExecutive::OldModel &e)
	{
		printf("modelo viejo\n");
	}
	catch(const RoboCompAGMExecutive::InvalidChange &e)
	{
		printf("modelo invalido\n");
	}
	catch(const Ice::Exception& e)
	{
		exit(1);
	}
	
}

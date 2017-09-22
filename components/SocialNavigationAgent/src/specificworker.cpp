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
#include <math.h> 

#define PI M_PI

/**
 * \brief Default constructor
 */

SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	
	active = false;
	active = false;	
	worldModel = AGMModel::SPtr(new AGMModel());
	worldModel->name = "worldModel";
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

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList paramsL)
{
	//Extract robot name 
	try{ robotname = paramsL.at("RobotName").value;} 
	catch(const std::exception &e){ std::cout << e.what() << "SpecificWorker::SpecificWorker - Robot name defined in config. Using default 'robot' " << std::endl;}
	
	try
	{		
//		RoboCompAGMWorldModel::World w = agmexecutive_proxy->getModel();
//		AGMModelConverter::fromIceToInternal(w, worldModel);
  		//innerModel = AGMInner::extractInnerModel(worldModel, "world", false);
			//		structuralChange(w);
//		rDebug2(("Extracting InnerModel"));	
	
//		RoboCompAGMWorldModel::World w = agmexecutive_proxy->getModel();
//		structuralChange(w);
	}		
	catch(...)
	{	rDebug2(("The executive is probably not running, waiting for first AGM model publication...")); }
	
// 	if( innerModel == nullptr)
// 		qFatal("SetParams: InnerModel could not be read from the DSR graph");
	
	innerModel = InnerModelMgr(std::make_shared<InnerModel>("/home/pbustos/robocomp/components/robocomp-araceli/etcSim/simulation.xml"));

	// Fold arm
 	innerModel->getNode<InnerModelJoint>("armX1")->setAngle(-1);
 	innerModel->getNode<InnerModelJoint>("armX2")->setAngle(2.5);
	
	std::shared_ptr<RoboCompCommonBehavior::ParameterList> configparams = std::make_shared<RoboCompCommonBehavior::ParameterList>(paramsL);
	
	// Initializing PathFinder
	pathfinder.initialize(innerModel, configparams, laser_proxy, omnirobot_proxy);
	
	// releasing pathfinder
	thread_pathfinder = std::thread(&robocomp::pathfinder::PathFinder::run, &pathfinder);
	rDebug2(("Pathfinder up and running"));

	#ifdef USE_QTGUI
		//viewer = new InnerViewer(innerModel);  // makes a copy of innermodel for internal use
		//viewer->start();	
	#endif

	qLog::getInstance()->setProxy("both", logger_proxy);
	rDebug2(("NavigationAgent started"));

	Period = 200;
	timer.start(Period);


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
// 	static bool first=true;
// 	if (first)
// 	{	
// 		qLog::getInstance()->setProxy("both", logger_proxy);
// 		rDebug2(("navigationAgent started"));
// 		first = false;
// 	}

	// PROVISIONAL read robot position from proxy
	omnirobot_proxy->getBaseState(bState);
	//qDebug() << "SpecificWorker::compute" << bState.x << bState.z << bState.alpha;
	//We need to secure access to InnerModel 
	innerModel.lock();
		innerModel->updateTransformValues("robot", bState.x,0,bState.z,0,bState.alpha,0);
	innerModel.unlock();
	
 	pathfinder.innerModelChanged(innerModel, false);
	
	//
	//updateRobotPosition();
	
	
	//qDebug() << SpecificWorker::compute";
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
// 
// 	//Check if the person is in the model
// 	for (uint i=0;i < pn.size();i++)
// 	{
// 
// 		if (pn[i]==false)
// 		{	
// 			std::string type = "person" + std::to_string(i+1);
// 			std::string name = "fakeperson" + std::to_string(i+1);
// 
// 			int idx=0;
// 			while ((personSymbolId = worldModel->getIdentifierByType(type, idx++)) != -1)
// 			{
// 				if (idx > 4) exit(0);
// 				if (worldModel->getSymbolByIdentifier(personSymbolId)->getAttribute("imName") == name)
// 				{
// 					pSymbolId[i]=personSymbolId;
// 					pn[i]=true;
// 
// 					break;
// 				}	  
// 			}			
// 		}
// 	}
// 
// 	//If a person has moved its pose it is updated reading it from the AGM again.
// 
// 	if (changepos)
// 	{
// 		totalp.clear();
// 		totalpmov.clear();
// 		totalpersons.clear();
// 		for (u_int16_t ind=0;ind < pn.size();ind++)
// 		{
// 			if (pn[ind])
// 			{
// 				AGMModelSymbol::SPtr personParent = worldModel->getParentByLink(pSymbolId[ind], "RT");
// 				AGMModelEdge &edgeRT = worldModel->getEdgeByIdentifiers(personParent->identifier, pSymbolId[ind], "RT");
// 				person.x = str2float(edgeRT.attributes["tx"])/1000;
// 				person.z = str2float(edgeRT.attributes["tz"])/1000;
// 				person.angle = str2float(edgeRT.attributes["ry"]);
// 				// 			person.vel=str2float(edgeRT.attributes["velocity"]);			
// 				totalpersons.push_back(person);		
// 
// 				// 			if(person.vel>0)
// 				// 				ppn[ind]=true;
// 				//				totalpmov.push_back(person);
// 				// 			else
// 				// 				ppn[ind]=false;
// 				totalp.push_back(person);		
// 			}
// 
// 			if (first)
// 			{
// 				totalaux.push_back(person);
// 				movperson=true;
// 			}
// 			else
// 			{
// 				if  (movperson==false){
// 					if ((totalaux[i].x!=person.x)||(totalaux[i].z!=person.z)||(totalaux[i].angle!=person.angle))
// 						movperson = true;
// 				}
// 
// 				totalaux[i]=person;
// 			}
// 		}
// 
// 		robotSymbolId = worldModel->getIdentifierByType("robot");
// 		AGMModelSymbol::SPtr robotparent = worldModel->getParentByLink(robotSymbolId, "RT");
// 		AGMModelEdge &edgeRTrobot  = worldModel->getEdgeByIdentifiers(robotparent->identifier, robotSymbolId, "RT");
// 
// 		robot.x=str2float(edgeRTrobot.attributes["tx"])/1000;
// 		robot.z=str2float(edgeRTrobot.attributes["tz"])/1000;
// 		robot.angle=str2float(edgeRTrobot.attributes["ry"]);
// 
// 		point.x=robot.x;
// 		point.z=robot.z;
// 
// 		if (poserobot.size()==0)
// 			poserobot.push_back(point);
// 
// 		else if ((poserobot[poserobot.size()-1].x!=point.x)||(poserobot[poserobot.size()-1].z!=point.z))		  
// 		{  
// 			float  dist=sqrt((point.x - poserobot[poserobot.size()-1].x)*(point.x - poserobot[poserobot.size()-1].x)
// 					+(point.z - poserobot[poserobot.size()-1].z)*(point.z - poserobot[poserobot.size()-1].z));
// 
// 			totaldist=totaldist + dist;
// 			qDebug()<<"Distancia calculada"<<dist<<"Distancia total"<<totaldist;
// 
// 			poserobot.push_back(point);  
// 		}		 	    
// 		first = false;
// 		changepos=false;	
// 	}
// 
// 	if (movperson)
// 	{
// 		qDebug ("A person has moved. Calling trajectory");		
// 		try
// 		{  
// 			SNGPolylineSeq seq = gauss(false);
// 			UpdateInnerModel(seq);
// 			RoboCompTrajectoryRobot2D::PolyLineList list;
// 			for(auto s: seq)
// 			{
// 				RoboCompTrajectoryRobot2D::PolyLine poly; 
// 				for(auto p: s)   
// 				{
// 					RoboCompTrajectoryRobot2D::PointL pointT = {p.x, p.z};
// 					poly.push_back(pointT);
// 				}
// 				list.push_back(poly);
// 			}
// 			//  trajectoryrobot2d_proxy->setHumanSpace(list);
// 		}
// 		catch( const Ice::Exception &e)
// 		{ 
// 			std::cout << e << std::endl;
// 		}		
// 		movperson = false;
// 	}	

	//actionExecution();
}	 	


float SpecificWorker::goReferenced(const TargetPose &target, const float xRef, const float zRef, const float threshold)
{
	//std::shared_ptr<InnerModel> in = std::make_shared<InnerModel>("/home/pbustos/robocomp/components/robocomp-araceli/etcSim/simulation.xml");
	InnerModelMgr newInnerModel = InnerModelMgr(std::make_shared<InnerModel>("/home/pbustos/robocomp/components/robocomp-araceli/etcSim/simulation.xml"));
	innerModel.reset(newInnerModel);
	pathfinder.innerModelChanged(newInnerModel, true);
	pathfinder.go(target.x, target.z); return 0.0;
};


/**
 * \brief Change the slider's value
 */

// void SpecificWorker::changevalue(int value)
// {
// 	prox=value;
// 	qDebug()<<"Proximity" << prox;
// }
// /**
//  * \brief This is for saving in txt files different informations (robotpose,personpose,polylines and dist)
//  */
// 
// 	//Check if the person is in the model
//  	for (int i=0;i<pn.size();i++)
// 	{
// 
// 		if (pn[i]==false)
// 		{	
// 			std::string type = "person" + std::to_string(i+1);
// 			std::string name = "fakeperson" + std::to_string(i+1);
// 			
// 			int idx=0;
// 			while ((personSymbolId = newM->getIdentifierByType(type, idx++)) != -1)
// 			{
// 				if (idx > 4) exit(0);
// 				if (newM->getSymbolByIdentifier(personSymbolId)->getAttribute("imName") == name)
// 				{
// 					pSymbolId[i]=personSymbolId;
// 					changepos=true;
// 					pn[i]=true;
// 					break;
// 				}	  
// 			}			
// 		}
// 	}

//If a person has moved its pose it is updated reading it from the AGM again.

// 	if (changepos)
// 	{
// 		totalpersons.clear();
// 		
// 		for (int ind=0;ind<pn.size();ind++)
// 		{
// 			if (pn[ind])
// 			{	
// 				
// 				
// 				AGMModelSymbol::SPtr personParent = newM->getParentByLink(pSymbolId[ind], "RT");
// 				AGMModelEdge &edgeRT = newM->getEdgeByIdentifiers(personParent->identifier, pSymbolId[ind], "RT");
// 				person.x = str2float(edgeRT.attributes["tx"])/1000;
// 				person.z = str2float(edgeRT.attributes["tz"])/1000;
// 				person.angle = str2float(edgeRT.attributes["ry"]);
// 	 			//person.vel=str2float(edgeRT.attributes["velocity"]);			
// 				person.vel=0;
// 				totalpersons.push_back(person);		
// 				qDebug() <<"PERSONA " <<ind+1  <<" Coordenada x"<< person.x << "Coordenada z"<< person.z << "Rotacion "<< person.angle/0.0175;			
// 				if (totalaux.empty())
// 				{
// 					//This must be changed. If the first human to be inserted is human2 it would be wrong
// 					//totalaux.push_back(person);
// 					totalaux[ind]=person;
// 					movperson=true;
// 				}
// 				else if  (movperson==false)
// 				{
// 					if ((totalaux[ind].x!=person.x)or(totalaux[ind].z!=person.z)or(totalaux[ind].angle!=person.angle))
// 						movperson = true;
// 			
// 					totalaux[ind]=person;  	  
// 					
// 				}
// 				
// 				/////////////////////checking if the person is looking at the robot /////////////////////////
// 				
// 				try
// 				{	
// 					qDebug()<<"------------------------------------------------";
// 					if (sr.checkHRI(person,ind+1,innerModel,newM) == true)
// 					{	
// 						qDebug()<<"SEND MODIFICATION PROPOSAL";
// 						sendChangesAGM = true;
// 						
// 					}
// 					else 
// 						qDebug()<<"NO HAY MODIFICACION";
// 				}
// 				catch(...)
// 				{
// 			
// 				}
// 			}
// 		}
// 		
// 		robotSymbolId = newM->getIdentifierByType("robot");
// 		AGMModelSymbol::SPtr robotparent = newM->getParentByLink(robotSymbolId, "RT");
// 		AGMModelEdge &edgeRTrobot  = newM->getEdgeByIdentifiers(robotparent->identifier, robotSymbolId, "RT");
// 			
// 		robot.x=str2float(edgeRTrobot.attributes["tx"])/1000;
// 		robot.z=str2float(edgeRTrobot.attributes["tz"])/1000;
// 		robot.angle=str2float(edgeRTrobot.attributes["ry"]);
// 
// 		point.x=robot.x;
// 		point.z=robot.z;
// 		 
// 		if (poserobot.size()==0)
// 			poserobot.push_back(point);
// 	  
// 		else if ((poserobot[poserobot.size()-1].x!=point.x)or(poserobot[poserobot.size()-1].z!=point.z))		  
// 		{  
// 		  float  dist=sqrt((point.x - poserobot[poserobot.size()-1].x)*(point.x - poserobot[poserobot.size()-1].x)
// 				+(point.z - poserobot[poserobot.size()-1].z)*(point.z - poserobot[poserobot.size()-1].z));
// 		    
// 		  totaldist=totaldist + dist;
// 		  qDebug()<<"Distancia calculada"<<dist<<"Distancia total"<<totaldist;
// 		    
// 		  poserobot.push_back(point);  
// 		}		 	
// 		
// 		first = false;
// 		changepos=false;	
// 	}
// 		  
// 	if (movperson)
// 	{
// 	
// 		try
// 		{
// 			RoboCompTrajectoryRobot2D::PolyLineList list = sr.ApplySocialRules(totalpersons);
// 			trajectoryrobot2d_proxy->setHumanSpace(list);
// 		}
// 		
// 		catch( const Ice::Exception &e)
// 		{ 
// //			std::cout << e << std::endl;
// 		}
// 		
// 	}	
// 	
// 	
// 	movperson=false;
// 	
// 	//qDebug()<<"Update actionEx";
// 	//aE.Update(action,params);
// 	
// 	if (sendChangesAGM)
// 	{	
// 		try
// 		{
// 			sendModificationProposal(newM,worldModel,"-");
// 		
// 		}
// 		catch(...){}
// 	}
// }
// 	 	

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
 * \brief If the person is in the model it is added to a vector of persons wich is sent to the socialnavigationGaussian
 * to model its personal space. 
 * The function returns a sequence of polylines.
 */

// SNGPolylineSeq SpecificWorker::gauss(bool draw)
// {
// 	sequence.clear();
// 	//sequence = socialnavigationgaussian_proxy-> getPersonalSpace(totalp, prox, draw);
// 	return sequence;
// }


/**
 * @brief Updates the robot's position in viewer
 * 
 */
 void SpecificWorker::updateRobotPosition()
 {
 	innerModel->updateTransformValues("robot", bState.x,0,bState.z,0,bState.alpha,0);
 	pathfinder.innerModelChanged(innerModel, false);
 }

/**
 * \brief The innerModel is extracted from the AGM and the polylines are inserted on it as a set of planes.
 */

void SpecificWorker::UpdateInnerModel(SNGPolylineSeq seq)
{
	QMutexLocker locker(mutex);
	qDebug() << __FUNCTION__ << "UpdadeInnerModel";

	// Extract innerModel	
	//innerModel.reset(AGMInner::extractInnerModel(worldModel, "world", false));  ///OJO esto deja a las otras copias con el antiguo
	
			
	//INSERT POLYLINES

// 	int count = 0;
// 
// 	for (auto s:seq)
// 	{
// 		auto previousPoint = s[s.size()-1];	
// 		for (auto currentPoint:s)
// 		{
// 			QString name = QString("polyline_obs_")+QString::number(count,10);
// 			//qDebug() << __FUNCTION__ << "nombre"<<name;
// 			QVec ppoint = QVec::vec3(previousPoint.x*1000, 1000, previousPoint.z*1000);
// 			QVec cpoint = QVec::vec3(currentPoint.x*1000, 1000, currentPoint.z*1000);
// 			QVec center = (cpoint + ppoint).operator*(0.5);
// 
// 			QVec normal = (cpoint-ppoint);
// 			float dist=normal.norm2();	
// 			float temp = normal(2);
// 			normal(2) = normal(0);
// 			normal(0) = -temp;
// 
// 			if (innerModel->getNode(name))
// 			{
// 				try
// 				{
// 					innerModel->removeNode(name);
// 				}
// 
// 				catch(QString es){ qDebug() << "EXCEPCION" << es;}
// 			}
// 
// 			InnerModelNode *parent = innerModel->getNode(QString("world"));			
// 			if (parent == NULL)
// 				printf("%s: parent does not exist\n", __FUNCTION__);
// 			else
// 			{			
// 				InnerModelPlane *plane;
// 				try
// 				{
// 					plane  = innerModel->newPlane(name, parent, QString("#FFFF00"), dist, 2000, 90, 1, normal(0), normal(1), normal(2), center(0), center(1), center(2), true);
// 					parent->addChild(plane); 
// 				}
// 				catch(QString es)
// 				{ 
// 					qDebug() << "EXCEPCION" << es;}
// 			}
// 			count++;
// 			previousPoint=currentPoint;
// 		}
// 	}
// 
// 	if (i==3) innerModel->save("innerInicio.xml"); 
// 	if (i==50) innerModel->save("innerfinal.xml"); 
// 
// 	i++;
}	


/**
 * \brief ESTE ES EL VERDADERO COMPUTE
 */ 
// void SpecificWorker::actionExecution()
// {
// 	QMutexLocker locker(mutex);
// 	qDebug() << "---------------------------------------------------";
// 	qDebug() <<__FUNCTION__ <<"Checking ACTION: " << QString::fromStdString(action);
// 			QVec normal = (cpoint-ppoint);
// 			float dist=normal.norm2();	
// 			float temp = normal(2);
// 			normal(2) = normal(0);
// 			normal(0) = -temp;
// 		
// 			if (innerModel->getNode(name))
// 			{
// 				try
// 				{
// 					innerModel->removeNode(name);
// 				}
// 				  
// 				catch(QString es){ qDebug() << "EXCEPCION" << es;}
// 			}
// 			
// 			InnerModelNode *parent = innerModel->getNode(QString("world"));			
// 			if (parent == NULL)
// 				printf("%s: parent does not exist\n", __FUNCTION__);
// 			else
// 			{			
// 				InnerModelPlane *plane;
// 				try
// 				{
// 					plane  = innerModel->newPlane(name, parent, QString("#FFFF00"), dist, 2000, 90, 1, normal(0), normal(1), normal(2), center(0), center(1), center(2), true);
// 					parent->addChild(plane); 
// 				}
// 				catch(QString es)
// 				{ 
// 					qDebug() << "EXCEPCION" << es;}
// 				}
// 			count++;
// 			previousPoint=currentPoint;
// 		}
// 	}
// 	
// }	
// 

// *****************************************************************************************++
// AGENT RELATED
// **********************************************************************************************

// 	if (action == "changeroom")
// 	{
// 		action_ChangeRoom(newAction);
// 	}
// 	else if (action == "findobjectvisuallyintable")
// 	{
// 		action_FindObjectVisuallyInTable(newAction);
// 	}
// 	else if (action == "setobjectreach" or action == "graspobject")
// 	{
// 		action_SetObjectReach(newAction);
// 	}
// 	else if (action == "detectperson")
// 	{
// 		action_DetectPerson(newAction);
// 	}
// 	else if (action == "handobject")
// 	{
// 		//		action_HandObject(newAction);
// 	}
// 	else if (action == "setstop")
// 	{
// 		action_Stop();
// 	}
// 	else if (action == "reachpose")
// 	{
// 		action_ReachPose();
// 	}
// 	else if (action == "noAction")
// 	{
// 
// 	}
// 	else if (action == "handobject_offer")
// 	{
// 		action_HandObject_Offer();
// 	}
// 	else if (action == "handobject_leave")
// bool SpecificWorker::activateAgent(const ParameterMap& prs)
// {
// 	bool activated = false;
// 	printf("<<activateAgent\n");
// 	if (setParametersAndPossibleActivation(prs, activated))
// 	{
// 			if (not activated)
// 			{
// 				printf("activateAgent 0 >>\n");
// 				return activate(p);
// 			}
// 	}
// 	else
// 	{
// 		printf("activateAgent 1 >>\n");
// 		return false;
// 	}
// 	manageReachedPose();
// 	// 	printf("actionExecution>>\n");
// 	printf("activateAgent 2 >>\n");
// 	return true;
// }

// bool SpecificWorker::deactivateAgent()
// {
// 		return deactivate();
// }
// 
// StateStruct SpecificWorker::getAgentState()
// {
// 
// 	static bool b=false;
// 	if (b==false)
// 	{
// 		//trajectoryrobot2d_proxy->stop();
// 		b=true;
// 	}
// 	StateStruct s;
// 	if (isActive())
// 	{
// 		s.state = Running;
// 	}
// 	else
// 	{
// 		s.state = Stopped;
// 	}
// 	s.info = p.action.name;
// 	return s;
// }
// 
// ParameterMap SpecificWorker::getAgentParameters()
// {
// 	return params;
// }
// 
// bool SpecificWorker::setAgentParameters(const ParameterMap& prs)
// {
// 	bool activated = false;
// 	return setParametersAndPossibleActivation(prs, activated);
// }
// 
// void SpecificWorker::killAgent()
// {
// }
// 
// Ice::Int SpecificWorker::uptimeAgent()
// {
// 	return 0;
// }

// 	// Get target
// 	int roomID, personID, robotID;
// 	try
// 	{
// 		if (symbols["room"].get() and symbols["person"].get() and symbols["robot"].get())
// 		{
// 			roomID = symbols["room"]->identifier;   //7 ROOM
// 			personID =symbols["person"]->identifier;//  PERSON
// 			robotID = symbols["robot"]->identifier; //1 ROBOT
// 		}
// 		else
// 		{
// 			printf("navigationAgent, action_HandObject: parameters not in the model yet\n");
// 			return;
// 		}
// 	}
// 	catch(...)
// 	{
// 		printf("navigationAgent, action_HandObject ERROR: SYMBOL DOESN'T EXIST \n");
// 		exit(2);
// 	}
// 
// 	// GET THE INNERMODEL NAMES OF TH SYMBOLS
// 	QString robotIMID;
// 	QString roomIMID;
// 	QString personIMID;
// 	try
// 	{
// 		robotIMID = QString::fromStdString(worldModel->getSymbol(robotID)->getAttribute("imName"));
// 		roomIMID = QString::fromStdString(worldModel->getSymbol(roomID)->getAttribute("imName"));
// 		//we need to obtain the imName of the torso node. TrackingId+"XN_SKEL_TORSO"
// 		QString trackingId= QString::fromStdString(worldModel->getSymbol(personID)->getAttribute("TrackingId"));
// 		personIMID = trackingId +"XN_SKEL_TORSO";
// 	}
// 	catch(...)
// 	{
// 		printf("navigationAgent, action_HandObject: ERROR IN GET THE INNERMODEL NAMES\n");
// 		qDebug()<<"[robotIMID"<<robotIMID<<"roomIMID"<<roomIMID<<"personIMID"<<personIMID<<"]";
// 		exit(2);
// 	}
// 
// 	// GET THE TARGET POSE: 
// 	RoboCompTrajectoryRobot2D::TargetPose tgt;
// 	try
// 	{
// 		if (not (innerModel->getNode(roomIMID) and innerModel->getNode(personIMID)))    return;
// 
// 		QVec poseInRoom = innerModel->transform6D(roomIMID, personIMID); // FROM OBJECT TO ROOM
// 		qDebug()<<"[robotIMID"<<robotIMID<<"roomIMID"<<roomIMID<<"personIMID"<<personIMID<<"]";
// 		qDebug()<<" TARGET POSE: "<< poseInRoom;
// 
// 		tgt.x = poseInRoom.x();
// 		tgt.y = 0;
// 		tgt.z = poseInRoom.z();
// 		tgt.rx = 0;
// 		tgt.ry = 0;
// 		tgt.rz = 0;
// 		tgt.doRotation = false;
// 
// 	}
// 	catch (...) 
// 	{ 
// 		qDebug()<<"navigationAgent, action_HandObject: innerModel exception";
// 	}
// 
// 	try
// 	{
// 		// 		if (!haveTarget)
// 		{
// 			try
// 			{
// 				QVec graspRef = innerModel->transformS(robotname, "shellyArm_grasp_pose");
// 				float th=20;
// 				go(tgt.x, tgt.z, tgt.ry, tgt.doRotation, graspRef.x(), graspRef.z(), th);
// 				std::cout << "trajectoryrobot2d->go(" << currentTarget.x << ", " << currentTarget.z << ", " << currentTarget.ry << ", " << currentTarget.doRotation << ", " << graspRef.x() << ", " << graspRef.z() << " )\n";
// 				haveTarget = true;
// 			}
// 			catch(const Ice::Exception &ex)
// 			{
// 				std::cout <<"navigationAgent, action_HandObject: ERROR trajectoryrobot2d->go "<< ex << std::endl;
// 				throw ex;
// 			}
// 		}
// 		string state;
// 		try
// 		{
// 			//state = trajectoryrobot2d_proxy->getState().state;
// 		}
// 		catch(const Ice::Exception &ex)
// 		{
// 			std::cout <<"navigationAgent, action_HandObject: trajectoryrobot2d->getState().state "<< ex << std::endl;
// 			throw ex;
// 		}
// 
// 		//state="IDLE";
// 		std::cout<<state<<" haveTarget "<<haveTarget;
// 		if (state=="IDLE" && haveTarget)
// 		{
// 			//std::cout<<"\ttrajectoryrobot2d_proxy->getState() "<<trajectoryrobot2d_proxy->getState().state<<"\n";
// 			try
// 			{
// 				// 				AGMModel::SPtr newModel(new AGMModel(worldModel));
// 				// 				int statusID =symbols["status"]->identifier;
// 				// 				newModel->getEdgeByIdentifiers(objectID, statusID, "noReach").setLabel("reach");
// 				// 				sendModificationProposal(worldModel, newModel);
// 				haveTarget=false;
// 			}
// 			catch (...)
// 			{
// 				std::cout<<"\neeeee"<< "\n";
// 			}
// 		}
// 	}
// 	catch(const Ice::Exception &ex)
// 	{
// 		std::cout << ex << std::endl;
// 	}

// bool SpecificWorker::reloadConfigAgent()
// {
// 	return true;
// }

////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////

void SpecificWorker::structuralChange(const RoboCompAGMWorldModel::World& modification)
{	qDebug()<<"StructuralChange";
	printf("pre <<structuralChange\n");
	QMutexLocker l(mutex);
	printf("<<structuralChange\n");

// 	AGMModelConverter::fromIceToInternal(modification, worldModel);
// 	
// 	//if (roomsPolygons.size()==0 and worldModel->numberOfSymbols()>0)
// 		//roomsPolygons = extractPolygonsFromModel(worldModel);
// 
// 	if (innerModel) delete innerModel;
// 	
// 	innerModel = AGMInner::extractInnerModel(worldModel, "world", false);
}

/**
 *  \brief Called when the robot is sent close to a person to offer the object
 */ 
// void SpecificWorker::action_HandObject_Offer(bool newAction)
// {
// 	// Get symbols' map
// 	std::map<std::string, AGMModelSymbol::SPtr> symbols;
// 	try
// 	{
// 		symbols = worldModel->getSymbolsMap(params/*,  "robot", "room", "object", "status"*/); //ALL THE SYMBOLS GIVEN IN THE RULE
// 
// 	}
// 	catch(...)
// 	{
// 		printf("navigationAgent: Couldn't retrieve action's parameters\n");
// 		printf("<<WORLD\n");
// 		AGMModelPrinter::printWorld(worldModel);
// 		printf("WORLD>>\n");
// 		if (worldModel->size() > 0) { exit(-1); }
// 	}
// 
// 	// Get target
// 	int roomID, personID, robotID;
// 	try
// 	{
// 		if (symbols["room"].get() and symbols["person"].get() and symbols["robot"].get() and symbols["status"].get())
// 		{
// 			roomID = symbols["room"]->identifier;
// 			personID =symbols["person"]->identifier;
// 			robotID = symbols["robot"]->identifier;
// 
// 			try // If we can access the 'reach' edge for the object status the action
// 			{   // is not really necessary. The planner is probably replanning.
// 				worldModel->getEdgeByIdentifiers(personID, symbols["status"]->identifier, "reach");
// 				{
// 					static QTime lastMsg = QTime::currentTime().addSecs(-1000);
// 					if (lastMsg.elapsed() > 1000)
// 					{
// 						rDebug2(("navigationAgent ignoring action setHandObject_Offer (person currently reached)"));
// 						lastMsg = QTime::currentTime();
// 						return;
// 					}
// 					printf("ask the platform to stop\n");
// 					stopL();
// 				}
// 			}
// 			catch(...)
// 			{
// 			}
// 
// 		}
// 		else
// 		{
// 			printf("parameters not in the model yet\n");
// 			return;
// 		}
// 	}
// 	catch(...)
// 	{
// 		printf("ERROR: SYMBOL DOESN'T EXIST \n");
// 		exit(2);
// 	}
// 
// 
// 	// GET THE INNERMODEL NAMES OF TH SYMBOLS
// 	QString robotIMID;
// 	QString roomIMID;
// 	QString personIMID;
// 	try
// 	{
// 		robotIMID = QString::fromStdString(worldModel->getSymbol(robotID)->getAttribute("imName"));
// 		roomIMID = QString::fromStdString(worldModel->getSymbol(roomID)->getAttribute("imName"));
// 		personIMID = QString::fromStdString(worldModel->getSymbol(personID)->getAttribute("imName"));
// 	}
// 	catch(...)
// 	{
// 		printf("ERROR IN GET THE INNERMODEL NAMES\n");
// 		exit(2);
// 	}
// 
// 	// GET THE TARGET POSE: 
// 	RoboCompTrajectoryRobot2D::TargetPose tgt;
// 	try
// 	{
// 		if (not (innerModel->getNode(roomIMID) and innerModel->getNode(personIMID)))    return;
// 		QVec poseInRoom = innerModel->transform6D(roomIMID, personIMID); // FROM PERSON TO ROOM
// 		qDebug() << __FUNCTION__ <<" Target pose: "<< poseInRoom;
// 
// 		tgt.x = poseInRoom.x();
// 		tgt.y = 0;
// 		tgt.z = poseInRoom.z();
// 		tgt.rx = 0;
// 		tgt.ry = poseInRoom.ry();
// 		tgt.rz = 0;
// 		tgt.doRotation = true;
// 	}
// 	catch (...) 
// 	{ 
// 		qDebug()<< __FUNCTION__ << "InnerModel Exception. Element not found in tree";
// 	}
// 
// 	// Execute target
// 	try
// 	{
// 		try
// 		{
// 			QVec O = innerModel->transform("shellyArm_grasp_pose", personIMID);
// 			QVec graspRef = innerModel->transformS(robotname, "shellyArm_grasp_pose");
// 			go(tgt.x, tgt.z, -3.141592, tgt.doRotation, graspRef.x(), graspRef.z(), 20);
// 			qDebug() << __FUNCTION__ << "trajectoryrobot2d->go(" << tgt.x << ", " << tgt.z << ", " << tgt.ry << ", " << graspRef.x() << ", " << graspRef.z() << " )\n";
// 			haveTarget = true;
// 		}
// 		catch(const RoboCompTrajectoryRobot2D::RoboCompException &ex)
// 		{
// 			std::cout << ex << " " << ex.text << std::endl;
// 			throw;
// 		}
// 		catch(const Ice::Exception &ex)
// 		{
// 			std::cout << ex << std::endl;
// 		}
// 	}
// 	catch(const Ice::Exception &ex)
// 	{
// 		std::cout << ex << std::endl;
// 	}
// 
// 
// }
// 
// 
// /**
//  *  \brief Called when the robot is sent close to an object's location
//  */ 
// void SpecificWorker::action_SetObjectReach(bool newAction)
// {	// Get symbols' map
// 	std::map<std::string, AGMModelSymbol::SPtr> symbols;
// 	try
// 	{
// 		symbols = worldModel->getSymbolsMap(params/*,  "robot", "room", "object", "status"*/); //ALL THE SYMBOLS GIVEN IN THE RULE
// 	}
// 	catch(...)
// 	{
// 		printf("navigationAgent: Couldn't retrieve action's parameters\n");
// 		printf("<<WORLD\n");
// 		AGMModelPrinter::printWorld(worldModel);
// 		printf("WORLD>>\n");
// 		if (worldModel->size() > 0) { exit(-1); }
// 	}
// 
// 	// Get target
// 	int roomID, objectID, robotID;
// 	try
// 	{
// 		if (symbols["room"].get() and symbols["object"].get() and symbols["robot"].get() and symbols["status"].get())
// 		{
// 			roomID = symbols["room"]->identifier;
// 			objectID =symbols["object"]->identifier;
// 			robotID = symbols["robot"]->identifier;
// 
// 			try // If we can access the 'reach' edge for the object status the action
// 			{   // is not really necessary. The planner is probably replanning.
// 				worldModel->getEdgeByIdentifiers(objectID, symbols["status"]->identifier, "reach");
// 				{
// 					static QTime lastMsg = QTime::currentTime().addSecs(-1000);
// 					if (lastMsg.elapsed() > 1000)
// 					{
// 						rDebug2(("navigationAgent ignoring action setObjectReach (object currently reached)"));
// 						lastMsg = QTime::currentTime();
// 						return;
// 					}
// 					printf("ask the platform to stop\n");
// 					stopL();
// 				}
// 			}
// 			catch(...)
// 			{
// 			}
// 
// 		}
// 		else
// 		{
// 			printf("parameters not in the model yet\n");
// 			return;
// 		}
// 	}
// 	catch(...)
// 	{
// 		printf("ERROR: SYMBOL DOESN'T EXIST \n");
// 		exit(2);
// 	}
// 
// 	// GET THE INNERMODEL NAMES OF TH SYMBOLS
// 	QString robotIMID;
// 	QString roomIMID;
// 	QString objectIMID;
// 	try
// 	{
// 		robotIMID = QString::fromStdString(worldModel->getSymbol(robotID)->getAttribute("imName"));
// 		roomIMID = QString::fromStdString(worldModel->getSymbol(roomID)->getAttribute("imName"));
// 		objectIMID = QString::fromStdString(worldModel->getSymbol(objectID)->getAttribute("imName"));
// 
// 		// check if object has reachPosition
// 		AGMModelSymbol::SPtr object = worldModel->getSymbol(objectID);
// 		for (auto edge = object->edgesBegin(worldModel); edge != object->edgesEnd(worldModel); edge++)
// 		{
// 			if (edge->getLabel() == "reachPosition")
// 			{
// 				const std::pair<int32_t, int32_t> symbolPair = edge->getSymbolPair();
// 				objectID = symbolPair.second;
// 				objectIMID = QString::fromStdString(worldModel->getSymbol(objectID)->getAttribute("imName"));
// 				qDebug() << __FUNCTION__ << "Target object " << symbolPair.first<<"->"<<symbolPair.second<<" object "<<objectIMID;
// 			}
// 		}
// 	}
// 	catch(...)
// 	{
// 		printf("ERROR IN GET THE INNERMODEL NAMES\n");
// 		exit(2);
// 	}
// 
// 	// GET THE TARGET POSE: 
// 	RoboCompTrajectoryRobot2D::TargetPose tgt;
// 	try
// 	{
// 		if (not (innerModel->getNode(roomIMID) and innerModel->getNode(objectIMID)))    return;
// 		QVec poseInRoom = innerModel->transform6D(roomIMID, objectIMID); // FROM OBJECT TO ROOM
// 		qDebug() << __FUNCTION__ <<" Target pose: "<< poseInRoom;
// 
// 		tgt.x = poseInRoom.x();
// 		tgt.y = 0;
// 		tgt.z = poseInRoom.z();
// 		tgt.rx = 0;
// 		tgt.ry = poseInRoom.ry();
// 		tgt.rz = 0;
// 		tgt.doRotation = true;
// 	}
// 	catch (...) 
// 	{ 
// 		qDebug()<< __FUNCTION__ << "InnerModel Exception. Element not found in tree";
// 	}
// 
// 	// Execute target
// 	try
// 	{
// 		// 		if (!haveTarget)
// 		{
// 			try
// 			{
// 				QVec O = innerModel->transform("shellyArm_grasp_pose", objectIMID);
// 				//O.print("	O pose relativa");
// 				//qDebug() << __FUNCTION__ << "O norm:" << O.norm2();
// 				QVec graspRef = innerModel->transformS(robotname, "shellyArm_grasp_pose");
// 				float th=20;
// 				go(tgt.x, tgt.z, tgt.ry, tgt.doRotation, graspRef.x(), graspRef.z(), th);
// 				qDebug() << __FUNCTION__ << "trajectoryrobot2d->go(" << tgt.x << ", " << tgt.z << ", " << tgt.ry << ", " << graspRef.x() << ", " << graspRef.z() << " )\n";
// 				haveTarget = true;
// 			}
// 			catch(const RoboCompTrajectoryRobot2D::RoboCompException &ex)
// 			{
// 				std::cout << ex << " " << ex.text << std::endl;
// 				throw;
// 			}
// 			catch(const Ice::Exception &ex)
// 			{
// 				std::cout << ex << std::endl;
// 			}
// 		}
// 		string state;
// 		try
// 		{
// 			//state = trajectoryrobot2d_proxy->getState().state;
// 		}
// 		catch(const Ice::Exception &ex)
// 		{
// 			std::cout <<"trajectoryrobot2d->getState().state "<< ex << std::endl;
// 			throw ex;
// 		}
// 
// 		//state="IDLE";
// 		std::cout<<state<<" haveTarget "<<haveTarget;
// 		if (state=="IDLE" && haveTarget)
// 		{
// 			//std::cout<<"\ttrajectoryrobot2d_proxy->getState() "<<trajectoryrobot2d_proxy->getState().state<<"\n";
// 			try
// 			{
// 				// 				AGMModel::SPtr newModel(new AGMModel(worldModel));
// 				// 				int statusID =symbols["status"]->identifier;
// 				// 				newModel->getEdgeByIdentifiers(objectID, statusID, "noReach").setLabel("reach");
// 				// 				sendModificationProposal(worldModel, newModel);
// 				haveTarget=false;
// 			}
// 			catch (...)
// 			{
// 				std::cout<<"\neeeee"<< "\n";
// 			}
// 		}
// 	}
// 	catch(const Ice::Exception &ex)
// 	{
// 		std::cout << ex << std::endl;
// 	}
// }
// 
// void SpecificWorker::manageReachedPose()
// {
// 	float schmittTriggerThreshold = 20;
// 	float THRESHOLD_POSE = 100;
// 	std::string m ="  ";
// 
// 	bool changed = false;
// 
// 	QMutexLocker locker(mutex);
// 
// 	AGMModel::SPtr newModel(new AGMModel(worldModel));
// 
// 	for (AGMModel::iterator symbol_itr=newModel->begin(); symbol_itr!=newModel->end(); symbol_itr++)
// 	{
// 		AGMModelSymbol::SPtr node = *symbol_itr;
// 		if (node->symboltype() == "pose")
// 		{
// 			/// Compute distance and new state
// 			float d2n;
// 			try
// 			{
// 				QVec arm = innerModel->transformS("world", "robot");
// 				QVec obj = innerModel->transformS("world", node->getAttribute("imName"));
// 				(arm-obj).print("error");
// 
// 				d2n = distanceToNode("robot", newModel, node);
// 				qDebug()<<"distance "<<d2n;
// 			}
// 			catch(...)
// 			{
// 				printf("Ref: robot: %p\n", (void *)innerModel->getNode("robot"));
// 				printf("Pose: %s: %p\n", node->getAttribute("imName").c_str(), (void *)innerModel->getNode(node->getAttribute("imName").c_str()));
// 				exit(1);
// 			}
// 
// 
// 			for (AGMModelSymbol::iterator edge_itr=node->edgesBegin(newModel); edge_itr!=node->edgesEnd(newModel); edge_itr++)
// 			{
// 				AGMModelEdge &edge = *edge_itr;
// 				if (edge->getLabel() == "reach" and d2n > THRESHOLD_POSE+schmittTriggerThreshold )
// 				{
// 					edge->setLabel("noReach");
// 					printf("pose %d STOPS REACH\n", node->identifier);
// 					m += " action " + action + " edge->toString() "+ edge->toString(newModel);
// 					changed = true;
// 					rDebug2(("pose %d no-reach") % node->identifier);
// 				}
// 				else if (edge->getLabel() == "noReach" and d2n < THRESHOLD_POSE/*-schmittTriggerThreshold*/)
// 				{
// 					edge->setLabel("reach");
// 					printf("___ %s ___\n", edge->getLabel().c_str());
// 					printf("pose %d STARTS REACH\n", node->identifier);
// 					m += " action " + action + " edge->toString() "+ edge->toString(newModel);
// 					changed = true;
// 					rDebug2(("pose %d reach") % node->identifier);
// 				}
// 			}
// 		}
// 	}
// 
// 	/// Publish new model if changed
// 	if (changed)
// 	{
// 		printf("PUBLISH!!!! version%d\n", newModel->version);
// 		sendModificationProposal(newModel, worldModel, m);
// 	}
// }
// 

// float SpecificWorker::distanceToNode(std::string reference_name, AGMModel::SPtr model, AGMModelSymbol::SPtr object)
// {
// 	QMutexLocker locker(mutex);
// 
// 	QVec arm = innerModel->transformS("world", reference_name);
// 	arm(1) = 0;
// 	QVec obj = innerModel->transformS("world", object->getAttribute("imName"));
// 	obj(1) = 0;
// 	return (arm-obj).norm2();
// 
// }
// 
// int32_t SpecificWorker::getIdentifierOfRobotsLocation(AGMModel::SPtr &worldModel)
// {
// 	for (AGMModel::iterator symbol_it=worldModel->begin(); symbol_it!=worldModel->end(); symbol_it++)
// 	{
// 		const AGMModelSymbol::SPtr &symbol = *symbol_it;
// 		if (symbol->symbolType == "robot")
// 		{
// 			for (AGMModelSymbol::iterator edge_it=symbol->edgesBegin(worldModel); edge_it!=symbol->edgesEnd(worldModel); edge_it++)
// 			{
// 				AGMModelEdge edge = *edge_it;
// 				if (edge.linking == "in")
// 				{
// 					return edge.symbolPair.second;
// 				}
// 			}
// 		}
// 	}
// 
// 	printf("wheres's the robot?\n");
// 	return -1;
// }
// 
// void SpecificWorker::setIdentifierOfRobotsLocation(AGMModel::SPtr &model, int32_t identifier)
// {
// 	bool didSomethin = false;
// 	for (AGMModel::iterator symbol_it=worldModel->begin(); symbol_it!=worldModel->end(); symbol_it++)
// 	{
// 		const AGMModelSymbol::SPtr &symbol = *symbol_it;
// 		if (symbol->symbolType == "robot")
// 		{
// 
// 			for (AGMModelSymbol::iterator edge_it=symbol->edgesBegin(worldModel); edge_it!=symbol->edgesEnd(worldModel); edge_it++)
// 			{
// 				if (edge_it->linking == "in")
// 				{
// 					printf("it was %d\n", edge_it->symbolPair.second);
// 				}
// 			}
// 			for (int32_t edgeIndex=0; edgeIndex<model->numberOfEdges(); edgeIndex++)
// 			{
// 				if (model->edges[edgeIndex].linking == "in")
// 				{
// 					if (model->edges[edgeIndex].symbolPair.first == symbol->identifier)
// 					{
// 						model->edges[edgeIndex].symbolPair.second = identifier;
// 						didSomethin = true;
// 					}
// 				}
// 			}
// 			for (AGMModelSymbol::iterator edge_it=symbol->edgesBegin(worldModel); edge_it!=symbol->edgesEnd(worldModel); edge_it++)
// 			{
// 				if (edge_it->linking == "in")
// 				{
// 					printf("now is %d\n", edge_it->symbolPair.second);
// 				}
// 			}
// 		}
// 	}
// 	if (not didSomethin)
// 		qFatal("couldn't update robot's room in the cog graph");
// }
// 
// void SpecificWorker::action_ChangeRoom(bool newAction)
// {
// 	static float lastX = std::numeric_limits<float>::quiet_NaN();
// 	static float lastZ = std::numeric_limits<float>::quiet_NaN();
// 
// 	auto symbols = worldModel->getSymbolsMap(params, "r2", "robot");
// 
// 
// 	try
// 	{
// 		printf("trying to get _in_ edge from %d to %d\n", symbols["robot"]->identifier, symbols["r2"]->identifier);
// 		AGMModelEdge e = worldModel->getEdge(symbols["robot"], symbols["r2"], "in");
// 		printf("STOP! WE ALREADY GOT THERE!\n");
// 		stopL();
// 		return;
// 	}
// 	catch(...)
// 	{
// 	}
// 
// 	int32_t roomId = symbols["r2"]->identifier;
// 	printf("room symbol: %d\n",  roomId);
// 	std::string imName = symbols["r2"]->getAttribute("imName");
// 	printf("imName: <%s>\n", imName.c_str());
// 
// 	const float refX = str2float(symbols["r2"]->getAttribute("x"));
// 	const float refZ = str2float(symbols["r2"]->getAttribute("z"));
// 
// 	QVec roomPose = innerModel->transformS("world", QVec::vec3(refX, 0, refZ), imName);
// 	roomPose.print("goal pose");
// 	// 	AGMModelSymbol::SPtr goalRoom = worldModel->getSymbol(str2int(params["r2"].value));
// 	// 	const float x = str2float(goalRoom->getAttribute("tx"));
// 	// 	const float z = str2float(goalRoom->getAttribute("tz"));
// 	const float x = roomPose(0);
// 	const float z = roomPose(2);
// 
// 	bool proceed = true;
// 	if ( (planningState.state=="PLANNING" or planningState.state=="EXECUTING") )
// 	{
// 		if (abs(lastX-x)<10 and abs(lastZ-z)<10)
// 			proceed = false;
// 		else
// 			printf("proceed because the coordinates differ (%f, %f), (%f, %f)\n", x, z, lastX, lastZ);
// 	}
// 	else
// 	{
// 		printf("proceed because it's stoped\n");
// 	}
// 
// 	if (proceed)
// 	{
// 		lastX = x;
// 		lastZ = z;
// 		printf("changeroom to %d\n", symbols["r2"]->identifier);
// 		go(x, z, 0, false, 0, 0, 1200);
// 	}
// 	else
// 	{
// 	}
// }
// 
// void SpecificWorker::action_Stop(bool newAction)
// {
// 	stopL();
// }
// 
// void SpecificWorker::action_ReachPose(bool newAction)
// {
// 	printf("action_ReachPose,%d: %d\n", __LINE__, newAction);
// 	static float lastX = std::numeric_limits<float>::quiet_NaN();
// 	printf("action_ReachPose,%d: %d\n", __LINE__, newAction);
// 	static float lastZ = std::numeric_limits<float>::quiet_NaN();
// 	printf("action_ReachPose,%d: %d\n", __LINE__, newAction);
// 
// 	auto symbols = worldModel->getSymbolsMap(params, "room", "pose");
// 	printf("action_ReachPose,%d: %d\n", __LINE__, newAction);
// 
// 	int32_t poseId = symbols["pose"]->identifier;
// 	printf("pose symbol: %d\n",  poseId);
// 	std::string imName = symbols["pose"]->getAttribute("imName");
// 	printf("imName: <%s>\n", imName.c_str());
// 
// 	QVec pose = innerModel->transform6D("world", QString::fromStdString(imName));
// 	pose.print("goal pose");
// 	const float x = pose(0);
// 	const float z = pose(2);
// 	const float ry = pose(4);
// 
// 	bool proceed = true;
// 	if ( (planningState.state=="PLANNING" or planningState.state=="EXECUTING") )
// 	{
// 		if (abs(lastX-x)<10 and abs(lastZ-z)<10)
// 			proceed = false;
// 		else
// 			printf("proceed because the coordinates differ (%f, %f), (%f, %f)\n", x, z, lastX, lastZ);
// 	}
// 	else
// 	{
// 		printf("proceed because it's stoped\n");
// 	}
// 
// 	if (proceed)
// 	{
// 		lastX = x;
// 		lastZ = z;
// 		printf("setpose %d\n", symbols["room"]->identifier);
// 		go(x, z, ry, true);
// 	}
// 	else
// 	{
// 	}
// }
// 
// void SpecificWorker::action_FindObjectVisuallyInTable(bool newAction)
// {
// 	// 	stopL();
// 
// 
// 	static float lastX = std::numeric_limits<float>::quiet_NaN();
// 	static float lastZ = std::numeric_limits<float>::quiet_NaN();
// 
// 	AGMModelSymbol::SPtr goalTable;
// 	AGMModelSymbol::SPtr robot;
// 	int32_t tableId = -1;
// 	try
// 	{
// 		tableId = str2int(params["container"].value);
// 		goalTable = worldModel->getSymbol(tableId);
// 		robot = worldModel->getSymbol(worldModel->getIdentifierByType("robot"));
// 	}
// 	catch(...)
// 	{
// 		printf("can't access robot or table\n");
// 		return;
// 	}
// 
// 	const float x = str2float(goalTable->getAttribute("tx"));
// 	const float z = str2float(goalTable->getAttribute("tz"));
// 	float alpha = tableId==7?-3.141592:0;
// 	// printf("%s: %d\n", __FILE__, __LINE__);
// 
// 	const float rx = str2float(robot->getAttribute("tx"));
// 	const float rz = str2float(robot->getAttribute("tz"));
// 	const float ralpha = str2float(robot->getAttribute("alpha"));
// 	// printf("%s: %d\n", __FILE__, __LINE__);
// 
// 	// Avoid repeating the same goal and confuse the navigator
// 	const float errX = abs(rx-x);
// 	const float errZ = abs(rz-z);
// 	float errAlpha = abs(ralpha-alpha);
// 	while (errAlpha > +M_PIl) errAlpha -= 2.*M_PIl;
// 	while (errAlpha < -M_PIl) errAlpha += 2.*M_PIl;
// 	errAlpha = abs(errAlpha);
// 	if (errX<20 and errZ<20 and errAlpha<0.02)
// 		return;
// 	// printf("%s: %d\n", __FILE__, __LINE__);
// 
// 	bool proceed = true;
// 	if ( (planningState.state=="IDLE")) //PLANNING" or planningState.state=="EXECUTING") )
// 	{
// 		if (abs(lastX-x)<10 and abs(lastZ-z)<10)
// 			proceed = false;
// 		else
// 			printf("proceed because the coordinates differ (%f, %f), (%f, %f)\n", x, z, lastX, lastZ);
// 	}
// 	else
// 	{
// 		printf("proceed because it's stoped\n");
// 	}
// 
// 	if (proceed)
// 	{
// 		lastX = x;
// 		lastZ = z;
// 		printf("find objects in table %d\n", tableId);
// 		go(x, tableId==7?z+550:z-550, tableId==7?-3.141592:0, true, 0, 500, 500);
// 	}
// 	else
// 	{
// 	}
// }
// 
// void SpecificWorker::action_NoAction(bool newAction)
// {
// 	try{
// 		std::string state;
// 		//state = trajectoryrobot2d_proxy->getState().state;
// 		if (state != "IDLE")
// 		{
// 			printf("trajectoryrobot2d state : %s\n", state.c_str());
// 			stopL();
// 		}
// 	}catch(...)
// 	{
// 		std::cout<< "Error retrieving trajectory state"<<std::endl;
// 	}
// }
// 

// Send target to trajectory
// void SpecificWorker::go(float x, float z, float alpha, bool rot, float xRef, float zRef, float threshold)
// {
// 	static bool first = true;
// 
// 	RoboCompTrajectoryRobot2D::TargetPose lastTarget = currentTarget;
// 
// 	currentTarget.x = x;
// 	currentTarget.z = z;
// 	currentTarget.y = 0;
// 	currentTarget.rx = 0;
// 	currentTarget.rz = 0;
// 
// 	if (rot)
// 	{
// 		currentTarget.ry = alpha;
// 		currentTarget.doRotation = true;
// 	}
// 	else
// 	{
// 		currentTarget.ry = 0;
// 		currentTarget.doRotation = false;
// 	}
// 
// 
// 	if ((not first) and (lastTarget == currentTarget))
// 		return;
// 
// 	try
// 	{
// 		std::cout<< "ENVIANDO A trajectoryrobot2d->go(" << currentTarget.x << ", " << currentTarget.z << ", " << currentTarget.ry << ", " << xRef << ", " << zRef << threshold << " )" <<std::endl;
// 		//trajectoryrobot2d_proxy->goReferenced(currentTarget, xRef, zRef, threshold);
// 	}
// 	catch(const RoboCompTrajectoryRobot2D::RoboCompException &ex)
// 	{
// 		std::cout << ex.text << std::endl;
// 	}
// 	catch(const Ice::Exception &ex)
// 	{
// 		std::cout << ex << std::endl;
// 	}
// 	catch(...)
// 	{
// 		printf("Exception: something else %d\n", __LINE__);
// 	}
// }
// 
// 
// void SpecificWorker::stopL()
// {
// 	try
// 	{
// 		//trajectoryrobot2d_proxy->stop();
// 	}
// 	catch(const Ice::Exception &ex)
// 	{
// 		std::cout << ex << std::endl;
// 	}
// 	catch(...)
// 	{
// 		printf("Exception: something else %d\n", __LINE__);
// 	}
// }
// 
// 
// 

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

// void SpecificWorker::structuralChange(const RoboCompAGMWorldModel::World& modification)
// {
//	static bool firsttime=true;
	
// 	if(firsttime)
// 	{
// 		qDebug()<<"StructuralChange";
// 		QMutexLocker l(mutex);
// 		
// 		AGMModelConverter::fromIceToInternal(modification, worldModel);
// 
// 		//if (roomsPolygons.size()==0 and worldModel->numberOfSymbols()>0)
// 		//roomsPolygons = extractPolygonsFromModel(worldModel);
// 
// 		if (innerModel) 
// 			delete innerModel;
// 
// 		innerModel = AGMInner::extractInnerModel(worldModel, "world", false);
// 		pathfinder.setInnerModel(innerModel);
// 		updateRobotPosition();
// 		changepos=true;
// 		printf("structuralChange>>\n");
// 		firsttime = false;
// 	}
// 	changepos=true;
// 	
// 	printf("structuralChange>>\n");
// 	
// }

void SpecificWorker::symbolUpdated(const RoboCompAGMWorldModel::Node& modification)
{
	qDebug()<<"symbolUpdated";
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
{	
	//qDebug()<<"edgesUpdated";
// 	changepos=true;
// 	QMutexLocker lockIM(mutex);
// 
// 	for (auto modification : modifications)
// 	{
// 		AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
// 		AGMModelEdge dst;
// 		AGMModelConverter::fromIceToInternal(modification,dst);
// 		AGMInner::updateImNodeFromEdge(worldModel, dst, innerModel);
// 		updateRobotPosition();
// 	}
}

/**
 * \brief ACTUALIZACION DEL ENLACE EN INNERMODEL
 */ 
void SpecificWorker::edgeUpdated(const RoboCompAGMWorldModel::Edge& modification)
{	
// 	changepos=true;
// 
// 	qDebug() << "edgeUpdated";
// 	QMutexLocker lockIM(mutex);
// 	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
// 	AGMModelEdge dst;
// 	AGMModelConverter::fromIceToInternal(modification,dst);
// 	AGMInner::updateImNodeFromEdge(worldModel, dst, innerModel);
// 	updateRobotPosition();
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

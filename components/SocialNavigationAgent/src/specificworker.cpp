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
	qDebug() << __FUNCTION__ ;
	active = false;
	active = false;	
	worldModel = AGMModel::SPtr(new AGMModel());
	worldModel->name = "worldModel";
	haveTarget = false;
	innerModel = std::make_shared<InnerModel>();
	
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

	proximidad->QSlider::setMinimum (10);
	proximidad->QSlider::setMaximum (90);	
	proximidad->QSlider::setTracking (false);	
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
	
	#ifdef USE_QTGUI
		viewer = std::make_shared<InnerViewer>(innerModel, "Social Navigation");  //InnerViewer copies internally innerModel so it has to be resynchronized
		//viewer->start();	
	#endif
	
	try
	{		
		RoboCompAGMWorldModel::World w = agmexecutive_proxy->getModel();
		structuralChange(w);
		rDebug2(("Leaving Structural Change"));
	}		
	catch(...)
	{	rDebug2(("The executive is probably not running, waiting for first AGM model publication...")); }
		
	//innerModel = std::make_shared<InnerModel>("/home/robocomp/robocomp/components/robocomp-araceli/etcSim/simulation.xml");
	

//	innerModel->getNode<InnerModelJoint>("armX1")->setAngle(-1);
//	innerModel->getNode<InnerModelJoint>("armX2")->setAngle(2.5);
	

	std::shared_ptr<RoboCompCommonBehavior::ParameterList> configparams = std::make_shared<RoboCompCommonBehavior::ParameterList>(paramsL);

	// Initializing PathFinder
	pathfinder.initialize(innerModel,viewer, configparams, laser_proxy, omnirobot_proxy);

	// releasing pathfinder
	//thread_pathfinder = std::thread(&robocomp::pathfinder::PathFinder::run, &pathfinder);
	//rDebug2(("Pathfinder up and running"));


	qLog::getInstance()->setProxy("both", logger_proxy);
	rDebug2(("NavigationAgent started"));

	Period = 200;
	timer.start(Period);
	
	//Proxies for actionExecution
	//aE.logger_proxy = logger_proxy;
	//aE.agmexecutive_proxy = agmexecutive_proxy;
	//aE.omnirobot_proxy = omnirobot_proxy;
	//aE.trajectoryn2d_proxy = trajectoryrobot2d_proxy;
	
	//Proxies for SocialRules
	sr.socialnavigationgaussian_proxy = socialnavigationgaussian_proxy;
	sr.agmexecutive_proxy = agmexecutive_proxy;
	sr.mux = mutex;
	sr.objectInteraction(false);
	
	rDebug2(("Leaving setParams"));

	checkNewPersonInModel();
	return true;
}

/**
 * \brief Check if persons are included in the AGM. 
 * Then, their pose is stored.
 * Everytime a person has moved, its position is updated
 */
void SpecificWorker::compute()
{
		
 	static bool first=true;
 	if (first)
 	{	
 		qLog::getInstance()->setProxy("both", logger_proxy);
 		rDebug2(("navigationAgent started"));
 		first = false;
 	}

	// PROVISIONAL read robot position from proxy
	try 
	{
		omnirobot_proxy->getBaseState(bState);
		innerModel->updateTransformValues("robot", bState.x,0,bState.z,0,bState.alpha,0);
	//	qDebug() << "SpecificWorker::compute" << bState.x << bState.z << bState.alpha;
	}
	catch(const Ice::Exception &ex)
	{
		printf("The executive is probably not running, waiting for first AGM model publication...");
		std::cout << ex << std::endl;
	}
	
	pathfinder.run();

	//update viewer
	QVec robotpos = innerModel->transformS6D("world", robotname);
	viewer->ts_updateTransformValues(QString::fromStdString(robotname), robotpos);
	viewer->run();
	
		
// 	qDebug()<<"Update actionEx";
// 	aE.Update(action,params);
// 	
	
	if (changepos)
	{
		checkMovement();
		checkRobotmov();
	
		changepos = false;
	}
	first = false;
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


void SpecificWorker::checkRobotmov()
{
	robotSymbolId = worldModel->getIdentifierByType("robot");
	AGMModelSymbol::SPtr robotparent = worldModel->getParentByLink(robotSymbolId, "RT");
	AGMModelEdge &edgeRTrobot  = worldModel->getEdgeByIdentifiers(robotparent->identifier, robotSymbolId, "RT");
		
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
}


void SpecificWorker::checkNewPersonInModel()
{	
	pSymbolId.clear();
	
	//Check if the person is in the model
 	for (uint i=0; i < 100; i++)
	{
		std::string type = "person" + std::to_string(i+1);
		std::string name = "fakeperson" + std::to_string(i+1);
		int idx = 0;
		while ((personSymbolId = worldModel->getIdentifierByType(type, idx++)) != -1)
		{
			if (worldModel->getSymbolByIdentifier(personSymbolId)->getAttribute("imName") == name)
			{
				pSymbolId.push_back(personSymbolId);
				std::cout<<"Person found "<<type<<" "<<name<<" "<<personSymbolId<<std::endl;
				break;
			}
		}
			
	}

	checkMovement();
}

void SpecificWorker::checkMovement()
{
	AGMModel::SPtr newM(new AGMModel(worldModel));
	bool sendChangesAGM = false;
	
	totalpersons.clear();
 	
	if (!pSymbolId.empty())
	{
		for (auto id: pSymbolId)
		{
			
			AGMModelSymbol::SPtr personParent = newM->getParentByLink(id, "RT");
			AGMModelEdge &edgeRT = newM->getEdgeByIdentifiers(personParent->identifier, id, "RT");
				
			person.x = str2float(edgeRT.attributes["tx"])/1000;
			person.z = str2float(edgeRT.attributes["tz"])/1000;
			person.angle = str2float(edgeRT.attributes["ry"]);
			//person.vel=str2float(edgeRT.attributes["velocity"]);
			person.vel = 0;
			totalpersons.push_back(person);
				
		/*	qDebug() <<"PERSONA " <<ind+1  <<" Coordenada x"<< person.x << "Coordenada z"<< person.z << Rotacion "<< person.angle;
			*/		
					
			/////////////////////checking if the person is looking at the robot /////////////////////////
	// 		try
	// 		{	
	// 			qDebug()<<"------------------------------------------------";
	// 			if (sr.checkHRI(person,ind+1,innerModel.get(),newM) == true)
	// 			{	
	// 				qDebug()<<"SEND MODIFICATION PROPOSAL";
	//					sendChangesAGM = true;
	// 				
	// 			}
	// 			else 
	// 				qDebug()<<"NO HAY MODIFICACION";
	// 		}
	//		catch(...)
	// 		{	
	// 		}
			
		}
	}
 	
		
	try
	{
		qDebug()<<"NUMERO DE PERSONAS "<< totalpersons.size();
		SNGPolylineSeq list = sr.ApplySocialRules(totalpersons);
		sequence.clear();
		sequence = list;
		UpdateInnerModel(list);
	}
		
	catch( const Ice::Exception &e)
	{ 
//		std::cout << e << std::endl;
	}

	if (sendChangesAGM)
	{	
		try
		{
			sendModificationProposal(newM,worldModel,"-");
		}
		catch(...){}
	}
}


/**
 * \brief The innerModel is extracted from the AGM and the polylines are inserted on it as a set of planes.
 */

void SpecificWorker::UpdateInnerModel(SNGPolylineSeq seq)
{
	QMutexLocker locker(mutex);
	qDebug() << "----------------------"<< __FUNCTION__ << "----------------------";
	
	// Extract innerModel
	InnerModel *inner  = AGMInner::extractInnerModel(worldModel, "world", false); 

	int count = 0;

	for (auto s:seq)
	{
		auto previousPoint = s[s.size()-1];
		for (auto currentPoint:s)
		{
			QString name = QString("polyline_obs_")+QString::number(count,10);
			qDebug() << __FUNCTION__ << "nombre"<<name;
			QVec ppoint = QVec::vec3(previousPoint.x*1000, 1000, previousPoint.z*1000);
			QVec cpoint = QVec::vec3(currentPoint.x*1000, 1000, currentPoint.z*1000);
			QVec center = (cpoint + ppoint).operator*(0.5);

			QVec normal = (cpoint-ppoint);
			float dist = normal.norm2();	
			float temp = normal(2);
			normal(2) = normal(0);
			normal(0) = -temp;

			if (inner->getNode(name))
			{
				try
				{
					inner->removeNode(name);
				}

				catch(QString es){ qDebug() << "EXCEPCION" << es;}
			}

			InnerModelNode *parent = inner->getNode(QString("world"));
			if (parent == NULL)
				printf("%s: parent does not exist\n", __FUNCTION__);
			else
			{			
				InnerModelPlane *plane;
				try
				{
					plane  = inner-> newPlane(name, parent, QString("#FFFF00"), dist, 2000, 90, 1, normal(0), normal(1), normal(2), center(0), center(1), center(2), true);
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
	
	innerModel.reset(inner);
	pathfinder.innerModelChanged(innerModel);	
	viewer->reloadInnerModel(innerModel);


}

// *****************************************************************************************
// AGENT RELATED
// *****************************************************************************************

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
{
	qDebug()<<"StructuralChange";
	QMutexLocker l(mutex);
	
    AGMModelConverter::fromIceToInternal(modification, worldModel);

	InnerModel *inner = AGMInner::extractInnerModel(worldModel, "world", false);
	innerModel.reset(inner);
	pathfinder.innerModelChanged(innerModel);
	
	//reload viewer
	viewer->reloadInnerModel(innerModel);
	//check if structural change include new Person
	checkNewPersonInModel();
	printf("FIN structuralChange>>\n");
}


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
// 	qDebug()<<"edgesUpdated";
	changepos=true;
	QMutexLocker lockIM(mutex);
	for (auto modification : modifications)
	{
		AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
		AGMModelEdge edge;
		AGMModelConverter::fromIceToInternal(modification, edge);
//TODO Guardar si el cambio se refiere a una persona?¿
		//Update InnerModel values
		if (edge->getLabel()=="RT" )
		{
			try{
				std::string songName= (worldModel->getSymbol( edge->getSymbolPair().second) )->getAttribute("imName");
				
				QVec vec = QVec::vec6();
				vec[0] = str2float(edge->getAttribute("tx"));
				vec[1] = str2float(edge->getAttribute("ty"));
				vec[2] = str2float(edge->getAttribute("tz"));
				vec[3] = str2float(edge->getAttribute("rx"));
				vec[4] = str2float(edge->getAttribute("ry"));
				vec[5] = str2float(edge->getAttribute("rz"));
				innerModel->updateTransformValues(QString::fromStdString(songName), vec);
				viewer->ts_updateTransformValues(QString::fromStdString(songName), vec);
			}
			catch (...)
			{
				qDebug()<<"EXCEPTION,RT label connect to a symbol without imName\n";
				std::cout<<(worldModel->getSymbol( edge->getSymbolPair().second))->toString(true);
			}
		}
	}
}

/**
 * \brief ACTUALIZACION DEL ENLACE EN INNERMODEL
 */ 
void SpecificWorker::edgeUpdated(const RoboCompAGMWorldModel::Edge& modification)
{
// 	qDebug() << "edgeUpdated";
	changepos=true;
	QMutexLocker lockIM(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
	AGMModelEdge edge;
	AGMModelConverter::fromIceToInternal(modification, edge);
//TODO Guardar si el cambio se refiere a una persona?¿
	//Update InnerModel values
	if (edge->getLabel()=="RT" )
	{
		try{
			std::string songName= (worldModel->getSymbol( edge->getSymbolPair().second) )->getAttribute("imName");
			
			QVec vec = QVec::vec6();
			vec[0] = str2float(edge->getAttribute("tx"));
			vec[1] = str2float(edge->getAttribute("ty"));
			vec[2] = str2float(edge->getAttribute("tz"));
			vec[3] = str2float(edge->getAttribute("rx"));
			vec[4] = str2float(edge->getAttribute("ry"));
			vec[5] = str2float(edge->getAttribute("rz"));
			innerModel->updateTransformValues(QString::fromStdString(songName), vec);
			viewer->ts_updateTransformValues(QString::fromStdString(songName), vec);
		}
		catch (...)
		{
			qDebug()<<"EXCEPTION,RT label connect to a symbol without imName\n";
			std::cout<<(worldModel->getSymbol( edge->getSymbolPair().second))->toString(true);
		}
	}
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

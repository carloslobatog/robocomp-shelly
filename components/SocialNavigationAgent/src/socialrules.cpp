
#include "socialrules.h"


void SocialRules::initialize(SocialNavigationGaussianPrx socialnavigationgaussian_proxy_,AGMExecutivePrx agmexecutive_proxy_,QMutex *mutex_,robocomp::pathfinder::PathFinder *pathfinder_,AGMModel::SPtr worldModel_, InnerPtr innerModel_)
{
	socialnavigationgaussian_proxy = socialnavigationgaussian_proxy_;
	agmexecutive_proxy = agmexecutive_proxy_;
	mux = mutex_;
	pathfinder = pathfinder_;
	innerModel = innerModel_;
	worldModel = worldModel_;
	
	objectInteraction(false);
}


/**
* \brief Change the h value
*/
void SocialRules::change_hvalue(float value)
{
	h = value;
	qDebug()<<"h = "<<value;
	
}
/**
* \brief If the person is in the model it is added to a vector of persons wich is sent to the socialnavigationGaussian
* to model its personal space. 
* The function returns a sequence of polylines.
*/

SNGPolylineSeq SocialRules::calculateGauss(bool draw)
{
	qDebug()<<__FUNCTION__;
	if (!quietperson.empty())
	{  	
		sequence.clear();
		sequence = socialnavigationgaussian_proxy-> getPersonalSpace(quietperson,h, draw);
	}
	return sequence;
}


SNGPolylineSeq SocialRules::PassOnRight(bool draw)
{
		
	qDebug()<<__FUNCTION__;
	if (!movperson.empty())
	{
		sequence2.clear();
		sequence2 = socialnavigationgaussian_proxy-> getPassOnRight(movperson, h, draw);
	}	    
	return sequence2;

	
}
 
SNGPolylineSeq SocialRules::objectInteraction(bool d)
{
	qDebug()<<__FUNCTION__;
	
	RoboCompAGMWorldModel::World w = agmexecutive_proxy->getModel();
	AGMModelConverter::fromIceToInternal(w, worldModel);
	
	objects.clear();
	sequenceObj.clear();
	try
	{
		int idx=0;
		while ((objectSymbolId = worldModel->getIdentifierByType("object_interaction", idx++)) != -1)
		{	
			
			AGMModelSymbol::SPtr objectP = worldModel->getParentByLink(objectSymbolId, "RT");
			AGMModelEdge &edgeRT  = worldModel->getEdgeByIdentifiers(objectP->identifier,objectSymbolId, "RT");
			SNGObject object;
			object.x = str2float(edgeRT.attributes["tx"])/1000;
			object.z = str2float(edgeRT.attributes["tz"])/1000;
			object.angle=str2float(edgeRT.attributes["ry"]);
			object.space=str2float(worldModel->getSymbolByIdentifier(objectSymbolId)->getAttribute("interaction"));
		
			objects.push_back(object);
			
			qDebug()<<"Object"<<"Pose x"<<object.x<<"Pose z"<<object.z<<"Angle"<<object.angle<<"Space"<<object.space;
		}
		
		sequenceObj = socialnavigationgaussian_proxy->getObjectInteraction(totalperson,objects,d);
	}
	catch(...){}
	
	return sequenceObj;
	
}

void SocialRules::goToPerson()
{
	qDebug()<<__FUNCTION__;
	
	if (personSymbolId.size() == 1)
	{
		auto id = personSymbolId[0];
		
		AGMModelSymbol::SPtr personParent = worldModel->getParentByLink(id, "RT");
		AGMModelEdge &edgeRT = worldModel->getEdgeByIdentifiers(personParent->identifier, id, "RT");
		
		person.x = str2float(edgeRT.attributes["tx"]);
		person.z = str2float(edgeRT.attributes["tz"]);
		person.angle = str2float(edgeRT.attributes["ry"]);
		//person.vel=str2float(edgeRT.attributes["velocity"]);
		person.vel = 0;
		
		qDebug()<<"PERSONA"<< person.x << " "<< person.z; 
	
		pathfinder->go(person.x+1000, person.z + 1500);
	}
	
	else
		qDebug()<<"Más de una persona en el modelo";
	
	
	
	
// 	for (auto p:totalperson)
// 		qDebug()<<"PERSONA" << p.x <<" " <<p.z<<" "<<p.angle;
// 		
// 	if (totalperson.size() != 1)
// 		qDebug()<<"Más de una persona en el modelo";
// 	else
// 		
		
		
	
}


bool SocialRules::checkHRI(SNGPerson p, int ind,InnerPtr i, AGMModel::SPtr w)
{	
	worldModel = w;
	
	bool changes = false;
	/////////////////////Checking if the person is close and looking at the robot
	std::string type = "person" + std::to_string(ind);
	std::string name = "fakeperson" + std::to_string(ind);
		
	qDebug()<<QString::fromStdString(type)<<"-"<<QString::fromStdString(name);

	bool looking = false;
	bool close = false;


	mux->lock();
	QVec pose = i->transform(QString::fromStdString(name),"robot");
	mux->unlock();

	float dist = sqrt(pose.x()*pose.x()+pose.z()*pose.z());
	float angle = atan2(pose.x(),pose.z());
	
	qDebug()<<"pose x"<<pose.x()<<"pose z"<<pose.z();
	qDebug()<<"dist"<<dist<<"angle"<<abs(angle/0.0175);
	
	if (abs(angle)<20*0.0175)
	{
		qDebug()<<"LOOOKING";
		looking = true;
	}

	
	if (dist<2000.0)
	{
		qDebug()<<"CLOOOOSE";
		close = true;
	}	
	///////////////////////Add edge interrupting////////////////////////
	
	int32_t Idperson = worldModel->getIdentifierByType(type);
	AGMModelSymbol::SPtr person = worldModel->getSymbolByIdentifier(Idperson);
	int32_t Idrobot = worldModel->getIdentifierByType("robot");
	AGMModelSymbol::SPtr robot = worldModel->getSymbolByIdentifier(Idrobot);

	if ((looking==true) and (close==true))
	{
		qDebug()<<"CERCA Y MIRANDO";
		try
		{		
			worldModel->addEdge(person,robot, "interrupting");
			qDebug()<<"SE AÑADE EL ENLACE";
			changes = true;
		}
		catch(...)
		{
			qDebug()<<"EXISTE EL ENLACE";
			changes = false;
		}		
	}	
	
	else 
	{	try
		{
			worldModel->removeEdge(person,robot,"interrupting");
			changes = true;
		}
		catch(...)
		{
			qDebug()<<"NO EXISTE EL ENLACE";
			changes = false;
		}
	}
	return changes;
}


SNGPolylineSeq SocialRules::ApplySocialRules(SNGPersonSeq tperson)
{
	qDebug()<<__FUNCTION__;
	totalperson=tperson;
	movperson.clear();
	quietperson.clear();
	seq.clear();
	
	//for each person check if the velocity is 0, if it is add to totalp, if not add to totalpmov
	
	for (auto p: totalperson)
	{
		if (p.vel > 0)
			movperson.push_back(p);
		else
			quietperson.push_back(p);
		
	}
	
	//RoboCompTrajectoryRobot2D::PolyLineList list;
	
	if (!quietperson.empty())
	{
		SNGPolylineSeq secuencia = calculateGauss(false);
		
		for(auto s: secuencia)
		{	
			seq.push_back(s);
		}			    
	}
	
	if (!movperson.empty())
	{
		SNGPolylineSeq secuencia2 = PassOnRight(false);
		for(auto s: secuencia2)
		{	
			seq.push_back(s);			
		}
		  
	}

	if (!objects.empty())	
	{  

		SNGPolylineSeq secuenciaobj = objectInteraction(false);
		for(auto s: secuenciaobj)
		{
			seq.push_back(s);
		}
		
	} 
	
	//SNGPolylineSeq seqpoints = socialnavigationgaussian_proxy->RemovePoints(seq);
	
/*	for(auto s: seq)
		{			
			RoboCompTrajectoryRobot2D::PolyLine poly;

			for(auto p: s)
			{
				RoboCompTrajectoryRobot2D::PointL punto = {p.x, p.z};
				poly.push_back(punto);
			}
			
			list.push_back(poly);
		}*/	
	
	return seq;
}



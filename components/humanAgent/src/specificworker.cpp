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

#ifdef USE_QTGUI
	innerModelViewer = NULL;
	osgView = new OsgView(this);
	osgGA::TrackballManipulator *tb = new osgGA::TrackballManipulator;
	osg::Vec3d eye(osg::Vec3(4000.,4000.,-1000.));
	osg::Vec3d center(osg::Vec3(0.,0.,-0.));
	osg::Vec3d up(osg::Vec3(0.,1.,0.));
	tb->setHomePosition(eye, center, up, true);
	tb->setByMatrix(osg::Matrixf::lookAt(eye,center,up));
	osgView->setCameraManipulator(tb);

#endif
	active = false;
	worldModel = AGMModel::SPtr(new AGMModel());
	worldModel->name = "worldModel";
	innerModel = new InnerModel();

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{

#ifdef USE_QTGUI
	innerModelViewer = new InnerModelViewer (innerModel, "root", osgView->getRootGroup(), true);
#endif


	try
	{
		RoboCompAGMWorldModel::World w = agmexecutive_proxy->getModel();
		structuralChange(w);
	}
	catch(...)
	{
		printf("The executive is probably not running, waiting for first AGM model publication...");
	}

    Period = 200;
    timer.start(Period);

	return true;
}



int SpecificWorker::includeInAGM(int id,const Pose3D &pose)
{
	printf("includeInAGM begins\n");

	std::string name = "person";
	std::string imName = "person" + std::to_string(id);
	int personSymbolId = -1;
	int idx=0;
	while ((personSymbolId = worldModel->getIdentifierByType(name, idx++)) != -1)
	{
		printf("%d %d\n", idx, personSymbolId);
		if (worldModel->getSymbolByIdentifier(personSymbolId)->getAttribute("imName") == imName)
		{
			printf("found %d!!\n", personSymbolId);
			break;
		}
	}
	if (personSymbolId != -1)
	{
		printf("Person already in the AGM model\n");
		return personSymbolId;
	}

	AGMModel::SPtr newModel(new AGMModel(worldModel));

	// Symbolic part
	AGMModelSymbol::SPtr person = newModel->newSymbol("person");
	personSymbolId = person->identifier;
	printf("Got personSymbolId: %d\n", personSymbolId);
	person->setAttribute("imName", imName);
	person->setAttribute("imType", "transform");
	AGMModelSymbol::SPtr personSt = newModel->newSymbol("personSt" + std::to_string(id));
	printf("person %d status %d\n", person->identifier, personSt->identifier);

	newModel->addEdge(person, personSt, "hasStatus");
	newModel->addEdge(person, personSt, "noReach");
	newModel->addEdge(person, personSt, name);
    newModel->addEdgeByIdentifiers(person->identifier, 3, "in");


	// Geometric part
	std::map<std::string, std::string> edgeRTAtrs;
	edgeRTAtrs["tx"] = std::to_string(pose.x);
	edgeRTAtrs["ty"] = "0";
	edgeRTAtrs["tz"] = std::to_string(pose.z);
	edgeRTAtrs["rx"] = "0";
	edgeRTAtrs["ry"] = std::to_string(pose.ry);
	edgeRTAtrs["rz"] = "0";
	newModel->addEdgeByIdentifiers(100, person->identifier, "RT", edgeRTAtrs);


	AGMModelSymbol::SPtr personMesh = newModel->newSymbol("human01.3ds");
	printf("personMesh %d\n", personMesh->identifier);
	personMesh->setAttribute("collidable", "false");
	personMesh->setAttribute("imName", imName + "_Mesh");
	personMesh->setAttribute("imType", "mesh");
	std::string meshPath = "/home/robocomp/robocomp/components/robocomp-araceli/models/human01.3ds";
	personMesh->setAttribute("path", meshPath);
	personMesh->setAttribute("render", "NormalRendering");
	personMesh->setAttribute("scalex", "12");
	personMesh->setAttribute("scaley", "12");
	personMesh->setAttribute("scalez", "12");

	edgeRTAtrs["tx"] = "0";
	edgeRTAtrs["ty"] = "0";
	edgeRTAtrs["tz"] = "0";
	edgeRTAtrs["rx"] = "1.570796326794";
	edgeRTAtrs["ry"] = "0";
	edgeRTAtrs["rz"] = "3.1415926535";
	newModel->addEdge(person, personMesh, "RT", edgeRTAtrs);

	while (true)
	{
		if(sendModificationProposal(worldModel, newModel))
		{
			break;
		}
		sleep(1);
	}
	printf("includeInAGM ends\n");
	return personSymbolId;
}

SpecificWorker::Pose3D SpecificWorker::getPoseRot (jointListType list)
{

	Pose3D personpose;
    int32_t SymbolId = -1;

    int idx=0;
    while ((SymbolId = worldModel->getIdentifierByType("camera_astra", idx++)) != -1)
    {
        if (worldModel->getSymbolByIdentifier(SymbolId)->getAttribute("imName") == "astraRGBD")
            qDebug()<<"CameraFound";
            break;

    }

    if (SymbolId == -1)
    {
        printf("Camera not found \n");
    }

    else
        {

            try
            {
                qDebug()<<"Reading MidSpine position";
                auto j = list["MidSpine"];

                if (j.size() == 3)
                {
                    std::cout<<"EN EL MUNDO DE LA CAMARA "<<j[0] <<" " <<j[1] << " "<<j[2] << std::endl;

                    QVec jointinworld = innerModel->transform("world", QVec::vec3(j[0],j[1],j[2]), "camera_astra");

                    std::cout<<"EN EL MUNDO"<<jointinworld.x() <<" " <<jointinworld.y() << " "<<jointinworld.z() << std::endl;

                    personpose.x =jointinworld.x();
                    personpose.y = 0;
                    personpose.z= jointinworld.z();

                    personpose.rx = 0;
                    personpose.ry = 0;
                    personpose.rz = 0;

                    qDebug()<<"FIRST, INCLUDING IN AGM";
                    includeInAGM(666,personpose);

                }
                else
                    qDebug()<<"Joint Error";
            }

            catch(...)
            {
                qDebug()<<"NO EXISTE EL JOINT MidSpine";
            }
        }



	return personpose;

}


void SpecificWorker::compute()
{
	QMutexLocker locker(mutex);

    try
    {
        PersonList users;
        humantracker_proxy-> getUsersList(users);
        //leer usuarios desde la camara

        if(users.size()!=0)
        {	//si ha detectado personas:

			for (auto p:users)
			{
				//comprobar si existe o si se ha eliminado
				//si no existe
				auto id = p.first;
                jointListType joints_person = p.second.joints;

				auto personpose = getPoseRot(joints_person);


//				includeInAGM(id,personpose);
//				//si se ha eliminado
//				removeFromAGM(id);
//
//				moveInAGM (id, personpose);
				//si id está en id list -> comprobar movimiento
				//si id no está -> incluir en agm
				//si id_list tiene id que no está entre users -> eliminar agm


			}
        }

//        for(auto u : users)
//        {
//
//
//            qDebug()<<"ID " <<u.first << "STATUS"<<u.second.state;
//            jointListType jointsperson;
//
//            jointsperson = u.second.joints;
//            qDebug()<<"-----------------JOINTS---------------------";
//            for (auto j : jointsperson)
//            {
//               std::cout << j.first << " " <<j.second << endl;
//            }
//        }

    }


    catch(...)
    {
        qDebug()<<"Si no enciendes la camara poco podemos hacer, chiqui" ;
    }






#ifdef USE_QTGUI
    if (innerModelViewer) innerModelViewer->update();
    osgView->frame();
#endif
}







///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SpecificWorker::reloadConfigAgent()
{
//implementCODE
	return true;
}

bool SpecificWorker::activateAgent(const ParameterMap &prs)
{
//implementCODE
	bool activated = false;
	if (setParametersAndPossibleActivation(prs, activated))
	{
		if (not activated)
		{
			return activate(p);
		}
	}
	else
	{
		return false;
	}
	return true;
}

bool SpecificWorker::setAgentParameters(const ParameterMap &prs)
{
//implementCODE
	bool activated = false;
	return setParametersAndPossibleActivation(prs, activated);
}

ParameterMap SpecificWorker::getAgentParameters()
{
//implementCODE
	return params;
}

void SpecificWorker::killAgent()
{
//implementCODE

}

int SpecificWorker::uptimeAgent()
{
//implementCODE
	return 0;
}

bool SpecificWorker::deactivateAgent()
{
//implementCODE
	return deactivate();
}

StateStruct SpecificWorker::getAgentState()
{
//implementCODE
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

void SpecificWorker::structuralChange(const RoboCompAGMWorldModel::World &w)
{
//subscribesToCODE
	QMutexLocker lockIM(mutex);
 	AGMModelConverter::fromIceToInternal(w, worldModel);
 
	delete innerModel;
	innerModel = AGMInner::extractInnerModel(worldModel);
	regenerateInnerModelViewer();
}

void SpecificWorker::edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications)
{
//subscribesToCODE
	QMutexLocker lockIM(mutex);
	for (auto modification : modifications)
	{
		AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
		AGMInner::updateImNodeFromEdge(worldModel, modification, innerModel);
	}

}

void SpecificWorker::edgeUpdated(const RoboCompAGMWorldModel::Edge &modification)
{
//subscribesToCODE
	QMutexLocker locker(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
	AGMInner::updateImNodeFromEdge(worldModel, modification, innerModel);

}

void SpecificWorker::symbolUpdated(const RoboCompAGMWorldModel::Node &modification)
{
//subscribesToCODE
	QMutexLocker locker(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);

}

void SpecificWorker::symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence &modifications)
{
//subscribesToCODE
	QMutexLocker l(mutex);
	for (auto modification : modifications)
		AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);

}



void SpecificWorker::regenerateInnerModelViewer()
{
	if (innerModelViewer)
	{
		osgView->getRootGroup()->removeChild(innerModelViewer);
	}

	innerModelViewer = new InnerModelViewer(innerModel, "root", osgView->getRootGroup(), true);
}


bool SpecificWorker::setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated) {
	printf("<<< setParametersAndPossibleActivation\n");
	// We didn't reactivate the component
	reactivated = false;

	// Update parameters
	params.clear();
	for (ParameterMap::const_iterator it = prs.begin(); it != prs.end(); it++) {
		params[it->first] = it->second;
	}

	try {
		action = params["action"].value;
		std::transform(action.begin(), action.end(), action.begin(), ::tolower);
		//TYPE YOUR ACTION NAME
		if (action == "actionname") {
			active = true;
		} else {
			active = true;
		}
	}
	catch (...) {
		printf("exception in setParametersAndPossibleActivation %d\n", __LINE__);
		return false;
	}

	// Check if we should reactivate the component
	if (active) {
		active = true;
		reactivated = true;
	}

	printf("setParametersAndPossibleActivation >>>\n");

	return true;

}

bool SpecificWorker::sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel)
{
	bool result = false;
	try
	{	qDebug()<<"Intentando sendModificationProposal";
		AGMMisc::publishModification(newModel, agmexecutive_proxy, "HumanAgent");
		qDebug()<<"sendModificationProposal";
		result = true;
	}
	catch(const RoboCompAGMExecutive::Locked &e)
	{
		printf("agmexecutive locked...\n");
	}
	catch(const RoboCompAGMExecutive::OldModel &e)
	{
		printf("agmexecutive oldModel...\n");
	}
	catch(const RoboCompAGMExecutive::InvalidChange &e)
	{
		printf("agmexecutive InvalidChange...\n");
	}
	catch(const Ice::Exception& e)
	{
		exit(1);
	}
	return result;
}

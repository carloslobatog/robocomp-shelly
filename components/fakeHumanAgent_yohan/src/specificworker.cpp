/*
 *    Copyright (C) 2016 by YOUR NAME HERE
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
#include <qt4/QtGui/qdial.h>



/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
				warpcontrol=0;
        active = false;
        worldModel = AGMModel::SPtr(new AGMModel());
        worldModel->name = "worldModel";
        innerModel = new InnerModel();
        humancount=1;
        humanAdvVel = 50;
        humanRot = 0;
        humannum=1;
        xpos=3500;
        ypos=0;
        zpos=1500;
        fakehuman="fakeperson1";
        fakehumanmesh="fakeperson_mesh1";
        humansymbol="person1";
        meshpath="/home/robocomp/robocomp/components/robocomp-shelly/files/mesh/human0";
        setWindowTitle("Humanfake");


//	lastJoystickEvent = QTime::currentTime();
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

void SpecificWorker::includeInRCIS()
{

        printf("includeInRCIS begins\n");
        try
          {


                  pose.x = xpos;
                  pose.y = ypos;
                  pose.z = zpos;
                  pose.rx = pose.ry = pose.rz = 0;

                  innermodelmanager_proxy->addTransform(fakehuman, "static", "root", pose);

                  RoboCompInnerModelManager::meshType mesh;
                  mesh.pose.x  = mesh.pose.y  = mesh.pose.z  = 0;
                  mesh.pose.rx = 1.57079632679;
                  mesh.pose.ry = 0;
                  mesh.pose.rz = 3.1415926535;
                  mesh.scaleX = mesh.scaleY = mesh.scaleZ = 12;
                  mesh.render = 0;
                  //mesh.meshPath = "/home/robocomp/robocomp/files/osgModels/Gualzru/Gualzru.osg";
                  mesh.meshPath = meshpath;
                  innermodelmanager_proxy->addMesh(fakehumanmesh, fakehuman, mesh);


          }
          catch (...)
          {
                  printf("Can't create fake peson\n");
          }


        printf("includeInRCIS ends\n");
}

void SpecificWorker::includeInAGM()
{



          printf("includeInAGM begins\n");
          int idx=0;

          while ((personSymbolId[0] = worldModel->getIdentifierByType(humansymbol, idx++)) != -1)
          {
                  printf("%d %d\n", idx, personSymbolId[0]);
                  if (idx > 5) exit(0);
                  if (worldModel->getSymbolByIdentifier(personSymbolId[0])->getAttribute("imName") == fakehuman)
                  {
                          printf("found %d!!\n", personSymbolId[0]);
                          break;
                  }

          }
          if (personSymbolId[0] != -1)
          {
                  printf("Fake person already in the AGM model\n");
                  return;
          }
					else
					std::cout<<"Adding to AGM model";

          AGMModel::SPtr newModel(new AGMModel(worldModel));
					for(int i=0;i<hnum;i++)
					{
						fakehuman="fakeperson";
						fakehuman.append(std::to_string(humancount));
						fakehumanmesh="fakepersonMesh";
						fakehumanmesh.append(std::to_string(humancount));
						humansymbol="person";
						humansymbol.append(std::to_string(humancount));
						meshpath="/home/robocomp/robocomp/components/robocomp-shelly/files/mesh/human0";
						meshpath.append(std::to_string(humancount));
						meshpath.append(".3ds");
						humancount++;
	          // Symbolic part
	          AGMModelSymbol::SPtr person =   newModel->newSymbol(humansymbol);
	          personSymbolId[i] = person->identifier;
	          printf("Got personSymbolId: %d\n", personSymbolId[i]);
	          person->setAttribute("imName", fakehuman);
	          person->setAttribute("imType", "transform");
	          AGMModelSymbol::SPtr personSt = newModel->newSymbol("personSt");
	          printf("person %d status %d\n", person->identifier, personSt->identifier);

	          newModel->addEdge(person, personSt, "hasStatus");
	          newModel->addEdge(person, personSt, "noReach");
	          newModel->addEdge(person, personSt, "person");

	          newModel->addEdgeByIdentifiers(person->identifier, 3, "in");


	          // Geometric part
	          std::map<std::string, std::string> edgeRTAtrs;
	          edgeRTAtrs["tx"] = std::to_string(xpos);
	          edgeRTAtrs["ty"] = "0";
	          edgeRTAtrs["tz"] = std::to_string(zpos);
	          edgeRTAtrs["rx"] = "0";
	          edgeRTAtrs["ry"] = "0";
	          edgeRTAtrs["rz"] = "0";
	          newModel->addEdgeByIdentifiers(100, person->identifier, "RT", edgeRTAtrs);


	          AGMModelSymbol::SPtr personMesh = newModel->newSymbol("mesh");
	          printf("personMesh %d\n", personMesh->identifier);
	          personMesh->setAttribute("collidable", "false");
	          personMesh->setAttribute("imName", fakehumanmesh);
	          personMesh->setAttribute("imType", "mesh");
	          personMesh->setAttribute("path", meshpath);
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
					}


	          while (true)
	          {
	                  try
	                  {

	                          sendModificationProposal(worldModel, newModel);

	                          break;
	                  }
	                  catch(const RoboCompAGMExecutive::Locked &e)
	                  {
	                          printf("agmexecutive locked...\n");
	                  }
	                  catch(const RoboCompAGMExecutive::OldModel &e)
	                  {
	                          return;
	                  }
	                  catch(const RoboCompAGMExecutive::InvalidChange &e)
	                  {
	                          exit(1);
	                  }
	                  sleep(1);
	          }


          printf("includeInAGM ends\n");

}



/*void SpecificWorker::receivedJoyStickEvent(int value, int type, int number)
{
        printf("*\n");
        fflush(stdout);
        if (type != 2)
                return;

        if (number == 0) // rot
        {
                humanRotVel = float(value)/32767.*1.;
                lastJoystickEvent = QTime::currentTime();
        }
        if (number == 1)
        {
                humanAdvVel = float(-value)/32767.*600.;
                lastJoystickEvent = QTime::currentTime();
        }
        printf("*");
        fflush(stdout);
}*/

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
        try
        {
                RoboCompAGMWorldModel::World w = agmexecutive_proxy->getModel();
                structuralChange(w);
        }
        catch(...)
        {
                printf("The executive is probably not running, waiting for first AGM model publication...");
        }
				gethnum:
        std::cout<<"Enter Number of Humans"<<endl;
        scanf("%d",&hnum);
        if(hnum>5||hnum<1)
        {
          std::cout<<"Enter Again";
          goto gethnum;
        }
				for(int i=0;i<hnum;i++)
        {
					fakehuman="fakeperson";
					fakehuman.append(std::to_string(humancount));
					fakehumanmesh="fakeperson_mesh";
					fakehumanmesh.append(std::to_string(humancount));
					meshpath="/home/robocomp/robocomp/components/robocomp-shelly/files/mesh/human0";
					meshpath.append(std::to_string(humancount));
					meshpath.append(".3ds");
					std::cout<<fakehuman<<"\n";
	        // Include person in RCIS
	        includeInRCIS();
					fakehumanmesh="fakepersonMesh";
					fakehumanmesh.append(std::to_string(humancount));
					humansymbol="person";
          humansymbol.append(std::to_string(humancount));

					xpos=xpos;
					zpos=zpos-400;
					humancount++;

				}
				humancount=1;
				fakehuman="fakeperson";
				fakehuman.append(std::to_string(humancount));
				fakehumanmesh="fakepersonMesh";
				fakehumanmesh.append(std::to_string(humancount));
				humansymbol="person";
				humansymbol.append(std::to_string(humancount));
				meshpath="/home/robocomp/robocomp/components/robocomp-shelly/files/mesh/human0";
				meshpath.append(std::to_string(humancount));
				meshpath.append(".3ds");
				// Include person in AGM
				includeInAGM();
        // Joystick
        /*printf("Creating joystick...\n");
        joystick = new QJoyStick("/dev/input/js0");
        if (!joystick->openQJoy())
        {
                cout << "[" << PROGRAM_NAME << "]: Unable to open device: " << joystick->getDeviceName() << endl;
                return EXIT_FAILURE;
        }
        joystick->start();
        printf("Connecting joystick...\n");
        connect(joystick, SIGNAL(inputEvent(int, int, int)), this, SLOT(receivedJoyStickEvent(int, int, int)));

*/

        timer.start(Period);


        //Teclado
            //UP
        connect(up,SIGNAL(pressed()),this,SLOT(upP()));
        connect(up,SIGNAL(released()),this,SLOT(upR()));

            //DOWN
        connect(down,SIGNAL(pressed()),this,SLOT(downP()));
        connect(down,SIGNAL(released()),this,SLOT(downR()));

            //RIGHT
        connect (right,SIGNAL(pressed()),this,SLOT(rightP()));
        connect (right,SIGNAL(released()),this,SLOT(rightR()));

            //LEFT
        connect(left,SIGNAL(pressed()),this,SLOT(leftP()));
        connect(left,SIGNAL(released()),this,SLOT(leftR()));

        //GIRO
        connect (giro,SIGNAL(valueChanged(int)),this,SLOT(rotar(int)));
        connect (giro,SIGNAL(sliderPressed()),this,SLOT(giroP()));
        connect (giro,SIGNAL(sliderReleased()),this,SLOT(giroR()));

        //giro->setNotchesVisible(true);
        giro->QAbstractSlider::setMinimum (0);
        giro->QAbstractSlider::setMaximum (360);
      	//Get Jump Positions
				connect (Send,SIGNAL(clicked()),this,SLOT(warp()));

        //RADIOBUTTON
        connect (radioButton_1,SIGNAL(clicked()),this,SLOT(rb1()));
        connect (radioButton_2,SIGNAL(clicked()),this,SLOT(rb2()));
        connect (radioButton_3,SIGNAL(clicked()),this,SLOT(rb3()));
				connect (radioButton_4,SIGNAL(clicked()),this,SLOT(rb4()));
				connect (radioButton_5,SIGNAL(clicked()),this,SLOT(rb5()));


        return true;
}
//warp
void SpecificWorker::warp(){
	warpcontrol=1;
}
//jump
void SpecificWorker::jump(){
	fakehuman="fakeperson";
	fakehuman.append(std::to_string(humannum));
	warpcontrol=0;
	xpos=(X_POS->text()).toInt();
	zpos=(Z_POS->text()).toInt();
	theta=(Theta->text()).toFloat();
	std::cout<<xpos<<endl;

	pose.x = xpos;
	pose.y = 0;
	pose.z = zpos;
	pose.rx = 0;
	pose.ry =theta;
	pose.rz = 0;


	qDebug()<<"Pose x: "<<pose.x <<"Pose z:"<<pose.z<<"Rotation:"<<pose.ry;

	innermodelmanager_proxy->setPoseFromParent(fakehuman, pose);


	AGMModelSymbol::SPtr personParent = worldModel->getParentByLink(personSymbolId[humannum-1], "RT");
	AGMModelEdge &edgeRT  = worldModel->getEdgeByIdentifiers(personParent->identifier, personSymbolId[humannum-1], "RT");
	edgeRT.attributes["tx"] = float2str(xpos);
	edgeRT.attributes["ty"] = float2str(0);
	edgeRT.attributes["tz"] = float2str(zpos);
	edgeRT.attributes["rx"] = "0";
	edgeRT.attributes["ry"] = float2str(theta);
	edgeRT.attributes["rz"] = "0";


	AGMMisc::publishEdgeUpdate(edgeRT, agmexecutive_proxy);

}

//UP
void SpecificWorker::upP(){
  tbutton.up =true;
}
void SpecificWorker::upR(){
  tbutton.up =false;
}
//DOWN
void SpecificWorker::downP(){
  tbutton.down =true;

}
void SpecificWorker::downR(){
 tbutton.down =false;
}

//RIGHT
void SpecificWorker::rightP(){
  tbutton.right =true;
}
void SpecificWorker::rightR(){
  tbutton.right =false;
}
//LEFT
void SpecificWorker::leftP(){
  tbutton.left =true;
}
void SpecificWorker::leftR(){
  tbutton.left =false;
}

//ROT
void SpecificWorker::rotar(int value){
   valorgiro=value;

}
void SpecificWorker::giroP(){
  tbutton.rotacion=true;
}
void SpecificWorker::giroR(){
  tbutton.rotacion=false;
}
//RADIOBUTTON
void SpecificWorker::rb1(){
        humannum=1;
        qDebug()<<humannum;
}
void SpecificWorker::rb2(){
        humannum=2;
        qDebug()<<humannum;
}
void SpecificWorker::rb3(){
        humannum=3;
        qDebug()<<humannum;
}
void SpecificWorker::rb4(){
        humannum=4;
        qDebug()<<humannum;
}
void SpecificWorker::rb5(){
        humannum=5;
        qDebug()<<humannum;
}


//MOVE

void SpecificWorker::move (){
	fakehuman="fakeperson";
	fakehuman.append(std::to_string(humannum));
  RoboCompInnerModelManager::coord3D coordInItem;
  RoboCompInnerModelManager::coord3D coordInBase;

    if (tbutton.up==true){
                coordInItem.x = 0;
                coordInItem.y = 0;
                coordInItem.z =humanAdvVel;
                innermodelmanager_proxy->transform("root", fakehuman, coordInItem, coordInBase);

         }
         else if (tbutton.down==true){
                coordInItem.x = 0;
                coordInItem.y = 0;
                coordInItem.z =-humanAdvVel;
                innermodelmanager_proxy->transform("root", fakehuman, coordInItem, coordInBase);
         }

         else if (tbutton.right==true){
                coordInItem.z = 0;
                coordInItem.y = 0;
                coordInItem.x =humanAdvVel;
                innermodelmanager_proxy->transform("root", fakehuman, coordInItem, coordInBase);
         }

         else if (tbutton.left==true){
                coordInItem.z = 0;
                coordInItem.y = 0;
                coordInItem.x =-humanAdvVel;
                innermodelmanager_proxy->transform("root", fakehuman, coordInItem, coordInBase);
         }

        else if (tbutton.rotacion==true){

                humanRot =valorgiro*0.0175;
                coordInItem.x=0;
                coordInItem.y=0;
                coordInItem.z=0;
                innermodelmanager_proxy->transform("root", fakehuman, coordInItem, coordInBase);

        }

                pose.x = coordInBase.x;
                pose.y = coordInBase.y;
                pose.z = coordInBase.z;
                pose.rx = 0;
                pose.ry =humanRot;
                pose.rz = 0;


                qDebug()<<"Pose x: "<<pose.x <<"Pose z:"<<pose.z<<"Rotacion:"<<pose.ry;

                innermodelmanager_proxy->setPoseFromParent(fakehuman, pose);


                AGMModelSymbol::SPtr personParent = worldModel->getParentByLink(personSymbolId[humannum-1], "RT");
                AGMModelEdge &edgeRT  = worldModel->getEdgeByIdentifiers(personParent->identifier, personSymbolId[humannum-1], "RT");
                edgeRT.attributes["tx"] = float2str(coordInBase.x);
                edgeRT.attributes["ty"] = float2str(coordInBase.y);
                edgeRT.attributes["tz"] = float2str(coordInBase.z);
                edgeRT.attributes["rx"] = "0";
                edgeRT.attributes["ry"] = float2str(humanRot);
                edgeRT.attributes["rz"] = "0";


                AGMMisc::publishEdgeUpdate(edgeRT, agmexecutive_proxy);

}


void SpecificWorker::compute()

{
        QMutexLocker locker(mutex);
        //static QTime lastCompute = QTime::currentTime();


        try
      	{

        	if ((tbutton.up==true)||(tbutton.down==true)||(tbutton.right==true)||(tbutton.left==true)||(tbutton.rotacion==true))
          {
						if(hnum>=humannum)
        	  move();
        	}
					if(warpcontrol==1)
					{
						if(hnum>=humannum)
						jump();
					}
      	}
      	catch(...)
      	{
      		std::cout<<"booboo";
      	}





        /*if (lastJoystickEvent.elapsed()  < 3000)
        {
                printf("vel: %f %f\n", humanAdvVel, humanRotVel);
                RoboCompInnerModelManager::coord3D coordInItem;
                coordInItem.x = 0;
                coordInItem.y = 0;
                coordInItem.z = humanAdvVel*0.001*lastCompute.elapsed();
                RoboCompInnerModelManager::coord3D coordInBase;
                printf("transform (%f, %f, %f) to fake person (%f, %f, %f)\n", coordInItem.x, coordInItem.y, coordInItem.z, coordInBase.x, coordInBase.y, coordInBase.z);
                innermodelmanager_proxy->transform("root", "fakeperson",  coordInItem, coordInBase);

                RoboCompInnerModelManager::Pose3D pose;
                humanRot += humanRotVel*0.001*lastCompute.elapsed();
                pose.x = coordInBase.x;
                pose.y = coordInBase.y;
                pose.z = coordInBase.z;
                pose.rx = 0;
                pose.ry = humanRot;
                pose.rz = 0;
                innermodelmanager_proxy->setPoseFromParent("fakeperson", pose);

                AGMModelSymbol::SPtr personParent = worldModel->getParentByLink(personSymbolId, "RT");
                AGMModelEdge &edgeRT  = worldModel->getEdgeByIdentifiers(personParent->identifier, personSymbolId, "RT");
                edgeRT.attributes["tx"] = float2str(coordInBase.x);
                edgeRT.attributes["ty"] = float2str(coordInBase.y);
                edgeRT.attributes["tz"] = float2str(coordInBase.z);
                edgeRT.attributes["rx"] = "0";
                edgeRT.attributes["ry"] = float2str(humanRot);
                edgeRT.attributes["rz"] = "0";
                printf("%d----[%f]--->%d\n", personParent->identifier, coordInBase.z, personSymbolId);
                AGMMisc::publishEdgeUpdate(edgeRT, agmexecutive_proxy);
        }
        else
        {
                printf(".");
                fflush(stdout);
        }



        lastCompute = QTime::currentTime();

*/
}
bool SpecificWorker::reloadConfigAgent()
{
        return true;
}

bool SpecificWorker::activateAgent(const ParameterMap &prs)
{
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
        bool activated = false;
        return setParametersAndPossibleActivation(prs, activated);
}

ParameterMap SpecificWorker::getAgentParameters()
{
        return params;
}

void SpecificWorker::killAgent()
{

}

int SpecificWorker::uptimeAgent()
{
        return 0;
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

void SpecificWorker::structuralChange(const RoboCompAGMWorldModel::World &w)
{
        mutex->lock();
        AGMModelConverter::fromIceToInternal(w, worldModel);

        delete innerModel;
        innerModel = AGMInner::extractInnerModel(worldModel);
        mutex->unlock();
}

void SpecificWorker::edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications)
{
        QMutexLocker locker(mutex);
        for (auto modification : modifications)
        {
                AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
                AGMInner::updateImNodeFromEdge(worldModel, modification, innerModel);
        }
}

void SpecificWorker::edgeUpdated(const RoboCompAGMWorldModel::Edge &modification)
{
        QMutexLocker locker(mutex);
        AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
        AGMInner::updateImNodeFromEdge(worldModel, modification, innerModel);
}

void SpecificWorker::symbolUpdated(const RoboCompAGMWorldModel::Node &modification)
{
        QMutexLocker locker(mutex);
        AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
}

void SpecificWorker::symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence &modifications)
{
        QMutexLocker l(mutex);
        for (auto modification : modifications)
                AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
}



bool SpecificWorker::setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated)
{
        QMutexLocker l(mutex);
        printf("<<< setParametersAndPossibleActivation\n");
        // We didn't reactivate the component
        reactivated = false;

        // Update parameters
        params.clear();
        for (ParameterMap::const_iterator it=prs.begin(); it!=prs.end(); it++)
        {
                params[it->first] = it->second;
        }

        try
        {
                action = params["action"].value;
                std::transform(action.begin(), action.end(), action.begin(), ::tolower);
                //TYPE YOUR ACTION NAME
                if (action == "actionname")
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
                printf("exception in setParametersAndPossibleActivation\n");
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

void SpecificWorker::sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel)
{
        try
        {	qDebug()<<"Intentando sendModificationProposal";
                AGMMisc::publishModification(newModel, agmexecutive_proxy, "fakeHumanAgentAgent");
                qDebug()<<"sendModificationProposal";
        }
/*	catch(const RoboCompAGMExecutive::Locked &e)
        {
        }
        catch(const RoboCompAGMExecutive::OldModel &e)

        }
        catch(const RoboCompAGMExecutive::InvalidChange &e)
        {
        }
*/
        catch(const Ice::Exception& e)
        {
                exit(1);
        }
}

/*
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
	innerModel = new InnerModel();
	haveTarget = false;

	//Timed slot to read TrajectoryRobot2D state
	connect(&trajReader, SIGNAL(timeout()), this, SLOT(readTrajState()));
	//Dibujar gaussiana
	connect(gaussiana,SIGNAL(clicked()),this, SLOT(gauss()));
	connect(por,SIGNAL(clicked()),this, SLOT(gausspor()));
	connect(objint,SIGNAL(clicked()),this, SLOT(objectInteraction()));
	//Sacar la pose del robot
	connect(datos,SIGNAL(clicked()),this, SLOT(grabarfichero()));
	//trajReader.start(1000);

	//SLIDER
	connect (proximidad,SIGNAL(valueChanged(int)),this,SLOT(cambiarvalor(int)));
	//connect (proximidad,SIGNAL(sliderMoved()),this,SLOT(sliderM()));


	proximidad->QSlider::setMinimum (0);
	proximidad->QSlider::setMaximum (100);
	proximidad->QSlider::setTracking (false);
	proximidad->QSlider::setValue (50);
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
}

void SpecificWorker::cambiarvalor(int value)
{
	valorprox=value;
	qDebug()<<"Proximidad"<<valorprox;

}

void SpecificWorker::grabarfichero()
{

  qDebug("escribimos en el fichero robotpose.txt la pose del robot");
  	ofstream fichero("robotpose.txt", ofstream::out);
	for (auto p:poserobot)
	{
		fichero<< p.x << " " <<p.z<< endl;
	}
	fichero.close();

qDebug("escribimos en el fichero personpose.txt la pose de las personas");
  	ofstream fichero2("personpose.txt", ofstream::out);
	for (auto person:totalpersons)
	{
		fichero2<< person.x << " " <<person.z<<" "<<person.angle<< endl;
	}
	fichero2.close();
	poserobot.clear();


	qDebug("escribimos en el fichero poly.txt la polilinea");
  	ofstream fichero3("poly.txt", ofstream::out);

	for (auto s:secuencia)
	{
		for (auto p: s){
			fichero3<< p.x << " " <<p.z<<" "<< endl;
		}
	}
	fichero3.close();

	qDebug()<<"escribimos en el fichero la distancia recorrida"<<totaldist;
  	ofstream fichero4("dist.txt", ofstream::out);
	fichero4<< totaldist << endl;
	totaldist = 0;
	fichero4.close();

	/////Guardar cada polilinea por separado
	int i = 0;
	for (auto s:secuencia){
		QString name = QString("polyline")+QString::number(i,10)+QString(".txt");
		ofstream fichero5(name.toUtf8().constData(), ofstream::out);
			for (auto p: s)
			{
				fichero5<< p.x << " " <<p.z<<" "<< endl;
			}
			i++;
			fichero5.close();
		}

}
SNGPolylineSeq SpecificWorker::gauss(bool dibujar)
{

/*	SNGPersonSeq persons;

	//push back es para incluir a la persona en el vector de personas
	if (p1)
	persons.push_back(person1);
	if (p2)
	persons.push_back(person2);
	if (p3)
	persons.push_back(person3);
	if (p4)
	persons.push_back(person4);
	if (p5)
	persons.push_back(person5);
	if (p6)
	persons.push_back(person6)*/;


	secuencia.clear();
	secuencia = socialnavigationgaussian_proxy-> getPersonalSpace(totalp, valorprox, dibujar);

	return secuencia;

}
SNGPolylineSeq SpecificWorker::gausspor(bool dibujar)
{

// 	SNGPersonSeq persons;
// 
// 	//push back es para incluir a la persona en el vector de personas
// 	if (pp1)	persons.push_back(person1);
// 	if (pp2)	persons.push_back(person2);
// 	if (pp3)	persons.push_back(person3);
// 	if (pp4)	persons.push_back(person4);
// 	if (pp5)	persons.push_back(person5);
// 	if (pp6)	persons.push_back(person6);

	secuencia2.clear();
	secuencia2 = socialnavigationgaussian_proxy-> getPassOnRight(totalpmov, valorprox, dibujar);

	return secuencia2;

	
}

void SpecificWorker::addObjects()
{
  qDebug()<<"AAAAAAADDDD OOOOOOBBBJEEEECTTTSS";
  //////////cafetera//////////////
	try
	{	
	   
		pose.x = 450;
		pose.y = 950;
		pose.z = -2250;
		pose.rx = 0;
		pose.ry = M_PIl/2;
		pose.rz = 0;
		innermodelmanager_proxy->addTransform("cafetera", "static", "root", pose);

		RoboCompInnerModelManager::meshType mesh;
		mesh.pose.x  = mesh.pose.y  = mesh.pose.z  = 0;
		mesh.pose.rx = M_PIl/2;
		mesh.pose.ry = 0;
		mesh.pose.rz = M_PIl/2;
		
		mesh.scaleX = mesh.scaleY = mesh.scaleZ = 120;
		mesh.render = 0;
		//mesh.meshPath = "/home/robocomp/robocomp/files/osgModels/Gualzru/Gualzru.osg";
		mesh.meshPath = "/home/robocomp/robocomp/components/robocomp-araceli/models/cafe1.3DS";
		//mesh.meshPath = "/home/robocomp/robocomp/files/osgModels/basics/cube2.3ds";
		innermodelmanager_proxy->addMesh("objectMesh", "cafetera", mesh);
	}
	catch (...)
	{
		printf("Can't create object\n");
	}

	printf("includeInRCIS ends\n");
	 
  /////////////TABLON///////////
	qDebug()<<"AAAAAAADDDD OOOOOOBBBJEEEECTTTSS";
	try
	{	
		pose.x = 5360;
		pose.y = 1200;
		pose.z = -270;
		pose.rx = 0;
		pose.ry = M_PIl;
		pose.rz = 0;
		innermodelmanager_proxy->addTransform("board", "static", "root", pose);

		RoboCompInnerModelManager::meshType mesh1;
		mesh1.pose.x  = mesh1.pose.y  = mesh1.pose.z  = 0;
		mesh1.pose.rx = M_PIl/2;
		mesh1.pose.ry = 0;
		mesh1.pose.rz = M_PIl;
		
		mesh1.scaleX = mesh1.scaleY = mesh1.scaleZ = 5;
		mesh1.render = 0;
		//mesh1.mesh1Path = "/home/robocomp/robocomp/files/osgModels/Gualzru/Gualzru.osg";
		mesh1.meshPath = "/home/robocomp/robocomp/components/robocomp-araceli/models/board.3DS";
		//mesh1.mesh1Path = "/home/robocomp/robocomp/files/osgModels/basics/cube2.3ds";
		innermodelmanager_proxy->addMesh("objectMesh1", "board", mesh1);
	}
	catch (...)
	{
		printf("Can't create object\n");
	}

	printf("includeInRCIS ends\n");
	
////////TELEFONO/////////////
	qDebug()<<"AAAAAAADDDD OOOOOOBBBJEEEECTTTSS";
	try
	{	
	  

		pose.x = 3000;
		pose.y = 750;
		pose.z = 3650;
		pose.rx = 0;
		pose.ry = M_PIl;
		pose.rz = 0;
		innermodelmanager_proxy->addTransform("phone", "static", "root", pose);

		RoboCompInnerModelManager::meshType mesh2;
		mesh2.pose.x  = mesh2.pose.y  = mesh2.pose.z  = 0;
		mesh2.pose.rx = M_PIl/2;
		mesh2.pose.ry = 0;
		mesh2.pose.rz = M_PIl;
		
		mesh2.scaleX = mesh2.scaleY = mesh2.scaleZ = 30;
		mesh2.render = 0;
		//mesh2.mesh2Path = "/home/robocomp/robocomp/files/osgModels/Gualzru/Gualzru.osg";
		mesh2.meshPath = "/home/robocomp/robocomp/components/robocomp-araceli/models/phone.3DS";
		//mesh2.mesh2Path = "/home/robocomp/robocomp/files/osgModels/basics/cube2.3ds";
		innermodelmanager_proxy->addMesh("objectMesh2", "phone", mesh2);
	}
	catch (...)
	{
		printf("Can't create object\n");
	}

	printf("includeInRCIS ends\n");

	
	
	
// 	static bool first = true;
// 	if (not first) return;
// 	first = false;
// 
// 	std::string m ="  ";
// 	int idx=0;
// 	while ((objectSymbolId = worldModel->getIdentifierByType("objectI", idx++)) != -1)
// 	{
// 		printf("%d %d\n", idx, objectSymbolId);
// 		if (idx > 4) exit(0);
// 		if (worldModel->getSymbolByIdentifier(objectSymbolId)->getAttribute("imName") == "cafetera")
// 		{
// 			printf("found %d!!\n", objectSymbolId);
// 			break;
// 		}
// 	}
// 	if (objectSymbolId != -1)
// 	{
// 		printf("Object already in the AGM model\n");
// 		return;
// 	}
// 
// 	AGMModel::SPtr newModel(new AGMModel(worldModel));
// 
// 	// Symbolic part
// 	AGMModelSymbol::SPtr cafetera =   newModel->newSymbol("objectI");
// 	objectSymbolId = cafetera->identifier;
// 	printf("Got objectSymbolId: %d\n", objectSymbolId);
// 	cafetera->setAttribute("imName", "cafetera");
// 	cafetera->setAttribute("imType", "transform");
// 	AGMModelSymbol::SPtr cafeteraSt = newModel->newSymbol("objectSt");
// 	printf("object %d status %d\n", cafetera->identifier, cafeteraSt->identifier);
// 
// 	newModel->addEdge(cafetera, cafeteraSt, "hasStatus");
// 	newModel->addEdge(cafetera, cafeteraSt, "noReach");
// 	newModel->addEdge(cafetera, cafeteraSt, "objectI");
// 	try
// 	{
// 	newModel->addEdgeByIdentifiers(cafetera->identifier, 3, "in");
// 	}
// 	catch(...)
// 	{
// 	  printf("PROOOOOBLEMA...\n");
// 	}
// 	
// 
// 
// 	// Geometric part
// 	std::map<std::string, std::string> edgeRTAtrs;
// 	edgeRTAtrs["tx"] = "3550";
// 	edgeRTAtrs["ty"] = "950";
// 	edgeRTAtrs["tz"] = "3650";
// 	edgeRTAtrs["rx"] = "0";
// 	edgeRTAtrs["ry"] = "3.1415926535";
// 	edgeRTAtrs["rz"] = "0";
// 	newModel->addEdgeByIdentifiers(100, cafetera ->identifier, "RT", edgeRTAtrs);
// 
// 
// 	AGMModelSymbol::SPtr cafeteraMesh = newModel->newSymbol("mesh");
// 	printf("cafeteraMesh %d\n", cafeteraMesh->identifier);
// 	cafeteraMesh->setAttribute("collidable", "false");
// 	cafeteraMesh->setAttribute("imName", "cafeteraMesh");
// 	cafeteraMesh->setAttribute("imType", "mesh");
// 	cafeteraMesh->setAttribute("path", "/home/robocomp/robocomp/components/robocomp-araceli/models/cafe.3ds");
// 	cafeteraMesh->setAttribute("render", "NormalRendering");
// 	cafeteraMesh->setAttribute("scalex", "20");
// 	cafeteraMesh->setAttribute("scaley", "20");
// 	cafeteraMesh->setAttribute("scalez", "20");
// 
// 	edgeRTAtrs["tx"] = "0";
// 	edgeRTAtrs["ty"] = "0";
// 	edgeRTAtrs["tz"] = "0";
// 	edgeRTAtrs["rx"] = "1.570796326794";
// 	edgeRTAtrs["ry"] = "0";
// 	edgeRTAtrs["rz"] = "3.1415926535";
// 	newModel->addEdge(cafetera, cafeteraMesh, "RT", edgeRTAtrs);
// 
// 
// 
// 	while (true)
// 	{
// 		try
// 		{
// 			sendModificationProposal(worldModel, newModel,m);
// 			break;
// 		}
// 		catch(const RoboCompAGMExecutive::Locked &e)
// 		{
// 			printf("agmexecutive locked...\n");
// 		}
// 		catch(const RoboCompAGMExecutive::OldModel &e)
// 		{
// 			return;
// 		}
// 		catch(const RoboCompAGMExecutive::InvalidChange &e)
// 		{ 
// 			exit(1);
// 		}
// 		sleep(1);
// 	}
// 
// 	printf("includeInAGM ends\n");
}

SNGPolylineSeq SpecificWorker::objectInteraction(bool d)
{
	
 	qDebug()<<"---------1------------";
	
	//SNGPersonSeq persons;
	SNGPersonSeq objects;
	
	//push back es para incluir a la persona en el vector de personas
// 	if (p1) 	persons.push_back(person1); 
// 	if (p2) 	persons.push_back(person2);
// 	if (p3) 	persons.push_back(person3);
// 	if (p4) 	persons.push_back(person4);
// 	if (p5) 	persons.push_back(person5);
// 	if (p6) 	persons.push_back(person6);

	totalpersons=totalpersons; 
	qDebug()<<"---------2----------";
	
	///cafetera
	object.x =0.450;
	object.z =-2.250;
	object.angle=3.1415926535/2;
	    
	objects.push_back(object);
	///nevera
	object.x =3.210;
	object.z =-3.720;
	object.angle=0;
	
	objects.push_back(object);
	
	///tablon
	object.x =5.360;
	object.z =-0.270;
	object.angle=3.1415926535;
	
	objects.push_back(object);
	
	///telefono
	object.x =3.000;
	object.z =3.650;
	object.angle=3.1415926535;
	
	objects.push_back(object);
	
	
	

	secuenciaObj.clear();
	secuenciaObj =socialnavigationgaussian_proxy->getObjectInteraction(totalp,objects,d);
	
	qDebug()<<"---------3----------";
	
// 	AGMModelSymbol::SPtr objectP = worldModel->getParentByLink(objectSymbolId, "RT");
// 	AGMModelEdge &edgeRT  = worldModel->getEdgeByIdentifiers(objectP->identifier,objectSymbolId, "RT");
// 	object.x = str2float(edgeRT.attributes["tx"])/1000;
// 	object.z = str2float(edgeRT.attributes["tz"])/1000;
// 			
// 		
// 	
// 
// 	qDebug()<<"Objeto"<<"Pose x"<<object.x<<"Pose z"<<object.z;

	
	
	/*
	for (auto p: persons)
	{
//  	float angleinf= (p.angle - 10)*0.0175;  
// 	float anglesup= (p.angle + 10)*0.0175;
// 	qDebug()<<"angleinf"<<angleinf<<"anglesup"<<anglesup;
 	float distancethr = 1500;
	
	int i=1;
	//problema.. puede estar la persona 1 y la 3 sin estar la dos entonces no iria bien
	QString name = QString("fakeperson") + QString::number(i,10);
	qDebug()<<"Persona"<<name;
	QVec pInperson = innerModel->transform(name, QVec::vec3(object.x, 0, object.z), "world");  
	
	qDebug()<<"pose x"<<pInperson.x()<<"pose z"<< pInperson.z();
	
	float distobj  = sqrt(pInperson.x()*pInperson.x() + pInperson.z()*pInperson.z());
	float angleobj = atan2(pInperson.x(),pInperson.z());
	
	
	
	qDebug()<<"dist al frigorifico"<<distobj<<"angle al frigorifico"<<angleobj;
	
	if (distobj<distancethr and abs(angleobj)<M_PIl/180.*20)
	{
	  qDebug()<<"YEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEESSSSSS";
	}
	else qDebug()<<"NOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOSS";
	
	i++;  
	}
	
	*/
	return secuenciaObj;
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	Period = 200;


	addObjects();
	
	timer.start(Period);
	return true;
}


// //////////////////////////////// COMPROBAMOS SI LAS DOS PERSONAS SE ESTAN COMUNICANDO //////////////////////////
// bool SpecificWorker::checkconversation(){
//
// 	//UMBRALES DE DISTANCIA Y ANGULO
// 	float anglethr = 30*0.0175;  //30 grados x 0,0175 para pasar a radianes
// 	float distancethr = 2.5;
//
// 	bool checkangle = false;
// 	bool checkdistance = false;
//
//  	//COMPROBAMOS ANGULO
// 	float angleinf = PI/2 - anglethr;
// 	float anglesup = PI/2 + anglethr;
//
//
//
// 	if (((angleinf<person1.angle && person1.angle < anglesup)&&
// 		(2*PI-angleinf > person2.angle && person2.angle > 2*PI - anglesup))||
// 		((2*PI-angleinf > person1.angle && person1.angle > 2*PI - anglesup)&&
// 		(angleinf<person2.angle && person2.angle < anglesup)))
//
//
// 		checkangle = true;
//
// 	else
// 		checkangle= false;
//
//
// 	//COMPROBAMOS DISTANCIA
// 	float distance = sqrt(((person2.x-person1.x)*(person2.x-person1.x))+((person2.z-person1.z)*(person2.z-person1.z)));
// 	qDebug()<<"Las dos personas se encuentran a "<<distance<<" metros de distancia";
//
// 	if (distance <= distancethr)
// 		checkdistance= true;
// 	else
// 		checkdistance= false;
//
//
// 	//SI SE DAN LAS DOS CONDICIONES LAS PERSONAS ESTAN HABLANDO
// 	if (checkangle && checkdistance)
// 		return true;
// 	else
// 		return false;
//
// }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void SpecificWorker::compute( )
{
	static bool first=true;
	if (first)
	{
		qLog::getInstance()->setProxy("both", logger_proxy);
		rDebug2(("navigationAgent started"));

	}

	if (worldModel->getIdentifierByType("robot") < 0)
	{
		try {

			qDebug()<<"Leo el mundo";
			agmexecutive_proxy->broadcastModel();

			return;
		}
		catch(...)
		{
			printf("The executive is probably not running, waiting for first AGM model publication...");
		}
	}

 	//Obtenemos el modelo de fake human

//  	if (p1==false){
//  	int idx=0;
// 
//         while ((personSymbolIdp1 = worldModel->getIdentifierByType("person1", idx++)) != -1)
// 	{
// 		if (idx > 4) exit(0);
//                 if (worldModel->getSymbolByIdentifier(personSymbolIdp1)->getAttribute("imName") == "fakeperson1")
// 		{
// 			p1=true;
// 			break;
// 		}
// 	}
// 	}
// 	if (p2==false){
// 	int idx=0;
// 	while ((personSymbolIdp2 = worldModel->getIdentifierByType("person2", idx++)) != -1)
// 	{
// 
// 		if (idx > 4) exit(0);
// 		if (worldModel->getSymbolByIdentifier(personSymbolIdp2)->getAttribute("imName") == "fakeperson2")
// 		{
// 			p2=true;
// 			break;
// 		}
// 	}
// 	}
// 	if (p3==false){
// 	int idx=0;
// 	while ((personSymbolIdp3 = worldModel->getIdentifierByType("person3", idx++)) != -1)
// 	{
// 
// 		if (idx > 4) exit(0);
// 		if (worldModel->getSymbolByIdentifier(personSymbolIdp3)->getAttribute("imName") == "fakeperson3")
// 		{
// 			p3=true;
// 			break;
// 		}
// 	}
// 	}
// 
// 	if (p4==false){
// 	int idx=0;
// 	while ((personSymbolIdp4 = worldModel->getIdentifierByType("person4", idx++)) != -1)
// 	{
// 
// 		if (idx > 4) exit(0);
// 		if (worldModel->getSymbolByIdentifier(personSymbolIdp4)->getAttribute("imName") == "fakeperson4")
// 		{
// 			p4=true;
// 			break;
// 		}
// 	}
// 	}
// 	if (p5==false){
// 	int idx=0;
// 	while ((personSymbolIdp5 = worldModel->getIdentifierByType("person5", idx++)) != -1)
// 	{
// 
// 		if (idx > 4) exit(0);
// 		if (worldModel->getSymbolByIdentifier(personSymbolIdp5)->getAttribute("imName") == "fakeperson5")
// 		{
// 			p5=true;
// 			break;
// 		}
// 	}
// 	}
// 	if (p6==false){
// 	int idx=0;
// 	while ((personSymbolIdp6 = worldModel->getIdentifierByType("person6", idx++)) != -1)
// 	{
// 
// 		if (idx > 4) exit(0);
// 		if (worldModel->getSymbolByIdentifier(personSymbolIdp6)->getAttribute("imName") == "fakeperson6")
// 		{
// 			p6=true;
// 			break;
// 		}
// 	}
// 	}
	
	
	
	for (int i=0;i<pn.size();i++)
	{
		
		if (pn[i]==false)
		{	

			std::string type = "person" + std::to_string(i+1);
			std::string name = "fakeperson" + std::to_string(i+1);
			
			int idx=0;
			while ((personSymbolId = worldModel->getIdentifierByType(type, idx++)) != -1)
			{
				if (idx > 4) exit(0);
				if (worldModel->getSymbolByIdentifier(personSymbolId)->getAttribute("imName") == name)
				{
					pSymbolId[i]=personSymbolId;
					pn[i]=true;
					cambiopos = true;
					break;
				}
			  
			}			
		}
		 
	}
	
	
	if (cambiopos)
	{
	  totalp.clear();
	  totalpmov.clear();
	  totalpersons.clear();

	  for (int ind=0;ind<pn.size();ind++)
	  {
		if (pn[ind])
		{
			AGMModelSymbol::SPtr personParent = worldModel->getParentByLink(pSymbolId[ind], "RT");
			AGMModelEdge &edgeRT = worldModel->getEdgeByIdentifiers(personParent->identifier, pSymbolId[ind], "RT");
			person.x = str2float(edgeRT.attributes["tx"])/1000;
			person.z = str2float(edgeRT.attributes["tz"])/1000;
			person.angle = str2float(edgeRT.attributes["ry"]);
// 			person.vel=str2float(edgeRT.attributes["velocity"]);
			
			totalpersons.push_back(person);
			
			
// 			if(person.vel>0)
// 				ppn[i]=true;
//				totalpmov.push_back(person);
// 			else
// 				ppn[i]=false;
				totalp.push_back(person);							
		}
		
		if (first)
		{
			totalaux.push_back(person);
			movperson=true;
		}
		else if  (movperson==false)
		{
			if ((totalaux[ind].x!=person.x)or(totalaux[ind].z!=person.z)or(totalaux[ind].angle!=person.angle))
				movperson = true;
			
			totalaux[ind]=person;
		}

	  }
	
	
	
	
// 	if (cambiopos==true)
// 	{
// 
// 	  
// 	  //IN THE PERSON STRUCTURE, THE DISTANCES ARE STORED IN METERS
// 
// 		if (p1){
// 
// 			AGMModelSymbol::SPtr personParentp1 = worldModel->getParentByLink(personSymbolIdp1, "RT");
// 			AGMModelEdge &edgeRTp1  = worldModel->getEdgeByIdentifiers(personParentp1->identifier, personSymbolIdp1, "RT");
// 
// 			person1.x = str2float(edgeRTp1.attributes["tx"])/1000;
// 			person1.z = str2float(edgeRTp1.attributes["tz"])/1000;
// 			person1.angle = str2float(edgeRTp1.attributes["ry"]);
// // 			person1.vel=str2float(edgeRTp1.attributes["velocity"]);
// // 			if(person1.vel>0)
// // 				pp1=true;
// // 			else
// // 				pp1=false;
// 
// 			//comprobamos si la persona se ha movido
// 			if (first){
// 				personaux1=person1;
// 				movperson=true;
// 			}
// 				else
// 			{
// 				if  (movperson==false){
// 					if ((personaux1.x!=person1.x)||(personaux1.z!=person1.z)||(personaux1.angle!=person1.angle))
// 						movperson = true;
// 				}
// 
// 				personaux1=person1;
// 			}
// 
// 
// // 			qDebug() << "------------------------------------------------------------";
// // 			qDebug() <<"PERSONA 1\n" <<"Coordenada x"<< person1.x << "Coordenada z"<< person1.z << "Rotacion "<< person1.angle;
// 		}
// 
// 		if (p2){
// 			AGMModelSymbol::SPtr personParentP2 = worldModel->getParentByLink(personSymbolIdp2, "RT");
// 			AGMModelEdge &edgeRTp2  = worldModel->getEdgeByIdentifiers(personParentP2->identifier, personSymbolIdp2, "RT");
// 
// 			person2.x=str2float(edgeRTp2.attributes["tx"])/1000;
// 			person2.z=str2float(edgeRTp2.attributes["tz"])/1000;
// 			person2.angle=str2float(edgeRTp2.attributes["ry"]);
// // 			person2.vel=str2float(edgeRTp2.attributes["velocity"]);
// // 			if(person2.vel>0)
// // 			pp2=true;
// // 			else
// // 			pp2=false;
// 			//comprobamos si la persona se ha movido
// 				if (first){
// 				personaux2=person2;
// 				movperson=true;
// 			}
// 			else
// 			{
// 				if  (movperson==false){
// 					if ((personaux2.x!=person2.x)||(personaux2.z!=person2.z)||(personaux2.angle!=person2.angle))
// 						movperson = true;
// 				}
// 
// 				personaux2=person2;
// 			}
// // 			qDebug() << "------------------------------------------------------------";
// // 			qDebug() <<"PERSONA 2\n" <<"Coordenada x"<< person2.x << "Coordenada z"<< person2.z << "Rotacion "<< person2.angle;
// 
// 			}
// 
// 		if (p3){
// 			AGMModelSymbol::SPtr personParentP3 = worldModel->getParentByLink(personSymbolIdp3, "RT");
// 			AGMModelEdge &edgeRTp3  = worldModel->getEdgeByIdentifiers(personParentP3->identifier, personSymbolIdp3, "RT");
// 
// 			person3.x=str2float(edgeRTp3.attributes["tx"])/1000;
// 			person3.z=str2float(edgeRTp3.attributes["tz"])/1000;
// 			person3.angle=str2float(edgeRTp3.attributes["ry"]);
// 			/*person3.vel=str2float(edgeRTp3.attributes["velocity"]);
// 			if(person3.vel>0)
// 			pp3=true;
// 			else
// 			pp3=false;*/			
// 			//comprobamos si la persona se ha movido
// 				if (first){
// 				personaux3=person3;
// 				movperson=true;
// 			}
// 			else
// 			{
// 				if  (movperson==false){
// 					if ((personaux3.x!=person3.x)||(personaux3.z!=person3.z)||(personaux3.angle!=person3.angle))
// 						movperson = true;
// 				}
// 
// 				personaux3=person3;
// 			}
// 
// 
// // 			qDebug() << "------------------------------------------------------------";
// // 			qDebug() <<"PERSONA 3\n" <<"Coordenada x"<< person3.x << "Coordenada z"<< person3.z << "Rotacion "<< person3.angle;
// //
// 			}
// 
//  		if (p4){
// 			AGMModelSymbol::SPtr personParentP4 = worldModel->getParentByLink(personSymbolIdp4, "RT");
// 			AGMModelEdge &edgeRTp4  = worldModel->getEdgeByIdentifiers(personParentP4->identifier, personSymbolIdp4, "RT");
// 
// 			person4.x=str2float(edgeRTp4.attributes["tx"])/1000;
// 			person4.z=str2float(edgeRTp4.attributes["tz"])/1000;
// 			person4.angle=str2float(edgeRTp4.attributes["ry"]);
// 	      //       person4.vel=str2float(edgeRTp4.attributes["velocity"]);
// 	      //       if(person4.vel>0)
// 	      //       pp4=true;
// 	      //       else
// 	      //       pp4=false;
// 			//comprobamos si la persona se ha movido
// 				if (first){
// 				personaux4=person4;
// 				movperson=true;
// 			}
// 			else
// 			{
// 				if  (movperson==false){
// 					if ((personaux4.x!=person4.x)||(personaux4.z!=person4.z)||(personaux4.angle!=person4.angle))
// 						movperson = true;
// 				}
// 
// 				personaux4=person4;
// 			}
// 			/*qDebug() << "------------------------------------------------------------";
// 			qDebug() <<"PERSONA 4\n" <<"Coordenada x"<< person4.x << "Coordenada z"<< person4.z << "Rotacion "<< person4.angle;
// 			*/
// 			}
// 
//  		if (p5){
// 			AGMModelSymbol::SPtr personParentP5 = worldModel->getParentByLink(personSymbolIdp5, "RT");
// 			AGMModelEdge &edgeRTp5  = worldModel->getEdgeByIdentifiers(personParentP5->identifier, personSymbolIdp5, "RT");
// 
// 			person5.x=str2float(edgeRTp5.attributes["tx"])/1000;
// 			person5.z=str2float(edgeRTp5.attributes["tz"])/1000;
// 			person5.angle=str2float(edgeRTp5.attributes["ry"]);
// // 			person5.vel=str2float(edgeRTp5.attributes["velocity"]);
// // 			if(person5.vel>0)
// // 			pp5=true;
// // 			else
// // 			pp5=false;
// 			//comprobamos si la persona se ha movido
// 				if (first){
// 				personaux5=person5;
// 				movperson=true;
// 			}
// 			else
// 			{
// 				if  (movperson==false){
// 					if ((personaux5.x!=person5.x)||(personaux5.z!=person5.z)||(personaux5.angle!=person5.angle))
// 						movperson = true;
// 				}
// 
// 				personaux5=person5;
// 			}
// 			/*qDebug() << "------------------------------------------------------------";
// 			qDebug() <<"PERSONA 5\n" <<"Coordenada x"<< person5.x << "Coordenada z"<< person5.z << "Rotacion "<< person5.angle;
// 			*/}
// 
// 		if (p6){
// 			AGMModelSymbol::SPtr personParentP6 = worldModel->getParentByLink(personSymbolIdp6, "RT");
// 			AGMModelEdge &edgeRTp6  = worldModel->getEdgeByIdentifiers(personParentP6->identifier, personSymbolIdp6, "RT");
// 
// 			person6.x=str2float(edgeRTp6.attributes["tx"])/1000;
// 			person6.z=str2float(edgeRTp6.attributes["tz"])/1000;
// 			person6.angle=str2float(edgeRTp6.attributes["ry"]);
// // 			person6.vel=str2float(edgeRTp6.attributes["velocity"]);
// // 			if(person6.vel>0)
// // 			pp6=true;
// // 			else
// // 			pp6=false;
// 			//comprobamos si la persona se ha movido
// 				if (first){
// 				personaux6=person6;
// 				movperson=true;
// 			}
// 			else
// 			{
// 				if  (movperson==false){
// 					if ((personaux6.x!=person6.x)||(personaux6.z!=person6.z)||(personaux6.angle!=person6.angle))
// 						movperson = true;
// 				}
// 
// 				personaux6=person6;
// 			}
// 
// // 			qDebug() << "------------------------------------------------------------";
// // 			qDebug() <<"PERSONA 6\n" <<"Coordenada x"<< person6.x << "Coordenada z"<< person6.z << "Rotacion "<< person6.angle;
// //
// 			}
// // 		agaussian(person,3.5,1.5);


	///////////////////////////OBTENER LA POSE DEL ROBOT /////////////////////////////////////
	///PARA QUE SE ACTUALICE TIENE QUE ESTAR ARRANCADO EL LOCALIZATION en el manager///

		robotSymbolId = worldModel->getIdentifierByType("robot");
		AGMModelSymbol::SPtr robotparent = worldModel->getParentByLink(robotSymbolId, "RT");
		AGMModelEdge &edgeRTrobot  = worldModel->getEdgeByIdentifiers(robotparent->identifier, robotSymbolId, "RT");

		robot.x=str2float(edgeRTrobot.attributes["tx"])/1000;
		robot.z=str2float(edgeRTrobot.attributes["tz"])/1000;
		robot.angle=str2float(edgeRTrobot.attributes["ry"]);
		/*
		qDebug() << "------------------------------------------------------------";
		qDebug() <<"ROBOT\n" <<"Coordenada x"<< robot.x << "Coordenada z"<< robot.z << "Rotacion "<< robot.angle;
		*/

		punto.x=robot.x;
		punto.z=robot.z;

// 		qDebug("Guardamos la pose del robot en el vector");
// 		qDebug("------------------------------------------");

		//Si el ultimo punto es igual que el actual no lo guardo
		if (poserobot.size()==0)
		  poserobot.push_back(punto);

		else
		  if ((poserobot[poserobot.size()-1].x!=punto.x)||(poserobot[poserobot.size()-1].z!=punto.z))
		  {

		    float  dist=sqrt((punto.x - poserobot[poserobot.size()-1].x)*(punto.x - poserobot[poserobot.size()-1].x)
		    +(punto.z - poserobot[poserobot.size()-1].z)*(punto.z - poserobot[poserobot.size()-1].z));

		    totaldist=totaldist + dist;

 		    qDebug()<<"Distancia calculada"<<dist<<"Distancia total"<<totaldist;

		    poserobot.push_back(punto);

		  }

		first = false;
		cambiopos=false;
	}




	//////LLAMAR AL TRAJECTORY////////// Solo se llama si la poscion de una perosna cambia

	if (movperson){
	qDebug ("se ha movido alguna pesona, se envia la polilinea");
	try
	{

		qDebug()<<"llamamos al trajectory";
		SNGPolylineSeq secuencia = gauss(false);
		// SNGPolylineSeq secuencia2=gausspor(false);
		SNGPolylineSeq secuenciaobj = objectInteraction(false);
		 
		 
		 RoboCompTrajectoryRobot2D::PolyLineList lista;

		 for(auto s: secuencia)
		 {
			RoboCompTrajectoryRobot2D::PolyLine poly;

			for(auto p: s)
			{
				RoboCompTrajectoryRobot2D::PointL punto = {p.x, p.z};
				poly.push_back(punto);

			}
			lista.push_back(poly);
		  }
		  

		   for(auto s: secuenciaobj)
		 {
			RoboCompTrajectoryRobot2D::PolyLine poly;

			for(auto p: s)
			{
				RoboCompTrajectoryRobot2D::PointL punto = {p.x, p.z};
				poly.push_back(punto);

			}
			lista.push_back(poly);
		  }
		 qDebug()<<"llamamos al SetHumanSpace";

		 trajectoryrobot2d_proxy->setHumanSpace(lista);
		// trajectoryrobot2d_proxy->setHumanSpace(lista2);
	}
	catch( const Ice::Exception &e)
	{ std::cout << e << std::endl;}

	movperson = false;
	}



	//actionExecution();

}
	


// double SpecificWorker::agaussian(Person person, float x, float y){
//
//
//   double sigma_h=2.0;
//   double sigma_r=1.0;
//   double sigma_s=4/3;
//
//   double alpha;
//   double nalpha;
//   double sigma;
//
//   //falta comprobar que tetha esta bien y representarlo
//
//   float tetha= PI/2 - person.angle;
//
//
//
//       alpha= atan2(y-person.z,x-person.x)- tetha + PI/2;
//       qDebug()<<"alpha"<<alpha;
//
//      nalpha= atan2(sin(alpha),cos(alpha));
//      qDebug()<<"alpha normalizado"<<nalpha;
//
//      if (nalpha<=0)
// 	sigma=sigma_r;
//      else
// 	sigma=sigma_h;
//
//
// 	double    a = pow(cos(tetha),2)/(2*pow(sigma,2)) + pow(sin(tetha),2)/(2*pow(sigma_s,2));
// 	double    b =  (sin(2*tetha))/(4*pow(sigma,2)) - (sin(2*tetha))/(4*pow(sigma_s,2));
// 	double    c = pow(sin(tetha),2)/(2*pow(sigma,2))+ pow(cos(tetha),2)/(2*pow(sigma_s,2));
//
// 	qDebug()<<"a"<<a<<"b"<<b<<"c"<<c;
//
// 	double  g = exp(-(a*pow((x - person.x),2) + 2*b*(x - person.x)*(y - person.z) + c*pow((y - person.z),2))) ;
//
//
//         qDebug()<<"El valor de la gaussiana g es "<<g;
//  return g;
//
// }


/**
 * \brief ESTE ES EL VERDADERO COMPUTE
 */
void SpecificWorker::actionExecution()
{
	QMutexLocker locker(mutex);
	qDebug() << "---------------------------------------------------";
	qDebug() <<__FUNCTION__ <<"Checking ACTION: " << QString::fromStdString(action);

	static std::string previousAction = "";
	bool newAction = (previousAction != action);

	if (newAction)
	{
		printf("prev:%s  new:%s\n", previousAction.c_str(), action.c_str());
		rDebug2(("action %s") % action.c_str() );
	}

	if (action == "changeroom")
	{
		action_ChangeRoom(newAction);
	}
	else if (action == "findobjectvisuallyintable")
	{
		action_FindObjectVisuallyInTable(newAction);
	}
	else if (action == "setobjectreach" or action == "graspobject")
	{
		action_SetObjectReach(newAction);
	}
	else if (action == "detectperson")
	{
		action_DetectPerson(newAction);
	}
	else if (action == "handobject")
	{
//		action_HandObject(newAction);
	}
	else if (action == "setstop")
	{
		action_Stop();
	}
	else if (action == "reachpose")
	{
		action_ReachPose();
	}
	else if (action == "noAction")
	{

	}
	else if (action == "handobject_offer")
	{
		action_HandObject_Offer();
	}
	else if (action == "handobject_leave")
	{
		action_HandObject_leave();
	}
	if (newAction)
	{
		previousAction = action;
		printf("New action: %s\n", action.c_str());
	}
	manageReachedPose();
// 	printf("actionExecution>>\n");
}

void SpecificWorker::action_HandObject_leave(bool newAction)
{
	try
	{
		trajectoryrobot2d_proxy->stop();
		omnirobot_proxy->setSpeedBase(0.,0.,0.0);
	}
	catch(...)
	{
		printf("Can't stop the robot!!\n");
	}
}
void SpecificWorker::action_DetectPerson(bool newAction)
{

	static bool b=false;
	if (b==false)
        {
		trajectoryrobot2d_proxy->stop();
		b=true;
	}
	try
	{
		omnirobot_proxy->setSpeedBase(0.,0.,0.1);
	}
	catch(...)
	{
		printf("Can't connect to the robot!!\n");
	}

}

//////////
// Slot
//////////

void SpecificWorker::readTrajState()
{
	try
	{
		planningState = trajectoryrobot2d_proxy->getState();
		std::cout << "State:" + planningState.state + ". Description: " + planningState.description << std::endl;
	}
	catch(const Ice::Exception &ex)
	{
		std::cout << ex << "Error talking to TrajectoryRobot2D" <<  std::endl;
	}
	catch(...)
	{
		printf("something else %d\n", __LINE__);
	}
}

void SpecificWorker::action_HandObject(bool newAction)
{
	// Get symbols' map
	std::map<std::string, AGMModelSymbol::SPtr> symbols;
	try
	{
		symbols = worldModel->getSymbolsMap(params/*,  "robot", "room", "object", "status"*/); //ALL THE SYMBOLS GIVEN IN THE RULE
	}
	catch(...)
	{
		printf("navigationAgent, action_HandObject: Couldn't retrieve action's parameters\n");
		printf("<<WORLD\n");
		AGMModelPrinter::printWorld(worldModel);
		printf("WORLD>>\n");
		if (worldModel->size() > 0) { exit(-1); }
	}

	// Get target
	int roomID, personID, robotID;
	try
	{
		if (symbols["room"].get() and symbols["person"].get() and symbols["robot"].get())
		{
			roomID = symbols["room"]->identifier;   //7 ROOM
			personID =symbols["person"]->identifier;//  PERSON
			robotID = symbols["robot"]->identifier; //1 ROBOT
		}
		else
		{
			printf("navigationAgent, action_HandObject: parameters not in the model yet\n");
			return;
		}
	}
	catch(...)
	{
		printf("navigationAgent, action_HandObject ERROR: SYMBOL DOESN'T EXIST \n");
		exit(2);
	}

	// GET THE INNERMODEL NAMES OF TH SYMBOLS
	QString robotIMID;
	QString roomIMID;
	QString personIMID;
	try
	{
		robotIMID = QString::fromStdString(worldModel->getSymbol(robotID)->getAttribute("imName"));
		roomIMID = QString::fromStdString(worldModel->getSymbol(roomID)->getAttribute("imName"));
		//we need to obtain the imName of the torso node. TrackingId+"XN_SKEL_TORSO"
		QString trackingId= QString::fromStdString(worldModel->getSymbol(personID)->getAttribute("TrackingId"));
		personIMID = trackingId +"XN_SKEL_TORSO";
	}
	catch(...)
	{
		printf("navigationAgent, action_HandObject: ERROR IN GET THE INNERMODEL NAMES\n");
		qDebug()<<"[robotIMID"<<robotIMID<<"roomIMID"<<roomIMID<<"personIMID"<<personIMID<<"]";
		exit(2);
	}

	// GET THE TARGET POSE:
	RoboCompTrajectoryRobot2D::TargetPose tgt;
	try
	{
		if (not (innerModel->getNode(roomIMID) and innerModel->getNode(personIMID)))    return;

		QVec poseInRoom = innerModel->transform6D(roomIMID, personIMID); // FROM OBJECT TO ROOM
		qDebug()<<"[robotIMID"<<robotIMID<<"roomIMID"<<roomIMID<<"personIMID"<<personIMID<<"]";
		qDebug()<<" TARGET POSE: "<< poseInRoom;

		tgt.x = poseInRoom.x();
		tgt.y = 0;
		tgt.z = poseInRoom.z();
		tgt.rx = 0;
		tgt.ry = 0;
		tgt.rz = 0;
		tgt.doRotation = false;

	}
	catch (...)
	{
		qDebug()<<"navigationAgent, action_HandObject: innerModel exception";
	}

	try
	{
// 		if (!haveTarget)
		{
			try
			{
				QVec graspRef = innerModel->transform("robot", "shellyArm_grasp_pose");
				float th=20;
				go(tgt.x, tgt.z, tgt.ry, tgt.doRotation, graspRef.x(), graspRef.z(), th);
				std::cout << "trajectoryrobot2d->go(" << currentTarget.x << ", " << currentTarget.z << ", " << currentTarget.ry << ", " << currentTarget.doRotation << ", " << graspRef.x() << ", " << graspRef.z() << " )\n";
				haveTarget = true;
			}
			catch(const Ice::Exception &ex)
			{
				std::cout <<"navigationAgent, action_HandObject: ERROR trajectoryrobot2d->go "<< ex << std::endl;
				throw ex;
			}
		}
		string state;
		try
		{
			state = trajectoryrobot2d_proxy->getState().state;
		}
		catch(const Ice::Exception &ex)
		{
			std::cout <<"navigationAgent, action_HandObject: trajectoryrobot2d->getState().state "<< ex << std::endl;
			throw ex;
		}

		//state="IDLE";
		std::cout<<state<<" haveTarget "<<haveTarget;
		if (state=="IDLE" && haveTarget)
		{
			//std::cout<<"\ttrajectoryrobot2d_proxy->getState() "<<trajectoryrobot2d_proxy->getState().state<<"\n";
			try
			{
// 				AGMModel::SPtr newModel(new AGMModel(worldModel));
// 				int statusID =symbols["status"]->identifier;
// 				newModel->getEdgeByIdentifiers(objectID, statusID, "noReach").setLabel("reach");
// 				sendModificationProposal(worldModel, newModel);
				haveTarget=false;
			}
			catch (...)
			{
				std::cout<<"\neeeee"<< "\n";
			}
		}
	}
	catch(const Ice::Exception &ex)
	{
		std::cout << ex << std::endl;
	}


}

/**
*  \brief Called when the robot is sent close to a person to offer the object
*/
void SpecificWorker::action_HandObject_Offer(bool newAction)
{
	// Get symbols' map
	std::map<std::string, AGMModelSymbol::SPtr> symbols;
	try
	{
		symbols = worldModel->getSymbolsMap(params/*,  "robot", "room", "object", "status"*/); //ALL THE SYMBOLS GIVEN IN THE RULE

	}
	catch(...)
	{
		printf("navigationAgent: Couldn't retrieve action's parameters\n");
		printf("<<WORLD\n");
		AGMModelPrinter::printWorld(worldModel);
		printf("WORLD>>\n");
		if (worldModel->size() > 0) { exit(-1); }
	}

	// Get target
	int roomID, personID, robotID;
	try
	{
		if (symbols["room"].get() and symbols["person"].get() and symbols["robot"].get() and symbols["status"].get())
		{
			roomID = symbols["room"]->identifier;
			personID =symbols["person"]->identifier;
			robotID = symbols["robot"]->identifier;

			try // If we can access the 'reach' edge for the object status the action
			{   // is not really necessary. The planner is probably replanning.
				worldModel->getEdgeByIdentifiers(personID, symbols["status"]->identifier, "reach");
				{
					static QTime lastMsg = QTime::currentTime().addSecs(-1000);
					if (lastMsg.elapsed() > 1000)
					{
						rDebug2(("navigationAgent ignoring action setHandObject_Offer (person currently reached)"));
						lastMsg = QTime::currentTime();
						return;
					}
					printf("ask the platform to stop\n");
					stop();
				}
			}
			catch(...)
			{
			}

		}
		else
		{
			printf("parameters not in the model yet\n");
			return;
		}
	}
	catch(...)
	{
		printf("ERROR: SYMBOL DOESN'T EXIST \n");
		exit(2);
	}


	// GET THE INNERMODEL NAMES OF TH SYMBOLS
	QString robotIMID;
	QString roomIMID;
	QString personIMID;
	try
	{
		robotIMID = QString::fromStdString(worldModel->getSymbol(robotID)->getAttribute("imName"));
		roomIMID = QString::fromStdString(worldModel->getSymbol(roomID)->getAttribute("imName"));
		personIMID = QString::fromStdString(worldModel->getSymbol(personID)->getAttribute("imName"));
	}
	catch(...)
	{
		printf("ERROR IN GET THE INNERMODEL NAMES\n");
		exit(2);
	}

	// GET THE TARGET POSE:
	RoboCompTrajectoryRobot2D::TargetPose tgt;
	try
	{
		if (not (innerModel->getNode(roomIMID) and innerModel->getNode(personIMID)))    return;
		QVec poseInRoom = innerModel->transform6D(roomIMID, personIMID); // FROM PERSON TO ROOM
		qDebug() << __FUNCTION__ <<" Target pose: "<< poseInRoom;

		tgt.x = poseInRoom.x();
		tgt.y = 0;
		tgt.z = poseInRoom.z();
		tgt.rx = 0;
		tgt.ry = poseInRoom.ry();
		tgt.rz = 0;
		tgt.doRotation = true;
	}
	catch (...)
	{
		qDebug()<< __FUNCTION__ << "InnerModel Exception. Element not found in tree";
	}

	// Execute target
	try
	{
		try
		{
			QVec O = innerModel->transform("shellyArm_grasp_pose", personIMID);
			QVec graspRef = innerModel->transform("robot", "shellyArm_grasp_pose");
			go(tgt.x, tgt.z, -3.141592, tgt.doRotation, graspRef.x(), graspRef.z(), 20);
			qDebug() << __FUNCTION__ << "trajectoryrobot2d->go(" << tgt.x << ", " << tgt.z << ", " << tgt.ry << ", " << graspRef.x() << ", " << graspRef.z() << " )\n";
			haveTarget = true;
		}
		catch(const RoboCompTrajectoryRobot2D::RoboCompException &ex)
		{
			std::cout << ex << " " << ex.text << std::endl;
			throw;
		}
		catch(const Ice::Exception &ex)
		{
			std::cout << ex << std::endl;
		}
	}
	catch(const Ice::Exception &ex)
	{
		std::cout << ex << std::endl;
	}


}


/**
*  \brief Called when the robot is sent close to an object's location
*/
void SpecificWorker::action_SetObjectReach(bool newAction)
{	// Get symbols' map
	std::map<std::string, AGMModelSymbol::SPtr> symbols;
	try
	{
		symbols = worldModel->getSymbolsMap(params/*,  "robot", "room", "object", "status"*/); //ALL THE SYMBOLS GIVEN IN THE RULE
	}
	catch(...)
	{
		printf("navigationAgent: Couldn't retrieve action's parameters\n");
		printf("<<WORLD\n");
		AGMModelPrinter::printWorld(worldModel);
		printf("WORLD>>\n");
		if (worldModel->size() > 0) { exit(-1); }
	}

	// Get target
	int roomID, objectID, robotID;
	try
	{
		if (symbols["room"].get() and symbols["object"].get() and symbols["robot"].get() and symbols["status"].get())
		{
			roomID = symbols["room"]->identifier;
			objectID =symbols["object"]->identifier;
			robotID = symbols["robot"]->identifier;

			try // If we can access the 'reach' edge for the object status the action
			{   // is not really necessary. The planner is probably replanning.
				worldModel->getEdgeByIdentifiers(objectID, symbols["status"]->identifier, "reach");
				{
					static QTime lastMsg = QTime::currentTime().addSecs(-1000);
					if (lastMsg.elapsed() > 1000)
					{
						rDebug2(("navigationAgent ignoring action setObjectReach (object currently reached)"));
						lastMsg = QTime::currentTime();
						return;
					}
					printf("ask the platform to stop\n");
					stop();
				}
			}
			catch(...)
			{
			}

		}
		else
		{
			printf("parameters not in the model yet\n");
			return;
		}
	}
	catch(...)
	{
		printf("ERROR: SYMBOL DOESN'T EXIST \n");
		exit(2);
	}

	// GET THE INNERMODEL NAMES OF TH SYMBOLS
	QString robotIMID;
	QString roomIMID;
	QString objectIMID;
	try
	{
		robotIMID = QString::fromStdString(worldModel->getSymbol(robotID)->getAttribute("imName"));
		roomIMID = QString::fromStdString(worldModel->getSymbol(roomID)->getAttribute("imName"));
		objectIMID = QString::fromStdString(worldModel->getSymbol(objectID)->getAttribute("imName"));

		// check if object has reachPosition
		AGMModelSymbol::SPtr object = worldModel->getSymbol(objectID);
		for (auto edge = object->edgesBegin(worldModel); edge != object->edgesEnd(worldModel); edge++)
		{
			if (edge->getLabel() == "reachPosition")
			{
				const std::pair<int32_t, int32_t> symbolPair = edge->getSymbolPair();
				objectID = symbolPair.second;
				objectIMID = QString::fromStdString(worldModel->getSymbol(objectID)->getAttribute("imName"));
				qDebug() << __FUNCTION__ << "Target object " << symbolPair.first<<"->"<<symbolPair.second<<" object "<<objectIMID;
			}
		}
	}
	catch(...)
	{
		printf("ERROR IN GET THE INNERMODEL NAMES\n");
		exit(2);
	}

	// GET THE TARGET POSE:
	RoboCompTrajectoryRobot2D::TargetPose tgt;
	try
	{
		if (not (innerModel->getNode(roomIMID) and innerModel->getNode(objectIMID)))    return;
		QVec poseInRoom = innerModel->transform6D(roomIMID, objectIMID); // FROM OBJECT TO ROOM
		qDebug() << __FUNCTION__ <<" Target pose: "<< poseInRoom;

		tgt.x = poseInRoom.x();
		tgt.y = 0;
		tgt.z = poseInRoom.z();
		tgt.rx = 0;
		tgt.ry = poseInRoom.ry();
		tgt.rz = 0;
		tgt.doRotation = true;
	}
	catch (...)
	{
		qDebug()<< __FUNCTION__ << "InnerModel Exception. Element not found in tree";
	}

	// Execute target
	try
	{
// 		if (!haveTarget)
		{
			try
			{
				QVec O = innerModel->transform("shellyArm_grasp_pose", objectIMID);
				//O.print("	O pose relativa");
				//qDebug() << __FUNCTION__ << "O norm:" << O.norm2();
				QVec graspRef = innerModel->transform("robot", "shellyArm_grasp_pose");
				float th=20;
				go(tgt.x, tgt.z, tgt.ry, tgt.doRotation, graspRef.x(), graspRef.z(), th);
				qDebug() << __FUNCTION__ << "trajectoryrobot2d->go(" << tgt.x << ", " << tgt.z << ", " << tgt.ry << ", " << graspRef.x() << ", " << graspRef.z() << " )\n";
				haveTarget = true;
			}
			catch(const RoboCompTrajectoryRobot2D::RoboCompException &ex)
			{
				std::cout << ex << " " << ex.text << std::endl;
				throw;
			}
			catch(const Ice::Exception &ex)
			{
				std::cout << ex << std::endl;
			}
		}
		string state;
		try
		{
			state = trajectoryrobot2d_proxy->getState().state;
		}
		catch(const Ice::Exception &ex)
		{
			std::cout <<"trajectoryrobot2d->getState().state "<< ex << std::endl;
			throw ex;
		}

		//state="IDLE";
		std::cout<<state<<" haveTarget "<<haveTarget;
		if (state=="IDLE" && haveTarget)
		{
			//std::cout<<"\ttrajectoryrobot2d_proxy->getState() "<<trajectoryrobot2d_proxy->getState().state<<"\n";
			try
			{
// 				AGMModel::SPtr newModel(new AGMModel(worldModel));
// 				int statusID =symbols["status"]->identifier;
// 				newModel->getEdgeByIdentifiers(objectID, statusID, "noReach").setLabel("reach");
// 				sendModificationProposal(worldModel, newModel);
				haveTarget=false;
			}
			catch (...)
			{
				std::cout<<"\neeeee"<< "\n";
			}
		}
	}
	catch(const Ice::Exception &ex)
	{
		std::cout << ex << std::endl;
	}
}

void SpecificWorker::manageReachedPose()
{
	float schmittTriggerThreshold = 20;
	float THRESHOLD_POSE = 100;
	std::string m ="  ";

	bool changed = false;

	QMutexLocker locker(mutex);

	AGMModel::SPtr newModel(new AGMModel(worldModel));

	for (AGMModel::iterator symbol_itr=newModel->begin(); symbol_itr!=newModel->end(); symbol_itr++)
	{
		AGMModelSymbol::SPtr node = *symbol_itr;
		if (node->symboltype() == "pose")
		{
			/// Compute distance and new state
			float d2n;
			try
			{
				QVec arm = innerModel->transformS("world", "robot");
				QVec obj = innerModel->transformS("world", node->getAttribute("imName"));
				(arm-obj).print("error");

				d2n = distanceToNode("robot", newModel, node);
qDebug()<<"distance "<<d2n;
			}
			catch(...)
			{
				printf("Ref: robot: %p\n", (void *)innerModel->getNode("robot"));
				printf("Pose: %s: %p\n", node->getAttribute("imName").c_str(), (void *)innerModel->getNode(node->getAttribute("imName").c_str()));
				exit(1);
			}


			for (AGMModelSymbol::iterator edge_itr=node->edgesBegin(newModel); edge_itr!=node->edgesEnd(newModel); edge_itr++)
			{
				AGMModelEdge &edge = *edge_itr;
				if (edge->getLabel() == "reach" and d2n > THRESHOLD_POSE+schmittTriggerThreshold )
				{
					edge->setLabel("noReach");
					printf("pose %d STOPS REACH\n", node->identifier);
					m += " action " + action + " edge->toString() "+ edge->toString(newModel);
					changed = true;
					rDebug2(("pose %d no-reach") % node->identifier);
				}
				else if (edge->getLabel() == "noReach" and d2n < THRESHOLD_POSE/*-schmittTriggerThreshold*/)
				{
					edge->setLabel("reach");
					printf("___ %s ___\n", edge->getLabel().c_str());
					printf("pose %d STARTS REACH\n", node->identifier);
					m += " action " + action + " edge->toString() "+ edge->toString(newModel);
					changed = true;
					rDebug2(("pose %d reach") % node->identifier);
				}
			}
		}
	}

	/// Publish new model if changed
	if (changed)
	{
		printf("PUBLISH!!!! version%d\n", newModel->version);
		sendModificationProposal(newModel, worldModel, m);
	}
}


float SpecificWorker::distanceToNode(std::string reference_name, AGMModel::SPtr model, AGMModelSymbol::SPtr object)
{
	QMutexLocker locker(mutex);

	QVec arm = innerModel->transformS("world", reference_name);
	arm(1) = 0;
	QVec obj = innerModel->transformS("world", object->getAttribute("imName"));
	obj(1) = 0;
	return (arm-obj).norm2();

}

int32_t SpecificWorker::getIdentifierOfRobotsLocation(AGMModel::SPtr &worldModel)
{
	for (AGMModel::iterator symbol_it=worldModel->begin(); symbol_it!=worldModel->end(); symbol_it++)
	{
		const AGMModelSymbol::SPtr &symbol = *symbol_it;
		if (symbol->symbolType == "robot")
		{
			for (AGMModelSymbol::iterator edge_it=symbol->edgesBegin(worldModel); edge_it!=symbol->edgesEnd(worldModel); edge_it++)
			{
				AGMModelEdge edge = *edge_it;
				if (edge.linking == "in")
				{
					return edge.symbolPair.second;
				}
			}
		}
	}

	printf("wheres's the robot?\n");
	return -1;
}

void SpecificWorker::setIdentifierOfRobotsLocation(AGMModel::SPtr &model, int32_t identifier)
{
	bool didSomethin = false;
	for (AGMModel::iterator symbol_it=worldModel->begin(); symbol_it!=worldModel->end(); symbol_it++)
	{
		const AGMModelSymbol::SPtr &symbol = *symbol_it;
		if (symbol->symbolType == "robot")
		{

			for (AGMModelSymbol::iterator edge_it=symbol->edgesBegin(worldModel); edge_it!=symbol->edgesEnd(worldModel); edge_it++)
			{
				if (edge_it->linking == "in")
				{
					printf("it was %d\n", edge_it->symbolPair.second);
				}
			}
			for (int32_t edgeIndex=0; edgeIndex<model->numberOfEdges(); edgeIndex++)
			{
				if (model->edges[edgeIndex].linking == "in")
				{
					if (model->edges[edgeIndex].symbolPair.first == symbol->identifier)
					{
						model->edges[edgeIndex].symbolPair.second = identifier;
						didSomethin = true;
					}
				}
			}
			for (AGMModelSymbol::iterator edge_it=symbol->edgesBegin(worldModel); edge_it!=symbol->edgesEnd(worldModel); edge_it++)
			{
				if (edge_it->linking == "in")
				{
					printf("now is %d\n", edge_it->symbolPair.second);
				}
			}
		}
	}
	if (not didSomethin)
		qFatal("couldn't update robot's room in the cog graph");
}

void SpecificWorker::action_ChangeRoom(bool newAction)
{
	static float lastX = std::numeric_limits<float>::quiet_NaN();
	static float lastZ = std::numeric_limits<float>::quiet_NaN();

	auto symbols = worldModel->getSymbolsMap(params, "r2", "robot");


	try
	{
		printf("trying to get _in_ edge from %d to %d\n", symbols["robot"]->identifier, symbols["r2"]->identifier);
		AGMModelEdge e = worldModel->getEdge(symbols["robot"], symbols["r2"], "in");
		printf("STOP! WE ALREADY GOT THERE!\n");
		stop();
		return;
	}
	catch(...)
	{
	}

	int32_t roomId = symbols["r2"]->identifier;
	printf("room symbol: %d\n",  roomId);
	std::string imName = symbols["r2"]->getAttribute("imName");
	printf("imName: <%s>\n", imName.c_str());

	const float refX = str2float(symbols["r2"]->getAttribute("x"));
	const float refZ = str2float(symbols["r2"]->getAttribute("z"));

	QVec roomPose = innerModel->transformS("world", QVec::vec3(refX, 0, refZ), imName);
	roomPose.print("goal pose");
	// 	AGMModelSymbol::SPtr goalRoom = worldModel->getSymbol(str2int(params["r2"].value));
	// 	const float x = str2float(goalRoom->getAttribute("tx"));
	// 	const float z = str2float(goalRoom->getAttribute("tz"));
	const float x = roomPose(0);
	const float z = roomPose(2);

	bool proceed = true;
	if ( (planningState.state=="PLANNING" or planningState.state=="EXECUTING") )
	{
		if (abs(lastX-x)<10 and abs(lastZ-z)<10)
			proceed = false;
		else
			printf("proceed because the coordinates differ (%f, %f), (%f, %f)\n", x, z, lastX, lastZ);
	}
	else
	{
		printf("proceed because it's stoped\n");
	}

	if (proceed)
	{
		lastX = x;
		lastZ = z;
		printf("changeroom to %d\n", symbols["r2"]->identifier);
		go(x, z, 0, false, 0, 0, 1200);
	}
	else
	{
	}
}

void SpecificWorker::action_Stop(bool newAction)
{
	stop();
}

void SpecificWorker::action_ReachPose(bool newAction)
{
	printf("action_ReachPose,%d: %d\n", __LINE__, newAction);
	static float lastX = std::numeric_limits<float>::quiet_NaN();
	printf("action_ReachPose,%d: %d\n", __LINE__, newAction);
	static float lastZ = std::numeric_limits<float>::quiet_NaN();
	printf("action_ReachPose,%d: %d\n", __LINE__, newAction);

	auto symbols = worldModel->getSymbolsMap(params, "room", "pose");
	printf("action_ReachPose,%d: %d\n", __LINE__, newAction);

	int32_t poseId = symbols["pose"]->identifier;
	printf("pose symbol: %d\n",  poseId);
	std::string imName = symbols["pose"]->getAttribute("imName");
	printf("imName: <%s>\n", imName.c_str());

	QVec pose = innerModel->transform6D("world", QString::fromStdString(imName));
	pose.print("goal pose");
	const float x = pose(0);
	const float z = pose(2);
	const float ry = pose(4);

	bool proceed = true;
	if ( (planningState.state=="PLANNING" or planningState.state=="EXECUTING") )
	{
		if (abs(lastX-x)<10 and abs(lastZ-z)<10)
			proceed = false;
		else
			printf("proceed because the coordinates differ (%f, %f), (%f, %f)\n", x, z, lastX, lastZ);
	}
	else
	{
		printf("proceed because it's stoped\n");
	}

	if (proceed)
	{
		lastX = x;
		lastZ = z;
		printf("setpose %d\n", symbols["room"]->identifier);
		go(x, z, ry, true);
	}
	else
	{
	}
}

void SpecificWorker::action_FindObjectVisuallyInTable(bool newAction)
{
// 	stop();


	static float lastX = std::numeric_limits<float>::quiet_NaN();
	static float lastZ = std::numeric_limits<float>::quiet_NaN();

	AGMModelSymbol::SPtr goalTable;
	AGMModelSymbol::SPtr robot;
	int32_t tableId = -1;
	try
	{
		tableId = str2int(params["container"].value);
		goalTable = worldModel->getSymbol(tableId);
		robot = worldModel->getSymbol(worldModel->getIdentifierByType("robot"));
	}
	catch(...)
	{
		printf("can't access robot or table\n");
		return;
	}

	const float x = str2float(goalTable->getAttribute("tx"));
	const float z = str2float(goalTable->getAttribute("tz"));
	float alpha = tableId==7?-3.141592:0;
// printf("%s: %d\n", __FILE__, __LINE__);

	const float rx = str2float(robot->getAttribute("tx"));
	const float rz = str2float(robot->getAttribute("tz"));
	const float ralpha = str2float(robot->getAttribute("alpha"));
// printf("%s: %d\n", __FILE__, __LINE__);

	// Avoid repeating the same goal and confuse the navigator
	const float errX = abs(rx-x);
	const float errZ = abs(rz-z);
	float errAlpha = abs(ralpha-alpha);
	while (errAlpha > +M_PIl) errAlpha -= 2.*M_PIl;
	while (errAlpha < -M_PIl) errAlpha += 2.*M_PIl;
	errAlpha = abs(errAlpha);
	if (errX<20 and errZ<20 and errAlpha<0.02)
		return;
// printf("%s: %d\n", __FILE__, __LINE__);

	bool proceed = true;
	if ( (planningState.state=="IDLE")) //PLANNING" or planningState.state=="EXECUTING") )
	{
		if (abs(lastX-x)<10 and abs(lastZ-z)<10)
			proceed = false;
		else
			printf("proceed because the coordinates differ (%f, %f), (%f, %f)\n", x, z, lastX, lastZ);
	}
	else
	{
		printf("proceed because it's stoped\n");
	}

	if (proceed)
	{
		lastX = x;
		lastZ = z;
		printf("find objects in table %d\n", tableId);
		go(x, tableId==7?z+550:z-550, tableId==7?-3.141592:0, true, 0, 500, 500);
	}
	else
	{
	}
}

void SpecificWorker::action_NoAction(bool newAction)
{
	try{
		std::string state = trajectoryrobot2d_proxy->getState().state;
		if (state != "IDLE")
		{
			printf("trajectoryrobot2d state : %s\n", state.c_str());
			stop();
		}
	}catch(...)
	{
		std::cout<< "Error retrieving trajectory state"<<std::endl;
	}
}


// Send target to trajectory
void SpecificWorker::go(float x, float z, float alpha, bool rot, float xRef, float zRef, float threshold)
{
	static bool first = true;

	RoboCompTrajectoryRobot2D::TargetPose lastTarget = currentTarget;

	currentTarget.x = x;
	currentTarget.z = z;
	currentTarget.y = 0;
	currentTarget.rx = 0;
	currentTarget.rz = 0;

	if (rot)
	{
		currentTarget.ry = alpha;
		currentTarget.doRotation = true;
	}
	else
	{
		currentTarget.ry = 0;
		currentTarget.doRotation = false;
	}


	if ((not first) and (lastTarget == currentTarget))
		return;

	try
	{
		std::cout<< "ENVIANDO A trajectoryrobot2d->go(" << currentTarget.x << ", " << currentTarget.z << ", " << currentTarget.ry << ", " << xRef << ", " << zRef << threshold << " )" <<std::endl;
		trajectoryrobot2d_proxy->goReferenced(currentTarget, xRef, zRef, threshold);
	}
	catch(const RoboCompTrajectoryRobot2D::RoboCompException &ex)
	{
		std::cout << ex.text << std::endl;
	}
	catch(const Ice::Exception &ex)
	{
		std::cout << ex << std::endl;
	}
	catch(...)
	{
		printf("Exception: something else %d\n", __LINE__);
	}
}


void SpecificWorker::stop()
{
	try
	{
		trajectoryrobot2d_proxy->stop();
	}
	catch(const Ice::Exception &ex)
	{
		std::cout << ex << std::endl;
	}
	catch(...)
	{
		printf("Exception: something else %d\n", __LINE__);
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
	innerModel = AGMInner::extractInnerModel(worldModel, "world", true);

	cambiopos=true;

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
	cambiopos=true;
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
	cambiopos=true;

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
		AGMMisc::publishModification(newModel, agmexecutive_proxy, std::string( "graspingAgent")+m);
	}
	catch(const RoboCompAGMExecutive::Locked &e)
	{
	}
	catch(const RoboCompAGMExecutive::OldModel &e)
	{
	}
	catch(const RoboCompAGMExecutive::InvalidChange &e)
	{
	}
	catch(const Ice::Exception& e)
	{
		exit(1);
	}
}



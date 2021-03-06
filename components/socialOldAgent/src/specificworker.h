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
#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <python2.7/Python.h>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <QFile>
#include <innermodel/innermodel.h>
#include <boost/format.hpp>
#define THRESHOLD 40

/**
       \brief
       @author authorname
*/

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:


	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	//bool para indicar si se ha movido la persona, lo utilizare para imprimir la coordenada de la persona cada vez que se mueva
	bool cambiopos=false;

	SNGPolylineSeq secuencia,secuencia2,secuenciaObj;

	//ESTRUCTURA PERSONA FORMADA POR ANGULO, POS X,POS Z
	SNGPersonSeq totalpersons; //este es para leer el fichero

	SNGPerson robot;
	
	SNGObject object;
	SNGObjectSeq objects;
	//estas guardaran la posicion anterior de cada persona, para comprobar si se ha movido

	//bool para saber si se ha movido alguna persona
	bool movperson = false;

	//PARA ALMACENAR EL SIMBOLO DE LA PERSONA AL LEERLA DEL MUNDO
	int32_t robotSymbolId;
	
	vector <bool> pn = {false,false,false,false,false,false}; 
	vector <bool> ppn = {false,false,false,false,false,false}; 
	
	int32_t personSymbolId;
	int32_t pSymbolId[6];
	SNGPerson person;
	SNGPerson personaux;
	SNGPersonSeq totalp;
	SNGPersonSeq totalpmov;
	SNGPersonSeq totalaux;
	
	
	int32_t objectSymbolId;

	
	bool staticperson = false;
	bool movingperson = false;


	//RoboCompInnerModelManager::Pose3D pose;
	//PARA GUARDAR LOS DATOS EN UN ARCHIVO

	struct Punto {
	  float x;
	  float z;
	};
	Punto punto;
	vector <Punto> poserobot;

	//PARA LEER EL VALOR DEL SLIDER
	int valorprox = 0;


	//PARA GUARDAR LA DISTANCIA RECORRIDA
	float totaldist=0;

	void addObjects();
      


	//////////////
	/// SERVANTS
	//////////////
	bool activateAgent(const ParameterMap& prs);
	bool deactivateAgent();
	StateStruct getAgentState();
	ParameterMap getAgentParameters();
	bool setAgentParameters(const ParameterMap& prs);
	void  killAgent();
	Ice::Int uptimeAgent();
	bool reloadConfigAgent();

	void  structuralChange(const RoboCompAGMWorldModel::World & modification);
	void  symbolUpdated(const RoboCompAGMWorldModel::Node& modification);
	void  symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence & modification);
	void  edgeUpdated(const RoboCompAGMWorldModel::Edge& modification);
	void edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications);

	//double agaussian(SNGPerson person, float x, float y);

public slots:
 	void compute();
	void readTrajState();
	SNGPolylineSeq gauss(bool dibujar=true);
	SNGPolylineSeq gausspor(bool dibujar=true);
	SNGPolylineSeq objectInteraction(bool dibujar = true);
	void cambiarvalor(int valor);
	void grabarfichero();

private:
	bool setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated);
	bool active;
	void sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel,std::string m);
	void includeMovementInRobotSymbol(AGMModelSymbol::SPtr robot);




	void go(float x, float z, float alpha=0, bool rot=false, float xRef=0, float zRef=0, float threshold=200);
	void stop();

	void actionExecution();
	int32_t getIdentifierOfRobotsLocation(AGMModel::SPtr &worldModel);
	void setIdentifierOfRobotsLocation(AGMModel::SPtr &worldModel, int32_t identifier);

private:
	RoboCompInnerModelManager::Pose3D pose;
	std::string action;
	ParameterMap params;
	AGMModel::SPtr worldModel;
	InnerModel *innerModel;
	bool haveTarget;
	QTimer trajReader;



	RoboCompTrajectoryRobot2D::NavState planningState;


	// Target info
	RoboCompTrajectoryRobot2D::TargetPose currentTarget;

	void manageReachedPose();
	float distanceToNode(std::string reference_name, AGMModel::SPtr model, AGMModelSymbol::SPtr object);
private:
	void action_WaitingToAchieve();
	void action_Stop(bool newAction = true);
	void action_ReachPose(bool newAction = true);
	void action_ChangeRoom(bool newAction = true);
	void action_FindObjectVisuallyInTable(bool newAction = true);
	void action_SetObjectReach(bool newAction = true);
//	void action_GraspObject(bool newAction = true);
	void action_DetectPerson (bool newAction = true);
	void action_HandObject(bool newAction = true);
	void action_NoAction(bool newAction = true);
	void action_HandObject_Offer(bool newAction = true);
	void action_HandObject_leave(bool newAction = true);
//CHECK
	//void updateRobotsCognitiveLocation();
//	std::map<int32_t, QPolygonF> roomsPolygons;
//	std::map<int32_t, QPolygonF> extractPolygonsFromModel(AGMModel::SPtr &worldModel);
//	RoboCompOmniRobot::TBaseState bState;

};


class TimedList
{
	class TimedDatum
	{
	public:
		 TimedDatum(float d)
		{
			datum = d;
			datum_time = QTime::currentTime();
		}
		float datum;
		QTime datum_time;
	};

public:
	TimedList(float msecs)
	{
		maxMSec = msecs;
	}
	void add(float datum)
	{
		data.push_back(TimedDatum(datum));
	}
	float getSum()
	{
		while (data.size()>0)
		{
			if (data[0].datum_time.elapsed() > maxMSec)
				data.pop_front();
			else
				break;
		}
		float acc = 0.;
		for (int i=0; i<data.size(); i++)
			acc += data[i].datum;
		return acc;
	}
private:
	float maxMSec;
	QList<TimedDatum> data;
};

#endif

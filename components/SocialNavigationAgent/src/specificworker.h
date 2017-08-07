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
#include "pathfinder.h"
#include <actionexecution.h>


//PROBLEMA: con python 3.5 da error al compilar

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
	bool changepos=false;
	
	SNGPolylineSeq sequence;
	//ESTRUCTURA PERSONA FORMADA POR ANGULO, POS X,POS Z
	SNGPersonSeq totalpersons; //este es para leer el fichero
	SNGPerson robot;	
	vector <bool> pn = {false,false,false,false,false,false};  // to check if the person is in the world	
	vector <bool> ppn = {false,false,false,false,false,false};  // if the person is moving
	int32_t personSymbolId;
	int32_t pSymbolId[6];
	
	SNGPerson person;
	SNGPerson personaux;
	SNGPersonSeq totalp; // quiet person
	SNGPersonSeq totalpmov; //moving person
	SNGPersonSeq totalaux; //to check if the person has changed its position
	
	//bool para saber si se ha movido alguna persona
	bool movperson = false;
	int32_t robotSymbolId;	

	struct Point { //PARA GUARDAR LOS DATOS EN UN ARCHIVO
	  float x;
	  float z;
	};
	Point point;
	vector <Point> poserobot;
	
	//PARA LEER EL VALOR DEL SLIDER
	int prox = 0;

	//PARA GUARDAR LA DISTANCIA RECORRIDA
	float totaldist=0;
	
	//PARA COMPROBAR SI DOS PERSONAS ESTAN HABLANDO. ya no se usa
	bool conversation = false;
	bool checkconversation();

	int i = 0;
	
	//Class ActionExecution
	ActionExecution aE;

	///////////////////////////////////////////////////////////////////////////
	/// SERVANTS
	//////////////////////////////////////////////////////////////////////////
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

	NavState getState(){ return NavState(); };
	float goBackwards(const TargetPose &target){};
	void stop(){};
	void setHumanSpace(const PolyLineList &polyList){};
	float goReferenced(const TargetPose &target, const float xRef, const float zRef, const float threshold){ pathfinder.go(target.x, target.z); };
	float changeTarget(const TargetPose &target){};
	void mapBasedTarget(const NavigationParameterMap &parameters){};
	float go(const TargetPose &target){ pathfinder.go(target.x, target.z); };

	
public slots:
 	void compute();
	//void readTrajState();
	SNGPolylineSeq gauss(bool draw=true);
	void changevalue(int value);
	void savedata();
	void UpdateInnerModel(SNGPolylineSeq seq);

private:
	bool setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated);
	bool active;
	void sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel,std::string m);
	
private:
	std::string action;
	ParameterMap params;
	AGMModel::SPtr worldModel;
	InnerModel *innerModel;
	bool haveTarget;
	QTimer trajReader;
	InnerModel *inner;
	AGMModel::SPtr world;	
	RoboCompTrajectoryRobot2D::NavState planningState;
	// Target info
	RoboCompTrajectoryRobot2D::TargetPose currentTarget;
	robocomp::pathfinder::PathFinder pathfinder;
	
	
//CHECK
	//void updateRobotsCognitiveLocation();
//	std::map<int32_t, QPolygonF> roomsPolygons;
//	std::map<int32_t, QPolygonF> extractPolygonsFromModel(AGMModel::SPtr &worldModel);	
//	RoboCompOmniRobot::TBaseState bState;
	
};


#endif



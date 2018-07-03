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
#ifndef SOCIALRULES_H
#define SOCIALRULES_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <boost/format.hpp>
#include <QObject>
#include <vector>
#include "pathfinder.h"


class SocialRules :public QObject
{
Q_OBJECT
public:
	using InnerPtr = std::shared_ptr<InnerModel>;
	
	void initialize(SocialNavigationGaussianPrx socialnavigationgaussian_proxy_,AGMExecutivePrx agmexecutive_proxy_,QMutex *mutex_,robocomp::pathfinder::PathFinder *pathfinder_,AGMModel::SPtr worldModel_, InnerPtr innerModel_);
	

	SocialNavigationGaussianPrx socialnavigationgaussian_proxy;
	AGMExecutivePrx agmexecutive_proxy;
	
	float h = 0.1; 
	QMutex *mux;

	SNGPersonSeq quietperson; // quiet person
	SNGPersonSeq movperson; //moving person
	SNGPersonSeq totalperson;
	
	SNGPolylineSeq sequence, sequence2, sequenceObj;
	
	SNGObjectSeq objects;
	SNGPolylineSeq seq;
	
	int32_t objectSymbolId;
	
	SNGPerson person;
	vector <int32_t> personSymbolId = {};
	
	SNGPolylineSeq ApplySocialRules(SNGPersonSeq tperson);
	void  structuralChange(const RoboCompAGMWorldModel::World & modification);
	bool checkHRI(SNGPerson p, int ind, InnerPtr i, AGMModel::SPtr w);
	void change_hvalue(float value);
	
public slots:
	void goToPerson();
	SNGPolylineSeq calculateGauss(bool draw = true);
	SNGPolylineSeq PassOnRight(bool draw = true);
	SNGPolylineSeq objectInteraction(bool d = true);
	
private:
	AGMModel::SPtr worldModel;
	InnerPtr innerModel;
public:
	robocomp::pathfinder::PathFinder *pathfinder;

};


#endif // SOCIALRULES_H

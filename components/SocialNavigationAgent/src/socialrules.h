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



class SocialRules :public QObject
{
Q_OBJECT
public:
	SocialRules();
	~SocialRules();
	
	SocialNavigationGaussianPrx socialnavigationgaussian_proxy;
	
	int prox = 0; //reading the slider
	

	SNGPersonSeq quietperson; // quiet person
	SNGPersonSeq movperson; //moving person
	SNGPersonSeq totalperson;
	
	SNGPolylineSeq sequence, sequence2, sequenceObj;
	
	SNGObject object;
	SNGObjectSeq objects;

	RoboCompTrajectoryRobot2D::PolyLineList ApplySocialRules(SNGPersonSeq tperson);
	
	
public slots:
  	void changevalue(int value);
	SNGPolylineSeq gauss(bool draw=true);
	SNGPolylineSeq PassOnRight(bool draw=true);
	SNGPolylineSeq objectInteraction(bool d = true);

};


#endif // SOCIALRULES_H

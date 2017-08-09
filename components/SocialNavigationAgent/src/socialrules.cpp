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

#include "socialrules.h"

SocialRules::SocialRules()
{

}

SocialRules::~SocialRules()
{

}
/**
* \brief Change the slider's value
*/
void SocialRules::changevalue(int value)
{
	prox=value;
	qDebug()<<"Proximity"<<prox;
}

/**
* \brief If the person is in the model it is added to a vector of persons wich is sent to the socialnavigationGaussian
* to model its personal space. 
* The function returns a sequence of polylines.
*/

SNGPolylineSeq SocialRules::gauss(bool draw)
{
	if (staticperson)
	{  
		sequence.clear();
		sequence = socialnavigationgaussian_proxy-> getPersonalSpace(totalp, prox, draw);
	}
	return sequence;
}


SNGPolylineSeq SocialRules::PassOnRight(bool draw)
{


	if (movingperson)
	{
		sequence2.clear();
		sequence2 = socialnavigationgaussian_proxy-> getPassOnRight(totalpmov, prox, draw);
	}	    
	return sequence2;

	
}

SNGPolylineSeq SocialRules::objectInteraction(bool d)
{
	//This function has to change, I want to insert the objects in the AGM and read them here
	objects.clear();
	
	///cafetera
	object.x =0.450;
	object.z =-2.250;
	object.angle=3.1415926535/2;
	object.space =0.75;
	    
	objects.push_back(object);
	///nevera
	object.x =3.210;
	object.z =-3.700;
	object.angle=0;
	object.space=1.0;
	
	objects.push_back(object);
	
	///tablon
	object.x =5.360;
	object.z =-0.270;
	object.angle=3.1415926535;
	object.space=1.5;
	
	objects.push_back(object);
	
	///telefono
	object.x =3.000;
	object.z =3.650;
	object.angle=3.1415926535;
	object.space = 0.75;
	
	objects.push_back(object);
	

	sequenceObj.clear();
	sequenceObj =socialnavigationgaussian_proxy->getObjectInteraction(totalperson,objects,d);
	

	return sequenceObj;
	
}

RoboCompTrajectoryRobot2D::PolyLineList SocialRules::ApplySocialRules(SNGPersonSeq tperson)
{
	totalperson=tperson;
	//for each person check if the velocity is 0, if it is add to totalp, if not add to totalpmov
  

}


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

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params) 
{ 
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
	return true;
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
	timer.start(Period); 
	return true;
 };


void SpecificWorker::compute( ) 
{ 
	int num;
	char input[0]; 
	std::cout << " Enter \n 1: forward \n 3: left \n 2: backward \n 4: right" << endl; 
	std::cin >> input; 
	istringstream ( input ) >> num; 
	

if (num == 1 )
{
    omnirobot_proxy->setSpeedBase(0,300,0); 
    usleep(750000);
    omnirobot_proxy->setSpeedBase(0,0, 0); 
    usleep(500000);
}
if (num == 2)
{
    omnirobot_proxy->setSpeedBase(0,-300, 0); 
    usleep(750000);
    omnirobot_proxy->setSpeedBase(0,0, 0); 
    usleep(500000);
}
if (num == 3)
{
    omnirobot_proxy->setSpeedBase(0,0,-0.8); 
    usleep(1000000);
    omnirobot_proxy->setSpeedBase(0,0, 0); 
    usleep(500000);
}
if (num == 4)
{
    omnirobot_proxy->setSpeedBase(0, 0,0.8); 
    usleep(1000000);
    omnirobot_proxy->setSpeedBase(0,0, 0); 
    usleep(500000);
}
else
{
    std::cout << "invalid entry" << endl;
}
}






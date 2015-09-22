/*
 *    Copyright (C) 2015 by YOUR NAME HERE
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
	goMotorsGO         = false;
	stateMachine       = State::IDLE;
	abortCorrection    = false;
	innerModel         = NULL;
	contador           = 0;
	timeSinMarca       = 0.0;
	mutexSolved        = new QMutex(QMutex::Recursive);
	mutexRightHand     = new QMutex(QMutex::Recursive);
	firstCorrection    = QVec::zeros(3);
	
#ifdef USE_QTGUI
	connect(this->goButton, SIGNAL(clicked()), this, SLOT(goYESButton()));
	innerViewer        = NULL;
	osgView            = new OsgView(this->widget);
 	show();
#endif
	
	QMutexLocker ml(mutex);
	INITIALIZED        = false;
}
/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
}


bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("InnerModel") ;
		if( QFile(QString::fromStdString(par.value)).exists() == true)
		{
			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Reading Innermodel file " << QString::fromStdString(par.value);
			innerModel = new InnerModel(par.value);
		}
		else
			qFatal("Exiting now.");
	}
	catch(std::exception e) { qFatal("Error reading Innermodel param");}

#ifdef USE_QTGUI
	if (innerViewer)
	{
		osgView->getRootGroup()->removeChild(innerViewer);
		delete innerViewer;
	}
	innerViewer = new InnerModelViewer(innerModel, "root", osgView->getRootGroup(), true);
#endif
	InnerModelNode *nodeParent = innerModel->getNode("root");
	if( innerModel->getNode("target") == NULL)
	{
		InnerModelTransform *node = innerModel->newTransform("target", "static", nodeParent, 0, 0, 0,        0, 0., 0,      0.);
		nodeParent->addChild(node);
	}
	rightHand = new VisualHand(innerModel, "grabPositionHandR");	
	timer.start(Period);		
	QMutexLocker ml(mutex);
	INITIALIZED = true;
	
	return true;
}



void SpecificWorker::compute()
{
#ifdef USE_QTGUI
		if (innerViewer)
		{
			innerViewer->update();
			osgView->autoResize();
			osgView->frame();
		}
#endif
	updateInnerModel_motors_target_and_visual();
	QMutexLocker ml(mutex);
	switch(stateMachine)
	{
		case State::IDLE:
			if (currentTarget.getState() == Target::State::WAITING)
			{
				stateMachine     = State::INIT_BIK;
				abortCorrection = false;
				qDebug()<<"Ha llegado un TARGET: "<<currentTarget.getPose();
				timeSinMarca = 0.0;
			}
		break;
		//---------------------------------------------------------------------------------------------
		case State::INIT_BIK:
			applyFirstApproximation();
			currentTarget.setState(Target::State::IN_PROCESS);
			correctedTarget = currentTarget;
			stateMachine    = State::WAIT_BIK;
		break;
		//---------------------------------------------------------------------------------------------
		case State::WAIT_BIK:
			// Wait for IK's lower levels to start corrections
			if (inversekinematics_proxy->getTargetState(currentTarget.getBodyPart(), currentTarget.getID_IK()).finish == true)
			{
				qDebug()<<"---> El IK ha terminado.";
				stateMachine = State::CORRECT_ROTATION;
			}
		break;
		//---------------------------------------------------------------------------------------------
		case State::CORRECT_ROTATION:

			if (inversekinematics_proxy->getTargetState(correctedTarget.getBodyPart(), correctedTarget.getID_IK()).finish == false) return;
			//updateMotors(inversekinematics_proxy->getTargetState(correctedTarget.getBodyPart(), correctedTarget.getID_IK()).motors);
			if (correctPose()==true or abortCorrection==true)
			{
				if(nextTargets.isEmpty()==false)
				{
					std::cout<<"--> Correccion completada.\nPasamos al siguiente target:\n";
					currentTarget = nextTargets.head();
					nextTargets.dequeue();
				}
				else
					currentTarget.setState(Target::State::IDLE);
				stateMachine = State::IDLE;
				//goHome("RIGHTARM");
			}
		break;
		//---------------------------------------------------------------------------------------------
		default:
			break;
	}
}


/**
 *\brief This method returns the state of the part.
 * @return bool: If the part hasn't got more pending targets, it returns TRUE, else it returns FALSE
 */
bool SpecificWorker::getPartState(const string &bodyPart)
{
	//return inversekinematics_proxy->getPartState(bodyPart);
	return (nextTargets.isEmpty() and currentTarget.getState()==Target::State::IDLE);;
}


/**
 * \brief this method returns the state of a determinate target.
 * @param part part of the robot that resolves the target
 * @param targetID target identifier
 * @return TargetState
 */
TargetState SpecificWorker::getTargetState(const string &bodyPart, const int targetID)
{
	//return inversekinematics_proxy->getTargetState(bodyPart, targetID);
	QMutexLocker ml(mutexSolved);
	RoboCompInverseKinematics::TargetState 	state;
	state.finish = false;
	
	for(int i=0; i<solvedList.size(); i++)
	{
		if(bodyPart=="RIGHTARM" and targetID==solvedList[i].getID_VIK())
		{
			state.finish = true;
			if(solvedList[i].getState()==Target::State::NOT_RESOLVED) state.state = "NOT_RESOLVED";
			if(solvedList[i].getState()==Target::State::RESOLVED)     state.state = "RESOLVED";
		}
	}
	return state;
}


/**
 * \brief This method reimplements the interface method setTargetPose6D of the inversekinematics component
 * in order to aply the visual correction
 * @param bodyPart name of the body part.
 * @param target   pose of the goal point.
 * @param weights  weights of each traslation and rotation components.
 * @return the identifier of the target (an int)
 */
int SpecificWorker::setTargetPose6D(const string &bodyPart, const Pose6D &target, const WeightVector &weights)
{
	QMutexLocker ml(mutex);
	cout<<"Recibido target"<<endl;
	if(currentTarget.getState()==Target::State::IDLE)
	{
		currentTarget.setBodyPart (bodyPart);
		currentTarget.setPose     (target);
		currentTarget.setWeights  (weights);
		currentTarget.setState    (Target::State::WAITING);
		currentTarget.setID_VIK   (contador);
	}
	else
	{
		Target auxnextTargets;
		auxnextTargets.setBodyPart (bodyPart);
		auxnextTargets.setPose     (target);
		auxnextTargets.setWeights  (weights);
		auxnextTargets.setState    (Target::State::WAITING);
		auxnextTargets.setID_VIK   (contador);
		nextTargets.enqueue(auxnextTargets);
	}
	contador++;
	
	return contador-1;
}


/**
 * @brief Make the body part advance along a given direction. It is meant to work as a simple translational joystick to facilitate grasping operations
 * @param bodyPart  name of the body part.
 * @param ax the direction
 * @param dist step to advance un milimeters
 * @return the identifier of the target (an int)
 */
int SpecificWorker::setTargetAdvanceAxis(const string &bodyPart, const Axis &ax, const float dist)
{
	return inversekinematics_proxy->setTargetAdvanceAxis(bodyPart, ax, dist);
}


/**
 * \brief This method  of the interface stores a new ALIGNAXIS target into the correspondig part of the robot.
 * @param bodypart part of the robot body
 * @param target pose of the goal position
 * @param ax axis to be aligned
 */
int SpecificWorker::setTargetAlignaxis(const string &bodyPart, const Pose6D &target, const Axis &ax)
{
	//NOTE: When the VIK is connected to IK directly
// 	int id = inversekinematics_proxy->setTargetAlignaxis(bodyPart, target, ax);
// 	while (inversekinematics_proxy->getTargetState(bodyPart, id).finish == false);
// 	updateMotors(inversekinematics_proxy->getTargetState(bodyPart, id).motors);
// 	return id;
	//NOTE: When the VIK is connected to GIK and GIK is connected to IK.
	return inversekinematics_proxy->setTargetAlignaxis(bodyPart, target, ax);
}


/**
 * \brief this method moves the motors to their home value
 * @param bodyPart the part of the robot that we want to move to the home
 */
void SpecificWorker::goHome(const string &bodyPart)
{
	inversekinematics_proxy->goHome(bodyPart);
}

void SpecificWorker::stop(const string &bodyPart)
{
	QMutexLocker ml(mutex);
	nextTargets.clear();
	currentTarget.setState(Target::State::IDLE);
	stateMachine = State::IDLE;
	
	inversekinematics_proxy->stop(bodyPart);
}

/**
 * \brief this method changes the position of a determina joint.
 * @param joint the joint to change
 * @param angle the new angle of the joint.
 * @param maxSpeed the speed of the joint
 */
void SpecificWorker::setJoint(const string &joint, const float angle, const float maxSpeed)
{
	inversekinematics_proxy->setJoint(joint, angle, maxSpeed);
}


/**
 * @brief Set the fingers of the right hand position so there is d mm between them
 * @param d milimeters between fingers
 */
void SpecificWorker::setFingers(const float d)
{
	inversekinematics_proxy->setFingers(d);
}


void SpecificWorker::newAprilTag(const tagsList &tags)
{
	if(INITIALIZED == true)
	{
		for (auto tag : tags)
		{
			if (tag.id == 25)
			{
				QMutexLocker ml(mutexRightHand);
				rightHand->setVisualPose(tag);
			}
		}
	}
}

void SpecificWorker::applyFirstApproximation()
{
	QVec target = QVec::vec3(currentTarget.getPose().x(), currentTarget.getPose().y(), currentTarget.getPose().z());
	
// 	QVec targetFirstCorrection = target + firstCorrection;
	Pose6D firstCorrectedTarget;
// 	firstCorrectedTarget.x = targetFirstCorrection.x();
// 	firstCorrectedTarget.y = targetFirstCorrection.y();
// 	firstCorrectedTarget.z = targetFirstCorrection.z();
// 	firstCorrectedTarget.rx = currentTarget.getPose6D().rx;
// 	firstCorrectedTarget.ry = currentTarget.getPose6D().ry;
// 	firstCorrectedTarget.rz = currentTarget.getPose6D().rz;
	firstCorrectedTarget.x  = currentTarget.getPose().x();
	firstCorrectedTarget.y  = currentTarget.getPose().y();
	firstCorrectedTarget.z  = currentTarget.getPose().z();
	firstCorrectedTarget.rx = currentTarget.getPose6D().rx;
	firstCorrectedTarget.ry = currentTarget.getPose6D().ry;
	firstCorrectedTarget.rz = currentTarget.getPose6D().rz;

	
	innerModel->updateTransformValues("corrected",
	  firstCorrectedTarget.x, firstCorrectedTarget.y, firstCorrectedTarget.z,
		firstCorrectedTarget.rx, firstCorrectedTarget.ry, firstCorrectedTarget.rz
	);

	
	printf("first pose command [ %f %f %f]\n", firstCorrectedTarget.x, firstCorrectedTarget.y, firstCorrectedTarget.z);
	
	try
	{
		int identifier = inversekinematics_proxy->setTargetPose6D(currentTarget.getBodyPart(), firstCorrectedTarget, currentTarget.getWeights6D());
		currentTarget.setID_IK(identifier);
	}
	catch (const Ice::Exception &ex){
		std::cout<<"EXCEPCION EN SET TARGET POSE 6D --> INIT IK: "<<ex<<std::endl;
	}
}

/*
void SpecificWorker::storeTargetCorrection()
{
	QVec auxTarget = QVec::vec3(currentTarget.getPose().x(), currentTarget.getPose().y(), currentTarget.getPose().z());
	QVec auxCorrect= QVec::vec3(correctedTarget.getPose().x(), correctedTarget.getPose().y(), correctedTarget.getPose().z());
	
	firstCorrection = auxCorrect - auxTarget;
}
*/

bool SpecificWorker::correctPose()
{
// 	static bool first = true;
// 	if (not first) return true;
// 	first = false;

	qDebug()<<"\n\n\n-------------------------";
	const float umbralMaxTime=25;
	const float tagLostThresholdTime=1;
	const float umbralErrorT=20.0, umbralErrorR=0.15;


	QString rightTip = rightHand->getTip();
	QVec rightHandVisualPose, rightHandInternalPose;
	QVec errorInv;
	{
		QMutexLocker ml(mutexRightHand);
		updateInnerModel_motors_target_and_visual();
		if (rightHand->getSecondsElapsed() > tagLostThresholdTime) // If the hand's tag is lost we assume that the internal position (according to the direct kinematics) is correct
		{
			qFatal("tag lost!");
			timeSinMarca = timeSinMarca+rightHand->getSecondsElapsed();
			rightHand->setVisualPosewithInternalError();
		}
		errorInv = rightHand->getTargetErrorInverse();
		rightHandVisualPose = rightHand->getVisualPose();
		printf("visualPose         [ %f %f %f ]\n", rightHandVisualPose(0), rightHandVisualPose(1), rightHandVisualPose(2));
		rightHandInternalPose = rightHand->getInternalPose();
		printf("internalPose       [ %f %f %f ]\n", rightHandInternalPose(0), rightHandInternalPose(1), rightHandInternalPose(2));

	}

	if (currentTarget.getRunTime()>umbralMaxTime)
	{
		abortCorrection = true;
		currentTarget.setState(Target::State::NOT_RESOLVED);
		errorInv.print("abort with error INV");
		QMutexLocker ml(mutexSolved);
		solvedList.enqueue(currentTarget);
		return false;
	}

	if (QVec::vec3(errorInv.x(), errorInv.y(), errorInv.z()).norm2()<umbralErrorT and QVec::vec3(errorInv.rx(), errorInv.ry(), errorInv.rz()).norm2()<umbralErrorR)
	{
		currentTarget.setState(Target::State::RESOLVED);
		errorInv.print("done with error INV");
// 		storeTargetCorrection();
		QMutexLocker ml(mutexSolved);
		solvedList.enqueue(currentTarget);
		return true;
	}

	// CORRECT TRANSLATION
	QVec errorInvP           = QVec::vec3(errorInv.x(), errorInv.y(), errorInv.z()).operator*(0.85);
	QVec errorInvP_from_root = innerModel->getRotationMatrixTo("root", "target") * errorInvP;
 

	printf("suma a la pose     [ %f %f %f ]\n", errorInvP_from_root(0), errorInvP_from_root(1), errorInvP_from_root(2));


	QVec poseCorregida   = innerModel->transform("root", "corrected") + errorInvP_from_root;
	QVec correccionFinal = QVec::vec6(0,0,0,0,0,0);
	correccionFinal(0) = poseCorregida(0);
	correccionFinal(1) = poseCorregida(1);
	correccionFinal(2) = poseCorregida(2);
	

	printf("correccion nueva   [ %f %f %f ]\n", poseCorregida(0), poseCorregida(1), poseCorregida(2));
	
	
	// CORRECT ROTATION
	QMat rotErrorInv = Rot3D(errorInv.rx(), errorInv.ry(), errorInv.rz());
 	QVec angulosFinales = (innerModel->getRotationMatrixTo("root", "target")*rotErrorInv.invert()).extractAnglesR_min();
// 	QVec angulosFinales = QVec::vec3(0, -1.5707, 0);
	correccionFinal(3) = angulosFinales(0);
	correccionFinal(4) = angulosFinales(1);
	correccionFinal(5) = angulosFinales(2);

	
	correctedTarget.setPose(correccionFinal);
	
	printf("correction pose    [ %f %f %f ]\n", correccionFinal(0), correccionFinal(1), correccionFinal(2));
	innerModel->updateTransformValues("corrected", correccionFinal(0), correccionFinal(1), correccionFinal(2), correccionFinal(3), correccionFinal(4), correccionFinal(5));
	
// 	qFatal("info 1");
	
// return false;



	// Call BIK and wait for it to finish
	int identifier = inversekinematics_proxy->setTargetPose6D(currentTarget.getBodyPart(), correctedTarget.getPose6D(), currentTarget.getWeights6D());
	correctedTarget.setID_IK(identifier);

	return false;
}



void SpecificWorker::updateInnerModel_motors_target_and_visual()
{
	//qDebug() << "--------";
	try
	{
		RoboCompJointMotor::MotorStateMap mMap;
		jointmotor_proxy->getAllMotorState(mMap);

		for (auto j : mMap)
		{
			innerModel->updateJointValue(QString::fromStdString(j.first), j.second.pos);
		//	qDebug() << QString::fromStdString(j.first) << j.second.pos;
		}
	}
	catch (const Ice::Exception &ex)
	{
		std::cout<<"--> Excepción en actualizar InnerModel"<<std::endl;
	}
	const Pose6D tt = currentTarget.getPose6D();
	innerModel->updateTransformValues("target", tt.x, tt.y, tt.z, tt.rx, tt.ry, tt.rz);
	const QVec pR = rightHand->getVisualPose();
	innerModel->updateTransformValues("visual_hand", pR.x(), pR.y(), pR.z(), pR.rx(), pR.ry(), pR.rz());
}


/**
 * \brief UPDATE MOTORS
 */
void SpecificWorker::updateMotors (RoboCompInverseKinematics::MotorList motors)
{
	for (auto motor : motors)
	{
		try
		{
			RoboCompJointMotor::MotorGoalPosition nodo;
			nodo.name = motor.name;
			nodo.position = motor.angle;
			nodo.maxSpeed = 1;
			jointmotor_proxy->setPosition(nodo);
		}
		catch (const Ice::Exception &ex)
		{
			std::cout<<"EXCEPTION IN UPDATE MOTORS: "<<ex<<std::endl;
		}
	}
	
	MotorStateMap allMotorsAct, allMotorsBack;
	jointmotor_proxy->getAllMotorState(allMotorsBack);
	for (bool allStill=false;   allStill==false;   allMotorsBack=allMotorsAct)
	{
		usleep(500000);
		jointmotor_proxy->getAllMotorState(allMotorsAct);
		allStill = true;
		for (auto v : allMotorsAct)
		{
			if (abs(v.second.pos - allMotorsBack[v.first].pos) > 0.01)
			{
				allStill = false;
				break;
			}
		}
	} 
	usleep(500000);

}

















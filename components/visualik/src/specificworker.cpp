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
	file.open("/home/robocomp/robocomp/components/robocomp-ursus/components/visualik/data.txt", ios::out | ios::app);
	if (file.is_open() == false)
		qFatal("ARCHIVO NO ABIERTO");

	goMotorsGO         = false;
	stateMachine       = State::IDLE;
	abortCorrection    = false;
	innerModel         = NULL;
	contador           = 0;
	timeSinMarca       = 0.0;
	mutexSolved        = new QMutex(QMutex::Recursive);
	firstCorrection    = QVec::zeros(3);
	
#ifdef USE_QTGUI
	connect(this->goButton, SIGNAL(clicked()), this, SLOT(goYESButton()));
	innerViewer        = NULL;
	//osgView          = new OsgView(this);
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
	file.close();
}


/**
 * \brief Metodo SET PARAM
 * Metodo desde el cual se cargaran los elementos que especifiquemos dentro del fichero config
 * del componente. Carga el innermodel, el osgview y actualiza el nodo target.
 * @param params mapa de parametros
 */
bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	qDebug()<<"YEAAAAAAAAH: 11111";

	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("InnerModel") ;
		if( QFile(QString::fromStdString(par.value)).exists() == true)
		{
			qDebug()<<"YEAAAAAAAAH: 22222";

			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Reading Innermodel file " << QString::fromStdString(par.value);
			innerModel = new InnerModel(par.value);
			qDebug()<<"YEAAAAAAAAH: 333333"<<QString::fromStdString(par.value);
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


///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/**
 * \brief Slot COMPUTE
 * Bucle ejecutado por el hilo del programa. Se encarga de actualizar el innermodel de la clase y de pintar la
 * posicion del target que ha llegado y la posición de la marca que esta viendo la camara.
 * MAQUINA DE ESTADOS:
 * 	-IDLE: estado en espera. Inicialmente el currentTarget se crea con estado IDLE, indicando que no tiene nada
 * 		   Pero cuando cambia a WAITING indica que ha llegado un target que espera ser ejecutado. En ese momento
 * 		   pasamos a INIT_BIK.
 * 	-INIT_BIK: independientemente de los pesos del currentTarget, enviamos el target al BIK para que se ejecute.
 * 			   Luego pasamos al estado WAIT_BIK.
 * 	-WAIT_BIK: esperamos a que el brazo deje de moverse para empezar con la correccion. Cuando se pare pasamos
 * 			   al estado CORRECT_TRASLATION.
 * 	-CORRECT_ROTATION: corrige los errores de la pose del tip del robot, arreglando tanto traslacion como rotacion.
 * 	-CORRECT_TRASLATION: corrige solo los errores de traslacion del tip.
 */
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
	updateAll();
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
			applyFirstCorrection();
// 			try
// 			{
// 				int identifier = inversekinematics_proxy->setTargetPose6D(currentTarget.getBodyPart(), currentTarget.getPose6D(), currentTarget.getWeights6D());
// 				currentTarget.setID_IK(identifier);
// 			}
// 			catch (const Ice::Exception &ex){
// 				std::cout<<"EXCEPCION EN SET TARGET POSE 6D --> INIT IK: "<<ex<<std::endl;
// 			}
			currentTarget.setState(Target::State::IN_PROCESS); // El currentTarget pasa a estar siendo ejecutado:
			correctedTarget = currentTarget;
			stateMachine    = State::WAIT_BIK; // Esperamos a que el BIK termine.
		break;
		//---------------------------------------------------------------------------------------------
		case State::WAIT_BIK:
			// Esperamos a que el BIK termine de mover el brazo para comenzar con las correcciones.
			// Dependiendo de si las rotaciones pesan o no tendremos dos metodos de correccion:
			if(inversekinematics_proxy->getTargetState(currentTarget.getBodyPart(), currentTarget.getID_IK()).finish == true)
			{
				qDebug()<<"---> El IK ha terminado.";
				//updateMotors(inversekinematics_proxy->getTargetState(currentTarget.getBodyPart(), currentTarget.getID_IK()).motors);
				stateMachine = State::CORRECT_ROTATION;
			}
		break;
		//---------------------------------------------------------------------------------------------
		case State::CORRECT_ROTATION:
			//la primera vez el ID de corrected es igaula al anterior así que entra seguro
			if(inversekinematics_proxy->getTargetState(correctedTarget.getBodyPart(), correctedTarget.getID_IK()).finish == false) return;
			//updateMotors(inversekinematics_proxy->getTargetState(correctedTarget.getBodyPart(), correctedTarget.getID_IK()).motors);
			if (correctRotation()==true or abortCorrection==true)
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
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////


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
 * @param d millimeters between fingers
 */
void SpecificWorker::setFingers(const float d)
{
	inversekinematics_proxy->setFingers(d);
}


///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::newAprilTag(const tagsList &tags)
{
	// Recibimos las marcas que la camara esta viendo: marca mano y marca target.
	QMutexLocker ml(mutex);
	if(INITIALIZED == true)
	{
		for (auto tag : tags)
		{
			if (tag.id == 25)
			{
				rightHand->setVisualPose(tag);
			}
		}
	}
}
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/**
 * \brief Metodo auxiliar que guarda en el fichero el resultado de la correccion de un target.
 * @param errorInv 
 */ 
void SpecificWorker::printXXX(QVec errorInv/*, bool camaraNoVista*/)
{	
	file<<"P: ("      <<currentTarget.getPose();
	file<<")   ErrorVisual_T:"<<QVec::vec3(errorInv.x(), errorInv.y(), errorInv.z()).norm2();
	file<<"   ErrorVisual_R:" <<QVec::vec3(errorInv.rx(), errorInv.ry(), errorInv.rz()).norm2();
	file<<"   ErrorDirecto_T:" <<inversekinematics_proxy->getTargetState(currentTarget.getBodyPart(), correctedTarget.getID_IK()).errorT;
	file<<"   ErrorDirecto_R: "<<inversekinematics_proxy->getTargetState(currentTarget.getBodyPart(), correctedTarget.getID_IK()).errorR;
	file<<"   END: "    <<currentTarget.getRunTime();
	file<<"   WHY?: "<<inversekinematics_proxy->getTargetState(currentTarget.getBodyPart(), correctedTarget.getID_IK()).state;
	if(timeSinMarca > (60/4))
		file<<"   CAMARA PERDIDA: "<<1<<" - "<<timeSinMarca<<endl;
	else
		file<<"   CAMARA PERDIDA: "<<0<<" - "<<timeSinMarca<<endl;
	flush(file);	
}

/**
 * \brief Metodo Aniadido APPLY CORRECTION
 * Metodo aniadido para aplicar a la cinematica inversa una correccion inicial del target.
 * La primera vez que se ejecuta la correccion vale 0, pero a medida que se van ejecutando 
 * los targets, el error calculado entre el target y el target corregido se almacena y se
 * aplica en esta correcion inicial.
 * TODO HAY QUE PROBARLO MUUUUUUUUCHO
 */ 
void SpecificWorker::applyFirstCorrection()
{
	QVec target = QVec::vec3(currentTarget.getPose().x(), currentTarget.getPose().y(), currentTarget.getPose().z());
	QVec targetFirstCorrection = target + firstCorrection;
	Pose6D firstCorrectedTarget;
	firstCorrectedTarget.x = targetFirstCorrection.x();
	firstCorrectedTarget.y = targetFirstCorrection.y();
	firstCorrectedTarget.z = targetFirstCorrection.z();
	firstCorrectedTarget.rx = currentTarget.getPose6D().rx;
	firstCorrectedTarget.ry = currentTarget.getPose6D().ry;
	firstCorrectedTarget.rz = currentTarget.getPose6D().rz;

	try
	{
		int identifier = inversekinematics_proxy->setTargetPose6D(currentTarget.getBodyPart(), firstCorrectedTarget, currentTarget.getWeights6D());
		currentTarget.setID_IK(identifier);
	}
	catch (const Ice::Exception &ex){
		std::cout<<"EXCEPCION EN SET TARGET POSE 6D --> INIT IK: "<<ex<<std::endl;
	}
}

/**
 * \brief Metodo aniadido STORE TARGET CORRECTION
 * Almacena en la variable de clase firstCorrection, la correccion entre el target original y la última posicion corregida
 * asociada a ese target para que pueda ser utilizada por el siguiente target como correccion inicial.
 * TODO HAY QUE PROBARLO
 */ 
void SpecificWorker::storeTargetCorrection()
{
	//CALCULAMOS EL VECTOR DE ERROR ENTRE TARGET Y CORRECTED TARGET
	QVec auxTarget = QVec::vec3(currentTarget.getPose().x(), currentTarget.getPose().y(), currentTarget.getPose().z());
	QVec auxCorrect= QVec::vec3(correctedTarget.getPose().x(), correctedTarget.getPose().y(), correctedTarget.getPose().z());
	
	firstCorrection = auxCorrect - auxTarget;
}

/**
 * \brief Metodo CORRECT ROTATION
 * Corrige la posicion de la mano en traslacion y en rotacion.
 * @return bool TRUE si el target esta perfecto o FALSE si no lo esta.
 */
bool SpecificWorker::correctRotation()
{
	qDebug()<<"\n\n\n-------------------------";
	updateAll();
	static float umbralMaxTime = 25, umbralMinTime = 3;
	static float umbralElapsedTime = 5.0, umbralErrorT = 5.0, umbralErrorR=0.2;

	// If the hand's tag is lost we assume that the internal possition (according to the direct kinematics) is correct
	if (rightHand->getSecondsElapsed() > umbralElapsedTime)
	{
		std::cout<<"La camara no ve la marca..."<<std::endl;
		timeSinMarca = timeSinMarca+rightHand->getSecondsElapsed();
		rightHand->setVisualPosewithInternal();
	}
	// COMPROBAMOS EL ERROR:
	QVec errorInv = rightHand->getErrorInverse(); //error: mano visualdesde el target
	if (currentTarget.getRunTime()>umbralMaxTime and currentTarget.getRunTime()>umbralMinTime)
	{
		abortCorrection = true;
		currentTarget.setState(Target::State::NOT_RESOLVED);
		qDebug()<<"Abort rotation: traslacion"<<QVec::vec3(errorInv.x(), errorInv.y(), errorInv.z()).norm2()<<"  rotacion: "<<QVec::vec3(errorInv.rx(), errorInv.ry(), errorInv.rz()).norm2();
		printXXX(errorInv);
		QMutexLocker ml(mutexSolved);
		solvedList.enqueue(currentTarget);
		return false;
	}
	// Si el error es miserable no hacemos nada y acabamos la corrección.
	if (QVec::vec3(errorInv.x(), errorInv.y(), errorInv.z()).norm2()<umbralErrorT and QVec::vec3(errorInv.rx(), errorInv.ry(), errorInv.rz()).norm2()<umbralErrorR)
	{
		currentTarget.setState(Target::State::RESOLVED);
		qDebug()<<"done!traslacion"<<QVec::vec3(errorInv.x(), errorInv.y(), errorInv.z()).norm2()<<"  rotacion: "<<QVec::vec3(errorInv.rx(), errorInv.ry(), errorInv.rz()).norm2();
		printXXX(errorInv);
		storeTargetCorrection();
		QMutexLocker ml(mutexSolved);
		solvedList.enqueue(currentTarget);
		return true;
	}

	QVec errorInvP = QVec::vec3(errorInv.x(), errorInv.y(), errorInv.z()).operator*(0.5);
	QVec errorInvP_from_root = innerModel->getRotationMatrixTo("root", rightHand->getTip()) * errorInvP;
 
	QVec poseCorregida = innerModel->transform("root", rightHand->getTip()) + errorInvP_from_root;
	QVec correccionFinal = QVec::vec6(0,0,0,0,0,0);
	correccionFinal.inject(poseCorregida,0);
<<<<<<< HEAD
	
	//Pasamos los angulos a matriz de rotacion
	QMat rotErrorInv = Rot3D(errorInv.rx(), errorInv.ry(), errorInv.rz()).invert();
	// multiplicamos la matriz del tip en el root por la matriz generada antes
	//extraemos los angulos de la nueva matriz y esos son los angulas ya corregidos
	QVec angulosFinales = (innerModel->getRotationMatrixTo("root", rightHand->getTip())*rotErrorInv).extractAnglesR_min();


	correccionFinal.inject(QVec::vec3(angulosFinales(0), angulosFinales(1), angulosFinales(2)), 3);

	correctedTarget.setPose(correccionFinal);
	
	
	rightHand->getVisualPose().print("VISUAL POSE: ");
	rightHand->getInternalPose().print("INTERNAL POSE: ");
	errorInv.print("ERROR INVERSE: ");
	correccionFinal.print("CORRECCION: ");
=======
		
	// ROTACION: 
// 	Rot3D matrizR_error (errorInv(3), errorInv(4), errorInv(5));
// 	QMat  matrizTip_root = innerModel->getRotationMatrixTo("root", "visual_hand");
// 	QMat  matrizRotacionFinal =  matrizTip_root * matrizR_error;
// 	QVec angulosFinales = matrizRotacionFinal.extractAnglesR_min();
// 	correccionFinal.inject(QVec::vec3(angulosFinales(0), angulosFinales(1), angulosFinales(2)),3);
	correccionFinal.inject(QVec::vec3(currentTarget.getPose().rx(), currentTarget.getPose().ry(), currentTarget.getPose().rz()),3);
	correctedTarget.setPose(correccionFinal);
	
	
	qDebug()<<"VISUAL POSE: "<<rightHand->getVisualPose();
	qDebug()<<"INTERNAL POSE: "<<rightHand->getInternalPose();
	qDebug()<<"ERROR INVERSE: "<< errorInv;
	qDebug()<<"ERROR T: "<< QVec::vec3(errorInv.x(), errorInv.y(), errorInv.z()).norm2();
	qDebug()<<"ERROR R: "<<QVec::vec3(errorInv.rx(), errorInv.ry(), errorInv.rz()).norm2();
	qDebug()<<"CORRECCION: "<< correccionFinal;
>>>>>>> 86fa073abbfe5a190a0cf1d2df1797cb0d3cd062

	//Llamamos al BIK con el nuevo target corregido y esperamos
	int identifier = inversekinematics_proxy->setTargetPose6D(currentTarget.getBodyPart(), correctedTarget.getPose6D(), currentTarget.getWeights6D());
	correctedTarget.setID_IK(identifier);
	return false;
}


/**
 * \brief Metodo UPDATE ALL
 * Se encarga de actualizar la posicion de los motores del robot (el innerModel),
 * la pose del target que le enviamos y la pose de la marca visual que ve.
 */
void SpecificWorker::updateAll()
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
 * \brief Metodo UPDATE MOTORS
 */
void SpecificWorker::updateMotors (RoboCompInverseKinematics::MotorList motors)
{
	for (auto motor : motors)
	{
		try
		{
			RoboCompJointMotor::MotorGoalPosition nodo;
			nodo.name = motor.name;
			nodo.position = motor.angle; // posición en radianes
			nodo.maxSpeed = 3; //radianes por segundo TODO Bajar velocidad.
			jointmotor_proxy->setPosition(nodo);
		}
		catch (const Ice::Exception &ex)
		{
			std::cout<<"EXCEPTION IN UPDATE MOTORS: "<<ex<<std::endl;
			//ABANDONAMOS TARGET SI LA IK NO PUEDE MOVERSE NI SIQUIERA CERCA DEL TARGET
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

void SpecificWorker::goYESButton()
{
	//goMotorsGO = true;
	qDebug() << "\n---------------------------------------------------------------";
	qDebug()<<"CALIBRATE!!!!!!!!!!!!!!";
	
	// Esto es sólo para mostrar la posición del tip vista desde la cámara y desde el innermodel expresadas en el sistema de referencia del mundo
	QVec grabInWorld = innerModel->transform6D("root", rightHand->getTip()); // Con traslaciones y rotaciones.
	qDebug()<<"internal TIP in root: "<<grabInWorld;
	QVec visualHandInWorld = innerModel->transform6D("root","visual_hand");
	qDebug() << "VISUAL_HAND en el mundo vista desde la camara" << visualHandInWorld;
	qDebug() << "Diferencia" <<  visualHandInWorld - grabInWorld;
	
	
	// Calculamos el error de la marca
	// Ponemos la marca vista desde la cámara en el sistema de coordenadas de la mano (TIP), si no hay error debería ser todo cero
	QVec visualMarcaInHandTip = innerModel->transform6D(rightHand->getTip(), "visual_hand");
	qDebug()<<" Visual in Tip: "<<visualMarcaInHandTip;
	QVec visualMarcaTInHandTip=QVec::vec3(visualMarcaInHandTip.x(), visualMarcaInHandTip.y(), visualMarcaInHandTip.z());

	// Cogemos la matriz de rotación de TIP (marca en la mano) con respecto al padre (ThandMesh2_pre) para que las nuevas rotaciones y translaciones que hemos calculado (visualMarcaTInHandTip) sean añadidas a las ya esistentes en tip
	QMat visualMarcaRInHandMarcaMat = innerModel->getRotationMatrixTo(rightHand->getTip(),"visual_hand");
	QMat handMarcaRInParentMat = innerModel->getRotationMatrixTo("ThandMesh2_pre",rightHand->getTip());
	 		
	// Multiplicamos las matrices de rotación para sumar la nueva rotación visualMarcaRInHandMarcaMat a la ya existente con respecto al padre
	QMat finalHandMarcaRMat = handMarcaRInParentMat * visualMarcaRInHandMarcaMat;
	QVec finalHandMarcaR = finalHandMarcaRMat.extractAnglesR_min();

	// Pasamos también las translaciones nuevas (visualMarcaTInHandTip) al padre y las sumamos con las existentes
	QVec handMarcaTInParent = innerModel->transform("ThandMesh2_pre", QVec::zeros(3), rightHand->getTip());
	QVec finalHandMarcaT = handMarcaTInParent + (handMarcaRInParentMat* visualMarcaTInHandTip);

	// Esto es sólo para mostar como está el TIP respecto al padre antes de las modificaciones
	QVec inicialHandMarca(6);
	inicialHandMarca.inject(handMarcaTInParent,0);
	inicialHandMarca.inject(handMarcaRInParentMat.extractAnglesR_min(),3);	
	qDebug() << "Posicion inicial del TIP respecto al padre" << inicialHandMarca;

	// Creamos el vector final con las rotaciones y translaciones del TIP con respecto al padre
	QVec finalHandMarca(6);
	finalHandMarca.inject(finalHandMarcaT,0);
	finalHandMarca.inject(finalHandMarcaR,3);

	qDebug() << "Posicion final corregida del TIP respecto al padre" << finalHandMarca;

	//Actualizamos el transform de la marca en la mano (ThandMesh1) con las rotaciones y translaciones calculadas
	mutex->lock();
		innerModel->updateTransformValues(rightHand->getTip(),finalHandMarca.x(), finalHandMarca.y(), finalHandMarca.z(), finalHandMarca.rx(), finalHandMarca.ry(), finalHandMarca.rz());	
	mutex->unlock();
	
	
	//Escribimos por pantalla como está el grab en el mundo despues de hacer las modificaciones
	grabInWorld = innerModel->transform6D("root", rightHand->getTip());
	qDebug() << "Grab en el mundo despues de modificar" << grabInWorld;
		
	qDebug() << "--------------------------------------------\n";
	
	//Actualizamos el inermodel del Bik con los datos del inermodel del tester
	// ALERT: PREGUNTAR A LUIS COMO PASAMOS EL CAMBIO A LA IK Y A LA IKG
	innerModel->updateTranslationValues(rightHand->getTip(), finalHandMarca.x(), finalHandMarca.y(), finalHandMarca.z());
// 	
	
	//We send now to inermodel BIK the new endEffector pose
// 	try 
// 	{
// 		RoboCompBodyInverseKinematics::Pose6D pose;
// 		pose.x = finalHandMarca.x();
// 		pose.y = finalHandMarca.y();
// 		pose.z = finalHandMarca.z();
// 		pose.rx = finalHandMarca.rx();
// 		pose.ry = finalHandMarca.ry();
// 		pose.rz = finalHandMarca.rz();
// 		inversekinematics_proxy->setTip("LEFTARM", "ThandMesh1", pose);
// 		
// 		qDebug() << __FUNCTION__ << visualMarcaInHandMarca;
// 		
// //		bodyinversekinematics_proxy->setTargetPose6D("LEFTARM", pose, pesos,10);
// 		
// 			
// 	} 
// 	catch (const Ice::Exception &ex) 
// 	{
// 		std::cout << ex << endl;
// 	}
	
	
	
	
	//Eliminamos el nodo creado
	
}


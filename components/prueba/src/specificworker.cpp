/*
 *    Copyright (C) 2017 by YOUR NAME HERE
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

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//       THE FOLLOWING IS JUST AN EXAMPLE
//
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		innermodel_path = par.value;
//		innermodel = new InnerModel(innermodel_path);
//	}
//	catch(std::exception e) { qFatal("Error reading config params"); }



	
	timer.start(Period);
	

	return true;
}

void SpecificWorker::compute()
{
	QMutexLocker locker(mutex);

	
}

RoboCompLaser::TLaserData ElasticBand::unionpoligonos(RoboCompLaser::TLaserData laserData, SafePolyList &safePolyList, InnerModel *innermodel)
{
	RoboCompLaser::TLaserData laserCombined; 
	laserCombined = laserData;
	// For each polyline
	LocalPolyLineList l = safePolyList.read(); 
	
	for (auto &laserSample: laserCombined)
	{
		for (auto polyline : l)
		{
			auto previousPoint = polyline[polyline.size()-1];
			QVec previousPointInLaser = innermodel->transform("laser", (QVec::vec3(previousPoint.x, 0, previousPoint.z)).operator*(1000), "world");
			float pDist  = std::sqrt(previousPointInLaser.x()*previousPointInLaser.x() + previousPointInLaser.z()*previousPointInLaser.z());
			float pAngle = atan2(previousPointInLaser.x(), previousPointInLaser.z());
			// For each polyline's point
			for (auto polylinePoint: polyline)
			{
				QVec currentPointInLaser = innermodel->transform("laser", (QVec::vec3(polylinePoint.x, 0, polylinePoint.z)).operator*(1000), "world");
				float cDist  = sqrt(currentPointInLaser.x()*currentPointInLaser.x() + currentPointInLaser.z()*currentPointInLaser.z());
				float cAngle = atan2(currentPointInLaser.x(), currentPointInLaser.z());

				const float m = std::min<float>(cAngle, pAngle);
				const float M = std::max<float>(cAngle, pAngle);
				if (m >  M_PI) printf("m>+p\n");
				if (m < -M_PI) printf("m<-p\n");
				if (M >  M_PI) printf("M>+p\n");
				if (M < -M_PI) printf("M<-p\n");
				if (laserSample.angle >  M_PI) printf("L> p\n");
				if (laserSample.angle < -M_PI) printf("L<-p\n");
				//printf("angulo: %f   p:%f  c:%f\n", laserSample.angle, cAngle, pAngle);
				if (laserSample.angle >= m and laserSample.angle <= M)
				{
					printf("dentro\n");
					float mean = (cDist + pDist) / 2.;
					if (mean<laserSample.dist) laserSample.dist = mean;
				}
				pDist = cDist;
				pAngle = cAngle;
			}
		}
	}
	return laserCombined;
} 







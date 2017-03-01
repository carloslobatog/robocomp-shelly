
#ifndef SAFEPOLYLIST_H
#define SAFEPOLYLIST_H

#include <math.h>
#include <innermodel/innermodel.h>
#include <TrajectoryRobot2D.h>


typedef struct { float x; float z;} LocalPoint;
typedef struct { float dist; float angle;} LocalPointPol;

typedef std::vector<LocalPointPol> PolyLinePol;
typedef std::vector<LocalPoint> LocalPolyLine;

typedef struct {PolyLinePol p; float min; float max;} LocalPolyLinePolMinMax;

typedef std::vector<LocalPolyLinePolMinMax> LocalPolyLineListPol;
typedef std::vector<LocalPolyLine> LocalPolyLineList;

						
class SafePolyList
{
  QMutex mutex;
  LocalPolyLineList polyLineList;
  LocalPolyLineListPol polyLineListPol;
  
  public:	
    void write(const RoboCompTrajectoryRobot2D::PolyLineList &secuencia)
    {
		QMutexLocker ml(&mutex);
		polyLineList.clear();
		for(auto s: secuencia)
		{
			LocalPolyLine puntos;
			for(auto p: s)
			{
				LocalPoint punto = {p.x, p.z};
				puntos.push_back(punto);
			}
			polyLineList.push_back(puntos);
		}
    }
    
    LocalPolyLineList read()
    {
		QMutexLocker ml(&mutex);
		return polyLineList;
    }

    LocalPolyLineListPol read(InnerModel *innermodel)
    {
		QMutexLocker ml(&mutex);
		QVec pInLaser;
		
		for( auto l : polyLineList)
		{
			float min = std::numeric_limits<float>::max();
			float max = std::numeric_limits<float>::min(); 
			
			PolyLinePol polyLinePol;
			for( auto p: l) 
			{
				LocalPointPol lPol;
				pInLaser = innermodel->transform("laser", QVec::vec3(p.x, 0, p.z)*(float)1000, "world");
				lPol.dist  = sqrt(pInLaser.x()*pInLaser.x() + pInLaser.z()*pInLaser.z());
				lPol.angle = atan2(pInLaser.x(), pInLaser.z());	
				
				polyLinePol.push_back(lPol);
				
				if( lPol.angle < min ) min = lPol.angle;
				if( lPol.angle > max ) max = lPol.angle;
			}
			LocalPolyLinePolMinMax pmm = {polyLinePol, min, max};
			polyLineListPol.push_back(pmm);	
		}
      return polyLineListPol;
    }
};

#endif

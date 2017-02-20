
#ifndef SAFEPOLYLIST_H
#define SAFEPOLYLIST_H

typedef struct { float x; float z;} LocalPoint;
typedef std::vector<std::vector<LocalPoint> > LocalPolyLineList;

class SafePolyList
{
  QMutex mutex;
  LocalPolyLineList polyLineList;
  
  public:
    void write(const RoboCompTrajectoryRobot2D::PolyLineList &secuencia)
    {
      QMutexLocker ml(&mutex);
      polyLineList.clear();
      
	for(auto s: secuencia)
	{
	  std::vector<LocalPoint> puntos;
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
};

#endif

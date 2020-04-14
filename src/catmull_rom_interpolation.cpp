#include "catmull_rom_interpolation.h"
#include <Eigen/Dense>

Eigen::Vector3d catmull_rom_interpolation(
  const std::vector<std::pair<double, Eigen::Vector3d> > & keyframes,
  double t)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
  // see reference: https://en.wikipedia.org/wiki/Centripetal_Catmull%E2%80%93Rom_spline
  if (keyframes.size() == 0){
  	return Eigen::Vector3d(0,0,0);
  }
  
  t = std::fmod(t, keyframes.back().first);

  int idx = keyframes.size()-1;  
  for (int i=0; i<keyframes.size(); i++){
		if (keyframes[i].first > t){
			idx = i;
			break;
		}
  }
  
  Eigen::Vector3d P0, P1, P2, P3;
  double t0, t1, t2, t3;

  P2 = keyframes[idx].second;
  t2 = keyframes[idx].first;
  if (idx == 0){ // at the first index
    P0 = keyframes[keyframes.size()-2].second;
    P1 = keyframes[keyframes.size()-1].second;
    P3 = keyframes[idx+1].second;		
    
    t0 = keyframes[keyframes.size()-2].first;
    t1 = keyframes[keyframes.size()-1].first;
    t3 = keyframes[idx+1].first;	
  } else if (idx == 1){ // at the second
    P0 = keyframes[keyframes.size()-1].second;
    P1 = keyframes[idx-1].second;
    P3 = keyframes[idx+1].second;	
    
    t0 = keyframes[keyframes.size()-1].first;
    t1 = keyframes[idx-1].first;
    t3 = keyframes[idx+1].first;
  } else if (idx == keyframes.size()-1){ // at the end
    P0 = keyframes[idx-2].second;
    P1 = keyframes[idx-1].second;
    P3 = keyframes[0].second;			
    
    t0 = keyframes[idx-2].first;
    t1 = keyframes[idx-1].first;
    t3 = keyframes[0].first;	
  } else{  // in the middles(general case)
    P0 = keyframes[idx-2].second;
    P1 = keyframes[idx-1].second;
    P3 = keyframes[idx+1].second;
    
    t0 = keyframes[idx-2].first;
    t1 = keyframes[idx-1].first;
    t3 = keyframes[idx+1].first;
  }
  
  Eigen::Vector3d A1, A2, A3, B1, B2, C;
  A1 = (t1-t)/(t1-t0)*P0 + (t-t0)/(t1-t0)*P1;
  A2 = (t2-t)/(t2-t1)*P1 + (t-t1)/(t2-t1)*P2;
  A3 = (t3-t)/(t3-t2)*P2 + (t-t2)/(t3-t2)*P3;
  
  B1 = (t2-t)/(t2-t0)*A1 + (t-t0)/(t2-t0)*A2;
  B2 = (t3-t)/(t3-t1)*A2 + (t-t1)/(t3-t1)*A3;
  
  C  = (t2-t)/(t2-t1)*B1 + (t-t1)/(t2-t1)*B2;
  return C;
  
 
 
  
  
  
  /////////////////////////////////////////////////////////////////////////////
}

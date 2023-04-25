//  state(1)    speed
//  state(2)    yaw_rate
//  state(3)    longitude_acc
//  state(4)    lateral_acc
//  state(5)    x_pos
//  state(6)    y_pos
//  state(7)    PHI

#include "nonlinearanalyticconditionalgaussianmobile.h"
#include <wrappers/rng/rng.h> // Wrapper around several rng
                                 // libraries
#define NUMCONDARGUMENTS_MOBILE 2

namespace BFL
{
  using namespace MatrixWrapper;


  NonLinearAnalyticConditionalGaussianMobile::NonLinearAnalyticConditionalGaussianMobile(const Gaussian& additiveNoise)
    : AnalyticConditionalGaussianAdditiveNoise(additiveNoise,NUMCONDARGUMENTS_MOBILE)
  {
  }


  NonLinearAnalyticConditionalGaussianMobile::~NonLinearAnalyticConditionalGaussianMobile(){}

  ColumnVector NonLinearAnalyticConditionalGaussianMobile::ExpectedValueGet() const
  {
    ColumnVector state = ConditionalArgumentGet(0);
    ColumnVector tim  = ConditionalArgumentGet(1);

    state(1) += state(3) * tim(1);                      //  speed
 
    // state(4) = state(1) * state(2);                     //  acc_lat 速度是0的时候算出来的yaw_rate会无穷大 所以更新侧向加速度而不是yaw_rate

    state(5) += cos(state(7)) * state(1) * tim(1);      //  X_pos       
    state(6) += sin(state(7)) * state(1) * tim(1);      //  Y_pos       
    state(7) += state(2) * tim(1);                      //  PHI         

    return state + AdditiveNoiseMuGet();
  }

  Matrix NonLinearAnalyticConditionalGaussianMobile::dfGet(unsigned int i) const
  {
    if (i==0)//derivative to the first conditional argument (x)
      {
  double speed = ConditionalArgumentGet(0)(1);
  double yaw_rate = ConditionalArgumentGet(0)(2);
  double acc_long = ConditionalArgumentGet(0)(3);
  double longitude_acc = ConditionalArgumentGet(0)(4);
  double x_pos = ConditionalArgumentGet(0)(5);
  double y_pos = ConditionalArgumentGet(0)(6);
  double yaw = ConditionalArgumentGet(0)(7);
  double tim = ConditionalArgumentGet(1)(1);
	Matrix df(7,7);
  df = 0.0;
  for (unsigned int i = 1; i <= 7; i++)
  {
      df(i, i) = 1;
  }
//  state(1)    speed
//  state(2)    yaw_rate
//  state(3)    longitude_acc
//  state(4)    lateral_acc
//  state(5)    x_pos
//  state(6)    y_pos
//  state(7)    PHI
  df(1,3) = tim;     //  只滤速度
  // df(2,4) = tim;
  // df(4,1) = yaw_rate;
  // df(4,2) = speed;
  df(5, 1) = tim * sin(yaw);                //  dX/dv
  df(5, 2) = speed * tim * cos(yaw) * tim;  //  dX/dw
  df(5, 7) = speed * tim * cos(yaw);        //  dX/dPHI
  df(6, 1) = tim * cos(yaw);                //  dX/dv
  df(6, 2) = -speed * tim * sin(yaw) * tim; //  dY/dw
  df(6, 7) = -speed * tim * sin(yaw);       //  dY/dPHI
  df(7, 2) = tim;

	return df;
      }
    else
      {
	if (i >= NumConditionalArgumentsGet())
	  {
	    cerr << "This pdf Only has " << NumConditionalArgumentsGet() << " conditional arguments\n";
	    exit(-BFL_ERRMISUSE);
	  }
	else{
	  cerr << "The df is not implemented for the" <<i << "th conditional argument\n";
	  exit(-BFL_ERRMISUSE);
	}
      }
  }

}//namespace BFL


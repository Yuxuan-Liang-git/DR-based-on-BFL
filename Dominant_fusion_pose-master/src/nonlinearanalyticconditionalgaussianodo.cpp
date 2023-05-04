//
// Created by spring on 18-10-25.
//

#include "nonlinearanalyticconditionalgaussianodo.h"
#include <bfl/wrappers/rng/rng.h>

#define NUMCONDARGUMENTS_MOBILE 2

namespace BFL
{
using namespace MatrixWrapper;

NonLinearAnalyticConditionalGaussianOdo::NonLinearAnalyticConditionalGaussianOdo(const Gaussian &additiveNoise)
    : AnalyticConditionalGaussianAdditiveNoise(additiveNoise, NUMCONDARGUMENTS_MOBILE),
      df(8, 8)
{
    // initialize df matrix
    for (unsigned int i = 1; i <= 8; i++)
    {
        for (unsigned int j = 1; j <= 8; j++)
        {
            if (i == j)
                df(i, j) = 1;
            else
                df(i, j) = 0;
        }
    }
}

NonLinearAnalyticConditionalGaussianOdo::~NonLinearAnalyticConditionalGaussianOdo() {}

ColumnVector NonLinearAnalyticConditionalGaussianOdo::ExpectedValueGet() const
{
    ColumnVector state = ConditionalArgumentGet(0);
    ColumnVector tim = ConditionalArgumentGet(1);
    state(1) += sin(state(5)) * state(7) * tim(1);
    state(3) += cos(state(5)) * state(7) * tim(1);
    state(5) += state(8) * tim(1);
    return state + AdditiveNoiseMuGet();
}

Matrix NonLinearAnalyticConditionalGaussianOdo::dfGet(unsigned int i) const
{
    if (i == 0) //derivative to the first conditional argument (x)
    {
        double v = ConditionalArgumentGet(0)(7);
        double w = ConditionalArgumentGet(0)(8);
        double yaw = ConditionalArgumentGet(0)(5);
        double tim = ConditionalArgumentGet(1)(1);

        //  1：X=v*tim*sin(yaw)
        //  3: Y=v*tim*cos(yaw)
        //  5: yaw
        //  7: v
        //  8：yaw_rate
        df(1, 5) = v * tim * cos(yaw);
        df(3, 5) = -v * tim * sin(yaw);

        df(1, 7) = tim * sin(yaw);
        df(3, 7) = tim * cos(yaw);

        df(1, 8) = tim * sin(yaw) * tim;
        df(3, 8) = tim * cos(yaw) * tim;

        df(5, 8) = tim;

        return df;
    }
    else
    {
        if (i >= NumConditionalArgumentsGet())
        {
            cerr << "This pdf Only has " << NumConditionalArgumentsGet() << " conditional arguments\n";
            exit(-BFL_ERRMISUSE);
        }
        else
        {
            cerr << "The df is not implemented for the" << i << "th conditional argument\n";
            exit(-BFL_ERRMISUSE);
        }
    }
}

} //namespace BFL

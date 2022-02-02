#pragma once

#include "Filter.hpp"

/*
 * This is a Wrench Estimator.
 */
template <typename genericVector>
class WrenchEstimator: public Filter<genericVector>{
  public:

    /**
      * @brief   Initilizes the coefficient of the filter calling FiliterInit().
      *
      * @param[in] w_cutoff     cut-off angular frequency
      * @param[in] w_sampling   sampling angular frequency
      **/
    WrenchEstimator(double const w_cutoff, double const w_sampling);

  private:
    /**
      * @brief  It update the internal coefficient of Filter.
      *
      * @param[in] ratio  ratio between cut-off and sampling frequencies
      **/
    void _updateCoefficient(double ratio);
};

/*
 *  Implementation
 */
template <typename genericVector>
WrenchEstimator<genericVector>::WrenchEstimator(double const w_cutoff, double const w_sampling)
{
  this->FilterInit(w_cutoff,w_sampling);
}

template <typename genericVector>
void WrenchEstimator<genericVector>::_updateCoefficient(double ratio)
{
    //this->c1 = (2 - 2*ratio*ratio)      / (1 + ratio + ratio*ratio);
    //this->c2 = (-1 + ratio - ratio*ratio) / (1 + ratio + ratio*ratio);
    //this->c3 = 0;
    //this->c4 = (ratio*ratio)             / (1 + ratio + ratio*ratio);
    //this->c5 = (2*ratio*ratio)           / (1 + ratio + ratio*ratio);
    //this->c6 = (ratio*ratio)             / (1 + ratio + ratio*ratio);
    //this->c7 = 0;
    this->c1 = 1 / (1 + 2*ratio);
    this->c2 = -2*ratio / (1 + 2*ratio);
    this->c3 = 0;
}
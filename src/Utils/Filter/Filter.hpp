#pragma once

#include <cmath>

/*
 * This is a generic Filter class.
 */
template <typename genericVector> class Filter{
  public:
    /**
      * @brief   Costructor of Filter's class.
      **/
    Filter();

    /**
      * @brief   Initilizes the coefficient of the filter. 
      *
      * @param[in] a0   Coefficient of s at denominator
      * @param[in] a1   Costant value at denominator 
      * @param[in] b0   Coefficient of s at nominator
      * @param[in] b1   Costant value at nominator
      * @param[in] Ts   sampling time
      **/
    void FilterInit(double a0, double a1, double b0, double b1, double Ts);

    /**
      * @brief  Return the filter value at time k.
      * @note   It's mandatory to call FilterInit() method before of call getFilteredValue()
      *         or it will throw an error
      *
      * @param[in] u_k          data to filter
      * @param[in] y_k          filtered data
      **/
    genericVector getFilteredValue(genericVector u_k);

  protected:
    /**
      * @brief   Internal coefficient of Filter. The coefficient follow this style
      *          y(k) = c1*u(k) + c2*u(k-1) + c3*y(k-1)
      *
      **/
    double c1, c2, c3;

  private:
    /**
      * @brief    Internal vector of Filter used to store old value of input and output.
      *           The vector follow this style: y(k - t) -> _y_k_t
      **/
    genericVector y_k_1, u_k_1;
};

/* Implementation */

template <typename genericVector>
Filter<genericVector>::Filter()
{
  this->y_k_1 = this->u_k_1 = genericVector::Zero();
  this->c1 = this->c2 = this->c3 = 0;
}

template <typename genericVector>
void Filter<genericVector>::FilterInit(double a0, double a1, double b0, double b1, double Ts)
{
  double K = 2 / Ts;
  this->c1 = ( b0*K + b1) / (a0*K + a1);
  this->c2 = (-b0*K + b1) / (a0*K + a1);
  this->c3 = ( a0*K - a1) / (a0*K + a1);
}

template <typename genericVector>
genericVector Filter<genericVector>::getFilteredValue(genericVector u_k)
{
  genericVector y_k = this->c1*u_k + this->c2*this->u_k_1 + this->c3*this->y_k_1;
  this->y_k_1 << y_k;
  this->u_k_1 << u_k;
    
  return y_k;
}
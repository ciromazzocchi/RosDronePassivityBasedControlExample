#pragma once

#include <cmath>

/*
 * This is a generic Differentiator class.
 */
template <typename genericVector> class Differentiator{
  public:
    /**
      * @brief   Costructor of Differentiator's class.
      **/
    Differentiator();

    /**
      * @brief   Initilizes the coefficient of the Differentiator. 
      *
      * @param[in] kf   cutoff frequency
      * @param[in] Ts   sampling time
      **/
    void DifferentiatorInit(double kf, double Ts);

    /**
      * @brief  Return the Differentiator value at time k.
      * @note   It's mandatory to call DifferentiatorInit() method before of call getDifferentiatoredValue()
      *         or it will throw an error
      *
      * @param[in] u_k          data to Differentiator
      * @param[in] y_k          Differentiatored data
      **/
    genericVector getDifferentiatoredValue(genericVector u_k);

  private:
    /**
      * @brief   Internal coefficient of Differentiator. The coefficient follow this style
      *          y(k) = c1*u(k) + c2*u(k-1) + c3*y(k-1)
      *
      **/
    double c1, c2, c3;

    /**
      * @brief    Internal vector of Differentiator used to store old value of input and output.
      *           The vector follow this style: y(k - t) -> _y_k_t
      **/
    genericVector y_k_1, u_k_1;
};

/* Implementation */

template <typename genericVector>
Differentiator<genericVector>::Differentiator()
{
  this->y_k_1 = this->u_k_1 = genericVector::Zero();
  this->c1 = this->c2 = this->c3 = 0;
}

template <typename genericVector>
void Differentiator<genericVector>::DifferentiatorInit(double kf, double Ts)
{
  this->c1 = kf / (1 + kf * Ts) ;
  this->c2 = -this->c1;
  this->c3 = 1 / (1 + kf * Ts);
}

template <typename genericVector>
genericVector Differentiator<genericVector>::getDifferentiatoredValue(genericVector u_k)
{
  genericVector y_k = this->c1*u_k + this->c2*this->u_k_1 + this->c3*this->y_k_1;
  this->y_k_1 << y_k;
  this->u_k_1 << u_k;
    
  return y_k;
}
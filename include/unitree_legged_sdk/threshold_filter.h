#pragma once

#include <stdlib.h>
#include <math.h>
#include <iostream>

namespace legged_robot {
  
class ThresholdFilter
{
public:
    ThresholdFilter();
    ThresholdFilter( double x_filter, double limit);
    ~ThresholdFilter();
    
    double Filter( double x );
    double GetXfilter();
    double GetLimit();
    void SetXfilter( double x_filter);
    void SetLimit( double limit);
private:
  double x_filter_;
  double limit_;
};

}
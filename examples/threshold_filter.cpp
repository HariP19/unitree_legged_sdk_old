#include "unitree_legged_sdk/threshold_filter.h"

namespace legged_robot
{
ThresholdFilter::ThresholdFilter()
{
    x_filter_ = 0;
    limit_ = 0;
}

ThresholdFilter::ThresholdFilter ( double x_filter, double limit )
{
    x_filter_ = x_filter;
    if ( limit <= 0 )
    {
        std::cout << " Limit of ThresholdFilter should be positive and now it has been inversed!!!" << std::endl;
    }
    limit_ = fabs ( limit );
}

ThresholdFilter::~ThresholdFilter()
{

}

double ThresholdFilter::Filter ( double x )
{
    if ( x - x_filter_ > limit_ )
    {
        x_filter_ += limit_ ;
    }
    else if ( x - x_filter_ < -limit_ )
    {
        x_filter_ -= limit_ ;
    }
    else
    {
        x_filter_ = x;
    }
    return x_filter_;
}

double ThresholdFilter::GetLimit()
{
    return limit_;
}

double ThresholdFilter::GetXfilter()
{
    return x_filter_;
}

void ThresholdFilter::SetLimit ( double limit )
{
    if ( limit <= 0 )
    {
        std::cout << " Limit of ThresholdFilter should be positive and now it has been inversed!!!" << std::endl;
    }
    limit_ = fabs ( limit );
}

void ThresholdFilter::SetXfilter ( double x_filter )
{
    x_filter_ = x_filter;
}

}

#ifndef __CUBIC_SPLINE_H__
#define __CUBIC_SPLINE_H__

class cubicSpline
{
public:
    typedef enum _BoundType
    {
        BoundType_First_Derivative,
        BoundType_Second_Derivative
    }BoundType;

public:
    cubicSpline();
    ~cubicSpline();

    void initParam();
    void releaseMem();

    bool loadData(double *x_data, double *y_data, int count, double bound1, double bound2, BoundType type);
    bool getYbyX(double &x_in, double &y_out);

protected:
    bool spline(BoundType type);

protected:
    double *x_sample_, *y_sample_;
    double *M_;
    int sample_count_;
    double bound1_, bound2_;
};

#endif /* __CUBIC_SPLINE_H__ */
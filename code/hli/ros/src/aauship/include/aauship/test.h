//=====================================================================================================
// MahonyAHRS.h
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MahonyAHRS_h
#define MahonyAHRS_h

class AHRS{
private:
 
    float KpA;                    // proportional gain (Kp)
    float KiA;                    // integral gain (Ki)
    float KpM;                    // proportional gain (Kp)
    float KiM;                    // integral gain (Ki)
    float q0, q1, q2, q3;            // quaternion of sensor frame relative to auxiliary frame
 
    float integralFBx;                // integral error terms scaled by Ki
    float integralFBy;                // integral error terms scaled by Ki
    float integralFBz;                // integral error terms scaled by Ki
 
    float correctedRateVectorX;     // Corrected Gyroscope Data
    float correctedRateVectorY;
    float correctedRateVectorZ;
 
    float eulerAngleX;
    float eulerAngleY;
    float eulerAngleZ;
 
    float ex;
    float ey;
    float ez;
    
    float magX;
    float magY;
    float magZ;
    
public:
    AHRS(float,float,float,float);
//    void initAHRS(float,float,float,float,float,float); 
    void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
    float invSqrt(float x);
//    void setTuning(float,float,float,float);
//    float getCorrectedRate(int);
//    float getEulerAngles(int);
//    void reset(void);
//    void calculateEulerAngles(void);
//    float getError(int);
};

#endif

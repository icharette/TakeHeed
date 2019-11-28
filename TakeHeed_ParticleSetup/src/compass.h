/* SOURCE:::
LSM9DS1_Basic_I2C.ino
SFE_LSM9DS1 Library Simple Example Code - I2C Interface
Jim Lindblom @ SparkFun Electronics
Original Creation Date: April 30, 2015
https://github.com/sparkfun/SparkFun_LSM9DS1_Particle_Library

*/

#include "Particle.h"

class IMU_sensor{

private:

//this code is derived from http://cache.freescale.com/files/sensors/doc/app_note/AN4248.pdf

/* roll pitch and yaw angles computed by iecompass */
static int16_t iPhi, iThe, iPsi;
/* magnetic field readings corrected for hard iron effects and PCB orientation */
static int16_t iBfx, iBfy, iBfz;
/* hard iron estimate */
static int16_t iVx, iVy, iVz;
/* tilt-compensated e-Compass code */

static const uint16_t MINDELTATRIG = 1; /* final step size for iTrig */
/* function to calculate ir = ix / sqrt(ix*ix+iy*iy) using binary division */

static const uint16_t MINDELTADIV = 1; /* final step size for iDivide */

/* fifth order of polynomial approximation giving 0.05 deg max error */
static const int16_t K1 = 5701;
static const int16_t K2 = -1645;
static const int16_t K3 = 446;

static float acceptedRangeLowBound;
static float acceptedRangeHighBound;
static float heading;
static int offset;

public:

IMU_sensor();
static void printIMUInfo();
static float getAcceptedRangeHighBound();
static float getAcceptedRangeLowBound();
static void iecompass(int16_t iBpx, int16_t iBpy, int16_t iBpz, int16_t iGpx, int16_t iGpy, int16_t iGpz);
static int16_t iTrig(int16_t ix, int16_t iy);
static int16_t iHundredAtan2Deg(int16_t iy, int16_t ix);
static int16_t iHundredAtanDeg(int16_t iy, int16_t ix);
static int16_t iDivide(int16_t iy, int16_t ix);

};
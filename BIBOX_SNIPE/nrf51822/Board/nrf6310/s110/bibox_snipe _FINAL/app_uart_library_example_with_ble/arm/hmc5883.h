#ifndef __HMC5883
#define __HMC5883

#include <math.h>
#include <stdbool.h>




#define HMC5883L_ADDRESS              (0x3c)
#define HMC5883L_REG_CONFIG_A         (0x00)
#define HMC5883L_REG_CONFIG_B         (0x01)
#define HMC5883L_REG_MODE             (0x02)
#define HMC5883L_REG_OUT_X_M          (0x03)
#define HMC5883L_REG_OUT_X_L          (0x04)
#define HMC5883L_REG_OUT_Z_M          (0x05)
#define HMC5883L_REG_OUT_Z_L          (0x06)
#define HMC5883L_REG_OUT_Y_M          (0x07)
#define HMC5883L_REG_OUT_Y_L          (0x08)
#define HMC5883L_REG_STATUS           (0x09)
#define HMC5883L_REG_IDENT_A          (0x0A)
#define HMC5883L_REG_IDENT_B          (0x0B)
#define HMC5883L_REG_IDENT_C          (0x0C)

#ifdef __CC_ARM 
	#define M_PI 3.141592653589793238462643
#endif
#define PI 3.1415926535897932384626433832795


typedef enum
{
    HMC5883L_SAMPLES_8     = 0x3,
    HMC5883L_SAMPLES_4     = 0x2,
    HMC5883L_SAMPLES_2     = 0x1,
    HMC5883L_SAMPLES_1     = 0x0
} hmc5883l_samples_t;

typedef enum
{
    HMC5883L_DATARATE_75HZ       = 0x6,
    HMC5883L_DATARATE_30HZ       = 0x5,
    HMC5883L_DATARATE_15HZ       = 0x4,
    HMC5883L_DATARATE_7_5HZ      = 0x3,
    HMC5883L_DATARATE_3HZ        = 0x2,
    HMC5883L_DATARATE_1_5HZ      = 0x1,
    HMC5883L_DATARATE_0_75_HZ    = 0x0
} hmc5883l_dataRate_t;

typedef enum
{
    HMC5883L_RANGE_8_1GA     = 0x7,
    HMC5883L_RANGE_5_6GA     = 0x6,
    HMC5883L_RANGE_4_7GA     = 0x5,
    HMC5883L_RANGE_4GA       = 0x4,
    HMC5883L_RANGE_2_5GA     = 0x3,
    HMC5883L_RANGE_1_9GA     = 0x2,
    HMC5883L_RANGE_1_3GA     = 0x1,
    HMC5883L_RANGE_0_88GA    = 0x0
} hmc5883l_range_t;

typedef enum
{
    HMC5883L_IDLE          = 0x2,
    HMC5883L_SINGLE        = 0x1,
    HMC5883L_CONTINOUS     = 0x0
} hmc5883l_mode_t;

#ifndef VECTOR_STRUCT_H
#define VECTOR_STRUCT_H
struct Vector
{
    float XAxis;
    float YAxis;
    float ZAxis;
};
#endif

/****************************************************************
**************** Initialize the hmc5883 *************************
****************************************************************/
bool Hmc5883_Initial(void);


/****************************************************************
**************** Read the raw values of x,y,x *******************
****************************************************************/
bool Hmc5883_readRawValues(signed int* X,signed int* Y,signed int* Z);



/****************************************************************
**************** Normalize the values   *************************
****************************************************************/
bool Hmc5883_readNormalize(void);



/****************************************************************
**************** Calculate the degree   *************************
****************************************************************/
bool Hmc5883_readDegree(float* deg);


#endif




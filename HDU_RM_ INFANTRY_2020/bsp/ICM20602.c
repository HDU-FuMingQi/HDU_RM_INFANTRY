#include "ICM20602.h"
#include "math.h" 
extern SPI_HandleTypeDef hspi5;

#define SPI5_NSS_Pin GPIO_PIN_6
#define SPI5_NSS_GPIO_Port GPIOF

static 	float _accel_scale;
static	float _gyro_scale;
static uint8_t tx, rx;
static uint8_t tx_buff[14];
#define ICM20602_ADDRESS	0xD2


struct _IMU_Vale IMU_Vale=
{
	0,0,0,
	0,0,0,
	0,0,0,
	0,0,0,
	0,0,0,
	0,0,0,
	0,0
};
  
uint8_t icm20602_read_buffer(uint8_t const regAddr, uint8_t *pData, uint8_t len)
{
  HAL_GPIO_WritePin(SPI5_NSS_GPIO_Port, SPI5_NSS_Pin,GPIO_PIN_RESET);
  tx = regAddr | 0x80;
  tx_buff[0] = tx;
  HAL_SPI_TransmitReceive(&hspi5, &tx, &rx, 1, 55);
  HAL_SPI_TransmitReceive(&hspi5, tx_buff, pData, len, 55);
  HAL_GPIO_WritePin(SPI5_NSS_GPIO_Port, SPI5_NSS_Pin,GPIO_PIN_SET);
  return 0;
}
 
uint8_t icm20602_write_reg(uint8_t reg,uint8_t val)
{
 
	HAL_GPIO_WritePin(SPI5_NSS_GPIO_Port, SPI5_NSS_Pin,GPIO_PIN_RESET);
	tx = reg & 0x7F;
  HAL_SPI_TransmitReceive(&hspi5, &tx, &rx, 1, 55);
  tx = val;
  HAL_SPI_TransmitReceive(&hspi5, &tx, &rx, 1, 55);
	HAL_GPIO_WritePin(SPI5_NSS_GPIO_Port, SPI5_NSS_Pin,GPIO_PIN_SET);
	return 0;
}
 
uint8_t icm20602_read_reg(uint8_t reg)
{
	HAL_GPIO_WritePin(SPI5_NSS_GPIO_Port, SPI5_NSS_Pin,GPIO_PIN_RESET);
	tx = reg | 0x80;
	HAL_SPI_TransmitReceive(&hspi5, &tx, &rx, 1, 55);
  HAL_SPI_TransmitReceive(&hspi5, &tx, &rx, 1, 55);
	HAL_GPIO_WritePin(SPI5_NSS_GPIO_Port, SPI5_NSS_Pin,GPIO_PIN_SET);
	return rx;
}
 
void ICM20602_Init()
{
    icm20602_write_reg(ICM20_PWR_MGMT_1,0x80);           // 复位
    HAL_Delay(50);
    icm20602_write_reg(ICM20_SIGNAL_PATH_RESET,0x07);
    HAL_Delay(50);
    icm20602_write_reg(ICM20_PWR_MGMT_1,0x01);           // 使用内部20M时钟
    HAL_Delay(50);
    icm20602_write_reg(ICM20_ACCEL_CONFIG,0x00);         // +-2G accel
    HAL_Delay(50);
    icm20602_write_reg(ICM20_ACCEL_CONFIG2,0x03);        // 41HZ加计低通滤波器
    HAL_Delay(50);
    icm20602_write_reg(ICM20_GYRO_CONFIG,0x18);          // +-2000 gyro
    HAL_Delay(50);
    icm20602_write_reg(ICM20_PWR_MGMT_2,0x00);
    HAL_Delay(50);
}

uint8_t icm20602_set_gyro_fullscale(uint8_t fs)
{
	switch(fs)
	{
		case ICM20_GYRO_FS_250:
			_gyro_scale = 1.0f/131.068f;	//32767/250
		break;
		case ICM20_GYRO_FS_500:
			_gyro_scale = 1.0f/65.534f;
		break;
		case ICM20_GYRO_FS_1000:
			_gyro_scale = 1.0f/32.767f;
		break;
		case ICM20_GYRO_FS_2000:
			_gyro_scale = 1.0f/16.4f;
		break;
		default:
			fs = ICM20_GYRO_FS_2000;
			_gyro_scale = 1.0f/16.3835f;
		break;
 
	}
	return icm20602_write_reg(ICM20_GYRO_CONFIG,fs);
}
 
uint8_t icm20602_set_accel_fullscale(uint8_t fs)
{
	switch(fs)
	{
		case ICM20_ACCEL_FS_2G:
			_accel_scale = 1.0f/16348.0f;
		break;
		case ICM20_ACCEL_FS_4G:
			_accel_scale = 1.0f/8192.0f;
		break;
		case ICM20_ACCEL_FS_8G:
			_accel_scale = 1.0f/4096.0f;
		break;
		case ICM20_ACCEL_FS_16G:
			_accel_scale = 1.0f/2048.0f;
		break;
		default:
			fs = ICM20_ACCEL_FS_8G;
			_accel_scale = 1.0f/4096.0f;
		break;
 
	}
	return icm20602_write_reg(ICM20_ACCEL_CONFIG,fs);
}
 

uint8_t icm20602_get_accel_adc(int16_t *accel)
{
	uint8_t buf[6];
	if(icm20602_read_buffer(ICM20_ACCEL_XOUT_H,buf,6))return 1;
	
	accel[0] = ((int16_t)buf[0]<<8) + buf[1];
	accel[1] = ((int16_t)buf[2]<<8) + buf[3];
	accel[2] = ((int16_t)buf[4]<<8) + buf[5];
	return 0;
}
 
uint8_t icm20602_get_gyro_adc(int16_t *gyro)
{
	uint8_t buf[6];
	if(icm20602_read_buffer(ICM20_GYRO_XOUT_H,buf,6))return 1;
	gyro[0] = (buf[0]<<8) + buf[1];
	gyro[1] = (buf[2]<<8) + buf[3];
	gyro[2] = (buf[4]<<8) + buf[5];
	return 0;
}
uint8_t icm20602_get_gyro(float *gyro)
{
	int16_t gyro_adc[3];
	if(icm20602_get_gyro_adc(gyro_adc))return 1;
	
	gyro[0] = _gyro_scale * gyro_adc[0];
	gyro[1] = _gyro_scale * gyro_adc[1];
	gyro[2] = _gyro_scale * gyro_adc[2];	
	return 0;
}
uint8_t icm20602_get_accel(float *accel)
{
	int16_t accel_adc[3];
	if(icm20602_get_accel_adc(accel_adc))return 1;
	accel[0] = _accel_scale * accel_adc[0];
	accel[1] = _accel_scale * accel_adc[1];
	accel[2] = _accel_scale * accel_adc[2];	
	return 0;
}
 
float icm20602_get_temp()
{
	int16_t temp_adc;
	uint8_t buf[2];
	if(icm20602_read_buffer(ICM20_TEMP_OUT_H,buf,2))return 0.0f;
 
	temp_adc = (buf[0]<<8)+buf[1];
 
	return (25.0f + (float)temp_adc/326.8f);
}


#define Buf_SIZE  5
int16_t MPU6500_FIFO[7][Buf_SIZE];
static unsigned char Wr_Index = 0;

static void MPU6500_NewVal(int16_t* buf,int16_t val) {
    buf[Wr_Index] = val;
}

static int16_t MPU6500_GetAvg(int16_t* buf)
{
    int i;
    int32_t sum = 0;
    for(i=0;i<Buf_SIZE;i++){
        sum += buf[i];
    }
    sum = sum / Buf_SIZE;
    return (int16_t)sum;
}

static int16_t gx,gy,gz;
static int16_t ax,ay,az;	
void MPU6500_readGyro_Acc(int16_t *gyro,int16_t *acc)
{
	static unsigned char buf[14];
	icm20602_read_buffer(ICM20_ACCEL_XOUT_H,buf,14);

	//acc
	MPU6500_NewVal(&MPU6500_FIFO[0][0],(int16_t)(((int16_t)buf[0]) << 8 | buf[1]));
	MPU6500_NewVal(&MPU6500_FIFO[1][0],(int16_t)(((int16_t)buf[2]) << 8 | buf[3]));
	MPU6500_NewVal(&MPU6500_FIFO[2][0],(int16_t)(((int16_t)buf[4]) << 8 | buf[5]));
	//temp
	MPU6500_NewVal(&MPU6500_FIFO[3][0],(int16_t)(((int16_t)buf[6]) << 8 | buf[7]));
	//gyro
	MPU6500_NewVal(&MPU6500_FIFO[4][0],(int16_t)(((int16_t)buf[8]) << 8 | buf[9]));
	MPU6500_NewVal(&MPU6500_FIFO[5][0],(int16_t)(((int16_t)buf[10]) << 8 | buf[11]));
	MPU6500_NewVal(&MPU6500_FIFO[6][0],(int16_t)(((int16_t)buf[12]) << 8 | buf[13]));

	Wr_Index = (Wr_Index + 1) % Buf_SIZE;

	gx =  MPU6500_GetAvg(&MPU6500_FIFO[4][0]);
	gy =  MPU6500_GetAvg(&MPU6500_FIFO[5][0]);
	gz =  MPU6500_GetAvg(&MPU6500_FIFO[6][0]);

	gyro[0] = gx - IMU_Vale.OffsetData_GyroX;   //gyro
	gyro[1] = gy - IMU_Vale.OffsetData_GyroY;
	gyro[2] = gz - IMU_Vale.OffsetData_GyroZ;

	ax = MPU6500_GetAvg(&MPU6500_FIFO[0][0]);
	ay = MPU6500_GetAvg(&MPU6500_FIFO[1][0]);
	az = MPU6500_GetAvg(&MPU6500_FIFO[2][0]);

	acc[0] = ax - IMU_Vale.OffsetData_AccX;     //acc
	acc[1] = ay - IMU_Vale.OffsetData_AccY;
	acc[2] = az;
}


#define sampleFreq	1000.0f			// sample frequency in Hz
#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain


//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float twoKp = twoKpDef;																		// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;																		// 2 * integral gain (Ki)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki

//float invSqrt(float x)
//{
//	float halfx = 0.5f * x;
//	float y = x;
//	long i = *(long*)&y;
//	i = 0x5f3759df - (i>>1);
//	y = *(float*)&i;
//	y = y * (1.5f - (halfx * y * y));
//	return y;
//}

float safe_asin(float v)
{
    if (isnan(v)) {
        return 0.0f;
    }
    if (v >= 1.0f) {
        return PI/2;
    }
    if (v <= -1.0f) {
        return -PI/2;
    }
    return asin(v);
}


void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;

		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
	float recipNorm;
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);
		return;
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
		hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
		bx = invSqrt(hx * hx + hy * hy);
		bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
		halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
		halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
		halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}


void IMU_getValues(float * values)
{
    int16_t accgyroval[9];
    int i;
    MPU6500_readGyro_Acc(&accgyroval[3],&accgyroval[0]);
    for(i = 0; i<6; i++){
        //转化为m/s^2
        if(i < 3){
            values[i] = ((float) accgyroval[i]) / 16384;//(量程/分辨率): 4/65536 = 1/16384 转化为m/s^2
        }
        //转化为rad/s
        else{
            values[i] = ((float) accgyroval[i]) / 16.384f;//(量程/分辨率): 4000/65536 = 1/16.384 转化为°/s
        }
    }
}


volatile float RDrone_R[3][3];
float Accel_Src[3];
void IMU_GetInfo()
{
    static float q[4];
    static float getValue[9];
    static float q0q0,q0q1,q0q2,q0q3,q1q1,q1q2,q1q3,q2q2,q2q3,q3q3;
    float Yaw_Temp;
    IMU_getValues(getValue);
    MahonyAHRSupdate(getValue[3]* PI/180, getValue[4]* PI/180, getValue[5]* PI/180,
                               getValue[0], getValue[1], getValue[2],0,0,0);
    q[0] = q0;
    q[1] = q1;
    q[2] = q2;
    q[3] = q3;
    //使用矩阵的时候可以快速使用
    q0q0 = q[0]*q[0];
    q0q1 = q[0]*q[1];
    q0q2 = q[0]*q[2];
    q0q3 = q[0]*q[3];
    q1q1 = q[1]*q[1];
    q1q2 = q[1]*q[2];
    q1q3 = q[1]*q[3];
    q2q2 = q[2]*q[2];
    q2q3 = q[2]*q[3];
    q3q3 = q[3]*q[3];

    RDrone_R[0][0] = q0q0 + q1q1 - q2q2 - q3q3;
    RDrone_R[0][1] = 2.0f * (q1q2 + q0q3);
    RDrone_R[0][2] = 2.0f * (q1q3 - q0q2);
    RDrone_R[1][0] = 2.0f * (q1q2 - q0q3);
    RDrone_R[1][1] = q0q0 - q1q1 + q2q2 - q3q3;
    RDrone_R[1][2] = 2.0f * (q2q3 + q0q1);
    RDrone_R[2][0] = 2.0f * (q1q3 + q0q2);
    RDrone_R[2][1] = 2.0f * (q2q3 - q0q1);
    RDrone_R[2][2] = q0q0 - q1q1 - q2q2 + q3q3;

    //加速度数据
    Accel_Src[0] = getValue[0];
    Accel_Src[1] = getValue[1];
    Accel_Src[2] = getValue[2];

    //角速度数据
    IMU_Vale.RateRoll = getValue[3] * 100;
    IMU_Vale.RatePitch= getValue[4] * 100;
    IMU_Vale.RateYaw  = getValue[5] * 100;

    //原始角速度数据
     IMU_Vale.GyroX = getValue[3];
     IMU_Vale.GyroY = getValue[4];
     IMU_Vale.GyroZ = getValue[5];

     //原始磁力计数据

//      RT_Info.MagX = getValue[6]  ;
//      RT_Info.MagY = getValue[7]  ;
//      RT_Info.MagZ = getValue[8]  ;

      //地理坐标系下的加速度
      IMU_Vale.accXaxis = ((q0q0 + q1q1 - q2q2 - q3q3)*getValue[0] + (2.f * (q1q2 - q0q3))*getValue[1]  +   (2.f * (q1q3 + q0q2))*getValue[2])  ;
      IMU_Vale.accYaxis = ((2.f * (q1q2 + q0q3))*getValue[0]  + (q0q0 - q1q1 + q2q2 - q3q3)*getValue[1] +   (2.f * (q2q3 - q0q1))*getValue[2]);
      IMU_Vale.accZaxis =  ((2.f * (q1q3 - q0q2))*getValue[0]   + (2.f * (q2q3 + q0q1))*getValue[1]        +   (q0q0 - q1q1 - q2q2 + q3q3)*getValue[2] - 9.797f);//转化为实际加速度

    //角度数据
    IMU_Vale.Roll = (atan2(2.0f*(q0q1 + q2q3),1 - 2.0f*(q1q1 + q2q2)))* 180/PI;
    IMU_Vale.Pitch = -safe_asin(2.0f*(q0q2 - q1q3))* 180/PI;
    Yaw_Temp = -atan2(2.0f*q1q2 + 2.0f*q0q3, -2.0f*q2q2 - 2.0f*q3q3 + 1) * 180/PI; // yaw
    if(Yaw_Temp<0)IMU_Vale.Yaw=Yaw_Temp+360;
    else IMU_Vale.Yaw=Yaw_Temp;


}

#define OFFSET_AV_NUM 500
u16 Offset_Cnt=0;
float sum_temp[6]={0};
u8 OffSet_Flag=0;
u8 IMU_Get_Offset()
{
	int16_t accgyroval[9];
	if(OffSet_Flag==1)
	{
		return 1;
	}
	if(OffSet_Flag==0)
	{
		MPU6500_readGyro_Acc(&accgyroval[3],&accgyroval[0]);
		Offset_Cnt++;
		if(Offset_Cnt<OFFSET_AV_NUM)
		{
			sum_temp[0] += gx*1.0;
			sum_temp[1] += gy*1.0;
			sum_temp[2] += gz*1.0;

			sum_temp[3] += ax*1.0;
			sum_temp[4] += ay*1.0;
			sum_temp[5] += az*1.0;		
		}
	}
	if(Offset_Cnt==OFFSET_AV_NUM)
	{
		OffSet_Flag=1;
		IMU_Vale.OffsetData_GyroX = sum_temp[0]/OFFSET_AV_NUM;
		IMU_Vale.OffsetData_GyroY = sum_temp[1]/OFFSET_AV_NUM; 
		IMU_Vale.OffsetData_GyroZ = sum_temp[2]/OFFSET_AV_NUM;

		IMU_Vale.OffsetData_AccX = sum_temp[3]/OFFSET_AV_NUM;
		IMU_Vale.OffsetData_AccY = sum_temp[4]/OFFSET_AV_NUM; 
		IMU_Vale.OffsetData_AccZ = sum_temp[5]/OFFSET_AV_NUM;
	}
	return 0;
}

//#define OFFSET_AV_NUM 100
//float sum_temp[6]={0};
//u16 sum_cnt = 0;
//void MPU_Data_Offset()
//{
//	if(IMU_Vale.acc_calibrate==1 || IMU_Vale.gyr_calibrate==1)
//	{
//    sum_cnt++;
//		
//		sum_temp[0] += gx*1.0;
//		sum_temp[1] += gy*1.0;
//		sum_temp[2] += gz*1.0;

//		sum_temp[3] += ax*1.0;
//		sum_temp[4] += ay*1.0;
//		sum_temp[5] += az*1.0;
//		
//    if( sum_cnt >= OFFSET_AV_NUM )
//		{			
//			Param.gyr_offset.x = sum_temp[0]/OFFSET_AV_NUM;
//			Param.gyr_offset.y = sum_temp[1]/OFFSET_AV_NUM;
//			Param.gyr_offset.z = sum_temp[2]/OFFSET_AV_NUM;

//			Param.acc_offset.x = sum_temp[3]/OFFSET_AV_NUM;
//			Param.acc_offset.y = sum_temp[4]/OFFSET_AV_NUM;
//			Param.acc_offset.z = sum_temp[5]/OFFSET_AV_NUM;
//			
//			sum_cnt =0;
//			Param_Save();
//						
//			IMU_Vale.OffsetData_GyroX = Param.gyr_offset.x;
//			IMU_Vale.OffsetData_GyroY = Param.gyr_offset.y; 
//			IMU_Vale.OffsetData_GyroZ = Param.gyr_offset.z;

//			IMU_Vale.OffsetData_AccX = Param.acc_offset.x;
//			IMU_Vale.OffsetData_AccY = Param.acc_offset.y; 
//			IMU_Vale.OffsetData_AccZ = Param.acc_offset.z;
//			
//			IMU_Vale.acc_calibrate = 0;
//			IMU_Vale.gyr_calibrate = 0;
//			
//			sum_temp[0] = 0;
//			sum_temp[1] = 0;
//			sum_temp[2] = 0;
//			sum_temp[3] = 0;
//			sum_temp[4] = 0;
//			sum_temp[5] = 0;

//		}
//	}
//}


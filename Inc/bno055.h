/*
 * bno055.h
 *
 *  Created on: 17 sept. 2018
 *      Author: j.solerh
 */

#ifndef BNO055_H_
#define BNO055_H_

#define BNO_ADDR 		0x29<<1		/*!< BNO Address           		 */
#define BNO_OPR_MODE 		0x3D		/*!< OPR_MODE Address      		  */
#define BNO_UNIT_SEL 		0x3B		/*!< UNIT_SEL Address		  		*/
#define BNO_SYS_TRIGGER		0x3F		/*!< SYS_TRIGGER Address     	   */
#define BNO_LIA_DATA_X_LSB 	0x28		/*!< LIA_DATA_X_LSB Address        */
#define BNO_LIA_DATA_X_MSB 	0x29		/*!< LIA_DATA_X_MSB Address        */
#define BNO_LIA_DATA_Y_LSB 	0x2A		/*!< LIA_DATA_Y_LSB Address        */
#define BNO_LIA_DATA_Y_MSB	0x2B		/*!< LIA_DATA_Y_MSB Address        */
#define BNO_LIA_DATA_Z_LSB 	0x2C		/*!< LIA_DATA_Z_LSB Address        */
#define BNO_LIA_DATA_Z_MSB 	0x2D		/*!< LIA_DATA_Z_MSB Address        */
#define BNO_GYR_DATA_X_LSB 	0x14		/*!< GRV_DATA_X_LSB Address        */
#define BNO_GYR_DATA_X_MSB 	0x15		/*!< GRV_DATA_X_MSB Address        */
#define BNO_GYR_DATA_Y_LSB 	0x16		/*!< GRV_DATA_Y_LSB Address        */
#define BNO_GYR_DATA_Y_MSB 	0x17		/*!< GRV_DATA_Y_MSB Address        */
#define BNO_GYR_DATA_Z_LSB 	0x18		/*!< GRV_DATA_Z_LSB Address        */
#define BNO_GYR_DATA_Z_MSB 	0x19		/*!< GRV_DATA_Z_MSB Address        */
#define BNO_GRV_DATA_X_LSB 	0x2E		/*!< GRV_DATA_X_LSB Address        */
#define BNO_GRV_DATA_X_MSB 	0x2F		/*!< GRV_DATA_X_MSB Address        */
#define BNO_GRV_DATA_Y_LSB 	0x30		/*!< GRV_DATA_Y_LSB Address        */
#define BNO_GRV_DATA_Y_MSB 	0x31		/*!< GRV_DATA_Y_MSB Address        */
#define BNO_GRV_DATA_Z_LSB 	0x32		/*!< GRV_DATA_Z_LSB Address        */
#define BNO_GRV_DATA_Z_MSB 	0x33		/*!< GRV_DATA_Z_MSB Address        */
#define BNO_EUL_DATA_X_LSB 	0x1A		/*!< GRV_DATA_X_LSB Address        */
#define BNO_EUL_DATA_X_MSB 	0x1B		/*!< GRV_DATA_X_MSB Address        */
#define BNO_EUL_DATA_Y_LSB 	0x1C		/*!< GRV_DATA_Y_LSB Address        */
#define BNO_EUL_DATA_Y_MSB 	0x1D		/*!< GRV_DATA_Y_MSB Address        */
#define BNO_EUL_DATA_Z_LSB 	0x1E		/*!< GRV_DATA_Z_LSB Address        */
#define BNO_EUL_DATA_Z_MSB 	0x1F		/*!< GRV_DATA_Z_MSB Address        */

typedef struct{

	int16_t BNO_LinearX;
	int16_t BNO_LinearY;
	int16_t BNO_LinearZ;

	int16_t BNO_GyroscopeX;
	int16_t BNO_GyroscopeY;
	int16_t BNO_GyroscopeZ;

	int16_t BNO_EulerX;
	int16_t BNO_EulerY;
	int16_t BNO_EulerZ;

}BNO_measure;

void BNO_Init(void);
void BNO_Read_Linear(BNO_measure *m);
void BNO_Read_Gyroscope(BNO_measure *m);
//void BNO_Read_Gravity(BNO_measure *m);
void BNO_Read_Euler(BNO_measure *m);
void BNO_Mide(BNO_measure *m);

#endif /* __BNO055_H_ */

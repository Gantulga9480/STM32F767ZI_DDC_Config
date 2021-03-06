/*
 * ddc.h
 *
 *  Created on: Dec 30, 2021
 *      Author: halo9
 */

#ifndef INC_DDC_H_
#define INC_DDC_H_

#define DDC_RESERVED            0x00
#define DDC_SOFT_RESET          0x01

#define MAX_RCF_RAM_SIZE        256
#define RCF_SIZE                100

#define SLAVE_SINGLE_REAL       0x00
#define SLAVE_DIVERSE_REAL      0x02
#define SLAVE_SINGLE_COMPLEX    0x04
#define MASTER_SINGLE_REAL      0x08
#define MASTER_DIVERSE_REAL     0x0A
#define MASTER_SINGLE_COMPLEX   0x0C

#define NCO_ACTIVE              0x00
#define NCO_BYPASS              0x01
#define NCO_ACTIVE_PD 			0x02
#define NCO_ACTIVE_AD           0x04
#define NCO_ACTIVE_PAD 			0x06

#define NCO_SYNC_MASK_DEFAULT   0x00FFFFFFFF
#define DDC_RCF_SCALE_DEFAULT   0x04

#define DDC_MODE  				0x300
#define DDC_NCO_MODE 			0x301
#define DDC_NCO_SYNC_MASK 		0x302
#define DDC_NCO_FREQUENCY 		0x303
#define DDC_NCO_PHASE_OFFSET 	0x304
#define DDC_CIC2_SCALE 			0x305
#define DDC_CIC2_DECIMATION 	0x306
#define DDC_CIC5_SCALE 			0x307
#define DDC_CIC5_DECIMATION 	0x308
#define DDC_RCF_SCALE 			0x309
#define DDC_RCF_DECIMATION 		0x30A
#define DDC_RCF_ADDRESS_OFFSET  0x30B
#define DDC_RCF_FILTER_TAPS     0x30C

// #define __DDC_ONE__
// #define __DDC_TWO__
#define __DDC_BOTH__

#if (defined(__DDC_ONE__) || defined(__DDC_TWO__)) && defined(__DDC_BOTH__)
#error DDC macro definition error. Select only one of (__DDC_ONE__, __DDC_TWO__, __DDC_BOTH__)
#endif

#if (defined(__DDC_ONE__) && defined(__DDC_TWO__))
#error DDC macro definition error. Select only one of (__DDC_ONE__, __DDC_TWO__, __DDC_BOTH__)
#endif

/*
 * Coefficients 5MHz
 * 500k      - 0x001999999A - NCO
 * 312.5k    - 0x0010000000 - NCO
 * 156.25k   - 0x0008000000 - NCO
 * 100k      - 0x00051EB852 - NCO
 * 10k       - 0x000083126F - NCO
*/

/*
 * Coefficients 30MHz
 * 500k      -  - NCO
 * 312.5k    -  - NCO
 * 156.25k   -  - NCO
 * 100k      - 0x0000DA740E - NCO
 * 10k       -  - NCO
*/

/* ATTENTION!!!
 * when using FIR
 * UPLOAD -> POWER RESET -> RUN */

typedef struct
{
	uint64_t DDC_Mode;   		 // Operating mode
	uint64_t NCO_Mode;  		 // NCO active, Phase-Amplitude Dither
	uint64_t NCO_SyncMask;  	 // NCO Sync mask
	uint64_t NCO_Frequency;  	 // NCO Frequency
	uint64_t NCO_PhaseOffset;  	 // NCO Phase offset
	uint64_t CIC2_Scale;  		 // In/CIC2 control
	uint64_t CIC2_Decimation;  	 // CIC2 Decimation N-1
	uint64_t CIC5_Scale;  		 // CIC5 Scale
	uint64_t CIC5_Decimation;  	 // CIC5 Decimation N-1
	uint64_t RCF_Scale;      	 // Out/RCF control
	uint64_t RCF_Decimation;     // RCF Decimation N-1
	uint64_t RCF_AddressOffset;  // RCF address offset
	uint64_t RCF_FilterTaps;     // N taps N-1
	uint8_t  FIR;				 // FIR using
} DDC_ConfigTypeDef;

void USR_DDC_Config_Init(DDC_ConfigTypeDef conf);
void USR_DDC_UdpHandler(uint8_t *udp_data);

uint64_t write_ddc(uint16_t address, uint64_t data);
uint64_t read_ddc(uint16_t address);

void hardReset();
void data_port_mode(uint32_t port_mode);
void ddc_rdy_int_mode(uint32_t mode);
void write_rcf_ram();
uint16_t uPort_read(uint8_t address);
uint8_t uPort_write(uint8_t address, uint8_t data);
uint16_t get_addr(uint8_t *s, int16_t start);
uint64_t get_value(uint8_t *s, int16_t start);
void flush_iq_ram();  // deprecated
void flush_RCF_ram();

#endif /* INC_DDC_H_ */

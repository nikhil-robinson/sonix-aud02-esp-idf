#ifndef _AUD02_H_
#define _AUD02_H_

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SET_BIT0 0x00000001
#define SET_BIT1 0x00000002
#define SET_BIT2 0x00000004
#define SET_BIT3 0x00000008
#define SET_BIT4 0x00000010
#define SET_BIT5 0x00000020
#define SET_BIT6 0x00000040
#define SET_BIT7 0x00000080
#define SET_BIT8 0x00000100
#define SET_BIT9 0x00000200
#define SET_BIT10 0x00000400
#define SET_BIT11 0x00000800
#define SET_BIT12 0x00001000
#define SET_BIT13 0x00002000
#define SET_BIT14 0x00004000
#define SET_BIT15 0x00008000
#define SET_BIT16 0x00010000
#define SET_BIT17 0x00020000
#define SET_BIT18 0x00040000
#define SET_BIT19 0x00080000
#define SET_BIT20 0x00100000
#define SET_BIT21 0x00200000
#define SET_BIT22 0x00400000
#define SET_BIT23 0x00800000
#define SET_BIT24 0x01000000
#define SET_BIT25 0x02000000
#define SET_BIT26 0x04000000
#define SET_BIT27 0x08000000
#define SET_BIT28 0x10000000
#define SET_BIT29 0x20000000
#define SET_BIT30 0x40000000
#define SET_BIT31 0x80000000

typedef enum {
  eAUD02_ERROR_CODE_SUCCESS = 0,
  eAUD02_ERROR_CODE_I2C_ERROR,
  eAUD02_ERROR_CODE_UNSUPPORTED_SAMPLING_RATE,
} AUD02_ERROR_CODE;

typedef enum { eAUD02_DISABLE = 0, eAUD02_ENABLE = 1 } AUD02_ABILITY;

typedef enum {
  eAUD02_LEFT_CHANNEL = 1,
  eAUD02_RIGHT_CHANNEL = 2
} AUD02_CHANNEL;

typedef enum {
  eAUD02_LEFT_CHANNEL_OUTPUT = 0,
  eAUD02_RIGHT_CHANNEL_OUTPUT = 1,
  eAUD02_MIX_RL_CHANNEL_OUTPUT = 2, /*(R+L)/2*/
} AUD02_AMP_OUT_CHANNEL_SEL;

typedef enum {
  eAUD_0Hz = 0U,
  eAUD_8000Hz = 8000U,
  eAUD_11025Hz = 11025U,
  eAUD_12000Hz = 12000U,
  eAUD_16000Hz = 16000U,
  eAUD_22050Hz = 22050U,
  eAUD_24000Hz = 24000U,
  eAUD_32000Hz = 32000U,
  eAUD_44100Hz = 44100U,
  eAUD_48000Hz = 48000U,
  eAUD_64000Hz = 64000U,
  eAUD_88200Hz = 88200U,
  eAUD_96000Hz = 96000U
} AUD_SAMPLING_RATE;

typedef enum {
  eAUD02_EXTERNAL_SYS_CLK = 0,
  eAUD02_INTERNAL_PLL_OUT = 1
} AUD02_CLOCK_SOURCE;

typedef enum { eAUD02_SLAVE = 0, eAUD02_MASTER = 1 } AUD02_MODE;

typedef enum {
  eAUD02_HPF_GAIN_0 = 0,
  eAUD02_HPF_GAIN_1 = 1,
  eAUD02_HPF_GAIN_2 = 2,
  eAUD02_HPF_GAIN_3 = 3,
  eAUD02_HPF_GAIN_4 = 4,
  eAUD02_HPF_GAIN_5 = 5,
  eAUD02_HPF_GAIN_6 = 6,
  eAUD02_HPF_GAIN_7 = 7,
  eAUD02_HPF_GAIN_8 = 8,
  eAUD02_HPF_GAIN_9 = 9,
  eAUD02_HPF_GAIN_10 = 10,
  eAUD02_HPF_GAIN_11 = 11,
  eAUD02_HPF_GAIN_12 = 12,
  eAUD02_HPF_GAIN_13 = 13,
  eAUD02_HPF_GAIN_14 = 14,
  eAUD02_HPF_GAIN_15 = 15
} AUD02_HPF_GAIN;

typedef enum {
  eAUD02_BOOST_GAIN_P_0dB = 0,
  eAUD02_BOOST_GAIN_P_20dB = 1,
  eAUD02_BOOST_GAIN_P_40dB = 2,
  eAUD02_BOOST_GAIN_P_50dB = 3
} AUD02_BOOST_GAIN;

typedef enum {
  eAUD02_PGA_GAIN_M_12dB = 0,
  eAUD02_PGA_GAIN_M_11P25dB = 1,
  eAUD02_PGA_GAIN_M_10P5dB = 2,
  eAUD02_PGA_GAIN_M_9P75dB = 3,
  eAUD02_PGA_GAIN_M_9dB = 4,
  eAUD02_PGA_GAIN_M_8P25dB = 5,
  eAUD02_PGA_GAIN_M_7P5dB = 6,
  eAUD02_PGA_GAIN_M_6P75dB = 7,
  eAUD02_PGA_GAIN_M_6dB = 8,
  eAUD02_PGA_GAIN_M_5P25dB = 9,
  eAUD02_PGA_GAIN_M_4P5dB = 10,
  eAUD02_PGA_GAIN_M_3P75dB = 11,
  eAUD02_PGA_GAIN_M_3dB = 12,
  eAUD02_PGA_GAIN_M_2P25dB = 13,
  eAUD02_PGA_GAIN_M_1P5dB = 14,
  eAUD02_PGA_GAIN_M_P75dB = 15,
  eAUD02_PGA_GAIN_P_0dB = 16,
  eAUD02_PGA_GAIN_P_0P75dB = 17,
  eAUD02_PGA_GAIN_P_1P5dB = 18,
  eAUD02_PGA_GAIN_P_2P25dB = 19,
  eAUD02_PGA_GAIN_P_3dB = 20,
  eAUD02_PGA_GAIN_P_3P75dB = 21,
  eAUD02_PGA_GAIN_P_4P5dB = 22,
  eAUD02_PGA_GAIN_P_5P25dB = 23,
  eAUD02_PGA_GAIN_P_6dB = 24,
  eAUD02_PGA_GAIN_P_6P75dB = 25,
  eAUD02_PGA_GAIN_P_7P5dB = 26,
  eAUD02_PGA_GAIN_P_8P25dB = 27,
  eAUD02_PGA_GAIN_P_9dB = 28,
  eAUD02_PGA_GAIN_P_9P75dB = 29,
  eAUD02_PGA_GAIN_P_10P5dB = 30,
  eAUD02_PGA_GAIN_P_11P25dB = 31,
  eAUD02_PGA_GAIN_P_12dB = 32,
  eAUD02_PGA_GAIN_P_12P75dB = 33,
  eAUD02_PGA_GAIN_P_13P5dB = 34,
  eAUD02_PGA_GAIN_P_14P25dB = 35,
  eAUD02_PGA_GAIN_P_15dB = 36,
  eAUD02_PGA_GAIN_P_15P75dB = 37,
  eAUD02_PGA_GAIN_P_16P5dB = 38,
  eAUD02_PGA_GAIN_P_17P25dB = 39,
  eAUD02_PGA_GAIN_P_18dB = 40,
  eAUD02_PGA_GAIN_P_18P75dB = 41,
  eAUD02_PGA_GAIN_P_19P5dB = 42,
  eAUD02_PGA_GAIN_P_20P25dB = 43,
  eAUD02_PGA_GAIN_P_21dB = 44,
  eAUD02_PGA_GAIN_P_21P75dB = 45,
  eAUD02_PGA_GAIN_P_22P5dB = 46,
  eAUD02_PGA_GAIN_P_23P25dB = 47,
  eAUD02_PGA_GAIN_P_24dB = 48,
  eAUD02_PGA_GAIN_P_24P75dB = 49,
  eAUD02_PGA_GAIN_P_25P5dB = 50,
  eAUD02_PGA_GAIN_P_26P25dB = 51,
  eAUD02_PGA_GAIN_P_27dB = 52,
  eAUD02_PGA_GAIN_P_27P75dB = 53,
  eAUD02_PGA_GAIN_P_28P5dB = 54,
  eAUD02_PGA_GAIN_P_29P25dB = 55,
  eAUD02_PGA_GAIN_P_30dB = 56,
  eAUD02_PGA_GAIN_P_30P75dB = 57,
  eAUD02_PGA_GAIN_P_31P5dB = 58,
  eAUD02_PGA_GAIN_P_32P25dB = 59,
  eAUD02_PGA_GAIN_P_33dB = 60,
  eAUD02_PGA_GAIN_P_33P75dB = 61,
  eAUD02_PGA_GAIN_P_34P5dB = 62,
  eAUD02_PGA_GAIN_P_35P25dB = 63
} AUD02_PGA_GAIN;

typedef enum {
  AMP_OPA_VOL_m24dB = 0,
  AMP_OPA_VOL_m18dB,
  AMP_OPA_VOL_m14p5dB,
  AMP_OPA_VOL_m12dB,
  AMP_OPA_VOL_m8p51dB,
  AMP_OPA_VOL_m6dB,
  AMP_OPA_VOL_m2p5dB,
  AMP_OPA_VOL_0dB,
} AUD02_AMP_GAIN;

typedef enum { DATA_OPT_0p85dB = 0, DATA_OPT_0p75dB } AUD_AMP_DATA_OPT;

typedef enum {
  CLASSD_0dB = 0,
  CLASSD_6dB,
} AUD02_CLASSD_GAIN;

typedef struct {
  // i2c_handler_t           pi2c;
  AUD02_MODE eMode;
  AUD02_CLOCK_SOURCE eClockSource;
  AUD_SAMPLING_RATE eSamplingRate;
  union {
    struct {
      AUD02_BOOST_GAIN eBoostGainL;
      AUD02_PGA_GAIN ePgaGainL;
    } ADC;
    struct {
      AUD02_AMP_GAIN eAmpGain;
    } DAC;
  };
} AUD02_STRUCT;

typedef void *aud02_handle_t;


AUD02_ERROR_CODE AUD_ADC_Init(AUD02_STRUCT *pAUD02Struct);
AUD02_ERROR_CODE AUD_DAC_Init(AUD02_STRUCT *pAUD02Struct);
AUD02_ERROR_CODE AUD_ADC_DeInit(void);
AUD02_ERROR_CODE AUD_DAC_DeInit(void);
/* AUD02 ADC Only Channel Left */
AUD02_ERROR_CODE AUD_Set_ADC_Channel_Filter(AUD02_ABILITY bDcCancellation,
                                            AUD02_ABILITY bNoiseSuppression,
                                            AUD02_ABILITY bOffsetCancellation,
                                            AUD02_HPF_GAIN eHpfGain);
AUD02_ERROR_CODE AUD_Set_ADC_Channel_Gain(AUD02_BOOST_GAIN eBoostGain,AUD02_PGA_GAIN ePgaGain);
AUD02_ERROR_CODE AUD_Set_ADC_Sampling_Rate(AUD_SAMPLING_RATE eSamplingRate);
AUD02_ERROR_CODE AUD_Set_DAC_Sampling_Rate(AUD_SAMPLING_RATE eSamplingRate);
AUD02_ERROR_CODE AUD_Set_AMP_Gain(AUD02_AMP_GAIN eAmpGain);
AUD02_ERROR_CODE AUD_Set_ClassD_Gain(AUD_AMP_DATA_OPT digital_gain, AUD02_CLASSD_GAIN vol_gain);
AUD02_ERROR_CODE AUD_Standby_Mode(void);
/* AUD02 New API */
AUD02_ERROR_CODE AUD_AMP_PWM_Mute(AUD02_ABILITY eMute);
AUD02_ERROR_CODE AUD_AMP_Mute(AUD02_ABILITY eMute);
AUD02_ERROR_CODE AUD_AMP_Output_Select(AUD02_AMP_OUT_CHANNEL_SEL eChannelOutput); /* Select the source of AMP output data */
void AUD_DAC_Mute(void);
void AUD_DAC_UnMute(void);
AUD02_ERROR_CODE AUD_DAC_SW_reset(void);


/**
 * @brief Create and init sensor object and return a sensor handle
 *
 * @param bus I2C bus object handle
 * @param dev_addr I2C device address of sensor
 *
 * @return
 *     - NULL Fail
 *     - Others Success
 */
aud02_handle_t aud02_create(i2c_bus_handle_t bus, uint8_t dev_addr);

/**
 * @brief Delete and release a sensor object
 *
 * @param sensor point to object handle of aud02
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t aud02_delete(aud02_handle_t *sensor);


#ifdef __cplusplus
}
#endif
#endif
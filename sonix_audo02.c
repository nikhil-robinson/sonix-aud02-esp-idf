#include "aud02.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include <math.h>
#define NO 0
#define YES 1
/* Weakly Drive mode define = 0, use headphone op to weakly drive RHP LHP , AC
 * COUPLED VDD_SPK_5V = 1mA */
/* Weakly Drive mode define = 0, use headphone op to weakly drive RHP LHP, VCOM
 * BUF turn on , CAPLESS VDD_SPK_5V = 1.4mA */
/* Weakly Drive mode define = 1, use vrefspk resistor to weakly drive RHP LHP ,
 * AC COUPLED VDD_SPK_5V = 70uA */
/* Weakly Drive mode define = 1, use vrefspk resistor to weakly drive RHP LHP
 * VCOM , CAPLESS VDD_SPK_5V = 70uA */
#define AUD02_WEAKLY_DRIVE_MODE 1
#define AUD02_ID 0x19 // 0X32 << 1
#define SNAUD_PRINTF_REG(str, reg, rdata)                                      \
  printf("[SNAUD02] [REG:%s] Reg_Addr=0x%x Rdata=0x%x \r\n", str, reg, rdata);

static const char *TAG = "AUD02";

typedef enum {
  eAUD02_REG_SYS_CLK_CTRL = 0x00,
  eAUD02_REG_FPLL_CTRL = 0x01,
  eAUD02_REG_LDO_CTRL = 0x02,
  eAUD02_REG_PMU_CTRL = 0x03,
  eAUD02_REG_FPLL_REG0 = 0x04,
  eAUD02_REG_FPLL_REG1 = 0x05,
  eAUD02_REG_FPLL_REG2 = 0x06,
  eAUD02_REG_FPLL_INT = 0x07,
  eAUD02_REG_I2S_ADC_CTRL0 = 0x08,
  eAUD02_REG_I2S_ADC_CTRL1 = 0x09,
  eAUD02_REG_I2S_DAC_CTRL0 = 0x0A,
  eAUD02_REG_I2S_DAC_CTRL1 = 0x0B,
  eAUD02_REG_I2C_ALTET_ADDR = 0x0C,
  eAUD02_REG_SWRST_CTRL = 0x0F,
  eAUD02_REG_ADC_MIC_CTRL = 0x10,
  eAUD02_REG_ADC_CTRL0 = 0x11,
  eAUD02_REG_ADC_CTRL1 = 0x12,
  eAUD02_REG_ADC_CTRL2 = 0x13,
  eAUD02_REG_ADC_ZCU_CTRL = 0x14,
  eAUD02_REG_ADC_PGA_L = 0x17,
  eAUD02_REG_ADC_FILTER_L = 0x18,
  eAUD02_REG_ADC_TEST = 0x19,
  eAUD02_REG_AMP_CTRL0 = 0x40,
  eAUD02_REG_AMP_CTRL1 = 0x41,
  eAUD02_REG_AMP_CTRL2 = 0x42,
  eAUD02_REG_DEBUG_MODE0 = 0x50,
} AUD02_REGISTER;

typedef enum { eAUD02_ADC = 0, eAUD02_DAC } AUD02_CODEC;

/*ADC*/
static AUD02_MODE eAdcMode;
static AUD02_CLOCK_SOURCE eAdcClockSource;
/*DAC*/
static AUD02_MODE eDacMode;
static AUD02_CLOCK_SOURCE eDacClockSource;

#define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ

typedef struct {
    i2c_bus_device_handle_t i2c_dev;
    uint8_t dev_addr;
} aud02_dev_t;


static aud02_handle_t aud02_handel = NULL:

aud02_handle_t aud02_create(i2c_bus_handle_t bus, uint8_t dev_addr)
{
    if (bus == NULL) {
        return NULL;
    }

    aud02_dev_t *sens = (aud02_dev_t *) calloc(1, sizeof(aud02_dev_t));
    sens->i2c_dev = i2c_bus_device_create(bus, dev_addr, i2c_bus_get_current_clk_speed(bus));
    if (sens->i2c_dev == NULL) {
        free(sens);
        return NULL;
    }
    sens->dev_addr = dev_addr;
    aud02_handel = (aud02_handle_t) sens;
    return aud02_handel;
}

esp_err_t aud02_delete()
{
    if (*aud02_handel == NULL) {
        return ESP_OK;
    }

    aud02_dev_t *sens = (aud02_dev_t *)(*aud02_handel);
    i2c_bus_device_delete(&sens->i2c_dev);
    free(sens);
    *aud02_handel = NULL;
    return ESP_OK;
}

/**********************aud02*********************************/

// Prototype: 	void AUD02_Write_Register(AUD02_REGISTER eRegister, unsigned short
// Data) Header File: Arguments��eRegister: AUD02_REGISTER enum-type��Write
// AUD02 register address 			 data:      send the data Return��		None
// Description��	Send data to the AUD02 register using I2C
// Notes��       None
// Example:
esp_err_t AUD02_Write_Register(AUD02_REGISTER eRegister, unsigned short Data) {

    aud02_dev_t *sens = (aud02_dev_t *) aud02_handel;
    return i2c_bus_write_byte(sens->i2c_dev, eRegister, Data);
}

// Prototype: 	void AUD02_Read_Register(AUD02_REGISTER eRegister, unsigned short*
// Data) Header File: Arguments��eRegister: AUD02_REGISTER enum-type��Read AUD02
// register address 			 Data:    The read data is stored at this address Return��
// None Description��	Read AUD02 register data  using I2C Notes��       None
// Example:
esp_err_t AUD02_Read_Register(AUD02_REGISTER eRegister, unsigned short *Data) {

    aud02_dev_t *sens = (aud02_dev_t *) aud02_handel;
    return i2c_bus_read_byte(sens->i2c_dev, eRegister, Data);

}

static AUD02_ERROR_CODE System_Power_Up(AUD02_MODE eMode,
                                        AUD02_CLOCK_SOURCE eClockSource,
                                        AUD02_CODEC eCodec,
                                        AUD_SAMPLING_RATE eSamplingRate) {
  unsigned short wdata, rdata;
  AUD02_REGISTER I2S_CTRL0, I2S_CTRL1;
  uint32_t div_value = 1;
  uint8_t set_div = 0;
  /* check I2C Communication. Please check MCLK and PWR_5V if API return
   * eAUD02_ERROR_CODE_I2C_ERROR !  */
  if (AUD02_Read_Register(eAUD02_REG_SYS_CLK_CTRL, &rdata) != 0)
    return eAUD02_ERROR_CODE_I2C_ERROR;

  /*MCLK	* INT.FRA	/ VCO	/ BCLK	/ I2S bits*/
  /*12M	* 9.408		/ 10	/ 4		/ 64  =  44.1k*/

  AUD02_Write_Register(eAUD02_REG_FPLL_INT, 0x09);

  switch (eSamplingRate) {
  case eAUD_64000Hz:
    /* 9 + 584406/1048576 = 9.557 */
    AUD02_Write_Register(eAUD02_REG_FPLL_REG0, 0x08); /*0x08EAD6 = 584406*/
    AUD02_Write_Register(eAUD02_REG_FPLL_REG1, 0xEA);
    AUD02_Write_Register(eAUD02_REG_FPLL_REG2, 0xD6);
    break;

  case eAUD_44100Hz:
  case eAUD_22050Hz:
  case eAUD_11025Hz:
    /* 9 + 427819/1048576 = 9.408 */
    AUD02_Write_Register(eAUD02_REG_FPLL_REG0, 0x06); /*0x06872B = 427819*/
    AUD02_Write_Register(eAUD02_REG_FPLL_REG1, 0x87);
    AUD02_Write_Register(eAUD02_REG_FPLL_REG2, 0x2B);
    break;

  case eAUD_96000Hz:
  case eAUD_48000Hz:
  case eAUD_32000Hz:
  case eAUD_24000Hz:
  case eAUD_16000Hz:
  case eAUD_12000Hz:
  case eAUD_8000Hz:
    /* 9 + 226492/1048576 = 9.216 */
    AUD02_Write_Register(eAUD02_REG_FPLL_REG0, 0x03); /*0x0374BC = 226492*/
    AUD02_Write_Register(eAUD02_REG_FPLL_REG1, 0x74);
    AUD02_Write_Register(eAUD02_REG_FPLL_REG2, 0xBC);
    break;

  default:
    return eAUD02_ERROR_CODE_UNSUPPORTED_SAMPLING_RATE;
  }

  if (eClockSource == eAUD02_INTERNAL_PLL_OUT) {
    switch (eSamplingRate) {
    case eAUD_96000Hz:
    case eAUD_48000Hz:
    case eAUD_32000Hz:
    case eAUD_24000Hz:
    case eAUD_16000Hz:
    case eAUD_12000Hz:
    case eAUD_8000Hz:
      /* FPLL VCO / 9 */
      AUD02_Write_Register(eAUD02_REG_SYS_CLK_CTRL, 0x88);
      break;

    case eAUD_44100Hz:
    case eAUD_22050Hz:
    case eAUD_11025Hz:
      /* FPLL VCO / 10 */
      AUD02_Write_Register(eAUD02_REG_SYS_CLK_CTRL, 0x98);
      break;

    case eAUD_64000Hz:
      /* FPLL VCO / 14 */
      AUD02_Write_Register(eAUD02_REG_SYS_CLK_CTRL, 0xA8);
      break;

    default:
      return eAUD02_ERROR_CODE_UNSUPPORTED_SAMPLING_RATE;
    }

    /* wait until FPLL ready */
    rdata = 0;
    while ((rdata & 0x01) != 0x01) {
      AUD02_Read_Register(eAUD02_REG_SYS_CLK_CTRL, &rdata);
    }

    AUD02_Write_Register(eAUD02_REG_FPLL_CTRL, 0x46);
  } else // if (eClockSource == eAUD02_EXTERNAL_SYS_CLK)
  {
    AUD02_Write_Register(eAUD02_REG_SYS_CLK_CTRL, 0x00);
  }

  AUD02_Write_Register(eAUD02_REG_LDO_CTRL, 0x00);
  AUD02_Write_Register(eAUD02_REG_PMU_CTRL, 0x00);

  if (eCodec == eAUD02_ADC) {
    I2S_CTRL0 = eAUD02_REG_I2S_ADC_CTRL0;
    I2S_CTRL1 = eAUD02_REG_I2S_ADC_CTRL1;
  } else // if (eCodec == eAUD02_DAC)
  {
    I2S_CTRL0 = eAUD02_REG_I2S_DAC_CTRL0;
    I2S_CTRL1 = eAUD02_REG_I2S_DAC_CTRL1;
  }

  wdata = 0;

  if (eClockSource == eAUD02_INTERNAL_PLL_OUT) {
    wdata |= SET_BIT1;
  } else // if (eClockSource == eAUD02_EXTERNAL_SYS_CLK)
  {
    wdata &= ~(SET_BIT1);
  }

  if (eMode == eAUD02_MASTER) {
    wdata |= SET_BIT0;
  } else // if (eMode == eAUD02_SLAVE)
  {
    wdata &= ~(SET_BIT0);
  }
  AUD02_Write_Register(I2S_CTRL0, wdata);

  switch (eSamplingRate) {
  case eAUD_64000Hz:
    /* BCLKDIV = 2 */
    AUD02_Write_Register(I2S_CTRL1, 0x00);
    break;
  case eAUD_96000Hz:
  case eAUD_48000Hz:
  case eAUD_32000Hz:
  case eAUD_24000Hz:
  case eAUD_16000Hz:
  case eAUD_12000Hz:
  case eAUD_8000Hz:

    div_value = 12288000 / 64 / eSamplingRate;
    set_div = (div_value - 2) / 2;
    AUD02_Write_Register(I2S_CTRL1, set_div);
    break;

  case eAUD_44100Hz:
  case eAUD_22050Hz:
  case eAUD_11025Hz:

    div_value = 11289600 / 64 / eSamplingRate;
    set_div = (div_value - 2) / 2;
    AUD02_Write_Register(I2S_CTRL1, set_div);
    break;

  default:
    return eAUD02_ERROR_CODE_UNSUPPORTED_SAMPLING_RATE;
  }

  return eAUD02_ERROR_CODE_SUCCESS;
}

static AUD02_ERROR_CODE DAC_Power_Up(AUD02_AMP_GAIN eAmpGain) {
  AUD02_Write_Register(eAUD02_REG_AMP_CTRL0, 0x13);
  AUD02_Write_Register(eAUD02_REG_AMP_CTRL1, (eAmpGain << 2));
  AUD02_Write_Register(eAUD02_REG_AMP_CTRL2, 0x23);
  AUD02_Write_Register(eAUD02_REG_LDO_CTRL, 0x04);

  return eAUD02_ERROR_CODE_SUCCESS;
}

static AUD02_ERROR_CODE DAC_Power_Down() {
  AUD02_Write_Register(eAUD02_REG_AMP_CTRL0, 0x00);
  AUD02_Write_Register(eAUD02_REG_AMP_CTRL1, 0x00);
  AUD02_Write_Register(eAUD02_REG_AMP_CTRL2, 0x01);
  AUD02_Write_Register(eAUD02_REG_LDO_CTRL, 0x00);

  return eAUD02_ERROR_CODE_SUCCESS;
}

static AUD02_ERROR_CODE ADC_Power_Up(AUD02_BOOST_GAIN boostgain,
                                     AUD02_PGA_GAIN pga_gain) {
  unsigned short rdata;

  AUD02_Write_Register(eAUD02_REG_ADC_TEST, 0x33);
  AUD02_Write_Register(eAUD02_REG_ADC_MIC_CTRL, 0xD9);
  AUD02_Write_Register(eAUD02_REG_ADC_CTRL0, 0xEE); // E0
  vTaskDelay(200 / portTICK_PERIOD_MS);

  AUD02_Write_Register(eAUD02_REG_ADC_CTRL1, 0xEE); // E0
  AUD02_Write_Register(eAUD02_REG_ADC_CTRL2, 0x39); // 30
  AUD02_Write_Register(eAUD02_REG_ADC_ZCU_CTRL, 0x00);
  AUD02_Write_Register(eAUD02_REG_ADC_PGA_L, boostgain << 6 | pga_gain); // 10
  AUD02_Write_Register(eAUD02_REG_ADC_FILTER_L, 0xB7);
  AUD02_Read_Register(eAUD02_REG_LDO_CTRL, &rdata);
  AUD02_Write_Register(eAUD02_REG_LDO_CTRL, rdata | (0x01 << 2));

  return eAUD02_ERROR_CODE_SUCCESS;
}

/*
power consumption ??
*/
static AUD02_ERROR_CODE ADC_Power_Down(void) {
  AUD02_Write_Register(eAUD02_REG_ADC_MIC_CTRL, 0x00);
  AUD02_Write_Register(eAUD02_REG_ADC_CTRL0, 0x11);
  AUD02_Write_Register(eAUD02_REG_ADC_CTRL1, 0x11);
  AUD02_Write_Register(eAUD02_REG_ADC_PGA_L, 0x00);
  return eAUD02_ERROR_CODE_SUCCESS;
}

AUD02_ERROR_CODE AUD_Set_ADC_Channel_Gain(AUD02_BOOST_GAIN eBoostGain,
                                          AUD02_PGA_GAIN ePgaGain) {
  AUD02_Write_Register(eAUD02_REG_ADC_PGA_L,
                 (unsigned short)(eBoostGain << 6) + (unsigned short)ePgaGain);
  return eAUD02_ERROR_CODE_SUCCESS;
}

AUD02_ERROR_CODE AUD_ADC_Init(AUD02_STRUCT *pAUD02Struct) {
  AUD02_ERROR_CODE eErrorCode;

  eAdcMode = pAUD02Struct->eMode;
  eAdcClockSource = pAUD02Struct->eClockSource;

  if ((eErrorCode = System_Power_Up(
           pAUD02Struct->eMode, pAUD02Struct->eClockSource, eAUD02_ADC,
           pAUD02Struct->eSamplingRate)) != eAUD02_ERROR_CODE_SUCCESS)
    return eErrorCode;

  ADC_Power_Up(pAUD02Struct->ADC.eBoostGainL, pAUD02Struct->ADC.ePgaGainL);

  return eAUD02_ERROR_CODE_SUCCESS;
}

AUD02_ERROR_CODE AUD_ADC_DeInit(void) { return ADC_Power_Down(); }

AUD02_ERROR_CODE AUD_Set_ADC_Sampling_Rate(AUD_SAMPLING_RATE eSamplingRate) {
  return System_Power_Up(eAdcMode, eAdcClockSource, eAUD02_ADC, eSamplingRate);
}

AUD02_ERROR_CODE AUD_Set_ADC_Channel_Filter(AUD02_ABILITY bDcCancellation,
                                            AUD02_ABILITY bNoiseSuppression,
                                            AUD02_ABILITY bOffsetCancellation,
                                            AUD02_HPF_GAIN eHpfGain) {
  unsigned short wdata;
  unsigned char DC, Noise, Offset;

  DC = (~bDcCancellation) & 0x01;
  Noise = (~bNoiseSuppression) & 0x01;
  Offset = (~bOffsetCancellation) & 0x01;

  wdata = (DC << 7) | (Noise << 5) | (Offset << 4) | eHpfGain;
  AUD02_Write_Register(eAUD02_REG_ADC_FILTER_L, wdata);

  return eAUD02_ERROR_CODE_SUCCESS;
}

AUD02_ERROR_CODE AUD_Set_AMP_Gain(AUD02_AMP_GAIN eAmpGain) {
  AUD02_Write_Register(eAUD02_REG_AMP_CTRL1, 0x80 | (eAmpGain << 2));

  return eAUD02_ERROR_CODE_SUCCESS;
}

AUD02_ERROR_CODE AUD_AMP_PWM_Mute(AUD02_ABILITY eMute) {
  unsigned short rdata;

  AUD02_Read_Register(eAUD02_REG_AMP_CTRL0, &rdata);

  if (eMute == eAUD02_ENABLE) {
    AUD02_Write_Register(eAUD02_REG_AMP_CTRL0, (rdata & 0xF7) | 0x08);
  } else {
    AUD02_Write_Register(eAUD02_REG_AMP_CTRL0, rdata & 0xF7);
  }

  return eAUD02_ERROR_CODE_SUCCESS;
}

AUD02_ERROR_CODE AUD_AMP_Mute(AUD02_ABILITY eMute) {
  unsigned short rdata;

  AUD02_Read_Register(eAUD02_REG_AMP_CTRL0, &rdata);

  if (eMute == eAUD02_ENABLE) {
    AUD02_Write_Register(eAUD02_REG_AMP_CTRL0, (rdata & 0xFB) | 0x04);
  } else {
    AUD02_Write_Register(eAUD02_REG_AMP_CTRL0, rdata & 0xFB);
  }

  return eAUD02_ERROR_CODE_SUCCESS;
}

void AUD_DAC_Mute(void) { AUD_AMP_PWM_Mute(eAUD02_ENABLE); }

void AUD_DAC_UnMute(void) { AUD_AMP_PWM_Mute(eAUD02_DISABLE); }

AUD02_ERROR_CODE
AUD_AMP_Output_Select(AUD02_AMP_OUT_CHANNEL_SEL eChannelOutput) {
  unsigned short rdata;

  AUD02_Read_Register(eAUD02_REG_AMP_CTRL0, &rdata);
  switch (eChannelOutput) {
  case eAUD02_LEFT_CHANNEL_OUTPUT:
    AUD02_Write_Register(eAUD02_REG_AMP_CTRL2, (0x01 | eChannelOutput << 4));
    break;
  case eAUD02_RIGHT_CHANNEL_OUTPUT:
    AUD02_Write_Register(eAUD02_REG_AMP_CTRL2, (0x01 | eChannelOutput << 4));
    break;
  case eAUD02_MIX_RL_CHANNEL_OUTPUT:
    AUD02_Write_Register(eAUD02_REG_AMP_CTRL2, (0x01 | eChannelOutput << 4));
    break;
  default: /* select left */
    /* AUD02_Write_Register(eAUD02_REG_AMP_CTRL1,0x01) */
    break;
  }

  return eAUD02_ERROR_CODE_SUCCESS;
}

AUD02_ERROR_CODE AUD_DAC_Init(AUD02_STRUCT *pAUD02Struct) {
  AUD02_ERROR_CODE eErrorCode;

  eDacMode = pAUD02Struct->eMode;
  eDacClockSource = pAUD02Struct->eClockSource;

  if ((eErrorCode = System_Power_Up(
           pAUD02Struct->eMode, pAUD02Struct->eClockSource, eAUD02_DAC,
           pAUD02Struct->eSamplingRate)) != eAUD02_ERROR_CODE_SUCCESS)
    return eErrorCode;

  DAC_Power_Up(pAUD02Struct->DAC.eAmpGain);

  return eAUD02_ERROR_CODE_SUCCESS;
}

AUD02_ERROR_CODE AUD_DAC_DeInit(void) {
  AUD02_ERROR_CODE eErrorCode;

  {
    if (((eErrorCode = DAC_Power_Down()) != eAUD02_ERROR_CODE_SUCCESS))
      return eErrorCode;
  }

  return eAUD02_ERROR_CODE_SUCCESS;
}

AUD02_ERROR_CODE AUD_Set_DAC_Sampling_Rate(AUD_SAMPLING_RATE eSamplingRate) {
  return System_Power_Up(eDacMode, eDacClockSource, eAUD02_DAC, eSamplingRate);
}

/* Enter Lower Power Mode */
AUD02_ERROR_CODE AUD_Standby_Mode(void) {
  /* Disenable MCLK for Controller */

  return eAUD02_ERROR_CODE_SUCCESS;
}
AUD02_ERROR_CODE AUD_SW_reset(void) {

  AUD02_Write_Register(eAUD02_REG_I2C_ALTET_ADDR, 0x51);

  return eAUD02_ERROR_CODE_SUCCESS;
}

AUD02_ERROR_CODE AUD_DAC_SW_reset(void) {

  AUD02_Write_Register(eAUD02_REG_AMP_CTRL2, 0x22);
  AUD02_Write_Register(eAUD02_REG_AMP_CTRL2, 0x23);
  return eAUD02_ERROR_CODE_SUCCESS;
}

AUD02_ERROR_CODE AUD_Set_ClassD_Gain(AUD_AMP_DATA_OPT digital_gain,
                                     AUD02_CLASSD_GAIN vol_gain) {
  unsigned short rdata;

  AUD02_Read_Register(eAUD02_REG_AMP_CTRL0, &rdata);

  if (vol_gain == CLASSD_0dB)
    rdata &= ~(1 << 5);
  else
    rdata |= (1 << 5);

  if (digital_gain == DATA_OPT_0p85dB)
    rdata &= ~(1 << 4);
  else
    rdata |= (1 << 4);

  AUD02_Write_Register(eAUD02_REG_AMP_CTRL0, rdata);

  return eAUD02_ERROR_CODE_SUCCESS;
}

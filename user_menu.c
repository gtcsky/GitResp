/*
 * user_menu.c
 *
 *  Created on: 2019年8月15日
 *      Author: Bough_004
 */
#include "user_advtime.h"
#include "user_basetime.h"
#include "io_define.h"
#include "user_menu.h"
#include "misc.h"
#include "user_key.h"
#include "user_lptimer.h"
#include "user_adc.h"
#include "user_led.h"
#include "user_lcd.h"
#include "user_color.h"
#include "user_lightEffect.h"
#include "user_ddl.h"
#include "user_color.h"
#include "user_spi.h"
#include "communication.h"
#include "user_charge.h"

float 	vSystemVddNormalMode=0;						//非充电模式下的电池电压
float	vBattVolGap=0;									//充电和非充电时的压差
u8		fIsSystemTempGot;
u8 		vTest;
u8 		vtIndex;
u16 		vTimeToSleep;								//vTimeToSleep*50ms  timeout , entry sleep mode
u8 		fIsCharging;
u8 		fIsBattFully;
float	vBattVol=0;
float	vAdVoltOfBatt=0;
float	vTestCompBatt=0;
float	vCompensationVolt=0;
float	vChargingCurrent=0;
//u8		vSkipDCUnpluggedCnt;
u8		vCurrentBattLv=0;
u8 		fIsNeedUpdateBattLv=FALSE;
u8 		vDisplayingBattLv=0;
//u8		fIsSystemFirstPowerON=FALSE;
//u8		vPowerOnCnt;
//u8		fIsChargingCntStart=0;
//u16		vL6ChargeTimer=0;
u8		vLightEfftectVoltDectCnt=0;
float 	vTemperatureCoe=1.0;							//由温度引起的亮度变化系数			>60时 vTemperatureCoe=0.8  <50度时恢复1.0
u8		fIsHotNow=0;
u8		fIsHotDisplayOn=0;							//hot 正在显示
u8		fIsBattAdcValueGot=0;							//已经成功获取到电池 的ADC值
u8 		fIsSystemInitialing;
u8		fIsChargeDisplaying=0;
u8 		fIsSystemOff=0;								//	0/1  系统开启/关闭
//u8 		fIsLightActive;									//	初始画面 灯光是否开启
u8		fPowerOffByBattLow=0;
UserPwmInitTypeDef  	userPwmParaStruct;
UserADCTypeDef		userAdcStruct;
u8		fNeedFirstGetChargeModeVolt=0;				//需要在充电后首次获取电池电压
displayParamsStruct 	displayParams;
displayParamsStruct  	 *pDisplayParams=&displayParams;

PcaDataStruct		pcaDataStruct;
PcaDataStruct		*pPcaDataStruct=&pcaDataStruct;

PcaDataStruct		targetPwmDataStruct;			//调节目标值
PcaDataStruct		oriPwmDataStruct;				//起调时,原始值

colorStructType  userColor;
colorStructType  *pUserColor=&userColor;

u8		fIsAdjustSlowMode=FALSE;		//调节色温时,采用缓慢变化模式,以降低调节时的闪烁现象
u8		vAdjustSlowIndex=0;
float	vAdjustSlowCwStep=0;
float 	vAdjustSlowMwStep=0;
u8		fIsLightEffectOn=FALSE;
//float vSystemTemperature=0;
u8	 	vSystemTemperature=0;
#define LED_PERIOD_CONST 40			//40*50=2s
u8 		fIsCharging=0;
u8		fTimerToPowerOnLED=0;
u8		vPowerOnLEDCnt=0;
u8		vChargePercent=0;
u8 		vDisableGetBattTimer=0;					//LED关闭一段时间内,不进行DA检测,防止电压还没有恢复
u8 		vLCDStatus=0;
u8		vTimeToTurnOffLCD=0;					//关机状态,退出充电模式后,延时一段时间关闭LCD,防止初IIC_Int再次唤醒
u8		fIsUpdateRemainTimer=0;
u16		vRemainTimer=0;
u16		vRemainCheckTimes=0;
u8		vCwMaxTimer=0;
u8		battIndex=0;
float 	battArray[BATT_ARRAY_SIZE]={0};
u8		fIsUpdateChargeingVolt=FALSE;
//u8	fTimeToI2cComm=FALSE;
u8		fIsPowerOnFirst10s=FALSE;
u8		fIsNewCMDGod=FALSE;
float 	vVap_Volt=SYSTEM_VCAP;
u8		fIsDriverVersionGot=FALSE;
factoryModeTypeDef	factoryParams={
		.factoryIndex=VersionDisplay,
		.factoryTimerCnt=0,
		.keyPresssedIndex=0,
		.factoryBrightIndex=50,
		.factoryIncIndex=50,
		.brightnessIncCnt=0,
		.brightnessDecCnt=0,
		.funcIncCnt=0,
		.funcDecCnt=0,
};

userSysTimesDef	sysTimes={0};

void powerOnIoInit(void) {
	GPIO_InitTypeDef GPIO_InitStruct;
	/*--------------------------function key配置-----------------------------*/
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;						//input 模式
	GPIO_InitStruct.GPIO_Speed = GPIO_Low_Speed;
//	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;					//GPIOx_OTYPER   0/1 推挽输出/开漏输出
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;						//上拉
	GPIO_InitStruct.GPIO_Pin = KeyPower_Pin;						//
	GPIO_Init(KeyPower_Port, &GPIO_InitStruct);						//位置  GPIOD.4 GPIOD.5
	GPIO_InitStruct.GPIO_Pin = KeySwitch_Pin;						//
	GPIO_Init(KeySwitch_Port, &GPIO_InitStruct);						//位置  GPIOD.4 GPIOD.5

	GPIO_InitStruct.GPIO_Pin = KeyInc_Pin | KeyDec_Pin;				//位置 . 3.4
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;						//上拉
	GPIO_Init(KeyInc_Port, &GPIO_InitStruct);							//GIOPC

	GPIO_InitStruct.GPIO_Pin = MotorFGPin;							//位置 . 3.4
	GPIO_Init(MotorFGPort, &GPIO_InitStruct);						//GIOPC
//	GPIO_InitStruct.GPIO_Pin = KeyBrightnessInc_Pin;					//位置 . 2
//	GPIO_Init(KeyBrightnessInc_Port, &GPIO_InitStruct);				//GIOPD
//	GPIO_InitStruct.GPIO_Pin = KeySwitch_Pin;						//位置 . 4
//	GPIO_Init(KeySwitch_Port, &GPIO_InitStruct);						//GIOPC
//
//
	GPIO_InitStruct.GPIO_Pin = ChargePin ;							//充电IC通信口
	GPIO_Init(ChargePort, &GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = BattFullPin ;							//充电IC通信口
	GPIO_Init(BattFullPort, &GPIO_InitStruct);
	//-----------------------以上为: 上拉输入口 设置------------------------------
//
	//-----------------------以上为: 不上拉输 入口设置------------------------------
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;					//output 模式
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;					//GPIOx_OTYPER   0/1 推挽输出/开漏输出
	GPIO_InitStruct.GPIO_Pin = Lcd_Res_Pin ;							//
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;					//不上拉
	GPIO_Init(Lcd_Res_Port, &GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = Lcd_Dc_Pin ;							//
	GPIO_Init(Lcd_Dc_Port, &GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = MotorPowerPin ;							//
	GPIO_Init(MotorPowerPort, &GPIO_InitStruct);
	set_LcdResPin;
	set_LcdDcPin;
	clr_MotorPowerPin;
//	GPIO_InitStruct.GPIO_Pin = DriverReset_Pin ;						//
//	GPIO_Init(DriverReset_Port, &GPIO_InitStruct);
//	set_DriverResetPin;
	//-----------------------以上为: 输出口 设置------------------------------
//	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
//	GPIO_InitStruct.GPIO_Pin = EEPROM_IIC_SCK_PIN|EEPROM_IIC_SDA_PIN;
//	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_Init(EEPROM_IIC_SCK_PORT,&GPIO_InitStruct);
	//-----------------------以上为: I2C口设置 设置------------------------------
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;						//output 模式
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;						//GPIOx_OTYPER   0/1 推挽输出/开漏输出
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;						//不上拉
	GPIO_InitStruct.GPIO_Pin = MotorDirPin;								//
	GPIO_Init(MotorDirPort, &GPIO_InitStruct);
	set_MotorDirPin;
	GPIO_InitStruct.GPIO_Pin = Motro_Ctrl_Pin;							//
	GPIO_Init(Motro_Ctrl_Port, &GPIO_InitStruct);
	CLR_Motro_Ctrl_Pin;
	//-----------------------以上为: 输出口 设置------------------------------
//	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
//	GPIO_InitStruct.GPIO_Pin = EEPROM_IIC_SCK_PIN|EEPROM_IIC_SDA_PIN;
//	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_Init(EEPROM_IIC_SCK_PORT,&GPIO_InitStruct);

	//-----------------------以上为: I2C口设置 设置------------------------------
	//-----------------------???: I2C??? ??------------------------------	
	GPIOA->AFR&=0xFFFF000F;
	GPIOB->AFR&=0xFF00FFFF;
	GPIOB->AFR|=0xFF00FFFF;
	GPIOC->AFR&=0xF0000FFF;
	GPIOC->AFR|=0x00000000;
	GPIOD->AFR&=0xF000000F;
	GPIOD->AFR|=  0x00000000;


}

void resetEntrySleepTime(void){
		if(fIsSystemOff)
			vTimeToSleep=POWER_DOWN_MODE_TIMEOUT;							//(5+1)*50ms=300ms
		else{
			vTimeToSleep=NORMAL_MODE_TIMEOUT;								//60s
		}
			//vTimeToSleep=60;								//60s
}
void entryNormalModeChk(void){
		LPTIMER_Cmd(DEBUG_LPTIMERx, ENABLE);			//醒来后重启LPT定时器
		resetEntrySleepTime();								//重置进入睡眠时间
}
//float userAbs(float f1,float f2){
//	if(f2>=f1)
//		return f2-f1;
//	else
//		return	f1-f2;
//}
//void TestColorTemp(void){
//	if(displayParams.colorTemperature++>=MAX_ColorTemp)
//		displayParams.colorTemperature=MIN_ColorTemp;
//	updateColorTemp(&displayParams);
//}
/***************************************************************
 *
 * 清除关机充电状态,拔出从充电头后,延时熄灭防止闪烁的计时状态
 *
***************************************************************/
void clrPowerTimerOutStts(void){
	vTimeToTurnOffLCD=0;
	vLCDStatus=LCDSleeping;
}
/**********************************************************************************
 *
 * 系统关机
 *
 ***********************************************************************************/
void systemPowerDown(void){
//	Sysctrl_SetRCHTrim(SysctrlRchFreq24MHz);
	fIsSystemOff=1;
	lcdEnterSleep();
	disableSpi();
//	LEDPowerOff ;
	turnOffAllLightEffect();
//	clrPowerKeyPressed;
//	vPowerKeyTimer=0;
//	vIsPowerKeyGod&=~(POWER_KEY_SHOT_PRRESS_GOD|POWER_KEY_LONG_PRRESS_GOD);
	clearPowerKeyInfo();
	vTimeToTurnOffLCD=0;
	fIsCharging=0;
	fIsBattFully=0;
	fIsChargeDisplaying=0;
	vLCDStatus = LCDSleeping;
	Enter_DeepSleep();
}

/***********************************************************************************************************
  *  @brief  			function will be  executed per 25ms
  *
  *  @param [in] :
  *
  *  @param [out] :
  *
  *  @return :
  *
  *  @note :
  ************************************************************************************************************/
void fucnPer25ms(void) {
	if (!fIsSystemOff) {
		holdingKeyFunc();
	}
}

/***********************************************************************************************************
  *  @brief  			function will be  executed per 50ms
  *
  *  @param [in] :
  *
  *  @param [out] :
  *
  *  @return :
  *
  *  @note :
  ************************************************************************************************************/
void fucnPer50ms(void) {

	brakeBattDetProcess();
	if(!GET_POWER_KEY_PIN_STTS||fIsCharging){						//关机模式按键和充放电阻止进入睡眠
		resetEntrySleepTime();
	}
	if(!fIsSystemOff){
		if(ON == displayParams.fIsMotorOn)							//马达开启时,阻止休眠
			resetEntrySleepTime();
//		if(displayParams.DisplayModeIndex>=ModeTDisplay)			//非初始画面都不休眠
//			resetEntrySleepTime();
	}
	if (vTimeToSleep&&!--vTimeToSleep){
		if(fIsSystemOff){
			if(vTimeToTurnOffLCD){
				clrPowerTimerOutStts();
				lcdEnterSleep();
			}
			clearPowerKeyInfo();
			Enter_DeepSleep();
		}else{
//			systemPowerDown();
			systemOffByManual();
		}
	}
//	if(fIsSystemOff){
//		if(fIsWakeupByCharge==TimeToPowerOff){
//			if(vTimeToTurnOffLCD&&!--vTimeToTurnOffLCD){
//				fIsWakeupByCharge=DoNothing;
//				fIsChargeDisplaying=FALSE;
//				lcdEnterSleep();
//			}
//		}
//	}

}
/***********************************************************************************************************
  *  @brief  			function will be  executed per 100ms
  *
  *  @param [in] :
  *
  *  @param [out] :
  *
  *  @return :
  *
  *  @note :
  ************************************************************************************************************/
void fucnPer100ms(void) {
}

/***********************************************************************************************************
  *  @brief  			function will be  executed per 500ms
  *
  *  @param [in] :
  *
  *  @param [out] :
  *
  *  @return :
  *
  *  @note :
  ************************************************************************************************************/
void fucnPer500ms(void) {
	if (fIsCharging) {
		if (fIsBattFully == 1) {
			if(!fIsHotNow)
				batterDisplay(Max_Batt_level);
		} else {
			if (vDisplayingBattLv > Max_Batt_level) {
				if (vCurrentBattLv > 1)
					vDisplayingBattLv = vCurrentBattLv - 1;
				else
					vDisplayingBattLv = 0;
			}
			if(!fIsHotNow)
				batterDisplay(vDisplayingBattLv++);
			if(vDisplayingBattLv>=2){
				if(fNeedFirstGetChargeModeVolt){						//充电首次电池图标跑2格时,检测电池端电压,用于记录与常规状态的压差
					fNeedFirstGetChargeModeVolt=0;
					float tempVolt = 0;
					tempVolt = calcBattVolt();
					vBattVol= (tempVolt>0)?tempVolt:vBattVol;
					getChargingCurrentByVolt(&vChargingCurrent,vBattVol);
					vBattVolGap=vChargingCurrent*BATTERY_RESISTANCE;
					setBattLevel(vBattVol-vBattVolGap);					//用于解决快速拔出充电器后,有时会显示在vCurrentBattLv=0的问题
				}
			}
		}
	}
	if (TRUE == fIsNeedUpdateBattLv) {
		fIsNeedUpdateBattLv = FALSE;
		batterDisplay(vCurrentBattLv);
	}
	if(!fIsSystemOff&&displayParams.fIsMotorOn){
		displayFanCircie(&displayParams);
	}
//	i2CTask();
}

/**************************************************************
 *
 *从高温模式值 获取常温模式PWM的值
 *
 ***************************************************************/
u16 getOriginalPWMData(u16 curData){
	u16	vtTemp16=0;
	vtTemp16=curData*TEMPERATURE_COE_NORMAL/TEMPERATURE_COE_LOW;
	return	((vtTemp16>PWM_FRQ_CONST)?PWM_FRQ_CONST:vtTemp16);
}

void checkTemperature(void) {
	if (sysTimes.vSystem1s % 2 == 0) {
//		if (!vIsKeyPressed  && fIsLightActive) {
//			vSystemTemperature = 0;
//			readTemperatureData();
//		}
	}
}

void hotFuncDeinit(void){

	vTemperatureCoe=TEMPERATURE_COE_NORMAL;				//由温度引起的亮度变化系数			>60时 vTemperatureCoe=0.8  <50度时恢复1.0
	fIsHotNow=FALSE;
	vSystemTemperature=0;
	if(!fIsSystemOff)
		batterDisplay(vCurrentBattLv);

}

void checkSystemHot(void) {
//	u8 vtBrightnessTemp = 0;
//	if (fIsLightActive == ON && vSystemTemperature) {
//		if (vSystemTemperature >= OVER_TEMPERATURE_VOLT_Lv1) {	//温度大于60度关机
//			fIsHotNow = FALSE;
//			vTemperatureCoe = TEMPERATURE_COE_NORMAL;
//			vKeyValue = POWER_KEY_LONG_PRESSED;
//		} else if (vSystemTemperature >= OVER_TEMPERATURE_VOLT_Lv0 && !fIsHotNow) {
//			fIsHotNow = TRUE;
//			vtBrightnessTemp=displayParams.brightness;
//			displayParams.brightness*=TEMPERATURE_COE_LOW;
//			if(!displayParams.brightness)
//				displayParams.brightness=0.01;
//			setBrightnessOnlyData(BRIGHTNESS_ONLY_LENGTH, &displayParams);
//			displayParams.brightness=vtBrightnessTemp;
//		} else if (vSystemTemperature <= NORMAL_TEMPERATURE_VOLT && fIsHotNow) {
//			hotFuncDeinit();
//			setBrightnessOnlyData(BRIGHTNESS_ONLY_LENGTH, &displayParams);
//		}
//	}
}
/***********************************************************************************************************
  *  @brief  			function will be  executed per 1000ms
  *
  *  @param [in] :
  *
  *  @param [out] :
  *
  *  @return :
  *
  *  @note :
  ************************************************************************************************************/
void fucnPer1s(void) {
	startBattVoltDect();
//	checkTemperature();
	updateHotDisplay();
	if(fIsCharging){
		fIsUpdateChargeingVolt=TRUE;
	}
}

/***********************************************************************************************************
  *  @brief			使用等待的方式获取电池的DA值
  *
  *  @param [in] :
  *
  *  @param [out] :
  *
  *  @return :
  *
  *  @note :
  ************************************************************************************************************/
float readBattInfoImmediately(void) {
	float tempVolt = 0;
	startBattVoltDect();
	do {
		tempVolt = calcBattVolt();
		if (tempVolt != 0)
			break;
	} while (sysTimes.vSystem5ms < 20);
	sysTimes.vSystem5ms = 0;
	if (tempVolt < 0) {
//		fIsDcMode = 1;
		fPowerOffByBattLow = 0;
	}
	return tempVolt;
}

/***********************************************************************************************************
  *  @brief  					计算电池电压
  *
  *  @param [in] :
  *
  *  @param [out] :			0: calc volt error
  *						-1:DC mode
  *  @return :
  *
  *  @note :
  ************************************************************************************************************/
float calcBattVolt(void) {
	if (adcConvertParams.vChlResultGot & ADC_BATT_CHL) {
		adcConvertParams.vChlResultGot &= ~ADC_BATT_CHL;
		float volt = 0;
		volt = ((adcConvertParams.aDaValues[ADC_BATT_No] * SYSTEM_VDD) / ADC_CONST);
		volt = volt * (BATT_DET_PULL_UP_RES + BATT_DET_PULL_DOWN_RES) / BATT_DET_PULL_DOWN_RES;
//		if (volt == 0)
		if (volt <= 0.1)
			return -1;
		else
			return volt/4;				//,换算单节
	} else
		return 0;
}

/***********************************************************************************************************
  *  @brief			电池电量相关信息处理
  *
  *  @param [in] :
  *
  *  @param [out] :
  *
  *  @return :
  *
  *  @note :
  ************************************************************************************************************/
void battInfoProcess(void) {
	u8 temp;
	float tempVolt = 0;
	if (!fIsCharging) {
		tempVolt = calcBattVolt();
		if (tempVolt < 0) {
//			fIsDcMode = 1;
			fPowerOffByBattLow = 0;
		} else if (tempVolt == 0) {
			return;
		} else {
			vAdVoltOfBatt = tempVolt;
//			fIsDcMode = 0;
		}
		//-------------------------------------------
		fIsBattAdcValueGot = 1;
		chkLightEffectModeVoltCompensation(&vAdVoltOfBatt);
		battArray[battIndex] = vAdVoltOfBatt;
		if (++battIndex >= BATT_ARRAY_SIZE)
			battIndex = 0;
		//-------------------------------------------
		if (fIsBattAdcValueGot) {
			vBattVol = vAdVoltOfBatt;
			vAdVoltOfBatt = vBattVol;
			vTestCompBatt = vBattVol;
			vCompensationVolt = vBattVol;
			vSystemVddNormalMode = vBattVol;
			if (vCurrentBattLv > Max_Batt_level)
				vCurrentBattLv = Max_Batt_level;
			temp = vCurrentBattLv;
			setBattLevel(vBattVol);
			if (vCurrentBattLv != temp) {
				fIsNeedUpdateBattLv = TRUE;
			}
		}
	}
}

void resetBrakeBattDet(void) {

	vDisableGetBattTimer = DISABLE_BATT_ADC_TIMER_CONST;

}
void brakeBattDetProcess(void) {
	if (vDisableGetBattTimer)
		vDisableGetBattTimer--;
}

void startCharging(void) {
	fIsCharging = 1;
	fIsChargeDisplaying = 0;
	fIsBattFully = 0;
//	   vBattFullTimes=0;
	fNeedFirstGetChargeModeVolt = 1;
	vChargingCurrent = MAX_CHARGE_CURRENT;						//初始设高,防止插上充电器一瞬间补偿后电压偏高的
//	   fIsDischarging=FALSE;
	vDisplayingBattLv = 0;											      //插上充电器时,第一次从第0格开始显示充电符号
//	   vNearBattFullCnt=0;
	if (fIsSystemOff) {
		enableSpi();
	}
}

void stopCharging(void) {
//	fIsVapCalibrated=FALSE;
	fIsCharging = 0;
	fIsBattFully = 0;
//	vBattFullTimes=0;
//	   vNearBattFullCnt=0;
//	else if(fIsChargeDisplaying){									//fIsCharging==0&&fIsChargeDisplaying==1
	if (fNeedFirstGetChargeModeVolt) {								//插入充电器,在还未完成首个充电图标显示循环时拔出充电器
		fNeedFirstGetChargeModeVolt = 0;
		vBattVol = vSystemVddNormalMode;
		vCurrentBattLv = Max_Batt_level;							//用于setBattLevel在非充电模式显示回真正的档位
		setBattLevel(vBattVol);
	}
//	deinitMaxLevelChargeData();
	fIsChargeDisplaying = 1;
	updateChargingIcon();
	fIsNeedUpdateBattLv = TRUE;									//关掉充电显示
	fIsChargeDisplaying = FALSE;
	powerDownFromChageMode();
//		}
}

/************************************************************************
 *
 * 处理充电显示相关内容
 *
 *************************************************************************/
void funcCharging(void) {
//	if(!Get_ChargePin_Stts&&!fIsCharging){
//		startCharging();
//	}
//	if(Get_ChargePin_Stts&&fIsCharging){
//		stopCharging();
//	}
	if(fIsCharging){
		if(Get_ChargePin_Stts&&Get_BattFully_Stts){
			stopCharging();
		}
	}else{
		if(!Get_ChargePin_Stts||!Get_BattFully_Stts){
			startCharging();
		}
	}

	getChargeStts(&fIsBattFully);
	if (fIsCharging) {
		if(!fNeedFirstGetChargeModeVolt&&fIsUpdateChargeingVolt){
			float tempVolt = 0;
			tempVolt = calcBattVolt();
			vBattVol= (tempVolt>0)?tempVolt:vBattVol;
			getChargingCurrentByVolt(&vChargingCurrent,vBattVol);
			vAdVoltOfBatt=vBattVol;
			if(fIsBattFully){
				if(fIsSystemOff){
					batteryPercentDisplay(100);
//					if(!fIsVapCalibrated){
//						fIsVapCalibrated=TRUE;
//						calibrateVcap();
//					}
				}
			}
		}
		if ((!fIsChargeDisplaying) ||(fIsSystemOff&&(LCDSleeping==vLCDStatus))){
//		if (!fIsChargeDisplaying) {
			fIsCharging = 1;
			fIsBattFully = 0;
			updateChargingIcon();
			if (!fIsSystemInitialing && !vDisableGetBattTimer) {
				vCurrentBattLv = 0;
			}
			clrPowerTimerOutStts();
			onChargingIconInPowerOffMode();
		}
		if(!fNeedFirstGetChargeModeVolt&&fIsUpdateChargeingVolt){
			fIsUpdateChargeingVolt=0;
			vBattVolGap=vChargingCurrent*BATTERY_RESISTANCE;
			vTestCompBatt=vBattVol-vBattVolGap;
			vCompensationVolt=vTestCompBatt;
			setBattLevel(vCompensationVolt);
//			if((Max_Batt_level!=Max_Batt_level)&&(vtTempBatt>=CHARGE_FINAL_VOLT)){		//恒压模式时,强制为电池6档
//				vCurrentBattLv=Max_Batt_level;
//			}
		}
	}
}
void fucnPer5ms(void) {
//		chargeSttsCheck();
	battInfoProcess();
	funcCharging();
	if (fPowerOffByBattLow) {
//		fPowerOffByBattLow = 0;
//		if (!fIsSystemOff) {
//			if (GET_POWER_KEY_PIN_STTS) {
//				systemOffByManual();
//			}
//		}
	}
	checkAdcConvertRetry();

	checkSystemHot();
	if (sysTimes.vSystemLIRC5ms % 2 == 0) {
		factoryModeCoolDownFunc();
	}
	if (sysTimes.vSystemLIRC5ms % 5 == 0) {
		fucnPer25ms();
	}
	if (sysTimes.fTime50ms == 1) {
		sysTimes.fTime50ms = 0;
		fucnPer50ms();
	}
	if (sysTimes.fTime100ms == 1) {
		sysTimes.fTime100ms = 0;
		fucnPer100ms();
	}
	if (sysTimes.fTime500ms == 1) {
		sysTimes.fTime500ms = 0;
		fucnPer500ms();
	}
	if (sysTimes.fTime1s == 1) {
		sysTimes.fTime1s = 0;
		sysTimes.vSystem1s += 1;
		fucnPer1s();
		if (fIsPowerOnFirst10s && sysTimes.vSystem1s > 10) {
			fIsPowerOnFirst10s = FALSE;
		}
//					if(sysTimes.vSystem1s>=20){
//						while(1);
//							RCC->REGLOCK = 0x55AA6699;
//							RCC->IOMUX =   0x5A690001;			//set swdio
//							RCC->REGLOCK = 0x55AA6698;
//					}
//		if (Get_IIC_Int_Stts) {
//			if (sysTimes.vSystem1s > 6) {
//				if (sysTimes.vSystem1s % 5 == 0) {
//					if (sysTimes.vSystem1s % 2 == 0)
//						writeByte2ChargeIc( battVolt4v35);
//					else
//						writeByte2ChargeIc( battVolt4v20);
//				}
//			}
//		}
	}
}

void factoryModeCoolDownFunc(void) {
//	if (FactoryMode == displayParams.DisplayModeIndex) {
//		if (factoryParams.factoryTimerCnt && --factoryParams.factoryTimerCnt == 0) {
//			switch (factoryParams.factoryIndex) {
//			case WaitFuncKey:
//				break;
//			case RedLedTesting:
////				displayParams.brightness=20;
//				factoryParams.factoryIndex++;
//				factoryParams.factoryTimerCnt = FACTORY_TIME_2S;
//				displayParams.hues = 120;
//				updateLEDStts(&displayParams);
//				displayFactoryMode(Green_Current);
//				break;
//			case GreenLedTesting:
//				factoryParams.factoryIndex++;
//				factoryParams.factoryTimerCnt = FACTORY_TIME_2S;
//				displayParams.hues = 240;
//				updateLEDStts(&displayParams);
//				displayFactoryMode(Blue_Current);
//				break;
//			case BlueLedTesting:
//				factoryParams.factoryIndex++;
//				factoryParams.factoryTimerCnt = FACTORY_TIME_2S;
//				displayFactoryMode(Cw_Current);
//				factoryModeTemperatureDisplay();
//				displayParams.colorTemperature = MAX_ColorTemp;
//				displayParams.arrowIndex = ColorTempSetting;
//				updateLEDStts(&displayParams);
//				break;
//			case CoolLedTesting:
//				factoryParams.factoryIndex++;
//				factoryParams.factoryTimerCnt = FACTORY_TIME_2S;
//				displayParams.colorTemperature = MIN_ColorTemp;
//				updateLEDStts(&displayParams);
//				displayFactoryMode(Mw_Current);
//				factoryModeTemperatureDisplay();
//				factoryParams.factoryTemp = vSystemTemperature;
//				break;
//			case WarmLedTesting:
//				factoryParams.factoryIndex++;
//				factoryParams.factoryTimerCnt = 0;
//				displayFactoryMode(Press_Bdec_key);
//				break;
//			case WaitBrightnesssDecKey:
//				break;
//			case IdleModeTesting:
//				factoryParams.factoryIndex++;
//				factoryParams.factoryTimerCnt = 2;
////				displayFactoryMode(System_Volt_Display);
//				break;
//			case SystemVoltTesting:
//				factoryParams.factoryIndex++;
//				factoryParams.factoryTimerCnt = 2;
////				displayFactoryMode(Press_Inc_key);
//				break;
//			case WaitIncKey:
//				factoryParams.factoryTimerCnt = 30;
//				break;
//			case SleepModeTesting:
//				break;
//			case VersionDisplay:
////				vTestCompBatt = getBattVolt();
//				factoryVoltDisplay(vTestCompBatt);
//				if (vTestCompBatt <= 4.00 || vTestCompBatt > 4.30) {
//					displayFactoryMode(VoltageErrorDisplay);
//					factoryVoltDisplay(vTestCompBatt);
//					while (1)
//						;
//				}
//			default:
//				factoryParams.factoryIndex++;
//				displayFactoryMode(Press_Func_key);
//				factoryParams.factoryTimerCnt = 0;
//				fIsCharging = FALSE;
//				break;
//			}
//		} else {
//			switch (factoryParams.factoryIndex) {
//			case VersionDisplay:
//				if (fIsDriverVersionGot) {
//					OLED_ShowString(8, 6, "Dr:");
////					OLED_ShowString(32, 6, Driver_Version);
//					fIsDriverVersionGot = FALSE;
//				}
//				break;
//			case WaitFuncKey:
//				if (!GET_POWER_KEY_PIN_STTS &&
//				GET_DEC_KEY_PIN_STTS &&
//				GET_INC_KEY_PIN_STTS &&
//				Get_Switch_Key_Pin_Status && (factoryParams.keyPresssedIndex == 0x0F)) {
//					factoryParams.factoryIndex++;
//					factoryParams.factoryTimerCnt = FACTORY_TIME_2S;
//					displayFactoryMode(Red_Current);
//					displayParams.saturation = 100;
//					displayParams.brightness = 100;
//					displayParams.hues = 0;
//					displayParams.arrowIndex = HuesSetting;
//					factoryParams.brightnessIncCnt = 0;
//					factoryParams.brightnessDecCnt = 0;
//					factoryParams.funcIncCnt = 0;
//					factoryParams.funcDecCnt = 0;
//					factoryParams.keyPresssedIndex = 0;
//					updateLEDStts(&displayParams);
//					vIsKeyPressed = 0;
//					vKeyValue = 0;
//				}
//				break;
//			case BlueLedTesting:
//				if (FACTORY_TIME_2S9 == factoryParams.factoryTimerCnt) {
////					readTemperatureData();
//				}
//				if (FACTORY_TIME_2S == factoryParams.factoryTimerCnt) {
//					factoryModeTemperatureDisplay();
//				}
//				break;
//			case WaitBrightnesssDecKey:
//				factoryParams.factoryIndex++;
//				factoryParams.factoryTimerCnt = FACTORY_TIME_2S;
//				displayFactoryMode(Standby_Current);
//				//------------------
//				//if(!factoryParams.factoryTimerCnt){
////				resetDriverMcu();
//				//}
//				//------------------
//				vIsKeyPressed = 0;
//				vKeyValue = 0;
//				break;
//			case SystemVoltTesting:
////						if(!factoryParams.factoryTimerCnt){
////						resetDriverMcu();
////						vTestCompBatt=getBattVolt();
////						OLED_ShowChar(40, 0, ((u8) vTestCompBatt) + '0');
////						OLED_ShowChar(40 + 8, 0, '.');
////						OLED_ShowChar(40 + 13, 0, ((u8) (vTestCompBatt * 10)) % 10 + '0');
////						OLED_ShowChar(40 + 21, 0, ((u16) (vTestCompBatt * 100)) % 10 + '0');
////						OLED_ShowChar(40 + 29, 0,  'V');
////						if(!factoryParams.factoryTimerCnt)
////							factoryParams.factoryTimerCnt=FACTORY_TIME_2S;
////						vIsKeyPressed=0;
////						vKeyValue=0;
////						}
//				break;
//			case WaitIncKey:
//				//if (factoryParams.factoryTemp < 19 || factoryParams.factoryTemp > 39) {
//				if (factoryParams.factoryTemp < 19 || factoryParams.factoryTemp > 39) {
//					clear();
//					vSystemTemperature = factoryParams.factoryTemp;
//					displayFactoryMode(TempErrorDisplay);
//					factoryModeTemperatureDisplay();
//					//OLED_ShowString(20,4,"Temp  Error");
//					while (1)
//						;
//				}
//				sysTimes.vSystem5ms = 0;
//				displayParams.DisplayModeIndex = IdleIamgeDisplay;
//				factoryParams.factoryTimerCnt = 0;
//				factoryParams.factoryBrightIndex = 0;
//				factoryParams.factoryIncIndex = 0;
//				factoryParams.factoryIndex = VersionDisplay;
//				factoryParams.keyPresssedIndex = 0;
////						sysTimes.vSystem5ms=1800;
////						while(1);
//				vIsKeyPressed = 0;
//				vKeyValue = 0;
//				powerDataInit();
//				systemOffByManual();
//				break;
//			}

//			if (vKeyValue) {
////				if(factoryParams.factoryIndex==WaitFuncKey){
////					if(vKeyValue==BRIGHTNESS_DOWN_KEY_SHORT_PRESSED){
////						factoryParams.keyPresssedIndex|=0x01;
////					}
////					if(vKeyValue==INC_KEY_SHORT_PRESSED){
////						factoryParams.keyPresssedIndex|=0x02;
////					}
////				}

//				if ((factoryParams.factoryIndex >= WaitFuncKey) && (factoryParams.factoryIndex <= WarmLedTesting)) {
//					//OLED_ShowNum
//					if (INC_KEY_SHORT_PRESSED == vKeyValue) {
//						if (++factoryParams.factoryIncIndex > 99)
//							factoryParams.factoryIncIndex = 99;
//						if (factoryParams.factoryIncIndex < 10)
//							factoryParams.factoryIncIndex = 10;
//						OLED_ShowNum(8, 2, factoryParams.factoryIncIndex);
//						if (factoryParams.funcIncCnt < FACTORY_KEY_COST)
//							factoryParams.funcIncCnt++;
//						else
//							factoryParams.keyPresssedIndex |= 0x01;
//					}
//					if (DEC_KEY_SHORT_PRESSED == vKeyValue) {
//						if (factoryParams.factoryIncIndex-- <= 10)
//							factoryParams.factoryIncIndex = 10;
//						OLED_ShowNum(8, 2, factoryParams.factoryIncIndex);
//						if (factoryParams.funcDecCnt < FACTORY_KEY_COST)
//							factoryParams.funcDecCnt++;
//						else
//							factoryParams.keyPresssedIndex |= 0x02;
//					}
////					if(SWITCH_KEY_SHORT_PRESSED==vKeyValue){
////							if(++factoryParams.factoryBrightIndex>99)
////								factoryParams.factoryBrightIndex=99;
////							if(factoryParams.factoryBrightIndex<10)
////								factoryParams.factoryBrightIndex=10;
////							OLED_ShowNum(104,2,factoryParams.factoryBrightIndex);
////							if(factoryParams.brightnessIncCnt<FACTORY_KEY_COST)
////								factoryParams.brightnessIncCnt++;
////							else
////								factoryParams.keyPresssedIndex|=0x04;
////					}
//					if (SWITCH_KEY_SHORT_PRESSED == vKeyValue) {
//						if (factoryParams.factoryBrightIndex-- <= 10)
//							factoryParams.factoryBrightIndex = 10;
//						OLED_ShowNum(104, 2, factoryParams.factoryBrightIndex);
//						if (factoryParams.brightnessDecCnt < FACTORY_KEY_COST)
//							factoryParams.brightnessDecCnt++;
//						else
////							factoryParams.keyPresssedIndex |= 0x08;
//							factoryParams.keyPresssedIndex |= 0x0C;
//					}
//					vKeyValue = 0;
//				}
//			}
//		}
//	}
}

/***********************************************************************************************************
  *  @brief			亮度+
  *
  *
  *  @param [in] :
  *
  *  @param [out] :
  *
  *  @return :
  *
  *  @note :
  ************************************************************************************************************/
//void brightnessIncFunc(void){
//	if (displayParams.brightness < 100) {
//		displayParams.brightness++;
//		updateBrightnessDisplay(&displayParams);
//		if (ColorTempSetting >= displayParams.arrowIndex) {
//			updateLEDStts(&displayParams);
//		} else {
//			setBrightnessOnlyData(BRIGHTNESS_ONLY_LENGTH, &displayParams);
//		}
//	}
//}
/***********************************************************************************************************
  *  @brief			亮度-
  *
  *
  *  @param [in] :
  *
  *  @param [out] :
  *
  *  @return :
  *
  *  @note :
  ************************************************************************************************************/
//void brightnessDecFunc(void) {
//	if (displayParams.brightness>1 ) {
//		displayParams.brightness--;
//		updateBrightnessDisplay(&displayParams);
//		if (ColorTempSetting >= displayParams.arrowIndex) {
//			updateLEDStts(&displayParams);
//		} else {
//			setBrightnessOnlyData(BRIGHTNESS_ONLY_LENGTH, &displayParams);
//		}
//	}
//}
/***********************************************************************************************************
  *  @brief			色温+
  *
  *
  *  @param [in] :
  *
  *  @param [out] :
  *
  *  @return :
  *
  *  @note :
 ************************************************************************************************************/
//void cctIncFunc(void) {
//	if (displayParams.colorTemperature < MAX_ColorTemp) {
//		displayParams.colorTemperature++;
//		updateColorTempDisplay(&displayParams);
//		fIsAdjustSlowMode = TRUE;
//		oriPwmDataStruct.valueOfMw = pcaDataStruct.valueOfMw;
//		oriPwmDataStruct.valueOfCw = pcaDataStruct.valueOfCw;
//	}
//}



/***********************************************************************************************************
  *  @brief
  *
  *  @param [in] :			*param 待调节参数
  *  						step 调节步进
  *  						min 最小值
  *  @param [out] :
  *
  *  @return :
  *
  *  @note :
  ************************************************************************************************************/
void	 publicDecFunc(uint8 * param,uint8 step,uint8 min){
	if ((*param >= min+step) && (*param != 1)) {
		*param -= step;
		if (*param<=min)
			*param = min;
	} else if (*param <=min+step) {
		*param = min;
	}
}


/***********************************************************************************************************
  *  @brief
  *
  *  @param [in] :			*param 待调节参数
  *  						step 调节步进
  *  						max:上限
  *  						min :下限
  *  @param [out] :
  *
  *  @return :
  *
  *  @note :
  ************************************************************************************************************/
void publicIncFunc(uint8 * param, uint8 step, uint8 max, uint8 min) {
	if (*param < (max + 1 - step)) {
		if ((*param <= min + 1) && (step != 1)) {
			*param = min + step;
		} else {
			*param += step;
		}
	} else {
		*param = max;
	}
}


/***********************************************************************************************************
  *  @brief			色温-
  *
  *
  *  @param [in] :
  *
  *  @param [out] :
  *
  *  @return :
  *
  *  @note :
  ************************************************************************************************************/
//void cctDecFunc(void) {
//	if (displayParams.colorTemperature > MIN_ColorTemp) {
//		displayParams.colorTemperature--;
//		updateColorTempDisplay(&displayParams);
//		fIsAdjustSlowMode = TRUE;
//		oriPwmDataStruct.valueOfMw = pcaDataStruct.valueOfMw;
//		oriPwmDataStruct.valueOfCw = pcaDataStruct.valueOfCw;
//	}
//}
/***********************************************************************************************************
  *  @brief				user interaction
  *
  *  @param [in] :
  *
  *  @param [out] :
  *
  *  @return :
  *
  *  @note :
  ************************************************************************************************************/
void MenuFunction(void ){
	if (vKeyValue == POWER_KEY_LONG_PRESSED) {
		fIsSystemOff ^= 0x01;
		if (fIsSystemOff) {			//turn off system
			systemOffByManual();
		} else {					//turn on system
			//lcdExitSleep();
			displayParams.DisplayModeIndex = IdleIamgeDisplay;
			powerOnLcdInit();
			hotFuncDeinit();
//			if(!GET_INC_KEY_PIN_STTS){
//				versionDisplay();
//				factoryParams.factoryIndex=VersionDisplay;
//				factoryParams.factoryTimerCnt=160;
//				factoryParams.factoryBrightIndex=50;
//				factoryParams.factoryIncIndex=50;
//				factoryParams.keyPresssedIndex=0;
//				return;
//			}
			resetEntrySleepTime();
			displaySystemMenu(&displayParams);
		}
	}
	if (!fIsSystemOff) {
		uint8 step=1;
		if (vKeyValue == FUNC_KEY_SHORT_PRESSED) {
			if (IdleIamgeDisplay == displayParams.DisplayModeIndex) {
				if(displayParams.fIsMotorOn){
					displayParams.fIsMotorOn=0;
					clr_MotorPowerPin;
					setMotorCtrlDuty(0);
					displayRpmValue(&displayParams);
					displayParams.fIsObliqueDisplaying=1;	// 用于静态时更新正反转图标回位
					displayFanCircie(&displayParams);
				}else{
					displayParams.fIsMotorOn=1;
					set_MotorPowerPin;
					setMotorCtrlDuty(displayParams.vMotorDuty*0.01);
					displayRpmValue(&displayParams);
				}
			}
		}
		else if (displayParams.fIsMotorOn && (vKeyValue & FAST_ADJUST_FALG_MASK) == INC_KEY_SHORT_PRESSED) {
			if (vKeyValue & FAST_ADJUST_FLAG)
				step = FAST_ADJUST_STEP;
			publicIncFunc(&displayParams.vMotorDuty, step, MAX_BRIGHTNESS, MIN_BRIGHTNESS);
			setMotorCtrlDuty(displayParams.vMotorDuty*0.01);
			displayRpmValue(&displayParams);
		} else if (displayParams.fIsMotorOn && (vKeyValue & FAST_ADJUST_FALG_MASK) == DEC_KEY_SHORT_PRESSED) {
			if (vKeyValue & FAST_ADJUST_FLAG)
				step = FAST_ADJUST_STEP;
			publicDecFunc(&displayParams.vMotorDuty, step, MIN_BRIGHTNESS);
			setMotorCtrlDuty(displayParams.vMotorDuty*0.01);
			displayRpmValue(&displayParams);
		} else if (vKeyValue == SWITCH_KEY_SHORT_PRESSED) {
			if(displayParams.fIsReversed){
				displayParams.fIsReversed=0;
				displayParams.fIsObliqueDisplaying=1;	// 用于静态时更新正反转图标
				displayFanCircie(&displayParams);
				set_MotorDirPin;
			}else{
				displayParams.fIsReversed=1;
				displayParams.fIsObliqueDisplaying=1;	// 用于静态时更新正反转图标
				displayFanCircie(&displayParams);
				clr_MotorDirPin;
			}
		}
		if(vKeyValue)
			resetEntrySleepTime();

	}
	vKeyValue = 0;
}

//void hotFuncDeinit(void){
//	vTemperatureCoe=TEMPERATURE_COE_NORMAL;				//由温度引起的亮度变化系数			>60时 vTemperatureCoe=0.8  <50度时恢复1.0
//	fIsHotNow=FALSE;
//	fIsHotDisplayOn=FALSE;
//	if(!fIsSystemInitialing){
//		batterDisplay(vCurrentBattLv);
//	}
//}

/***********************************************************************************************************
  *  @brief
  *
  *  @param [in] :
  *
  *  @param [out] :
  *
  *  @return :
  *
  *  @note :
  ************************************************************************************************************/
void powerDataInit(void) {
//		adcThresholdGenerate();
		userPwmParaStruct.period = ADVICE_TIM_PERIOD;
		displayParams.battIndex = 6;
		displayParams.fIsMotorOn=DEFAULT_MOTOR_STATUS;
		displayParams.fIsReversed=DEFAULT_REVERSED_STATUS;				//正反转
		displayParams.fIsObliqueDisplaying=DEFAULT_OBLIQUE_DISPLAYING;		//倾斜显示
		displayParams.vMotorDuty=DEFAULT_MOTOR_DUTY;
		displayParams.DisplayModeIndex=DEFAULT_DIAPLAY_MODE;
		vCurrentBattLv = Max_Batt_level;
		fIsCharging = 0;
		fIsBattAdcValueGot = 0;
//		hotFuncDeinit();
		vChargingCurrent = 1.0;
		vVap_Volt = SYSTEM_VCAP;
}


//float getResistanceByDuty(float tDuty){
//	if(tDuty>=0.80)
//		return DISCHARGE_RESISTENCE1D10;
//	else if(tDuty>=0.70)
//		return DISCHARGE_RESISTENCE1D08;
//	else if(tDuty>=0.60)
//		return DISCHARGE_RESISTENCE1D05;
//	else if(tDuty>=0.50)
//		return DISCHARGE_RESISTENCE;
//	else if(tDuty>=0.40)
//		return DISCHARGE_RESISTENCE0D95;
//	else if(tDuty>=0.30)
//		return DISCHARGE_RESISTENCE0D90;
//	else if(tDuty>=0.20)
//		return DISCHARGE_RESISTENCE0D88;
//	else if(tDuty>=0.10)
//		return DISCHARGE_RESISTENCE0D85;
//	else
//		return	DISCHARGE_RESISTENCE0D82;
//}
/***************************************************************************
 *
 *  	电压补偿.灯开启时的电压补偿
 *
 ***************************************************************************/
void battVoltCompensation(float *volt) {
	float vtCurrent=0;
	if (ON == displayParams.fIsMotorOn) {				//在灯开启的情况下
		if (!fIsLightEffectOn) {
//			if(displayParams.arrowIndex<=SaturationSetting){
//				vtCurrent+=pUserColor->red*RED_POWER_RATING/(*volt);
//				vtCurrent+=pUserColor->green*GREEN_POWER_RATING/(*volt);
//				vtCurrent+=pUserColor->blue*BLUE_POWER_RATING/(*volt);
//			}else if(displayParams.arrowIndex<=ColorTempSetting){
//				vtCurrent+=pUserColor->cWhite*CW_POWER_RATING/(*volt);
//				vtCurrent+=pUserColor->wWhite*MW_POWER_RATING/(*volt);
//			}
		}else{
//			if ((Style1Setting == displayParams.arrowIndex)
//					|| ((Style2Setting == displayParams.arrowIndex) && (displayParams.style2Value != 2))
//					|| ((Style3Setting == displayParams.arrowIndex) && (displayParams.style2Value == 2))) {			//有暖光或白光

//				vtCurrent=CW_POWER_RATING*MAX_CW_DUTY/(*volt);
//			}else{
//				vtCurrent=BLUE_POWER_RATING*MAX_MW_DUTY/(*volt);
//			}
		}
	}
	if(fIsHotNow){
		vtCurrent*=TEMPERATURE_COE_LOW;
	}
//	if(fIsDischarging){
//		if(fIsCharging){
//			vtCurrent+=0.35;					//放电模式最大电池设定为1.0
//		} else {
//			if (*volt >= BATT_LV4_THESHOLD)
//				vtCurrent += 1.2;				//放电模式最大电池设定为1.0
//			else if (*volt >= BATT_LV1_THESHOLD)
//				vtCurrent += 0.8;
//			else if (*volt >= BATT_LV0_THESHOLD)
//				vtCurrent += 0.5;
//		}
//	}
	*volt += vtCurrent * DISCHARGE_RESISTENCE;
//	if(fIsDischarging){
//		*volt+=MAX_DISCHARGE_CURRENT*DISCHARGE_RESISTENCE;
//	}
	vTestCompBatt = *volt;
	vCompensationVolt = vTestCompBatt;
}
/***************************************************************************
 *
 *  	ADC电压补偿.在灯开启时的电压补偿
 *
 ***************************************************************************/
void adcVoltCompensation(float *volt) {
	if (fIsCharging)
		return;
	battVoltCompensation(volt);
}

/***********************************************************************************************************
  *  @brief 		show charging in power down mode
  *
  *  @param [in] :
  *
  *  @param [out] :
  *
  *  @return :
  *
  *  @note :
  ************************************************************************************************************/
void onChargingIconInPowerOffMode(void) {
	if (fIsSystemOff) {
		vLCDStatus = LCDWorking;
		powerOnLcdInit();
		hotFuncDeinit();
		displayParams.DisplayModeIndex = ChargingAtPowerDown;
		displaySystemMenu(&displayParams);
		vChargePercent = 0;
//		fIsSystemFirstPowerON = TRUE;			//仅用于下次开机,刷新剩余时间
	}
}

/***********************************************************************************************************
  *  @brief  			entry sleep mode when exit charging mode and system  is off
  *
  *  @param [in] :
  *
  *  @param [out] :
  *
  *  @return :
  *
  *  @note :
  ************************************************************************************************************/
void powerDownFromChageMode(void){
	if(fIsSystemOff){
		vLCDStatus = LCDSleeping;
		lcdEnterSleep();
	}
}
/************************************************************************************
 *
 * RisingEnable	:FALSE     level允许上升
 * 				:TRUE     level不允许上升
 *
 *************************************************************************************/
void  processBattLevel(float vtBattReal,u8 RisingEnable){
//	if (vtBattReal >= BATT_LV6_THESHOLD) {
//		if(!RisingEnable){
//			if (vCurrentBattLv >= Max_Batt_level)
//				vCurrentBattLv = Max_Batt_level;
//		}else{
//				vCurrentBattLv = Max_Batt_level;
//		}
//	} else if (vtBattReal >= BATT_LV5_THESHOLD) {
//		if(!RisingEnable){
//			if (vCurrentBattLv >= 5)//非充电模式, 电池电量不允许往上升
//				vCurrentBattLv = 5;
//		}else
//				vCurrentBattLv = 5;
//	} else
	if (vtBattReal >= BATT_LV4_THESHOLD) {
		if (!RisingEnable) {
			if (vCurrentBattLv >= 4)
				vCurrentBattLv = 4;
		} else {
			vCurrentBattLv = 4;
		}
	} else if (vtBattReal >= BATT_LV3_THESHOLD) {
		if(!RisingEnable){
			if (vCurrentBattLv >= 3)
				vCurrentBattLv = 3;
		}else{
				vCurrentBattLv = 3;
		}
	} else if (vtBattReal >= BATT_LV2_THESHOLD) {
		if(!RisingEnable){
			if (vCurrentBattLv >= 2)
				vCurrentBattLv = 2;
		}else{
				vCurrentBattLv = 2;
		}
	} else if (vtBattReal >= BATT_LV1_THESHOLD) {
		if(!RisingEnable){
			if (vCurrentBattLv >= 1)
				vCurrentBattLv = 1;
		}else{
				vCurrentBattLv = 1;
		}
	} else if (vtBattReal < BATT_LV1_THESHOLD) {
		vCurrentBattLv = 0;
	}
}


void setBattLevel(float vBattReal) {
	if (!fIsSystemInitialing && !vDisableGetBattTimer) {
		if (fIsCharging) {														//充电IC处于工作状态,读取充电IC中的电池数据
			battVoltCompensation(&vBattReal);
			vTestCompBatt = vBattReal;
			vCompensationVolt = vBattReal;
			if (OFF == displayParams.fIsMotorOn) {
				if (vBattReal >= BATT_LV4_THESHOLD&&(vCurrentBattLv<=4)) {
					vCurrentBattLv = 4;
				} else if (vBattReal >= BATT_LV3_THESHOLD&&(vCurrentBattLv<=3)) {
					vCurrentBattLv = 3;
				} else if (vBattReal >= BATT_LV2_THESHOLD&&(vCurrentBattLv<=2)) {
					vCurrentBattLv = 2;
				} else if (vBattReal >= BATT_LV1_THESHOLD&&(vCurrentBattLv<=1)) {
					vCurrentBattLv = 1;
				} else if (vBattReal < BATT_LV1_THESHOLD&&(vCurrentBattLv<=0)) {
					vCurrentBattLv = 0;
				}
			} else {
				processBattLevel(vBattReal,TRUE);
				if(vBattReal<BATT_LV0_THESHOLD){
					fPowerOffByBattLow=1;
				}
			}
		} else {
			adcVoltCompensation(&vBattReal);
			processBattLevel(vBattReal,FALSE);
			if(vBattReal<BATT_LV0_THESHOLD){
				fPowerOffByBattLow=1;
			}
		}
		displayParams.battIndex = vCurrentBattLv;
	}
}
/***********************************************************************************************************
  *  @brief			Turn Off System By User
  *
  *  @param [in] :
  *
  *  @param [out] :
  *
  *  @return :
  *
  *  @note :
  ************************************************************************************************************/
void systemOffByManual(void) {
	fIsSystemOff = TRUE;
	hotFuncDeinit();
	clear();
	displayParams.fIsMotorOn = 0;
	clr_MotorPowerPin;
//	displayParams.fIsReversed=0;
	displayParams.fIsObliqueDisplaying = 0;
	setMotorCtrlDuty(0);
	if (fIsCharging || !Get_ChargePin_Stts) {
		fIsCharging = TRUE;
		turnOffAllLightEffect();
		clearPowerKeyInfo();
		vTimeToTurnOffLCD = 0;
		fIsChargeDisplaying = 0;
		vLCDStatus = LCDWorking;
		displayParams.DisplayModeIndex = ChargingAtPowerDown;
		displaySystemMenu(&displayParams);
		if (fIsBattFully) {
			batteryPercentDisplay(100);
		}
		vChargePercent = 0;
	} else{
		systemPowerDown();
	}
}
/***********************************************************************************************************
  *  @brief
  *
  *  @param [in] :
  *
  *  @param [out] :
  *
  *  @return :
  *
  *  @note :
  ************************************************************************************************************/
void checkRealEntryPowerDownMode(void){

	systemOffByManual();

}
/***********************************************************************************************************
  *  @brief
  *
  *  @param [in] :
  *
  *  @param [out] :
  *
  *  @return :
  *
  *  @note :
  ************************************************************************************************************/
//void updateLEDStts(displayParamsStruct * disParams) {
//	if ((disParams->arrowIndex == HuesSetting) || (disParams->arrowIndex == SaturationSetting)) {
//		pUserColor->hues = disParams->hues;
//		pUserColor->saturation = (float) disParams->saturation / 100;
//		pUserColor->brightness = ((float) disParams->brightness) * (100 - RGBLED_REAL_START_DUTY) / (100 * 100) + (float) RGBLED_REAL_START_DUTY / 100;			//从9%起步
//		hsb2Rgb(pUserColor);			//数据仅用于电池电压补偿
//		setRgbData(RGB_DATA_LENGTH, disParams);
//	} else if (ColorTempSetting == disParams->arrowIndex) {
//		updateColorTemp(disParams, pUserColor);
//		setColorTempData(COLOR_TEMP_LENGTH, disParams);
//	} else if (disParams->arrowIndex >= Style1Setting && disParams->arrowIndex <= Style3Setting) {
//		fIsLightEffectOn = TRUE;
//		setEffectData(PRESET_EFFECT_DATA_LENGTH, disParams);
//	}
//}


//void	deinitMaxLevelChargeData(void){
//	fIsChargingCntStart=FALSE;
//	vL6ChargeTimer=0;
//}
///***************************************************************************
// *
// * Level 6 充电时,电池实际电压估算
// *
// ****************************************************************************/
//void MaxLevelChargeVoltCompensation(void) {
//	if (fIsCharging) {
//		if (vBattVol >= CHARGE_FINAL_VOLT) {
//			if (OFF == fIsLightActive) {									//LED 关闭时
//				if (!fIsChargingCntStart) {
//					fIsChargingCntStart = TRUE;
//					vL6ChargeTimer = 0x10000 - sysTimes.vSystem1s;
//				} else {
//					if (((vL6ChargeTimer + sysTimes.vSystem1s) & 0xffff) >= CHARGE_TIMER_CONST_STEP) {
//						vL6ChargeTimer = 0x10000 - sysTimes.vSystem1s;			//重新计时
//						if (vBattVolGap > 0.01) {
//							vBattVolGap -= 0.01;
//						}
//					}
//				}
//			}
//		} else if(vBattVol <= (CHARGE_FINAL_VOLT-0.02) ){
//			deinitMaxLevelChargeData();
//		}
//	}
//}
/***************************************************************************
 *
 * 剩余时间估算
 *
 * ignoreCompare:  TRUE忽略结果与原来值的比较,直接赋值
 * 				 FALSE:比较结果,新生成的值小于原来的值才赋值
 ****************************************************************************/
//u16 calcRemainTime(u8 ignoreCompare) {
//	float vtSpeedCW = 0, vtSpeedMW = 0;
////	float vtTemp = 0;
//	u16 totalTimer = 0;
//	if (ON == fIsLightActive) {					//LED ON
//			if (vCompensationVolt >= REMAIN_TIME_STEP5_THRESHOLD) {
//				vCwMaxTimer = REMAIN_TIME_STEP5_START + (REMAIN_TIME_STEP5_LAST * (vCompensationVolt - REMAIN_TIME_STEP5_THRESHOLD) / REMAIN_TIME_STEP5_VOLT_LAST);
//			} else if (vCompensationVolt >= REMAIN_TIME_STEP4_THRESHOLD) {
//				vCwMaxTimer = REMAIN_TIME_STEP4_START + (REMAIN_TIME_STEP4_LAST * (vCompensationVolt - REMAIN_TIME_STEP4_THRESHOLD) / REMAIN_TIME_STEP4_VOLT_LAST);
//			} else if (vCompensationVolt >= REMAIN_TIME_STEP3_THRESHOLD) {
//				vCwMaxTimer = REMAIN_TIME_STEP3_START + (REMAIN_TIME_STEP3_LAST * (vCompensationVolt - REMAIN_TIME_STEP3_THRESHOLD) / REMAIN_TIME_STEP3_VOLT_LAST);
//			} else if (vCompensationVolt >= REMAIN_TIME_STEP2_THRESHOLD) {
//				vCwMaxTimer = REMAIN_TIME_STEP2_START + (REMAIN_TIME_STEP2_LAST * (vCompensationVolt - REMAIN_TIME_STEP2_THRESHOLD) / REMAIN_TIME_STEP2_VOLT_LAST);
//			} else if (vCompensationVolt >= REMAIN_TIME_STEP1_THRESHOLD) {
//				vCwMaxTimer = REMAIN_TIME_STEP1_START + (REMAIN_TIME_STEP1_LAST * (vCompensationVolt - REMAIN_TIME_STEP1_THRESHOLD) / REMAIN_TIME_STEP1_VOLT_LAST);
//			}
//			if (!fIsLightEffectOn) {						//普通模式
//				if (pcaDataStruct.valueOfCw) {
//					vtSpeedCW = ((float) pcaDataStruct.valueOfCw / PWM_FRQ_CONST) * CW_MAX_TIMER_CONST;
//					//vtSpeedCW *= 1.0 / vCwMaxTimer;
//					vtSpeedCW /= vCwMaxTimer;
//				}
//				if (pcaDataStruct.valueOfMw) {
//					vtSpeedMW = ((float) pcaDataStruct.valueOfMw / PWM_FRQ_CONST) * MW_MAX_TIMER_CONST;
//					//vtSpeedMW *= 1.0 / vCwMaxTimer;
//					vtSpeedMW /= vCwMaxTimer;
//				}
//			} else {									//灯效模式
//				if ((Style1Setting == displayParams.arrowIndex)) {
//					if (displayParams.style1Value == 0) {
//						vtSpeedCW = displayParams.brightness*0.18 * CW_MAX_TIMER_CONST / (100*vCwMaxTimer);//等效0.18  CW duty
//					} else if (displayParams.style1Value == 1) {
//						vtSpeedCW =  displayParams.brightness*0.22 * CW_MAX_TIMER_CONST / (100*vCwMaxTimer);//等效0.22  CW duty
//					}
//					vtSpeedMW = 0;
//				}
//				if (Style2Setting == displayParams.arrowIndex) {
//					if (displayParams.style1Value == 0) {
//						vtSpeedCW =  displayParams.brightness*0.55 * CW_MAX_TIMER_CONST / (100*vCwMaxTimer);//等效0.55  CW duty
//						vtSpeedMW = 0;
//					} else if (displayParams.style1Value == 1) {
//						vtSpeedMW =  displayParams.brightness*0.55 * MW_MAX_TIMER_CONST / (100*vCwMaxTimer);//等效0.55  CW duty
//						vtSpeedCW = 0;
//					}
//				}
//			}
//			vtSpeedMW+=(0.01*MW_MAX_TIMER_CONST)/vCwMaxTimer;				//加上1%约13mA的LCD+系统耗电
//			totalTimer = (u16) (1.0 / (vtSpeedCW + vtSpeedMW));
//			if(fIsCharging){
//				ignoreCompare=TRUE;				//充电状态,允许剩余时间增加变化
//			}
//			if (vRemainTimer != totalTimer) {
//				if (ignoreCompare) {
//					updateRemainingTimeByValue(totalTimer);
//					vRemainTimer = totalTimer;
//				} else if (!fIsCharging && vRemainTimer && vRemainTimer > totalTimer) {
//					updateRemainingTimeByValue(totalTimer);
//
//					vRemainTimer = totalTimer;
//				}
//			}
//			return totalTimer;
////		}else {				//charging
////			return FALSE;
////		}
//	}
//	return FALSE;
//}
//void freshRemainTimeCheck(void) {
//	if (ON == fIsLightActive && (sysTimes.vSystem1s % 10 == 0)) {
//		u16 oldData = 0, newData = 0;
//		oldData = vRemainTimer;
//		newData = calcRemainTime(FALSE);
//		if (newData && (oldData <=newData)) {
////			if(++vRemainCheckTimes>6){
//			if(++vRemainCheckTimes>=6){
//				vRemainCheckTimes=0;
//				if(oldData>0){
//					vRemainTimer = oldData - 1;
//				}else{
//					vRemainTimer=0;
//				}
//				updateRemainingTimeByValue(vRemainTimer);
//			}
//		}else if(oldData > newData){
//			vRemainCheckTimes=0;
//		}
//	}
//}

/***************************************************************************
 *
 *  	电压补偿.灯开启时的电压补偿
 *
 ***************************************************************************/
void chkLightEffectModeVoltCompensation(float *volt){
	float vtMin=5.0;
	int i=0;
	if(fIsLightEffectOn){
		if(++vLightEfftectVoltDectCnt>BATT_ARRAY_SIZE){
			vLightEfftectVoltDectCnt=PARAMS_DATA_RESET;
			for(;i<BATT_ARRAY_SIZE;i++){
				if(vtMin>battArray[i]){
					vtMin=battArray[i];
				}
			}
			*volt=vtMin;
			fIsBattAdcValueGot=1;
		}else{
			fIsBattAdcValueGot=0;
		}
	}else{
			fIsBattAdcValueGot=1;
	}
}

/*********************************************************************
 *
 * 充电状态,根据当前电池两端电压估算充电电流
 *
 **********************************************************************/
void getChargingCurrentByVolt(float *vtCurrent, float vtVolt) {
	if (vtVolt <= KEEP_VOLT_THRESHOLD) {
		*vtCurrent = MAX_CHARGE_CURRENT;
	} else {
		*vtCurrent = MAX_CHARGE_CURRENT * (1 - (vtVolt - KEEP_VOLT_THRESHOLD) * (1.0 / (CHRAGE_FULL_VOLT - KEEP_VOLT_THRESHOLD)));
	}
}

/***********************************************************************************************************
  *  @brief				emulation pin as IO function
  *
  *  @param [in] :
  *
  *  @param [out] :
  *
  *  @return :
  *
  *  @note :
  ************************************************************************************************************/
void setSWD2NormalIO(void) {

		RCC->REGLOCK = UNLOCK_RAM_CONST;
		RCC->IOMUX = 0x5A690000;					//set swdio			b0=0/1   普通IO/仿真端口
		RCC->REGLOCK = LOCK_RAM_CONST;

}

/***********************************************************************************************************
  *  @brief				emulation pin as SWD function
  *
  *  @param [in] :
  *
  *  @param [out] :
  *
  *  @return :
  *
  *  @note :
  ************************************************************************************************************/
void set2SWDMode(void) {

		RCC->REGLOCK = UNLOCK_RAM_CONST;
		RCC->IOMUX = 0x5A690001;					//设置为swdio
		RCC->REGLOCK = LOCK_RAM_CONST;

}

void turnOffAllLightEffect(void) {

	fIsLightEffectOn = 0;
//	setOnOffData(ON_OFF_DATA_LENGTH,&displayParams,OFF);
}

void	 setMotorCtrlDuty(float duty){
	updatePWMChlDuty(Motro_Ctrl_CHANNEL,(1-duty),ADVTIM1);
}

//void setMotorCtrlData(uint16 pwmData){
//
//}


/***********************************************************************************************************
  *  @brief 			reset driver MCU
  *
  *  @param [in] :
  *
  *  @param [out] :
  *
  *  @return :
  *
  *  @note :
  ************************************************************************************************************/
//void resetDriverMcu(void) {

//	GPIO_InitTypeDef GPIO_InitStruct;
//	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;						//output 模式
//	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;						//GPIOx_OTYPER   0/1 推挽输出/开漏输出
//	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;						//不上拉
//	GPIO_InitStruct.GPIO_Pin = DriverReset_Pin;							//
//	GPIO_Init(DriverReset_Port, &GPIO_InitStruct);

//	clr_DriverResetPin;
//	sysTimes.vSystem5ms = 0;
//	while (sysTimes.vSystem5ms < 2);									//10ms reset signal
//	sysTimes.vSystem5ms = 0;
//	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;							//input 模式
//	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;							//
//	GPIO_Init(DriverReset_Port, &GPIO_InitStruct);
////	set_DriverResetPin;
//}

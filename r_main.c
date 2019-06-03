/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products.
* No other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
* applicable laws, including copyright laws. 
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING THIS SOFTWARE, WHETHER EXPRESS, IMPLIED
* OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NON-INFRINGEMENT.  ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY
* LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE FOR ANY DIRECT,
* INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR
* ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability 
* of this software. By using this software, you agree to the additional terms and conditions found by accessing the 
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2011, 2018 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/

/***********************************************************************************************************************
* File Name    : r_main.c
* Version      : CodeGenerator for RL78/G13 V2.05.03.01 [12 Nov 2018]
* Device(s)    : R5F101AA
* Tool-Chain   : CCRL
* Description  : This file implements main function.
* Creation Date: 2019/05/30
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "r_cg_cgc.h"
#include "r_cg_port.h"
#include "r_cg_serial.h"
#include "r_cg_timer.h"
#include "r_cg_wdt.h"
#include "r_cg_it.h"
/* Start user code for include. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"

/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
/* Start user code for pragma. Do not edit comment generated here */
enum{ JET_STOP=0, JET_SLOWUP, JET_HIGH, JET_LOW, JET_SLOWDOWN, JET_U_ERROR, JET_M_ERROR };
#define WDT		WDTE = 0xAC;

#define Start250nsCounter		(TS0=1<<4)
#define Set250nsCounterValue(x)	TDR04=x
#define Is250nsOverflow			(IF1H&0x80)
#define Reset250nsOverflow		(IF1H&=~0x80)
#define Stop250nsCounter		(TT0=1<<4)
#define Read250nsCounterValue	TCR04

#define Start10msTimer		(TS0=1<<0)
#define Is10msOverflow		(IF1L&0x10)
#define Reset10msOverflow	(IF1L&=~0x10)
#define Stop10msTimer		(TT0=1<<0)

/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */

uint8_t gMainLoop;

void MoveInit( void );
void MoveForceCycle( void );
void MovePwmUp( void );
uint8_t MovePamUp( int8_t setLv, uint16_t errHighValue, uint8_t isEndAuto );
void SetMovePamLv(int8_t lv );

void ResetCycleCounter(void);
uint16_t ReadAndTaskCycleCounter(void);

uint8_t DRV_DetectMotorPos( void );
void DRV_SetNextMovePatternWithPwm( void );
void DRV_SetNextMovePattern( void );
void DRV_ResetMovePattern( void );
void DRV_SetPam( int8_t val );

void Init250nsCounter( void );
void Init10msTimer( void );
void SeqTestMode( void );


/* End user code. Do not edit comment generated here */
void R_MAIN_UserInit(void);

/***********************************************************************************************************************
* Function Name: main
* Description  : This function implements main function.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void main(void)
{
    R_MAIN_UserInit();
    /* Start user code. Do not edit comment generated here */
	
	R_TAU0_Channel2_Start();
	R_IT_Start();
	Init10msTimer();
	Init250nsCounter();	

	DRV_SetPam(-1);
	
	// 基板検査シーケンスに移行
	while( !P6_bit.no0 && !P13_bit.no7 ){
//	while( !P6_bit.no0 ){
		static uint16_t cnt=0;
		if( gMainLoop ){
			WDT;
			gMainLoop = 0;
			if( cnt < 200 ) cnt ++;
			else{
				SeqTestMode();
				break;
			}
		}
	}

	SetMovePamLv(-1);
	DRV_ResetMovePattern();
	
	while (1){
		static uint8_t jetBusState = JET_STOP;
		static uint8_t PamOnDly=10;
		
		if( gMainLoop ){
			WDT;
			gMainLoop = 0;

			switch( jetBusState ){
			case JET_STOP:
				if( isJetSw() ){
					SetMovePamLv(0);
					while( PamOnDly ){
						if( gMainLoop ){
							WDT;
							gMainLoop=0;
							PamOnDly --;
						}
					}
					PamOnDly = 10;
					jetBusState = JET_SLOWUP;
				}else{
					PamOnDly = 10;
					SetMovePamLv(-1);
					DRV_ResetMovePattern();
				}
				break;
			case JET_SLOWUP:
				MoveInit();
				MoveForceCycle();
				MovePwmUp();
				jetBusState = JET_HIGH;
			case JET_HIGH:
				switch( MovePamUp(25,272,0) ){
				case 1: jetBusState = JET_M_ERROR;	break;
				case 2: jetBusState = JET_U_ERROR;	break;
				case 3: jetBusState = JET_LOW;		break;
				case 4: jetBusState = JET_SLOWDOWN;	break;
				}
				break;
			case JET_LOW:
				switch( MovePamUp(11,192,0) ){
				case 1: jetBusState = JET_M_ERROR;	break;
				case 2: jetBusState = JET_U_ERROR;	break;
				case 3:
				case 4: jetBusState = JET_SLOWDOWN; break;
				}
				break;
			case JET_SLOWDOWN:
				MovePamUp(0,192,1);
//				DRV_SetPam(-1);
				jetBusState = JET_STOP;
				break;
			case JET_U_ERROR:
				DRV_ResetMovePattern();
				SetMovePamLv(0);
				if( isJetSw() ) jetBusState = JET_SLOWUP;
				break;
			case JET_M_ERROR:
				DRV_ResetMovePattern();
				SetMovePamLv(0);
				if( isJetSw() ) jetBusState = JET_STOP;				
				break;
			}
		}
    }
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: R_MAIN_UserInit
* Description  : This function adds user code before implementing main function.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_MAIN_UserInit(void)
{
    /* Start user code. Do not edit comment generated here */
    EI();
    /* End user code. Do not edit comment generated here */
}

/* Start user code for adding. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: MoveInit
* Description  : 仕様書:ソフトスタート STEP-1 初期位置合わせ
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void MoveInit( void ){
	uint8_t cycCnt=10;		// 出力回数をカウント
	uint8_t timCnt=0;		// 10msの回数をカウント
	
	// U-Y間に 50ms/10ms間隔で50%PWM入力
	// 50%パルス
	TDR03 = 8000;
	TDR05 = 8000;
	TDR07 = 8000;

	P1= 0x02;		TOE0|= 0x20;
	Start10msTimer;
	while(cycCnt){
		WDT;
		if( Is10msOverflow ){
			Reset10msOverflow;
			if( timCnt == 4 ){
				if( cycCnt ){
					P1= 0x02;	TOE0|= 0x20;					// 出力する。
					cycCnt --;
				}else return;
				timCnt = 0;
			}else{
				P1 = 0x00;	TOE0&= ~0xA8;	TO0 |= 0xA8;		// 停止する。
				timCnt ++;
			}
		}
	}
}

/***********************************************************************************************************************
* Function Name: MoveForceCycle
* Description  : 仕様書:ソフトスタート STEP-2 強制転流
* Arguments    : None
* Return Value : None
* Ext Variable : sTrdValue -> 現在の転流タイマ
***********************************************************************************************************************/
uint16_t sTrdValue;				// 全体の回転周期同期用の変数。
void MoveForceCycle( void ){
	uint16_t cycCnt = 0;
	uint16_t duty = 9600;

	sTrdValue = 0xFFFF;

	// 60%パルス
	TDR03 = duty;
	TDR05 = duty;
	TDR07 = duty;
	IF1L &= ~0x40;
	TOE0 = 0x20;

	Set250nsCounterValue(sTrdValue);
	Start250nsCounter;
	
//	TS0 = 0x00AE;
	DRV_SetNextMovePatternWithPwm();

	while(sTrdValue >= 0x61A8 ){
		WDT;
		if( Is250nsOverflow ){
			Reset250nsOverflow;
			DRV_SetNextMovePatternWithPwm();
			if( cycCnt == 6 ){
				cycCnt = 0;
#if 0	// ここでPWMを大きくする実験用
				if( duty < 16100 ) duty += 800;
				TDR03 = duty;
				TDR05 = duty;
				TDR07 = duty;
#endif
				sTrdValue -= 0x0800;
			}else cycCnt ++;
			Set250nsCounterValue(sTrdValue);
		}
	}
	// sTrdValue : 現在の250nsカウンター値
}

/***********************************************************************************************************************
* Function Name: MovePwmUp
* Description  : 仕様書:ソフトスタート STEP-3 DUTY UP(PWM)
* Arguments    : None
* Return Value : None
* Ext Variable : sTrdValue -> 現在の転流タイマ
***********************************************************************************************************************/
void MovePwmUp( void ){
	static uint8_t DetectDly=0;
	static uint16_t TimSampStart;
	static uint16_t TimModeChange;
	static uint16_t TmpModeChange;
	uint16_t Duty = 9600;
	uint8_t State=0;
	uint8_t endFlg=0;
	
	Start10msTimer;
	Set250nsCounterValue(0xFFFF);
	Start250nsCounter;

	TimSampStart = 0xFFFF -( sTrdValue -(sTrdValue>>3));	// 初期値-変数値がカウンター値
	TmpModeChange = sTrdValue>>4;							// カウンター値

	while(1){
		WDT;

		if( Is10msOverflow ){
			Reset10msOverflow;
			if( Duty != 16200 ){
				Duty += 200;
				TDR03 = Duty;
				TDR05 = Duty;
				TDR07 = Duty;
			}else{
				endFlg = 1;
			}
		}

		switch( State ){
		case 0:		// 無検知区間
			if( Read250nsCounterValue <= TimSampStart){
				State = 1;
				DetectDly = 1;
			}else{
				break;
			}
		case 1:		// 検知タイミング
			if( TO0 != 0x00A8 ){
//			if( 1 ){
				if( !DetectDly && DRV_DetectMotorPos() ){
//				if( 1 ){
					State = 2;
					TimModeChange = Read250nsCounterValue - TmpModeChange;	// 初期値-変数値がカウンター値
					break;
				}else{
					DetectDly = 0;
				}
			}else{
				DetectDly = 1;
			}
		
			if( Is250nsOverflow ){
				Reset250nsOverflow;
				Start250nsCounter;
				DRV_SetNextMovePatternWithPwm();
				State = 0;
				TimSampStart = 0xFFFF - (0xFFFF>>3);
				TmpModeChange = 0xFFFF >>4;
			}
			break;
		case 2:		// 転流タイミング
			if( Read250nsCounterValue <= TimModeChange ){
				Set250nsCounterValue(0xFFFF);
				Start250nsCounter;
				DRV_SetNextMovePatternWithPwm();
				State = 0;
				
				// 終了処理（次に続くPAM_UPとの同期の為、この位置で終了させる
				if( endFlg ){
					sTrdValue = 0xFFFF -TimModeChange;			// 250nsカウンター値
					return;
				}
				
				TimSampStart = TimModeChange +((0xFFFF-TimModeChange)>>3);	// 初期値-変数値がカウンター値
				TmpModeChange = (0xFFFF -TimModeChange) >>4;				// カウンター値
			}
			break;
		}
	}
}

		
/***********************************************************************************************************************
* Function Name: SetMovePamLv
* Description  : PAM値を強制的に変更する（MovePamUp関数以外で値を触りたい時用）
* Arguments    : PAM設定値
* Return Value : None
* Ext Variable : sPos -> 現在の転流位置記録
***********************************************************************************************************************/
int8_t sPamLv = 1;
void SetMovePamLv(int8_t lv ){
	sPamLv = lv;
	DRV_SetPam(lv);
}

/***********************************************************************************************************************
* Function Name: MovePamUp
* Description  : 仕様書:ソフトスタート STEP-4 電圧 UP(PAM) 他 PAM10mSec変動用
* Arguments    : setLv -> 目標PAM値(10mSec毎で1変動）
*				 errHighValue -> 回転数検知時の高回転側エラー閾値(50msの転流回数×8回)
*				 isEndAuto -> 目標PAM値到達後、自動で完了するか？JetSw押下で完了するか？
* Return Value : (1)機器エラー (2)ユーザエラー (3)正常終了
* Ext Variable : sPamLv -> 現在設定しているPAM値
*			   : sTrdValue -> 現在の転流タイマ
***********************************************************************************************************************/
uint8_t MovePamUp( int8_t setLv, uint16_t errHighValue, uint8_t isEndAuto ){
	static uint8_t DetectDly=0;
	static uint16_t TimSampStart;
	static uint16_t TimModeChange;
	static uint16_t TmpModeChange;
	uint32_t OffTimer = 0;
	uint8_t State=0;
	
	static uint16_t Cyc;
	uint8_t errCntHigh,errCntLow;
	
	uint8_t endFlg=0;

	Start10msTimer;
	
	TimSampStart = 0xFFFF -sTrdValue;					// 初期値-変数値がカウンター値
	TmpModeChange = sTrdValue>>4;						// 差分の時間を格納
	ResetCycleCounter();
	errCntHigh = 0;		errCntLow = 0;
	
	while(1){
		WDT;

		if( Is10msOverflow ){
			Reset10msOverflow;
			OffTimer ++;
			
			if( sPamLv > setLv ){
				sPamLv --;
				DRV_SetPam(sPamLv);
			}else if( sPamLv < setLv ){
				sPamLv ++;
				DRV_SetPam(sPamLv);
			}else if( isEndAuto || isJetSw() ){
				endFlg = 1;
			}
		}

		switch(State){
		case 0:		// 無検知区間
			if( Read250nsCounterValue <= TimSampStart){
				State = 1;
				DetectDly = 1;
			}
			break;
		case 1:		// 検知タイミング
			if( TO0 != 0x00A8 ){
//			if( 1 ){
				if( !DetectDly && DRV_DetectMotorPos() ){
//				if( 1 ){
					State = 2;
					TimModeChange = Read250nsCounterValue - TmpModeChange;
				}else{
					DetectDly = 0;
				}
			}else{
				DetectDly = 1;
			}
		
			if( Is250nsOverflow ){
				return 1;
			}
			break;
		case 2:		// 転流タイミング
			if( Read250nsCounterValue <= TimModeChange ){
				Set250nsCounterValue(0xFFFF);
				Start250nsCounter;
				DRV_SetNextMovePattern();
				State = 0;

				// 終了処理（次に続くPAM_UPとの同期の為、この位置で終了させる
				if( endFlg ){
					sTrdValue = 0xFFFF -TimModeChange;			// 250nsカウンター値
					return 3;
//				}else if( OffTimer>=(90000/15) ){
				}else if( OffTimer>=(90000) ){
					return 4;
				}
				
				
				Cyc = ReadAndTaskCycleCounter();
				if( Cyc != 0xFFFF){
					if( Cyc > errHighValue ){
						errCntHigh ++;
						if( errCntHigh > 10 ) return 1;
					}else errCntHigh = 0;
					
					if( Cyc <= 24 ){
						errCntLow ++;
						if( errCntLow > 60 ) return 2;
					}else errCntLow = 0;
				}
				
				TimSampStart = TimModeChange +((0xFFFF-TimModeChange)>>3);	// 初期値-変数値がカウンター値
				TmpModeChange = (0xFFFF -TimModeChange) >>4;				// カウンター値

			}
			break;
		}
	}
}


/***********************************************************************************************************************
* Function Name: ResetCycleCounter
* Description  : 回転数検出処理。データを初期化する
* Arguments    : None
* Return Value : None
* Ext Variable : sCycleValue[10] -> 転流の回数を数える配列
*			   : sCycleWp -> 配列用のイテレータ
***********************************************************************************************************************/
uint16_t sCycleValue[10];
uint8_t sCycleWp;
void ResetCycleCounter(void){
	gMainLoop = 0;
	sCycleWp = 0;
	sCycleValue[0] = 0;
}

/***********************************************************************************************************************
* Function Name: CalCycleValue
* Description  : 回転数検出処理。配列を埋めた後、上下フィルタ付合計処理を行う(平均×8)
* Arguments    : None
* Return Value : None
* Ext Variable : sCycleValue[10] -> 転流の回数を数える配列
*			   : sCycleWp -> 配列用のイテレータ
***********************************************************************************************************************/
uint16_t CalCycleValue( void ){
	uint16_t min=0xFFFF,max=0;
	uint16_t tmp=0,i;

	for( i=0;i<10;i++){
		tmp += sCycleValue[i];
		if( min > sCycleValue[i] ) min = sCycleValue[i];
		if( max < sCycleValue[i] ) max = sCycleValue[i];
	}

	tmp -= (min+max);
	return tmp;
}
	
/***********************************************************************************************************************
* Function Name: ReadAndTaskCycleCounter
* Description  : 回転数検出処理。gMainLoopを利用し50ms毎データを10回取る。10回目で結果を出力
* Arguments    : None
* Return Value : (0xFFFF)データ蓄え中フラグ (other)計算結果
* Ext Variable : sCycleValue[10] -> 転流の回数を数える配列
*			   : sCycleWp -> 配列用のイテレータ
***********************************************************************************************************************/
uint16_t ReadAndTaskCycleCounter(void){
	static uint16_t r;
	
	sCycleValue[sCycleWp] ++;
	if( gMainLoop >= 5){
		gMainLoop = 0;
		if( sCycleWp < 9 ){
			sCycleWp ++;
			sCycleValue[sCycleWp] = 0;
		}else{
			sCycleWp = 0;
			r = CalCycleValue();
			sCycleValue[sCycleWp] = 0;
			return r;
		}
	}
	return 0xffff;
}		
		
// H/ U:P11  V:P13  W:P15
// L/ X:TO07(P10) Y:TO05(P12) Z:TO03(P14)
/***********************************************************************************************************************
* Function Name: SetNextMovePatternWithPwm
* Description  : BLMの制御をひとつ転流させる。(Dutyは外部で設定)
* Arguments    : None
* Return Value : None
* Ext Variable : sPos -> 現在の転流位置記録
***********************************************************************************************************************/
static uint8_t sPos = 5;
void DRV_SetNextMovePatternWithPwm( void ){

	sPos = (sPos +1) %6;	// 内部処理としてローテーションさせる。
	
	// DeadTime生成
	P1 = 0x00;
	TOE0&= ~0xA8;
	TO0 |= 0xA8;		// Under停止ラインをHighに持ち上げる
	
	// Switch
	switch( sPos ){
	case 5:	P1= 0x02;		TOE0|= 0x20;	break;		// HU LV(Y)
	case 0:	P1= 0x02;		TOE0|= 0x08;	break;		// HU LW(Z)
	case 1:	P1= 0x08;		TOE0|= 0x08;	break;		// HV LW(Z)
	case 2:	P1= 0x08;		TOE0|= 0x80;	break;		// HV LU(X)
	case 3:	P1= 0x20;		TOE0|= 0x80;	break;		// HW LU(X)
	case 4:	P1= 0x20;		TOE0|= 0x20;	break;		// HW LV(Y)
	}
}

/***********************************************************************************************************************
* Function Name: DRV_SetNextMovePattern
* Description  : BLMの制御をひとつ転流させる。(Duty100%強制)
* Arguments    : none
* Return Value : none
* Ext Variable : sPos -> 現在の転流位置記録
***********************************************************************************************************************/
void DRV_SetNextMovePattern( void ){

	// DeadTime生成
	P1 = 0x00;
	TOE0&= ~0xA8;
	TO0 |= 0xA8;		// Under停止ラインをHighに持ち上げる
	sPos = (sPos +1) %6;	// 内部処理としてローテーションさせる。
	
	// Switch
	switch( sPos ){
	case 5:	P1= 0x02;		TO0&= ~0x20;	break;		// HU LV(Y)
	case 0:	P1= 0x02;		TO0&= ~0x08;	break;		// HU LW(Z)
	case 1:	P1= 0x08;		TO0&= ~0x08;	break;		// HV LW(Z)
	case 2:	P1= 0x08;		TO0&= ~0x80;	break;		// HV LU(X)
	case 3:	P1= 0x20;		TO0&= ~0x80;	break;		// HW LU(X)
	case 4:	P1= 0x20;		TO0&= ~0x20;	break;		// HW LV(Y)
	}
}

/***********************************************************************************************************************
* Function Name: DRV_ResetMovePattern
* Description  : BLMの制御を初期化する。
* Arguments    : none
* Return Value : none
* Ext Variable : sPos -> 現在の転流位置記録
***********************************************************************************************************************/
void DRV_ResetMovePattern( void ){
	sPos = 5;
	
	P1 = 0x00;
	TOE0&= ~0xA8;
	TO0 |= 0xA8;		// Under停止ラインをHighに持ち上げる
}

/***********************************************************************************************************************
* Function Name: DetectMotorPos
* Description  : BLMからのFBから位置を検出する。
* Arguments    : mode -> 検知したいモードパターン
* Return Value : 検知判断
***********************************************************************************************************************/
uint8_t DRV_DetectMotorPos(void);
#pragma inline DRV_DetectMotorPos
uint8_t sCnt=0;
const uint8_t DetectData[] = { 0x3,0x2,0x6,0x4,0x5,0x1 };
uint8_t DRV_DetectMotorPos( void ){
	// P120:U P121:V P122:W
	if( (P12&0x07) == DetectData[sPos] ){
		if( sCnt != 4 ) sCnt ++;
		else{
			sCnt = 0;
			return 1;
		}
	}else{
		sCnt = 0;
	}
	return 0;
//	return 1;
}


/***********************************************************************************************************************
* Function Name: Init250nsCounter
* Description  : 250nsカウンター初期化
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void Init250nsCounter( void ){
	TT0 = 1<<4;
	TMR04 = 0x8000;
}

/***********************************************************************************************************************
* Function Name: Init10msTimer
* Description  : 10msタイマーの初期化
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void Init10msTimer( void ){
	TT0 = 1<<0;
	TMR00 = 0x8000;
	TCR00 = 40000;
	TDR00 = 40000;
}

/***********************************************************************************************************************
* Function Name: DRV_SetPam
* Description  : PAM回路の出力レベルを設定する。(0-32)
* Arguments    : val -> 設定値
* Return Value : none
***********************************************************************************************************************/
void DRV_SetPam(int8_t val){
	if( val > 32 ) val = 32;
	
	if( val != -1 ){
		P3_bit.no0 = 1;
		P14_bit.no7 = (val&0x10) ?1 :0;
		P2 = val&0x0F;
	}else{
		P2 = 0;
		P14_bit.no7 = 0;
		P3_bit.no0 = 0;
	}
}

/***********************************************************************************************************************
* Function Name: SeqTestMode
* Description  : 基板検査を行うシーケンス
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/
void SeqTestMode( void ){
	uint16_t secVal = 0;
	uint8_t pamVal = 0;
	uint8_t fbRCnt,fbResult=1;
	
	DRV_SetPam( pamVal );
	Start10msTimer;

	// 停止
	P1 = 0x00;	TOE0&= ~0xA8;	TO0 |= 0xA8;		// Under停止ラインをHighに持ち上げる
	
//	while(1){
//		if( Is10msOverflow ){
//			Reset10msOverflow;
//			secVal++;
//			if( (secVal/100) <= 31 ) DRV_SetPam(secVal/100);
//			else DRV_SetPam(0);
//		}
//	}
	
	
	// PAMを0x00にして、3秒まで待機
	while( secVal < 300 ){
		if( Is10msOverflow ){
			Reset10msOverflow;
			secVal++;
			WDT;
		}
	}

	// PAMを0x0Bまで上げて、8秒まで待機
	while( secVal < 800 ){
		if( Is10msOverflow ){
			Reset10msOverflow;
			if( pamVal < 0x0B ) DRV_SetPam( ++pamVal );
			secVal++;
			WDT;
		}
	}
	
	// PAMを0x19まで上げて、15秒まで待機
	while( secVal < 1300 ){
		if( Is10msOverflow ){
			Reset10msOverflow;
			secVal++;
			if( pamVal < 0x19 ) DRV_SetPam( ++pamVal );
			WDT;
		}
	}
	
	// PAMを0x1Fまで上げて、18秒まで待機
	while( secVal < 1800 ){
		if( Is10msOverflow ){
			Reset10msOverflow;
			secVal++;
			if( pamVal < 0x1F ) DRV_SetPam( ++pamVal );
			WDT;
		}
	}
	
//	secVal = 4000;
	

	// PAMを0x1Eに設定して、41秒まで待機
	P1 = 0x00;	TO0 |= 0xA8;
	while( secVal < 4100 ){
		if( Is10msOverflow ){
			Reset10msOverflow;
			secVal++;
			if( pamVal == 0x1F ) DRV_SetPam( --pamVal );
//			if( pamVal > 25 ) DRV_SetPam( --pamVal );
			WDT;
		}
	}
	
	// UY相に印加して、FBが U=H,Y=L
	P1= 0x02;	TO0 &= ~0x20;
	fbRCnt = 4;
	while( secVal < 4400 ){
		if( Is10msOverflow ){
			Reset10msOverflow;
			if( fbRCnt && (secVal > 4200) ){
				fbRCnt --;
				if( (P12&0x03) != 0x01 ) fbResult = 0;
			}
			secVal++;
			WDT;
		}
	}

	// 停止
	P1 = 0x00;	TO0 |= 0xA8;
	while( secVal < 4500 ){
		if( Is10msOverflow ){
			Reset10msOverflow;
			secVal++;
			WDT;
		}
	}

	// VZ相に印加して、FBが V=H,Z=L
	P1= 0x08;	TO0 &= ~0x08;
	fbRCnt = 4;
	while( secVal < 4800 ){
		if( Is10msOverflow ){
			Reset10msOverflow;
			if( fbRCnt && (secVal > 4600) ){
				fbRCnt --;
				if( (P12&0x06) != 0x02 ) fbResult = 0;
			}
			secVal++;
			WDT;
		}
	}

	// 停止
	P1 = 0x00;	TO0 |= 0xA8;
	while( secVal < 4900 ){
		if( Is10msOverflow ){
			Reset10msOverflow;
			secVal++;
			WDT;
		}
	}
	
	// WX相に印加して、FBが W=H,X=L
	P1= 0x20;	TO0 &= ~0x80;
	fbRCnt = 4;
	while( secVal < 5200 ){
		if( Is10msOverflow ){
			Reset10msOverflow;
			if( fbRCnt && (secVal > 5000) ){
				fbRCnt --;
				if( (P12&0x05) != 0x04 ) fbResult = 0;
			}
			secVal++;
			WDT;
		}
	}
		
	// FB入力検査結果をPAMで出力
	P1 = 0x00;	TO0 |= 0xA8;
	while( secVal < 5700 ){
		if( Is10msOverflow ){
			Reset10msOverflow;
			secVal++;
			if( pamVal > ((fbResult) ?0x08 :0x11) ) DRV_SetPam( --pamVal );
			WDT;
		}
	}

	// 
	while( secVal < 6000 ){
		if( Is10msOverflow ){
			Reset10msOverflow;
			secVal++;
			if( pamVal > ((fbResult) ?0x00 :0x11) ) DRV_SetPam( --pamVal );
			WDT;
		}
	}	
}

/* End user code. Do not edit comment generated here */

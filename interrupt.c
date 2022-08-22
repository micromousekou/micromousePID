#include "iodefine.h"
#include "interrupt.h"
#include "glob_var.h"
#include "parameters.h"
#include "spi.h"
#include "i2c.h"
#include "run.h"
#include "mytypedef.h"
#include "portdef.h"
#include "interface.h"
#include "machine.h"


void int_cmt0(void)
{
	/*****************************************************************************************
	タイマのカウント
		
	*****************************************************************************************/
	timer++;
	cnt++;
	
	
	/*****************************************************************************************
	PID制御
		
	*****************************************************************************************/
	//各パラメータ設定---------------------------------------------------------------------------------
	float output_R, output_L, trim_R, trim_L, targetV, Perr_R, Perr_L, Ierr_L, Ierr_R, Derr_L, Derr_R; //関数設定
	short NR , NL;
	Perr_R = Perr_L = 0; //誤差
	//output_L = speed_l*100; //出力値左
	//output_R = speed_r*100; //出力値右
	MSP = 239; //最大モータ出力
	
	//MOT_CWCCW_R = MOT_R_FORWARD; //右モータ回転方向設定
	//MOT_CWCCW_L = MOT_L_FORWARD; //左モータ回転方向設定
						
	//変更する必要のあるパラメータ左モータ-----------------------
	KP_L = 0.02;//	0.02; 	//比例ゲイン左設定
	KI_L = 0;//0.0008/1000;//1/1000; 	//積分ゲイン左設定
	KD_L = 0;//1*1000; 	//微分ゲイン左設定
	trim_L = 0.4;//0.379; 	//左モータトリム設定
	
	Perr_L = Ierr_L = Derr_L = 0;
	//変更する必要のあるパラメータ右モータ-----------------------
	KP_R = 0.02;//0.01; 	//比例ゲイン右設定
	KI_R = 0;//0.0008/1000;// 1/1000; 	//積分ゲイン右設定
	KD_R = 0;//1*1000; 	//微分ゲイン右設定
	trim_R = 0.4;//0.379; 	//右モータトリム設定
	
	
	Perr_R = Ierr_R = Derr_R = 0;
	targetV = 3000;//2000; //目標値[rpm]
	
	// ---------------------------------------------------------------------------------------------------
	
	/*モータ制御---------------------------------------
	for (timer=0; timer<1000;timer){
	
		if(timer<101){    //100ms待機時間
		
			while(timer<100);
			
			NL = (short)(speed_l*1000*60/(TIRE_DIAMETER*PI));//単位は[rpm]
			NR = (short)(speed_r*1000*60/(TIRE_DIAMETER*PI));
			
			log[12][timer] = NL;
			log[13][timer] = MOT_OUT_L;
			log[3][timer] = NR;
			log[4][timer] = MOT_OUT_R;
			
				}
		
		MOT_POWER_ON; //モータ回転開始＆回転時のデータ取得開始
		*/
		
										
		output_L = NL;
		output_R = NR;
		
		//比例要素-----(目標値 - 出力)
		Perr_L = targetV-output_L;
		Perr_R = -1*targetV-output_R;
		
		//積分要素-----(今までのエラー + 現在のエラー)
		Ierr_L = Ierr_L+Perr_L;
		Ierr_R = Ierr_R+Perr_R;
		
		//微分要素-----(現在のエラー - ひとつ前のエラー)
		Derr_L = Perr_L-Derr_L;
		Derr_R = Perr_R-Derr_R;
		
		//PID制御を用いた計算----------------------------------------
		MOT_OUT_L = MSP*trim_L+KP_L*Perr_L+KI_L*Ierr_L+KD_L*Derr_L; 					
		MOT_OUT_R = MSP*trim_R+KP_R*Perr_R+KI_R*Ierr_R+KD_R*Derr_R;
		
		//リミッター設定---------------------------------------------
		if (MOT_OUT_L >239){
			MOT_OUT_L =239;
		}
		if (MOT_OUT_L <0){
			MOT_OUT_L =0;
		}
		
		if (MOT_OUT_R >239){
			MOT_OUT_R =239;
		}
		if (MOT_OUT_R <0){
			MOT_OUT_R =0;
		}
		//-----------------------------------------------------------
		
		NL = (short)(speed_l*1000*60/(TIRE_DIAMETER*PI));//単位は[rpm]
		NR = (short)(speed_r*1000*60/(TIRE_DIAMETER*PI));
		
		log[12][timer] = NL;
		log[13][timer] = MOT_OUT_L;
		log[3][timer] = NR;
		log[4][timer] = MOT_OUT_R;	
	
	
}

void int_cmt1(void)		//センサ読み込み用り込み
{
	/*****************************************************************************************
	A/D変換
		センサとバッテリー電圧取得
	*****************************************************************************************/
	static int state = 0;	//読み込むセンサのローテーション管理用変数
	int i;
	//long log_cnt;
	
	switch(state)
	{
		case 0:		//右センサ読み込み

			//差分フィルタ
			S12AD.ADANS0.BIT.ANS0=0x0004;			//AN002
			S12AD.ADCSR.BIT.ADST=1;				//AD変換開始
			while(S12AD.ADCSR.BIT.ADST);			//AD変換終了まで待つ
			sen_r.d_value = S12AD.ADDR2;			//環境値を保存
			
			SLED_R = 1;					//LED点灯
			for(i = 0; i < WAITLOOP_SLED; i++);		//フォトトランジスタの応答待ちループ
			S12AD.ADANS0.BIT.ANS0=0x0004;			//AN002
			S12AD.ADCSR.BIT.ADST=1;				//AD変換開始
			while(S12AD.ADCSR.BIT.ADST);			//AD変換終了まで待つ
			SLED_R = 0;					//LED消灯
			
			sen_r.value = (S12AD.ADDR2 - sen_r.d_value);	//値を保存
			
			break;


		case 1:		//前左センサ読み込み

			//差分フィルタ
			S12AD.ADANS0.BIT.ANS0=0x0001;			//AN000
			S12AD.ADCSR.BIT.ADST=1;				//AD変換開始
			while(S12AD.ADCSR.BIT.ADST);			//AD変換終了まで待つ
			sen_fl.d_value = S12AD.ADDR0;			//値を保存
		
			SLED_FL = 1;					//LED点灯
			for(i = 0; i < WAITLOOP_SLED; i++);		//フォトトランジスタの応答待ちループ
			S12AD.ADANS0.BIT.ANS0=0x0001;			//AN000
			S12AD.ADCSR.BIT.ADST=1;				//AD変換開始
			while(S12AD.ADCSR.BIT.ADST);			//AD変換終了まで待つ
			SLED_FL = 0;					//LED消灯

			sen_fl.value = (S12AD.ADDR0 - sen_fl.d_value);	//値を保存

			break;


		case 2:		//前右センサ読み込み
		
			//差分フィルタ
			S12AD.ADANS0.BIT.ANS0=0x0040;			//AN006
			S12AD.ADCSR.BIT.ADST=1;				//AD変換開始
			while(S12AD.ADCSR.BIT.ADST);			//AD変換終了まで待つ
			sen_fr.d_value = S12AD.ADDR6;			//値を保存
		
			SLED_FR = 1;					//LED点灯
			for(i = 0; i < WAITLOOP_SLED; i++);		//フォトトランジスタの応答待ちループ
			S12AD.ADANS0.BIT.ANS0=0x0040;			//AN006
			S12AD.ADCSR.BIT.ADST=1;				//AD変換開始
			while(S12AD.ADCSR.BIT.ADST);			//AD変換終了まで待つ
			SLED_FR = 0;					//LED消灯
			
			sen_fr.value = (S12AD.ADDR6 - sen_fr.d_value);	//値を保存
			
			break;


		case 3:		//左センサ読み込み
		
			//差分フィルタ
			S12AD.ADANS0.BIT.ANS0=0x0002;			//AN001
			S12AD.ADCSR.BIT.ADST=1;				//AD変換開始
			while(S12AD.ADCSR.BIT.ADST);			//AD変換終了まで待つ
			sen_l.d_value = S12AD.ADDR1;			//値を保存
			
			SLED_L = 1;					//LED点灯
			for(i = 0; i < WAITLOOP_SLED; i++)	;	//フォトトランジスタの応答待ちループ
			S12AD.ADANS0.BIT.ANS0=0x0002;			//AN001
			S12AD.ADCSR.BIT.ADST=1;				//AD変換開始
			while(S12AD.ADCSR.BIT.ADST);			//AD変換終了まで待つ
			SLED_L = 0;					//LED消灯
			
			sen_l.value = (S12AD.ADDR1 - sen_l.d_value);	//値を保存
			
			break;
	}
	
	state++;		//4回ごとに繰り返す
	if(state > 3)
	{
		state = 0;
	}
	
	S12AD.ADANS0.BIT.ANS0=0x0200;			//AN009
	S12AD.ADCSR.BIT.ADST=1;				//AD変換開始
	while(S12AD.ADCSR.BIT.ADST);			//AD変換終了まで待つ
	V_bat = (2.0*3.3*(float)(S12AD.ADDR9/4095.0) );
	if(V_bat < 3.5){
		
		//モータ止める
		Duty_r = 0;
		Duty_l = 0;
		MOT_POWER_OFF;	//PC6(SLEEPピン)
		
		//ブザー鳴らし続ける
		while(1){
			BEEP();
		}
	}
	/*****************************************************************************************
	1kHzごとにログを取得
		
	*****************************************************************************************/
	
	if(log_timer % 4 == 0 && log_flag==1 ){
		if(log_timer < (LOG_CNT*4)){

			log[0][log_timer/4] = (int)(len_mouse);
			log[1][log_timer/4] = (int)(1000*tar_speed);
			log[2][log_timer/4] = (int)(1000*speed);
			//log[3][log_timer/4] = (int)(100*Duty_r);
			//log[4][log_timer/4] = (int)(100*Duty_l);
			log[5][log_timer/4] = (int)(1000*V_bat);
			log[6][log_timer/4] = (int)(tar_degree*10);
			log[7][log_timer/4] = (int)(degree*10);
			log[8][log_timer/4] = (int)(tar_ang_vel*1000);
			log[9][log_timer/4] = (int)(ang_vel*1000);
			log[10][log_timer/4] = (int)(I_tar_ang_vel);
			log[11][log_timer/4] = (int)(ang_acc*1000);
			//log[12][log_timer/4] =(int)(speed_l*100);
			//log[13][log_timer/4] =(int)(speed_l*100);
			
		}
	}	
	
	log_timer++;
		
}

void int_cmt2(void)
{
	static unsigned int	enc_data_r;	//エンコーダの生データ
	static unsigned int	enc_data_l;	//エンコーダの生データ 
	static short	state;
	/*****************************************************************************************
	エンコーダ関連
		値の取得　速度更新　距離積分など
	*****************************************************************************************/	
	if(state == 0){
		RSPI0.SPCMD0.BIT.SSLA = 	0x00;	//SSL信号アサート設定(SSL0を使う)
		preprocess_spi_enc(0xFFFF);	//Read Angle
		enc_data_r = Get_enc_data();
		state = 1;
	}else{
		RSPI0.SPCMD0.BIT.SSLA = 	0x02;	//SSL信号アサート設定(SSL2を使う)
		preprocess_spi_enc(0xFFFF);	//Read Angle
		enc_data_l = Get_enc_data();
		
		//左右エンコーダから角度取得
		//4096で一回転(360deg = 0deg)
		locate_r = enc_data_r;
		locate_l = enc_data_l;
		
		//右エンコーダの現在の位置と,1msec前の位置との差分を計算
		//単位時間（1msec）あたりの変位量を計算
		diff_pulse_r = (locate_r - before_locate_r);
		//変化点を1023から0//へ移動したときの補正
		if((diff_pulse_r > ENC_RES_HALF || diff_pulse_r < -ENC_RES_HALF) && before_locate_r >ENC_RES_HALF){
			diff_pulse_r = (((ENC_RES_MAX - 1) - before_locate_r) + locate_r);
		}
		//変化点を0から1023へ移動したときの補正
		else if((diff_pulse_r > ENC_RES_HALF || diff_pulse_r < -ENC_RES_HALF) && before_locate_r <=ENC_RES_HALF){
			diff_pulse_r = 1*(before_locate_r + ((ENC_RES_MAX - 1) - locate_r));
		}
		
		//左エンコーダの現在の位置と,1msec前の位置との差分を計算
		//単位時間（1msec）あたりの変位量を計算
		diff_pulse_l = (-locate_l + before_locate_l);
		//変化点を1023から0//へ移動したときの補正
		if((diff_pulse_l > ENC_RES_HALF || diff_pulse_l < -ENC_RES_HALF) && before_locate_l >ENC_RES_HALF){
			diff_pulse_l = 1*(((ENC_RES_MAX - 1) - before_locate_l) + locate_l);
		}
		//変化点を0から1023へ移動したときの補正
		else if((diff_pulse_l > ENC_RES_HALF || diff_pulse_l < -ENC_RES_HALF) && before_locate_l <=ENC_RES_HALF){
			diff_pulse_l = (before_locate_l + ((ENC_RES_MAX - 1) - locate_l));
		}
				
		//現在速度を算出
		speed_new_r = (float)((float)diff_pulse_r * (float)MMPP);
		speed_new_l = (float)((float)diff_pulse_l * (float)MMPP);
		
		//過去の値を保存
		speed_old_r= speed_r;
		speed_old_l= speed_l;
		
		//速度のローパスフィルタ
		speed_r = speed_new_r * 0.1 + speed_old_r * 0.9;
		speed_l = speed_new_l * 0.1 + speed_old_l * 0.9;
		
		p_speed = speed;
		//車体速度を計算
		speed = ((speed_r + speed_l)/2.0);
		
		//I成分のオーバーフローとアンダーフロー対策
		I_speed += speed;
		if(I_speed >30*10000000000){
			I_speed = 30*10000000000;
		}else if(I_speed < -1*10000000000){
			I_speed = -1*10000000000;
		}


		
		//距離の計算
		len_mouse += (speed_new_r + speed_new_l)/2.0;
		
		//過去の値を保存
		before_locate_r = locate_r;
		before_locate_l = locate_l;
		
		
				
		state = 0;
		
	}
	
	
	/*****************************************************************************************
	ジャイロ関連(ヨー軸)
		値の取得　角度の積分　
	*****************************************************************************************/
	if(state == 1){
		//ジャイロセンサの値の更新
		preprocess_spi_gyro(0xB70000);
		
		//LowPass Filter
		gyro_x_new = (float)((short)(read_gyro_data() & 0x0000FFFF));
		gyro_x = (gyro_x_new -gyro_ref );
		
		//角速度の更新
		p_ang_vel = ang_vel;
		ang_vel = ((2000.0*(gyro_x)/32767.0))*PI/180.0;
		//積分値の更新
		I_ang_vel += ang_vel;
		if(I_ang_vel >30*10000000000){
			I_ang_vel = 30*10000000000;
		}else if(I_ang_vel < -1*10000000000){
			I_ang_vel = -1*10000000000;
		}
		
		//ジャイロの値を角度に変換
		degree += (2.0*(gyro_x_new - gyro_ref)/32767.0);
		
	}	
	
}


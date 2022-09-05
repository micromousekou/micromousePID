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


volatile int Logcounter;
volatile long	log2[1000];
volatile long	log3[1000];
volatile long	log4[1000];
volatile long	log5[1000];


	/*****************************************************************************************
	��`����
		
	*****************************************************************************************/
float daikei(float V_max, float S, float a)
{
	int t=0;
	t=t+timer;
	S=V_max*t;
	


	return 0;
}

void int_cmt0(void)
{	
	/*****************************************************************************************
	PID����
		
	*****************************************************************************************/
	//�e�p�����[�^�ݒ�---------------------------------------------------------------------------------
	if (PID_ON==1){
		float  trim_R, trim_L, targetV, Perr_R, Perr_L, Ierr_L, Ierr_R, Derr_L, Derr_R; //�֐��ݒ�
		
		Perr_R = Perr_L = 0; //�덷
		
		MSP = 239; //�ő僂�[�^�o��
		
		//MOT_CWCCW_R = MOT_R_FORWARD; //�E���[�^��]�����ݒ�
		//MOT_CWCCW_L = MOT_L_FORWARD; //�����[�^��]�����ݒ�
							
		//�ύX����K�v�̂���p�����[�^�����[�^-----------------------
		KP_L = 0.02;//	0.02; 	//���Q�C�����ݒ�
		KI_L = 0.0008/1000;	//�ϕ��Q�C�����ݒ�
		KD_L = 0;//1*1000; 	//�����Q�C�����ݒ�
		trim_L = 0.4;//0.379; 	//�����[�^�g�����ݒ�
		
		Perr_L = Ierr_L = Derr_L = 0;
		//�ύX����K�v�̂���p�����[�^�E���[�^-----------------------
		KP_R = 0.02;//0.01; 	//���Q�C���E�ݒ�
		KI_R = 0.0008/1000;	//�ϕ��Q�C���E�ݒ�
		KD_R = 0;//1*1000; 	//�����Q�C���E�ݒ�
		trim_R = 0.4;//0.379;	//�E���[�^�g�����ݒ�
		
		
		Perr_R = Ierr_R = Derr_R = 0;
		targetV = 3000;		 //�ڕW�l[rpm]
		
		
		
		// ---------------------------------------------------------------------------------------------------
		
		rpm_l = (speed_l*1000*60/(TIRE_DIAMETER*PI));//�P�ʂ�[rpm]
		rpm_r = (speed_r*1000*60/(TIRE_DIAMETER*PI));
		
			
		//PID�����p�����v�Z----------------------------------------
											
			
		//���v�f-----(�ڕW�l - �o��)
		Perr_L = targetV-rpm_l;
		Perr_R = targetV+rpm_r;
		
		//�ϕ��v�f-----(���܂ł̃G���[ + ���݂̃G���[)
		Ierr_L = Ierr_L+Perr_L;
		Ierr_R = Ierr_R+Perr_R;
		
		//�����v�f-----(���݂̃G���[ - �ЂƂO�̃G���[)
		Derr_L = Perr_L-Derr_L;
		Derr_R = Perr_R-Derr_R;
		
		
		//���~�b�^�[�ݒ�---------------------------------------------
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
		
		MOT_OUT_L = MSP*trim_L+KP_L*Perr_L+KI_L*Ierr_L+KD_L*Derr_L; 					
		MOT_OUT_R = MSP*trim_R+KP_R*Perr_R+KI_R*Ierr_R+KD_R*Derr_R;
							
		
		log2[Logcounter] = 1000;//rpm_l;
		log3[Logcounter] = 2000;//MOT_OUT_L;
		log4[Logcounter] = 3000;//rpm_r;
		log5[Logcounter] = 4000;//MOT_OUT_R;
		
		Logcounter++;

			}
	
	else{
	Logcounter=0;
	}
	
	/*****************************************************************************************
	�^�C�}�̃J�E���g
		
	*****************************************************************************************/
	timer++;
	cnt++;
	
	}
		

	
	


void int_cmt1(void)		//�Z���T�ǂݍ��ݗp�荞��
{
	/*****************************************************************************************
	A/D�ϊ�
		�Z���T�ƃo�b�e���[�d���擾
	*****************************************************************************************/
	static int state = 0;	//�ǂݍ��ރZ���T�̃��[�e�[�V�����Ǘ��p�ϐ�
	int i;
	//long log_cnt;
	
	switch(state)
	{
		case 0:		//�E�Z���T�ǂݍ���

			//�����t�B���^
			S12AD.ADANS0.BIT.ANS0=0x0004;			//AN002
			S12AD.ADCSR.BIT.ADST=1;				//AD�ϊ��J�n
			while(S12AD.ADCSR.BIT.ADST);			//AD�ϊ��I���܂ő҂�
			sen_r.d_value = S12AD.ADDR2;			//���l��ۑ�
			
			SLED_R = 1;					//LED�_��
			for(i = 0; i < WAITLOOP_SLED; i++);		//�t�H�g�g�����W�X�^�̉����҂����[�v
			S12AD.ADANS0.BIT.ANS0=0x0004;			//AN002
			S12AD.ADCSR.BIT.ADST=1;				//AD�ϊ��J�n
			while(S12AD.ADCSR.BIT.ADST);			//AD�ϊ��I���܂ő҂�
			SLED_R = 0;					//LED����
			
			sen_r.value = (S12AD.ADDR2 - sen_r.d_value);	//�l��ۑ�
			
			break;


		case 1:		//�O���Z���T�ǂݍ���

			//�����t�B���^
			S12AD.ADANS0.BIT.ANS0=0x0001;			//AN000
			S12AD.ADCSR.BIT.ADST=1;				//AD�ϊ��J�n
			while(S12AD.ADCSR.BIT.ADST);			//AD�ϊ��I���܂ő҂�
			sen_fl.d_value = S12AD.ADDR0;			//�l��ۑ�
		
			SLED_FL = 1;					//LED�_��
			for(i = 0; i < WAITLOOP_SLED; i++);		//�t�H�g�g�����W�X�^�̉����҂����[�v
			S12AD.ADANS0.BIT.ANS0=0x0001;			//AN000
			S12AD.ADCSR.BIT.ADST=1;				//AD�ϊ��J�n
			while(S12AD.ADCSR.BIT.ADST);			//AD�ϊ��I���܂ő҂�
			SLED_FL = 0;					//LED����

			sen_fl.value = (S12AD.ADDR0 - sen_fl.d_value);	//�l��ۑ�

			break;


		case 2:		//�O�E�Z���T�ǂݍ���
		
			//�����t�B���^
			S12AD.ADANS0.BIT.ANS0=0x0040;			//AN006
			S12AD.ADCSR.BIT.ADST=1;				//AD�ϊ��J�n
			while(S12AD.ADCSR.BIT.ADST);			//AD�ϊ��I���܂ő҂�
			sen_fr.d_value = S12AD.ADDR6;			//�l��ۑ�
		
			SLED_FR = 1;					//LED�_��
			for(i = 0; i < WAITLOOP_SLED; i++);		//�t�H�g�g�����W�X�^�̉����҂����[�v
			S12AD.ADANS0.BIT.ANS0=0x0040;			//AN006
			S12AD.ADCSR.BIT.ADST=1;				//AD�ϊ��J�n
			while(S12AD.ADCSR.BIT.ADST);			//AD�ϊ��I���܂ő҂�
			SLED_FR = 0;					//LED����
			
			sen_fr.value = (S12AD.ADDR6 - sen_fr.d_value);	//�l��ۑ�
			
			break;


		case 3:		//���Z���T�ǂݍ���
		
			//�����t�B���^
			S12AD.ADANS0.BIT.ANS0=0x0002;			//AN001
			S12AD.ADCSR.BIT.ADST=1;				//AD�ϊ��J�n
			while(S12AD.ADCSR.BIT.ADST);			//AD�ϊ��I���܂ő҂�
			sen_l.d_value = S12AD.ADDR1;			//�l��ۑ�
			
			SLED_L = 1;					//LED�_��
			for(i = 0; i < WAITLOOP_SLED; i++)	;	//�t�H�g�g�����W�X�^�̉����҂����[�v
			S12AD.ADANS0.BIT.ANS0=0x0002;			//AN001
			S12AD.ADCSR.BIT.ADST=1;				//AD�ϊ��J�n
			while(S12AD.ADCSR.BIT.ADST);			//AD�ϊ��I���܂ő҂�
			SLED_L = 0;					//LED����
			
			sen_l.value = (S12AD.ADDR1 - sen_l.d_value);	//�l��ۑ�
			
			break;
	}
	
	state++;		//4�񂲂ƂɌJ��Ԃ�
	if(state > 3)
	{
		state = 0;
	}
	
	S12AD.ADANS0.BIT.ANS0=0x0200;			//AN009
	S12AD.ADCSR.BIT.ADST=1;				//AD�ϊ��J�n
	while(S12AD.ADCSR.BIT.ADST);			//AD�ϊ��I���܂ő҂�
	V_bat = (2.0*3.3*(float)(S12AD.ADDR9/4095.0) );
	if(V_bat < 3.5){
		
		//���[�^�~�߂�
		Duty_r = 0;
		Duty_l = 0;
		MOT_POWER_OFF;	//PC6(SLEEP�s��)
		
		//�u�U�[�炵������
		while(1){
			BEEP();
		}
	}
	/*****************************************************************************************
	1kHz���ƂɃ��O���擾
		
	*****************************************************************************************/
	/*
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
	*/
	log_timer++;
		
}

void int_cmt2(void)
{
	static unsigned int	enc_data_r;	//�G���R�[�_�̐��f�[�^
	static unsigned int	enc_data_l;	//�G���R�[�_�̐��f�[�^ 
	static short	state;
	/*****************************************************************************************
	�G���R�[�_�֘A
		�l�̎擾�@���x�X�V�@�����ϕ��Ȃ�
	*****************************************************************************************/	
	if(state == 0){
		RSPI0.SPCMD0.BIT.SSLA = 	0x00;	//SSL�M���A�T�[�g�ݒ�(SSL0���g��)
		preprocess_spi_enc(0xFFFF);	//Read Angle
		enc_data_r = Get_enc_data();
		state = 1;
	}else{
		RSPI0.SPCMD0.BIT.SSLA = 	0x02;	//SSL�M���A�T�[�g�ݒ�(SSL2���g��)
		preprocess_spi_enc(0xFFFF);	//Read Angle
		enc_data_l = Get_enc_data();
		
		//���E�G���R�[�_����p�x�擾
		//4096�ň��](360deg = 0deg)
		locate_r = enc_data_r;
		locate_l = enc_data_l;
		
		//�E�G���R�[�_�̌��݂̈ʒu��,1msec�O�̈ʒu�Ƃ̍������v�Z
		//�P�ʎ��ԁi1msec�j������̕ψʗʂ��v�Z
		diff_pulse_r = (locate_r - before_locate_r);
		//�ω��_��1023����0//�ֈړ������Ƃ��̕␳
		if((diff_pulse_r > ENC_RES_HALF || diff_pulse_r < -ENC_RES_HALF) && before_locate_r >ENC_RES_HALF){
			diff_pulse_r = (((ENC_RES_MAX - 1) - before_locate_r) + locate_r);
		}
		//�ω��_��0����1023�ֈړ������Ƃ��̕␳
		else if((diff_pulse_r > ENC_RES_HALF || diff_pulse_r < -ENC_RES_HALF) && before_locate_r <=ENC_RES_HALF){
			diff_pulse_r = 1*(before_locate_r + ((ENC_RES_MAX - 1) - locate_r));
		}
		
		//���G���R�[�_�̌��݂̈ʒu��,1msec�O�̈ʒu�Ƃ̍������v�Z
		//�P�ʎ��ԁi1msec�j������̕ψʗʂ��v�Z
		diff_pulse_l = (-locate_l + before_locate_l);
		//�ω��_��1023����0//�ֈړ������Ƃ��̕␳
		if((diff_pulse_l > ENC_RES_HALF || diff_pulse_l < -ENC_RES_HALF) && before_locate_l >ENC_RES_HALF){
			diff_pulse_l = 1*(((ENC_RES_MAX - 1) - before_locate_l) + locate_l);
		}
		//�ω��_��0����1023�ֈړ������Ƃ��̕␳
		else if((diff_pulse_l > ENC_RES_HALF || diff_pulse_l < -ENC_RES_HALF) && before_locate_l <=ENC_RES_HALF){
			diff_pulse_l = (before_locate_l + ((ENC_RES_MAX - 1) - locate_l));
		}
				
		//���ݑ��x���Z�o
		speed_new_r = (float)((float)diff_pulse_r * (float)MMPP);
		speed_new_l = (float)((float)diff_pulse_l * (float)MMPP);
		
		//�ߋ��̒l��ۑ�
		speed_old_r= speed_r;
		speed_old_l= speed_l;
		
		//���x�̃��[�p�X�t�B���^
		speed_r = speed_new_r * 0.1 + speed_old_r * 0.9;
		speed_l = speed_new_l * 0.1 + speed_old_l * 0.9;
		
		p_speed = speed;
		//�ԑ̑��x���v�Z
		speed = ((speed_r + speed_l)/2.0);
		
		//I�����̃I�[�o�[�t���[�ƃA���_�[�t���[�΍�
		I_speed += speed;
		if(I_speed >30*10000000000){
			I_speed = 30*10000000000;
		}else if(I_speed < -1*10000000000){
			I_speed = -1*10000000000;
		}


		
		//�����̌v�Z
		len_mouse += (speed_new_r + speed_new_l)/2.0;
		
		//�ߋ��̒l��ۑ�
		before_locate_r = locate_r;
		before_locate_l = locate_l;
		
		
				
		state = 0;
		
	}
	
	
	/*****************************************************************************************
	�W���C���֘A(���[��)
		�l�̎擾�@�p�x�̐ϕ��@
	*****************************************************************************************/
	if(state == 1){
		//�W���C���Z���T�̒l�̍X�V
		preprocess_spi_gyro(0xB70000);
		
		//LowPass Filter
		gyro_x_new = (float)((short)(read_gyro_data() & 0x0000FFFF));
		gyro_x = (gyro_x_new -gyro_ref );
		
		//�p���x�̍X�V
		p_ang_vel = ang_vel;
		ang_vel = ((2000.0*(gyro_x)/32767.0))*PI/180.0;
		//�ϕ��l�̍X�V
		I_ang_vel += ang_vel;
		if(I_ang_vel >30*10000000000){
			I_ang_vel = 30*10000000000;
		}else if(I_ang_vel < -1*10000000000){
			I_ang_vel = -1*10000000000;
		}
		
		//�W���C���̒l���p�x�ɕϊ�
		degree += (2.0*(gyro_x_new - gyro_ref)/32767.0);
		
	}	
	
}


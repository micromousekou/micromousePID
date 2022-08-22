/***********************************************************************/
/*                                                                     */
/*  FILE        :Main.c or Main.cpp                                    */
/*  DATE        :2017/6/27	                                       */
/*  DESCRIPTION :Main Program                                          */
/*  CPU TYPE    :RX631 48P                                             */
/*                                                                     */
/*  NOTE:THIS IS A TYPICAL EXAMPLE.                                    */
/*                                                                     */
/***********************************************************************/
//#include "typedefine.h"
#include "iodefine.h"
#include "mathf.h"
#include "sci.h"
#include "init.h"
#include "spi.h"
#include "i2c.h"
#include "parameters.h"
#include "glob_var.h"
#include "run.h"
#include "interface.h"
#include "DataFlash.h"
#include "portdef.h"
#include "fast.h"
#include "search.h"

#ifdef __cplusplus
//#include <ios>                        // Remove the comment when you use ios
//_SINT ios_base::Init::init_cnt;       // Remove the comment when you use ios
#endif

void main(void);
#ifdef __cplusplus
extern "C" {
void abort(void);
}
#endif


extern void wait_ms(int wtime);
extern void adjust(void);

void main(void)
{

	init_all();
	unsigned long i = 0;
	

	
	//�u�U�[
	BEEP();
	//�ŏ���0���Ă���
	speed_r=0;
	speed_l=0;
	
	//�N�����̃��O�͂Ƃ�Ȃ�
	log_flag = 0;
	short mode = 1;
	while(1){
		I_tar_ang_vel = 0;
		I_ang_vel = 0;
		I_tar_speed = 0;
		I_speed = 0;

		switch(mode){
			
			case 1:
				/****************************************
				*MODE LED STATE				*
				*					*
				*	D3	D4	D5	D6	*
				*	O	X	X	X	*
				*					*
				*****************************************/
				//�Z���T�[�̑O�Ɏ���������ăX�^�[�g
				if(sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4){
					BEEP();
					//�ǐ����L���ɂ���
					con_wall.enable = true;
					while(1){
						//A/D sensor
						SCI_printf("sen_r.value: %d\n\r",sen_r.value);
						SCI_printf("sen_l.value: %d\n\r",sen_l.value);
						SCI_printf("sen_fr.value: %d\n\r",sen_fr.value);
						SCI_printf("sen_fl.value: %d\n\r",sen_fl.value);
						SCI_printf("V_bat: %d\n\r",(int)(V_bat*1000));
						SCI_printf("sen_r.th_wall: %d\n\r",sen_r.th_wall);
						SCI_printf("sen_l.th_wall: %d\n\r",sen_l.th_wall);
						SCI_printf("sen_fr.th_wall: %d\n\r",sen_fr.th_wall);
						SCI_printf("sen_fl.th_wall: %d\n\r",sen_fl.th_wall);
						SCI_printf("con_wall.omega: %d\n\r",(int)(con_wall.omega*1000));
						SCI_printf("speed_r: %d\n\r", (int)(speed_r*100));
						SCI_printf("speed_l: %d\n\r", (int)(speed_l*100));
						//gyro
						SCI_printf("degree: %d\n\r",(int)degree*10);;			
						SCI_printf("gyro: %d\n\r", (int)(ang_vel*1000) );
						//encoder
						SCI_printf("locate_r: %d\n\r",locate_r);
						SCI_printf("locate_l: %d\n\r",locate_l);	
					
						//switch
						SCI_printf("switchC: %d\n\r",SW_C);
						SCI_printf("switchU: %d\n\r",SW_U);
						SCI_printf("switchD: %d\n\r",SW_D);
						wait_ms(100);
						//��ʃN���A�V�[�P���X
						SCI_printf("\x1b[2J");				//�N���A�X�N���[��[CLS]
						SCI_printf("\x1b[0;0H");			//�J�[�\����0,0�Ɉړ�
						
						//�v�b�V���X�C�b�`�p����
						push_switch = IOex_SWITCH();
			
						if(SW_C == 1){
							BEEP();
							break;	
						}
					}
				}
				
				
				
				break;
				
			case 2:
				/****************************************
				*MODE LED STATE				*
				*					*
				*	D3	D4	D5	D6	*
				*	X	O	X	X	*
				*					*
				*****************************************/
				
				/****************************************
				*      ���[�^�f�[�^�擾�p�v���O����     *
				*****************************************/
				
				
				//�Z���T�[�̑O�Ɏ���������ăX�^�[�g
				if(sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4){
					BEEP();
					
					
					//�d���m�F-------------------------------------
					int V;
					V=S12AD.ADDR9*3.3*2/4.096;
					SCI_printf("V = %d[mV]\n\r",V);
					
					//---------------------------------------------
				
					//���[�^�̑��x(�o��)�ݒ�
					int motorsp;
					motorsp=239;//���[�^���x

					MOT_OUT_R=motorsp*0.5; //�ő�l��239  0.55
					MOT_OUT_L=motorsp*0.5; //�ő�l��239  0.4
					
					//MOT_CWCCW_R = MOT_R_FORWARD;
					
					timer=0; //timer���Z�b�g
					
					//���[�^���x�v��---------------------------------------
					for (timer=0; timer<1000;timer){
					
						if(timer<101){    //100ms�ҋ@����
						
							while(timer<100);
						log[12][timer] = (short)(speed_l*1000*60/(TIRE_DIAMETER*PI));//�P�ʂ�[rpm]
						log[13][timer] = (short)(speed_r*1000*60/(TIRE_DIAMETER*PI));
						}
						
						MOT_POWER_ON; //���[�^��]�J�n����]���̃f�[�^�擾�J�n
						log[12][timer] = (short)(speed_l*1000*60/(TIRE_DIAMETER*PI));//�P�ʂ�[rpm]
						log[13][timer] = (short)(speed_r*1000*60/(TIRE_DIAMETER*PI));
					}
					//------------------------------------------------------
					
					//���[�^��~
					MOT_OUT_R=0; 
					MOT_OUT_L=0; 
					MOT_POWER_OFF;
					timer=0;
					while(timer<1000);
					BEEP();
					break;
					
				}
				break;
				
			case 3:
				/****************************************
				*MODE LED STATE				*
				*					*
				*	D3	D4	D5	D6	*
				*	O	O	X	X	*
				*					*
				*****************************************/
				
				/****************************************
				*      ���[�^�f�[�^�o�͗p�v���O����     *
				*****************************************/
				
				
				//�Z���T�[�̑O�Ɏ���������ăX�^�[�g
				if(sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4){
					BEEP();
					SCI_printf("time[msec],speed_l[m/s],speed_r[m/s]\n\r");
					
					for(i = 0; i <1000; i++){
						
						SCI_printf("%d,",i);              //time[msec]
						SCI_printf("%d,",log[12][i]);     //speed_L/100[m/s]
						SCI_printf("%d,",log[13][i]);
						SCI_printf("%d,",log[3][i]);
						SCI_printf("%d\n\r",log[4][i]);  //speed_R/100[m/s]
			
					}
					
				break;
				}
				
			case 4:
				/****************************************
				*MODE LED STATE				*
				*					*
				*	D3	D4	D5	D6	*
				*	X	X	O	X	*
				*					*
				*****************************************/
				
				//�Z���T�[�̑O�Ɏ���������ăX�^�[�g
				if(sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4){
					
					//�ǐ����L���ɂ���
					con_wall.enable = true;
					
					timer=0;
					while(timer<3000){
					BEEP();}
					SCI_printf("timer=0:%d\n\r",timer);
				
				}
				
				break;
				
			case 5:
				/****************************************
				*MODE LED STATE				*
				*					*
				*	D3	D4	D5	D6	*
				*	O	X	O	X	*
				*					*
				*****************************************/
				
				
				/*****************************************************
				*    ���[�^��]���擾�p�v���O����(��ڐG�v��p����)  *
				*****************************************************/
				
				
				//�Z���T�[�̑O�Ɏ���������ăX�^�[�g
				if(sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4){
					BEEP();
					
					
					//�d���m�F-------------------------------------
					int V;
					V=S12AD.ADDR9*3.3*2/4.096;
					SCI_printf("V = %d[mV]\n\r",V);
					
					//---------------------------------------------
				
					//���[�^�̑��x(�o��)�ݒ�
					int motorsp;
					motorsp=239;//���[�^���x

					MOT_OUT_R=motorsp*0.55; //�ő�l��239
					MOT_OUT_L=motorsp*0.55; //�ő�l��239
					
					MOT_CWCCW_R = MOT_R_FORWARD;
					
					timer=0; //timer���Z�b�g
					
					
					
					MOT_POWER_ON; //���[�^��]�J�n����]���̃f�[�^�擾�J�n�i��ڐG�v�j
					while(timer<2000);//��]�J�n��2�b�ԑҋ@
					timer=0;
					i=0;
					//���[�^���x�v��---------------------------------------
					while(1){//n�b�ԉ�]
						
						short SL = (short)(speed_l*1000*60/(TIRE_DIAMETER*PI));//�P�ʂ�[rpm]
						short SR = (short)(speed_r*1000*60/(TIRE_DIAMETER*PI));
						
						while(timer<10000);//5�b�����ɑ���
						BEEP();
						//SCI_printf("L[rpm]=%d   :",SL);
						SCI_printf("%d,",i);
						SCI_printf("%d\n\r",SL);
						timer=0;
						i++;
						
					}
					
					//���[�^��~
					MOT_OUT_R=0; 
					MOT_OUT_L=0; 
					MOT_POWER_OFF;
					timer=0;
					while(timer<1000);
					BEEP();
				}
				break;
						
				
			case 6:
				/****************************************
				*MODE LED STATE				*
				*					*
				*	D3	D4	D5	D6	*
				*	X	O	O	X	*
				*					*
				*****************************************/
			
				/*************************
				*   ��ᐧ��v���O����   *
				*************************/
				//------------------------------------------------------
				/*memo
					
				------------------------------------------------------*/
				/******************************************
				�u���b�N�}
				
				  ref  +         duty
				------>��--->[Kp]--->[Motor]---��------>��
				     - ��			|
				       |			|
				       --------------------------
				*******************************************/
				
				//�Z���T�[�̑O�Ɏ���������ăX�^�[�g
				if(sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4){
					BEEP();
					
					//�e�p�����[�^�ݒ�---------------------------------------------------------------------------------
					float output_R, output_L, trim_R, trim_L, targetV, Perr_R, Perr_L, Ierr_L, Ierr_R, Derr_L, Derr_R; //�֐��ݒ�
					short NR , NL;
					Perr_R = Perr_L = 0; //�덷
					//output_L = speed_l*100; //�o�͒l��
					//output_R = speed_r*100; //�o�͒l�E
					MSP = 239; //�ő僂�[�^�o��
					
					//MOT_CWCCW_R = MOT_R_FORWARD; //�E���[�^��]�����ݒ�
					//MOT_CWCCW_L = MOT_L_FORWARD; //�����[�^��]�����ݒ�
										
					//�ύX����K�v�̂���p�����[�^�����[�^-----------------------
					KP_L = 0.02;//	0.02; 	//���Q�C�����ݒ�
					KI_L = 0;//0.0008/1000;//1/1000; 	//�ϕ��Q�C�����ݒ�
					KD_L = 0;//1*1000; 	//�����Q�C�����ݒ�
					trim_L = 0.4;//0.379; 	//�����[�^�g�����ݒ�
					
					Perr_L = Ierr_L = Derr_L = 0;
					//�ύX����K�v�̂���p�����[�^�E���[�^-----------------------
					KP_R = 0.02;//0.01; 	//���Q�C���E�ݒ�
					KI_R = 0;//0.0008/1000;// 1/1000; 	//�ϕ��Q�C���E�ݒ�
					KD_R = 0;//1*1000; 	//�����Q�C���E�ݒ�
					trim_R = 0.4;//0.379; 	//�E���[�^�g�����ݒ�
					
					
					Perr_R = Ierr_R = Derr_R = 0;
					targetV = 3000;//2000; //�ڕW�l[rpm]
					
					// ---------------------------------------------------------------------------------------------------
					
					//���[�^����---------------------------------------
					for (timer=0; timer<1000;timer){
					
						if(timer<101){    //100ms�ҋ@����
						
							while(timer<100);
							
							NL = (short)(speed_l*1000*60/(TIRE_DIAMETER*PI));//�P�ʂ�[rpm]
							NR = (short)(speed_r*1000*60/(TIRE_DIAMETER*PI));
							
							log[12][timer] = NL;
							log[13][timer] = MOT_OUT_L;
							log[3][timer] = NR;
							log[4][timer] = MOT_OUT_R;
							
								}
						
						MOT_POWER_ON; //���[�^��]�J�n����]���̃f�[�^�擾�J�n
														
						output_L = NL;
						output_R = NR;
						
						//���v�f-----(�ڕW�l - �o��)
						Perr_L = targetV-output_L;
						Perr_R = -1*targetV-output_R;
						
						//�ϕ��v�f-----(���܂ł̃G���[ + ���݂̃G���[)
						Ierr_L = Ierr_L+Perr_L;
						Ierr_R = Ierr_R+Perr_R;
						
						//�����v�f-----(���݂̃G���[ - �ЂƂO�̃G���[)
						Derr_L = Perr_L-Derr_L;
						Derr_R = Perr_R-Derr_R;
						
						//PID�����p�����v�Z----------------------------------------
						MOT_OUT_L = MSP*trim_L+KP_L*Perr_L+KI_L*Ierr_L+KD_L*Derr_L; 					
						MOT_OUT_R = MSP*trim_R+KP_R*Perr_R+KI_R*Ierr_R+KD_R*Derr_R;
						
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
						
						NL = (short)(speed_l*1000*60/(TIRE_DIAMETER*PI));//�P�ʂ�[rpm]
						NR = (short)(speed_r*1000*60/(TIRE_DIAMETER*PI));
						
						log[12][timer] = NL;
						log[13][timer] = MOT_OUT_L;
						log[3][timer] = NR;
						log[4][timer] = MOT_OUT_R;	
					}
					
					//���[�^��~
					MOT_OUT_R=0; 
					MOT_OUT_L=0; 
					MOT_POWER_OFF;
					timer=0;
					while(timer<1000);
					BEEP();
					
				}
				break;
				
			case 7:
				/****************************************
				*MODE LED STATE				*
				*					*
				*	D3	D4	D5	D6	*
				*	O	O	O	X	*
				*					*
				*****************************************/
				
				//�Z���T�[�̑O�Ɏ���������ăX�^�[�g
				if(sen_fr.value + sen_fl.value + sen_r.value + sen_l.value > SEN_DECISION * 4){
					BEEP();
					
				
				
					}
				
				
				break;
				
			case 8:
				/****************************************
				*MODE LED STATE				*
				*					*
				*	D3	D4	D5	D6	*
				*	X	X	X	O	*
				*					*
				*****************************************/
			
				
				
				break;
				
			case 9:
				/****************************************
				*MODE LED STATE				*
				*					*
				*	D3	D4	D5	D6	*
				*	O	X	X	O	*
				*					*
				*****************************************/
			
				
				
				break;
				
			case 10:
				/****************************************
				*MODE LED STATE				*
				*					*
				*	D3	D4	D5	D6	*
				*	X	O	X	O	*
				*					*
				*****************************************/
			

				break;
				
			case 11:
				/****************************************
				*MODE LED STATE				*
				*					*
				*	D3	D4	D5	D6	*
				*	O	O	X	O	*
				*					*
				*****************************************/
			
				
				break;
				
			case 12:
				/****************************************
				*MODE LED STATE				*
				*					*
				*	D3	D4	D5	D6	*
				*	X	X	O	O	*
				*					*
				*****************************************/
			
				
				
				break;
				
				
			case 13:
				/****************************************
				*MODE LED STATE				*
				*					*
				*	D3	D4	D5	D6	*
				*	O	X	O	O	*
				*					*
				*****************************************/
			
				
				
				break;
				
			case 14:
				/****************************************
				*MODE LED STATE				*
				*					*
				*	D3	D4	D5	D6	*
				*	X	O	O	O	*
				*					*
				*****************************************/
			
				
				
				break;
				
			case 15:
				/****************************************
				*MODE LED STATE				*
				*					*
				*	D3	D4	D5	D6	*
				*	O	O	O	O	*
				*					*
				*****************************************/
				
				break;
				
			//mode0~15�ȊO�̏ꍇ�B�������Ȃ��B
			default:
				break;
			
		}
		
		//���[�h�؂�ւ��p����
		if(speed > 0.1){
			if(mode == 15){
				mode = 1;
			}else{
				mode ++;
			}
			for(i = 0; i < 100*1000*10; i++);
			BEEP();
		}
		
		if(speed < -0.1){
			if(mode == 1){
				mode = 15;
			}else{
				mode --;
		}
			for(i = 0; i < 100*1000*10; i++);
			BEEP(); 
		}
		LED(mode);
		
		//�v�b�V���X�C�b�`�p����
		push_switch = IOex_SWITCH();
		MOT_POWER_OFF;
	
	}
	

}

#ifdef __cplusplus
void abort(void)
{

}
#endif

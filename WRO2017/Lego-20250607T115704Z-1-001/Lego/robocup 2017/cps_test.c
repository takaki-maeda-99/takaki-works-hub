#pragma config(Sensor, S1,     Lsensor,        sensorI2CCustom)
#pragma config(Sensor, S2,     Rsensor,        sensorI2CCustom)
#pragma config(Sensor, S3,		 TIR,						 sensorEV3_GenericI2C)
#pragma config(Motor,  motorA,          Lmotor,        tmotorEV3_Large, PIDControl, encoder)
#pragma config(Motor,  motorB,          Rmotor,        tmotorEV3_Large, PIDControl, encoder)
#pragma config(Motor,  motorC,          Arm,           tmotorEV3_Medium, PIDControl, encoder)
#pragma config(Motor,  motorD,          Gate,          tmotorEV3_Medium, PIDControl, encoder)


#include "drivers\S_UMX.h"
#include "drivers/ultrasonic_UMX.h"
#include "library.h"


#define smux_port S3
#define smux_address 0x08
#define SIKII 65
#define L 1
#define R 2

task main()
{
	moveMotorTarget(Arm,200,100);	sleep(1000);
	margin=SETmargin();
	setup_ultrasonic_mux(S4 ,0X04,3);
	setup_smux(smux_port,smux_address);
	led_con(LED_on);
	cps_level(1);
	RST();
	//while(1) trace();
	//haikai();
	opentest();
	while(1!=getButtonPress(buttonDown)){
		if(digital_read(D0)==1)	syougai_butu(L);
		if(s_hitec(Lsensor) < SIKII && s_hitec(Rsensor) < SIKII && saka() == 0)	kousaten();
		if(saka()==1) Slope();
		if(cps_read(Ta1)){
			for(int i;i<50;i++)playImmediateTone(400,30);
			haikai();}
		if(analog_read(A3)>100) Gap(0);
		else Gap(1);
		trace();
	}
	led_con(LED_ten);
}

void trace(void){
	int Rs = s_hitec(Rsensor) ,Ls = s_hitec(Lsensor)+margin;
	float	sa = (Rs-Ls)*GEIN;
	motor[Lmotor]=sa-FPOWER;
	motor[Rmotor]=-sa-FPOWER;
	displayTextLine(1,"LSENSOR:%d",s_hitec(S1)+margin);
	displayTextLine(2,"RSENSOR:%d",s_hitec(S2));
}


void MV(int xp,int yp){
	motor[Lmotor]=yp;
	motor[Rmotor]=xp;
}


void syougai_butu(int RL){
	if(RL == 1){
		Ad(300,-60);
		Right_Spin(650,50);
		Ad(700,60);
		Left_Turn(1300,50);
		Ad(2000,60);
		Left_Spin(650,60);
		line_search();
		Rturn();
		}else if(RL == 2){
		Ad(300,-60);
		Left_Spin(650,50);
		Ad(700,60);
		Right_Turn(1300,50);
		Ad(2000,60);
		Right_Spin(650,60);
		line_search();
		Lturn();
	}
	playImmediateTone(400,30);
}

void Judgement(int msec){
	int whatB=0,a=0;
	MV(50,-50);
	for( a=0; a<40&&whatB==0; a++ ){ if(s_hitec(Lsensor)<SIKII ) whatB=1; sleep(10); }	RST();
	if(whatB==0){
		MV(50,-50);
		for( a=0; a<msec-40&&whatB==0; a++ ){ if(s_hitec(Rsensor)<SIKII ) whatB=1; sleep(10); }	RST();
		MV(-50,50);
		for( a=0; a<msec*2&&whatB==0; a++ ){ if(s_hitec(Lsensor)<SIKII ) whatB=1; sleep(10); }	RST();
	}
}

void kousaten(void){
	int pattern_no = 0,LmidoriF = 0,RmidoriF = 0, a=0 ;
	int speed=50,time=25;

	RST(); playImmediateTone(400,30); Ad(160,-50);

	MV(speed,-speed);
	for(a=0;a<time;a++){ if(HTCSreadColor(Lsensor)==4) LmidoriF=1; sleep(10); }	RST();
	if(LmidoriF==1) playImmediateTone(200,60);

	MV(-speed,speed);
	for(a=0;a<time*2;a++){ if(HTCSreadColor(Rsensor)==4) RmidoriF=1; sleep(10); }	RST();
	if(RmidoriF==1) playImmediateTone(200,60);

	MV(speed,-speed);
	for(a=0;a<time;a++){ if(HTCSreadColor(Lsensor)==4) LmidoriF=1; sleep(10); }	RST();
	if(LmidoriF==1) playImmediateTone(200,60);

	if(LmidoriF == 0 && RmidoriF == 0){ Ad(500,50); Judgement(60); pattern_no = 0;}
	else if(LmidoriF == 1 && RmidoriF == 0){Lturn(); pattern_no = 1;}
	else if(LmidoriF == 0 && RmidoriF == 1){Rturn(); pattern_no = 2;}
	else if(LmidoriF == 1 && RmidoriF == 1){Uturn(); pattern_no = 3;}
	displayTextLine(6,"GreenPattern_no:%d",pattern_no);
}


void Rturn(void){
	Ad(400,60);
	MV(40,-40);
	while(s_hitec(Rsensor)>SIKII); RST();
	Right_Spin(100,60);
}


void Lturn(void){
	Ad(400,60);
	MV(-40,40);
	while(s_hitec(Lsensor)>SIKII); RST();
	Left_Spin(100,60);
}


void Uturn(void){
	Right_Spin(400,60);
	while(s_hitec(Rsensor)>SIKII){
		motor[Lmotor]=-50;
		motor[Rmotor]=50;
	}
	Right_Spin(100,60);
}


void line_search(void){
	motor[Lmotor]=-50;
	motor[Rmotor]=-50;
	while((s_hitec(Lsensor)>SIKII)&&(s_hitec(Rsensor)>SIKII));

}


int saka(void){
	static int time = 0;
	int pattern = 0;
	if(digital_read(D4) == 1) time++;
	else time = 0;

	if(time > 30)
		pattern = 1;
	else
		pattern = 0;

	return(pattern);
}

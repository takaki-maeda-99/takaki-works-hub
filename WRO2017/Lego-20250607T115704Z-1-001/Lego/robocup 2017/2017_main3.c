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
#define SIKII 75
#define L 1
#define R 2

task main()
{
	moveMotorTarget(Arm,200,100);	sleep(1000);
	//margin=SETmargin();
	setup_ultrasonic_mux(S4 ,0X04,3);
	setup_smux(smux_port,smux_address);
	led_con(LED_on);
	cps_level(20);
	//opentest();
	//haikai();
	//opentest();
	while(1!=getButtonPress(buttonDown)){
		if(digital_read(D14)==1){ RST(); sleep(1000); if(digital_read(D14)==1) syougai_butu(R); }
		if(s_hitec(Lsensor) < SIKII && s_hitec(Rsensor) < SIKII && saka() == 0)	kousaten();
		/*if(digital_read(D5)||digital_read(D15)){
		if(digital_read(D5)||digital_read(D15)){
		resetMotorEncoder(Rmotor); resetMotorEncoder(Lmotor); FPOWER=20; playImmediateTone(1000,100); GEIN=0.2;
		while(getMotorEncoder(Rmotor)>-1000&&getMotorEncoder(Lmotor)>-700) trace();
		FPOWER=NPW; GEIN=NGEIN;
		}
		}*/
		//if(saka()) Slope();
		if(analog_read(A3)>100) Gap(0);
		else{ if(cps_read(Ta1)){ for(int i;i<5;i++) playImmediateTone(800,30);	haikai(); } Gap(1); }
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
	if(RL == 2){
		Ad(500,-100);
		Right_Spin(650,100);
		Ad(700,100);
		Left_Turn(1300,100);
		resetMotorEncoder(Rmotor);
		line_search();
		if(s_hitec(S1)<SIKII||s_hitec(S1)<SIKII){
			Ad(500,100);
			Right_Spin(650,100);}
		else {
			Left_Turn(1250,100);
			line_search();
			Rturn();
		}
		}else if(RL == 1){
		Ad(500,-100);
		Left_Spin(650,100);
		Ad(700,100);
		Right_Turn(1300,100);
		resetMotorEncoder(Rmotor);
		line_search();
		if(s_hitec(S1)<SIKII||s_hitec(S1)<SIKII){
			Ad(500,100);
			Left_Spin(650,100);}
		else {
			Right_Turn(1250,100);
			line_search();
			line_search();
			Lturn();
		}
	}
	playImmediateTone(400,30);
}

void Judgement(int msec){
	int whatB=0,a=0;

	MV(50,-50);
	for( a=0; a<10&&whatB==0; a++ ){ if(analog_read(A3)< 70 ) whatB=1; sleep(10); }	RST();
	MV(-50,50);
	for( a=0; a<20&&whatB==0; a++ ){ if(analog_read(A3)< 70 ) whatB=1; sleep(10); }	RST();
	MV(50,-50);
	for( a=0; a<10&&whatB==0; a++ ){ if(analog_read(A3)< 70 ) whatB=1; sleep(10); }	RST();

	if(whatB==0){
		MV(-50,50);
		for( a=0; a<msec*2&&whatB==0; a++ ){ if(s_hitec(Lsensor)<SIKII ) whatB=1; sleep(10); }	RST();
		MV(50,-50);
		for( a=0; a<msec&&whatB==0; a++ ){ if(s_hitec(Rsensor)<SIKII ) whatB=1; sleep(10); }	RST();
	}
}

void kousaten(void){
	int pattern_no = 0,LmidoriF = 0,RmidoriF = 0, a=0 ;
	int speed=70,time=17;

	RST(); playImmediateTone(400,30); Ad(160,-50);

	MV(speed,-speed);
	for(a=0;a<time;a++){ if(HTCSreadColor(Lsensor)==4) LmidoriF=1; sleep(10); }	RST();
	if(LmidoriF==1) playImmediateTone(200,60);

	MV(-speed,speed);
	for(a=0;a<time*2;a++){ if(HTCSreadColor(Rsensor)==4) RmidoriF=1; sleep(10); }	RST();
	if(RmidoriF==1) playImmediateTone(200,60);

	MV(speed,-speed);
	for(a=0;a<time;a++){ if(HTCSreadColor(Lsensor)==4) LmidoriF=1; sleep(10); }	RST();

	if(LmidoriF == 0 && RmidoriF == 0){ Ad(500,50); Judgement(60); pattern_no = 0;}
	else if(LmidoriF == 1 && RmidoriF == 0){Lturn(); pattern_no = 1;}
	else if(LmidoriF == 0 && RmidoriF == 1){Rturn(); pattern_no = 2;}
	else if(LmidoriF == 1 && RmidoriF == 1){Uturn(); pattern_no = 3;}
	displayTextLine(6,"GreenPattern_no:%d",pattern_no);
}


void Rturn(void){
	Ad(400,60);
	MV(40,-40);
	sleep(300);
	while(s_hitec(Rsensor)>SIKII); RST();
	Right_Spin(100,60);
}


void Lturn(void){
	Ad(400,60);
	MV(-40,40);
	sleep(300);
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
	resetMotorEncoder(Rmotor);
	motor[Lmotor]=-50;
	motor[Rmotor]=-50;
	while(s_hitec(Lsensor)>SIKII&&s_hitec(Rsensor)>SIKII&&analog_read(A3)>80/*&&analog_read(A1)>150&&analog_read(A2)>150*/
		&&getMotorEncoder(Rmotor)>-1400);
	RST();
	//if(analog_read(A2)<150){ Left_Turn(850,-100); Right_Turn(850,-100); }
	//if(analog_read(A1)<150){ Right_Turn(850,-100); Left_Turn(850,-100); }

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

#include "drivers\hitechnic-colour-v1.h"
#define Ad Advance
#define NGEIN 0.8
#define NPW 55

float GEIN=NGEIN;
int FPOWER=NPW;
int margin=3;

void opentest(void);
int s_hitec(tSensors port);
void syougai_butu(int RL);
void kousaten(void);
void trace(void);
void MV(int xp,int yp);
void StopF(int Power);
void Rturn(void);
void Lturn(void);
void Uturn(void);
void line_search(void);
void Judgement(int msec);
int saka(void);



void opentest(void){
	int sa;
	eraseDisplay();
	displayBigTextLine(0, "OpenTest");
	while(1!=getButtonPress(buttonEnter)){
		sa=s_hitec(S1)-s_hitec(S2);
		displayTextLine(3,"LSENSOR:%d",s_hitec(Lsensor));
		displayTextLine(4,"RSENSOR:%d",s_hitec(Rsensor));
		displayTextLine(5,"1:%d",digital_read(D1));
		displayTextLine(6,"2:%d",digital_read(D2));
		displayTextLine(7,"6:%d",digital_read(D6));
		displayTextLine(8,"7:%d",digital_read(D14));
		displayTextLine(9,"sa;%d",s_hitec(Rsensor)-s_hitec(Lsensor));
		displayTextLine(10,"L:%d",ultrasonic_read(U0));
		displayTextLine(11,"S:%d",ultrasonic_read(U1));
		displayTextLine(12,"R:%d",ultrasonic_read(U2));
	}
	eraseDisplay();
}

int s_hitec(tSensors port){
	int red,green,blue;
	HTCSreadRGB(port,red,green,blue);
	return(green);
}

void RST(){
	motor[Rmotor]=0; motor[Lmotor]=0;
}

void Advance(float cm,int Power){
	moveMotorTarget(Lmotor,cm,-Power); moveMotorTarget(Rmotor,cm,-Power);
	waitUntilMotorStop(Lmotor); waitUntilMotorStop(Rmotor); RST();
}

void Right_Spin(float Angle,int Power){
	moveMotorTarget(Lmotor,Angle,-Power); moveMotorTarget(Rmotor,Angle,Power);
	waitUntilMotorStop(Lmotor);	RST();
}

void Left_Spin(float Angle,int Power){
	moveMotorTarget(Lmotor,Angle,Power); moveMotorTarget(Rmotor,Angle,-Power);
	waitUntilMotorStop(Rmotor);	RST();
}

void Right_Turn(float Angle,int Power){
	moveMotorTarget(Lmotor,Angle,-Power);	moveMotorTarget(Rmotor,Angle,0);
	waitUntilMotorStop(Lmotor);	RST();
}

void Left_Turn(float Angle,int Power){
	moveMotorTarget(Lmotor,Angle,0); moveMotorTarget(Rmotor,Angle,-Power);
	waitUntilMotorStop(Rmotor); RST();
}

void StopF(int Power){
	while(digital_read(D1)==0||digital_read(D2)==0){
		if(digital_read(D2)==0) motor[Rmotor]=-Power;
		else{ motor[Rmotor]=0; }
		if(digital_read(D1)==0) motor[Lmotor]=-Power;
		else{ motor[Lmotor]=0; }
	}
	playImmediateTone(400,30);
	RST();
}

void StopFU(int Power){
	while(digital_read(D6)==0||digital_read(D7)==0){
		if(digital_read(D6)==0) motor[Rmotor]=-Power;
		else{ motor[Rmotor]=0; }
		if(digital_read(D7)==0) motor[Lmotor]=-Power;
		else{ motor[Lmotor]=0; }
	}
	playImmediateTone(400,30);
	RST();
}

void Bget(void){
	Ad(300,-50);
	moveMotorTarget(Arm,200,100); sleep(1000);
	RST();
	moveMotorTarget(Arm,50,-100);sleep(500);
	moveMotorTarget(Arm,200,-100); sleep(1000);
}

void Load(int *m){
	int a=0;
	for(a=0;a<20;a++){

		if(m[a]>10) Left_Spin(m[a],50);
		else if(m[a]<-10) Right_Spin(m[a],50);

	}
}

void Slope(void){
FPOWER=20; GEIN=0.2;
while(digital_read(D3)==1) trace();
FPOWER=NPW; GEIN=NGEIN;
}

void Gap(int a){
	static int white=0;
	white++;
	if(a==1) white=0;
	else playImmediateTone(400,10);
	if(white>3){
		MV(-50,-50);
		while(analog_read(A3)>80&&analog_read(A1)>150&&analog_read(A2)>150&&s_hitec(S1)>100&&s_hitec(S2)>100) white++; RST();
		if(analog_read(A2)<150&&white>30){ Left_Turn(850,-100); Right_Turn(850,-100); }
		if(analog_read(A1)<150&&white>30){ Right_Turn(850,-100); Left_Turn(850,-100); }
	}
}

int SETmargin(void){
	int a,sam=0;
	while(1!=getButtonPress(buttonEnter))displayTextLine(1,"SETmargin");
	for(a=0;a<30;a++)	sam+=s_hitec(Rsensor)-s_hitec(Lsensor);
	sam=sam/30;
	playImmediateTone(800,30);
	return(sam);
}

/////////////////////////////////////////////////////////////////////////////////

void hosei(int count){
		int iriguti=0;
		int a=3,b=0;
		int r=a,l=b;
		moveMotorTarget(Arm,200,-100);	sleep(1000);
		while(digital_read(D1)==0&&digital_read(D2)==0){
			while(digital_read(D1)==0&&digital_read(D2)==0){
				MV(-70,-70);
				if(ultrasonic_read(U2)>r&&ultrasonic_read(U2)<15){ Right_Spin(60,50); sleep(300); r=ultrasonic_read(U2); l=b; }
				if(ultrasonic_read(U2)<l){ Left_Spin(40,50); sleep(300); l=ultrasonic_read(U2); r=a; }
			}
			if(ultrasonic_read(U2)>50&&iriguti==0){
				if(count==3){ Ad(400,-100); moveMotorTarget(Arm,200,100);	sleep(1000); Right_Turn(1300,100); Ad(1000,100); }
				Left_Turn(500,-100); Right_Turn(500,-100); iriguti=1; }
			RST();	Ad(50,-100);	Left_Spin(50,100);
		}
		sleep(500);
}

void kado(void){
	Left_Turn(650,-100); Right_Turn(650,-100); Ad(200,-100); Left_Turn(1300,100); Ad(400,100); Ad(600,-100);
}

void yoke(int kyori){
			resetMotorEncoder(Rmotor);
			while(digital_read(D2)==0||digital_read(D1)==0){
				if(digital_read(D2)==1){
					MV(-50,20);}
				else MV(-60,-60);
				if(getMotorEncoder(Rmotor)<-kyori) break;
			}
			RST();
}

void haikai(void){
	//Left_Turn(300,100);
	int count=0;

	while(count<4){
	kabezoi:
		resetMotorEncoder(Rmotor);
		hosei(count);

		if(ultrasonic_read(U1)>25){
			count++;
			Ad(50,-100);
			yoke(2500);
			if(count==1||count==2){  //goal

			Ad(50,-100);	Left_Spin(50,100);
			moveMotorTarget(Arm,200,100);	sleep(1000);
			Left_Turn(1300,-100);	Ad(100,100); Right_Turn(1400,100);
			moveMotorTarget(Gate,200,-10);	sleep(3000);	moveMotorTarget(Gate,200,100);
			Right_Turn(1400,-100);	Left_Turn(1400,100);
			Ad(700,-100);
			moveMotorTarget(Arm,200,-100);	sleep(1000);

			}

			yoke(2500);
			//goto kabezoi;
		}

		else{
			StopF(30);	sleep(300);
			Ad(500,-50);
			moveMotorTarget(Arm,200,100);	sleep(1000);
			StopFU(70);
			kado();
		}
	}

	opentest();

}

//////////////////////////////////////////////////////////////////////////////////////

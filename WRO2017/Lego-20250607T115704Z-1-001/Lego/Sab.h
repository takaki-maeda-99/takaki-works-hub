#define HiBRV 50
#define EvBRV 20
#define Pmargin 0
#define R 197
#define U 'U'
#define C 'C'
#define T 'T'
#define F 'F'
#define H2 160
#define H4 400
#define H5 550
#define S 200
#define M 400
#define L 550
#define Sca 100
#define Mca 200
#define Lca 400

int static margin=10;
float static GEIN=0.07;
int static FPOWER=20;
int static X=1;
int static Mode=0;
int static Wall[5]={0,1,3,5,7};//{RED,GREEN,YELLOW,BLUE,BLACK}
int static TP[8]={0,1,2,3,0,1,2,3};//Turbines Position

int s_hitec(tSensors port){
	static int red,green,blue;
	HTCSreadRGB(port,red,green,blue);
	return((red+green+blue)/3);
}

void opentest(void){
	int sa;
	eraseDisplay();
	while(1!=getButtonPress(buttonEnter)){
		sa=s_hitec(S1)-s_hitec(S2);
		displaybigTextLine(0,"OPENTEST");
		displayTextLine(3,"RSENSOR:%d",s_hitec(S1));
		displayTextLine(4,"LSENSOR:%d",s_hitec(S2));
		displayTextLine(5,"RSENSORColor:%d",HTCSreadColor(S1));
		displayTextLine(6,"LSENSORColor:%d",HTCSreadColor(S2));
		displayTextLine(7,"RBSENSOR:%d",getColorReflected(S3));
		displayTextLine(8,"LBSENSOR:%d",getColorReflected(S4));
		displayTextLine(9,"SA:%d",sa);
	}
	eraseDisplay();
}



void Arm(char mode,int x,int Power,char wait){
	if(mode==U){ motor[up]=0; x=-x+getMotorEncoder(up);	moveMotorTarget(up,x,Power); if(wait==T)	waitUntilMotorStop(up); }
	if(mode==C){ motor[ca]=0; x=-x+getMotorEncoder(ca); moveMotorTarget(ca,x,Power); if(wait==T)	waitUntilMotorStop(ca);	}
	displayTextLine(4,"Up:%d",getMotorEncoder(up));
	displayTextLine(6,"Ca:%d",getMotorEncoder(ca));
}



void trace(void){
	if(getMotorEncoder(up)>100) Arm(U,100,-100,T);
	if(getMotorEncoder(up)<100) Arm(U,100,100,T);
	float	sa=0;
	sa=(s_hitec(S1)-(s_hitec(S2)+margin))*GEIN;
	motor[Lmotor]=sa-FPOWER;
	motor[Rmotor]=-sa-FPOWER;
}



void ARST(){
	motor[up]=0;
	motor[ca]=0;
}



void MRST(){
	motor[Rmotor]=0;
	motor[Lmotor]=0;
}



void ST(float cm,int Power){
	resetMotorEncoder(Rmotor);
	resetMotorEncoder(Lmotor);
	float Angle=cm*14.55,x=Angle;
	int p=1;
	if(Power>0) p=-1;
	MRST();
	if(Mode==1){ Angle=Angle-(Power*0.3-4.6)*14.55; setMotorBrakeMode(motorB,motorCoast); setMotorBrakeMode(motorC,motorCoast); }
	moveMotorTarget(Lmotor,Angle,-(Power+Pmargin));
	moveMotorTarget(Rmotor,Angle,-Power);
	if(X==1) while(p*getMotorEncoder(motorB)<x&&p*getMotorEncoder(motorC)<x) wait1Msec(1);
	if(Mode==1){ setMotorBrakeMode(motorB,motorBrake); setMotorBrakeMode(motorC,motorBrake); }
	MRST();
}



void RT(float Angle,int Power){
	if(X==1) MRST();
	moveMotorTarget(Lmotor,Angle,-Power);
	moveMotorTarget(Rmotor,Angle,Power);
	if(X==1){ waitUntilMotorStop(Rmotor);	MRST();	}
}



void LT(float Angle,int Power){
	if(X==1) MRST();
	moveMotorTarget(Lmotor,Angle,(Power+Pmargin));
	moveMotorTarget(Rmotor,Angle,-Power);
	if(X==1){	waitUntilMotorStop(Rmotor);	MRST();	}
}



void TR(float Angle,int Power){
	Angle=Angle*2;
	playImmediateTone(220,30);
	if(X==1) MRST();
	moveMotorTarget(Lmotor,Angle,-(Power+Pmargin));
	moveMotorTarget(Rmotor,Angle,0);
	if(X==1){	waitUntilMotorStop(Lmotor);	MRST();	}
}



void TL(float Angle,int Power){
	Angle=Angle*2;
	if(X==1) MRST();
	moveMotorTarget(Lmotor,Angle,0);
	moveMotorTarget(Rmotor,Angle,-Power);
	if(X==1){ waitUntilMotorStop(Rmotor);	MRST(); }
}



void StopF(int Power){
	int StR=0,StL=0;
	while(StR!=1||StL!=1){
		if(s_hitec(S1)>HiBRV&&StR==0) motor[Rmotor]=-Power;
		else{ StR=1; motor[Rmotor]=0; }
		if(s_hitec(S2)>HiBRV&&StL==0) motor[Lmotor]=-Power;
		else{ StL=1; motor[Lmotor]=0; }
	}
	playImmediateTone(400,30);
}



void StopB(int Power){
	int StR=0,StL=0;
	while(StR!=1||StL!=1){
		if(getColorReflected(S3)>EvBRV&&StR==0) motor[Rmotor]=-Power;
		else{ StR=1; motor[Rmotor]=0; }
		if(getColorReflected(S4)>EvBRV&&StL==0) motor[Lmotor]=-Power;
		else{ StL=1; motor[Lmotor]=0; }
	}
	playImmediateTone(400,30);
}


void Mv(int Power){
	motor[Rmotor]=-Power;
	motor[Lmotor]=-Power;
}



void Ready(void){
	ARST();
	Arm(U,600,-100,F);
	Arm(C,600,-100,F);
	sleep(700);
	ARST();
	resetMotorEncoder(up);
	resetMotorEncoder(ca);
}

void Btrace(void){
	while(s_hitec(S1)>HiBRV||s_hitec(S2)>HiBRV) trace();
	playImmediateTone(600,30);
}

int blue(int color){
	if(color>=1&&color<=3){
		playSoundFile("Blue");
		return(1);
	}
	return(0);
}

int kuro(int color){
	if(color==0){
		playSoundFile("Black");
		return(2);
	}
	return(0);
}

int green(int color){
	if(color==4){
		playSoundFile("Green");
		return(3);
	}
	return(0);
}

int yellow(int color){
	if(color>=5&&color<=6){
		playSoundFile("Yellow");
		return(4);
	}
	return(0);
}

int red(int color){
	if(color>=7&&color<=10){
		playSoundFile("left");
		return(5);
	}
	return(0);
}

int white(int color){
	if(color>=11&&color<=17){
		playSoundFile("White");
		return(6);
	}
	return(0);
}

int colorsound(int x){
	int c=0;
	if(c==0) c=blue(x);
	if(c==0) 	c=kuro(x);
	if(c==0) 	c=green(x);
	if(c==0) 	c=yellow(x);
	if(c==0) 	c=red(x);
	if(c==0) 	c=white(x);
	return(c);
}

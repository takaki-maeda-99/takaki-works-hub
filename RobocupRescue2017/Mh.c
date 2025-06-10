#define smux_port S4
#define smux_address 0x08
#define GEIN 0.2
#define OGEIN 0.45
#define TG 100//100
#define BL 80

#define p_gain 0.05//0.1
#define d_gain 0.15//0.3
#define a_gain 0.6//0.7
#define hi_gain 0

void BB(int Rs,int Ls,int St);
int SETP(int x);

int FPOWER=40;

int SETP(int x){
	int a,sam=0;
	while(1!=getButtonPress(buttonEnter))displayTextLine(1,"SET");
	for(a=0;a<30;a++)	sam+=x;
	sam=sam/30;
	playImmediateTone(800,30);
	return(sam);
}

int s_hitec(tSensors port){
	int red,green,blue;
	HTCSreadRGB(port,red,green,blue);
	return(green);
}

void opentest(void){
	eraseDisplay();
	displayBigTextLine(0, "OpenTest");
	while(1!=getButtonPress(buttonEnter)){
		displayTextLine(2,"IR-LSENSOR:%d",analog_read(A1));
		displayTextLine(3,"IR-RSENSOR:%d",analog_read(A2));
		displayTextLine(4,"IR-CSENSOR:%d",analog_read(A0));
		displayTextLine(5,"HT-LSENSOR:%d",s_hitec(Lsensor));
		displayTextLine(6,"HT-RSENSOR:%d",s_hitec(Rsensor));
		displayTextLine(7,"1:%d",digital_read(D13));
		displayTextLine(8,"2:%d",digital_read(D2));
	}
	eraseDisplay();
}

void RST(void){
	motor[Lmotor]=0;
	motor[Rmotor]=0;
	motor[motorC]=0;
	motor[motorD]=0;
}

void MV(int p){
	motor[Lmotor]=p;
	motor[Rmotor]=p;
}

void mv(int m,int p){
	moveMotorTarget(Lmotor,m,p);
	moveMotorTarget(Rmotor,m,p);
	waitUntilMotorStop(Lmotor);
	waitUntilMotorStop(Rmotor);
}

void roll(int m,int p){
	moveMotorTarget(Lmotor,m,p);
	moveMotorTarget(Rmotor,m,-p);
	waitUntilMotorStop(Lmotor);
	waitUntilMotorStop(Rmotor);
}

void turn(int m,int p){
	RST();
	if(p>0)	moveMotorTarget(Lmotor,m,p); 	waitUntilMotorStop(Lmotor);
	if(p<0) moveMotorTarget(Rmotor,m,-p); 	waitUntilMotorStop(Rmotor);
}

void Bturn(int m,int p){
	RST();
	if(p>0)	moveMotorTarget(Lmotor,m,-p); 	waitUntilMotorStop(Lmotor);
	if(p<0) moveMotorTarget(Rmotor,m,p); 	waitUntilMotorStop(Rmotor);
}

void Rt (void){
	roll(60,70); motor[Lmotor]=50; motor[Rmotor]=-50; while(s_hitec(Rsensor)>60); roll(75,70);
}

void Lt (void){
	roll(60,-70); motor[Lmotor]=-50; motor[Rmotor]=50; while(s_hitec(Lsensor)>60); roll(75,-70);
}

int	AllS(int mode){
	int a=0;
	if(mode==0&&(analog_read(A2)<BL||analog_read(A1)<BL||analog_read(A0)<BL||s_hitec(Lsensor)<BL||s_hitec(Rsensor)<BL)) a=1;
	if(mode==1&&(analog_read(A2)<TG||analog_read(A1)<TG||analog_read(A0)<TG||s_hitec(Lsensor)<TG||s_hitec(Rsensor)<TG)) a=1;
	return(a);
}

void Gap(void){
	MV(-50);
	while(AllS(1)==0); RST();
	//if(s_hitec(Rsensor)<BL);
	//else if(s_hitec(Lsensor)<BL);
	if(analog_read(A2)<BL);
	else if(analog_read(A1)<BL);
	else{ mv(100,50); MV(50); while(AllS(1)==0);}
	//mv(100,-50);
}

void HTtrace(void){
	int Rs=s_hitec(Rsensor),Ls=s_hitec(Lsensor),light=Rs-Ls;
	int static light_tmp=0;
	float	sa=p_gain*light+d_gain*(light-light_tmp);
	motor[Lmotor]=-sa+FPOWER;
	motor[Rmotor]=sa+FPOWER;
	light_tmp=light;
}

void trace(void){
	//int Rs=analog_read(A2),Ls=analog_read(A1),St=analog_read(A0);
	int static white=0;
	int Rs=analog_read(A2),Ls=analog_read(A1),Cs=analog_read(A0),Rhi=s_hitec(Rsensor),Lhi=s_hitec(Lsensor);
	float static light_tmp=0,light,GEAR=1;
	BB(Rs,Ls,Cs);
	if(Rs<TG||Ls<TG){
		FPOWER=10;
		float	margin=(Rs-Ls)*a_gain;
		motor[Lmotor]=-margin+FPOWER;
		motor[Rmotor]=margin+FPOWER;
		GEAR=1;
	}
	else {
		GEAR++;
		if(GEAR>=10) FPOWER=40;
		else if(GEAR>=5) FPOWER=30;
		else if(GEAR>=1) FPOWER=20;
		light=Rhi-Lhi;
		float sa=p_gain*light+d_gain*(light-light_tmp);
		motor[Lmotor]=-sa+FPOWER;
		motor[Rmotor]=sa+FPOWER;
		light_tmp=light;
	}
	/*
	BB(Rs,Ls,St);
	if(Rs<TG||Ls<TG){
		float	sa=(Rs-Ls)*OGEIN;
		FPOWER=10;
		motor[Lmotor]=-sa+FPOWER;
		motor[Rmotor]=sa+FPOWER;
	}
	else HTtrace();*/
	if(Cs>50&&AllS(1)==0){white++; if(white>5){Gap(); white=0;}}
	else white=0;
}

void BB(int Rs,int Ls,int St){
	if((St<BL&&Rs<BL)||(St<BL&&Ls<BL)){
		playImmediateTone(400,30);
		mv(100,30);
		if(analog_read(A0)>TG&&s_hitec(Lsensor)>TG&&s_hitec(Rsensor)>TG){
			if(Ls<TG) Lt();
			else Rt();
		}
		for(int b=0;b<10;b++){ trace(); sleep(10); }
	}
}

void kousaten(void){
	if(HTCSreadColor(Lsensor)==4||HTCSreadColor(Rsensor)==4){
		RST();sleep(500);if(HTCSreadColor(Lsensor)==4||HTCSreadColor(Rsensor)==4){
		int a=0,Rm=0,Lm=0;
		MV(30);
		for(a=0;a<10;a++){if(HTCSreadColor(Lsensor)==4) Lm=1; if(HTCSreadColor(Rsensor)==4) Rm=1; sleep(10);}
		if(Lm&&Rm) roll(700,50);
		else if(Lm){mv(100,70); Lt();}
		else {mv(100,70); Rt();}
		}
	}
}

void syougai(void){
	if(digital_read(D9)==1){
		mv(200,-70); roll(350,50); mv(700,70); roll(350,-50); mv(1150,70); roll(350,-50);
		while(analog_read(A0)>=TG) MV(70);
		Rt();
	}
}

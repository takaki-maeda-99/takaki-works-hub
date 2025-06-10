#define smux_port S4
#define smux_address 0x08
#define GEIN 0.2
#define OGEIN 0.4
#define TG 150
#define BL 80

int FPOWER=50;

void BB(int Rs,int Ls,int St);

void opentest(void){
	eraseDisplay();
	displayBigTextLine(0, "OpenTest");
	while(1!=getButtonPress(buttonEnter)){
		displayTextLine(3,"LSENSOR:%d",analog_read(A6));
		displayTextLine(4,"RSENSOR:%d",analog_read(A7));
		displayTextLine(5,"1:%d",digital_read(D10));
		displayTextLine(6,"2:%d",digital_read(D2));
	}
	eraseDisplay();
}

void RST(void){
	motor[Lmotor]=0;
	motor[Rmotor]=0;
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

int s_hitec(tSensors port){
	int red,green,blue;
	HTCSreadRGB(port,red,green,blue);
	return(green);
}

void Rt (void){
	roll(60,70); motor[Lmotor]=50; motor[Rmotor]=-50; while(s_hitec(Rsensor)>60); roll(75,70);
}

void Lt (void){
	roll(60,-70); motor[Lmotor]=-50; motor[Rmotor]=50; while(s_hitec(Lsensor)>60); roll(75,-70);
}

int AllS(int mode){
	if(mode==1) if(analog_read(A7)<BL||analog_read(A6)<BL||analog_read(A5)<BL||s_hitec(Lsensor)<BL||s_hitec(Rsensor)<BL)	return(1);
	return(0);
}

void Gap(void){
	MV(-50);
	while(AllS(1)==0); RST();
	if(analog_read(A7)<100);
	else if(analog_read(A6)<100);
	else{ mv(100,50); MV(50); while(AllS(1)==0); }
}

void trace2(void){
	int Rs = s_hitec(Rsensor),Ls = s_hitec(Lsensor);
	float	sa = (Rs-Ls)*GEIN;
	motor[Lmotor]=-sa+FPOWER;
	motor[Rmotor]=sa+FPOWER;
	FPOWER=50;
}

void trace(void){
	int Rs = analog_read(A7),Ls = analog_read(A6),St = analog_read(A5);
	int static white=0;
	BB(Rs,Ls,St);
	if(Rs<TG||Ls<TG){
		float	sa = (Rs-Ls)*OGEIN;
		FPOWER=15;
		motor[Lmotor]=-sa+FPOWER;
		motor[Rmotor]=sa+FPOWER;
	}
	else trace2();
	if(St>170){ white++; if(white>10){ Gap(); white=0; } }
	else white=0;
}

void BB(int Rs,int Ls,int St){
	if((St<BL&&Rs<BL)||(St<BL&&Ls<BL)){
		playImmediateTone(400,30);
		mv(100,30);
		if(analog_read(A5)>150&&s_hitec(Lsensor)>150&&s_hitec(Rsensor)>150){
			if(Ls<100) Lt();
			else Rt();
		}
		for(int b=0;b<10;b++){ trace(); sleep(10); }
	}
}

void kousaten(void){
	if(HTCSreadColor(Lsensor)==4||HTCSreadColor(Rsensor)==4){
		int a=0,Rm=0,Lm=0;
		MV(30);
		for(a=0;a<10;a++){
			if(HTCSreadColor(Lsensor)==4) Lm=1; if(HTCSreadColor(Rsensor)==4) Rm=1; sleep(10); }
			if(Lm&&Rm) roll(700,50);
		else if(Lm){ mv(100,70); Lt(); }
		else { mv(100,70); Rt(); }
	}
}

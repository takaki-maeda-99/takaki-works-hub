#define smux_port S4
#define smux_address 0x08
#define TG 100			//AM:150 PM:100
#define BL 80
#define cm 37
#define p_gain 0.05  //AM:0.05 PM:0.1 HT
#define d_gain 0.15  //AM:0.15 PM:0.3 sa.dousa
#define a_gain 0.55   //AM:0.6  PM:0.7 IR


/*kihon*/

int FPOWER=40;
int tizu[][]={{-1,-1,-1,-1},{-1,-1,-1,-1},{-1,-1,-1,-1},{-1,-1,-1,-1}};  //-1:kokuu,0:syokiti,1:mu,2:object,3:goal
int kyujo=1;
char flg='N';

int s_hitec(tSensors port){
	int red,green,blue;
	HTCSreadRGB(port,red,green,blue);
	return(green);
}

void opentest(int mode){
	eraseDisplay();
	displayBigTextLine(0, "OpenTest");
	while(1!=getButtonPress(buttonEnter)){
		if(mode==0){
			displayTextLine(2,"ANALOG");
			//displayTextLine(2,"Silver:%d",analog_read(A4));
			displayTextLine(3,"IR-Center:%d",analog_read(A0));
			displayTextLine(4,"   Right :%d",analog_read(A2));
			displayTextLine(5,"   Left  :%d",analog_read(A1));
			displayTextLine(6,"HT-Right :%d",s_hitec(Rsensor));
			displayTextLine(7,"   Left  :%d",s_hitec(Lsensor));
			displayTextLine(8,"US-Center:%d",ultrasonic_read(U1));
			displayTextLine(9,"   Right :%d",ultrasonic_read(U2));
			displayTextLine(10,"   Left  :%d",ultrasonic_read(U0));
			displayTextLine(11,"   BRight:%d",ultrasonic_read(U3));
			displayTextLine(12,"   BLeft :%d",ultrasonic_read(U4));
			displayTextLine(13,"DIGITAL");
			//displayTextLine(13,"block :%d",digital_read(D9));
			displayTextLine(14,"Right :%d Left :%d Pass:%d",digital_read(D1),digital_read(D2),analog_read(A3));
			displayTextLine(15,"BRight:%d BLeft:%d Wall:%d",digital_read(D3),digital_read(D4),digital_read(D8));
		}
		if (mode==1){
			displayTextLine(3,"[ ][0][1][2][3]");
			displayTextLine(4,"[0][%d][%d][%d][%d]",tizu[0][0],tizu[0][1],tizu[0][2],tizu[0][3]);
			displayTextLine(5,"[1][%d][%d][%d][%d]",tizu[1][0],tizu[1][1],tizu[1][2],tizu[1][3]);
			displayTextLine(6,"[2][%d][%d][%d][%d]",tizu[2][0],tizu[2][1],tizu[2][2],tizu[2][3]);
			displayTextLine(7,"[3][%d][%d][%d][%d]",tizu[3][0],tizu[3][1],tizu[3][2],tizu[3][3]);
		}
	}
	eraseDisplay();
}

void Arm(int m,int p,int mode){
	RST();
	resetMotorEncoder(arm);
	resetMotorEncoder(arm2);
	moveMotorTarget(motorC,m,p);
	moveMotorTarget(motorD,m,p);
	if(mode){
		waitUntilMotorStop(motorC);
		waitUntilMotorStop(motorD);}
}

void RST(void){
	motor[Lmotor]=0;
	motor[Rmotor]=0;
	motor[arm]=0;
	motor[arm2]=0;
}

void MV(int p){
	motor[Lmotor]=p;
	motor[Rmotor]=p;
}

int mv(int m,int p,int mode){
	moveMotorTarget(Lmotor,m,p);
	moveMotorTarget(Rmotor,m,p);
	if(mode){
		waitUntilMotorStop(Lmotor);
		waitUntilMotorStop(Rmotor);
	}
	return(1);
}

void roll(int m,int p){
	moveMotorTarget(Lmotor,m,p);
	moveMotorTarget(Rmotor,m,-p);
	waitUntilMotorStop(Lmotor);
	waitUntilMotorStop(Rmotor);
}

void turn(int m,int p){
	RST();
	if(p>0){ moveMotorTarget(Lmotor,m,p); 	waitUntilMotorStop(Lmotor); }
	if(p<0){ moveMotorTarget(Rmotor,m,-p); 	waitUntilMotorStop(Rmotor); }
}

void Bturn(int m,int p){
	RST();
	if(p>0){ moveMotorTarget(Lmotor,m,-p); 	waitUntilMotorStop(Lmotor); }
	if(p<0){ moveMotorTarget(Rmotor,m,p); 	waitUntilMotorStop(Rmotor); }
}

void Rt(void){
	roll(60,70); motor[Lmotor]=50; motor[Rmotor]=-50; while(s_hitec(Rsensor)>60); roll(75,70);
}

void Lt(void){
	roll(60,-70); motor[Lmotor]=-50; motor[Rmotor]=50; while(s_hitec(Lsensor)>60); roll(75,-70);
}

/*rescue*////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void TYjudge(){
	int tate,yoko;
	if(ultrasonic_read(U0)>=90)
		for(tate=0;tate<=3;tate++) for(yoko=0;yoko<=2;yoko++){tizu[tate][yoko]=0; flg='T';}
	else
		for(tate=0;tate<=2;tate++) for(yoko=0;yoko<=3;yoko++){tizu[tate][yoko]=0; flg='Y';}
}

void kiroku(int tate,int yoko,int muki,int US){ //tamakioku
	int kyori;
	if(US>5&&US<8) kyori=0;
	else if(US<38) kyori=1;
	else if(US<45) kyori=2;
	switch(muki){
	case 0:tizu[tate][yoko+kyori]=1; break;
	case 1:tizu[tate+kyori][yoko]=1; break;
	case 2:tizu[tate][yoko-kyori]=1; break;
	case 3:tizu[tate-kyori][yoko]=1; break;
	}
}

void moveArm(int mode){

void mArm(int x){
	RST();
	if(x==0){	Arm(50,50,1); sleep(500); RST();	Arm(150,30,0); sleep(1000); }
	if(x==1){	mv(300,70); mv(300,-70); Arm(300,-50,0); sleep(1000); }
	if(x==2){	mv(300,70); mv(400,-70); Arm(300,-50,0); sleep(1000); }
	if(x==3) Arm(300,-50,0);
	RST();
}

void yoke(int x1,int x2,int x3,int x4,int port){
	if(port==1)	{ motor[Lmotor]=x1; motor[Rmotor]=x2;	}
	else				{ motor[Lmotor]=x3; motor[Rmotor]=x4;	}
}

int pass(){
	int get=0;
	int static cou=0;
	if(analog_read(A3)!=0){cou++; sleep(5);
		if(cou>3){ moveArm(0); moveArm(2); get=1; kyujo=1; playImmediateTone(400,60);}
	}
	else cou=0;
	return(get);
}

void Btouch(){
	int a=0,b=0;
	while(digital_read(D3)==0||digital_read(D4)==0){
		if(digital_read(D3)==1){ motor[Rmotor]=0; a++; }
		else{ motor[Rmotor]=-30; a=0; }
		if(digital_read(D4)==1){ motor[Lmotor]=0; b++; }
		else{ motor[Lmotor]=-30; b=0; }
		sleep(10);
		if(a>3) while(digital_read(D3)==1) turn(10,-70);
		if(b>3) while(digital_read(D4)==1) turn(10,70);
	}
	RST();
}

void goal(){
	int static time=0;
	while(digital_read(D2)==0){
		yoke(-20,30,50,50,digital_read(D1));
		if(digital_read(D1)==0) time++;
		if(time>30&&kyujo==1){ RST();
			moveArm(0); Bturn(300,-70); Bturn(300,70); mv(230,70,1); turn(700,-70);
			mv(300,-70,1); Btouch(); mv(250,100,1); mv(300,-100,1); sleep(1000); mv(320,70,1);
			Bturn(700,-70); moveArm(1); kyujo=0;
		}
	}
	time=0;
}

void now(int muki){
	map[my][mx]=0;
	switch(muki){
	case 0:my++ ; break;
	case 1:mx++ ; break;
	case 2:my-- ; break;
	case 3:mx-- ; break;
	}
	map[my][mx]=2;
	if(my==0&&mx==1) opentest(1);
}



/*void beta(){
int Xlen,Ylen;
int static tate=0,yoko=0;
mv(1100,40); tate++; roll(350,-50); mv(130,-50);
if(flg=='T'){Xlen=3; Ylen=2;}
if(flg=='Y'){Xlen=2; Ylen=3;}
resetMotorEncoder(Lmotor);
while((tate<Xlen)&&(yoko<Ylen)){
while(getMotorEncoder(Lmotor)<1100){ MV(40);
if(ultrasonic_read(U1)<=30){tizu[tate+1][yoko]=1;}
if(ultrasonic_read(U2)<=30){tizu[tate-1][yoko]=1;}
}
sleep(10); yoko++; resetMotorEncoder(Lmotor);
}
RST();
opentest(1);
}*/

/*void map(){
int kyori,run=0,memo=0,ball=0;
int static tate=0,yoko=0,muki=0/*0:S,1:E,2:N,3:W;
resetMotorEncoder(Lmotor);
while((tate<Xlen)&&(tate>=0)&&(yoko<Ylen)&&(yoko>=0)){
while(run<1100){
MV(60); run=memo+getMotorEncoder(Lmotor);
if(ultrasonic_read(U0)<=10){Bturn(350,-50); Bturn(350,50); turn(700,-50); mv(300,-50); if(muki==3) muki=0; else muki++; break;}
switch(muki){case 0:tate--; break; case 1:yoko++; break; case 2:tate++; break; case 3:yoko--; break;}
if(ultrasonic_read(U2)>5&&ultrasonic_read(U2)<45){
RST(); sleep(100); ball++; memo=getMotorEncoder(Lmotor);
if(ball==4){kiroku(tate,yoko,muki,ultrasonic_read(U1)<5);
kyori=ultrasonic_read(U2)*cm+100; roll(350,-50); resetMotorEncoder(Lmotor);
/*if((tu-ka.Sensor no hantei)==0||kyori<=getMotorEncoder(Lmotor)){MV(50);} RST(); arm.ageru mv(kyori,50);
kyori=getMotorEncoder(Lmotor); mv(kyori,-70); roll(350,50); ball=0;}}
}
playImmediateTone(600,30); run=0; memo=0; resetMotorEncoder(Lmotor);
}
RST(); opentest(1);
}*/

/*trace*///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void BB(int Rs,int Ls,int St);

void syougai(void){
	if(digital_read(D9)==1){
		mv(200,-70,1); roll(350,50); mv(700,70,0); roll(350,-50); mv(1150,70,1); roll(350,-50);
		while(analog_read(A0)>=TG) MV(70);
		Rt();
	}
}

int	AllS(int mode){
	int a=0;
	if(mode==0) if(analog_read(A2)<BL||analog_read(A1)<BL||analog_read(A0)<BL||s_hitec(Lsensor)<BL||s_hitec(Rsensor)<BL) a=1;
	if(mode==1) if(analog_read(A2)<TG||analog_read(A1)<TG||analog_read(A0)<TG||s_hitec(Lsensor)<TG||s_hitec(Rsensor)<TG) a=1;
	return(a);
}

void Gap(void){
	MV(-50);
	while(AllS(1)==0); RST();
	if(analog_read(A2)<BL);
	else if(analog_read(A1)<BL);
	else{ mv(100,50,1); MV(50); while(AllS(1)==0); }
}

void trace(void){
	int Rs=analog_read(A2) ,Ls=analog_read(A1) ,Cs=analog_read(A0) ,Rhi=s_hitec(Rsensor) ,Lhi=s_hitec(Lsensor) ;
	float static light_tmp=0 ,light ,GEAR=1 ;
	int static white=0;

	BB(Rs,Ls,Cs);
	syougai();

	if(AllS(1)==0){ white++; if(white>5){ Gap(); white=0; }}
	else white=0;

	if(Rs<TG||Ls<TG){
		FPOWER=10;
		float	margin=(Rs-Ls)*a_gain;
		motor[Lmotor]=-margin+FPOWER;
		motor[Rmotor]=margin+FPOWER;
		GEAR=1;
	}
	else {
		GEAR++;
		light=Rhi-Lhi;
		if(GEAR>=10) FPOWER=40;
		else if(GEAR>=5) FPOWER=30;
		else if(GEAR>=1) FPOWER=20;
		float sa=p_gain*light+d_gain*(light-light_tmp);
		motor[Lmotor]=-sa+FPOWER;
		motor[Rmotor]=sa+FPOWER;
		light_tmp=light;
	}

}

void BB(int Rs,int Ls,int St){
	if((St<BL&&Rs<BL)||(St<BL&&Ls<BL)){
		playImmediateTone(400,30);
		mv(100,30,1);
		if(analog_read(A0)>TG&&s_hitec(Lsensor)>TG&&s_hitec(Rsensor)>TG){
			if(Ls<TG) Lt();
			else Rt();
		}
		else for(int b=0;b<10;b++){ trace(); sleep(10); }
	}
}

void kousaten(void){
	if(HTCSreadColor(Lsensor)==4||HTCSreadColor(Rsensor)==4){
		int a=0,Rm=0,Lm=0;
		MV(30);
		for(a=0;a<10;a++){
			if(HTCSreadColor(Lsensor)==4) Lm=1;
			if(HTCSreadColor(Rsensor)==4) Rm=1;
			sleep(10);
		}
		if(Lm&&Rm) roll(700,50);
		else if(Lm){ mv(100,70,1); Lt(); }
		else { mv(100,70,1); Rt(); }
	}
}

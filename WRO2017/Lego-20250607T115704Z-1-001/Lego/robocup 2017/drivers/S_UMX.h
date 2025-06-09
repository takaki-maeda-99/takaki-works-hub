/*S_UMX Ver.1.2.1*/

#define D0 11
#define D1 12
#define D2 13
#define D3 14
#define D4 15
#define D5 16
#define D6 17
#define D7 18
#define D8 19
#define D9 20

#define D10 21
#define D11 22
#define D12 23
#define D13 24
#define D14 25
#define D15 26
#define D16 27
#define D17 28
#define D18 29
#define D19 30
#define D20 31

#define A0 0
#define A1 1
#define A2 2
#define A3 3
#define A4 4
#define A5 5
#define A6 6
#define A7 7
#define A8 8
#define A9 9
//#define A10 10

#define Ta1 40



#define LED_off 100
#define LED_on 101
#define LED_ten 102


void setup_smux(int portn ,int addres);
int analog_read(char sensor_port);
int digital_read(int sensor_port);
int cps_read(int sensor_port);
void cps_level(float sensor_level);
void led_con(int sensor_port);
void send(byte addres, ubyte data);
int read(byte addres);



int EV3_port = 0;
int smux_addres = 0;

void setup_smux(int portn ,int addres){
    EV3_port = portn;
    smux_addres = addres<<1;
}

int analog_read(char sensor_port){
    int data = 0;
    if(sensor_port>=0 && sensor_port <= 9){
        send(smux_addres, sensor_port);
        wait1Msec(15);
        data = read(smux_addres);
    }else data = -1;
    return(data);
}

int digital_read(int sensor_port){
    int data = 0, read_data = 0;
    if(sensor_port>=11 && sensor_port <= 31){
        if(sensor_port >= 21) sensor_port -= 21;
        send(smux_addres, sensor_port);
        wait1Msec(15);
	    read_data = read(smux_addres);
        if(read_data > 127)
            data = 0;
        else
            data = 1;
    }else data = -1;
    return(data);
}


int cps_read(int sensor_port){
		int data = 0, read_data = 0;
    if(sensor_port>=40 && sensor_port <= 49){
    		send(smux_addres, sensor_port);
        wait1Msec(15);
	    	read_data = read(smux_addres);
        data = read_data;
  	}else data = -1;
    return(data);
}


void cps_level(float sensor_level){
		if(sensor_level >=1 && sensor_level <=30){
				int data = sensor_level + 49;
				send(smux_addres, data);
    		wait1Msec(15);
    		read(smux_addres);
  	}
}


void led_con(int sensor_port){
    send(smux_addres, sensor_port);
    wait1Msec(15);
    read(smux_addres);
}


void send(byte addres, ubyte data)
{
	ubyte send_data[] = {2, addres, data};
	sendI2CMsg(EV3_port, &send_data[0], 1);
}

int read(byte addres){
		char read_data[1] ;
		memset(read_data, 0, sizeof(read_data));
    readI2CReply(EV3_port, &read_data[0], 1);
    int x = read_data[0];
	return x;
}

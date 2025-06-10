/*S_UMX.h Ver.2.1*/
#define version_No 2.1

#define A0 0
#define A1 1
#define A2 2
#define A3 3
#define A4 4/*
#define A5 5
#define A6 6
#define A7 7
#define A8 8
#define A9 9
#define A10 10*/

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
/*#define D15 26
#define D16 27
#define D17 28
#define D18 29
#define D19 30
#define D20 31*/

#define U0 50
#define U1 51
#define U2 52
#define U3 53
#define U4 54
#define ultrasonic_on 60
#define ultrasonic_off 61

#define LED_off 100
#define LED_on 101
#define LED_ten 102

#include "i2c.h"

void setup_smux(int portn ,int addres);
int analog_read(char sensor_port);
int digital_read(int sensor_port);
void ultrasonic_config(int on_off, ubyte use_port);
int ultrasonic_read(int sensor_port);
void led_con(int sensor_port);
int debug(int sensor_port);



int EV3_port = 0;
int smux_addres = 0;


void setup_smux(int portn ,int addres){
		int error_count = 0;
		int reply_data;
    EV3_port = portn;
    smux_addres = addres<<1;
    while(version_No * 10 != (reply_data = i2c_communication(EV3_port,smux_addres, 255))){
  			error_count++;
  			if(error_count >= 3){
  					eraseDisplay();
	  				if(reply_data == 0){
	  						//displayTextLine(1, "%d",reply_data);
  							displayTextLine(4, "Sensor multiplexer");
  							displayBigTextLine(6, "Connect Error");
  							displayBigTextLine(9, "Program");
  							displayBigTextLine(11, " please restart");
  							while();
	  				}else{
	  						displayTextLine(4, "Sensor multiplexer");
  							displayBigTextLine(6, "Version Error");
  							displayBigTextLine(9, "Check header");
  							displayBigTextLine(11, " file version");
  							while();
	  				}
  			}
  			wait1Msec(5);
    }
    wait1Msec(10);
    led_con(LED_on);
    wait1Msec(10);
}

int analog_read(char sensor_port){
    int data = 0;
    if(sensor_port>=0 && sensor_port <= 9){
        data = i2c_communication(EV3_port,smux_addres, sensor_port);
    }else data = -1;
    return(data);
}

int digital_read(int sensor_port){
    int data = 0, read_data = 0;
    if(sensor_port>=11 && sensor_port <= 31){
        read_data = i2c_communication(EV3_port,smux_addres, sensor_port);
        if(read_data == 1)
            data = 1;
        else
            data = 0;
    }else data = -1;
    return(data);
}


void ultrasonic_config(ubyte on_off, ubyte use_port){
    int portn;
    use_port = use_port & 0b00011111;
    if(on_off == 60)
        portn = use_port | 0b10100000;
    else if(on_off == 61)
        portn = use_port | 0b10000000;

    i2c_communication(EV3_port,smux_addres, portn);
}



int ultrasonic_read(int sensor_port){
    int data = 0, read_data = 0;
		if(sensor_port>=50 && sensor_port <= 59){
    		read_data = i2c_communication(EV3_port,smux_addres, sensor_port);
    		data = read_data;
  	}else
  			data = -1;
		return(data);
}


void led_con(int sensor_port){
    i2c_communication(EV3_port,smux_addres, sensor_port);
}


int debug(int sensor_port){
    int data = i2c_communication(EV3_port,smux_addres, sensor_port);
    return(data);
}

/*ultrasonic_UMX Ver.1.0*/

#define U0 50
#define U1 51
#define U2 52


void setup_ultrasonic_mux(int portn ,int addres);
int ultrasonic_read(int sensor_port);
void senda(byte addres, ubyte data);
int reada(byte addres);

int ultrasonic_EV3_port = 0;
int ultrasonic_smux_addres = 0;
int arduino_port = 101;


void setup_ultrasonic_mux(int port_no ,int addres,int use_port){
    ultrasonic_EV3_port = port_no;
    ultrasonic_smux_addres = addres<<1;
    arduino_port = use_port + 100;
    senda(ultrasonic_smux_addres, arduino_port);
    wait1Msec(15);
    reada(ultrasonic_smux_addres);
}


int ultrasonic_read(int sensor_port){
    int data = 0, read_data = 0;
if(sensor_port>=50 && sensor_port <= 59){
        senda(ultrasonic_smux_addres, sensor_port);
    wait1Msec(15);
        read_data = reada(ultrasonic_smux_addres);
    data = read_data;
  }else data = -1;
return(data);
}


void senda(byte addres, ubyte data)
{
	ubyte send_data[] = {2, addres, data};
	sendI2CMsg(ultrasonic_EV3_port, &send_data[0], 1);
}

int reada(byte addres){
		char read_data[1] ;
		memset(read_data, 0, sizeof(read_data));
    readI2CReply(ultrasonic_EV3_port, &read_data[0], 1);
    int x = read_data[0];
	return x;
}

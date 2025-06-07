/*i2c.h Ver.1.0*/

int i2c_communication(int ev3_port_no, byte device_addres, ubyte send_data);
void send(int ev3_port,byte addres, ubyte data);
int read(byte ev3_port);

char read_data[4] ;


int i2c_communication(int ev3_port_no, byte device_addres, ubyte send_data){
		send(ev3_port_no,device_addres, send_data);
    wait1Msec(15);
    return(read(ev3_port_no));
}


void send(int ev3_port,byte addres, ubyte data)
{
	ubyte send_data[] = {2, addres, data};
	sendI2CMsg(ev3_port, &send_data[0],1);
}

int read(byte ev3_port){
		memset(read_data, 0, sizeof(read_data));
    readI2CReply(ev3_port, &read_data[0], 1);
    int x = read_data[0];
	return x;
}

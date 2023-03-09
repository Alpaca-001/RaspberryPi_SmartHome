/**********************************************************************
* Filename    : Barometer.c
* Description : Read atmospheric pressure, temperature and current altitude of BMP180 pressure sensor.
* Author      : Yuqi Nai
* modification: 2023/2/20
**********************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include "smbus.h" 
#include <math.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <pcf8574.h>
#include <lcd.h>
#include <time.h>

#define bmp180_I2C_ADDRESS 0x77
#define BASE 64         // BASE any number above 64
//Define the output pins of the PCF8574, which are directly connected to the LCD1602 pin.
#define RS      BASE+0
#define RW      BASE+1
#define EN      BASE+2
#define LED     BASE+3
#define D4      BASE+4
#define D5      BASE+5
#define D6      BASE+6
#define D7      BASE+7
int pcf8574_address = 0x27;        // PCF8574T:0x27, PCF8574AT:0x3F
const unsigned char bmp180_OVERSAMPLING_SETTING = 3;

// Calibration values - These are stored in the BMP180
short int ac1;
short int ac2; 
short int ac3; 
unsigned short int ac4;
unsigned short int ac5;
unsigned short int ac6;
short int b1; 
short int b2;
short int mb;
short int mc;
short int md;
int b5; 
unsigned int temperature, pressure, altitude;

int lcdhd;// used to handle LCD

void printCPUTemperature(){// sub function used to print CPU temperature
    FILE *fp;
    char str_temp[15];
    float CPU_temp;
    // CPU temperature data is stored in this directory.
    fp=fopen("/sys/class/thermal/thermal_zone0/temp","r");
    fgets(str_temp,15,fp);      // read file temp
    CPU_temp = atof(str_temp)/1000.0;   // convert to Celsius degrees
    printf("CPU's temperature : %.2f \n",CPU_temp);
    lcdPosition(lcdhd,0,0);     // set the LCD cursor position to (0,0) 
    lcdPrintf(lcdhd,"CPU's Temp:%.2fC",CPU_temp);// Display CPU temperature on LCD
    fclose(fp);
}
void printDataTime(){//used to print system time 
    time_t rawtime;
    struct tm *timeinfo;
    time(&rawtime);// get system time
    timeinfo = localtime(&rawtime);//convert to local time
    const char* ps[] ={
        "Sun",
        "Mon",
        "Tue",
        "Wed",
        "Thu",
        "Fri",
        "Sat"
    };
    printf("%s \n",asctime(timeinfo));
    lcdPosition(lcdhd,0,1);// set the LCD cursor position to (0,1) 
    lcdPrintf(lcdhd,"Day:%s:%02d:%02d:%02d",ps[timeinfo->tm_wday],timeinfo->tm_hour,timeinfo->tm_min,timeinfo->tm_sec); //Display system time on LCD
}
int detectI2C(int addr){
    int _fd = wiringPiI2CSetup (addr);   
    if (_fd < 0){		
        printf("Error address : 0x%x \n",addr);
        return 0 ;
    } 
    else{	
        if(wiringPiI2CWrite(_fd,0) < 0){
            printf("Not found device in address 0x%x \n",addr);
            return 0;
        }
        else{
            printf("Found device in address 0x%x \n",addr);
            return 1 ;
        }
    }
}

// Open a connection to the bmp180
// Returns a file id
int bmp180_i2c_Begin(){
    int fd;
    char *fileName = "/dev/i2c-1";
    // Open port for reading and writing
    if ((fd = open(fileName, O_RDWR)) < 0)
        exit(1);
    // Set the port options and set the address of the device
if (ioctl(fd, I2C_SLAVE, bmp180_I2C_ADDRESS) < 0){
        close(fd);
        exit(1);
    }
    return fd;
}
    // Read two words from the bmp180 and supply it as a 16 bit integer
__s32 bmp180_i2c_Read_Int(int fd, __u8 address){
    __s32 res = i2c_smbus_read_word_data(fd, address);
    if (res < 0){
    	    close(fd);
        exit(1);
    }
    // Convert result to 16 bits and swap bytes
    res = ((res<<8) & 0xFF00) | ((res>>8) & 0xFF);
    return res;
}
    //Write a byte to the bmp180
void bmp180_i2c_Write_Byte(int fd, __u8 address, __u8 value){
    if (i2c_smbus_write_byte_data(fd, address, value) < 0) {
        close(fd);
        exit(1);
    }
}
    // Read a block of data bmp180
void bmp180_i2c_Read_Block(int fd, __u8 address, __u8 length, __u8 *values){
    if(i2c_smbus_read_i2c_block_data(fd, address,length,values)<0){
        	close(fd);
        exit(1);
    }
}
void bmp180_Calibration(){
    int fd = bmp180_i2c_Begin();
    ac1 = bmp180_i2c_Read_Int(fd,0xAA);
    ac2 = bmp180_i2c_Read_Int(fd,0xAC);
    ac3 = bmp180_i2c_Read_Int(fd,0xAE);
    ac4 = bmp180_i2c_Read_Int(fd,0xB0);
    ac5 = bmp180_i2c_Read_Int(fd,0xB2);
    ac6 = bmp180_i2c_Read_Int(fd,0xB4);
    b1 = bmp180_i2c_Read_Int(fd,0xB6);
    b2 = bmp180_i2c_Read_Int(fd,0xB8);
    mb = bmp180_i2c_Read_Int(fd,0xBA);
    mc = bmp180_i2c_Read_Int(fd,0xBC);
    md = bmp180_i2c_Read_Int(fd,0xBE);
    close(fd);
}
    // Read the uncompensated temperature value
unsigned int bmp180_ReadUT(){
    unsigned int ut = 0;
    int fd = bmp180_i2c_Begin();
    // Write 0x2E into Register 0xF4
    // This requests a temperature reading
    bmp180_i2c_Write_Byte(fd,0xF4,0x2E);
    // Wait at least 5ms
    usleep(5000);
    // Read the two byte result from address 0xF6
    ut = bmp180_i2c_Read_Int(fd,0xF6);
    // Close the i2c file
    close (fd);
    return ut;
}
// Read the uncompensated pressure value
unsigned int bmp180_ReadUP(){
    unsigned int up = 0;
    int fd = bmp180_i2c_Begin();
    // Write 0x34+(bmp180_OVERSAMPLING_SETTING<<6) into register 0xF4
    // Request a pressure reading w/ oversampling setting
    bmp180_i2c_Write_Byte(fd,0xF4,0x34 + (bmp180_OVERSAMPLING_SETTING<<6));
    // Wait for conversion, delay time dependent on oversampling setting·
    usleep((2 + (3<<bmp180_OVERSAMPLING_SETTING)) * 1000);
    // Read the three byte result from 0xF6
    // 0xF6 = MSB, 0xF7 = LSB and 0xF8 = XLSB
    __u8 values[3];
    bmp180_i2c_Read_Block(fd, 0xF6, 3, values);
    up = (((unsigned int) values[0] << 16) | ((unsigned int) values[1] << 8) | (unsigned int) values[2]) >> (8-bmp180_OVERSAMPLING_SETTING);
    return up;
}
// Calculate pressure given uncalibrated pressure
unsigned int bmp180_GetPressure(unsigned int up){
    int x1, x2, x3, b3, b6, p;
    unsigned int b4, b7;
    b6 = b5 - 4000;
    // Calculate B3
    x1 = (b2 * (b6 * b6)>>12)>>11;
    x2 = (ac2 * b6)>>11;
    x3 = x1 + x2;
    b3 = (((((int)ac1)*4 + x3)<<bmp180_OVERSAMPLING_SETTING) + 2)>>2;
    // Calculate B4
    x1 = (ac3 * b6)>>13;
    x2 = (b1 * ((b6 * b6)>>12))>>16;
    x3 = ((x1 + x2) + 2)>>2;
    b4 = (ac4 * (unsigned int)(x3 + 32768))>>15;
    b7 = ((unsigned int)(up - b3) * (50000>>bmp180_OVERSAMPLING_SETTING));
    if (b7 < 0x80000000)
        p = (b7<<1)/b4;
    else
        p = (b7/b4)<<1;
    x1 = (p>>8) * (p>>8);
    x1 = (x1 * 3038)>>16;
    x2 = (-7357 * p)>>16;
    p += (x1 + x2 + 3791)>>4;
    return p;
}
// Calculate temperature given uncalibrated temperature
// Value returned will be in units of 0.1 deg C
unsigned int bmp180_GetTemperature(unsigned int ut){
    int x1, x2;
    x1 = (((int)ut - (int)ac6)*(int)ac5) >> 15;
    x2 = ((int)mc << 11)/(x1 + md);
    b5 = x1 + x2;
    unsigned int result = ((b5 + 8)>>4);  
    return result;
}
unsigned int bmp180_Altitude(float pressure){
    float A = pressure/101325.0;
    float B = 1/5.25588;
    float C = pow(A,B);
    C = 1 - C;
    C = C / 0.0000225577;
    return C;
}
void printTemperature(){//
    bmp180_Calibration();
    temperature = bmp180_GetTemperature(bmp180_ReadUT());
    lcdPosition(lcdhd,0,0);     // set the LCD cursor position to (0,0) 
    lcdPrintf(lcdhd,"Temperature:%0.1f", ((double)temperature)/10);// Display temperature on LCD
}
void printPressure(){//
    bmp180_Calibration();
    pressure = bmp180_GetPressure(bmp180_ReadUP());
    lcdPosition(lcdhd,0,1);// set the LCD cursor position to (0,1) 
    lcdPrintf(lcdhd,"Pressure:%0.2f", ((double)pressure)/100); //Display pressure on LCD
}
int main(int argc, char **argv){
    int i;
    printf("Program is starting ... \n");
    wiringPiSetup();  
    if(detectI2C(0x27)){
        pcf8574_address = 0x27;
    }else if(detectI2C(0x3F)){
        pcf8574_address = 0x3F;
    }else{
        printf("No correct I2C address found, \n"
        "Please use command 'i2cdetect -y 1' to check the I2C address! \n"
        "Program Exit. \n");
        return -1;
    }
    pcf8574Setup(BASE,pcf8574_address);//initialize PCF8574
    for(i=0;i<8;i++){
        pinMode(BASE+i,OUTPUT);     //set PCF8574 port to output mode
    } 
    digitalWrite(LED,HIGH);     //turn on LCD backlight
    digitalWrite(RW,LOW);       //allow writing to LCD
	lcdhd = lcdInit(2,16,4,RS,EN,D4,D5,D6,D7,0,0,0,0);// initialize LCD and return “handle” used to handle LCD
    if(lcdhd == -1){
        printf("lcdInit failed !");
        return 1;
    }
    while(1){
        printCPUTemperature();//print CPU temperature
        printDataTime();        // print system time
        delay(2000);
        

        bmp180_Calibration();
        temperature = bmp180_GetTemperature(bmp180_ReadUT());
        pressure = bmp180_GetPressure(bmp180_ReadUP());
        altitude = bmp180_Altitude(pressure);
        printf("Temperature\t%0.1f c\n", ((double)temperature)/10);
        printf("Pressure\t%0.2f hPa\n", ((double)pressure)/100);
        printf("Altitude\t%0.1f m\n\n", (double)altitude);
        printTemperature();
        printPressure();
        usleep(2000*1000);
    }
    return 0;
}

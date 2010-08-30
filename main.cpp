//*******************************************************
//					Package Tracker Firmware
//*******************************************************
#include <stdio.h>
#include <string.h>

#include "main.h"
//*******************************************************
//				Memory Management Libraries
//*******************************************************
extern "C"{
#include "rootdir.h"
#include "sd_raw.h"
#include "LPC214x.h"
#include "serial.h"
#include "rprintf.h"
#include "spi0.h"
#include "target.h"
#include "main_msc.h"
#include "fat16.h"
}

//*******************************************************
//					External Component Libs
//*******************************************************
#include "ADXL345.h"
//#include "gps.h"


//*******************************************************
//					Core Functions
//*******************************************************
void bootUp(void);
void goToSleep(int duration);
static void ISR_RxData1(void);
static void ISR_RTC(void);
static void ISR_Timer0(void);
static void ISR_EINT2(void);
void createLogFile(void);
void parseGGA(const char *gps_string);
int parseRMC(const char *gps_string);
void saveData(struct fat16_file_struct **fd, const char * const buf, const int buf_size);
void itoa(int n, char s[]);
void reverse(char s[]);
void wakeUp(void);
int get_adc_1(char channel);
void reset(void);
void initializeGps(void);


//*******************************************************
//					Global Variables
//*******************************************************
#define NMEA_FILE_HEADER "Message ID, Time, Status, Lat., N/S, Long., E/W, Speed, Course, Date, Magnetic Var.\n"

//GPS variables
char gps_message_complete=0, new_gps_data=0, RTC_Set, alarm_set;	//Notification Flags
//char final_message[GPS_BUFFER_SIZE], gps_message[GPS_BUFFER_SIZE];	//Buffers for holding GPS messages
int gps_message_index=0, gps_message_size=0;	//index for copying messages to different buffers
int final_gps_message_size=0;
//GPSdata GPS;	//GPS Struct to hold GPS coordinates.  See PackageTracker.h for Structure definition

//Pressure Sensor (SCP100) Values
unsigned int scp_pressure;
int scp_temp;
char new_scp_data;

//Humidity Sensor (SHT15) Values
unsigned int sht_temp, sht_humidity;
char new_sht_data;

//Accelerometer (ADXL345) Values
signed int acceleration_x, acceleration_y, acceleration_z;

//Logging Parameters
char file_name[32]; 	
int battery_level; 
int log_count;	//Keeps track of how many logs we've made since we've been awake.  Get's reset before going to sleep.

//Log Parameters for logging the Sensor Data
struct fat16_file_struct * LOG_FILE; //File structure for current log file
char log_data[512], log_buffer[200];//log_buffer holds data before putting it into log_data
int log_data_index;	//Keeps track of current position in log_data

//Log Parameters for logging the NMEA file
struct fat16_file_struct * NMEA_FILE; //File structure for current log file
char nmea_data[1024];//
int nmea_data_index=0;

char led_blink=0;

//Sleep Parameters
unsigned int power_register_values;			//Holds the value to load to the power register after waking from sleep
char read_sensors, new_sensor_data;			//Global flag indicating an accelerometer reading has been completed
char wake_event=0;

ADXL345 accelerometer(0, 17);

int main (void)
{
//*******************************************************
//					Main Code
//*******************************************************
	//Initialize ARM I/O
	bootUp();			//Init. I/O ports, Comm protocols and interrupts
	createLogFile();	//Create a new log file in the root of the SD card
	
	rprintf("File created and written to\n\r");
	while(1);
	/*
	//Initialize the GPS
	initializeGps();		//Send the initialization strings
	enable_gps_rmc_msgs(1);

	//Initialize the sensors
	SCPon();		//Turn on the SCP Sensor
	delay_ms(100);	//Allow SCP sensor to initialize
	SCPinit();		//Initialize the SCP sensor
	
	initAccel();	//Initialize Accelerometer

	//SHT15 shouldn't need to be initialized(it just needs power)
	
	*/
	
	/*
	VICIntEnable |= UART1_INT | TIMER0_INT;	//Enable UART1 and Timer0 Interrupts
	while(1){
		while(log_count < (TIMER_FREQ*60)*WAKE_MINUTES){		//After WAKE_MINUTES we will stop logging and go to sleep.
			if(gps_message_complete==1){		//If we've received a new GPS message, record it.
				VICIntEnClr |= UART1_INT | TIMER0_INT;		//Stop the UART1 interrupts while we read the message
				for(int i=0; i<gps_message_size; i++){ //Transfer the GPS message to the final_message buffer
					final_message[i]=gps_message[i];
					gps_message[i]='\0';
				}
				final_gps_message_size=gps_message_size;
				gps_message_complete=0;			//Clear the message complete flag
				VICIntEnable |= UART1_INT | TIMER0_INT;		//Re-Enable the UART0 Interrupts to get next GPS message				
				
				//Populate GPS struct with time, position, fix, date
				//If we received a valid RMC message, log it to the NMEA file
				//if(parseRMC(final_message)){
					//If we were able to parse the entire gps message, than we have new gps data to log
					new_gps_data=1;
					
					//Add the gps message to the nmea buffer
					for(int i=0; i<final_gps_message_size; i++)nmea_data[nmea_data_index++]=final_message[i];
					 
					//If the nmea buffer is full then log the buffer to the NMEA file
					if(nmea_data_index >= MAX_BUFFER_SIZE){
						VICIntEnClr = TIMER0_INT | UART1_INT;
						UnselectSCP();
						saveData(&NMEA_FILE, nmea_data, nmea_data_index);
						nmea_data_index=0;
						VICIntEnable = TIMER0_INT | UART1_INT;
						SelectSCP();
						unselect_card();
						SCPinit();
						delay_ms(10);
						for(int i=0; i<nmea_data_index; i++)nmea_data[i]='\0';
						nmea_data_index=0;
					}
				//}

				CCR = (1<<1); 	//Disable and Reset the RTC
				HOUR = ((GPS.Time[0]-'0')*10) + (GPS.Time[1]-'0');
				MIN = ((GPS.Time[2]-'0')*10) + (GPS.Time[3]-'0');
				SEC = ((GPS.Time[4]-'0')*10) + (GPS.Time[5] -'0');	
				RTC_Set=1;		//Set the RTC_Set flag since we have a valid time in the RTC registers					
				CCR = (1<<0);	//Start the RTC

			}
			
			//Check to see if it's time to read the sensors
			if(read_sensors==1){
				VICIntEnClr |= TIMER0_INT;
								
				//Get Acceleration
				acceleration_x = accelX();
				acceleration_y = accelY();
				acceleration_z = accelZ();
				
				//Get Battery Voltage
				battery_level = ((get_adc_1(0x10)*3300)/1024)*2;	//Report Battery voltage in mV. from AD1.4(P0.13)

				//If the SCP1000 has new data ready, then grab it!
				if(IOPIN0 & SCP_DRDY){
					unselect_card();
					SelectSCP();
					readSCP(&scp_pressure, &scp_temp);//Get temperature and pressure values from SCP1000
					new_scp_data=1;
				}
				//Convert the values of scp_pressure to Pa and scp_temp to degrees C
				scp_pressure /=4;
				if(scp_temp > 0){
					if((scp_temp & (1<<13))==(1<<13))scp_temp = (~scp_temp+1);	//Get the two's compliment of the temp if negative
					scp_temp/=2;
					scp_temp = (scp_temp*1.8)+320;	//Convert from Celsius to Farenheit	
				}
				
				//Get Hum. values every second (Reading SHT15 takes a long time, so we don't read it very often!)
				if(log_count%TIMER_FREQ==0){
					//Get Humidity
					sht15_read(&sht_temp, &sht_humidity);	//Get temp. and humidity values from SHT15 (Values are reported from sht_read function in degrees C)
					new_sht_data=1;
				}
				
				new_sensor_data=1;	//Set the flag to tell the logging routine that there is new sensor data to be saved
				read_sensors=0;		//Reset the "read sensor" flag

				VICIntEnable |= TIMER0_INT;
			}
			
			//If we have new data, lets log it!
			//We will save the data into a CSV file that is stored on the SD card.  
			//To create the CSV file, we'll store the data in a text array with the following format:
			// DATE, UTC_TIME, X ACCEL, Y ACCEL, Z ACCEL, BATT. LEVEL(mV), SCP PRESSURE, SCP TEMPERATURE, SHT TEMPERATURE, SHT HUMIDITY, LATITUDE, LAT. DIRECTION, LONGITUDE, LONG. DIRECTION
			if(new_sensor_data || new_gps_data){ 							
				//Log Time
				//If there is GPS data, use this time and date
				if(new_gps_data && GPS.Fix=='A'){
					for(int i=0; i<6; i++)log_data[log_data_index++]=GPS.Date[i];
					log_data[log_data_index++]=',';
					for(int i=0; i<10; i++)log_data[log_data_index++]=GPS.Time[i];
				}
				//If there is not GPS data, use the RTC time
				else{
					//Put a place marker for the date!
					log_data[log_data_index++]=',';
					if(RTC_Set){
						log_data[log_data_index++]=(HOUR / 10) + '0';
						log_data[log_data_index++]=(HOUR % 10) + '0';
						log_data[log_data_index++]=(MIN / 10) + '0';
						log_data[log_data_index++]=(MIN % 10) + '0';
						log_data[log_data_index++]=(SEC / 10) + '0';
						log_data[log_data_index++]=(SEC % 10) + '0';	
					}
				}
				log_data[log_data_index++]=',';
				
				//Log Acceleration and Battery Values (ADC Values)
				if(new_sensor_data){
					//Log Acceleration
					log_data_index += (int) sprintf(log_data+log_data_index, "%d,%d,%d,%d,", acceleration_x, acceleration_y, acceleration_z, battery_level);
					acceleration_x=0;
					acceleration_y=0;
					acceleration_z=0;
					battery_level=0;
				}
				else for(int i=0; i<4; i++)log_data[log_data_index++]=',';
				
				//Log Pressure Values (SCP1000 Values)
				if(new_scp_data){
					//Log Acceleration
					log_data_index += (int) sprintf(log_data+log_data_index, "%d,%d,", scp_pressure, scp_temp);
					scp_pressure=0;
					scp_temp=0;
					new_scp_data=0;
				}
				else for(int i=0; i<2; i++)log_data[log_data_index++]=',';
				
				//Log Humidity Values (SHT15 Values)
				if(new_sht_data){
					//Log Acceleration
					log_data_index += (int) sprintf(log_data+log_data_index, "%d,%d,", sht_temp, sht_humidity);
					sht_temp=0;
					sht_humidity=0;
					new_sht_data=0;
				}
				else for(int i=0; i<2; i++)log_data[log_data_index++]=',';				
				
				//If we have GPS data, add it to the log buffer
				if(new_gps_data){
					//Log Fix Indicator
					log_data[log_data_index++]=GPS.Fix;
					log_data[log_data_index++]=',';
					
					//Log latitiude
					for(int i=0; i<9; i++)log_data[log_data_index++]=GPS.Latitude.position[i];
					log_data[log_data_index++]=',';
					
					log_data[log_data_index++]=GPS.Latitude.direction;
					log_data[log_data_index++]=',';
					
					//Log longitude
					for(int i=0; i<10; i++)log_data[log_data_index++]=GPS.Longitude.position[i];
					log_data[log_data_index++]=',';
					
					log_data[log_data_index++]=GPS.Longitude.direction;
					log_data[log_data_index++]=',';
					
				}
				else for(int i=0; i<6; i++)log_data[log_data_index++]=',';
				log_data[log_data_index++]='\n';
					
				
				//Only Save Data if the buffer is full! This saves write cycles to the SD card
				if(log_data_index >= MAX_BUFFER_SIZE){
					VICIntEnClr |= TIMER0_INT | UART1_INT;
					UnselectSCP();
					saveData(&LOG_FILE, log_data, log_data_index);
					VICIntEnable |= TIMER0_INT | UART1_INT;
					SelectSCP();
					unselect_card();
					SCPinit();
					delay_ms(10);
					for(int i=0; i<log_data_index; i++)log_data[i]='\0';
					log_data_index=0;
				}
				new_gps_data=0;	//We've saved the GPS coordinates, so clear the GPS data flag
				new_sensor_data=0;	//We've save the accel values, so clear the accel flag
			}
			//If a USB Cable gets plugged in, stop everything!
			if(IOPIN0 & (1<<23))
			{
				VICIntEnClr = UART1_INT | TIMER0_INT | RTC_INT | EINT2_INT;	//Stop all running interrupts
				//Save current logged data and close the file before allowing USB communication
				if ( NULL != LOG_FILE ) {
					UnselectSCP();
					saveData(&LOG_FILE, log_data, log_data_index);
					fat16_close_file(LOG_FILE);
					log_data_index=0;
				}
				if ( NULL != NMEA_FILE ) {
					UnselectSCP();
					saveData(&NMEA_FILE, nmea_data, nmea_data_index);
					fat16_close_file(NMEA_FILE);
					nmea_data_index=0;
				}			
				main_msc();								//Open the mass storage device
				reset();								//Reset to check for new FW
			}
		}
		//After logging for "WAKE MINUTES" we will go to sleep for a while. We will wake up if
		//A.) SLEEP MINUTES expires
		//or
		//B.) The accelerometer detects a free fall
		goToSleep(SLEEP_MINUTES);	//Send the LPC2148 to sleep mode for SLEEP_MINUTES (Defined in PackageTracker.h)
		wakeUp();					//After the RTC alarm goes off or we detect a free fall, the LPC2148 will wake up again.
		
	}
	*/
    return 0;
}


//Usage: delay_ms(1000);
//Inputs: int count: Number of milliseconds to delay
//The function will cause the firmware to delay for "count" milleseconds.
void delay_ms(int count)
{
    int i;
    count *= 10000;
    for (i = 0; i < count; i++)
        asm volatile ("nop");
}

//Usage: bootUp();
//Inputs: None
//This function initializes the serial port, the SD card, the I/O pins and the interrupts
void bootUp(void)
{
	//Initialize UART for RPRINTF
    //rprintf_devopen(putc_serial1); //Init rprintf
	//init_serial1(4800);
    rprintf_devopen(putc_serial0); //Init rprintf
	init_serial0(115200);	
	delay_ms(10); //Delay for power to stablize

    //Bring up SD and FAT
    if(!sd_raw_init())
    {
        //rprintf("SD Init Error\n");
		reset();
    }
    if(openroot())
    {
        //rprintf("SD OpenRoot Error\n");
		reset();
    }
	PINSEL0 &= ~((3<<4)|(3<<6));
	
	/*
	//Enable AD conversion on P0.13(AD1.4) FOR BATT_MEAS
	PINSEL0 |= (3<<26);
	
	//Set up the EINT2 External Interrupt Functionality
	PINSEL0 &= ~(3<<30);	//Clear P0.15 special function
	PINSEL0 |= (2<<30);	//Set P0.15 to EINT2
	VICIntEnClr |= EINT2_INT;//Make sure EINT2 interrupts are disabled
	EXTINT |= (1<<2);		//Clear the EINT2 Interrupt bit
	EXTMODE |= (1<<2);		//Set EINT2 to be edge sensitive
	EXTINT |= (1<<2);		//Clear the EINT2 Interrupt bit
	EXTPOLAR |= (1<<2);	//Set EINT2 to detect rising edges
	INTWAKE |= (1<<2);		//ARM will wake up from power down on an EINT2 interrupt
	EXTINT |= (1<<2);		//Clear the EINT2 Interrupt bit	
	
	//Initialize I/O Ports and Peripherals
	IODIR0 = SCLK | MOSI | SD_CS | ACCEL_CS | GPS_EN | I2C_SCL | LED;
	IODIR0 &= ~(MISO | SCP_DRDY | ACCEL_INT2 | ACCEL_INT1 | BATT_MEAS);
	
	//IODIR1 = SCP_EN | SCP_CS;	
	
	//Make sure peripheral devices are not selected
	UnselectAccelerometer();
	UnselectSCP();	
	
	//Initialize the SPI bus
	SPI0_Init();			//Select pin functions for SPI signals.
    S0SPCCR = 64;           // SCK = 1 MHz (60MHz / 64 ~= 1Mhz)
    S0SPCR  = 0x20;         // Master, no interrupt enable, 8 bits	
		
    //Setup the Interrupts
	//Enable Interrupts
	VPBDIV=1;										// Set PCLK equal to the System Clock	
	VICIntSelect = ~(UART1_INT | TIMER0_INT | RTC_INT | EINT2_INT);
	VICVectCntl0 = 0x20 | 7;						//Set up the UART0 interrupt
	VICVectAddr0 = (unsigned int)ISR_RxData1;
	VICVectCntl1 = 0x20 | 13;						//Set up the RTC interrupt
	VICVectAddr1 = (unsigned int)ISR_RTC;	
	VICVectCntl2 = 0x20 | 4;						//Timer 0 Interrupt
	VICVectAddr2 = (unsigned int)ISR_Timer0;
	VICVectCntl3 = 0x20 | 16;						//EINT2 External Interrupt 
	VICVectAddr3 = (unsigned int)ISR_EINT2;
	
	//Setup the UART0 Interrupt
	U1IER = 0x01;				//Enable FIFO on UART with RDA interrupt (Receive Data Available)
	U1FCR &= 0x3F;				//Enable FIFO, set RDA interrupt for 1 character	
	
	//Setupt the Timer0 Interrupt
	T0PR = 1200;				//Divide Clock(60MHz) by 1200 for 50kHz PS
	T0TCR |=0X01;				//Enable the clock
	T0CTCR=0;					//Timer Mode
	T0MCR=0x0003;				//Interrupt and Reset Timer on Match
	T0MR0=(50000/TIMER_FREQ);	//Set Interrupt frequency by dividing system clock (50KHz) by TIMER_FREQ (defined in PackageTracker.h as 10) 
								//Value will result in Timer 0 interrupts at TIMER_FREQ
	
	//Set up the RTC so it can be used for sleeping
	CCR = ~(1<<0);				//use the system clock, and disable RTC for now
	CIIR = 0;					//Don't allow any increment interrupts
	AMR = ~(1<<1);				//Only check the minutes value of the alarm	
	//Set up prescaler so RTC runs at 32.768 Khz
	PREINT = 1830;				//Prescale Integer = (60MHz/32768)-1
	PREFRAC = 1792;				//Prescale Fraction = 60MHz - ((PREINT+1)*32768)	
	*/
}
/*
//Usage: go_to_sleep(5);
//Inputs: int duration - length in minutes the device should sleep for
//This function will turn off all external components, set the
//RTC alarme to wake up after "duration" minutes, and enter IDLE mode
void goToSleep(int duration)
{
	CCR = (1<<1);	//Disable and Reset the RTC
	
	//Save the buffered data before going to sleep
	LED_OFF();
	SCPoff();
	delay_ms(10);
	
	if(log_data_index>0){
		saveData(&LOG_FILE, log_data, log_data_index);	
		for(int i=0; i<log_data_index; i++)log_data[i]='\0';
		log_data_index=0;	
	}
	
	//Turn Off External Peripheral Devices
	GPSoff();
	//Leave Acceleromter ON to generate interrupts
	//SHT15 automatically goes to sleep after a measurement
	
	//Set the alarm to wake up after "duration" minutes
	SEC=0;
	ALSEC = SEC;
	//MIN=0;
	//ALMIN=duration;
	if (MIN+duration>=60) ALMIN = duration-(60-MIN);
	else ALMIN = MIN + duration;	
		
	sprintf(log_data, "Sleep\n");
	saveData(&LOG_FILE, log_data, strlen(log_data));
	
	CCR = (1<<0);	//Enable the RTC
		
	//Configure Interrupts
	VICIntEnClr |= (UART1_INT | TIMER0_INT); 	//Stop UART and Timer interrupts
	VICIntEnable |= (EINT2_INT | RTC_INT);	//Turn on RTC and Accel. interrupts

	//Read the accel once to clear any interrupts
	acceleration_x = accelX();
	acceleration_y = accelY();
	acceleration_z = accelZ();			
	adxl345_read(INT_SOURCE);
	
	//Turn Off Internal Peripheral Modules in Power Control Register
	power_register_values = PCONP; //Save the power register so we know what to load when we wake up
	PCONP = (1<<9);		//Turn off power to all peripherals except the RTC
	//Go to IDLE mode
	PCON = (1<<0);			//Go into IDLE mode
}
*/
/*
//Usage: wake_up();
//Inputs: None
//This function will turn on and initialize the peripheral sensors
//and re-enable the UART0 interrupts
void wakeUp(void)
{
	//Turn on power to ARM peripheral devices
	PCONP |= power_register_values; //Load the saved power register values
	VICIntEnClr |= EINT2_INT | RTC_INT;

	sprintf(log_data, "Wake\n");
	saveData(&LOG_FILE, log_data, strlen(log_data));	

	//Check the ADXL345 Interrupt Source register to clear any pending interrupts
	adxl345_read(INT_SOURCE);

	//Clear program flags to 'start fresh'
	gps_message_complete=0;
	read_sensors=0;
	
	//Power up and Init. the External peripheral devices
	initializeGps();		//Send the initialization strings
	enable_gps_rmc_msgs(1);
	SCPon();		//Turn on the SCP sensor
	delay_ms(10);	//Allow SCP to initialize
	SCPinit();		//Initialize the SCP sensor
	
	//SHT15 and ADXL345 shouldn't need to be initialized
	
	delay_ms(1000);	//Wait for GPS to stablize
	log_count=0;	//Clear the log count
	RTC_Set=0;
	
	//Enable UART0 and Timer Interrupts
	VICIntEnable |= UART1_INT | TIMER0_INT;
}
*/
/*
//Usage: None (Automatically Called by FW)
//Inputs: None
//Description: Called when a character is received on UART1.  
static void ISR_RxData1(void)
{
	char val = (char)U1RBR;
	//When we get a character on UART1, save it to the GPS message buffer
	if(val=='\n'){ 	//Newline means the current message is complete
		gps_message[gps_message_index]= val;
		gps_message_complete=1;					//Set a flag for the main FW
		gps_message_size=gps_message_index+1;
		gps_message_index=0;
	}
	else{
		//If we get the start character, reset the index
		if(val == '$')gps_message_index=0;
		gps_message[gps_message_index++]= val;
		gps_message_complete=0;
	}
	VICVectAddr =0;						//Update the VIC priorities
}
*/
/*
//Usage: None (Automatically Called by FW)
//Inputs: None
//Description: Called when the RTC alarm goes off.  This wakes
//				the Package Tracker from sleep mode.
static void ISR_RTC(void)
{	
	//Clear the Alarm Interrupt bit from the ILR
	ILR = ((1<<1)|(1<<0));
	wake_event=RTC_TIMEOUT_WAKE;
	VICVectAddr =0;		//Update the VIC priorities
}
*/
//Usage: createLogFile();
//Inputs: None
//Outputs: None
//Description: Creates a log file in the root directory of the SD card with the name
//				PackageTrackerXX.csv.  XX increments to the next available number each
//				time the function is called.
void createLogFile(void){
	static int file_number;

//Create the Sensor Data Log File	
	//Set an initial file name
	sprintf(file_name, "PackageTracker%03d.csv", file_number);
	//Check to see if the file already exists in the root directory.
    while(root_file_exists(file_name))
    {
        file_number++;	//If the file already exists, increment the file number and check again.
        if(file_number == 250)
        {
            //rprintf("\nToo many files in root!\n");
        }
        sprintf(file_name, "PackageTracker%03d.csv", file_number);
    }
    //Get the file handle of the new file.  We will log the data to this file
	LOG_FILE = root_open_new(file_name);
	//Now that we have the file opened, let's put a label in the first row
	fat16_write_file(LOG_FILE, (unsigned char*)"Date, UTC, X, Y, Z, Batt, Pres., SCP Temp., SHT Temp, Humidity, Fix, Lat., Lat. Dir., Long., Long. Dir.,\n", 105);
	sd_raw_sync();

//Create the NMEA Log File	
	//Set an initial file name
	sprintf(file_name, "PackageTrackerNMEA%03d.csv", file_number);
	//Check to see if the file already exists in the root directory.
    while(root_file_exists(file_name))
    {
        file_number++;	//If the file already exists, increment the file number and check again.
        if(file_number == 250)
        {
            //rprintf("\nToo many files in root!\n");
        }
        sprintf(file_name, "PackageTrackerNMEA%03d.csv", file_number);
    }
    //Get the file handle of the new file.  We will log the data to this file
	NMEA_FILE = root_open_new(file_name);
	//Now that we have the file opened, let's put a label in the first row
	fat16_write_file(NMEA_FILE, (const unsigned char *)NMEA_FILE_HEADER, strlen(NMEA_FILE_HEADER));
	sd_raw_sync();	
}
/*
//Usage: parseGGA(final_message);
//Inputs: const char *gps_string - GGA NMEA string
//This functions splits a GGA message into the
//portions and assigns them to components of
//a GPS structure
void parseGGA(const char *gps_string){
	int i=0;
	//Parse the GGA Message.  1st portion dismissed
	while(gps_string[i] != ',')i++;
	i++;
	//Second portion is UTC timestamp
	for(int j=0;gps_string[i] != ','; j++){
		GPS.Time[j]=gps_string[i];
		i++;
	}
	i++;
	//Third portion is Latitude
	for(int j=0;gps_string[i] != ',';j++){
		GPS.Latitude.position[j]=gps_string[i];
		i++;
	}
	i++;			
	//Fourth portion is Latitude direction
	for(int j=0;gps_string[i] != ','; j++){
		GPS.Latitude.direction=gps_string[i];
		i++;
	}
	i++;	
	//Fifth portion is Long.
	for(int j=0;gps_string[i] != ','; j++){
		GPS.Longitude.position[j]=gps_string[i];
		i++;
	}
	i++;			
	//Sixth portion is Long direction
	while(gps_string[i] != ','){
		GPS.Longitude.direction=gps_string[i];
		i++;
	}
	i++;		
	//Seventh portion is fix
	while(gps_string[i] != ','){
		GPS.Fix=gps_string[i];
		i++;
	}
	i++;
	//8th portion dismissed
	while(gps_string[i] != ',')i++;
	i++;				
	//8th portion dismissed
	while(gps_string[i] != ',')i++;
	i++;				
	//10th portion is Altitude
	for(int j=0;gps_string[i] != ','; j++){
		GPS.Altitude[j]=gps_string[i];
		i++;
	}	
}
*/
/*
//Usage: parseRMC(final_message);
//Inputs: const char *gps_string - RMC NMEA string
//This functions splits a GGA message into the
//portions and assigns them to components of
//a GPS structure
//This functions splits a RMC message into the
//portions and assigns them to components of
//a GPS structure
int parseRMC(const char *gps_string){
	int i=0;
	int comma_count=0, character_count=0;
	
	for(int j=0; gps_string[j]!= '\n'; j++){
		if(gps_string[j] == ',')comma_count+=1;
	}
	
	//If we didn't receive all of the RMC fields, then return an error
	if(comma_count != 11)return 0;
	//If we didn't receive the correct SiRF header, the return an error
	if(gps_string[0] != '$' || gps_string[1] != 'G' || gps_string[2] != 'P')return 0;
	
	//Parse the GGA Message.  1st portion dismissed
	while(gps_string[i] != ',')i++;
	i++;
	//Second portion is UTC timestamp
	for(int j=0;gps_string[i] != ','; j++){
		GPS.Time[j]=gps_string[i];
		i++;
		character_count+=1;
	}
	//Make sure we received 10 character for Time
	if(character_count != 10)return 0;
	character_count=0;
	
	i++;
	//Third portion is fix
	while(gps_string[i] != ','){
		GPS.Fix=gps_string[i];
		i++;
	}	
	i++;
	if(GPS.Fix != 'A')return 0;
	
	//Fourth portion is Latitude
	for(int j=0;gps_string[i] != ',';j++){
		GPS.Latitude.position[j]=gps_string[i];
		i++;
		character_count +=1;
	}
	//Make sure we received 9 characters for the Latitude
	if(character_count != 9)return 0;
	character_count=0;
	i++;	
	//Fifth portion is Latitude direction
	for(int j=0;gps_string[i] != ','; j++){
		GPS.Latitude.direction=gps_string[i];
		i++;
	}
	i++;	
	//Sixth portion is Long.
	for(int j=0;gps_string[i] != ','; j++){
		GPS.Longitude.position[j]=gps_string[i];
		i++;
		character_count++;
	}
	//Make sure we received 10 characters for longitude
	if(character_count != 10)return 0;
	character_count=0;
	
	i++;			
	//Seventh portion is Long direction
	while(gps_string[i] != ','){
		GPS.Longitude.direction=gps_string[i];
		i++;
	}
	i++;		
	//8th portion dismissed
	while(gps_string[i] != ',')i++;
	i++;				
	//9th portion dismissed
	while(gps_string[i] != ',')i++;
	i++;				
	//10th portion is Date
	for(int j=0;gps_string[i] != ','; j++){
		GPS.Date[j]=gps_string[i];
		i++;
	}
	
	return 1;
}
*/
//Usage: saveData(log_data, log_data_index);
//Inputs: char *buf - character array to be saved
//		  int buf_size - size of character array
//Output: buffer array is saved to LOG_FILE
//Description: Saves the buf character array to the SD card.
//CONDITIONS: LOG_FILE must be initialized to the handle of an open file.
void saveData(struct fat16_file_struct **fd, const char * const buf, const int buf_size)
{
	int error=0;
	
	if((buf_size > 0) && (*fd != NULL)){
		//Try writing the data to the card up to 10 times.
		while(error<10){
			if(fat16_write_file(*fd, (const unsigned char*)buf, buf_size) < 0)error+=1;
			else break;
			delay_ms(100);
		}
		//If we've tried writing the data 10 times and still haven't succeeded, reset the device.
		if(error==10)reset();
		
		error=0;
		//Try syncing the card up to 10 times
		while(error<10){
			if(!sd_raw_sync())error+=1;
			else break;
			delay_ms(100);
		}
		//If we've tried syncing 10 times and still haven't succeeded, reset the device
		if(error==10)reset();
	}
}

//Usage: itoa(batt_level, log_buffer)
//Inputs: int n - integer to convert
//Outputs:  char s[]-contains ascii rerpresentation of 'n'
/* itoa:  convert n to characters in s */
void itoa(int n, char s[])
{
    int i=0;

    do {       /* generate digits in reverse order */
        s[i++] = n % 10 + '0';   /* get next digit */
    } while ((n /= 10) > 0);     /* delete it */

    s[i] = '\0';
    reverse(s);
}

//Usage: reverse(s);
//Inputs: char s[] - contains a character string
//Outputs: char s[] - Reversed the order of original characters
/* reverse:  reverse string s in place */
void reverse(char s[])
{
    int c, i, j;

    for (i = 0, j = strlen(s)-1; i<j; i++, j--) {
        c = s[i];
        s[i] = s[j];
        s[j] = c;
    }
}
/*
//Usage: None (Automatically Called by FW)
//Inputs: None
//This function is a global interrupt called by a match on the Timer 1 match.  The interrupt
// is responsible for determining if a button has been pressed or if the screen has been rotated
// and setting the appropriate global flag if either has occured.
static void ISR_Timer0(void)
{
	//Interrupt Code Here
	read_sensors=1;
	log_count++;
	
	//Update the Status LED
	led_blink++;
	if(led_blink > TIMER_FREQ)led_blink=0;
	if(led_blink > (TIMER_FREQ % 10))LED_OFF();
	else LED_ON();
	
	T0IR = 0xFF;						//Clear the timer interrupt
	VICVectAddr =0;						//Update the VIC priorities
}
*/
//Usage: accel = get_adc_1(CHANNEL);
//Inputs: int channel - integer corresponding to the ADC channel to be converted
//Outputs: None
//Description: Returns the raw analog to digital conversion of the input channel.  
int get_adc_1(char channel)
{
    int val;
    AD1CR = 0;
    AD1GDR = 0;

    //AD1CR = 0x00200600 | channel;
	AD1CR = 0x00200E00 | channel;
    AD1CR |= 0x01000000;
    do
    {
        val = AD1GDR;                   // Read A/D Data Register
    }
    while ((val & 0x80000000) == 0);  //Wait for the conversion to complete
    val = ((val >> 6) & 0x03FF);  //Extract the A/D result

    return val;
}

//Usage: reset();
//Inputs: None
//Description: Resets the LPC2148
void reset(void)
{
    // Intentionally fault Watchdog to trigger a reset condition
    WDMOD |= 3;
    WDFEED = 0xAA;
    WDFEED = 0x55;
    WDFEED = 0xAA;
    WDFEED = 0x00;
}
/*
static void ISR_EINT2(void){
	VICIntEnClr = (1<<16);			//Temporarily disable EINT2 Interrupts
	EXTINT |= (1<<2);				//Clear the interrupt bit in EINT2
	
	wake_event=ACCELEROMETER_WAKE;			//Tell the main code that a free-fall has been detected!
	
	VICIntEnable = (1<<16);		//Re-enable the EINT2 Interrupts
	VICVectAddr =0;		//Update the VIC priorities
}
*/
/*
void initializeGps(void){
	//Initialize the GPS receiver
	GPSon();
	delay_ms(200);

	disable_all_gps_msgs();
	delay_ms(200);
	
	enable_waas();
	delay_ms(200);
}
*/

#include <cairo.h>
#include <gdk/gdkkeysyms.h>
#include <gtk/gtk.h>
#include <stdlib.h>
#include <rtl-sdr.h>
#include <common/mavlink.h> //For Serial Parsing

// Standard includes
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>

// Serial includes  /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#ifdef __linux
#include <sys/ioctl.h>
#endif

// Latency Benchmarking
#include <sys/time.h>
#include <time.h>

#define FILE_BUFFER_SIZE			256
#define AUX_STRING_SIZE				256
#define EXTRA_DATA_SIZE				1
#define RAW_DATA_FILENAME 			"/media/RAW_DATA/rct/RAW_DATA_"
#define META_FILE_PREFIX 			"/media/RAW_DATA/rct/RUN_META_"
#define FILE_COUNTER_PATH 			"/media/RAW_DATA/rct/fileCount"
#define CONFIG_FILE_PATH 			"/media/RAW_DATA/rct/cfg"

// Signal Buffer Arrays
unsigned char *time_buffer 			= NULL;	// Time domain buffer
unsigned char *rawFileBuffer 		= NULL;	// Data Storage Time domain buffer

// File Handling Variables
FILE *fileStream;
fpos_t filePos; 
char fileBuffer[FILE_BUFFER_SIZE]; 
char meta_file_path[FILE_BUFFER_SIZE]; 
int currentRun						= 0; 
int currentRunFile					= 1; 

// Config Variables 
const int valid_gain_values[] 		= { 0, 9, 14, 27, 37, 77, 87, 125, 144, 157, 166, 197, 207, 229, 254, 280, 297, 328, 338, 364, 372, 386, 402, 421, 434, 439, 445, 480, 496 };
const int max_gain_index 			= 28;
int  current_gain_index 			= 0;
unsigned int center_freq 			= 0;
unsigned int samp_rate 				= 0;
int timeout_interrupt				= 0; 
int goal_signal_amplitude 			= 0; 
int controller_coef 				= 0; 
int number_frames_per_file 			= 0; 

unsigned int time_buffer_len 		= 0;
unsigned int raw_file_buffer_len 	= 0;
unsigned int dev_index 				= 0;

// Aux Variables
int frame_counter					= 0; 
int maxFindAuxInt					= 0; 
int intAux							= 0;
char auxString[AUX_STRING_SIZE]; 
int device_count;
int r;

// GUI variables
unsigned int tid 					= 0;
cairo_t *cr 						= NULL;
GtkWidget *window 					= NULL;
rtlsdr_dev_t *dev 					= NULL;

//Serial Variables

struct timeval tv;	    ///< System time

// Settings
int sysid = 42;             ///< The unique system id of this MAV, 0-127. Has to be consistent across the system
int compid = 110;
int serial_compid = 0;
bool silent = false;        ///< Wether console output should be enabled
bool verbose = false;       ///< Enable verbose output
bool debug = false;         ///< Enable debug functions and output
int fd;

// Message #24  GPS_RAW_INT 
uint8_t    ap_fixtype = 3;               //   0= No GPS, 1 = No Fix, 2 = 2D Fix, 3 = 3D Fix
uint8_t    ap_sat_visible = 0;           // numbers of visible satelites

// FrSky Taranis uses the first recieved lat/long as homeposition. 
int32_t    ap_latitude = 0;              // 585522540;
int32_t    ap_longitude = 0;             // 162344467;
int32_t    ap_gps_altitude = 0;          // 1000 = 1m

// These are special for FrSky
int32_t   adc2 = 0;               // 100 = 1.0V
int32_t   vfas = 0;             // 100 = 1,0V
int32_t   gps_status = 0;       // (ap_sat_visible * 10) + ap_fixtype
uint8_t   ap_cell_count = 0;

// ******************************************
uint8_t     MavLink_Connected;
uint8_t     buf[MAVLINK_MAX_PACKET_LEN];

uint16_t  hb_count;

unsigned long MavLink_Connected_timer;
unsigned long hb_timer;
unsigned long acc_timer;

int bytesRead;
uint8_t cp;
mavlink_message_t message;
mavlink_status_t status;
uint8_t msgReceived = false;


void destroy(GtkWidget *widget, gpointer data);

void init_memmory();

void update_meta(); 

void clean_up_memmory();

void load_files();

int loadParameter(); 

int timeout_cb(gpointer darea);

void compile_data(); 

void store_data(); 

gboolean read_rtlsdr();

void adjust_gain();

void setup_rtlsdr();

int initSerial();

int open_port(const char* port);

bool setup_port(int fd, int baud, int data_bits, int stop_bits, bool parity, bool hardware_control);

int serial_read();

void close_port(int fd);

int main(int argc, char *argv[]){

	load_files(); 

    setup_rtlsdr();

    init_memmory();

    fd = initSerial();

    tid = g_timeout_add(timeout_interrupt, timeout_cb, window);
    
	gtk_main();
    
    g_source_remove(tid);    

	clean_up_memmory();

	close_port();
    
    return 0;
}

void destroy(GtkWidget *widget, gpointer data){
	cairo_destroy(cr);
	gtk_main_quit();
}

int timeout_cb(gpointer darea){ 

	// Gets samples from rtlsdr
	if (read_rtlsdr())
        return FALSE;  // error handling
    
	compile_data();
	
	printf("Current Frame: %03d Gain: %03d\n", frame_counter, valid_gain_values[(int)current_gain_index]);

	frame_counter++; 
	if(frame_counter >= number_frames_per_file){
		printf("FILE: %06d\n", currentRunFile); 
		store_data(); 
		frame_counter = 0; 
		update_meta(); 
		currentRunFile++; 
	}

	adjust_gain();
	
	return TRUE;
}

void compile_data(){
	for (int i = 0; i < time_buffer_len/2; i++){
		rawFileBuffer[frame_counter*(time_buffer_len+EXTRA_DATA_SIZE)+2*i] 		= time_buffer[2*i];
		rawFileBuffer[frame_counter*(time_buffer_len+EXTRA_DATA_SIZE)+2*i+1] 	= time_buffer[2*i+1];
	}
	rawFileBuffer[frame_counter*(time_buffer_len+EXTRA_DATA_SIZE)+time_buffer_len] = current_gain_index; 
}

void store_data(){
	sprintf(auxString, "%s%06d_%06d", RAW_DATA_FILENAME, currentRun, currentRunFile);
	fileStream = fopen (auxString, "wb");
	fwrite(rawFileBuffer, sizeof(unsigned char), raw_file_buffer_len*sizeof(unsigned char), fileStream);
	fclose(fileStream);
}

void init_memmory(){
	time_buffer 		= malloc(time_buffer_len * sizeof(unsigned char));
	rawFileBuffer 		= malloc(raw_file_buffer_len * sizeof(unsigned char));
}

void update_meta(){

	sprintf(meta_file_path, "%s%06d", META_FILE_PREFIX, currentRun);
	fileStream = fopen (meta_file_path, "wb");
	fprintf(fileStream, "center_freq: %d \nsamp_rate: %d \ntimeout_interrupt: %d \ngoal_signal_amplitude: %d \ncontroller_coef: %d \nnumber_frames_per_file: %d \ncurrentRunFile: %d \n ",
	 center_freq, samp_rate, timeout_interrupt, goal_signal_amplitude, controller_coef, number_frames_per_file, currentRunFile);
	fclose(fileStream);
}

void clean_up_memmory(){
	rtlsdr_close(dev);
	free(time_buffer);
	free(rawFileBuffer);
}

void load_files(){   

	fileStream 				= fopen(FILE_COUNTER_PATH, "r+");
	fgetpos(fileStream, &filePos); 
	currentRun 				= loadParameter(); 
	currentRun++; 
	fsetpos(fileStream, &filePos); 
	fprintf(fileStream, "currentRun: %d \n", currentRun);
 	fclose(fileStream);

	fileStream 						= fopen(CONFIG_FILE_PATH, "r");
	center_freq 					= loadParameter();
	samp_rate 						= loadParameter();
	timeout_interrupt 				= loadParameter();
	goal_signal_amplitude 			= loadParameter();
	controller_coef 				= loadParameter();
	number_frames_per_file 			= loadParameter();
 	fclose(fileStream);

	time_buffer_len 				= (int)(((float)timeout_interrupt*samp_rate)/500); 
	raw_file_buffer_len 			= number_frames_per_file*(time_buffer_len+EXTRA_DATA_SIZE);

}

int loadParameter(){
    if (fgets(fileBuffer, FILE_BUFFER_SIZE, fileStream) == NULL) {
        fprintf(stderr, "ERROR Reading Data Config File!\n");
		exit(1);
    }
    intAux = 0; 
    while(fileBuffer[intAux] != ':')
    	intAux++;
    if(intAux >= FILE_BUFFER_SIZE){
        fprintf(stderr, "ERROR Reading Data Config File!\n");
		exit(1);
    }
    return atoi(fileBuffer+intAux+1); 
}

gboolean read_rtlsdr(){

    int n_read;
	
	for (int i = 0; i < time_buffer_len; i++)
		*(time_buffer+i) = 0; 

    if (rtlsdr_read_sync(dev, time_buffer, time_buffer_len, &n_read) < 0) {
        fprintf(stderr, "WARNING: sync read failed. time_buffer_len: %d.\n", time_buffer_len);
    	return TRUE;
    }

    if ((unsigned int)n_read < time_buffer_len) {
        fprintf(stderr, "Short read (%d / %d), samples lost, exiting!\n", n_read, time_buffer_len);
    	return TRUE;
    }

    return FALSE;
}

void adjust_gain(){
    
    //  Finding max signal value
    maxFindAuxInt = 0; 
    for (int i = 0; i < time_buffer_len; i+= 2)
    	if(time_buffer[i] > maxFindAuxInt)
    		maxFindAuxInt = time_buffer[i]; 
    maxFindAuxInt -= 128; 

	// Gain controller
	current_gain_index += (goal_signal_amplitude - abs(maxFindAuxInt))/controller_coef; 
		
	// Ensuring array bounds
	if(current_gain_index > max_gain_index)
		current_gain_index = max_gain_index; 
	else if(current_gain_index < 0)
		current_gain_index = 0; 
	// Setting gain value
	if (rtlsdr_set_tuner_gain(dev, valid_gain_values[(int)current_gain_index]) < 0)
		fprintf(stderr, "WARNING: Failed to set up fixed gain.\n");

}

void setup_rtlsdr(){

	device_count = rtlsdr_get_device_count();
	if (!device_count){
		fprintf(stderr, "No supported devices found.\n");
		exit(1);
	}

	r = rtlsdr_open(&dev, dev_index);
	if (r < 0){
		fprintf(stderr, "Failed to open rtlsdr device #%d.\n", dev_index);
		exit(1);
	}

	r = rtlsdr_set_sample_rate(dev, samp_rate);
	if (r < 0)
		fprintf(stderr, "WARNING: Failed to set sample rate.\n");

	r = rtlsdr_set_center_freq(dev, center_freq);
	if (r < 0)
		fprintf(stderr, "WARNING: Failed to set center freq.\n");

	// Setting gain mode (auto(0) or manual(1))
	r = rtlsdr_set_tuner_gain_mode(dev, 1);
	if (r < 0)
		fprintf( stderr, "WARNING: Failed to enable manual gain.\n");

	// Setting gain value
	r = rtlsdr_set_tuner_gain(dev, valid_gain_values[(int)current_gain_index]);
	if (r < 0)
		fprintf(stderr, "WARNING: Failed to set up fixed gain.\n");

	r = rtlsdr_reset_buffer(dev);
	if (r < 0)
		fprintf(stderr, "WARNING: Failed to reset buffers.\n");

}

int initSerial() {

	/* default values for arguments */
	char *uart_name = (char*)"/dev/ttyUSB0";
	int baudrate = 57600;
	const char *commandline_usage = "\tusage: %s -d <devicename> -b <baudrate> [-v/--verbose] [--debug]\n\t\tdefault: -d %s -b %i\n";

	if (!silent) printf("Trying to connect to %s.. ", uart_name);
	fflush(stdout);

	fd = open_port(uart_name);
	if (fd == -1)
	{
		if (!silent) printf("failure, could not open port.\n");
		exit(EXIT_FAILURE);
	}
	else
	{
		if (!silent) printf("success.\n");
	}
	if (!silent) printf("Trying to configure %s.. ", uart_name);

	bool setup = setup_port(fd, baudrate, 8, 1, false, false);

	if (!setup)
	{
		if (!silent) printf("failure, could not configure port.\n");
		exit(EXIT_FAILURE);
	}
	else
	{
		if (!silent) printf("success.\n");
	}

	int noErrors = 0;
	if (fd == -1 || fd == 0)
	{
		if (!silent) fprintf(stderr, "Connection attempt to port %s with %d baud, 8N1 failed, exiting.\n", uart_name, baudrate);
		exit(EXIT_FAILURE);
	}
	else
	{
		if (!silent) fprintf(stderr, "\nConnected to %s with %d baud, 8 data bits, no parity, 1 stop bit (8N1)\n", uart_name, baudrate);
	}
	
	if(fd < 0)
	{
		exit(noErrors);
	}
	return fd;
}

int open_port(const char* port)
{
	int fd;
	fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1)
	{
		return(-1);
	}
	else
	{
		fcntl(fd, F_SETFL, 0);
	}
	return (fd);
}

bool setup_port(int fd, int baud, int data_bits, int stop_bits, bool parity, bool hardware_control)
{	
	struct termios  config;
	if(!isatty(fd))
	{
		fprintf(stderr, "\nERROR: file descriptor %d is NOT a serial port\n", fd);
		return false;
	}
	if(tcgetattr(fd, &config) < 0)
	{
		fprintf(stderr, "\nERROR: could not read configuration of fd %d\n", fd);
		return false;
	}
	
	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
	
	config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);

	#ifdef OLCUC 
  		config.c_oflag &= ~OLCUC; 
	#endif

  	#ifdef ONOEOT
  		config.c_oflag &= ~ONOEOT;
  	#endif

	config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	config.c_cflag &= ~(CSIZE | PARENB);
	config.c_cflag |= CS8;
	
	config.c_cc[VMIN]  = 1;
	config.c_cc[VTIME] = 10; 
	
	switch (baud)
	{
		case 1200:
			if (cfsetispeed(&config, B1200) < 0 || cfsetospeed(&config, B1200) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 1800:
			cfsetispeed(&config, B1800);
			cfsetospeed(&config, B1800);
			break;
		case 9600:
			cfsetispeed(&config, B9600);
			cfsetospeed(&config, B9600);
			break;
		case 19200:
			cfsetispeed(&config, B19200);
			cfsetospeed(&config, B19200);
			break;
		case 38400:
			if (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 57600:
			if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 115200:
			if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;

		// These two non-standard (by the 70'ties ) rates are fully supported on
		// current Debian and Mac OS versions (tested since 2010).
		case 460800:
			if (cfsetispeed(&config, 460800) < 0 || cfsetospeed(&config, 460800) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 921600:
			if (cfsetispeed(&config, 921600) < 0 || cfsetospeed(&config, 921600) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		default:
			fprintf(stderr, "ERROR: Desired baud rate %d could not be set, aborting.\n", baud);
			return false;
			
			break;
	}
	
	if(tcsetattr(fd, TCSAFLUSH, &config) < 0)
	{
		fprintf(stderr, "\nERROR: could not set configuration of fd %d\n", fd);
		return false;
	}
	return true;
}

int serial_read()
{	
	mavlink_status_t lastStatus;
	lastStatus.packet_rx_drop_count = 0;
	
	bytesRead=read(fd, &cp, 1);
	
	if (bytesRead > 0)
	{
		msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);
		if (lastStatus.packet_rx_drop_count != status.packet_rx_drop_count)
		{
			if (verbose || debug) printf("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
			if (debug)
			{
				unsigned char v=cp;
				fprintf(stderr,"%02x ", v);
			}
		}
		lastStatus = status;
	}
	else
	{
		if (!silent) fprintf(stderr, "ERROR: Could not read from fd %d\n", fd);
	}
	
	// If a message could be decoded, handle it
	if(msgReceived)
		{

			if(message.msgid == MAVLINK_MSG_ID_GPS_RAW_INT)   // 24
		    	{
			     ap_fixtype = mavlink_msg_gps_raw_int_get_fix_type(&message);                         // 0 = No GPS, 1 =No Fix, 2 = 2D Fix, 3 = 3D Fix``
			     ap_sat_visible =  mavlink_msg_gps_raw_int_get_satellites_visible(&message);          // numbers of visible satelites
			     gps_status = (ap_sat_visible*10) + ap_fixtype; 
			     
			     if(ap_fixtype == 3)  
			     {
			         ap_latitude = mavlink_msg_gps_raw_int_get_lat(&message);
			         ap_longitude = mavlink_msg_gps_raw_int_get_lon(&message);
			         ap_gps_altitude = mavlink_msg_gps_raw_int_get_alt(&message);                       // 1m = 1000
			     }
			 }
		}
	}
   return 0;
}

void close_port(int fd)
{
	close(fd);
}

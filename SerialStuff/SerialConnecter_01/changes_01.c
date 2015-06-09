//***************************************************************THIS STUFF GOES ABOVE MAIN*******************************************************************
#define GPS_MSG_LENGTH 			22

char *serialPortFilename = "/dev/ttyACM0"; // Teensy

// Open Serial Port
int serPort;  // Send to Teensy

char buff[BUFFER_SIZE];
int num_bytes_read;
struct termios options;

static void print_gps();

//*************************************************************************************************************************************************************



int main(int argc, char *argv[]){


//*********************************************************************GOES IN MAIN*****************************************************************************


#ifdef TEENSY_CONNECTED  //Connect to the Teesny over serial
    (void)printf("Starting up\n");
    serPort = open(serialPortFilename, O_RDWR | O_NOCTTY);
    (void)printf("Open teensy port %s result %d\n", serialPortFilename, serPort);
    if( serPort == -1 )
    {
        (void)printf("Teensy Not Connected");
    }
    else
    {
    	tcgetattr(serPort, &options);

	    cfsetispeed(&options, B57600);

	    cfsetospeed(&options, B57600);

	    options.c_cflag |= (CLOCAL |CREAD);

	    tcsetattr(serPort, TCSANOW, &options);

	    (void)printf("Connected to Teensy\n");
    }
#endif
//***************************************************************************************************************************************************************





   
//******************************************************************************BELOW MAIN**************************************************************************
//Prints out GPS coordinates to the screen
static void print_gps(){ 
#ifdef TEENSY_CONNECTED   

        num_bytes_read = read(serPort, &buff, BUFFER_SIZE);
	if(num_bytes_read != GPS_MSG_LENGTH)
            (void)printf("%s", buff); //print gps in latitude,longitude,altitude
#endif
} 

//****************************************************************************************************************************************************************

#include <fcntl.h>
#include <termios.h>
#include <strings.h>
#include <stdlib.h>

#define GPS_CONNECTED
#define GPS_MSG_LENGTH	22
#define BUFFER_SIZE  100

char *serialPortFilename = "/dev/ttyUSB0"; 
int serPort; 

char buff[BUFFER_SIZE];
int num_bytes_read;
struct termios options;

static void print_gps();

int main(int argc, char *argv[]){

#ifdef GPS_CONNECTED 
    (void)printf("Starting up\n");
    serPort = open(serialPortFilename, O_RDWR | O_NOCTTY);
    (void)printf("Open GPS  port %s result %d\n", serialPortFilename, serPort);
    if( serPort == -1 )
    {
        (void)printf("GPS Not Connected");
    }
    else
    {
    	tcgetattr(serPort, &options);

	    cfsetispeed(&options, B57600);

	    cfsetospeed(&options, B57600);

	    options.c_cflag |= (CLOCAL |CREAD);

	    tcsetattr(serPort, TCSANOW, &options);

	    (void)printf("Connected to GPS\n");

	while(1){
		print_gps();
		
	}
    }
#endif
}
   
static void print_gps(){ 
#ifdef GPS_CONNECTED   

        num_bytes_read = read(serPort, &buff, BUFFER_SIZE);
	if(num_bytes_read != GPS_MSG_LENGTH)
            (void)printf("%s", buff); //print gps in latitude,longitude,altitude
#endif
} 



all: mavlinkSerial

mavlinkSerial: mavlinkSerial.o
	gcc mavlinkSerial.o -o mavlinkSerial

mavlinkSerial.o: mavlinkSerial.c
	gcc -I mavlink/include/mavlink/v1.0 -c mavlinkSerial.c

clean:
	rm -rf *o mavlinkSerial
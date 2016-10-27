all: collarTracker_wGPS

collarTracker_wGPS: collarTracker_wGPS.o
	gcc collarTracker_wGPS.o -o collarTracker_wGPS


collarTracker_wGPS.o: collarTracker_wGPS.c
	gcc -I mavlink/include/mavlink/v1.0 -c collarTracker_wGPS.c

clean:
	rm -rf *o collarTracker_wGPS

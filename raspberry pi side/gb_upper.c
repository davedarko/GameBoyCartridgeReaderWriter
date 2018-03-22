#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>


// To compile on Ubuntu or Mac OS X with xcode:
//   gcc -O3 -Wall -o gb_upper gb_upper.c

// call 
// ./gb_upper /dev/cu.usbmodem1421

const char *begin_string="ROM_START";
const char *end_string="ROM_END";

#define END_LEN 7

int main(int argc, char **argv)
{
	FILE *fp;
	int c;
	fp = fopen("ROMS/HaD.gb", "r");

	int port;
	struct termios settings;

	if(NULL == fp) 
	{
		printf("Konnte Datei \"ROMS/tetris.gb\" nicht öffnen!\n");
		return EXIT_FAILURE;
	}

	port = open(argv[1], O_RDWR);
	if (port < 0) 
	{
		fprintf(stderr, "Unable to open %s\n", argv[1]);
		return 1;
	}

	// Configure the port
	tcgetattr(port, &settings);
	cfmakeraw(&settings);
	tcsetattr(port, TCSANOW, &settings);

	while(1)
	{
		c = fgetc(fp);
		if( feof(fp) )
		{
			break ;
		}
		write(port, &c, 1);
		//printf("%c", c);
		usleep(500);
	}

	// printf("\n\n buffer %s \n",buf);
	
	fclose(fp);
	close(port);

	return 0;
}


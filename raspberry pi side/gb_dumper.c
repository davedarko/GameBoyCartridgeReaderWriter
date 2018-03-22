#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

// To compile on Ubuntu or Mac OS X with xcode:
//   gcc -O3 -Wall -o gb_dumper gb_dumper.c

const char *begin_string="ROM_START";
const char *end_string="ROM_END";

#define END_LEN 7

int main(int argc, char **argv)
{
	FILE *datei;
	datei = fopen("dump.gb", "w+b");
	if(NULL == datei) {
		printf("Konnte Datei \"test.txt\" nicht öffnen!\n");
		return EXIT_FAILURE;
	}

	char buf[1024];
	int port;
	long n=0;
	struct termios settings;

	if (argc < 2) {
		fprintf(stderr, "Usage:   gb_dumper <port>\n");
		fprintf(stderr, "Example: gb_dumper /dev/cu.usbmodem1241\n");
		return 1;
	}

	// Open the serial port
	port = open(argv[1], O_RDONLY);
	if (port < 0) 
	{
		fprintf(stderr, "Unable to open %s\n", argv[1]);
		return 1;
	}

	// Configure the port
	tcgetattr(port, &settings);
	cfmakeraw(&settings);
	tcsetattr(port, TCSANOW, &settings);
	
	// Read data until we get a see the end string or get an error
	printf("Reading from %s\n", argv[1]);
	while (1) 
	{
		n = read(port, buf, sizeof(buf));
		int write_to_file = 1;
		if (n < 1) 
		{
			fprintf(stderr, "error reading from %s\n", argv[1]);
			break;
		}
		if (
			n < 50 && 
			memcmp(buf, begin_string, strlen(begin_string)) == 0
		) {
			printf("read: %ld (begin string)\n", n);
			write_to_file = 0;
			continue;
		}

		if (
			n < sizeof(buf) && 
			n >= END_LEN &&
		    memcmp(buf + n - END_LEN, end_string, END_LEN) == 0
	  	) {
		   	fwrite(&buf, n - END_LEN, 1, datei);
			printf("(end string)\n");
			break;
		}

		if (write_to_file) 
		{
			fwrite(&buf, n, 1, datei);
			printf(".");
		}
	}

	// printf("\n\n buffer %s \n",buf);
	
	fclose(datei);
	close(port);

	return 0;
}


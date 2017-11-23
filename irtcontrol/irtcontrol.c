#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>

int fd;

int getResponse(char * buffer) {
	char *bufptr = buffer; 
	while (bufptr < buffer + 256) {
		if (read(fd, bufptr, 1) <= 0) {
			// read timeout
			return -1;
		}
		bufptr++;
		if (bufptr >= buffer + 2 && bufptr[-2] == '\r' && bufptr[-1] == '\n') {
			bufptr[-2] = 0;
			return 0;
		}
	}
	// buffer overflow
	return -2;
}


int sendCommand(const char * command) {

	while(*command != 0) {
		write(fd, command++, 1);
	}
	write(fd, "\r", 1);


	int res;
	char buffer[256];
	while((res = getResponse(buffer)) == 0) {
		if (strcmp(buffer, "OK") == 0) {
			return 0;
		} else if (sscanf(buffer, "ERROR %02X", &res) == 1) {
			return res;
		} else {
			puts(buffer);
		}
	}
	return res;
}

int main(int argc, char *argv[]) {

	if (argc < 3) {
		printf("Usage: ircontrol port [*repeat] command ...\n");
		return -4;
	}

	fd = open(argv[1], O_RDWR | O_NOCTTY);
	if (fd == -1) {
		perror("Unable to open serial port");
		return -3;
	}

	struct termios options;
	tcgetattr(fd, &options);
	cfsetispeed(&options, B19200);
	cfsetospeed(&options, B19200);
	cfmakeraw(&options);
	options.c_cc[VMIN] = 0;
	options.c_cc[VTIME] = 30;
	tcsetattr(fd, TCSANOW, &options);

	int i;
	int multiply = 1;
	for (i = 2; i < argc; i++) {
		int tmp;
		if (sscanf(argv[i], "*%d", &tmp) == 1) {
			multiply = tmp;
		} else {
			int j;
			for (j = 0; j < multiply; j++) { 
				int res = sendCommand(argv[i]);
				if (res != 0) {
					switch (res) {
					case -1:
						fprintf(stderr, "Device read timeout\n");
						break;
					case -2:
						fprintf(stderr, "Read buffer overflow (no CR LF)\n");
						break;
					default:
						if (res < 0) {
							fprintf(stderr, "Unknown error %d\n", res);
						} else {
							fprintf(stderr, "Device error %02X\n", res);
						}
						break;
					}
					close(fd);
					return res;
				}
			}
			multiply = 1;
		}
	}

	close(fd);
	return 0;
}


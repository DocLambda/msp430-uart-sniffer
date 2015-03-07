/* --COPYRIGHT--,BSD 3-clause
 * 
 * Copyright (c) 2015 -- Sven Rebhan <odinshorse@googlemail.com>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 * --COPYRIGHT--
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <libgen.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#define DATALEN		30

struct sniffer_output
{
	struct timespec  logtime;
	unsigned char    channel;
	char             data[DATALEN];
	int              cursor;
};

static int fd = -1;

static int sniffer_open(const char *dev)
{
	int fd;

	/* Open the device for read and write and prevent it from
	 * becoming a control TTY.
	 */
	fd = open(dev, O_RDONLY | O_NOCTTY);
	if (fd < 0)
		return fd;

	/* Make sure the given device is a tty. */
	if (!isatty(fd))
	{
		errno = ENOTTY;
		return -1;
	}

	return fd;
}

static int sniffer_setup(int fd)
{
	struct termios tty_attr;

	/* Setup the port for communication. */
	tcgetattr(fd, &tty_attr);
	tcflush(fd, TCIOFLUSH);
	cfsetspeed(&tty_attr, B9600);
	cfmakeraw(&tty_attr);
	tcflush(fd, TCIOFLUSH);

	/* Set port to blocking */
	tty_attr.c_cc[VMIN]  = 1;
	tty_attr.c_cc[VTIME] = 0;

	/* Actually set the new configuration */
	if (tcsetattr(fd, TCSANOW, &tty_attr))
		return errno;

	/* Make sure settings are actually transmitted */
	return tcdrain(fd);
}

static void sniffer_close(int fd)
{
	/* Clear all remaining data on port */
	tcflush(fd, TCIOFLUSH);
	close(fd);
}

static void on_exit_handler(void)
{
	if (fd >= 0)
		sniffer_close(fd);
}

static void reset_output(struct sniffer_output *output)
{
	/* Reset output message */
	memset(output->data, 0, DATALEN);
	
	/* Reset cursor */
	output->cursor = 0;
}

static void log_output_header(struct sniffer_output *output, unsigned char channel)
{
	clock_gettime(CLOCK_REALTIME, &output->logtime);
	output->channel = channel;
}

static void log_output_data(struct sniffer_output *output, char c)
{
	output->data[output->cursor++] = c;
}

static int output_is_full(struct sniffer_output *output)
{
	return (output->cursor >= DATALEN);
}

static void print_output(struct sniffer_output *output)
{
	int extra = 0;
	int i;
	
	/* Print log time and channel */
	printf("\r%.3f [%u]: ", output->logtime.tv_sec + (float)output->logtime.tv_nsec*10E-9, output->channel);

	/* Print 'normal' output */
	for (i = 0; i < output->cursor; i++)
	{
		if (isprint(output->data[i]))
			printf("%c", output->data[i]);
		else
		{
			printf("<%02x>", output->data[i]);
			extra++;
		}
	}
	for (; i < DATALEN - 3*extra; i++)
	{
		printf(" ");
	}
	printf(" | ");

	/* Print 'hex' output */
	for (i = 0; i < output->cursor; i++)
	{
		printf("%02x ", output->data[i]);
	}
	for (; i < DATALEN; i++)
	{
		printf("   ");
	}

	fflush(stdout);
}

int main(int argc, char **argv)
{
	struct sniffer_output   output;
	fd_set         fdset;
	char          *dev;
	int            t = 10;	// 1 second
	struct timeval timeout;
	unsigned char  c;
	unsigned char  channel;
	unsigned char  data;
	unsigned char  lastchannel = 255;
	int            retval;
	
	if (argc < 2)
	{
		printf("Usage: %s <device> [timeout in 1/10s]\n", basename(argv[0]));
		exit(EXIT_FAILURE);
	}
	dev = argv[1];
	timeout.tv_sec  = 1;	// seconds
	timeout.tv_usec = 0;	// microseconds
	if (argc > 2)
		t = atoi(argv[2]);

	printf("Using device %s with timeout of %.1f seconds...\n", dev, t*0.1f);
	
	/* Open sniffer device */
	fd = sniffer_open(dev);
	if (fd < 0)
	{
		fprintf(stderr, "Opening sniffer at %s failed: %s\n", dev, strerror(errno));
		exit(EXIT_FAILURE);
	}
	atexit(on_exit_handler);
	
	retval = sniffer_setup(fd);
	if (retval)
	{
		fprintf(stderr, "Setting up sniffer at %s failed: %s\n", dev, strerror(retval));
		exit(EXIT_FAILURE);
	}
	
	while(1)
	{
		/* Fill the select filedescriptor set */
		FD_ZERO(&fdset);
		FD_SET(fd, &fdset);
	
		/* Set timeout */
		timeout.tv_sec  =  t / 10; 		// seconds
		timeout.tv_usec = (t % 10) * 10E5;	// microseconds

		/* Wait for data to arrive */
		retval = select(FD_SETSIZE, &fdset, NULL, NULL, &timeout);
		if (retval < 0)				// Error
		{
			fprintf(stderr, "\nWaiting for sniffer at %s failed: %s\n", dev, strerror(errno));
			exit(EXIT_FAILURE);
		}
		else if (retval == 0)			// Timeout
		{
			lastchannel = 255;
			errno = 0;
		}
		else					// Data available
		{
			if (read(fd, &c, 1*sizeof(char)) < 1)
			{
				fprintf(stderr, "\nReading from sniffer at %s failed: %s\n", dev, strerror(errno));
				exit(EXIT_FAILURE);
			}
			data    = c & 0x7F;
			channel = c >> 7;
			if (channel != lastchannel)
			{
				printf("\n");
				reset_output(&output);
				log_output_header(&output, channel);
			}
			lastchannel = channel;
			log_output_data(&output, data);
			print_output(&output);
			if (output_is_full(&output))
			{
				lastchannel = 255;
			}
		}
	}
	
	/* Close sniffer device */
	sniffer_close(fd);
	fd = -1;
	
	return EXIT_SUCCESS;
}

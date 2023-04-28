/*
 *
 * Copyright (C) 
 *
 * Contact: Hari Nagalla (hnagalla@ti.com)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <syslog.h>
#include <termios.h>
#include <time.h>
#include <poll.h>
#include <sys/time.h>
#include <sys/param.h>
#include <sys/ioctl.h>

#define uint16_t unsigned short
#define uint8_t unsigned char

#define MAKEWORD(a, b)  ((uint16_t)(((uint8_t)(a)) | ((uint16_t)((uint8_t)(b))) << 8))

#define TI_MANUFACTURER_ID  13



int uart_speed(int s)
{
    switch (s) {
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    case 230400:
        return B230400;
    case 460800:
        return B460800;
    case 500000:
        return B500000;
    case 576000:
        return B576000;
    case 921600:
        return B921600;
    case 1000000:
        return B1000000;
    case 1152000:
        return B1152000;
    case 1500000:
        return B1500000;
    case 2000000:
        return B2000000;
    case 2500000:
        return B2500000;
    case 3000000:
        return B3000000;
    case 3500000:
        return B3500000;
    // case 3710000:
      //  return B3710000;
    case 4000000:
        return B4000000;
    default:
        return B57600;
    }
}

int set_speed(int fd, struct termios *ti, int speed)
{
    if (cfsetospeed(ti, uart_speed(speed)) < 0)
        return -errno;

    if (cfsetispeed(ti, uart_speed(speed)) < 0)
        return -errno;

    if (tcsetattr(fd, TCSANOW, ti) < 0)
        return -errno;

}


int init_uart(char *dev)
{
	struct termios ti;
	int fd, i;


	fd = open(dev, O_RDWR | O_NOCTTY);
	if (fd < 0)
	{
		fprintf(stdout,"Can't open serial port");
		return -1;
	}

	tcflush(fd, TCIOFLUSH);

	if (tcgetattr(fd, &ti) < 0) {
		fprintf(stdout,"Can't get port settings \n");
		goto fail;
	}

	cfmakeraw(&ti);

	ti.c_cflag |= CLOCAL;
	ti.c_cflag |= CRTSCTS;

	if (tcsetattr(fd, TCSANOW, &ti) < 0) {
		fprintf(stdout,"Can't set port settings");
		goto fail;
	}

	fprintf(stdout,"Configuring for initial baudrate of 115200 with Hardware flow control \n");

	if (set_speed(fd, &ti, 115200) < 0) {
		fprintf(stdout,"Can't set initial baud rate \n");
		goto fail;
	}
	
	tcflush(fd, TCIOFLUSH);
	
	return fd;

fail:
	close(fd);
	return -1;
}

/*
 * Read an HCI event from the given file descriptor.
 */
int read_hci_event(int fd, unsigned char* buf, int size)
{
    int remain, r;
    int count = 0;

    if (size <= 0)
        return -1;

    /* The first byte identifies the packet type. For HCI event packets, it
     * should be 0x04, so we read until we get to the 0x04. */
    while (1) {
        r = read(fd, buf, 1);
        if (r <= 0)
            return -1;
        if (buf[0] == 0x04)
            break;
    }
    count++;

    /* The next two bytes are the event code and parameter total length. */
    while (count < 3) {
        r = read(fd, buf + count, 3 - count);
        if (r <= 0)
            return -1;
        count += r;
    }

    /* Now we read the parameters. */
    if (buf[2] < (size - 3))
        remain = buf[2];
    else
        remain = size - 3;

    while ((count - 3) < remain) {
        r = read(fd, buf + count, remain - (count - 3));
        if (r <= 0)
            return -1;
        count += r;
    }

    return count;
}

static int is_it_texas(const unsigned char *respond)
{
    unsigned short manufacturer_id;

    manufacturer_id = MAKEWORD(respond[11], respond[12]);

    return TI_MANUFACTURER_ID == manufacturer_id ? 1 : 0;
}

static const char *get_firmware_name(const uint8_t *respond)
{
    static char firmware_file_name[100] = {0};
    uint16_t version = 0, chip = 0, min_ver = 0, maj_ver = 0;

    version = MAKEWORD(respond[13], respond[14]);
    chip =  (version & 0x7C00) >> 10;
    min_ver = (version & 0x007F);
    maj_ver = (version & 0x0380) >> 7;

    if (version & 0x8000)
        maj_ver |= 0x0008;

    sprintf(firmware_file_name, "TIInit_%d.%d.%d.bts", chip, maj_ver, min_ver);

    return firmware_file_name;
}


int check_chip_version(int fd)
{

	char cmd[4];
	unsigned char resp[100];
	int n;
	char *bts_file;
	
	cmd[0] = 1;
	cmd[1] = 0x01;
	cmd[2] = 0x10;
	cmd[3] = 0x00;
	
	fprintf(stdout,"Send HCI command : HCI_Read_Local_Version_Information \n");

	do {
			n = write(fd, cmd, 4);
			if (n < 0) {
					fprintf(stdout,"Failed to write init command (READ_LOCAL_VERSION_INFORMATION)");
					return -1;
			}
			if (n < 4) {
					fprintf(stdout, "Wanted to write 4 bytes, could only write %d. Stop\n", n);
					return -1;
			}

			/* Read reply. */
			if (read_hci_event(fd, resp, 100) < 0) {
					fprintf(stdout,"Failed to read init response (READ_LOCAL_VERSION_INFORMATION)");
					return -1;
			}

			/* Wait for command complete event for our Opcode */
	} while (resp[4] != cmd[1] && resp[5] != cmd[2]);

	fprintf(stdout,"Recived HCI Event/Response \n");

  /* Verify manufacturer */
    if (! is_it_texas(resp)) {
        fprintf(stdout,"ERROR: module's manufacturer is not Texas Instruments\n");
        return -1;
    }

	fprintf(stdout, "Found a Texas Instruments' chip!\n");
	bts_file = get_firmware_name(resp);
	fprintf(stdout, "Needed Firmware file : %s\n", bts_file);
	
	return 1;
}


int get_bd_address(int fd)
{
	char cmd[4];
	unsigned char resp[100];
	int n,i;
	
	cmd[0] = 1;
	cmd[1] = 0x09;
	cmd[2] = 0x10;
	cmd[3] = 0x00;

	fprintf(stdout,"Send HCI command : HCI_Read_BD_ADDR \n");

	do {
			n = write(fd, cmd, 4);
			if (n < 0) {
					fprintf(stdout,"Failed to write read BD address command (READ_BD_ADDRESS)");
					return -1;
			}
			if (n < 4) {
					fprintf(stdout, "Wanted to write 4 bytes, could only write %d. Stop\n", n);
					return -1;
			}

			/* Read reply. */
			if (read_hci_event(fd, resp, 100) < 0) {
					fprintf(stdout,"Failed to read init response (READ_LOCAL_VERSION_INFORMATION)");
					return -1;
			}

			/* Wait for command complete event for our Opcode */
	} while (resp[4] != cmd[1] && resp[5] != cmd[2]);

	fprintf(stdout,"Recived HCI Event/Response \n");

	n = (int) resp[1];

	for (i=0; i <= n; i++)
	{
		fprintf(stdout, "response byte[%d] = 0x%02x \n", i, (int) resp[i]);
	}
	
	return 1;

}

int change_speed(int fd, int speed)
{
	struct termios ti;
	int ret;
	char cmd[20];
	unsigned char resp[100];
	int n,i;
	
	cmd[0] = 0x01;
	cmd[1] = 0x36;
	cmd[2] = 0xFF;
	cmd[3] = 0x04;
	cmd[4] = (unsigned char) ((0x000000FF) & speed);
	cmd[5] = (unsigned char) (((0x0000FF00) & speed)>>8);
	cmd[6] = (unsigned char) (((0x00FF0000) & speed)>>16);
	cmd[7] = (unsigned char) (((0xFF000000) & speed)>>24);

	fprintf(stdout,"Send HCI command : HCI_VS_Update_Uart_HCI_Baudrate \n");

	n = write(fd, cmd, 8);
	if (n < 0) {
			fprintf(stdout,"Failed to write read BD address command (READ_BD_ADDRESS)");
			return -1;
	}
	if (n < 8) {
			fprintf(stdout, "Wanted to write 4 bytes, could only write %d. Stop\n", n);
			return -1;
	}

	fprintf(stdout,"Send baudrate change command to Controller \n");
			/* Read reply. */
	if (read_hci_event(fd, resp, 100) < 0) {
			fprintf(stdout,"Failed to read init response (READ_LOCAL_VERSION_INFORMATION)");
			return -1;
	}
	n = (int) resp[1];
	// fprintf(stdout,"%s, response length = %d \n",__func__,n);
	fprintf(stdout,"Recived HCI Event/Response \n");

	for (i=0; i <= n; i++)
	{
		fprintf(stdout, "response byte[%d] = 0x%02x \n", i, (int) resp[i]);
	}

	// Change internal baudrate
	fprintf(stdout," \n");
	fprintf(stdout,"Changing host UART baudrate \n");
	tcflush(fd, TCIOFLUSH);

	if (tcgetattr(fd, &ti) < 0) {
		fprintf(stdout,"Can't get port settings \n");
		goto fail;
	}
	ret = set_speed(fd, &ti, speed);
	
	if (ret < 0)
	{
		fprintf(stdout,"Failed to set host UART baud rate !!! \n");
		
		return ret;
	}			

	
	return ret;		
	
fail:
	return -1;

}

int execute_sys_cmd(char *cmd_pattern,int nShutDownGpio)
{
    char cmd[60];

    sprintf(cmd, cmd_pattern,  nShutDownGpio);
    printf("%s\n", cmd);
    
	return system(cmd);
}


int enable_bt(int gpio)
{
	execute_sys_cmd("echo %d > /sys/class/gpio/export",gpio);
	execute_sys_cmd("echo out > /sys/class/gpio/gpio%d/direction",gpio);

	execute_sys_cmd("echo 0 > /sys/class/gpio/gpio%d/value",gpio);
	execute_sys_cmd("echo 1 > /sys/class/gpio/gpio%d/value",gpio);
} 

__asm__(".symver realpath,realpath@GLIBC_2.30");

int main(int argc, char *argv[])
{
	int fd;	
	int ret;
	char dev[20];	
	int bt_en_gpio;
	int speed;
	struct termios *ti;
	
	// get the parameters
	// dev -- tty device
	// speed -- new speed
	
	if (argc < 3)
	{
		fprintf(stdout,"usage : hci-test <dev-name> <baud rate> <bt_en_gpio> \n");
		exit(1);
	}
	
	speed = atoi(argv[2]);

	bt_en_gpio = atoi(argv[3]);
 		
	strcpy(dev,argv[1]);

	fprintf(stdout, "opening device: %s for final baudrate=%d : bt_en_gpio=%d \n",dev,speed,bt_en_gpio);
	
	enable_bt(bt_en_gpio);

	fd = init_uart(dev);

	if (fd > 0) 
	{
		fprintf(stdout,"UART initialized !!! \n");

	} 
	else
	{
		fprintf(stdout,"Failed to Initialize UART !!!!! \n");
		exit(1);
	}	
	// read chip version

	check_chip_version(fd);
#if 1
	fprintf(stdout,"changing HCI baudrate to %d \n",speed);
	if (change_speed(fd, speed) < 0) {
		fprintf(stdout,"Can't set baud rate to %d \n",speed);
		close (fd);
		exit (1);
	}
#endif
	check_chip_version(fd);
	get_bd_address(fd);
	
	close(fd);
}


#include <string.h>
#include <avr/pgmspace.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include "fat.h"
#include "fat_config.h"
#include "partition.h"
#include "sd_raw.h"
#include "sd_raw_config.h"
#include "uart.h"
#include "RingBuffer.h"

#define DEBUG 0

/**
 * \mainpage MMC/SD/SDHC card library
 *
 * This project provides a general purpose library which implements read and write
 * support for MMC, SD and SDHC memory cards.
 *
 * It includes
 * - low-level \link sd_raw MMC, SD and SDHC read/write routines \endlink
 * - \link partition partition table support \endlink
 * - a simple \link fat FAT16/FAT32 read/write implementation \endlink
 *
 * \section circuit The circuit
 * The circuit which was mainly used during development consists of an Atmel AVR
 * microcontroller with some passive components. It is quite simple and provides
 * an easy test environment. The circuit which can be downloaded on the
 * <a href="http://www.roland-riegel.de/sd-reader/">project homepage</a> has been
 * improved with regard to operation stability.
 *
 * I used different microcontrollers during development, the ATmega8 with 8kBytes
 * of flash, and its pin-compatible alternative, the ATmega168 with 16kBytes flash.
 * The first one is the one I started with, but when I implemented FAT16 write
 * support, I ran out of flash space and switched to the ATmega168. For FAT32, an
 * ATmega328 is required.
 * 
 * The circuit board is a self-made and self-soldered board consisting of a single
 * copper layer and standard DIL components, except of the MMC/SD card connector.
 *
 * The connector is soldered to the bottom side of the board. It has a simple
 * eject button which, when a card is inserted, needs some space beyond the connector
 * itself. As an additional feature the connector has two electrical switches
 * to detect wether a card is inserted and wether this card is write-protected.
 * 
 * \section pictures Pictures
 * \image html pic01.jpg "The circuit board used to implement and test this application."
 * \image html pic02.jpg "The MMC/SD card connector on the soldering side of the circuit board."
 *
 * \section software The software
 * The software is written in pure standard ANSI-C. It might not be the smallest or
 * the fastest one, but I think it is quite flexible. See the project's
 * <a href="http://www.roland-riegel.de/sd-reader/benchmarks/">benchmark page</a> to get an
 * idea of the possible data rates.
 *
 * I implemented an example application providing a simple command prompt which is accessible
 * via the UART at 9600 Baud. With commands similiar to the Unix shell you can browse different
 * directories, read and write files, create new ones and delete them again. Not all commands are
 * available in all software configurations.
 * - <tt>cat \<file\></tt>\n
 *   Writes a hexdump of \<file\> to the terminal.
 * - <tt>cd \<directory\></tt>\n
 *   Changes current working directory to \<directory\>.
 * - <tt>disk</tt>\n
 *   Shows card manufacturer, status, filesystem capacity and free storage space.
 * - <tt>init</tt>\n
 *   Reinitializes and reopens the memory card.
 * - <tt>ls</tt>\n
 *   Shows the content of the current directory.
 * - <tt>mkdir \<directory\></tt>\n
 *   Creates a directory called \<directory\>.
 * - <tt>rm \<file\></tt>\n
 *   Deletes \<file\>.
 * - <tt>sync</tt>\n
 *   Ensures all buffered data is written to the card.
 * - <tt>touch \<file\></tt>\n
 *   Creates \<file\>.
 * - <tt>write \<file\> \<offset\></tt>\n
 *   Writes text to \<file\>, starting from \<offset\>. The text is read
 *   from the UART, line by line. Finish with an empty line.
 *
 * \htmlonly
 * <p>
 * The following table shows some typical code sizes in bytes, using the 20090330 release with a
 * buffered read-write MMC/SD configuration, FAT16 and static memory allocation:
 * </p>
 *
 * <table border="1" cellpadding="2">
 *     <tr>
 *         <th>layer</th>
 *         <th>code size</th>
 *         <th>static RAM usage</th>
 *     </tr>
 *     <tr>
 *         <td>MMC/SD</td>
 *         <td align="right">2410</td>
 *         <td align="right">518</td>
 *     </tr>
 *     <tr>
 *         <td>Partition</td>
 *         <td align="right">456</td>
 *         <td align="right">17</td>
 *     </tr>
 *     <tr>
 *         <td>FAT16</td>
 *         <td align="right">7928</td>
 *         <td align="right">188</td>
 *     </tr>
 * </table>
 *
 * <p>
 * The static RAM is mostly used for buffering memory card access, which
 * improves performance and reduces implementation complexity.
 * </p>
 * 
 * <p>
 * Please note that the numbers above do not include the C library functions
 * used, e.g. some string functions. These will raise the numbers somewhat
 * if they are not already used in other program parts.
 * </p>
 * 
 * <p>
 * When opening a partition, filesystem, file or directory, a little amount
 * of RAM is used, as listed in the following table. Depending on the library
 * configuration, the memory is either allocated statically or dynamically.
 * </p>
 *
 * <table border="1" cellpadding="2">
 *     <tr>
 *         <th>descriptor</th>
 *         <th>dynamic/static RAM</th>
 *     </tr>
 *     <tr>
 *         <td>partition</td>
 *         <td align="right">17</td>
 *     </tr>
 *     <tr>
 *         <td>filesystem</td>
 *         <td align="right">26</td>
 *     </tr>
 *     <tr>
 *         <td>file</td>
 *         <td align="right">53</td>
 *     </tr>
 *     <tr>
 *         <td>directory</td>
 *         <td align="right">49</td>
 *     </tr>
 * </table>
 * 
 * \endhtmlonly
 *
 * \section adaptation Adapting the software to your needs
 * The only hardware dependent part is the communication layer talking to the
 * memory card. The other parts like partition table and FAT support are
 * completely independent, you could use them even for managing Compact Flash
 * cards or standard ATAPI hard disks.
 *
 * By changing the MCU* variables in the Makefile, you can use other Atmel
 * microcontrollers or different clock speeds. You might also want to change
 * the configuration defines in the files fat_config.h, partition_config.h,
 * sd_raw_config.h and sd-reader_config.h. For example, you could disable
 * write support completely if you only need read support.
 *
 * For further information, visit the project's
 * <a href="http://www.roland-riegel.de/sd-reader/faq/">FAQ page</a>.
 * 
 * \section bugs Bugs or comments?
 * If you have comments or found a bug in the software - there might be some
 * of them - you may contact me per mail at feedback@roland-riegel.de.
 *
 * \section acknowledgements Acknowledgements
 * Thanks go to Ulrich Radig, who explained on his homepage how to interface
 * MMC cards to the Atmel microcontroller (http://www.ulrichradig.de/).
 * I adapted his work for my circuit. Although this is a very simple
 * solution, I had no problems using it.
 * 
 * \section copyright Copyright 2006-2009 by Roland Riegel
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation (http://www.gnu.org/copyleft/gpl.html).
 * At your option, you can alternatively redistribute and/or modify the following
 * files under the terms of the GNU Lesser General Public License version 2.1
 * as published by the Free Software Foundation (http://www.gnu.org/copyleft/lgpl.html):
 * - byteordering.c
 * - byteordering.h
 * - fat.c
 * - fat.h
 * - fat_config.h
 * - partition.c
 * - partition.h
 * - partition_config.h
 * - sd_raw.c
 * - sd_raw.h
 * - sd_raw_config.h
 * - sd-reader_config.h
 */

#define soft_reset()        \
do                          \
{                           \
	wdt_enable(WDTO_15MS);  \
	for(;;)                 \
	{                       \
	}                       \
} while(0)

const char* CRLF = "\r\n";
char buffer[20];
static RingBuffer_t Buffer_Rx;
static uint8_t      Buffer_Rx_Data[256];

void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3")));

static uint8_t read_line(char* buffer, uint8_t buffer_length);
static uint32_t strtolong(const char* str);
static uint8_t find_file_in_dir(struct fat_fs_struct* fs, struct fat_dir_struct* dd, const char* name, struct fat_dir_entry_struct* dir_entry);
static struct fat_file_struct* open_file_in_dir(struct fat_fs_struct* fs, struct fat_dir_struct* dd, const char* name); 
static uint8_t print_disk_info(const struct fat_fs_struct* fs);

char wait_for_answer();
char make_file(struct fat_fs_struct* fs, struct fat_dir_struct* dd,char* command);
char exec_cmd(struct fat_fs_struct* fs, struct fat_dir_struct* dd,char* command);
void cmd_cd(struct fat_fs_struct* fs, struct fat_dir_struct* dd,char* command);
void cmd_ls(struct fat_fs_struct* fs, struct fat_dir_struct* dd,char* command);
void cmd_cat(struct fat_fs_struct* fs, struct fat_dir_struct* dd,char* command);
void cmd_rm(struct fat_fs_struct* fs, struct fat_dir_struct* dd,char* command);
void cmd_touch(struct fat_fs_struct* fs, struct fat_dir_struct* dd,char* command);
void cmd_write(struct fat_fs_struct* fs, struct fat_dir_struct* dd,char* command);
void cmd_mkdir(struct fat_fs_struct* fs, struct fat_dir_struct* dd,char* command);
void cmd_test(struct fat_fs_struct* fs, struct fat_dir_struct* dd,char* command);
//void cmd_cd(struct fat_fs_struct* fs, struct fat_dir_struct* dd,char* command);
//void cmd_cd(struct fat_fs_struct* fs, struct fat_dir_struct* dd,char* command);

void wdt_init(void)
{
	MCUSR = 0;
	wdt_disable();

	return;
}


int usart_putchar_printf(char var, FILE *stream) {
	uart_putc(var);
	return 0;
}

volatile int max;
volatile int x;
static FILE mystdout = FDEV_SETUP_STREAM(usart_putchar_printf, NULL, _FDEV_SETUP_WRITE);

int main()
{
    /* Set the clock prescaler */
    clock_prescale_set(clock_div_1);
    
    /* we will just use ordinary idle mode */
    set_sleep_mode(SLEEP_MODE_IDLE);

	RingBuffer_InitBuffer(&Buffer_Rx, Buffer_Rx_Data, sizeof(Buffer_Rx_Data));

    /* setup uart */
    uart_init();
	stdout = &mystdout;

    while(1)
    {
        /* setup sd card slot */
        if(!sd_raw_init())
        {
#if DEBUG
            uart_puts_p(PSTR("error in MMC/SD initialization\n"));
#endif
            continue;
        }

        /* open first partition */
        struct partition_struct* partition = partition_open(sd_raw_read, sd_raw_read_interval,
                                                            sd_raw_write, sd_raw_write_interval, 0);

        if(!partition)
        {
            /* If the partition did not open, assume the storage device
             * is a "superfloppy", i.e. has no MBR.
             */
            partition = partition_open(sd_raw_read, sd_raw_read_interval,
                                       sd_raw_write, sd_raw_write_interval, -1);
            if(!partition)
            {
#if DEBUG
                uart_puts_p(PSTR("error opening partition\n"));
				
#endif
				soft_reset();
                continue;
            }
        }

        /* open file system */
        struct fat_fs_struct* fs = fat_open(partition);
        if(!fs)
        {
#if DEBUG
            uart_puts_p(PSTR("error opening filesystem\n"));
#endif
			// Sometimes this causes hanging. Reset the device
			soft_reset();
            continue;
        }

        /* open root directory */
        struct fat_dir_entry_struct directory;
        fat_get_dir_entry_of_path(fs, "/", &directory);

        struct fat_dir_struct* dd = fat_open_dir(fs, &directory);
        if(!dd)
        {
#if DEBUG
            uart_puts_p(PSTR("error opening root directory failed\n"));
#endif
            continue;
        }

        /* print some card information as a boot message */
        //print_disk_info(fs);

		//uart_putc('s');

        // provide a simple shell 
        while(1)
        {
			char success = 0;
			char filename_available = 0;
			volatile uint8_t  errors = 0;
			RingBuffer_Flush(&Buffer_Rx);

			uart_putc('t');
			
			if(wait_for_answer() == 's')
			{
				char filename[10];
				char filenum[4];
				
				for(char i=0; i<100; i++)
				{
					strcpy(filename, "dump");
					strcat(filename, itoa(i, filenum, 10));
					struct fat_dir_entry_struct subdir_entry;
					if(!find_file_in_dir(fs, dd, filename, &subdir_entry))
					{
						filename_available = 1;
						break;
					}
				}
				if(filename_available)
				{
					filename_available = 0;
					// Create the file
					if(!make_file(fs, dd, filename))
						continue;
						
					// Check if slave is ready for memory transfer
					uart_putc('r');
					if(wait_for_answer() == 'a')
					{
						// Open the file
						struct fat_file_struct* fd = open_file_in_dir(fs, dd, filename);
						if(fd)
						{
							uart_putc('m');
							if(wait_for_answer() == 'a')
							{
								uint8_t data_len;
								volatile int i;
								//max=0;
								for(i=0; i<512; i++)
								{	x=i;
									//printf("L%d\n", i);
									//uart_putc(i/100 + 48);						
									data_len = read_line(buffer, sizeof(buffer)-3);
									if(!data_len)
									{
										errors++;
										continue;
									}
									//buffer[16] = 13;
									//buffer[17] = 10;
									//buffer[18] = 0;
									//buffer[19] = 0;
									strcat(buffer, CRLF);
									data_len += 2;
									/* write text to file */
									if(fat_write_file(fd, (uint8_t*) buffer, data_len) != data_len)
									{
										break;
									}
									
									//fat_write_file(fd, CRLF, 2);
								}
								fat_close_file(fd);
								if(i == 512)
									success = 1;
							}
						}
					}
				}
				
				
            // print prompt 
            //uart_putc('>');
            //uart_putc(' ');

            //// read command 
            //char* command = buffer;
            //if(read_line(command, sizeof(buffer)) < 1)
                //continue;
//
            //// execute command
			//char result = exec_cmd(fs, dd, command);
			//if(!result)
				//break;        
			}
		if(success)
			uart_puts("Success\n");
		//else
			//uart_puts("Errors\n");

		_delay_ms(5000);
		_delay_ms(100);
		break;
		}
        /* close file system */
        fat_close(fs);

        /* close partition */
        partition_close(partition);
	}
    return 0;
}

// Waits 10ms for an answer from slave
char wait_for_answer()
{
	for(char i=0; i<100; i++)
	{
		_delay_ms(100);
		if(!RingBuffer_IsEmpty(&Buffer_Rx))
			return RingBuffer_Remove(&Buffer_Rx);
	}
	return 0;
}

char make_file(struct fat_fs_struct* fs, struct fat_dir_struct* dd,char* filename)
{
	struct fat_dir_entry_struct file_entry;
	if(!fat_create_file(dd, filename, &file_entry))
		return 0;
	return 1;
}

char exec_cmd(struct fat_fs_struct* fs, struct fat_dir_struct* dd,char* command)
{
	if(strcmp_P(command, PSTR("init")) == 0)
    {
        return 0;
    }
    else if(strncmp_P(command, PSTR("cd "), 3) == 0)
    {
        cmd_cd(fs, dd, command);
    }
    else if(strcmp_P(command, PSTR("ls")) == 0)
    {
		cmd_ls(fs, dd, command);
    }
    else if(strncmp_P(command, PSTR("cat "), 4) == 0)
    {
        cmd_cat(fs, dd, command);
    }
    else if(strcmp_P(command, PSTR("disk")) == 0)
    {
        if(!print_disk_info(fs))
            uart_puts_p(PSTR("error reading disk info\n"));		
    }
    else if(strncmp_P(command, PSTR("rm "), 3) == 0)
    {
        cmd_rm(fs, dd, command);
    }
    else if(strncmp_P(command, PSTR("touch "), 6) == 0)
    {
        cmd_touch(fs, dd, command);
    }
    else if(strncmp_P(command, PSTR("write "), 6) == 0)
    {
        cmd_write(fs, dd, command);
    }
    else if(strncmp_P(command, PSTR("mkdir "), 6) == 0)
    {
        cmd_mkdir(fs, dd, command);
    }
#if SD_RAW_WRITE_BUFFERING
    else if(strcmp_P(command, PSTR("sync")) == 0)
    {
        if(!sd_raw_sync())
            uart_puts_p(PSTR("error syncing disk\n"));
    }
#endif
	else if(strcmp_P(command, PSTR("test")) == 0)
	{									
		cmd_test(fs, dd, command);
	}
    else
    {
        uart_puts_p(PSTR("error unknown command: "));
        uart_puts(command);
        uart_putc('\n');
    }
	return 1;
}

void cmd_cd(struct fat_fs_struct* fs, struct fat_dir_struct* dd,char* command)
{
	command += 3;
	if(command[0] == '\0')
		return;

	/* change directory */
	struct fat_dir_entry_struct subdir_entry;
	if(find_file_in_dir(fs, dd, command, &subdir_entry))
	{
		struct fat_dir_struct* dd_new = fat_open_dir(fs, &subdir_entry);
		if(dd_new)
		{
			fat_close_dir(dd);
			dd = dd_new;
			return;
		}
	}

	uart_puts_p(PSTR("error directory not found: "));
	uart_puts(command);
	uart_putc('\n');
}

void cmd_ls(struct fat_fs_struct* fs, struct fat_dir_struct* dd,char* command)
{
	/* print directory listing */
	struct fat_dir_entry_struct dir_entry;
	while(fat_read_dir(dd, &dir_entry))
	{
		uint8_t spaces = sizeof(dir_entry.long_name) - strlen(dir_entry.long_name) + 4;

		uart_puts(dir_entry.long_name);
		uart_putc(dir_entry.attributes & FAT_ATTRIB_DIR ? '/' : ' ');
		while(spaces--)
		uart_putc(' ');
		uart_putdw_dec(dir_entry.file_size);
		uart_putc('\n');
	}
}

void cmd_cat(struct fat_fs_struct* fs, struct fat_dir_struct* dd,char* command)
{
	command += 4;
	if(command[0] == '\0')
		return;
	
	/* search file in current directory and open it */
	struct fat_file_struct* fd = open_file_in_dir(fs, dd, command);
	if(!fd)
	{
		uart_puts_p(PSTR("error opening "));
		uart_puts(command);
		uart_putc('\n');
		return;
	}

	/* print file contents */
	uint8_t buffer[8];
	uint8_t size;
	uint32_t offset = 0;
	while((size = fat_read_file(fd, buffer, sizeof(buffer))) > 0)
	{
		uart_putdw_hex(offset);
		uart_putc(':');
		for(uint8_t i = 0; i < 8; ++i)
		{
			uart_putc(' ');
			uart_putc_hex(buffer[i]);
		}
		
		uart_putc(' ');
		
		/* Display printable characters */
		for(uint8_t i = 0; i < size; ++i)
		{
			if ((buffer[i] >= 32) && (buffer[i] <= 126))
			uart_putc(buffer[i]);
			else
			uart_putc('.');
		}
		
		uart_putc('\n');
		offset += 8;
	}

	fat_close_file(fd);
}

void cmd_rm(struct fat_fs_struct* fs, struct fat_dir_struct* dd,char* command)
{
	command += 3;
	if(command[0] == '\0')
		return;
	
	struct fat_dir_entry_struct file_entry;
	if(find_file_in_dir(fs, dd, command, &file_entry))
	{
		if(fat_delete_file(fs, &file_entry))
			return;
	}

	uart_puts_p(PSTR("error deleting file: "));
	uart_puts(command);
	uart_putc('\n');
}

void cmd_touch(struct fat_fs_struct* fs, struct fat_dir_struct* dd,char* command)
{
	command += 6;
	if(command[0] == '\0')
		return;

	struct fat_dir_entry_struct file_entry;
	if(!fat_create_file(dd, command, &file_entry))
	{
		uart_puts_p(PSTR("error creating file: "));
		uart_puts(command);
		uart_putc('\n');
	}
}

void cmd_write(struct fat_fs_struct* fs, struct fat_dir_struct* dd,char* command)
{
	command += 6;
	if(command[0] == '\0')
		return;

	char* offset_value = command;
	while(*offset_value != ' ' && *offset_value != '\0')
		++offset_value;

	if(*offset_value == ' ')
		*offset_value++ = '\0';
	else
		return;

	/* search file in current directory and open it */
	struct fat_file_struct* fd = open_file_in_dir(fs, dd, command);
	if(!fd)
	{
		uart_puts_p(PSTR("error opening "));
		uart_puts(command);
		uart_putc('\n');
		return;
	}

	int32_t offset = strtolong(offset_value);
	if(!fat_seek_file(fd, &offset, FAT_SEEK_SET))
	{
		uart_puts_p(PSTR("error seeking on "));
		uart_puts(command);
		uart_putc('\n');

		fat_close_file(fd);
			return;
	}

	/* read text from the shell and write it to the file */
	uint8_t data_len;
	while(1)
	{
		/* give a different prompt */
		uart_putc('<');
		uart_putc(' ');

		/* read one line of text */
		data_len = read_line(buffer, sizeof(buffer));
		if(!data_len)
			break;

		/* write text to file */
		if(fat_write_file(fd, (uint8_t*) buffer, data_len) != data_len)
		{
			uart_puts_p(PSTR("error writing to file\n"));
			break;
		}
	}
	fat_close_file(fd);
}

void cmd_mkdir(struct fat_fs_struct* fs, struct fat_dir_struct* dd,char* command)
{
	command += 6;
	if(command[0] == '\0')
		return;

	struct fat_dir_entry_struct dir_entry;
	if(!fat_create_dir(dd, command, &dir_entry))
	{
		uart_puts_p(PSTR("error creating directory: "));
		uart_puts(command);
		uart_putc('\n');
	}
}

void cmd_test(struct fat_fs_struct* fs, struct fat_dir_struct* dd,char* command)
{
	struct fat_dir_entry_struct file_entry;
	if(!fat_create_file(dd, "txt3", &file_entry))
	uart_puts_p(PSTR("error creating\n"));
	else
	{
		/* search file in current directory and open it */
		struct fat_file_struct* fd = open_file_in_dir(fs, dd, "txt3");
		if(!fd)
		{
			uart_puts_p(PSTR("error opening\n"));
			return;
		}
		else
		uart_puts_p(PSTR("ok\n"));
		/* read text from the shell and write it to the file */
		uint8_t data_len, lf_times=0;
		while(1)
		{
			/* read one line of text */
			data_len = read_line(buffer, sizeof(buffer));
			if(!data_len)
			{
				uint8_t eol = '\n';
				if(fat_write_file(fd, &eol, 1) != 1)
				{
					uart_puts_p(PSTR("error writing\n"));
					break;
				}
				continue;
			}
			
			if(strcmp_P(buffer, PSTR("end of file")) == 0)
			break;
			
			/* write text to file */
			if(fat_write_file(fd, (uint8_t*) buffer, data_len) != data_len)
			{
				uart_puts_p(PSTR("error writing to file\n"));
				break;
			}
		}

		fat_close_file(fd);
	}
}

void cmd_tmp1(struct fat_fs_struct* fs, struct fat_dir_struct* dd,char* command)
{
	
}

void cmd_tmp2(struct fat_fs_struct* fs, struct fat_dir_struct* dd,char* command)
{
	
}

uint8_t read_line(char* buffer, uint8_t buffer_length)
{
    memset(buffer, 0, buffer_length);

    uint8_t read_length = 0;
    while(read_length < buffer_length - 1)
    {
		uint8_t c;
		uint16_t i=0; 
		// If nothing is received for a while report failure
		while(RingBuffer_IsEmpty(&Buffer_Rx))
			if(i++>1000)
				return 0;
		c = RingBuffer_Remove(&Buffer_Rx);

        //if(c == 0x08 || c == 0x7f)
        //{
            //if(read_length < 1)
                //continue;
            //--read_length;
            //buffer[read_length] = '\0';
            //continue;
        //}

        if(c == '\n')
        {
            buffer[read_length] = '\0';
            break;
        }
        else
        {
            buffer[read_length] = c;
            ++read_length;
        }
    }

    return read_length;
}

uint32_t strtolong(const char* str)
{
    uint32_t l = 0;
    while(*str >= '0' && *str <= '9')
        l = l * 10 + (*str++ - '0');

    return l;
}

uint8_t find_file_in_dir(struct fat_fs_struct* fs, struct fat_dir_struct* dd, const char* name, struct fat_dir_entry_struct* dir_entry)
{
    while(fat_read_dir(dd, dir_entry))
    {
        if(strcmp(dir_entry->long_name, name) == 0)
        {
            fat_reset_dir(dd);
            return 1;
        }
    }

    return 0;
}

struct fat_file_struct* open_file_in_dir(struct fat_fs_struct* fs, struct fat_dir_struct* dd, const char* name)
{
    struct fat_dir_entry_struct file_entry;
    if(!find_file_in_dir(fs, dd, name, &file_entry))
        return 0;

    return fat_open_file(fs, &file_entry);
}

uint8_t print_disk_info(const struct fat_fs_struct* fs)
{
    if(!fs)
        return 0;

    struct sd_raw_info disk_info;
    if(!sd_raw_get_info(&disk_info))
        return 0;

    uart_puts_p(PSTR("manuf:  0x")); uart_putc_hex(disk_info.manufacturer); uart_putc('\n');
    uart_puts_p(PSTR("oem:    ")); uart_puts((char*) disk_info.oem); uart_putc('\n');
    uart_puts_p(PSTR("prod:   ")); uart_puts((char*) disk_info.product); uart_putc('\n');
    uart_puts_p(PSTR("rev:    ")); uart_putc_hex(disk_info.revision); uart_putc('\n');
    uart_puts_p(PSTR("serial: 0x")); uart_putdw_hex(disk_info.serial); uart_putc('\n');
    uart_puts_p(PSTR("date:   ")); uart_putw_dec(disk_info.manufacturing_month); uart_putc('/');
                                   uart_putw_dec(disk_info.manufacturing_year); uart_putc('\n');
    uart_puts_p(PSTR("size:   ")); uart_putdw_dec(disk_info.capacity / 1024 / 1024); uart_puts_p(PSTR("MB\n"));
    uart_puts_p(PSTR("copy:   ")); uart_putw_dec(disk_info.flag_copy); uart_putc('\n');
    uart_puts_p(PSTR("wr.pr.: ")); uart_putw_dec(disk_info.flag_write_protect_temp); uart_putc('/');
                                   uart_putw_dec(disk_info.flag_write_protect); uart_putc('\n');
    uart_puts_p(PSTR("format: ")); uart_putw_dec(disk_info.format); uart_putc('\n');
    uart_puts_p(PSTR("free:   ")); uart_putdw_dec(fat_get_fs_free(fs)); uart_putc('/');
                                   uart_putdw_dec(fat_get_fs_size(fs)); uart_putc('\n');

    return 1;
}

#if FAT_DATETIME_SUPPORT
void get_datetime(uint16_t* year, uint8_t* month, uint8_t* day, uint8_t* hour, uint8_t* min, uint8_t* sec)
{
    *year = 2007;
    *month = 1;
    *day = 1;
    *hour = 0;
    *min = 0;
    *sec = 0;
}
#endif

/** ISR to manage the reception of data from the serial port, placing received bytes into a circular buffer
 *  for later transmission to the host.
 */
ISR(USART1_RX_vect, ISR_BLOCK)
{
	uint8_t ReceivedByte = UDR1;

	if ( !RingBuffer_IsFull(&Buffer_Rx))
	  RingBuffer_Insert(&Buffer_Rx, ReceivedByte);
	//else if ( RingBuffer_IsFull(&Buffer_Rx))
	  //RingBuffer_Insert(&Buffer_Rx, ReceivedByte);
	//if(max<Buffer_Rx.Count)
		//max = Buffer_Rx.Count;
}

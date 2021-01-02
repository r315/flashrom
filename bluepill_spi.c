/*
 * This file is part of the flashrom project.
 *
 * Copyright (C) 2009, 2010, 2011, 2012 Carl-Daniel Hailfinger
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <stdio.h>
#include <strings.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include "flash.h"
#include "programmer.h"
#include "spi.h"

#define BP_DEFAULTBAUD 115200
/* Default buffer size is 19: 16 bytes data, 3 bytes control. */
#define DEFAULT_BUFSIZE (16 + 3)

static unsigned char *bp_commbuf = NULL;
static int bp_commbufsize = 0;

static int serialport_setup(char *dev)
{
	/* 115200bps, 8 databits, no parity, 1 stopbit */
	sp_fd = sp_openserport(dev, BP_DEFAULTBAUD);
	if (sp_fd == SER_INV_FD)
		return 1;
	return 0;
}

static int buspirate_commbuf_grow(int bufsize)
{
	unsigned char *tmpbuf;

	/* Never shrink. realloc() calls are expensive. */
	if (bufsize <= bp_commbufsize)
		return 0;

	tmpbuf = realloc(bp_commbuf, bufsize);
	if (!tmpbuf) {
		/* Keep the existing buffer because memory is already tight. */
		msg_perr("Out of memory!\n");
		return ERROR_OOM;
	}

	bp_commbuf = tmpbuf;
	bp_commbufsize = bufsize;
	return 0;
}

static int buspirate_sendrecv(unsigned char *buf, unsigned int writecnt,
			      unsigned int readcnt)
{
	unsigned int i;
	int ret = 0;

	msg_pspew("%s: write %i, read %i ", __func__, writecnt, readcnt);
	if (!writecnt && !readcnt) {
		msg_perr("Zero length command!\n");
		return 1;
	}

	if (writecnt){
		msg_pspew("Sending");
    }

	for (i = 0; i < writecnt; i++){
		msg_pspew(" 0x%02x", buf[i]);
    }

#ifdef FAKE_COMMUNICATION
	/* Placate the caller for now. */
	if (readcnt) {
		buf[0] = 0x01;
		memset(buf + 1, 0xff, readcnt - 1);
	}
	ret = 0;
#else
	if (writecnt)
		ret = serialport_write(buf, writecnt);
	if (ret)
		return ret;
	if (readcnt)
		ret = serialport_read(buf, readcnt);
	if (ret)
		return ret;
#endif
	
    if (readcnt){
		msg_pspew(", receiving");
    }

	for (i = 0; i < readcnt; i++){
		msg_pspew(" 0x%02x", buf[i]);
    }

	msg_pspew("\n");
	
    return 0;
}

static int buspirate_wait_for_string(unsigned char *buf, const char *key)
{
	unsigned int keylen = strlen(key);
	int ret;

	ret = buspirate_sendrecv(buf, 0, keylen);
	while (!ret) {
		if (!memcmp(buf, key, keylen))
			return 0;
		memmove(buf, buf + 1, keylen - 1);
		ret = buspirate_sendrecv(buf + keylen - 1, 0, 1);
	}
	return ret;
}

static int bluepill_spi_send_command(struct flashctx *flash, unsigned int writecnt, unsigned int readcnt,
					 const unsigned char *writearr, unsigned char *readarr)
{
	int ret = 0;

	if (writecnt > 16 || readcnt > 16 || (readcnt + writecnt) > 16)
		return SPI_INVALID_LENGTH;

	/* 3 bytes extra for CS#, len, CS#. */
	if (buspirate_commbuf_grow(writecnt + readcnt + 3))
		return ERROR_OOM;

	/**
     * Build message
     * 
     * Sync | WR + RD len | write buffer data | read buffer data |
     * */
    unsigned int i = 0;
	bp_commbuf[i++] = ':'; // Sync
	bp_commbuf[i++] = (char)(writecnt + readcnt);

	memcpy(bp_commbuf + i, writearr, writecnt);
	i += writecnt;
	memset(bp_commbuf + i, 0, readcnt);
	i += readcnt;
	
	ret = buspirate_sendrecv(bp_commbuf, i, readcnt + writecnt);

	if (ret) {
		msg_perr("communication error!\n");
		return SPI_GENERIC_ERROR;
	}

    /* skip message header */
	memcpy(readarr, bp_commbuf + writecnt, readcnt);

	return ret;
}

static struct spi_master spi_master_busbluepill = {
	.features	    = SPI_MASTER_4BA,
	.max_data_read	= MAX_DATA_UNSPECIFIED,
	.max_data_write	= MAX_DATA_UNSPECIFIED,
	.command	    = bluepill_spi_send_command,
	.multicommand	= default_spi_send_multicommand,
	.read		    = default_spi_read,
	.write_256	    = default_spi_write_256,
	.write_aai	    = default_spi_write_aai,
};

#if 0
static const struct buspirate_speeds spispeeds[] = {
	{"30k",		0x0},
	{"125k",	0x1},
	{"250k",	0x2},
	{"1M",		0x3},
	{"2M",		0x4},
	{"2.6M",	0x5},
	{"4M",		0x6},
	{"8M",		0x7},
	{NULL,		0x0}
};
#endif
static int bluepill_spi_shutdown(void *data)
{
	int ret = 0;
	/* No need to allocate a buffer here, we know that bp_commbuf is at least DEFAULT_BUFSIZE big. */

	/* Exit raw SPI mode (enter raw bitbang mode) */
	bp_commbuf[0] = 'q';
	buspirate_sendrecv(bp_commbuf, 1, 0);
	
	/* Shut down serial port communication */
	ret = serialport_shutdown(NULL);
		
	bp_commbufsize = 0;
	free(bp_commbuf);
	bp_commbuf = NULL;
	
	return ret;
}

/**
 * 
 * 
 */
int bluepill_spi_init(void)
{
	char *dev;
	//int spispeed = 0x7;
	int ret = 0;

	dev = extract_programmer_param("dev");
	if (dev && !strlen(dev)) {
		free(dev);
		dev = NULL;
	}
	if (!dev) {
		msg_perr("No serial device given. Use flashrom -p bluepill_spi:dev=/dev/ttyACM0\n");
		return 1;
	}

	bp_commbuf = malloc(DEFAULT_BUFSIZE);
	if (!bp_commbuf) {
		bp_commbufsize = 0;
		msg_perr("Out of memory!\n");
		free(dev);
		return ERROR_OOM;
	}
	bp_commbufsize = DEFAULT_BUFSIZE;

	ret = serialport_setup(dev);
	free(dev);
	if (ret) {
		bp_commbufsize = 0;
		free(bp_commbuf);
		bp_commbuf = NULL;
		return ret;
	}

	if (register_shutdown(bluepill_spi_shutdown, NULL) != 0) {
		bp_commbufsize = 0;
		free(bp_commbuf);
		bp_commbuf = NULL;
		return 1;
	}

    bp_commbuf[0] = 'q';
    bp_commbuf[1] = '\n';

    if ((ret = buspirate_sendrecv(bp_commbuf, 2, 0))){
        return ret;
	}

	if ((ret = buspirate_wait_for_string(bp_commbuf, "bluepill>"))){
		return ret;
	}

    msg_pdbg("bluepill detected!\n");

    strcpy((char*)bp_commbuf, "flashrom\n");

    if ((ret = buspirate_sendrecv(bp_commbuf, strlen((char*)bp_commbuf) , 0)))
        return ret;

    if ((ret = buspirate_wait_for_string(bp_commbuf, "FLASHROM OK")))
		return ret;

    msg_pdbg("bluepill in flashrom mode\n");

    spi_master_busbluepill.max_data_read = 12;
	spi_master_busbluepill.max_data_write = 12;
	//spi_master_busbluepill.command = bluepill_spi_send_command;

    register_spi_master(&spi_master_busbluepill);

	return 0;
}
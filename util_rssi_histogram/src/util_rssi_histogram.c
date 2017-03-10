/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    ©2013 Semtech-Cycleo

Description:
	Configure Lora concentrator and record RSSI captures in log files

License: Revised BSD License, see LICENSE.TXT file include in the project
Maintainer: Matthieu Leurent
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

/* fix an issue between POSIX and C99 */
#if __STDC_VERSION__ >= 199901L
	#define _XOPEN_SOURCE 600
#else
	#define _XOPEN_SOURCE 500
#endif

#include <stdint.h>		/* C99 types */
#include <stdbool.h>	/* bool type */
#include <stdio.h>		/* printf fprintf sprintf fopen fputs */

#include <string.h>		/* memset */
#include <signal.h>		/* sigaction */
#include <time.h>		/* time clock_gettime strftime gmtime clock_nanosleep*/
#include <unistd.h>		/* getopt access */
#include <stdlib.h>		/* atoi */
#include <getopt.h>

#include "loragw_reg.h"
#include "loragw_aux.h"


/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a)	(sizeof(a) / sizeof((a)[0]))
#define MSG(args...)	fprintf(stderr,args) /* message that is destined to the user */
#define DEBUG_MSG(args...)	//fprintf(stderr,"DEBUG: " args) /* message for debug */
#define IF_HZ_TO_REG(f)		(f << 5)/15625

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define FORCE_SX1255			(0)

#define SX125x_32MHz_FRAC		15625	/* irreductible fraction for PLL register caculation */
#define SX125x_CLK_OUT_EN	    1
#define SX125x_RF_LOOP_BACK_EN	0
#define SX125x_DIG_LOOP_BACK_EN	0
#define	SX125x_TX_DAC_CLK_SEL	1	/* 0:int, 1:ext */
#define	SX125x_TX_DAC_GAIN		2	/* 3:0, 2:-3, 1:-6, 0:-9 dBFS (default 2) */
#define	SX125x_TX_MIX_GAIN		14	/* -38 + 2*TxMixGain dB (default 14) */
#define	SX125x_TX_PLL_BW		3	/* 0:75, 1:150, 2:225, 3:300 kHz (default 3) */
#define	SX125x_TX_ANA_BW		0	/* 17.5 / 2*(41-TxAnaBw) MHz (default 0) */
#define	SX125x_TX_DAC_BW		5	/* 24 + 8*TxDacBw Nb FIR taps (default 2, max 5) */
#define	SX125x_RX_LNA_GAIN		1	/* 1 to 6, 1 highest gain */
#define	SX125x_RX_BB_GAIN		12	/* 0 to 15 , 15 highest gain */
#define SX125x_LNA_ZIN			1	/* 0:50, 1:200 Ohms (default 1) */
#define	SX125x_RX_ADC_BW		7	/* 0 to 7, 2:100<BW<200, 5:200<BW<400,7:400<BW kHz SSB (default 7) */
#define	SX125x_RX_ADC_TRIM		6	/* 0 to 7, 6 for 32MHz ref, 5 for 36MHz ref */
#define SX125x_RX_BB_BW			0	/* 0:750, 1:500, 2:375; 3:250 kHz SSB (default 1, max 3) */
#define SX125x_RX_PLL_BW		0	/* 0:75, 1:150, 2:225, 3:300 kHz (default 3, max 3) */
#define SX125x_ADC_TEMP			0	/* ADC temperature measurement mode (default 0) */
#define SX125x_XOSC_GM_STARTUP	13	/* (default 13) */
#define SX125x_XOSC_DISABLE		2	/* Disable of Xtal Oscillator blocks bit0:regulator, bit1:core(gm), bit2:amplifier */
#define	PLL_LOCK_MAX_ATTEMPTS	6

#define MCU_AGC_WAIT_CMD		16
#define MCU_AGC_ABORT_CMD		17

#define	RSSI_OFFSET             -137
#define	RSSI_OFFSET_SX1255      -147
#define IF_CHAN                 100000
#define NB_RSSI					128
#define PRINT_CAPTURE			10

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES (GLOBAL) ------------------------------------------- */

#include "agc_fw.var"  /* external definition of the variable */

int32_t read_val;

int chip_type = 1;		//0: sx1255, others: sx1257
int rssi_offset = RSSI_OFFSET;

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DECLARATION ---------------------------------------- */

void sx125x_write_local(uint8_t channel, uint8_t addr, uint8_t data);

uint8_t sx125x_read_local(uint8_t channel, uint8_t addr);

int sx125x_pll_lock(uint8_t rf_chain, uint32_t freq_hz);

int setup_sx125x_local(uint8_t rf_chain, uint32_t freq_hz);

void lgw_agc_start(uint8_t radio_select, uint8_t channel_select);

void usage(void);

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */

void sx125x_write_local(uint8_t channel, uint8_t addr, uint8_t data) {
	int reg_add, reg_dat, reg_cs;

	if (addr >= 0x7F) {
		DEBUG_MSG("ERROR: ADDRESS OUT OF RANGE\n");
		return;
	}

	/* selecting the target radio */
	switch (channel) {
		case 0:
			reg_add = LGW_SPI_RADIO_A__ADDR;
			reg_dat = LGW_SPI_RADIO_A__DATA;
			reg_cs	= LGW_SPI_RADIO_A__CS;
			break;

		case 1:
			reg_add = LGW_SPI_RADIO_B__ADDR;
			reg_dat = LGW_SPI_RADIO_B__DATA;
			reg_cs	= LGW_SPI_RADIO_B__CS;
			break;

		default:
			DEBUG_MSG("ERROR: UNEXPECTED VALUE %d IN SWITCH STATEMENT\n", channel);
			return;
	}

	/* SPI master data write procedure */
	lgw_reg_w(reg_cs, 0);
	lgw_reg_w(reg_add, 0x80 | addr); /* MSB at 1 for write operation */
	lgw_reg_w(reg_dat, data);
	lgw_reg_w(reg_cs, 1);
	lgw_reg_w(reg_cs, 0);

	return;
}

uint8_t sx125x_read_local(uint8_t channel, uint8_t addr) {
	int reg_add, reg_dat, reg_cs, reg_rb;
	int32_t read_value;

	if (addr >= 0x7F) {
		DEBUG_MSG("ERROR: ADDRESS OUT OF RANGE\n");
		return 0;
	}

	/* selecting the target radio */
	switch (channel) {
		case 0:
			reg_add = LGW_SPI_RADIO_A__ADDR;
			reg_dat = LGW_SPI_RADIO_A__DATA;
			reg_cs	= LGW_SPI_RADIO_A__CS;
			reg_rb	= LGW_SPI_RADIO_A__DATA_READBACK;
			break;

		case 1:
			reg_add = LGW_SPI_RADIO_B__ADDR;
			reg_dat = LGW_SPI_RADIO_B__DATA;
			reg_cs	= LGW_SPI_RADIO_B__CS;
			reg_rb	= LGW_SPI_RADIO_B__DATA_READBACK;
			break;

		default:
			DEBUG_MSG("ERROR: UNEXPECTED VALUE %d IN SWITCH STATEMENT\n", channel);
			return 0;
	}

	/* SPI master data read procedure */
	lgw_reg_w(reg_cs, 0);
	lgw_reg_w(reg_add, addr); /* MSB at 0 for read operation */
	lgw_reg_w(reg_dat, 0);
	lgw_reg_w(reg_cs, 1);
	lgw_reg_w(reg_cs, 0);
	lgw_reg_r(reg_rb, &read_value);

	return (uint8_t)read_value;
}

int sx125x_pll_lock(uint8_t rf_chain, uint32_t freq_hz) {
	uint32_t part_int;
	uint32_t part_frac;
	int cpt_attempts = 0;

	if(chip_type == 0){
		/*sx1255*/
		part_int = freq_hz / (SX125x_32MHz_FRAC << 7); /* integer part, gives the MSB */
		part_frac = ((freq_hz % (SX125x_32MHz_FRAC << 7)) << 9) / SX125x_32MHz_FRAC; /* fractional part, gives middle part and LSB */
	}else{
		/*sx1257*/
		part_int = freq_hz / (SX125x_32MHz_FRAC << 8); /* integer part, gives the MSB */
		part_frac = ((freq_hz % (SX125x_32MHz_FRAC << 8)) << 8) / SX125x_32MHz_FRAC; /* fractional part, gives middle part and LSB */
	}

	/* set RX PLL frequency */
	//part_int = freq_hz / (SX125x_32MHz_FRAC << 8); /* integer part, gives the MSB */
	//part_frac = ((freq_hz % (SX125x_32MHz_FRAC << 8)) << 8) / SX125x_32MHz_FRAC; /* fractional part, gives middle part and LSB */
	sx125x_write_local(rf_chain, 0x01,0xFF & part_int); /* Most Significant Byte */
	sx125x_write_local(rf_chain, 0x02,0xFF & (part_frac >> 8)); /* middle byte */
	sx125x_write_local(rf_chain, 0x03,0xFF & part_frac); /* Least Significant Byte */

	/* start and PLL lock */
	do {
		if (cpt_attempts >= PLL_LOCK_MAX_ATTEMPTS) {
			MSG("ERROR: FAIL TO LOCK PLL\n");
			return -1;
		}
		sx125x_write_local(rf_chain, 0x00, 1); /* enable Xtal oscillator */
		sx125x_write_local(rf_chain, 0x00, 3); /* Enable RX (PLL+FE) */
		++cpt_attempts;
		DEBUG_MSG("SX125x #%d PLL start (attempt %d)\n", rf_chain, cpt_attempts);
		wait_ms(1);
	} while((sx125x_read_local(rf_chain, 0x11) & 0x02) == 0);

	return 0;
}

int setup_sx125x_local(uint8_t rf_chain, uint32_t freq_hz) {

	/* Aux */
	sx125x_write_local(rf_chain, 0x10, SX125x_TX_DAC_CLK_SEL + SX125x_CLK_OUT_EN*2 + SX125x_RF_LOOP_BACK_EN*4 + SX125x_DIG_LOOP_BACK_EN*8); /* Enable 'clock out' for both radios */

	if(chip_type == 0){
		/*sx1255*/
		sx125x_write_local(rf_chain, 0x28, SX125x_XOSC_GM_STARTUP + SX125x_XOSC_DISABLE*16);
	}else{
		/*sx1257*/
		sx125x_write_local(rf_chain, 0x26, SX125x_XOSC_GM_STARTUP + SX125x_XOSC_DISABLE*16);
	}

	/* Tx */
	sx125x_write_local(rf_chain, 0x08, SX125x_TX_MIX_GAIN + SX125x_TX_DAC_GAIN*16);
	sx125x_write_local(rf_chain, 0x0A, SX125x_TX_ANA_BW + SX125x_TX_PLL_BW*32);
	sx125x_write_local(rf_chain, 0x0B, SX125x_TX_DAC_BW);

	/* Rx */
	sx125x_write_local(rf_chain, 0x0C, SX125x_LNA_ZIN + SX125x_RX_BB_GAIN*2 + SX125x_RX_LNA_GAIN*32);
	sx125x_write_local(rf_chain, 0x0D, SX125x_RX_BB_BW + SX125x_RX_ADC_TRIM*4 + SX125x_RX_ADC_BW*32);
	sx125x_write_local(rf_chain, 0x0E, SX125x_ADC_TEMP + SX125x_RX_PLL_BW*2);

	/* start and PLL lock */
	return sx125x_pll_lock(rf_chain, freq_hz);
}

void lgw_agc_start(uint8_t radio_select, uint8_t channel_select) {

	DEBUG_MSG("MCU AGC initialisation....\n");
	lgw_reg_w(LGW_MCU_RST_1, 1);
	lgw_reg_w(LGW_MCU_RST_1, 0);

	lgw_reg_r(LGW_MCU_AGC_STATUS, &read_val);
	DEBUG_MSG("MCU status: %2X\n", (uint8_t)read_val);

	lgw_reg_w(LGW_RADIO_SELECT, 16);
	lgw_reg_w(LGW_RADIO_SELECT, 17);
	DEBUG_MSG("Tx gain LUT update skipped: using default LUT\n");

	lgw_reg_r(LGW_MCU_AGC_STATUS, &read_val);
	DEBUG_MSG("MCU status: %2X\n", (uint8_t)read_val);

	lgw_reg_w(LGW_RADIO_SELECT, 16);
	lgw_reg_w(LGW_RADIO_SELECT, channel_select);

	lgw_reg_r(LGW_MCU_AGC_STATUS, &read_val);
	DEBUG_MSG("MCU status: %2X\n", (uint8_t)read_val);

	lgw_reg_w(LGW_RADIO_SELECT, 16);
	lgw_reg_w(LGW_RADIO_SELECT, radio_select);

	lgw_reg_r(LGW_MCU_AGC_STATUS, &read_val);
	DEBUG_MSG("MCU status: %2X\n", (uint8_t)read_val);
}

/* describe command line options */
void usage(void) {
	printf( "\nAvailable options:\n");
	printf( " -h print this help\n");
	printf( " --file log file name\n");
	printf( " --fmin start frequency in Hz, default is 863 MHz\n");
	printf( " --fmax stop frequency in Hz, default is 870 MHz\n");
	printf( " --fstep frequency resolution in Hz, default is 50 kHz\n");
	printf( " -n number of RSSI captures, each capture is 4096 samples long, default is 90 (3s for 125Khz capture rate)\n");
	printf( " -p div ratio of capture rate (32 MHz/p), default is 256 (125 kHz)\n\n");
}

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main(int argc, char **argv)
{
	int i, j, k, m; /* loop and temporary variables */

	/* application parameters */
	int option_index = 0;
	static struct option long_options[] = {
        {"fmin", 1, 0, 0},
        {"fmax", 1, 0, 0},
        {"fstep", 1, 0, 0},
        {"file", 1, 0, 0},
        {0, 0, 0, 0}
    };
	char log_file_name[64] = "rssi_histogram.csv";
	unsigned long int fmin = 863000000;
	unsigned long int fmax = 870000000;
	unsigned long int fstep = 50000;
	int nb_captures = 90; /*Number of captures */
	int capture_period = 256; /* Capture period, 32:1MHz */

	int fstep_nb; /*Number of channel frequencies */
	unsigned long int rssi_hist[NB_RSSI];
	unsigned long int chan_freq;
	int reg_stat;
	uint8_t radio_select;
	FILE * log_file = NULL;
	uint8_t capture_buffer[256];
	uint8_t rssi;
	uint8_t rssi_20, rssi_50, rssi_80;
	unsigned long int rssi_cumu;
	int fprintf_success;
	float capture_time;
	float capture_rate;
	int capture_wait;

	/* parse command line options */
	while ((i = getopt_long (argc, argv, "hn:p:", long_options, &option_index)) != -1) {
		switch (i) {
			case 0:
				if (strcmp(long_options[option_index].name,"fmin") == 0) {
					i = sscanf(optarg, "%lu", &fmin);
				}
				else if (strcmp(long_options[option_index].name,"fmax") == 0) {
					i = sscanf(optarg, "%lu", &fmax);
				}
				else if (strcmp(long_options[option_index].name,"fstep") == 0) {
					i = sscanf(optarg, "%lu", &fstep);
				}
				else if (strcmp(long_options[option_index].name,"file") == 0) {
					sprintf(log_file_name, "%s", optarg);
				}
				else {
					MSG("ERROR: argument parsing use -h option for help\n");
					usage();
					return EXIT_FAILURE;
				}
				break;
			case 'h':
				usage();
				return EXIT_FAILURE;
				break;

			case 'f': /* -f log file name prefix */
				sprintf(log_file_name, "%s", optarg);
				break;

			case 'n': /* -n <int> Number of captures */
				i = sscanf(optarg, "%i", &nb_captures);
				break;

			case 'p': /* -p <int> capture period */
				i = sscanf(optarg, "%i", &capture_period);
				break;

			default:
				MSG("ERROR: argument parsing use -h option for help\n");
				usage();
				return EXIT_FAILURE;
		}
	}



	if((fmin<100000000) || (fmax<100000000)){
		MSG("ERROR: frequency too low\n");
		return 0;
	}else if((fmin>1000000000) || (fmax>1000000000)){
		MSG("ERROR: frequency too high\n");
		return 0;
	}else if((fmin>100000000) && (fmax<520000000)){
		MSG("SX1255 is used\n");
		chip_type = 0;
		rssi_offset = RSSI_OFFSET_SX1255;
	}else if((fmax<1000000000) && (fmin>520000000)){
		MSG("SX1257 is used\n");
		chip_type = 1;
		rssi_offset = RSSI_OFFSET;
	}else{
		MSG("Unknown front end\n");
		return 0;
	}

	MSG("fmin: %ld, fmax: %ld, fstep: %ld, out: %s\n", fmin, fmax, fstep, log_file_name);

	/*--- Set concentrator in minimum config for RSSI capture --------*/

	reg_stat = lgw_connect(false, 0);
	if (reg_stat == LGW_REG_ERROR) {
		MSG("ERROR: FAIL TO CONNECT BOARD\n");
		return -1;
	}

	lgw_soft_reset();

	lgw_reg_w(LGW_RADIO_A_EN, 1);
	lgw_reg_w(LGW_RADIO_B_EN, 1);
	wait_ms(500);
	lgw_reg_w(LGW_RADIO_RST, 1);
	wait_ms(5);
	lgw_reg_w(LGW_RADIO_RST, 0);

	lgw_reg_w(LGW_GLOBAL_EN, 1);

	/* Config channels */
	setup_sx125x_local(0, fmin);
	lgw_reg_w(LGW_IF_FREQ_0, IF_HZ_TO_REG(IF_CHAN));
	radio_select = 0;

	/* load AGC firmware */
	lgw_reg_w(LGW_MCU_RST_1, 1); /* reset the MCU */
	lgw_reg_w(LGW_MCU_SELECT_MUX_1, 0); /* set mux to access MCU program RAM */
	lgw_reg_w(LGW_MCU_PROM_ADDR, 0); /* set address to 0 */
	lgw_reg_wb(LGW_MCU_PROM_DATA, agc_firmware, 8192); /* write the program in one burst */
	lgw_reg_w(LGW_MCU_SELECT_MUX_1, 1); /* give back control of the MCU program ram to the MCU */

	/* Settings for AGC */
	lgw_reg_w(LGW_RSSI_BB_FILTER_ALPHA, 6);
	lgw_reg_w(LGW_RSSI_DEC_FILTER_ALPHA, 7);
	lgw_reg_w(LGW_RSSI_CHANN_FILTER_ALPHA, 7);
	lgw_reg_w(LGW_RSSI_BB_DEFAULT_VALUE, 23);
	lgw_reg_w(LGW_RSSI_DEC_DEFAULT_VALUE, 66);
	lgw_reg_w(LGW_RSSI_CHANN_DEFAULT_VALUE, 85);

	/* Give control to AGC */
	lgw_reg_w(LGW_FORCE_HOST_RADIO_CTRL, 0);
	lgw_reg_w(LGW_FORCE_HOST_FE_CTRL, 0);
	lgw_reg_w(LGW_FORCE_DEC_FILTER_GAIN, 0);

	/* Capture RAM config */
	lgw_reg_w(LGW_CAPTURE_PERIOD,capture_period-1);
	capture_time = ((float)capture_period/32e6)*4096;
	capture_rate = 32e6/(float)capture_period;
	capture_wait = (int)(capture_time*1010);
	MSG("Capture config: %.2f ms, %.2f kHz\n", capture_time*1e3, capture_rate/1e3);
	lgw_reg_w(LGW_CAPTURE_SOURCE,25); /* Set capture source to AGC GPIO */

	/* create log file */
	log_file = fopen(log_file_name, "w");
	if (log_file == NULL) {
		MSG("ERROR: impossible to create log file %s\n", log_file_name);
		exit(EXIT_FAILURE);
	}
	MSG("Writing to file: %s\n", log_file_name);

	fstep_nb = (int)((fmax - fmin) / fstep); /* Number of frequency steps */

	for (m=0; m<=fstep_nb; m++)
	{
		/* Set channel frequency */
		chan_freq = fmin + m * fstep;
		lgw_reg_w(LGW_FORCE_HOST_RADIO_CTRL, 1);
		sx125x_pll_lock(0, (uint32_t)(chan_freq) - IF_CHAN);
		lgw_reg_w(LGW_FORCE_HOST_RADIO_CTRL, 0);
		lgw_agc_start(radio_select, 0); /* Initialise AGC */

		/* Reset histogram */
		for (i=0; i<=NB_RSSI; i++) {
			rssi_hist[i] = 0;
		}

		MSG("Channel: %.3f MHz, Capturing...", chan_freq/1e6);

		for (k=0; k<nb_captures; k++)
		{
			#if (PRINT_CAPTURE > 0)
				if ((((k+1) % PRINT_CAPTURE) == 0) || (k == nb_captures-1)) {
					MSG(" %d", k+1);
				}
			#endif

			lgw_reg_w(LGW_CAPTURE_START,1);
			wait_ms(capture_wait);
			lgw_reg_w(LGW_CAPTURE_START,0);

			lgw_reg_w(LGW_CAPTURE_RAM_ADDR,0);
			for (i=0; i<64; i++) {
				lgw_reg_rb(LGW_CAPTURE_RAM_DATA, capture_buffer, 256);
				for (j=0; j<64; j++) {
					rssi = capture_buffer[j*4+3];
					if (rssi > NB_RSSI) {
						rssi = NB_RSSI;
					}
					rssi_hist[(int)rssi]++;
				}
			}
			if (k == nb_captures-1) {
				MSG(" done\n");
				rssi_20 = 0;
				rssi_50 = 0;
				rssi_80 = 0;
				rssi_cumu = 0;
				for (i=0; i<NB_RSSI; i++) {
					rssi_cumu = rssi_cumu + rssi_hist[i];
					if ((rssi_20 == 0) && (rssi_cumu > 0.2*(k+1)*4096)) {
						rssi_20 = i;
					}
					if ((rssi_50 == 0) && (rssi_cumu > 0.5*(k+1)*4096)) {
						rssi_50 = i;
					}
					if ((rssi_80 == 0) && (rssi_cumu > 0.8*(k+1)*4096)) {
						rssi_80 = i;
					}
				}
				MSG("RSSI 20%%: %d, 50%%: %d, 80%%: %d\n", rssi_20 + rssi_offset, rssi_50 + rssi_offset, rssi_80 + rssi_offset);
			}
		}

		fprintf_success = fprintf(log_file, "%lu", chan_freq);
		for (i=0; i<NB_RSSI; i++) {
			fprintf_success = fprintf(log_file, ", %d, %lu", i + rssi_offset, rssi_hist[i]);
		}
		fprintf_success = fprintf(log_file, "\n");
		if (fprintf_success < 0) {
			MSG("ERROR: impossible to write to file: %s\n", log_file_name);
			exit(EXIT_FAILURE);
		}

	}

	/* Close log file */
	fclose(log_file);
	DEBUG_MSG("log file %s closed\n", log_file_name);

	/* clean up before leaving */
	lgw_soft_reset();
	lgw_disconnect();
	MSG("concentrator stopped\n");

	MSG("Exiting capture program\n");
	return EXIT_SUCCESS;
}

/* --- EOF ------------------------------------------------------------------ */


/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2013 Semtech-Cycleo

Description:
	Configure LoRa concentrator and record received packets in a log file

License: Revised BSD License, see LICENSE.TXT file include in the project
Maintainer: Sylvain Miermont
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

#include "parson.h"
#include "loragw_hal.h"
#include "loragw_reg.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a)	(sizeof(a) / sizeof((a)[0]))
#define MSG(args...)	fprintf(stderr,"util_rx_test: " args) /* message that is destined to the user */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES (GLOBAL) ------------------------------------------- */

/* signal handling variables */
struct sigaction sigact; /* SIGQUIT&SIGINT&SIGTERM signal handling */
static int exit_sig = 0; /* 1 -> application terminates cleanly (shut down hardware, close open files, etc) */
static int quit_sig = 0; /* 1 -> application terminates without shutting down the hardware */

/* configuration variables needed by the application  */
uint64_t lgwm = 0; /* LoRa gateway MAC address */
char lgwm_str[17];

//struct lgw_conf_rxrf_s rfconf;
struct lgw_conf_rxif_s ifconf_tab[10];
struct lgw_conf_rxrf_s rfconf_tab[2];
uint32_t ifconf_freq_tab[10];
bool rfconf_enable_tab[2]={false, false};
uint32_t sx1301_rxbw_max;
int32_t sx1301_rssi_offset;
bool loramac_flag;
bool lgw_rxif_log[10];

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DECLARATION ---------------------------------------- */

static void sig_handler(int sigio);

int parse_SX1301_configuration(const char * conf_file);

int parse_gateway_configuration(const char * conf_file);

void open_log(void);

void usage (void);

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */

static void sig_handler(int sigio) {
	if (sigio == SIGQUIT) {
		quit_sig = 1;;
	} else if ((sigio == SIGINT) || (sigio == SIGTERM)) {
		exit_sig = 1;
	}
}

int parse_SX1301_configuration(const char * conf_file) {
	int i;
	const char conf_obj[] = "SX1301_conf";
	char param_name[32]; /* used to generate variable parameter names */

	struct lgw_conf_rxif_s ifconf;
	struct lgw_conf_rxrf_s rfconf;

	uint32_t rf_chain_freq_max[2]={0}, rf_chain_freq_min[2]={0};

	JSON_Value *root_val;
	JSON_Object *root = NULL;
	JSON_Object *conf = NULL;
	JSON_Value *val;
	uint32_t sf, bw;

	uint32_t freq_up_edge, freq_down_edge;
	uint32_t lora_std_bw = 0, fsk_bw = 0;

	/* try to parse JSON */
	root_val = json_parse_file_with_comments(conf_file);
	root = json_value_get_object(root_val);
	if (root == NULL) {
		MSG("ERROR: %s is not a valid JSON file\n", conf_file);
		exit(EXIT_FAILURE);
	}
	conf = json_object_get_object(root, conf_obj);
	if (conf == NULL) {
		MSG("INFO: %s does not contain a JSON object named %s\n", conf_file, conf_obj);
		return -1;
	} else {
		MSG("INFO: %s does contain a JSON object named %s, parsing SX1301 parameters\n", conf_file, conf_obj);
	}

	/* set configuration for LoRa multi-SF channels (bandwidth cannot be set) */
	for (i = 0; i < LGW_MULTI_NB; ++i) {
		memset(&ifconf, 0, sizeof(ifconf)); /* initialize configuration structure */
		sprintf(param_name, "chan_multiSF_%i", i); /* compose parameter path inside JSON structure */
		val = json_object_get_value(conf, param_name); /* fetch value (if possible) */
		if (json_value_get_type(val) != JSONObject) {
			MSG("INFO: no configuration for LoRa multi-SF channel %i\n", i);
			continue;
		}
		/* there is an object to configure that LoRa multi-SF channel, let's parse it */
		sprintf(param_name, "chan_multiSF_%i.enable", i);
		val = json_object_dotget_value(conf, param_name);
		if (json_value_get_type(val) == JSONBoolean) {
			ifconf.enable = (bool)json_value_get_boolean(val);
		} else {
			ifconf.enable = false;
		}

		// get log enable flag
		sprintf(param_name, "chan_multiSF_%i.log", i);
		val = json_object_dotget_value(conf, param_name);
		if (json_value_get_type(val) == JSONBoolean) {
			lgw_rxif_log[i] = (bool)json_value_get_boolean(val);
			if(lgw_rxif_log[i] == true){
				MSG("Chain %d log is enabled\n", i);
			}else{
				MSG("Chain %d log is disabled\n", i);
			}
		} else {
			lgw_rxif_log[i] = true;
			MSG("Chain %d log is enabled\n", i);
		}

		if (ifconf.enable == false) { /* LoRa multi-SF channel disabled, nothing else to parse */
			MSG("INFO: LoRa multi-SF channel %i disabled\n", i);
		} else  { /* LoRa multi-SF channel enabled, will parse the other parameters */
			sprintf(param_name, "chan_multiSF_%i.radio", i);
			ifconf.rf_chain = (uint32_t)json_object_dotget_number(conf, param_name);
			sprintf(param_name, "chan_multiSF_%i.freq", i);
			ifconf_freq_tab[i] = (uint32_t)((double)((json_object_dotget_number(conf, param_name) * 1e6)));
			ifconf.freq_hz = 0;
			// TODO: handle individual SF enabling and disabling (spread_factor)
			MSG("INFO: LoRa multi-SF channel %i enabled, radio %i selected, IF %i Hz, 125 kHz bandwidth, SF 7 to 12\n", i, ifconf.rf_chain, ifconf.freq_hz);
		}
		ifconf_tab[i] = ifconf;
	}

	/* set configuration for LoRa standard channel */
	memset(&ifconf, 0, sizeof(ifconf)); /* initialize configuration structure */
	val = json_object_get_value(conf, "chan_Lora_std"); /* fetch value (if possible) */
	if (json_value_get_type(val) != JSONObject) {
		MSG("INFO: no configuration for LoRa standard channel\n");
	} else {
		val = json_object_dotget_value(conf, "chan_Lora_std.enable");
		if (json_value_get_type(val) == JSONBoolean) {
			ifconf.enable = (bool)json_value_get_boolean(val);
		} else {
			ifconf.enable = false;
		}

		// get log enable flag
		val = json_object_dotget_value(conf, "chan_Lora_std.log");
		if (json_value_get_type(val) == JSONBoolean) {
			lgw_rxif_log[8] = (bool)json_value_get_boolean(val);
			if(lgw_rxif_log[8] == true){
				MSG("Chain 8 log is enabled\n");
			}else{
				MSG("Chain 8 log is disabled\n");
			}
		} else {
			MSG("Chain 8 log is enabled\n");
			lgw_rxif_log[8] = true;
		}


		if (ifconf.enable == false) {
			MSG("INFO: LoRa standard channel 8 disabled\n");
		} else  {
			ifconf.rf_chain = (uint32_t)json_object_dotget_number(conf, "chan_Lora_std.radio");
			ifconf_freq_tab[8] = (uint32_t)((double)((json_object_dotget_number(conf, "chan_Lora_std.freq") * 1e6)));
			ifconf.freq_hz = 0;
			bw = (uint32_t)json_object_dotget_number(conf, "chan_Lora_std.bandwidth");
			switch(bw) {
				case 500000:
					ifconf.bandwidth = BW_500KHZ;
					lora_std_bw = 500000;
					break;
				case 250000:
					ifconf.bandwidth = BW_250KHZ;
					lora_std_bw = 250000;
					break;
				case 125000:
					ifconf.bandwidth = BW_125KHZ;
					lora_std_bw = 125000;
					break;
				default:
					ifconf.bandwidth = BW_UNDEFINED;
					lora_std_bw = 0;
			}
			sf = (uint32_t)json_object_dotget_number(conf, "chan_Lora_std.spread_factor");
			switch(sf) {
				case  7: ifconf.datarate = DR_LORA_SF7;  break;
				case  8: ifconf.datarate = DR_LORA_SF8;  break;
				case  9: ifconf.datarate = DR_LORA_SF9;  break;
				case 10: ifconf.datarate = DR_LORA_SF10; break;
				case 11: ifconf.datarate = DR_LORA_SF11; break;
				case 12: ifconf.datarate = DR_LORA_SF12; break;
				default: ifconf.datarate = DR_UNDEFINED;
			}
			MSG("INFO: LoRa standard channel enabled, radio %i selected, IF %i Hz, %u Hz bandwidth, SF %u\n", ifconf.rf_chain, ifconf.freq_hz, bw, sf);
		}
		ifconf_tab[8] = ifconf;
	}

	/* set configuration for FSK channel */
	memset(&ifconf, 0, sizeof(ifconf)); /* initialize configuration structure */
	val = json_object_get_value(conf, "chan_FSK"); /* fetch value (if possible) */
	if (json_value_get_type(val) != JSONObject) {
		MSG("INFO: no configuration for FSK channel\n");
	} else {
		val = json_object_dotget_value(conf, "chan_FSK.enable");
		if (json_value_get_type(val) == JSONBoolean) {
			ifconf.enable = (bool)json_value_get_boolean(val);
		} else {
			ifconf.enable = false;
		}

		val = json_object_dotget_value(conf, "chan_FSK.log");
		if (json_value_get_type(val) == JSONBoolean) {
			lgw_rxif_log[9] = (bool)json_value_get_boolean(val);
			if(lgw_rxif_log[9] == true){
				MSG("Chain 9 log is enabled\n");
			}else{
				MSG("Chain 9 log is disabled\n");
			}
		} else {
			MSG("Chain 9 log is enabled\n");
			lgw_rxif_log[9] = false;
		}

		if (ifconf.enable == false) {
			MSG("INFO: FSK channel 9 disabled\n");
		} else  {
			ifconf.rf_chain = (uint32_t)json_object_dotget_number(conf, "chan_FSK.radio");
			ifconf_freq_tab[9] = (uint32_t)((double)((json_object_dotget_number(conf, "chan_FSK.freq") * 1e6)));
			bw = (uint32_t)json_object_dotget_number(conf, "chan_FSK.bandwidth");
			if (bw <= 7800){
				ifconf.bandwidth = BW_7K8HZ;
				fsk_bw = 7800;
			}else if (bw <= 15600){
				ifconf.bandwidth = BW_15K6HZ;
				fsk_bw = 15600;
			}else if (bw <= 31200){
				ifconf.bandwidth = BW_31K2HZ;
				fsk_bw = 31200;
			}else if (bw <= 62500){
				ifconf.bandwidth = BW_62K5HZ;
				fsk_bw = 62500;
			}else if (bw <= 125000){
				ifconf.bandwidth = BW_125KHZ;
				fsk_bw = 125000;
			}else if (bw <= 250000){
				ifconf.bandwidth = BW_250KHZ;
				fsk_bw = 250000;
			}else if (bw <= 500000){
				ifconf.bandwidth = BW_500KHZ;
				fsk_bw = 500000;
			}else{
				ifconf.bandwidth = BW_UNDEFINED;
				fsk_bw = 0;
			}
			ifconf.datarate = (uint32_t)json_object_dotget_number(conf, "chan_FSK.datarate");
			MSG("INFO: FSK channel enabled, radio %i selected, IF %i Hz, %u Hz bandwidth, %u bps datarate\n", ifconf.rf_chain, ifconf.freq_hz, bw, ifconf.datarate);
		}
		ifconf_tab[9] = ifconf;
	}

	sx1301_rxbw_max = (uint32_t)json_object_get_number(root, "rx_bandwidth_max");
	if(sx1301_rxbw_max == 0){
		sx1301_rxbw_max = 1000000;
		printf("SX1301 rx bandwidth has been set to %d\n", sx1301_rxbw_max);
	}else{
		printf("SX1301 rx bandwidth has been set to %d\n", sx1301_rxbw_max);
	}
	sx1301_rssi_offset = (int32_t)json_object_get_number(root, "rssi_offset");
	if(sx1301_rssi_offset == 0){
		sx1301_rssi_offset = -166;
		printf("SX1301 rssi_offset has been set to %d\n", sx1301_rssi_offset);
	}else{
		printf("SX1301 rssi_offset has been set to %d\n", sx1301_rssi_offset);
	}
	loramac_flag = (bool)json_object_get_boolean(root, "loramac");


	json_value_free(root_val);

	rfconf_enable_tab[0] = false;
	rfconf_enable_tab[1] = false;

	/** write configuration to sx1301 */
	for(i=0; i<LGW_IF_CHAIN_NB; i++){
		if(ifconf_freq_tab[i] == 0){
			printf("chan%d is empty\n", i);
			continue;
		}
		if(ifconf_tab[i].enable == false){
			printf("chan%d is disabled\n", i);
			continue;
		}
		printf("chan%d = %d\n", i, ifconf_freq_tab[i]);
		if( i < LGW_MULTI_NB){
			freq_up_edge = ifconf_freq_tab[i] + 125000/2;
			freq_down_edge = ifconf_freq_tab[i] - 125000/2;
		}else if( i == 8 ){
			freq_up_edge = ifconf_freq_tab[i] + lora_std_bw/2;
			freq_down_edge = ifconf_freq_tab[i] - lora_std_bw/2;
		}else if( i == 9 ){
			freq_up_edge = ifconf_freq_tab[i] + fsk_bw/2;
			freq_down_edge = ifconf_freq_tab[i] - fsk_bw/2;
		}

		switch(ifconf_tab[i].rf_chain){
		case 0:
			rfconf_enable_tab[0] = true;
			if(freq_up_edge > rf_chain_freq_max[0]){
				rf_chain_freq_max[0] = freq_up_edge;
			}
			if( (freq_down_edge < rf_chain_freq_min[0]) || (rf_chain_freq_min[0] == 0)){
				rf_chain_freq_min[0] = freq_down_edge;
			}
			break;
		case 1:
			rfconf_enable_tab[1] = true;
			if(freq_up_edge > rf_chain_freq_max[1]){
				rf_chain_freq_max[1] = freq_up_edge;
			}
			if( (freq_down_edge < rf_chain_freq_min[1]) || (rf_chain_freq_min[1] == 0)){
				rf_chain_freq_min[1] = freq_down_edge;
			}
			break;
		default:
			printf("RF Chain error");
			break;
		}
	}

	printf("RF Chain 0, Min: %d, Max: %d\n", rf_chain_freq_min[0], rf_chain_freq_max[0]);
	printf("RF Chain 1, Min: %d, Max: %d\n", rf_chain_freq_min[1], rf_chain_freq_max[1]);

	for (i = 0; i < LGW_RF_CHAIN_NB; ++i) {
		memset(&rfconf, 0, sizeof(rfconf)); /* initialize configuration structure */
		rfconf.freq_hz = (rf_chain_freq_min[i] + rf_chain_freq_max[i])/2;
		rfconf.enable = rfconf_enable_tab[i];
		rfconf.rssi_offset = sx1301_rssi_offset;
		if(rfconf.freq_hz > 520000000){
			rfconf.type = LGW_RADIO_TYPE_SX1257;
		}else{
			rfconf.type = LGW_RADIO_TYPE_SX1255;
		}
		if( (rf_chain_freq_max[i] - rf_chain_freq_min[i]) > sx1301_rxbw_max ){
			printf("RF Chian%d RX bandwidth (%d) is out of range (%d)\n", i, (rf_chain_freq_max[i] - rf_chain_freq_min[i]), sx1301_rxbw_max);
		}
		printf("RF Chian%d Central Frequency %d\n", i, rfconf.freq_hz);
		rfconf_tab[i] = rfconf;
		/* all parameters parsed, submitting configuration to the HAL */
		if (lgw_rxrf_setconf(i, rfconf) != LGW_HAL_SUCCESS) {
			MSG("WARNING: invalid configuration for radio %i\n", i);
		}
	}

	/** calculate if value */
	for( i=0; i<LGW_IF_CHAIN_NB; i++){
		if(ifconf_freq_tab[i] == 0){
			continue;
		}
		if(ifconf_tab[i].enable == false){
			continue;
		}
		switch(ifconf_tab[i].rf_chain){
		case 0:
			ifconf_tab[i].freq_hz = ifconf_freq_tab[i] - rfconf_tab[0].freq_hz;
			break;
		case 1:
			ifconf_tab[i].freq_hz = ifconf_freq_tab[i] - rfconf_tab[1].freq_hz;
			break;
		default:
			printf("RF Chain error");
			break;
		}
		//MSG("INFO: LoRa multi-SF channel %i enabled, radio %i selected, IF %i Hz, 125 kHz bandwidth, SF 7 to 12\n", i, ifconf_tab[i].rf_chain, ifconf_tab[i].freq_hz);
		MSG("chan: %i, radio: %i, IF: %iHz, BW: %d DR: %d\n", i, ifconf_tab[i].rf_chain, ifconf_tab[i].freq_hz, ifconf_tab[i].bandwidth, ifconf_tab[i].datarate);
		/* all parameters parsed, submitting configuration to the HAL */
		if (lgw_rxif_setconf(i, ifconf_tab[i]) != LGW_HAL_SUCCESS) {
			MSG("WARNING: invalid configuration for LoRa multi-SF channel %i\n", i);
		}
	}

	return 0;
}

/* describe command line options */
void usage(void) {
	printf("*** Library version information ***\n%s\n\n", lgw_version_info());
	printf( "Available options:\n");
	printf( " -h print this help\n");
	printf( " -c configuration file\n");
}

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main(int argc, char **argv)
{
	int i, j; /* loop and temporary variables */
	struct timespec sleep_time = {0, 3000000}; /* 3 ms */
	struct lgw_conf_board_s boardconf;

	/* configuration file related */
	const char *conf_fname = "config.json"; /* contain global (typ. network-wide) configuration */
	char buf[128];

	/* allocate memory for packet fetching and processing */
	struct lgw_pkt_rx_s rxpkt[16]; /* array containing up to 16 inbound packets metadata */
	struct lgw_pkt_rx_s *p; /* pointer on a RX packet */
	int nb_pkt;

	/* parse command line options */
	while ((i = getopt (argc, argv, "hc:")) != -1) {
		switch (i) {
			case 'h':
				usage();
				return EXIT_FAILURE;
				break;

			case 'c': /* -f <float> target frequency in MHz */
				sscanf(optarg, "%s", buf);
				conf_fname = buf;
				break;

			default:
				MSG("ERROR: argument parsing use -h option for help\n");
				usage();
				return EXIT_FAILURE;
		}
	}

	/* configure signal handling */
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigact.sa_handler = sig_handler;
	sigaction(SIGQUIT, &sigact, NULL);
	sigaction(SIGINT, &sigact, NULL);
	sigaction(SIGTERM, &sigact, NULL);

	/* configuration files management */
	if (access(conf_fname, R_OK) == 0) {
	/* if there is a global conf, parse it and then try to parse local conf  */
		MSG("INFO: found global configuration file %s, trying to parse it\n", conf_fname);
		parse_SX1301_configuration(conf_fname);
	} else {
		MSG("ERROR: failed to find any configuration file named %s\n", conf_fname);
		return EXIT_FAILURE;
	}

	printf("parse done.\n");

	boardconf.lorawan_public = true;
	boardconf.clksrc = 1;
	lgw_board_setconf(boardconf);

	/* starting the concentrator */
	i = lgw_start();
	if (i == LGW_HAL_SUCCESS) {
		MSG("INFO: concentrator started, packet can now be received\n");
	} else {
		MSG("ERROR: failed to start the concentrator\n");
		return EXIT_FAILURE;
	}

	if(loramac_flag){
		printf("LORAMAC MODE\n");
		lgw_reg_w(LGW_FRAME_SYNCH_PEAK1_POS,3); /* default 1 */
		lgw_reg_w(LGW_FRAME_SYNCH_PEAK2_POS,4); /* default 2 */
	}else{
		printf("NORMAL MODE\n");
		lgw_reg_w(LGW_FRAME_SYNCH_PEAK1_POS,1); /* default 1 */
		lgw_reg_w(LGW_FRAME_SYNCH_PEAK2_POS,2); /* default 2 */
	}

	/* main loop */
	while ((quit_sig != 1) && (exit_sig != 1)) {
		/* fetch packets */
		nb_pkt = lgw_receive(ARRAY_SIZE(rxpkt), rxpkt);

		if (nb_pkt == LGW_HAL_ERROR) {
			MSG("ERROR: failed packet fetch, exiting\n");
			return EXIT_FAILURE;
		} else if (nb_pkt == 0) {
			clock_nanosleep(CLOCK_MONOTONIC, 0, &sleep_time, NULL); /* wait a short time if no packets */
		} else {

		}

		/* log packets */
		for (i=0; i < nb_pkt; ++i) {
			p = &rxpkt[i];
			if(lgw_rxif_log[p->if_chain] == false){
				continue;
			}
			if (p->status == STAT_CRC_OK) {
				printf("---\nRcv pkt #%d >>", i+1);
				printf("freq:%d\n", ifconf_freq_tab[p->if_chain]);
				printf(" if_chain:%2d", p->if_chain);
				printf(" tstamp:%010u", p->count_us);
				printf(" size:%3u", p->size);
				switch (p-> modulation) {
					case MOD_LORA: printf(" LoRa"); break;
					case MOD_FSK: printf(" FSK"); break;
					default: printf(" modulation?");
				}
				switch (p->datarate) {
					case DR_LORA_SF7: printf(" SF7"); break;
					case DR_LORA_SF8: printf(" SF8"); break;
					case DR_LORA_SF9: printf(" SF9"); break;
					case DR_LORA_SF10: printf(" SF10"); break;
					case DR_LORA_SF11: printf(" SF11"); break;
					case DR_LORA_SF12: printf(" SF12"); break;
					default: printf(" datarate?");
				}
				switch (p->coderate) {
					case CR_LORA_4_5: printf(" CR1(4/5)"); break;
					case CR_LORA_4_6: printf(" CR2(2/3)"); break;
					case CR_LORA_4_7: printf(" CR3(4/7)"); break;
					case CR_LORA_4_8: printf(" CR4(1/2)"); break;
					default: printf(" coderate?");
				}
				printf("\n");
				printf(" RSSI:%+6.1f SNR:%+5.1f (min:%+5.1f, max:%+5.1f) payload:\n", p->rssi, p->snr, p->snr_min, p->snr_max);

				for (j = 0; j < p->size; ++j) {
					printf(" %02X", p->payload[j]);
				}
				printf(" #\n");
			} else if (p->status == STAT_CRC_BAD) {
				// printf(" if_chain:%2d", p->if_chain);
				// printf(" tstamp:%010u", p->count_us);
				// printf(" size:%3u\n", p->size);
				// printf(" CRC error, damaged packet\n\n");
			} else if (p->status == STAT_NO_CRC){
				printf(" if_chain:%2d", p->if_chain);
				printf(" tstamp:%010u", p->count_us);
				printf(" size:%3u\n", p->size);
				printf(" no CRC\n\n");
			} else {
				printf(" if_chain:%2d", p->if_chain);
				printf(" tstamp:%010u", p->count_us);
				printf(" size:%3u\n", p->size);
				printf(" invalid status ?!?\n\n");
			}
		}

	}

	if (exit_sig == 1) {
		/* clean up before leaving */
		i = lgw_stop();
		if (i == LGW_HAL_SUCCESS) {
			MSG("INFO: concentrator stopped successfully\n");
		} else {
			MSG("WARNING: failed to stop concentrator successfully\n");
		}
	}

	MSG("INFO: Exiting packet logger program\n");
	return EXIT_SUCCESS;
}

/* --- EOF ------------------------------------------------------------------ */

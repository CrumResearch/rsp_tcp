/*
 * rtl-sdr, turns your Realtek RTL2832 based DVB dongle into a SDR receiver
 * Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de>
 * Copyright (C) 2012-2013 by Hoernchen <la@tfc-server.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <mirsdrapi-rsp.h>

static pthread_t stdout_worker_thread;
//static pthread_cond_t exit_cond;
//static pthread_mutex_t exit_cond_lock;

static pthread_mutex_t ll_mutex;
static pthread_cond_t cond;

struct llist {
	char *data;
	size_t len;
	struct llist *next;
};

typedef struct { /* structure size must be multiple of 2 bytes */
	char magic[4];
	uint32_t tuner_type;
	uint32_t tuner_gain_count;
} dongle_info_t;

double atofs(char *s)
/* standard suffixes */
{
	char last;
	int len;
	double suff = 1.0;
	len = strlen(s);
	last = s[len-1];
	s[len-1] = '\0';
	switch (last) {
		case 'g':
		case 'G':
			suff *= 1e3;
			/* fall-through */
		case 'm':
		case 'M':
			suff *= 1e3;
			/* fall-through */
		case 'k':
		case 'K':
			suff *= 1e3;
			suff *= atof(s);
			s[len-1] = last;
			return suff;
	}
	s[len-1] = last;
	return atof(s);
}

static int global_numq = 0;
static struct llist *ll_buffers = 0;
static int llbuf_num = 512;
static int ignore_f_command = 0;
static int ignore_s_command = 0;

static volatile int do_exit = 0;
static volatile int ctrlC_exit = 0;

#define MAX_DEVS 8
#define WORKER_TIMEOUT_SEC 3
#define DEFAULT_BW_T mir_sdr_BW_1_536
#define DEFAULT_AGC_SETPOINT -40 // original -24 //Bas -34
#define DEFAULT_GAIN_REDUCTION 44 // original 40 //Bas 34
#define DEFAULT_LNA 0 // 0 = off to 9
#define RTLSDR_TUNER_R820T 5
#define MAX_DECIMATION_FACTOR 32

static int devModel = 0;
static int bwType = DEFAULT_BW_T;
static int agcSetPoint = DEFAULT_AGC_SETPOINT;
static int gainReduction = DEFAULT_GAIN_REDUCTION;
static int rspLNA = DEFAULT_LNA;
static int infoOverallGr;
static int samples_per_packet;
static int verbose = 0;
static int wideband = 2; // wideband 0=small / 1=wide / 2=optimised for samplerate
static int edgefilter = 0;

////waardes
static int devAvail = 0;
static int device = 0;
static int antenna = 0;
static int enable_biastee = 0;
static int enable_dabnotch = 1;
static int enable_broadcastnotch = 1;
static int enable_refout = 0;

////AGC beware to change all!
static int agc_type = mir_sdr_AGC_100HZ; //AGC 5-50-100HZ or DISABLE
static int agctype = 100; // just the number of above

static void sighandler(int signum)
{
	fprintf(stderr, "Signal (%d) caught, ask for exit!\n", signum);
	// exit(signum); // can be used to force ctrl-c instant
	do_exit = 1;
	pthread_cond_signal(&cond);
}

void gc_callback(unsigned int gRdB, unsigned int lnaGRdB, void* cbContext )
{
	if (gRdB == mir_sdr_ADC_OVERLOAD_DETECTED)
	{
		fprintf(stderr, "adc overload detected\n");
		mir_sdr_GainChangeCallbackMessageReceived(); 
	}
	else if (gRdB == mir_sdr_ADC_OVERLOAD_CORRECTED)
	{
		fprintf(stderr,"adc overload corrected\n");
		mir_sdr_GainChangeCallbackMessageReceived(); 
	}
	if (verbose)
		fprintf(stderr, "new gain reduction (%d), lna gain reduction (%d)\n", gRdB, lnaGRdB);
}

void rx_callback(short *xi, short *xq, unsigned int firstSampleNum, int grChanged, int rfChanged, int fsChanged, unsigned int numSamples, unsigned int reset, unsigned int hwRemoved, void* cbContext)
{
        unsigned int i;
	short xi2=0;
	short xq2=0;
        if(!do_exit) {
                struct llist *rpt = (struct llist*)malloc(sizeof(struct llist));
		rpt->data = (char*)malloc(2 * numSamples);
			// assemble the data
                        unsigned char *data;
                        data = (unsigned char*)rpt->data;

			for (i = 0; i < numSamples; i++, xi++, xq++) {
				xi2 = *xi + 1536;
				xq2 = *xq + 1536;

				if (*xi < -1536 || *xi > 1535 || *xq < -1536 || *xq > 1535) {
					xi2 = 0;
					xq2 = 0;
				}

				*(data++) = (unsigned char)(xi2 / 12);
                                *(data++) = (unsigned char)(xq2 / 12);

// I/Q value reader - if enabled show values
//if (*xi > 1500 || *xi < -1500 || *xq > 1500 || *xq < -1500) {
//fprintf(stderr,"xi=%hd,xq=%hd\n",(*xi),(*xq));}

                        rpt->len = 2 * numSamples;
                }

		rpt->next = NULL;

		pthread_mutex_lock(&ll_mutex);

		if (ll_buffers == NULL) {
			ll_buffers = rpt;
		} else {
			struct llist *cur = ll_buffers;
			int num_queued = 0;

			while (cur->next != NULL) {
				cur = cur->next;
				num_queued++;
			}

			if(llbuf_num && llbuf_num == num_queued-2){
				struct llist *curelem;

				free(ll_buffers->data);
				curelem = ll_buffers->next;
				free(ll_buffers);
				ll_buffers = curelem;
			}

			cur->next = rpt;

			global_numq = num_queued;
		}
		pthread_cond_signal(&cond);
		pthread_mutex_unlock(&ll_mutex);
	}
}

static void *stdout_worker(void *arg)
{
	struct llist *curelem,*prev;

	while(1) {
		if(do_exit)
			pthread_exit(0);

		pthread_mutex_lock(&ll_mutex);
		curelem = ll_buffers;
		ll_buffers = 0;
		pthread_mutex_unlock(&ll_mutex);

		while(curelem != 0) {
            fwrite(&curelem->data[0], 1, curelem->len, stdout);
			prev = curelem;
			curelem = curelem->next;
			free(prev->data);
			free(prev);
		}
	}
}

// gain reduction list in back order to emulate R820T gain
//--bas--
// const int gain_list[] = { 78, 75, 72, 69, 66, 63, 60, 57, 54, 51, 48, 45, 42, 39, 36, 33, 30, 27, 24, 21, 18, 15, 12, 9, 6, 3, 0 };
const int gain_list[] = { 40 };

struct command{
	unsigned char cmd;
	unsigned int param;
}__attribute__((packed));

void usage(void)
{
	fprintf(stderr, "rsp_stdout, an I/Q spectrum driver for SDRPlay receivers "
#ifdef SERVER_VERSION
		"VERSION "SERVER_VERSION
#endif
		"\n\n Usage:\n"
		"\t-d RSP device to use (default: 1, first found)\n"
		"\t-P Antenna Port select (0/1/2, default: 0, Port A)\n"
		"\t-r Gain reduction (default: 44  / values 20-59)\n"
		"\t-l Low Noise Amplifier level (default: 0 / values 0-9)\n"
		"\t-T Bias-T enable* (default: disabled)\n"
		"\t-D DAB bandfilter* (default: enabled)\n"
		"\t-B MW bandfilter* (default: enabled)\n"
		"\t-R Refclk output* (default: disabled)\n"
		"\t-f frequency to tune to [Hz] - If freq set centerfreq and progfreq is ignored!!\n"
		"\t-s samplerate in [Hz] - If sample rate is set it will be ignored from client!!\n"
		"\t-W wideband enable (default: 2 / values: 0 small / 1 wide / 2 = optimised)\n"
		"\t-E Edgefilter digital enable* (default: disabled)\n"
		"\t-A Auto Gain Control setpoint (default: -40 / values -1 to -69 / other disabled)\n"
		"\t-G Auto Gain Control speed in Hz (default: 100 / values 0/5/50/100)\n"
		"\t-n Max number of linked list buffers to keep (default: 512)\n"
		"\t-v Verbose output (debug) enable* (default: disabled)\n"
		"\n\t* marked options are switches they toggle on/off\n\n" );
	exit(1);
}

int main(int argc, char **argv)
{
	int r, opt, i;
	uint32_t frequency = 1000000;
	uint32_t samp_rate = 2048000;
	struct llist *curelem,*prev;
	pthread_attr_t attr;
	void *status;
	dongle_info_t dongle_info;

	float ver;
	mir_sdr_DeviceT devices[MAX_DEVS];
	unsigned int numDevs;
/////waardes

	struct sigaction sigact, sigign;

	while ((opt = getopt(argc, argv, "a:p:r:f:s:n:d:l:P:A:G:W:TvDBRE")) != -1) {
		switch (opt) {
		case 'd':
			device = atoi(optarg) - 1;
			break;
		case 'P':
			antenna = atoi(optarg);
			break;
		case 'r':
			gainReduction = atoi(optarg);
			break;
		case 'f':
			frequency = (uint32_t)atofs(optarg);
			ignore_f_command = 1;
			break;
		case 's':
			samp_rate = (uint32_t)atofs(optarg);
			ignore_s_command = 1;
			break;
		case 'A':
                        agcSetPoint = atoi(optarg);
                        break;
		case 'n':
			llbuf_num = atoi(optarg);
			break;
                case 'W':
                        wideband = atoi(optarg);
                        break;
		case 'E':
                        edgefilter = 1;
                        break;
		case 'l':
			rspLNA = atoi(optarg);
			break;
                case 'G':
                        agctype = atoi(optarg);
                        break;
		case 'T':
			enable_biastee = 1;
			break;
		case 'D':
                        enable_dabnotch = 0;
                        break;
		case 'B':
			enable_broadcastnotch = 0;
			break;
		case 'R':
			enable_refout = 1;
			break;
		case 'v':
			verbose = 1;
			break;
		default:
			usage();
			break;
		}
	}

	if (agctype == 5) agc_type = mir_sdr_AGC_5HZ;
	else if (agctype == 50) agc_type = mir_sdr_AGC_50HZ;
	else if (agctype == 100) agc_type = mir_sdr_AGC_100HZ;
	else { agc_type = mir_sdr_AGC_DISABLE;
		agctype = 0;}

	if (gainReduction < 20 || gainReduction > 59) gainReduction = DEFAULT_GAIN_REDUCTION;
	if (wideband < 0 || wideband > 2 ) wideband = 2;

	// check API version
	r = mir_sdr_ApiVersion(&ver);
	if (ver != MIR_SDR_API_VERSION) {
		//  Error detected, include file does not match dll. Deal with error condition.
		fprintf(stderr, "library libmirsdrapi-rsp must be version %f\n", ver);
		exit(1);
	}
	fprintf(stderr, "libmirsdrapi-rsp version %.2f found\n", ver);

	// enable debug output
	if (verbose)
		mir_sdr_DebugEnable(1);

        // select RSP device
        r = mir_sdr_GetDevices(&devices[0], &numDevs, MAX_DEVS);
        if (r != mir_sdr_Success) {
                fprintf(stderr, "Failed to get device list (%d)\n", r);
                exit(1);
        }

        for (i = 0; i < numDevs; i++) {
                if (devices[i].devAvail == 1) {
                        devAvail++;
                }
        }

        if (devAvail == 0) {
                fprintf(stderr, "no RSP devices available.\n");
                exit(1);
        }

        if (devices[device].devAvail != 1) {
                fprintf(stderr, "RSP selected (%d) is not available.\n", (device + 1));
                exit(1);
        }

        r = mir_sdr_SetDeviceIdx(device);
        if (r != mir_sdr_Success) {
                fprintf(stderr, "Failed to set device index (%d)\n", r);
                exit(1);
        }

	// get RSP model and display modelname.
	devModel = devices[device].hwVer;
	if (devModel == 1) fprintf(stderr, "detected RSP model (hw version %d) = RSP1\n", devModel);
	else if (devModel == 2) fprintf(stderr, "detected RSP model (hw version %d) = RSP2\n", devModel);
	else if (devModel == 3) fprintf(stderr, "detected RSP model (hw version %d) = RSPduo\n", devModel);
	else if (devModel == 255) fprintf(stderr, "detected RSP model (hw version %d) = RSP1A\n", devModel);
	else fprintf(stderr, "detected RSP model (hw version %d) = Unknown\n", devModel);

	// select antenna
	switch (antenna) {
		case 1:
			mir_sdr_RSPII_AntennaControl(mir_sdr_RSPII_ANTENNA_B);
			mir_sdr_AmPortSelect(0);
			break;
		case 2:
			mir_sdr_AmPortSelect(1);
			break;
		default:
			mir_sdr_RSPII_AntennaControl(mir_sdr_RSPII_ANTENNA_A);
			mir_sdr_AmPortSelect(0);
	}

	sigact.sa_handler = sighandler;
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigign.sa_handler = SIG_IGN;
	sigaction(SIGINT, &sigact, NULL);
	sigaction(SIGTERM, &sigact, NULL);
	sigaction(SIGQUIT, &sigact, NULL);
	sigaction(SIGPIPE, &sigign, NULL);

	//pthread_mutex_init(&exit_cond_lock, NULL);
	pthread_mutex_init(&ll_mutex, NULL);
	//pthread_mutex_init(&exit_cond_lock, NULL);
	pthread_cond_init(&cond, NULL);
	//pthread_cond_init(&exit_cond, NULL);

	while(1) {
		fprintf(stderr, "outputting to stdout...\n");

		fprintf(stderr, "parameters:\n");
        fprintf(stderr, "AGC-type set %dHz (0 means disabled)\n", agctype);
        fprintf(stderr, "Low-Noise-Amp mode set %u \n", rspLNA);
        fprintf(stderr, "Gain-Reduction set %d \n", gainReduction);
        fprintf(stderr, "AGC-Gain-Setpoint set %d \n", agcSetPoint);
        fprintf(stderr, "Edgefilter set %d (0=off 1=on)\n", edgefilter);

		memset(&dongle_info, 0, sizeof(dongle_info));
		memcpy(&dongle_info.magic, "RTL0", 4);

		dongle_info.tuner_type = htonl(RTLSDR_TUNER_R820T);
		dongle_info.tuner_gain_count = htonl(sizeof(gain_list)/sizeof(gain_list[0]) - 1);

		fwrite((const char *)&dongle_info, 1, sizeof(dongle_info), stdout);

		// must start the stdout_worker before the first samples are available from the rx
		// because the rx_callback tries to send a condition to the worker thread
		pthread_attr_init(&attr);
		pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
		r = pthread_create(&stdout_worker_thread, &attr, stdout_worker, NULL);
		// initialise API and start the rx
		// r = mir_sdr_StreamInit(&gainReduction, (samp_rate/1e6), (frequency/1e6), bwType, 0, rspLNA, &infoOverallGr, mir_sdr_USE_SET_GR_ALT_MODE, &samples_per_packet, rx_callback, gc_callback, (void *)NULL);
		// Changes by PA0SIM =============================
		r = mir_sdr_StreamInit(&gainReduction, (samp_rate/1e6), (frequency/1e6), bwType, 0, rspLNA, &infoOverallGr, mir_sdr_USE_RSP_SET_GR, &samples_per_packet, rx_callback, gc_callback, (void *)NULL);
		if (r != mir_sdr_Success)
		{
			fprintf(stderr, "failed to start the RSP device, return (%d)\n", r);
			break;
		}
		fprintf(stderr,"started rx\n");

		//Notches and other stuff must be here!!....

		// enable DC offset and IQ imbalance correction
	        mir_sdr_DCoffsetIQimbalanceControl(1, 1);
	        // enable AGC with a setPoint of -30dBfs
	        mir_sdr_AgcControl(agc_type, agcSetPoint, 0, 0, 0, 0, rspLNA);
	        // set the DC offset correction mode for the tuner (moved from below)
	        mir_sdr_SetDcMode(4, 1);
	        // set the time period over which the DC offset is tracked when in one shot mode.
	        mir_sdr_SetDcTrackTime(10);
	        // set Bias-T
	        mir_sdr_RSPII_BiasTControl(enable_biastee);
	        mir_sdr_rsp1a_BiasT(enable_biastee);
	        mir_sdr_rspDuo_BiasT(enable_biastee);
	        // set Notch
	        mir_sdr_RSPII_RfNotchEnable(enable_broadcastnotch);
	        mir_sdr_rsp1a_DabNotch(enable_dabnotch);
	        mir_sdr_rsp1a_BroadcastNotch(enable_broadcastnotch);
	        mir_sdr_rspDuo_DabNotch(enable_dabnotch);
	        mir_sdr_rspDuo_BroadcastNotch(enable_broadcastnotch);
	        mir_sdr_rspDuo_Tuner1AmNotch(enable_broadcastnotch);
	        // set external reference output
	        mir_sdr_RSPII_ExternalReferenceControl(enable_refout);
        	mir_sdr_rspDuo_ExtRef(enable_refout);

		pthread_attr_destroy(&attr);

		// wait for the workers to exit
		pthread_join(stdout_worker_thread, &status);

		// stop the receiver
		mir_sdr_StreamUninit();

		fprintf(stderr, "all threads dead..\n");

		curelem = ll_buffers;
		ll_buffers = 0;

		while(curelem != 0) {
			prev = curelem;
			curelem = curelem->next;
			free(prev->data);
			free(prev);
		}

		global_numq = 0;

		do_exit = 0;
	}

	mir_sdr_StreamUninit();
	mir_sdr_ReleaseDeviceIdx();

	fprintf(stderr, "bye!\n");
	return r >= 0 ? r : -r;
}

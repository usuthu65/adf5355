/*
 * Linux-userspace ADF5355 program using Analog Devices' no-OS driver.
 *
 * The no-OS driver calculates and writes the ADF5355 registers. This program
 * supplies command-line configuration and the Raspberry Pi SPI transport.
 */

#include <errno.h>
#include <getopt.h>
#include <inttypes.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "no_os_spi.h"
#include "linux_spi.h"
#include "adf5355.h"

#define DEFAULT_RFOUTA_HZ 2100000000ULL
#define DEFAULT_REFERENCE_HZ 125000000U
#define DEFAULT_CHARGE_PUMP_UA 3125U
#define DEFAULT_OUTPUT_POWER_DBM 2
#define DEFAULT_SPI_SPEED_HZ 10000000U
#define DEFAULT_SPI_DEVICE 0U
#define DEFAULT_SPI_CHIP_SELECT 0U

enum {
	OPT_REFERENCE_MODE = 1000,
	OPT_REF_DIV2,
	OPT_NO_REF_DIV2,
	OPT_REFERENCE_DOUBLER,
	OPT_NEGATIVE_BLEED,
	OPT_NO_NEGATIVE_BLEED,
	OPT_GATED_BLEED,
	OPT_MUTE_TILL_LOCK,
	OPT_MUXOUT,
	OPT_SPI_DEVICE,
	OPT_SPI_CHIP_SELECT,
};

static void print_usage(const char *program)
{
	printf(
		"Usage: %s [OPTIONS] [RFOUTA_HZ]\n"
		"\n"
		"Program RFOUTA through SPI0 using ADI's no-OS ADF5355 driver.\n"
		"The positional RFOUTA_HZ is retained for convenience; --rf-output-hz\n"
		"is preferred for scripts. All frequencies are in Hz.\n"
		"\n"
		"Frequency and reference options:\n"
		"  -f, --rf-output-hz HZ        RFOUTA frequency (default: %" PRIu64 ")\n"
		"  -r, --reference-hz HZ        REFIN frequency (default: %" PRIu32 ")\n"
		"      --reference-mode MODE    single-ended or differential\n"
		"                              (default: single-ended)\n"
		"      --ref-div2               Enable reference divide-by-2 (default)\n"
		"      --no-ref-div2            Disable reference divide-by-2\n"
		"      --reference-doubler      Enable the reference doubler\n"
		"\n"
		"Loop and output options:\n"
		"  -c, --charge-pump-current-ua UA\n"
		"                              315 to 5040; driver rounds to a CP code\n"
		"                              (default: %" PRIu32 " uA)\n"
		"  -p, --rf-output-power DBM    RFOUTA: -4, -1, 2, or 5\n"
		"                              (default: %d dBm)\n"
		"      --negative-bleed         Request fractional-N negative bleed (default)\n"
		"      --no-negative-bleed      Disable negative bleed\n"
		"      --gated-bleed            Enable gated negative bleed\n"
		"      --mute-till-lock         Enable RF mute-until-lock\n"
		"      --muxout MODE            digital, analog, r-divider, n-divider,\n"
		"                              high, low, or three-state (default: digital)\n"
		"\n"
		"SPI and reporting options:\n"
		"  -s, --spi-speed-hz HZ        SPI speed (default: %" PRIu32 ")\n"
		"      --spi-device N           Linux SPI controller (default: %" PRIu32 ")\n"
		"      --spi-chip-select N      Linux SPI chip select (default: %" PRIu32 ")\n"
		"  -v, --verbose                Print the selected configuration and registers\n"
		"  -h, --help                   Show this help and exit\n"
		"\n"
		"Fixed by the current ADI no-OS API: ADF5355 device type, RFOUTA channel,\n"
		"positive phase-detector polarity, MUXOUT 3.3 V logic, RFOUTB disabled,\n"
		"and automatic R-counter/MOD2 selection. It does not expose Python's\n"
		"channel-spacing, GPIO digital-lock wait, or lock-time measurement features.\n",
		program, (uint64_t)DEFAULT_RFOUTA_HZ,
		(uint32_t)DEFAULT_REFERENCE_HZ,
		(uint32_t)DEFAULT_CHARGE_PUMP_UA, DEFAULT_OUTPUT_POWER_DBM,
		(uint32_t)DEFAULT_SPI_SPEED_HZ, (uint32_t)DEFAULT_SPI_DEVICE,
		(uint32_t)DEFAULT_SPI_CHIP_SELECT);
}

static int parse_u64(const char *text, uint64_t maximum, uint64_t *value)
{
	char *end;
	unsigned long long parsed;

	errno = 0;
	parsed = strtoull(text, &end, 0);
	if (errno || *text == '\0' || *end != '\0' || parsed == 0 ||
	    parsed > maximum)
		return -1;

	*value = parsed;
	return 0;
}

static int parse_u32(const char *text, uint32_t *value)
{
	uint64_t parsed;

	if (parse_u64(text, UINT32_MAX, &parsed))
		return -1;

	*value = (uint32_t)parsed;
	return 0;
}

static int parse_output_power(const char *text, uint8_t *power_code,
			      int *power_dbm)
{
	char *end;
	long parsed;

	errno = 0;
	parsed = strtol(text, &end, 10);
	if (errno || *text == '\0' || *end != '\0')
		return -1;

	switch (parsed) {
	case -4:
		*power_code = 0;
		break;
	case -1:
		*power_code = 1;
		break;
	case 2:
		*power_code = 2;
		break;
	case 5:
		*power_code = 3;
		break;
	default:
		return -1;
	}

	*power_dbm = (int)parsed;
	return 0;
}

static int parse_reference_mode(const char *text, bool *differential)
{
	if (!strcmp(text, "single-ended")) {
		*differential = false;
		return 0;
	}

	if (!strcmp(text, "differential")) {
		*differential = true;
		return 0;
	}

	return -1;
}

static int parse_muxout(const char *text, enum adf5355_mux_out_sel *muxout)
{
	if (!strcmp(text, "three-state")) {
		*muxout = ADF5355_MUXOUT_THREESTATE;
	} else if (!strcmp(text, "high")) {
		*muxout = ADF5355_MUXOUT_DVDD;
	} else if (!strcmp(text, "low")) {
		*muxout = ADF5355_MUXOUT_GND;
	} else if (!strcmp(text, "r-divider")) {
		*muxout = ADF5355_MUXOUT_R_DIV_OUT;
	} else if (!strcmp(text, "n-divider")) {
		*muxout = ADF5355_MUXOUT_N_DIV_OUT;
	} else if (!strcmp(text, "analog")) {
		*muxout = ADF5355_MUXOUT_ANALOG_LOCK_DETECT;
	} else if (!strcmp(text, "digital")) {
		*muxout = ADF5355_MUXOUT_DIGITAL_LOCK_DETECT;
	} else {
		return -1;
	}

	return 0;
}

static const char *muxout_name(enum adf5355_mux_out_sel muxout)
{
	static const char *const names[] = {
		"three-state", "high", "low", "r-divider", "n-divider",
		"analog", "digital"
	};

	if ((unsigned int)muxout >= sizeof(names) / sizeof(names[0]))
		return "unknown";

	return names[muxout];
}

static void print_configuration(const struct adf5355_init_param *init,
				const struct no_os_spi_init_param *spi,
				const struct adf5355_dev *device,
				int output_power_dbm, bool verbose)
{
	bool fractional = device->fract1 || device->fract2;
	uint32_t cp_code = (device->regs[ADF5355_REG(4)] >> 10) & 0xF;
	uint32_t bleed_code = (device->regs[ADF5355_REG(6)] >> 13) & 0xFF;
	bool negative_bleed = (device->regs[ADF5355_REG(6)] >> 29) & 1;
	bool gated_bleed = (device->regs[ADF5355_REG(6)] >> 30) & 1;
	unsigned int reg;

	printf("ADF5355 configuration:\n");
	printf("  RFOUTA: %" PRIu64 " Hz (%s-N)\n", device->freq_req,
	       fractional ? "fractional" : "integer");
	printf("  REFIN: %" PRIu32 " Hz, %s, RDiv2 %s, doubler %s\n",
	       init->clkin_freq, init->ref_diff_en ? "differential" : "single-ended",
	       init->ref_div2_en ? "enabled" : "disabled",
	       init->ref_doubler_en ? "enabled" : "disabled");
	printf("  PFD: %" PRIu32 " Hz; R counter: %" PRIu16
	       "; RF divider code: %" PRIu8 "\n",
	       device->fpfd, device->ref_div_factor, device->rf_div_sel);
	printf("  N: INT=%" PRIu32 ", FRAC1=%" PRIu32
	       ", FRAC2=%" PRIu32 ", MOD2=%" PRIu32 "\n",
	       device->integer, device->fract1, device->fract2, device->mod2);
	printf("  Charge pump: requested %" PRIu32 " uA; Register 4 code %" PRIu32 "\n",
	       init->cp_ua, cp_code);
	printf("  Negative bleed: %s; code %" PRIu32 "; gated %s\n",
	       negative_bleed ? "enabled" : "disabled", bleed_code,
	       gated_bleed ? "enabled" : "disabled");
	printf("  RFOUTA power: %d dBm; mute-till-lock %s\n",
	       output_power_dbm, init->mute_till_lock_en ? "enabled" : "disabled");
	printf("  MUXOUT: %s; SPI: /dev/spidev%" PRIu32 ".%" PRIu8
	       " at %" PRIu32 " Hz\n",
	       muxout_name(init->mux_out_sel), spi->device_id, spi->chip_select,
	       spi->max_speed_hz);

	if (!verbose)
		return;

	printf("  Register map:\n");
	for (reg = 0; reg <= 12; reg++)
		printf("    R%u = 0x%08" PRIX32 "\n", reg, device->regs[reg]);
}

int main(int argc, char **argv)
{
	struct no_os_spi_init_param spi_init = {
		.device_id = DEFAULT_SPI_DEVICE,
		.max_speed_hz = DEFAULT_SPI_SPEED_HZ,
		.chip_select = DEFAULT_SPI_CHIP_SELECT,
		.mode = NO_OS_SPI_MODE_0,
		.bit_order = NO_OS_SPI_BIT_ORDER_MSB_FIRST,
		.lanes = NO_OS_SPI_SINGLE_LANE,
		.platform_ops = &linux_spi_ops,
	};
	struct adf5355_init_param init = {
		.spi_init = &spi_init,
		.dev_id = ADF5355,
		.freq_req = DEFAULT_RFOUTA_HZ,
		.freq_req_chan = 0,
		.clkin_freq = DEFAULT_REFERENCE_HZ,
		.cp_ua = DEFAULT_CHARGE_PUMP_UA,
		.cp_neg_bleed_en = true,
		.cp_gated_bleed_en = false,
		.cp_bleed_current_polarity_en = false,
		.mute_till_lock_en = false,
		.outa_en = true,
		.outb_en = false,
		.outa_power = 2,
		.outb_power = 0,
		.phase_detector_polarity_neg = false,
		.ref_diff_en = false,
		.mux_out_3v3_en = true,
		.ref_doubler_en = false,
		.ref_div2_en = true,
		.mux_out_sel = ADF5355_MUXOUT_DIGITAL_LOCK_DETECT,
		.outb_sel_fund = false,
	};
	static const struct option long_options[] = {
		{"rf-output-hz", required_argument, NULL, 'f'},
		{"reference-hz", required_argument, NULL, 'r'},
		{"charge-pump-current-ua", required_argument, NULL, 'c'},
		{"rf-output-power", required_argument, NULL, 'p'},
		{"spi-speed-hz", required_argument, NULL, 's'},
		{"verbose", no_argument, NULL, 'v'},
		{"help", no_argument, NULL, 'h'},
		{"reference-mode", required_argument, NULL, OPT_REFERENCE_MODE},
		{"ref-div2", no_argument, NULL, OPT_REF_DIV2},
		{"no-ref-div2", no_argument, NULL, OPT_NO_REF_DIV2},
		{"reference-doubler", no_argument, NULL, OPT_REFERENCE_DOUBLER},
		{"negative-bleed", no_argument, NULL, OPT_NEGATIVE_BLEED},
		{"no-negative-bleed", no_argument, NULL, OPT_NO_NEGATIVE_BLEED},
		{"gated-bleed", no_argument, NULL, OPT_GATED_BLEED},
		{"mute-till-lock", no_argument, NULL, OPT_MUTE_TILL_LOCK},
		{"muxout", required_argument, NULL, OPT_MUXOUT},
		{"spi-device", required_argument, NULL, OPT_SPI_DEVICE},
		{"spi-chip-select", required_argument, NULL, OPT_SPI_CHIP_SELECT},
		{0, 0, 0, 0},
	};
	struct adf5355_dev *adf5355 = NULL;
	bool frequency_set = false;
	bool verbose = false;
	int output_power_dbm = DEFAULT_OUTPUT_POWER_DBM;
	int option;
	int32_t ret;
	uint64_t parsed_u64;
	uint32_t parsed_u32;

	opterr = 0;
	while ((option = getopt_long(argc, argv, "f:r:c:p:s:vh",
				    long_options, NULL)) != -1) {
		switch (option) {
		case 'f':
			if (frequency_set ||
			    parse_u64(optarg, UINT64_MAX, &init.freq_req)) {
				fprintf(stderr, "Invalid RFOUTA frequency: %s\n", optarg);
				return EXIT_FAILURE;
			}
			frequency_set = true;
			break;
		case 'r':
			if (parse_u32(optarg, &init.clkin_freq)) {
				fprintf(stderr, "Invalid reference frequency: %s\n", optarg);
				return EXIT_FAILURE;
			}
			break;
		case 'c':
			if (parse_u32(optarg, &init.cp_ua)) {
				fprintf(stderr, "Invalid charge-pump current: %s\n", optarg);
				return EXIT_FAILURE;
			}
			break;
		case 'p':
			if (parse_output_power(optarg, &init.outa_power,
					       &output_power_dbm)) {
				fprintf(stderr, "RFOUTA power must be -4, -1, 2, or 5 dBm\n");
				return EXIT_FAILURE;
			}
			break;
		case 's':
			if (parse_u32(optarg, &spi_init.max_speed_hz)) {
				fprintf(stderr, "Invalid SPI speed: %s\n", optarg);
				return EXIT_FAILURE;
			}
			break;
		case 'v':
			verbose = true;
			break;
		case OPT_REFERENCE_MODE:
			if (parse_reference_mode(optarg, &init.ref_diff_en)) {
				fprintf(stderr,
					"Reference mode must be single-ended or differential\n");
				return EXIT_FAILURE;
			}
			break;
		case OPT_REF_DIV2:
			init.ref_div2_en = true;
			break;
		case OPT_NO_REF_DIV2:
			init.ref_div2_en = false;
			break;
		case OPT_REFERENCE_DOUBLER:
			init.ref_doubler_en = true;
			break;
		case OPT_NEGATIVE_BLEED:
			init.cp_neg_bleed_en = true;
			break;
		case OPT_NO_NEGATIVE_BLEED:
			init.cp_neg_bleed_en = false;
			break;
		case OPT_GATED_BLEED:
			init.cp_gated_bleed_en = true;
			break;
		case OPT_MUTE_TILL_LOCK:
			init.mute_till_lock_en = true;
			break;
		case OPT_MUXOUT:
			if (parse_muxout(optarg, &init.mux_out_sel)) {
				fprintf(stderr, "Invalid MUXOUT mode: %s\n", optarg);
				return EXIT_FAILURE;
			}
			break;
		case OPT_SPI_DEVICE:
			if (parse_u32(optarg, &spi_init.device_id)) {
				fprintf(stderr, "Invalid SPI device: %s\n", optarg);
				return EXIT_FAILURE;
			}
			break;
		case OPT_SPI_CHIP_SELECT:
			if (parse_u32(optarg, &parsed_u32) || parsed_u32 > UINT8_MAX) {
				fprintf(stderr, "Invalid SPI chip select: %s\n", optarg);
				return EXIT_FAILURE;
			}
			spi_init.chip_select = (uint8_t)parsed_u32;
			break;
		case 'h':
			print_usage(argv[0]);
			return EXIT_SUCCESS;
		default:
			print_usage(argv[0]);
			return EXIT_FAILURE;
		}
	}

	if (optind + 1 < argc ||
	    (optind < argc &&
	     (frequency_set ||
	      parse_u64(argv[optind], UINT64_MAX, &parsed_u64)))) {
		fprintf(stderr, "Specify RFOUTA_HZ once, as an option or positionally.\n");
		return EXIT_FAILURE;
	}

	if (optind < argc)
		init.freq_req = parsed_u64;

	if (init.clkin_freq < 10000000U ||
	    init.clkin_freq > (init.ref_diff_en ? 600000000U : 250000000U)) {
		fprintf(stderr, "Reference frequency is outside the selected input-mode range\n");
		return EXIT_FAILURE;
	}

	if (init.cp_ua < 315U || init.cp_ua > 5040U) {
		fprintf(stderr, "Charge-pump current must be from 315 to 5040 uA\n");
		return EXIT_FAILURE;
	}

	if (init.cp_gated_bleed_en && !init.cp_neg_bleed_en) {
		fprintf(stderr, "--gated-bleed requires --negative-bleed\n");
		return EXIT_FAILURE;
	}

	ret = adf5355_init(&adf5355, &init);
	if (ret) {
		fprintf(stderr, "adf5355_init failed: %" PRId32 "\n", ret);
		return EXIT_FAILURE;
	}

	print_configuration(&init, &spi_init, adf5355, output_power_dbm, verbose);

	ret = adf5355_remove(adf5355);
	if (ret) {
		fprintf(stderr, "adf5355_remove failed: %" PRId32 "\n", ret);
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}


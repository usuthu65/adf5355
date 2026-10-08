/*
 * Linux-userspace ADF5355 program using Analog Devices' immutable no-OS
 * driver for device/SPI setup and a Python-compatible userspace register map.
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
#define DEFAULT_CHANNEL_SPACING_HZ 200000U
#define DEFAULT_CHARGE_PUMP_UA 3125U
#define DEFAULT_OUTPUT_POWER_DBM 2
#define DEFAULT_SPI_SPEED_HZ 10000000U
#define DEFAULT_SPI_DEVICE 0U
#define DEFAULT_SPI_CHIP_SELECT 0U

#define MOD1 16777216ULL
#define MIN_PFD_HZ 10000000ULL
#define MAX_PFD_HZ 125000000ULL
#define MIN_RFOUTA_HZ 54000000ULL
#define MAX_RFOUTA_HZ 6800000000ULL

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
	OPT_CHANNEL_SPACING,
	OPT_DRY_RUN,
};

struct python_compatible_parameters {
	uint64_t rfouta_hz;
	uint64_t vco_hz;
	uint64_t pfd_num;
	uint64_t pfd_den;
	uint32_t r_counter;
	uint32_t rf_divider;
	uint32_t integer;
	uint32_t frac1;
	uint32_t frac2;
	uint32_t mod2;
	uint32_t adc_divider;
	uint8_t cp_code;
	bool negative_bleed_enabled;
	uint8_t negative_bleed_code;
	uint32_t registers[13];
};

static int32_t deferred_spi_write_and_read(struct no_os_spi_desc *desc,
					   uint8_t *data, uint16_t bytes)
{
	(void)desc;
	(void)data;
	(void)bytes;
	return 0;
}

static int32_t deferred_spi_init(
	struct no_os_spi_desc **desc,
	const struct no_os_spi_init_param *parameters)
{
	return linux_spi_ops.init(desc, parameters);
}

static int32_t deferred_spi_remove(struct no_os_spi_desc *desc)
{
	return linux_spi_ops.remove(desc);
}

/*
 * adf5355_init() normally writes immediately. This wrapper opens the real
 * Linux spidev device but discards those first writes; the application later
 * sends its corrected Python-compatible map through linux_spi_ops.
 */
static const struct no_os_spi_platform_ops deferred_linux_spi_ops = {
	.init = deferred_spi_init,
	.write_and_read = deferred_spi_write_and_read,
	.remove = deferred_spi_remove,
};

static uint64_t gcd_u64(uint64_t left, uint64_t right)
{
	while (right) {
		uint64_t remainder = left % right;
		left = right;
		right = remainder;
	}

	return left;
}

static uint64_t ceil_div_u128(__uint128_t numerator, __uint128_t denominator)
{
	return (uint64_t)((numerator + denominator - 1) / denominator);
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
	case -4: *power_code = 0; break;
	case -1: *power_code = 1; break;
	case 2: *power_code = 2; break;
	case 5: *power_code = 3; break;
	default: return -1;
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
	if (!strcmp(text, "three-state"))
		*muxout = ADF5355_MUXOUT_THREESTATE;
	else if (!strcmp(text, "high"))
		*muxout = ADF5355_MUXOUT_DVDD;
	else if (!strcmp(text, "low"))
		*muxout = ADF5355_MUXOUT_GND;
	else if (!strcmp(text, "r-divider"))
		*muxout = ADF5355_MUXOUT_R_DIV_OUT;
	else if (!strcmp(text, "n-divider"))
		*muxout = ADF5355_MUXOUT_N_DIV_OUT;
	else if (!strcmp(text, "analog"))
		*muxout = ADF5355_MUXOUT_ANALOG_LOCK_DETECT;
	else if (!strcmp(text, "digital"))
		*muxout = ADF5355_MUXOUT_DIGITAL_LOCK_DETECT;
	else
		return -1;

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

static uint8_t charge_pump_code(uint32_t requested_ua)
{
	/* no-OS's documented current-to-code rounding, expressed explicitly. */
	return (uint8_t)((requested_ua - 315U + 157U) / 315U);
}

static int select_rf_divider(uint64_t rfouta_hz, uint32_t *divider,
			     uint32_t *divider_code)
{
	uint32_t candidate;

	for (candidate = 1; candidate <= 64; candidate <<= 1) {
		if (rfouta_hz >= 3400000000ULL / candidate &&
		    rfouta_hz <= 6800000000ULL / candidate) {
			*divider = candidate;
			*divider_code = 0;
			while ((1U << *divider_code) != candidate)
				(*divider_code)++;
			return 0;
		}
	}

	return -1;
}

static int calculate_python_compatible_parameters(
	const struct adf5355_init_param *init, uint32_t channel_spacing_hz,
	struct python_compatible_parameters *parameters)
{
	uint64_t base_num;
	uint64_t base_den;
	uint64_t pfd_num;
	uint64_t pfd_den;
	uint64_t reference_divider;
	uint64_t n_numerator;
	uint64_t fractional_numerator;
	uint64_t frac1;
	uint64_t residue;
	uint64_t mod2;
	uint64_t frac2;
	uint64_t pfd_gcd;
	uint64_t timeout_synth;
	uint64_t timeout_alc;
	uint64_t timeout;
	uint64_t vco_band_divider;
	uint64_t adc_divider;
	uint64_t cp_code;
	uint32_t rf_divider_code;
	__uint128_t product;

	if (init->freq_req < MIN_RFOUTA_HZ || init->freq_req > MAX_RFOUTA_HZ ||
	    !channel_spacing_hz)
		return -1;

	if (init->clkin_freq < MIN_PFD_HZ ||
	    init->clkin_freq > (init->ref_diff_en ? 600000000U : 250000000U))
		return -1;

	if (init->cp_ua < 315U || init->cp_ua > 5040U)
		return -1;

	if (select_rf_divider(init->freq_req, &parameters->rf_divider,
			      &rf_divider_code))
		return -1;

	/* Python chooses RDiv2 for REFIN >= 20 MHz, then maximizes fPFD <=125 MHz. */
	base_num = (uint64_t)init->clkin_freq *
		   (init->ref_doubler_en ? 2U : 1U);
	base_den = init->ref_div2_en ? 2U : 1U;
	reference_divider = ceil_div_u128(base_num,
					 (__uint128_t)base_den * MAX_PFD_HZ);
	if (!reference_divider || reference_divider > 1023)
		return -1;

	pfd_num = base_num;
	pfd_den = base_den * reference_divider;
	pfd_gcd = gcd_u64(pfd_num, pfd_den);
	pfd_num /= pfd_gcd;
	pfd_den /= pfd_gcd;

	parameters->vco_hz = init->freq_req * parameters->rf_divider;
	product = (__uint128_t)parameters->vco_hz * pfd_den;
	n_numerator = (uint64_t)product;
	parameters->integer = n_numerator / pfd_num;
	fractional_numerator = n_numerator % pfd_num;
	if (parameters->integer < 23 || parameters->integer > 32767)
		return -1;

	product = (__uint128_t)fractional_numerator * MOD1;
	frac1 = (uint64_t)(product / pfd_num);
	residue = (uint64_t)(product - (__uint128_t)frac1 * pfd_num);

	if (!residue) {
		mod2 = 2;
		frac2 = 0;
	} else {
		pfd_gcd = gcd_u64(pfd_num,
				   (uint64_t)channel_spacing_hz * pfd_den);
		mod2 = pfd_num / pfd_gcd;
		if (mod2 < 2 || mod2 > 16383)
			return -1;
		product = (__uint128_t)residue * mod2;
		if (product % pfd_num)
			return -1;
		frac2 = (uint64_t)(product / pfd_num);
		if (frac2 >= mod2)
			return -1;
	}

	cp_code = charge_pump_code(init->cp_ua);
	if (cp_code > 15)
		return -1;

	parameters->rfouta_hz = init->freq_req;
	parameters->pfd_num = pfd_num;
	parameters->pfd_den = pfd_den;
	parameters->r_counter = (uint32_t)reference_divider;
	parameters->frac1 = (uint32_t)frac1;
	parameters->frac2 = (uint32_t)frac2;
	parameters->mod2 = (uint32_t)mod2;
	parameters->cp_code = (uint8_t)cp_code;
	parameters->negative_bleed_enabled =
		init->cp_neg_bleed_en && (frac1 || frac2) &&
		pfd_num <= 100000000ULL * pfd_den;
	parameters->negative_bleed_code = 0;
	if (parameters->negative_bleed_enabled) {
		parameters->negative_bleed_code =
			(uint8_t)(4U * 256U / parameters->integer + 1U);
	}

	/* Python Register 0 through Register 8. */
	parameters->registers[0] =
		(parameters->integer << 4) |
		(parameters->integer >= 75 ? (1U << 20) : 0U) |
		(1U << 21);
	parameters->registers[1] = parameters->frac1 << 4 | 1U;
	parameters->registers[2] = parameters->mod2 << 4 |
		parameters->frac2 << 18 | 2U;
	parameters->registers[3] = 3U;
	parameters->registers[4] =
		((uint32_t)init->mux_out_sel << 27) |
		(1U << 8) |
		(init->ref_diff_en ? (1U << 9) : 0U) |
		((uint32_t)parameters->cp_code << 10) |
		(1U << 7) | (1U << 14) |
		(parameters->r_counter << 15) |
		(init->ref_div2_en ? (1U << 25) : 0U) |
		(init->ref_doubler_en ? (1U << 26) : 0U) | 4U;
	parameters->registers[5] = 0x00800025U;
	parameters->registers[6] = 0x14000006U |
		(parameters->rf_divider == 1 ? 0U :
		 ((uint32_t)__builtin_ctz(parameters->rf_divider) << 21)) |
		(1U << 24) |
		((uint32_t)init->outa_power << 4) |
		(init->mute_till_lock_en ? (1U << 11) : 0U) |
		(1U << 6);
	if (parameters->negative_bleed_enabled) {
		parameters->registers[6] |= (1U << 29) |
			((uint32_t)parameters->negative_bleed_code << 13);
		if (init->cp_gated_bleed_en)
			parameters->registers[6] |= 1U << 30;
	}
	/*
	 * DB4 selects the lock-detect mode. Use the fractional-N, 12 ns
	 * precision mode when either fraction is nonzero; use the 2.9 ns
	 * integer-N mode otherwise. The other fields retain the Python
	 * settings: reserved bits, LE synchronization, and LOL disabled.
	 */
	parameters->registers[7] = (frac1 || frac2) ?
		0x12000067U : 0x12000077U;
	parameters->registers[8] = 0x102D0428U;

	vco_band_divider = ceil_div_u128(pfd_num,
					 (__uint128_t)pfd_den * 2400000U);
	timeout_synth = ceil_div_u128((__uint128_t)pfd_num * 20000U,
				      (__uint128_t)pfd_den * 1000000000U * 12U);
	timeout_alc = (uint64_t)((__uint128_t)pfd_num * 50000U /
				 ((__uint128_t)pfd_den * 1000000000U * 30U)) + 1U;
	timeout = timeout_synth > timeout_alc ? timeout_synth : timeout_alc;
	if (!vco_band_divider || vco_band_divider > 255 || !timeout ||
	    timeout > 1023)
		return -1;
	parameters->registers[9] = (uint32_t)(vco_band_divider << 24) |
		((uint32_t)timeout << 14) | (30U << 9) | (12U << 4) | 9U;

	/* adc_div = clamp(ceil(((fPFD / 100 kHz) - 2) / 4), 1, 255). */
	product = (__uint128_t)pfd_num;
	if (product <= (__uint128_t)200000U * pfd_den)
		adc_divider = 1;
	else
		adc_divider = ceil_div_u128(
			product - (__uint128_t)200000U * pfd_den,
			(__uint128_t)400000U * pfd_den);
	if (adc_divider < 1)
		adc_divider = 1;
	if (adc_divider > 255)
		adc_divider = 255;
	parameters->adc_divider = (uint32_t)adc_divider;
	parameters->registers[10] = 0x00C0000AU | (1U << 4) | (1U << 5) |
		((uint32_t)adc_divider << 6);
	parameters->registers[11] = 0x0061300BU;
	parameters->registers[12] = 0x0001041CU;

	return 0;
}

static int32_t write_register_word(struct no_os_spi_desc *spi, uint32_t word)
{
	uint8_t bytes[4] = {
		(uint8_t)(word >> 24), (uint8_t)(word >> 16),
		(uint8_t)(word >> 8), (uint8_t)word
	};

	return no_os_spi_write_and_read(spi, bytes, sizeof(bytes));
}

static int32_t program_compatible_register_map(
	struct no_os_spi_desc *spi,
	const struct python_compatible_parameters *parameters)
{
	int reg;
	int32_t ret;

	for (reg = 12; reg >= 0; reg--) {
		ret = write_register_word(spi, parameters->registers[reg]);
		if (ret)
			return ret;
	}

	return 0;
}

static void print_initialization_register_report(
	const struct python_compatible_parameters *parameters)
{
	unsigned int step;
	unsigned int reg;

	printf("Beginning Register Initialization Sequence: 13 register writes\n");
	for (step = 1; step <= 13; step++) {
		reg = 13 - step;
		printf("Step %02u/13: initialization; Register %u; value=0x%08"
		       PRIX32 "; start=n/a; end=n/a; transfer=n/a; gap=n/a\n",
		       step, reg, parameters->registers[reg]);
	}
}

static void print_configuration(const struct adf5355_init_param *init,
				const struct no_os_spi_init_param *spi,
				const struct python_compatible_parameters *parameters,
				int output_power_dbm, uint32_t channel_spacing_hz,
				bool verbose)
{
	bool fractional = parameters->frac1 || parameters->frac2;

	printf("ADF5355 configuration:\n");
	printf("  RFOUTA: %" PRIu64 " Hz (%s-N)\n", parameters->rfouta_hz,
	       fractional ? "fractional" : "integer");
	printf("  REFIN: %" PRIu32 " Hz, %s, RDiv2 %s, doubler %s\n",
	       init->clkin_freq, init->ref_diff_en ? "differential" : "single-ended",
	       init->ref_div2_en ? "enabled" : "disabled",
	       init->ref_doubler_en ? "enabled" : "disabled");
	printf("  PFD: %" PRIu64 "/%" PRIu64 " Hz; R counter: %" PRIu32
	       "; RF divider: %" PRIu32 "\n",
	       parameters->pfd_num, parameters->pfd_den, parameters->r_counter,
	       parameters->rf_divider);
	printf("  Channel spacing: %" PRIu32 " Hz\n", channel_spacing_hz);
	printf("  N: INT=%" PRIu32 ", FRAC1=%" PRIu32
	       ", FRAC2=%" PRIu32 ", MOD2=%" PRIu32 "\n",
	       parameters->integer, parameters->frac1, parameters->frac2,
	       parameters->mod2);
	printf("  Charge pump: requested %" PRIu32 " uA; Register 4 code %" PRIu8 "\n",
	       init->cp_ua, parameters->cp_code);
	printf("  Negative bleed: %s; code %" PRIu8 "; gated %s\n",
	       parameters->negative_bleed_enabled ? "enabled" : "disabled",
	       parameters->negative_bleed_code,
	       init->cp_gated_bleed_en ? "enabled" : "disabled");
	printf("  RFOUTA power: %d dBm; mute-till-lock %s\n",
	       output_power_dbm, init->mute_till_lock_en ? "enabled" : "disabled");
	printf("  MUXOUT: %s; SPI: /dev/spidev%" PRIu32 ".%" PRIu8
	       " at %" PRIu32 " Hz\n",
	       muxout_name(init->mux_out_sel), spi->device_id, spi->chip_select,
	       spi->max_speed_hz);
	if (verbose)
		print_initialization_register_report(parameters);
}

static void print_usage(const char *program)
{
	printf(
		"Usage: %s [OPTIONS] [RFOUTA_HZ]\n\n"
		"Program RFOUTA through immutable ADI no-OS setup plus a userspace\n"
		"register-compatibility layer. The R12-to-R0 words written to SPI match\n"
		"the Python implementation for equivalent options.\n\n"
		"  -f, --rf-output-hz HZ        RFOUTA frequency (default: %" PRIu64 ")\n"
		"  -r, --reference-hz HZ        REFIN frequency (default: %" PRIu32 ")\n"
		"  -k, --channel-spacing-hz HZ Python-compatible spacing (default: %" PRIu32 ")\n"
		"  -c, --charge-pump-current-ua UA\n"
		"                              315 to 5040; rounded to a CP code\n"
		"  -p, --rf-output-power DBM    RFOUTA: -4, -1, 2, or 5\n"
		"  -s, --spi-speed-hz HZ        SPI speed\n"
		"      --reference-mode MODE    single-ended or differential\n"
		"      --ref-div2 | --no-ref-div2 | --reference-doubler\n"
		"      --negative-bleed | --no-negative-bleed | --gated-bleed\n"
		"      --mute-till-lock\n"
		"      --muxout MODE            digital, analog, r-divider, n-divider,\n"
		"                              high, low, or three-state\n"
		"      --spi-device N | --spi-chip-select N\n"
		"      --dry-run                Calculate and print; do not open SPI\n"
		"  -v, --verbose                Print corrected transmitted words\n"
		"  -h, --help                   Show this help and exit\n",
		program, (uint64_t)DEFAULT_RFOUTA_HZ,
		(uint32_t)DEFAULT_REFERENCE_HZ,
		(uint32_t)DEFAULT_CHANNEL_SPACING_HZ);
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
		.platform_ops = &deferred_linux_spi_ops,
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
		{"channel-spacing-hz", required_argument, NULL, 'k'},
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
		{"dry-run", no_argument, NULL, OPT_DRY_RUN},
		{0, 0, 0, 0},
	};
	struct adf5355_dev *adf5355 = NULL;
	struct python_compatible_parameters parameters;
	bool frequency_set = false;
	bool reference_div2_explicit = false;
	bool verbose = false;
	bool dry_run = false;
	uint32_t channel_spacing_hz = DEFAULT_CHANNEL_SPACING_HZ;
	int output_power_dbm = DEFAULT_OUTPUT_POWER_DBM;
	int option;
	int32_t ret;
	uint64_t parsed_u64;
	uint32_t parsed_u32;

	opterr = 0;
	while ((option = getopt_long(argc, argv, "f:r:k:c:p:s:vh",
				    long_options, NULL)) != -1) {
		switch (option) {
		case 'f':
			if (frequency_set || parse_u64(optarg, UINT64_MAX, &init.freq_req)) {
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
		case 'k':
			if (parse_u32(optarg, &channel_spacing_hz)) {
				fprintf(stderr, "Invalid channel spacing: %s\n", optarg);
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
			if (parse_output_power(optarg, &init.outa_power, &output_power_dbm)) {
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
				fprintf(stderr, "Reference mode must be single-ended or differential\n");
				return EXIT_FAILURE;
			}
			break;
		case OPT_REF_DIV2:
			init.ref_div2_en = true;
			reference_div2_explicit = true;
			break;
		case OPT_NO_REF_DIV2:
			init.ref_div2_en = false;
			reference_div2_explicit = true;
			break;
		case OPT_REFERENCE_DOUBLER: init.ref_doubler_en = true; break;
		case OPT_NEGATIVE_BLEED: init.cp_neg_bleed_en = true; break;
		case OPT_NO_NEGATIVE_BLEED: init.cp_neg_bleed_en = false; break;
		case OPT_GATED_BLEED: init.cp_gated_bleed_en = true; break;
		case OPT_MUTE_TILL_LOCK: init.mute_till_lock_en = true; break;
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
		case OPT_DRY_RUN:
			dry_run = true;
			verbose = true;
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
	     (frequency_set || parse_u64(argv[optind], UINT64_MAX, &parsed_u64)))) {
		fprintf(stderr, "Specify RFOUTA_HZ once, as an option or positionally.\n");
		return EXIT_FAILURE;
	}
	if (optind < argc)
		init.freq_req = parsed_u64;

	if (!reference_div2_explicit)
		init.ref_div2_en = init.clkin_freq >= 20000000U;

	if (init.cp_gated_bleed_en && !init.cp_neg_bleed_en) {
		fprintf(stderr, "--gated-bleed requires --negative-bleed\n");
		return EXIT_FAILURE;
	}
	if (calculate_python_compatible_parameters(&init, channel_spacing_hz,
						   &parameters)) {
		fprintf(stderr, "Cannot calculate a Python-compatible ADF5355 register map\n");
		return EXIT_FAILURE;
	}

	if (dry_run) {
		print_configuration(&init, &spi_init, &parameters, output_power_dbm,
				    channel_spacing_hz, true);
		return EXIT_SUCCESS;
	}

	ret = adf5355_init(&adf5355, &init);
	if (ret) {
		fprintf(stderr, "adf5355_init failed: %" PRId32 "\n", ret);
		return EXIT_FAILURE;
	}

	/* Switch from the write-suppressing setup wrapper to the real SPI backend. */
	adf5355->spi_desc->platform_ops = &linux_spi_ops;
	if (adf5355->spi_desc->bus)
		adf5355->spi_desc->bus->platform_ops = &linux_spi_ops;

	ret = program_compatible_register_map(adf5355->spi_desc, &parameters);
	if (ret) {
		fprintf(stderr, "Corrected register programming failed: %" PRId32 "\n", ret);
		adf5355_remove(adf5355);
		return EXIT_FAILURE;
	}

	print_configuration(&init, &spi_init, &parameters, output_power_dbm,
			    channel_spacing_hz, verbose);

	ret = adf5355_remove(adf5355);
	if (ret) {
		fprintf(stderr, "adf5355_remove failed: %" PRId32 "\n", ret);
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}


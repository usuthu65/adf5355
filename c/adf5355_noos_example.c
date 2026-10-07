/*
 * Minimal Linux-userspace ADF5355 program using Analog Devices' no-OS driver.
 *
 * Usage:
 *     sudo ./adf5355_noos_example [RFOUTA_HZ]
 *
 * It uses SPI0 CE0 (/dev/spidev0.0) for the ADF5355 LE connection.
 * The ADF5355 CE pin must already be held high by hardware.
 */

#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "no_os_spi.h"
#include "linux_spi.h"
#include "adf5355.h"

#define DEFAULT_RFOUTA_HZ 2100000000ULL
#define REFERENCE_HZ      125000000U
#define SPI_MAX_HZ        10000000U

static int parse_frequency(const char *text, uint64_t *frequency_hz)
{
	char *end;
	unsigned long long value;

	errno = 0;
	value = strtoull(text, &end, 0);
	if (errno || *text == '\0' || *end != '\0' || value == 0)
		return -1;

	*frequency_hz = value;
	return 0;
}

int main(int argc, char **argv)
{
	struct no_os_spi_init_param spi_init = {
		.device_id = 0,
		.max_speed_hz = SPI_MAX_HZ,
		.chip_select = 0,
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
		.clkin_freq = REFERENCE_HZ,
		.cp_ua = 3125,
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
	struct adf5355_dev *adf5355 = NULL;
	uint64_t requested_hz = DEFAULT_RFOUTA_HZ;
	int32_t ret;

	if (argc > 2 ||
	    (argc == 2 && parse_frequency(argv[1], &requested_hz))) {
		fprintf(stderr, "Usage: %s [RFOUTA_HZ]\n", argv[0]);
		return EXIT_FAILURE;
	}

	init.freq_req = requested_hz;
	ret = adf5355_init(&adf5355, &init);
	if (ret) {
		fprintf(stderr, "adf5355_init failed: %" PRId32 "\n", ret);
		return EXIT_FAILURE;
	}

	printf("RFOUTA requested: %" PRIu64 " Hz\n", requested_hz);
	printf("PFD: %" PRIu32 " Hz, INT=%" PRIu32
	       ", FRAC1=%" PRIu32 ", FRAC2=%" PRIu32
	       ", MOD2=%" PRIu32 "\n",
	       adf5355->fpfd, adf5355->integer, adf5355->fract1,
	       adf5355->fract2, adf5355->mod2);

	ret = adf5355_remove(adf5355);
	if (ret) {
		fprintf(stderr, "adf5355_remove failed: %" PRId32 "\n", ret);
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}

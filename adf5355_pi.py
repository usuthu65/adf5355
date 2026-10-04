#!/usr/bin/env python3
"""
adf5355_pi.py

ADF5355 Raspberry Pi SPI0 driver.

SPI0:

    Raspberry Pi                         ADF5355
    ------------------------------------------------
    GPIO11 / physical pin 23 / SPI0_SCLK  CLK
    GPIO10 / physical pin 19 / SPI0_MOSI  DATA
    GPIO8  / physical pin 24 / SPI0_CE0   LE
    GND / physical pin 6 or 9              GND

MUXOUT:

    Raspberry Pi                         ADF5355
    ------------------------------------------------
    GPIO25 / physical pin 22              MUXOUT, pin 30
    GND                                   GND

PDBRF:

    ADF5355 PDBRF, pin 26 -> ADF5355 DVDD, approximately 3.3 V

PDBRF is an active-low hardware power-down input for RFOUTA+ and
RFOUTA-. It must not be left floating.

RFOUTB is disabled by default and can be enabled explicitly.

Corrected fixed registers:

    Register 7  = 0x120000E7
    Register 9  = calculated from fPFD for calibration timing
    Register 10 = calculated from fPFD
    Register 12 = 0x0001041C

Additional configured fields:

    Register 4 DB13:DB10 = 0b1001
        Charge-pump-current code 9

    Register 4 DB7 = 1
        Positive phase-detector polarity

    Register 6 negative bleed is disabled for integer-N operation and
    calculated automatically for fractional-N operation.

Examples:

    python3 adf5355_pi.py --show-wiring

    python3 adf5355_pi.py --verify

    sudo python3 adf5355_pi.py \
        --rf-output-hz 1800000000 \
        --reference-hz 125000000 \
        --channel-spacing-hz 200000 \
        --verbose

    sudo python3 adf5355_pi.py \
        --rf-output-hz 1800000000 \
        --reference-hz 125000000 \
        --set-rf-output-power 5
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from fractions import Fraction
from math import gcd
from typing import Iterable, Optional

try:
    import spidev
except ImportError:
    spidev = None

try:
    import RPi.GPIO as GPIO
except ImportError:
    GPIO = None


# ============================================================================
# Defaults and limits
# ============================================================================

DEFAULT_REFERENCE_HZ = 125_000_000
DEFAULT_REFERENCE_MODE = "single-ended"
DEFAULT_CHANNEL_SPACING_HZ = 200_000
DEFAULT_RF_OUTPUT_POWER_DBM = -4
DEFAULT_MUXOUT_LOCK_DETECT = "digital"
DEFAULT_MUTE_TILL_LOCK = False

REFERENCE_MODE_SINGLE_ENDED = "single-ended"
REFERENCE_MODE_DIFFERENTIAL = "differential"

REFERENCE_MODES = (
    REFERENCE_MODE_SINGLE_ENDED,
    REFERENCE_MODE_DIFFERENTIAL,
)

MUXOUT_ANALOG_LOCK_DETECT = "analog"
MUXOUT_DIGITAL_LOCK_DETECT = "digital"

MUXOUT_LOCK_DETECT_MODES = (
    MUXOUT_ANALOG_LOCK_DETECT,
    MUXOUT_DIGITAL_LOCK_DETECT,
)

MUXOUT_GPIO_BCM = 25
MUXOUT_GPIO_PHYSICAL_PIN = 22

MUXOUT_ANALOG_LOCK_DETECT_CODE = 5
MUXOUT_DIGITAL_LOCK_DETECT_CODE = 6

MIN_RF_OUTPUT_HZ = 54_000_000
MAX_RF_OUTPUT_HZ = 6_800_000_000

MIN_REFERENCE_HZ = 10_000_000
MAX_REFERENCE_HZ_SINGLE_ENDED = 250_000_000
MAX_REFERENCE_HZ_DIFFERENTIAL = 600_000_000

MAX_PFD_HZ = 125_000_000
HIGH_PFD_THRESHOLD_HZ = 75_000_000

MIN_R_COUNTER = 1
MAX_R_COUNTER = 1023

MIN_INT_4_5_PRESCALER = 23
MAX_INT_4_5_PRESCALER = 32_767

MOD1 = 2**24

MIN_FRAC1 = 0
MAX_FRAC1 = MOD1 - 1

MIN_MOD2 = 2
MAX_MOD2 = 16_383

MIN_FRAC2 = 0
MAX_FRAC2 = MAX_MOD2 - 1

MIN_SPI_SPEED_HZ = 1
MAX_SPI_SPEED_HZ = 19_999_999

ADC_TARGET_HZ = 100_000
ADC_CLK_DIV_MIN = 1
ADC_CLK_DIV_MAX = 255

REQUIRED_ADC_CYCLES = 16
TIMING_MARGIN_NS = 10_000

# Correct fixed register values.
REGISTER_7_VALUE = 0x120000E7

# Register 9 timing fields. These must be calculated from fPFD; a fixed
# Register 9 value is not valid when the reference configuration changes.
VCO_BAND_DIVIDER_MAX = 255
TIMEOUT_MAX = 1023
ALC_WAIT = 30
SYNTHESIZER_LOCK_TIMEOUT = 12
MIN_SYNTHESIZER_LOCK_SETTLING_NS = 20_000
MIN_ALC_SETTLING_NS = 50_000
NS_PER_SECOND = 1_000_000_000

# Figure 53 Register 12 values.
PHASE_RESYNC_CLOCK_DIVIDER = 1
PHASE_RESYNC_TIMEOUT = 0x041

# Requested Register 4 settings.
DEFAULT_CHARGE_PUMP_CURRENT_CODE = 0b1001
PHASE_DETECTOR_POLARITY_POSITIVE = True

DIFFERENTIAL_REFERENCE_WARNING = """
WARNING: Differential operation selected.

The physical reference source must be connected to both REFINA and
REFINB. Selecting the command-line option only configures Register 4
DB9; it does not convert a single-ended signal into a differential one.

Differential mode permits reference frequencies up to 600 MHz.
Single-ended mode is limited to 250 MHz.
""".strip()


MUXOUT_WARNING = """
WARNING: MUXOUT monitoring is enabled.

Connect ADF5355 MUXOUT, pin 30, to Raspberry Pi GPIO25,
physical pin 22. Connect the ADF5355 ground to Raspberry Pi ground.

MUXOUT is configured for 3.3 V logic. Do not apply 5 V to GPIO25.
""".strip() + "\n"


# ============================================================================
# Register addresses
# ============================================================================

REG_R0 = 0
REG_R1 = 1
REG_R2 = 2
REG_R3 = 3
REG_R4 = 4
REG_R5 = 5
REG_R6 = 6
REG_R7 = 7
REG_R8 = 8
REG_R9 = 9
REG_R10 = 10
REG_R11 = 11
REG_R12 = 12


# ============================================================================
# Register fields
# ============================================================================

# Register 0.
R0_INT_SHIFT = 4
R0_AUTOCAL_SHIFT = 21

# Register 1.
R1_FRAC1_SHIFT = 4

# Register 2.
R2_MOD2_SHIFT = 4
R2_FRAC2_SHIFT = 18

# Register 4.
R4_COUNTER_RESET_MASK = 1 << 4
R4_PHASE_DETECTOR_POLARITY_MASK = 1 << 7
R4_MUXOUT_SHIFT = 27
R4_MUXOUT_MASK = 0x7 << R4_MUXOUT_SHIFT
R4_MUXOUT_LOGIC_SHIFT = 8
R4_REFERENCE_MODE_SHIFT = 9
R4_CHARGE_PUMP_CURRENT_SHIFT = 10
R4_CHARGE_PUMP_CURRENT_MASK = 0xF << R4_CHARGE_PUMP_CURRENT_SHIFT
R4_R_COUNTER_SHIFT = 15
R4_RDIV2_SHIFT = 25

# Register 6.
R6_RF_DIVIDER_SHIFT = 21
R6_NEGATIVE_BLEED_ENABLE_MASK = 1 << 29
R6_FEEDBACK_FUNDAMENTAL_MASK = 1 << 24
R6_MUTE_TILL_LOCK_MASK = 1 << 11
R6_RFOUTB_ENABLE_MASK = 1 << 10
R6_RFOUTA_ENABLE_MASK = 1 << 6
R6_RFOUTA_POWER_SHIFT = 4
R6_CP_BLEED_CURRENT_SHIFT = 13
R6_CP_BLEED_CURRENT_MASK = 0xFF << R6_CP_BLEED_CURRENT_SHIFT

# Register 10.
R10_ADC_CONVERSION_ENABLE_MASK = 1 << 4
R10_ADC_ENABLE_MASK = 1 << 5
R10_ADC_CLK_DIV_SHIFT = 6
R10_ADC_CLK_DIV_MASK = 0xFF << R10_ADC_CLK_DIV_SHIFT

# Register 12.
R12_PHASE_RESYNC_CLOCK_DIV_SHIFT = 16
R12_TIMEOUT_SHIFT = 4


# ============================================================================
# Encodings
# ============================================================================

RF_DIVIDER_TO_CODE = {
    1: 0,
    2: 1,
    4: 2,
    8: 3,
    16: 4,
    32: 5,
    64: 6,
}

RF_OUTPUT_POWER_TO_CODE = {
    -4: 0b00,
    -1: 0b01,
    2: 0b10,
    5: 0b11,
}


# ============================================================================
# Utility functions
# ============================================================================

def ceil_fraction(value: Fraction) -> int:
    """Return ceil(value) for a Fraction."""
    if value.denominator == 1:
        return value.numerator

    return (
        value.numerator + value.denominator - 1
    ) // value.denominator


def fraction_to_float(value: Fraction) -> float:
    """Convert a Fraction to float."""
    return value.numerator / value.denominator


def rfouta_pfd_multiplier(
    parameters: SynthesizerParameters,
) -> Fraction:
    """Return the exact multiplier from PFD to RFOUTA."""
    return Fraction(parameters.rf_out_hz, 1) / parameters.pfd_hz


def print_n_divider_configuration(
    parameters: SynthesizerParameters,
) -> None:
    """Print the selected N-divider values for verbose operation."""
    calculation_mode = (
        "integer-N"
        if parameters.frac1 == 0 and parameters.frac2 == 0
        else "fractional-N"
    )
    print(f"N-divider calculation mode: {calculation_mode}")
    print(
        f"N: {parameters.int_value}, "
        f"FRAC1: {parameters.frac1}, "
        f"FRAC2: {parameters.frac2}, "
        f"MOD2: {parameters.mod2}"
    )
    reference_to_pfd_divider = parameters.reference_divider
    reference_to_pfd_description = str(parameters.reference_divider)
    if parameters.reference_divide_by_2:
        reference_to_pfd_divider *= 2
        reference_to_pfd_description += " × 2"
    print(f"RF divider: {parameters.rf_divider}")
    print(
        "Reference-to-PFD divider: "
        f"{reference_to_pfd_description} = "
        f"{reference_to_pfd_divider}"
    )
    multiplier = rfouta_pfd_multiplier(parameters)
    if calculation_mode == "integer-N":
        print(
            "RFOUTA/PFD multiplier: N / RF divider = "
            f"{parameters.int_value} / {parameters.rf_divider} = "
            f"{fraction_to_float(multiplier):.9f}"
        )
    else:
        print(
            "RFOUTA/PFD multiplier: "
            "(N + (FRAC1 + FRAC2/MOD2) / MOD1) / RF divider = "
            f"({parameters.int_value} + "
            f"({parameters.frac1} + {parameters.frac2}/"
            f"{parameters.mod2}) / {parameters.mod1}) / "
            f"{parameters.rf_divider} = "
            f"{fraction_to_float(multiplier):.9f}"
        )
    if parameters.negative_bleed_enabled:
        print(
            "Negative bleed current: enabled; "
            f"code {parameters.negative_bleed_current_code}; "
            f"{float(parameters.negative_bleed_current_ma):.6f} mA"
        )


def validate_reference_mode(reference_mode: str) -> None:
    if reference_mode not in REFERENCE_MODES:
        raise ValueError(
            f"reference mode must be one of {REFERENCE_MODES}"
        )


def validate_muxout_lock_detect(muxout_lock_detect: str) -> None:
    if muxout_lock_detect not in MUXOUT_LOCK_DETECT_MODES:
        raise ValueError(
            f"MUXOUT selection must be one of "
            f"{MUXOUT_LOCK_DETECT_MODES}"
        )


def maximum_reference_hz(reference_mode: str) -> int:
    validate_reference_mode(reference_mode)

    if reference_mode == REFERENCE_MODE_DIFFERENTIAL:
        return MAX_REFERENCE_HZ_DIFFERENTIAL

    return MAX_REFERENCE_HZ_SINGLE_ENDED


def validate_rf_output_power(output_power_dbm: int) -> None:
    if output_power_dbm not in RF_OUTPUT_POWER_TO_CODE:
        raise ValueError(
            "RF output power must be one of -4, -1, +2, or +5 dBm"
        )


def validate_charge_pump_current_code(code: int) -> None:
    """Validate the ADF5355's four-bit charge-pump current setting."""
    if not 0 <= code <= 0xF:
        raise ValueError(
            "Charge-pump current code must be an integer from 0 to 15"
        )


def muxout_code(muxout_lock_detect: str) -> int:
    validate_muxout_lock_detect(muxout_lock_detect)

    if muxout_lock_detect == MUXOUT_ANALOG_LOCK_DETECT:
        return MUXOUT_ANALOG_LOCK_DETECT_CODE

    return MUXOUT_DIGITAL_LOCK_DETECT_CODE


def print_hardware_wiring() -> None:
    """Print Raspberry Pi-to-ADF5355 wiring information."""
    print(
        """
ADF5355 / Raspberry Pi hardware wiring
======================================

SPI0:

    GPIO11 / physical pin 23 -> ADF5355 CLK
    GPIO10 / physical pin 19 -> ADF5355 DATA
    GPIO8  / physical pin 24 -> ADF5355 LE
    Raspberry Pi GND          -> ADF5355 GND

MUXOUT:

    GPIO25 / physical pin 22 <- ADF5355 MUXOUT, pin 30
    Raspberry Pi GND          -> ADF5355 GND

PDBRF:

    ADF5355 PDBRF, pin 26 -> ADF5355 DVDD, approximately 3.3 V

    PDBRF must not be left floating.

Reference input:

    Single-ended:  source -> REFINA
    Differential:  source+ -> REFINA
                   source- -> REFINB

RFOUTB is disabled by default. Use --enable-rf-output-b to enable it.
"""
    )


# ============================================================================
# ADC clock
# ============================================================================

@dataclass(frozen=True)
class ADCClockConfiguration:
    pfd_hz: Fraction
    adc_clk_div: int
    adc_clock_hz: Fraction
    required_interval_ns: int
    enforced_interval_ns: int

    @property
    def adc_clock_khz(self) -> float:
        return fraction_to_float(self.adc_clock_hz) / 1_000.0


def calculate_adc_clock(
    pfd_hz: Fraction | int,
    adc_clk_div: Optional[int] = None,
) -> ADCClockConfiguration:
    """Calculate ADC_CLK_DIV and timing."""
    pfd_hz = Fraction(pfd_hz)

    if pfd_hz <= 0:
        raise ValueError("PFD frequency must be positive")

    if adc_clk_div is None:
        requested_divider = ceil_fraction(
            (
                (pfd_hz / ADC_TARGET_HZ)
                - 2
            ) / 4
        )

        adc_clk_div = min(
            ADC_CLK_DIV_MAX,
            max(ADC_CLK_DIV_MIN, requested_divider),
        )

    if not ADC_CLK_DIV_MIN <= adc_clk_div <= ADC_CLK_DIV_MAX:
        raise ValueError("ADC_CLK_DIV is outside the valid range")

    adc_clock_hz = pfd_hz / (4 * adc_clk_div + 2)

    required_interval_ns = (
        ceil_fraction(
            Fraction(REQUIRED_ADC_CYCLES * 1_000_000_000, 1)
            / adc_clock_hz
        )
        + 1
    )

    return ADCClockConfiguration(
        pfd_hz=pfd_hz,
        adc_clk_div=adc_clk_div,
        adc_clock_hz=adc_clock_hz,
        required_interval_ns=required_interval_ns,
        enforced_interval_ns=required_interval_ns + TIMING_MARGIN_NS,
    )


# ============================================================================
# Synthesizer calculation
# ============================================================================

@dataclass(frozen=True)
class SynthesizerParameters:
    rf_out_hz: int
    reference_hz: int
    reference_mode: str
    channel_spacing_hz: int
    muxout_lock_detect: str
    mute_till_lock: bool
    charge_pump_current_code: int

    rf_divider: int
    vco_hz: int
    pfd_hz: Fraction

    reference_divider: int
    reference_divide_by_2: bool

    int_value: int
    frac1: int
    mod1: int
    frac2: int
    mod2: int

    negative_bleed_enabled: bool
    negative_bleed_current_code: int
    negative_bleed_current_ma: Fraction

    adc_clock: ADCClockConfiguration


@dataclass(frozen=True)
class ProgrammingStep:
    register: int
    value: int
    description: str
    adc_clock_for_following_r0: Optional[ADCClockConfiguration] = None
    delay_after_ns: int = 0


def select_rf_divider(rf_out_hz: int) -> int:
    for rf_divider in RF_DIVIDER_TO_CODE:
        if 3_400_000_000 <= rf_out_hz * rf_divider <= 6_800_000_000:
            return rf_divider

    raise ValueError("RFOUTA frequency cannot be generated")


def choose_reference_configuration(
    reference_hz: int,
) -> tuple[int, bool, Fraction]:
    reference_divide_by_2 = reference_hz >= 20_000_000

    base_pfd_hz = Fraction(
        reference_hz,
        2 if reference_divide_by_2 else 1,
    )

    reference_divider = max(
        1,
        ceil_fraction(base_pfd_hz / MAX_PFD_HZ),
    )

    if reference_divider > MAX_R_COUNTER:
        raise ValueError("Reference R counter exceeds its limit")

    pfd_hz = base_pfd_hz / reference_divider

    if not 0 < pfd_hz <= MAX_PFD_HZ:
        raise ValueError("Calculated PFD frequency is invalid")

    return reference_divider, reference_divide_by_2, pfd_hz


def charge_pump_current_ma(code: int) -> Fraction:
    """Return the nominal charge-pump current in mA for a Register 4 code."""
    validate_charge_pump_current_code(code)
    return Fraction((code + 1) * 5, 16)


def select_negative_bleed_current_code(
    feedback_counter: int,
) -> int:
    """Select the lowest 1/256 ICP bleed ratio above 4 / feedback_counter."""
    if feedback_counter <= 0:
        raise ValueError("Feedback counter must be positive")

    minimum_code = (4 * 256) // feedback_counter + 1
    maximum_code = (10 * 256 - 1) // feedback_counter

    if not 1 <= minimum_code <= min(0xFF, maximum_code):
        raise ValueError(
            "No valid negative bleed current is available for INT"
        )

    return minimum_code


def calculate_synthesizer_parameters(
    rf_out_hz: int,
    reference_hz: int = DEFAULT_REFERENCE_HZ,
    reference_mode: str = DEFAULT_REFERENCE_MODE,
    channel_spacing_hz: int = DEFAULT_CHANNEL_SPACING_HZ,
    muxout_lock_detect: str = DEFAULT_MUXOUT_LOCK_DETECT,
    mute_till_lock: bool = DEFAULT_MUTE_TILL_LOCK,
    charge_pump_current_code: int = DEFAULT_CHARGE_PUMP_CURRENT_CODE,
) -> SynthesizerParameters:
    validate_reference_mode(reference_mode)
    validate_muxout_lock_detect(muxout_lock_detect)
    validate_charge_pump_current_code(charge_pump_current_code)

    if not MIN_RF_OUTPUT_HZ <= rf_out_hz <= MAX_RF_OUTPUT_HZ:
        raise ValueError("RFOUTA frequency is outside the allowed range")

    if not (
        MIN_REFERENCE_HZ
        <= reference_hz
        <= maximum_reference_hz(reference_mode)
    ):
        raise ValueError("Reference frequency is outside the allowed range")

    if channel_spacing_hz <= 0:
        raise ValueError("Channel spacing must be positive")

    reference_divider, reference_divide_by_2, pfd_hz = (
        choose_reference_configuration(reference_hz)
    )

    rf_divider = select_rf_divider(rf_out_hz)
    vco_hz = rf_out_hz * rf_divider

    n_ratio = Fraction(vco_hz, 1) / pfd_hz
    int_value = n_ratio.numerator // n_ratio.denominator
    fractional_part = n_ratio - int_value

    frac1 = (
        fractional_part.numerator * MOD1
    ) // fractional_part.denominator

    remainder = fractional_part * MOD1 - frac1

    if remainder == 0:
        mod2 = MIN_MOD2
        frac2 = 0
    else:
        mod2 = pfd_hz.numerator // gcd(
            pfd_hz.numerator,
            channel_spacing_hz * pfd_hz.denominator,
        )

        if not MIN_MOD2 <= mod2 <= MAX_MOD2:
            raise ValueError("Calculated MOD2 is invalid")

        frac2_fraction = remainder * mod2

        if frac2_fraction.denominator != 1:
            raise ValueError("Calculated FRAC2 is not an integer")

        frac2 = frac2_fraction.numerator

    if not MIN_INT_4_5_PRESCALER <= int_value <= MAX_INT_4_5_PRESCALER:
        raise ValueError("Calculated INT is invalid")

    if not MIN_FRAC1 <= frac1 <= MAX_FRAC1:
        raise ValueError("Calculated FRAC1 is invalid")

    if not MIN_FRAC2 <= frac2 < mod2:
        raise ValueError("Calculated FRAC2 is invalid")

    negative_bleed_enabled = frac1 != 0 or frac2 != 0
    if negative_bleed_enabled:
        negative_bleed_current_code = (
            select_negative_bleed_current_code(int_value)
        )
        negative_bleed_current_ma = (
            charge_pump_current_ma(charge_pump_current_code)
            * Fraction(negative_bleed_current_code, 256)
        )
    else:
        negative_bleed_current_code = 0
        negative_bleed_current_ma = Fraction(0, 1)

    return SynthesizerParameters(
        rf_out_hz=rf_out_hz,
        reference_hz=reference_hz,
        reference_mode=reference_mode,
        channel_spacing_hz=channel_spacing_hz,
        muxout_lock_detect=muxout_lock_detect,
        mute_till_lock=mute_till_lock,
        charge_pump_current_code=charge_pump_current_code,
        rf_divider=rf_divider,
        vco_hz=vco_hz,
        pfd_hz=pfd_hz,
        reference_divider=reference_divider,
        reference_divide_by_2=reference_divide_by_2,
        int_value=int_value,
        frac1=frac1,
        mod1=MOD1,
        frac2=frac2,
        mod2=mod2,
        negative_bleed_enabled=negative_bleed_enabled,
        negative_bleed_current_code=negative_bleed_current_code,
        negative_bleed_current_ma=negative_bleed_current_ma,
        adc_clock=calculate_adc_clock(pfd_hz),
    )


# ============================================================================
# Register construction
# ============================================================================

def make_register_0(
    parameters: SynthesizerParameters,
    autocal_enabled: bool = True,
) -> int:
    value = REG_R0 | (parameters.int_value << R0_INT_SHIFT)

    if autocal_enabled:
        value |= 1 << R0_AUTOCAL_SHIFT

    return value


def make_register_1(parameters: SynthesizerParameters) -> int:
    return REG_R1 | (parameters.frac1 << R1_FRAC1_SHIFT)


def make_register_2(parameters: SynthesizerParameters) -> int:
    return (
        REG_R2
        | (parameters.mod2 << R2_MOD2_SHIFT)
        | (parameters.frac2 << R2_FRAC2_SHIFT)
    )


def make_register_4(
    parameters: SynthesizerParameters,
    counter_reset: bool = False,
) -> int:
    value = REG_R4

    value |= (
        muxout_code(parameters.muxout_lock_detect)
        << R4_MUXOUT_SHIFT
    )

    value |= 1 << R4_MUXOUT_LOGIC_SHIFT

    if parameters.reference_mode == REFERENCE_MODE_DIFFERENTIAL:
        value |= 1 << R4_REFERENCE_MODE_SHIFT

    value |= (
        parameters.charge_pump_current_code
        << R4_CHARGE_PUMP_CURRENT_SHIFT
    )

    if PHASE_DETECTOR_POLARITY_POSITIVE:
        value |= R4_PHASE_DETECTOR_POLARITY_MASK

    value |= parameters.reference_divider << R4_R_COUNTER_SHIFT

    if parameters.reference_divide_by_2:
        value |= 1 << R4_RDIV2_SHIFT

    if counter_reset:
        value |= R4_COUNTER_RESET_MASK

    return value


def make_register_6(
    parameters: SynthesizerParameters,
    output_power_dbm: int,
    enable_rfout_a: bool,
    enable_rfout_b: bool = False,
) -> int:
    validate_rf_output_power(output_power_dbm)

    value = 0x14000000 | REG_R6
    value |= RF_DIVIDER_TO_CODE[parameters.rf_divider] << R6_RF_DIVIDER_SHIFT
    value |= R6_FEEDBACK_FUNDAMENTAL_MASK

    if parameters.negative_bleed_enabled:
        value |= R6_NEGATIVE_BLEED_ENABLE_MASK
        value |= (
            parameters.negative_bleed_current_code
            << R6_CP_BLEED_CURRENT_SHIFT
        )

    value |= (
        RF_OUTPUT_POWER_TO_CODE[output_power_dbm]
        << R6_RFOUTA_POWER_SHIFT
    )

    if parameters.mute_till_lock:
        value |= R6_MUTE_TILL_LOCK_MASK

    if enable_rfout_a:
        value |= R6_RFOUTA_ENABLE_MASK

    if enable_rfout_b:
        value |= R6_RFOUTB_ENABLE_MASK

    return value


def make_register_9(parameters: SynthesizerParameters) -> int:
    """Build Register 9 with datasheet-compliant calibration timing."""
    pfd_hz = parameters.pfd_hz

    vco_band_divider = ceil_fraction(pfd_hz / 2_400_000)

    # Register 9 requires at least 20 us for VTUNE/DAC settling. The
    # ALC wait requirement is strict: it must be greater than 50 us.
    minimum_timeout_for_synth_lock = ceil_fraction(
        pfd_hz
        * Fraction(
            MIN_SYNTHESIZER_LOCK_SETTLING_NS,
            NS_PER_SECOND * SYNTHESIZER_LOCK_TIMEOUT,
        )
    )
    alc_timeout_threshold = (
        pfd_hz
        * Fraction(MIN_ALC_SETTLING_NS, NS_PER_SECOND)
        / ALC_WAIT
    )
    minimum_timeout_for_alc = (
        alc_timeout_threshold.numerator
        // alc_timeout_threshold.denominator
        + 1
    )
    timeout = max(
        minimum_timeout_for_synth_lock,
        minimum_timeout_for_alc,
    )

    if not 1 <= vco_band_divider <= VCO_BAND_DIVIDER_MAX:
        raise ValueError("VCO band divider is outside the valid range")

    if not 1 <= timeout <= TIMEOUT_MAX:
        raise ValueError("Register 9 timeout is outside the valid range")

    return (
        (vco_band_divider << 24)
        | (timeout << 14)
        | (ALC_WAIT << 9)
        | (SYNTHESIZER_LOCK_TIMEOUT << 4)
        | REG_R9
    )


def make_register_10(parameters: SynthesizerParameters) -> int:
    """
    Create Register 10.

    ADC enable and ADC conversion enable are both set.
    """
    value = 0x00C0000A

    value |= (
        R10_ADC_ENABLE_MASK
        | R10_ADC_CONVERSION_ENABLE_MASK
    )

    value |= parameters.adc_clock.adc_clk_div << R10_ADC_CLK_DIV_SHIFT

    return value


def make_register_12() -> int:
    """Return Register 12 = 0x0001041C."""
    return (
        (1 << 16)
        | (PHASE_RESYNC_TIMEOUT << 4)
        | REG_R12
    )


def make_register_map(
    parameters: SynthesizerParameters,
    output_power_dbm: int,
    enable_rfout_a: bool,
    enable_rfout_b: bool = False,
    autocal_enabled: bool = True,
    counter_reset: bool = False,
) -> dict[int, int]:
    """Create the complete ADF5355 register map."""
    return {
        REG_R0: make_register_0(parameters, autocal_enabled),
        REG_R1: make_register_1(parameters),
        REG_R2: make_register_2(parameters),
        REG_R3: 0x00000003,
        REG_R4: make_register_4(parameters, counter_reset),
        REG_R5: 0x00800025,
        REG_R6: make_register_6(
            parameters,
            output_power_dbm,
            enable_rfout_a,
            enable_rfout_b,
        ),
        REG_R7: REGISTER_7_VALUE,
        REG_R8: 0x102D0428,
        REG_R9: make_register_9(parameters),
        REG_R10: make_register_10(parameters),
        REG_R11: 0x0061300B,
        REG_R12: make_register_12(),
    }


# ============================================================================
# Initialization and update sequences
# ============================================================================

def make_initialization_sequence(
    parameters: SynthesizerParameters,
    output_power_dbm: int,
    enable_rfout_a: bool,
    enable_rfout_b: bool = False,
) -> list[ProgrammingStep]:
    """Create the complete Register Initialization Sequence."""
    registers = make_register_map(
        parameters,
        output_power_dbm,
        enable_rfout_a,
        enable_rfout_b,
    )

    order = [
        REG_R12,
        REG_R11,
        REG_R10,
        REG_R9,
        REG_R8,
        REG_R7,
        REG_R6,
        REG_R5,
        REG_R4,
        REG_R3,
        REG_R2,
        REG_R1,
        REG_R0,
    ]

    return [
        ProgrammingStep(
            register=register,
            value=registers[register],
            description="initialization",
            adc_clock_for_following_r0=(
                parameters.adc_clock
                if register == REG_R1
                else None
            ),
        )
        for register in order
    ]


def make_frequency_update_sequence(
    parameters: SynthesizerParameters,
    output_power_dbm: int,
    enable_rfout_a: bool,
    enable_rfout_b: bool = False,
) -> list[ProgrammingStep]:
    """Select the update sequence using calculated fPFD."""
    reset = make_register_map(
        parameters,
        output_power_dbm,
        enable_rfout_a,
        enable_rfout_b,
        autocal_enabled=False,
        counter_reset=True,
    )

    normal = make_register_map(
        parameters,
        output_power_dbm,
        enable_rfout_a,
        enable_rfout_b,
        autocal_enabled=True,
        counter_reset=False,
    )

    if parameters.pfd_hz <= HIGH_PFD_THRESHOLD_HZ:
        return [
            ProgrammingStep(REG_R10, reset[REG_R10], "update R10"),
            ProgrammingStep(REG_R4, reset[REG_R4], "update R4, reset enabled"),
            ProgrammingStep(REG_R2, reset[REG_R2], "update R2"),
            ProgrammingStep(REG_R1, reset[REG_R1], "update R1"),
            ProgrammingStep(REG_R0, reset[REG_R0], "AUTOCAL disabled"),
            ProgrammingStep(
                REG_R4,
                normal[REG_R4],
                "reset disabled",
                delay_after_ns=parameters.adc_clock.enforced_interval_ns,
            ),
            ProgrammingStep(REG_R0, normal[REG_R0], "AUTOCAL enabled"),
        ]

    half_adc_clock = calculate_adc_clock(parameters.pfd_hz / 2)

    return [
        ProgrammingStep(REG_R10, reset[REG_R10], "half-PFD R10"),
        ProgrammingStep(REG_R4, reset[REG_R4], "half-PFD R4"),
        ProgrammingStep(REG_R2, reset[REG_R2], "half-PFD R2"),
        ProgrammingStep(REG_R1, reset[REG_R1], "half-PFD R1"),
        ProgrammingStep(REG_R0, reset[REG_R0], "half-PFD AUTOCAL off"),
        ProgrammingStep(
            REG_R4,
            normal[REG_R4],
            "half-PFD reset disabled",
            delay_after_ns=half_adc_clock.enforced_interval_ns,
        ),
        ProgrammingStep(REG_R0, normal[REG_R0], "half-PFD AUTOCAL on"),
        ProgrammingStep(REG_R10, reset[REG_R10], "final-PFD R10"),
        ProgrammingStep(REG_R4, reset[REG_R4], "final-PFD R4"),
        ProgrammingStep(REG_R2, reset[REG_R2], "final-PFD R2"),
        ProgrammingStep(REG_R1, reset[REG_R1], "final-PFD R1"),
        ProgrammingStep(REG_R0, reset[REG_R0], "final-PFD AUTOCAL off"),
        ProgrammingStep(
            REG_R4,
            normal[REG_R4],
            "final-PFD reset disabled",
            delay_after_ns=parameters.adc_clock.enforced_interval_ns,
        ),
        ProgrammingStep(REG_R0, normal[REG_R0], "final-PFD AUTOCAL on"),
    ]


# ============================================================================
# MUXOUT handling
# ============================================================================

def read_muxout_gpio() -> int:
    """Read GPIO25."""
    if GPIO is None:
        raise RuntimeError("RPi.GPIO is not installed")

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(MUXOUT_GPIO_BCM, GPIO.IN, pull_up_down=GPIO.PUD_OFF)

    try:
        return int(GPIO.input(MUXOUT_GPIO_BCM))
    finally:
        GPIO.cleanup(MUXOUT_GPIO_BCM)


def wait_for_digital_lock(timeout_s: float) -> None:
    """Wait for active-high digital MUXOUT lock detection on GPIO25."""
    if GPIO is None:
        raise RuntimeError("--wait-for-lock requires RPi.GPIO")

    deadline_ns = time.monotonic_ns() + int(
        timeout_s * NS_PER_SECOND
    )

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(MUXOUT_GPIO_BCM, GPIO.IN, pull_up_down=GPIO.PUD_OFF)

    try:
        while not GPIO.input(MUXOUT_GPIO_BCM):
            remaining_ns = deadline_ns - time.monotonic_ns()

            if remaining_ns <= 0:
                raise TimeoutError(
                    "digital lock did not assert within "
                    f"{timeout_s * 1000.0:.3f} ms"
                )

            time.sleep(min(0.001, remaining_ns / NS_PER_SECOND))
    finally:
        GPIO.cleanup(MUXOUT_GPIO_BCM)


# ============================================================================
# SPI driver
# ============================================================================

class ADF5355:
    """ADF5355 driver restricted to Raspberry Pi SPI0 CE0."""

    SPI_BUS = 0
    SPI_DEVICE = 0
    SPI_MODE = 0

    def __init__(
        self,
        bus: int = SPI_BUS,
        device: int = SPI_DEVICE,
        max_speed_hz: int = 10_000_000,
        spi=None,
        verbose: bool = False,
    ) -> None:
        if bus != self.SPI_BUS:
            raise ValueError("This driver only allows SPI0")

        if device != self.SPI_DEVICE:
            raise ValueError("This driver only allows SPI0 CE0")

        if not MIN_SPI_SPEED_HZ <= max_speed_hz <= MAX_SPI_SPEED_HZ:
            raise ValueError("SPI speed is outside the allowed range")

        self.verbose = verbose

        if spi is None:
            if spidev is None:
                raise RuntimeError(
                    "spidev is not installed; install python3-spidev"
                )

            spi = spidev.SpiDev()
            spi.open(self.SPI_BUS, self.SPI_DEVICE)
            self._owns_spi = True
        else:
            self._owns_spi = False

        self.spi = spi
        self.spi.mode = self.SPI_MODE
        self.spi.max_speed_hz = max_speed_hz
        self.spi.lsbfirst = False
        self.spi.cshigh = False

    @staticmethod
    def format_register(value: int) -> str:
        return f"0x{value:08X}"

    def _write_register(self, value: int) -> tuple[int, int]:
        """Write one 32-bit register."""
        data = [
            (value >> 24) & 0xFF,
            (value >> 16) & 0xFF,
            (value >> 8) & 0xFF,
            value & 0xFF,
        ]

        start_ns = time.monotonic_ns()
        self.spi.xfer2(data)
        end_ns = time.monotonic_ns()

        return start_ns, end_ns

    @staticmethod
    def wait_until(deadline_ns: int) -> None:
        while True:
            remaining_ns = deadline_ns - time.monotonic_ns()

            if remaining_ns <= 0:
                return

            if remaining_ns > 100_000:
                time.sleep(
                    (remaining_ns - 50_000)
                    / 1_000_000_000
                )
            else:
                time.sleep(0)

    def write_sequence(
        self,
        steps: Iterable[ProgrammingStep],
        sequence_name: str,
    ) -> None:
        """Write a register sequence."""
        step_list = list(steps)
        sequence_start_ns = time.monotonic_ns()

        previous_end_ns: Optional[int] = None
        pending_r1_end_ns: Optional[int] = None
        pending_adc_clock: Optional[ADCClockConfiguration] = None

        if self.verbose:
            print(
                f"Beginning {sequence_name}: "
                f"{len(step_list)} register writes"
            )

        for index, step in enumerate(step_list, start=1):
            if (step.value & 0xF) != step.register:
                raise ValueError("Register address mismatch")

            if (
                step.register == REG_R0
                and pending_r1_end_ns is not None
                and pending_adc_clock is not None
            ):
                self.wait_until(
                    pending_r1_end_ns
                    + pending_adc_clock.enforced_interval_ns
                )

            start_ns, end_ns = self._write_register(step.value)

            if step.register == REG_R1:
                pending_r1_end_ns = end_ns
                pending_adc_clock = step.adc_clock_for_following_r0

            if step.delay_after_ns:
                self.wait_until(end_ns + step.delay_after_ns)

            if self.verbose:
                start_us = (start_ns - sequence_start_ns) / 1000.0
                end_us = (end_ns - sequence_start_ns) / 1000.0
                transfer_us = (end_ns - start_ns) / 1000.0

                if previous_end_ns is None:
                    gap_text = "n/a"
                else:
                    gap_text = (
                        f"{(start_ns - previous_end_ns) / 1000.0:.3f} us"
                    )

                print(
                    f"Step {index:02d}/{len(step_list):02d}: "
                    f"{step.description}; "
                    f"Register {step.register}; "
                    f"value={self.format_register(step.value)}; "
                    f"start=+{start_us:.3f} us; "
                    f"end=+{end_us:.3f} us; "
                    f"transfer={transfer_us:.3f} us; "
                    f"gap={gap_text}"
                )

            previous_end_ns = end_ns

            if step.register == REG_R0:
                pending_r1_end_ns = None
                pending_adc_clock = None

    def program_initial_frequency(
        self,
        parameters: SynthesizerParameters,
        output_power_dbm: int,
        enable_rfout_a: bool,
        enable_rfout_b: bool = False,
    ) -> None:
        self.write_sequence(
            make_initialization_sequence(
                parameters,
                output_power_dbm,
                enable_rfout_a,
                enable_rfout_b,
            ),
            "Register Initialization Sequence",
        )

    def close(self) -> None:
        if self._owns_spi:
            self.spi.close()

    def __enter__(self) -> "ADF5355":
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()


# ============================================================================
# Verification
# ============================================================================

class FakeSPI:
    """Minimal SPI replacement."""

    def __init__(self) -> None:
        self.transfers = []

    def xfer2(self, data) -> None:
        self.transfers.append(list(data))

    def close(self) -> None:
        pass


def run_verification() -> None:
    """Run software-only checks."""
    parameters = calculate_synthesizer_parameters(
        2_400_000_000,
        reference_hz=125_000_000,
    )

    registers = make_register_map(
        parameters,
        output_power_dbm=5,
        enable_rfout_a=True,
    )

    assert registers[REG_R7] == 0x120000E7
    assert registers[REG_R9] == 0x1B1A7CC9
    assert registers[REG_R10] == 0x00C0273A
    assert registers[REG_R12] == 0x0001041C
    assert not registers[REG_R6] & R6_RFOUTB_ENABLE_MASK

    rfout_b_register_6 = make_register_6(
        parameters,
        output_power_dbm=5,
        enable_rfout_a=True,
        enable_rfout_b=True,
    )
    assert rfout_b_register_6 & R6_RFOUTB_ENABLE_MASK

    integer_parameters = calculate_synthesizer_parameters(
        1_000_000_000,
        reference_hz=125_000_000,
    )
    integer_register_6 = make_register_6(
        integer_parameters,
        output_power_dbm=5,
        enable_rfout_a=True,
    )
    assert not integer_parameters.negative_bleed_enabled
    assert rfouta_pfd_multiplier(integer_parameters) == 16
    assert integer_parameters.negative_bleed_current_code == 0
    assert integer_parameters.negative_bleed_current_ma == 0
    assert not integer_register_6 & R6_NEGATIVE_BLEED_ENABLE_MASK
    assert not integer_register_6 & R6_CP_BLEED_CURRENT_MASK

    fractional_register_6 = registers[REG_R6]
    bleed_ratio = (
        parameters.negative_bleed_current_ma
        / charge_pump_current_ma(parameters.charge_pump_current_code)
    )
    assert parameters.negative_bleed_enabled
    assert rfouta_pfd_multiplier(parameters) == Fraction(192, 5)
    assert parameters.negative_bleed_current_code == 14
    assert fractional_register_6 & R6_NEGATIVE_BLEED_ENABLE_MASK
    assert (
        (fractional_register_6 & R6_CP_BLEED_CURRENT_MASK)
        >> R6_CP_BLEED_CURRENT_SHIFT
        == parameters.negative_bleed_current_code
    )
    assert Fraction(4, parameters.int_value) < bleed_ratio
    assert bleed_ratio < Fraction(10, parameters.int_value)

    for charge_pump_current_code in range(16):
        charge_pump_parameters = calculate_synthesizer_parameters(
            2_400_000_000,
            charge_pump_current_code=charge_pump_current_code,
        )
        register_4 = make_register_4(charge_pump_parameters)
        assert (
            register_4 & R4_CHARGE_PUMP_CURRENT_MASK
            == charge_pump_current_code
            << R4_CHARGE_PUMP_CURRENT_SHIFT
        )

    high_pfd_parameters = calculate_synthesizer_parameters(
        2_400_000_000,
        reference_hz=250_000_000,
        reference_mode=REFERENCE_MODE_DIFFERENTIAL,
    )

    assert high_pfd_parameters.pfd_hz == 125_000_000
    assert make_register_9(high_pfd_parameters) == 0x35347CC9

    fake_spi = FakeSPI()
    device = ADF5355(spi=fake_spi)

    device._write_register(REG_R6 | R6_RFOUTB_ENABLE_MASK)
    assert fake_spi.transfers[-1] == [0, 0, 4, 6]

    device.close()

    print("All ADF5355 checks passed.")


# ============================================================================
# Command-line helpers
# ============================================================================

def bounded_integer(
    value: str,
    minimum: int,
    maximum: int,
    name: str,
) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            f"{name} must be an integer"
        ) from exc

    if not minimum <= parsed <= maximum:
        raise argparse.ArgumentTypeError(
            f"{name} must be between {minimum} and {maximum}"
        )

    return parsed


def positive_integer(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            "value must be an integer"
        ) from exc

    if parsed <= 0:
        raise argparse.ArgumentTypeError(
            "value must be positive"
        )

    return parsed


def positive_float(value: str) -> float:
    try:
        parsed = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            "value must be numeric"
        ) from exc

    if not math.isfinite(parsed) or parsed <= 0:
        raise argparse.ArgumentTypeError(
            "value must be finite and positive"
        )

    return parsed


def rf_output_frequency_arg(value: str) -> int:
    return bounded_integer(
        value,
        MIN_RF_OUTPUT_HZ,
        MAX_RF_OUTPUT_HZ,
        "RF output frequency",
    )


def reference_frequency_arg(value: str) -> int:
    parsed = int(value)

    if parsed < MIN_REFERENCE_HZ:
        raise argparse.ArgumentTypeError(
            "reference frequency is too low"
        )

    return parsed


def channel_spacing_arg(value: str) -> int:
    return bounded_integer(
        value,
        1,
        2**63 - 1,
        "channel spacing",
    )


def spi_speed_arg(value: str) -> int:
    return bounded_integer(
        value,
        MIN_SPI_SPEED_HZ,
        MAX_SPI_SPEED_HZ,
        "SPI speed",
    )


# ============================================================================
# Main
# ============================================================================

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Program ADF5355 RFOUTA through Raspberry Pi SPI0",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    parser.add_argument(
        "--show-wiring",
        action="store_true",
        help="print wiring information and exit",
    )

    parser.add_argument(
        "--verify",
        action="store_true",
        help="run software-only checks",
    )

    parser.add_argument(
        "--rf-output-hz",
        type=rf_output_frequency_arg,
        help="RFOUTA frequency in Hz",
    )

    parser.add_argument(
        "--reference-hz",
        type=reference_frequency_arg,
        default=DEFAULT_REFERENCE_HZ,
        help="reference input frequency in Hz",
    )

    parser.add_argument(
        "--reference-mode",
        choices=REFERENCE_MODES,
        default=DEFAULT_REFERENCE_MODE,
        help="single-ended or differential reference input",
    )

    parser.add_argument(
        "--channel-spacing-hz",
        type=channel_spacing_arg,
        default=DEFAULT_CHANNEL_SPACING_HZ,
        help="channel spacing in Hz",
    )

    parser.add_argument(
        "--muxout-lock-detect",
        choices=MUXOUT_LOCK_DETECT_MODES,
        default=DEFAULT_MUXOUT_LOCK_DETECT,
        help="analog or digital MUXOUT lock detect",
    )

    parser.add_argument(
        "--wait-for-lock",
        action="store_true",
        help=(
            "wait for active-high digital MUXOUT lock detection on "
            "GPIO25 before exiting"
        ),
    )

    parser.add_argument(
        "--lock-timeout-ms",
        type=positive_float,
        default=20.0,
        help="maximum wait for digital lock in milliseconds",
    )

    parser.add_argument(
        "--check-muxout",
        action="store_true",
        help="read MUXOUT on GPIO25 after programming",
    )

    parser.add_argument(
        "--mute-till-lock",
        action="store_true",
        default=DEFAULT_MUTE_TILL_LOCK,
        help="enable Register 6 MTLD",
    )

    parser.add_argument(
        "--charge-pump-current-code",
        type=int,
        choices=range(16),
        default=DEFAULT_CHARGE_PUMP_CURRENT_CODE,
        metavar="CODE",
        help=(
            "Register 4 charge-pump-current code. Current mapping "
            "(mA): 0=0.3125, 1=0.6250, 2=0.9375, 3=1.2500, "
            "4=1.5625, 5=1.8750, 6=2.1875, 7=2.5000, "
            "8=2.8125, 9=3.1250, 10=3.4375, 11=3.7500, "
            "12=4.0625, 13=4.3750, 14=4.6875, 15=5.0000"
        ),
    )

    power_group = parser.add_mutually_exclusive_group()

    power_group.add_argument(
        "--set-rf-output-power",
        type=int,
        choices=[-4, -1, 2, 5],
        metavar="DBM",
        help="RFOUTA output power in dBm",
    )

    power_group.add_argument(
        "--disable-rf-output",
        action="store_true",
        help="disable RFOUTA",
    )

    parser.add_argument(
        "--enable-rf-output-b",
        action="store_true",
        help="enable RFOUTB (disabled by default)",
    )

    parser.add_argument(
        "--max-speed-hz",
        type=spi_speed_arg,
        default=10_000_000,
        help="SPI0 clock rate in Hz",
    )

    parser.add_argument(
        "--verbose",
        action="store_true",
        help="print programming steps and register values",
    )

    args = parser.parse_args()

    if args.show_wiring:
        print_hardware_wiring()
        return

    if args.verify:
        run_verification()
        return

    if args.reference_mode == REFERENCE_MODE_DIFFERENTIAL:
        print(
            DIFFERENTIAL_REFERENCE_WARNING,
            file=sys.stderr,
        )

    if args.check_muxout:
        print(
            MUXOUT_WARNING,
            file=sys.stderr,
        )

    if args.wait_for_lock:
        if args.muxout_lock_detect != MUXOUT_DIGITAL_LOCK_DETECT:
            parser.error(
                "--wait-for-lock requires digital MUXOUT"
            )

        print(
            MUXOUT_WARNING,
            file=sys.stderr,
        )

    if args.rf_output_hz is None:
        parser.error(
            "--rf-output-hz is required unless --verify is used"
        )

    output_power_dbm = (
        DEFAULT_RF_OUTPUT_POWER_DBM
        if args.set_rf_output_power is None
        else args.set_rf_output_power
    )

    try:
        parameters = calculate_synthesizer_parameters(
            args.rf_output_hz,
            args.reference_hz,
            args.reference_mode,
            args.channel_spacing_hz,
            args.muxout_lock_detect,
            args.mute_till_lock,
            args.charge_pump_current_code,
        )

        validate_rf_output_power(output_power_dbm)

    except (TypeError, ValueError) as exc:
        parser.error(str(exc))

    try:
        with ADF5355(
            max_speed_hz=args.max_speed_hz,
            verbose=args.verbose,
        ) as device:
            device.program_initial_frequency(
                parameters,
                output_power_dbm,
                not args.disable_rf_output,
                args.enable_rf_output_b,
            )

            if args.check_muxout:
                time.sleep(0.1)
                muxout_state = read_muxout_gpio()

            if args.wait_for_lock:
                wait_for_digital_lock(
                    args.lock_timeout_ms / 1000.0
                )

    except (TypeError, ValueError, RuntimeError) as exc:
        parser.error(str(exc))

    print("ADF5355 programmed through SPI0 CE0")
    print(
        f"RFOUTA: "
        f"{parameters.rf_out_hz / 1e6:.9f} MHz"
    )
    print(
        f"RFOUTB (VCO): "
        f"{parameters.vco_hz / 1e6:.9f} MHz"
    )
    print(
        f"Reference input: "
        f"{parameters.reference_hz / 1e6:.9f} MHz"
    )
    print(
        f"PFD: "
        f"{fraction_to_float(parameters.pfd_hz) / 1e6:.9f} MHz"
    )
    if args.verbose:
        print_n_divider_configuration(parameters)
    print(f"Reference mode: {parameters.reference_mode}")
    print(f"MUXOUT function: {parameters.muxout_lock_detect}")
    print("MUXOUT logic level: 3.3 V")
    print(f"Mute till lock detect: {parameters.mute_till_lock}")
    print(
        "Charge-pump current: "
        f"code {parameters.charge_pump_current_code} "
        f"({(parameters.charge_pump_current_code + 1) * 0.3125:.4f} mA)"
    )
    if parameters.negative_bleed_enabled:
        print(
            "Negative bleed current: enabled; "
            f"code {parameters.negative_bleed_current_code}; "
            f"{float(parameters.negative_bleed_current_ma):.6f} mA"
        )
    else:
        print("Negative bleed current: disabled (integer-N mode)")
    print(f"RFOUTA power: {output_power_dbm:+d} dBm")
    print(f"RFOUTB enabled: {args.enable_rf_output_b}")

    if args.check_muxout:
        print(
            f"MUXOUT state: "
            f"{'HIGH' if muxout_state else 'LOW'}"
        )

    if args.wait_for_lock:
        print("Digital lock: asserted")


if __name__ == "__main__":
    main()

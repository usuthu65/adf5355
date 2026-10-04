#!/usr/bin/env python3
"""
adf5355_pi_sweep.py

Continuous ADF5355 frequency sweep driver for Raspberry Pi SPI0.

There is no --sweep option and no --rf-output-hz option.

The ADF5355 serial-interface CLK is permitted up to 50 MHz. The
default SPI speed remains 10 MHz.

Timing options:

    --freq-step-verbose

        Print absolute elapsed time, commanded frequency, and duration
        of each individual frequency command.

    --chirp-time-verbose

        Print one-way timing and detailed preparation/command statistics.

Preparation profiling sections:

    initial validation
    reference configuration
    RF-divider selection
    N-divider integer arithmetic
    MOD2/FRAC2 arithmetic
    final validation
    ADC-clock calculation
    parameter-object update

Command profiling sections:

    sequence construction
    SPI transfer
    protocol waits
    digital-lock wait
    driver overhead
    total command time

Digital-lock measurement:

    --measure-lock-time arms a GPIO25 rising-edge event before each
    frequency update. It measures from completion of the final,
    AUTOCAL-enabled Register 0 write to the MUXOUT digital-lock edge.
    The wait is included as the "digital-lock wait" command component.
    This mode waits for every update, so it measures lock-limited sweep
    timing rather than an unrestricted chirp rate.

Sequence-construction optimization:

    Frequency updates construct only the registers actually used by the
    update sequence. The previous implementation constructed two complete
    register maps for every update, even though most registers were not
    transmitted during a frequency update.

Corrected registers:

    Register 7  = 0x120000E7
    Register 9  = calculated from fPFD for calibration timing
    Register 10 = calculated from fPFD
    Register 12 = 0x0001041C

Register 4:

    Charge-pump-current code = 9
    Positive phase-detector polarity
    MUXOUT logic level       = 3.3 V

Register 6:

    CP bleed-current code = 16
    RFOUTB                = disabled
"""

from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from dataclasses import dataclass, field
from fractions import Fraction
from math import gcd
from statistics import mean, median
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
# Constants
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
MAX_SPI_SPEED_HZ = 50_000_000
DEFAULT_SPI_SPEED_HZ = 10_000_000

ADC_TARGET_HZ = 100_000
ADC_CLK_DIV_MIN = 1
ADC_CLK_DIV_MAX = 255

REQUIRED_ADC_CYCLES = 16
TIMING_MARGIN_NS = 10_000
NS_PER_SECOND = 1_000_000_000

REGISTER_7_VALUE = 0x120000E7

# Register 9 timing fields. These must be calculated from fPFD; a fixed
# Register 9 value is not valid when the reference configuration changes.
VCO_BAND_DIVIDER_MAX = 255
TIMEOUT_MAX = 1023
ALC_WAIT = 30
SYNTHESIZER_LOCK_TIMEOUT = 12
MIN_SYNTHESIZER_LOCK_SETTLING_NS = 20_000
MIN_ALC_SETTLING_NS = 50_000
VCO_BAND_SELECTION_MAX_HZ = 150_000
VCO_BAND_SELECTION_CYCLES = 11

PHASE_RESYNC_TIMEOUT = 0x041

DEFAULT_CHARGE_PUMP_CURRENT_CODE = 9
PHASE_DETECTOR_POLARITY_POSITIVE = True
CP_BLEED_CURRENT_CODE = 16

PROFILE_SECTIONS = (
    "initial validation",
    "reference configuration",
    "RF-divider selection",
    "N-divider integer arithmetic",
    "MOD2/FRAC2 arithmetic",
    "final validation",
    "ADC-clock calculation",
    "parameter-object update",
)

COMMAND_SECTIONS = (
    "sequence construction",
    "SPI transfer",
    "protocol waits",
    "digital-lock wait",
    "driver overhead",
    "total command time",
)


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

R0_INT_SHIFT = 4
R0_AUTOCAL_SHIFT = 21

R1_FRAC1_SHIFT = 4

R2_MOD2_SHIFT = 4
R2_FRAC2_SHIFT = 18

R4_COUNTER_RESET_MASK = 1 << 4
R4_PHASE_DETECTOR_POLARITY_MASK = 1 << 7
R4_MUXOUT_SHIFT = 27
R4_MUXOUT_LOGIC_SHIFT = 8
R4_REFERENCE_MODE_SHIFT = 9
R4_CHARGE_PUMP_CURRENT_SHIFT = 10
R4_R_COUNTER_SHIFT = 15
R4_RDIV2_SHIFT = 25

R6_RF_DIVIDER_SHIFT = 21
R6_FEEDBACK_FUNDAMENTAL_MASK = 1 << 24
R6_MUTE_TILL_LOCK_MASK = 1 << 11
R6_RFOUTB_ENABLE_MASK = 1 << 10
R6_RFOUTA_ENABLE_MASK = 1 << 6
R6_RFOUTA_POWER_SHIFT = 4
R6_CP_BLEED_CURRENT_SHIFT = 13

R10_ADC_CONVERSION_ENABLE_MASK = 1 << 4
R10_ADC_ENABLE_MASK = 1 << 5
R10_ADC_CLK_DIV_SHIFT = 6


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
# Warnings
# ============================================================================

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
""".strip()


# ============================================================================
# Data classes
# ============================================================================

@dataclass
class ParameterPreparationStatistics:
    section_samples_ns: dict[str, list[int]] = field(
        default_factory=lambda: {
            section: []
            for section in PROFILE_SECTIONS
        }
    )
    total_samples_ns: list[int] = field(default_factory=list)

    def record_section(
        self,
        section: str,
        duration_ns: int,
    ) -> None:
        self.section_samples_ns[section].append(duration_ns)

    def record_total(self, duration_ns: int) -> None:
        self.total_samples_ns.append(duration_ns)


@dataclass
class CommandTimingStatistics:
    section_samples_ns: dict[str, list[int]] = field(
        default_factory=lambda: {
            section: []
            for section in COMMAND_SECTIONS
        }
    )

    def record(
        self,
        sequence_construction_ns: int,
        spi_transfer_ns: int,
        protocol_wait_ns: int,
        digital_lock_wait_ns: int,
        driver_overhead_ns: int,
        total_ns: int,
    ) -> None:
        self.section_samples_ns[
            "sequence construction"
        ].append(sequence_construction_ns)

        self.section_samples_ns[
            "SPI transfer"
        ].append(spi_transfer_ns)

        self.section_samples_ns[
            "protocol waits"
        ].append(protocol_wait_ns)

        self.section_samples_ns[
            "digital-lock wait"
        ].append(digital_lock_wait_ns)

        self.section_samples_ns[
            "driver overhead"
        ].append(driver_overhead_ns)

        self.section_samples_ns[
            "total command time"
        ].append(total_ns)


@dataclass(frozen=True, slots=True)
class ADCClockConfiguration:
    pfd_hz: Fraction
    adc_clk_div: int
    adc_clock_hz: Fraction
    required_interval_ns: int
    enforced_interval_ns: int


@dataclass(frozen=True, slots=True)
class ReferenceConfiguration:
    reference_hz: int
    reference_mode: str
    reference_divider: int
    reference_divide_by_2: bool
    pfd_hz: Fraction
    adc_clock: ADCClockConfiguration


@dataclass(slots=True)
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

    adc_clock: ADCClockConfiguration


@dataclass(frozen=True, slots=True)
class ProgrammingStep:
    register: int
    value: int
    description: str
    adc_clock_for_following_r0: Optional[ADCClockConfiguration] = None
    delay_after_ns: int = 0
    starts_lock_measurement: bool = False


@dataclass(frozen=True, slots=True)
class CommandTimingSample:
    total_ns: int
    sequence_construction_ns: int
    spi_transfer_ns: int
    protocol_wait_ns: int
    digital_lock_wait_ns: int
    driver_overhead_ns: int
    lock_latency_ns: Optional[int]
    lock_timed_out: bool


# ============================================================================
# Utility functions
# ============================================================================

def ceil_fraction(value: Fraction) -> int:
    if value.denominator == 1:
        return value.numerator

    return (
        value.numerator + value.denominator - 1
    ) // value.denominator


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


def validate_fixed_configuration() -> None:
    if not 0 <= CP_BLEED_CURRENT_CODE <= 0xFF:
        raise ValueError("Invalid CP bleed-current code")


def muxout_code(muxout_lock_detect: str) -> int:
    validate_muxout_lock_detect(muxout_lock_detect)

    if muxout_lock_detect == MUXOUT_ANALOG_LOCK_DETECT:
        return MUXOUT_ANALOG_LOCK_DETECT_CODE

    return MUXOUT_DIGITAL_LOCK_DETECT_CODE


def print_hardware_wiring() -> None:
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

Reference input:

    Single-ended:  source -> REFINA
    Differential:  source+ -> REFINA
                   source- -> REFINB

RFOUTB is disabled by this driver. Only RFOUTA is used.
"""
    )


# ============================================================================
# ADC/reference calculations
# ============================================================================

def calculate_adc_clock(
    pfd_hz: Fraction | int,
    adc_clk_div: Optional[int] = None,
) -> ADCClockConfiguration:
    pfd_hz = Fraction(pfd_hz)

    if pfd_hz <= 0:
        raise ValueError("PFD frequency must be positive")

    if adc_clk_div is None:
        requested_divider = ceil_fraction(
            ((pfd_hz / ADC_TARGET_HZ) - 2) / 4
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
            Fraction(REQUIRED_ADC_CYCLES * NS_PER_SECOND, 1)
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


def build_reference_configuration(
    reference_hz: int,
    reference_mode: str,
) -> ReferenceConfiguration:
    validate_reference_mode(reference_mode)

    if not (
        MIN_REFERENCE_HZ
        <= reference_hz
        <= maximum_reference_hz(reference_mode)
    ):
        raise ValueError("Reference frequency is outside the allowed range")

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

    return ReferenceConfiguration(
        reference_hz=reference_hz,
        reference_mode=reference_mode,
        reference_divider=reference_divider,
        reference_divide_by_2=reference_divide_by_2,
        pfd_hz=pfd_hz,
        adc_clock=calculate_adc_clock(pfd_hz),
    )


def select_rf_divider(rf_out_hz: int) -> int:
    for rf_divider in RF_DIVIDER_TO_CODE:
        if 3_400_000_000 <= rf_out_hz * rf_divider <= 6_800_000_000:
            return rf_divider

    raise ValueError("RFOUTA frequency cannot be generated")


# ============================================================================
# Exact integer parameter calculation
# ============================================================================

def calculate_synthesizer_parameters(
    rf_out_hz: int,
    reference_hz: int = DEFAULT_REFERENCE_HZ,
    reference_mode: str = DEFAULT_REFERENCE_MODE,
    channel_spacing_hz: int = DEFAULT_CHANNEL_SPACING_HZ,
    muxout_lock_detect: str = DEFAULT_MUXOUT_LOCK_DETECT,
    mute_till_lock: bool = DEFAULT_MUTE_TILL_LOCK,
    charge_pump_current_code: int = DEFAULT_CHARGE_PUMP_CURRENT_CODE,
    reference_configuration: Optional[
        ReferenceConfiguration
    ] = None,
    parameters_out: Optional[SynthesizerParameters] = None,
    preparation_statistics: Optional[
        ParameterPreparationStatistics
    ] = None,
) -> SynthesizerParameters:
    profile_start_ns = (
        time.perf_counter_ns()
        if preparation_statistics is not None
        else None
    )

    section_start_ns = (
        time.perf_counter_ns()
        if preparation_statistics is not None
        else None
    )

    validate_fixed_configuration()
    validate_muxout_lock_detect(muxout_lock_detect)
    validate_charge_pump_current_code(charge_pump_current_code)

    if not MIN_RF_OUTPUT_HZ <= rf_out_hz <= MAX_RF_OUTPUT_HZ:
        raise ValueError("RFOUTA frequency is outside the allowed range")

    if channel_spacing_hz <= 0:
        raise ValueError("Channel spacing must be positive")

    if reference_configuration is None:
        reference_configuration = build_reference_configuration(
            reference_hz,
            reference_mode,
        )
    elif (
        reference_configuration.reference_hz != reference_hz
        or reference_configuration.reference_mode != reference_mode
    ):
        raise ValueError(
            "Cached reference configuration does not match inputs"
        )

    if preparation_statistics is not None:
        preparation_statistics.record_section(
            "initial validation",
            time.perf_counter_ns() - section_start_ns,
        )

    section_start_ns = (
        time.perf_counter_ns()
        if preparation_statistics is not None
        else None
    )

    reference_divider = reference_configuration.reference_divider
    reference_divide_by_2 = (
        reference_configuration.reference_divide_by_2
    )
    pfd_hz = reference_configuration.pfd_hz
    adc_clock = reference_configuration.adc_clock

    if preparation_statistics is not None:
        preparation_statistics.record_section(
            "reference configuration",
            time.perf_counter_ns() - section_start_ns,
        )

    section_start_ns = (
        time.perf_counter_ns()
        if preparation_statistics is not None
        else None
    )

    rf_divider = select_rf_divider(rf_out_hz)
    vco_hz = rf_out_hz * rf_divider

    if preparation_statistics is not None:
        preparation_statistics.record_section(
            "RF-divider selection",
            time.perf_counter_ns() - section_start_ns,
        )

    section_start_ns = (
        time.perf_counter_ns()
        if preparation_statistics is not None
        else None
    )

    pfd_numerator = pfd_hz.numerator
    pfd_denominator = pfd_hz.denominator
    vco_numerator = vco_hz * pfd_denominator

    int_value, remainder_numerator = divmod(
        vco_numerator,
        pfd_numerator,
    )

    frac1, frac1_remainder_numerator = divmod(
        remainder_numerator * MOD1,
        pfd_numerator,
    )

    remainder = frac1_remainder_numerator

    if preparation_statistics is not None:
        preparation_statistics.record_section(
            "N-divider integer arithmetic",
            time.perf_counter_ns() - section_start_ns,
        )

    section_start_ns = (
        time.perf_counter_ns()
        if preparation_statistics is not None
        else None
    )

    if remainder == 0:
        mod2 = MIN_MOD2
        frac2 = 0
    else:
        mod2 = pfd_numerator // gcd(
            pfd_numerator,
            channel_spacing_hz * pfd_denominator,
        )

        if not MIN_MOD2 <= mod2 <= MAX_MOD2:
            raise ValueError("Calculated MOD2 is invalid")

        frac2, frac2_remainder = divmod(
            remainder * mod2,
            pfd_numerator,
        )

        if frac2_remainder != 0:
            raise ValueError(
                "Calculated FRAC2 is not an integer"
            )

    if preparation_statistics is not None:
        preparation_statistics.record_section(
            "MOD2/FRAC2 arithmetic",
            time.perf_counter_ns() - section_start_ns,
        )

    section_start_ns = (
        time.perf_counter_ns()
        if preparation_statistics is not None
        else None
    )

    if not MIN_INT_4_5_PRESCALER <= int_value <= MAX_INT_4_5_PRESCALER:
        raise ValueError("Calculated INT is invalid")

    if not MIN_FRAC1 <= frac1 <= MAX_FRAC1:
        raise ValueError("Calculated FRAC1 is invalid")

    if not MIN_FRAC2 <= frac2 < mod2:
        raise ValueError("Calculated FRAC2 is invalid")

    if preparation_statistics is not None:
        preparation_statistics.record_section(
            "final validation",
            time.perf_counter_ns() - section_start_ns,
        )

    section_start_ns = (
        time.perf_counter_ns()
        if preparation_statistics is not None
        else None
    )

    if preparation_statistics is not None:
        preparation_statistics.record_section(
            "ADC-clock calculation",
            time.perf_counter_ns() - section_start_ns,
        )

    section_start_ns = (
        time.perf_counter_ns()
        if preparation_statistics is not None
        else None
    )

    if parameters_out is None:
        parameters = SynthesizerParameters(
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
            adc_clock=adc_clock,
        )
    else:
        parameters_out.rf_out_hz = rf_out_hz
        parameters_out.reference_hz = reference_hz
        parameters_out.reference_mode = reference_mode
        parameters_out.channel_spacing_hz = channel_spacing_hz
        parameters_out.muxout_lock_detect = muxout_lock_detect
        parameters_out.mute_till_lock = mute_till_lock
        parameters_out.charge_pump_current_code = (
            charge_pump_current_code
        )
        parameters_out.rf_divider = rf_divider
        parameters_out.vco_hz = vco_hz
        parameters_out.pfd_hz = pfd_hz
        parameters_out.reference_divider = reference_divider
        parameters_out.reference_divide_by_2 = reference_divide_by_2
        parameters_out.int_value = int_value
        parameters_out.frac1 = frac1
        parameters_out.mod1 = MOD1
        parameters_out.frac2 = frac2
        parameters_out.mod2 = mod2
        parameters_out.adc_clock = adc_clock
        parameters = parameters_out

    if preparation_statistics is not None:
        preparation_statistics.record_section(
            "parameter-object update",
            time.perf_counter_ns() - section_start_ns,
        )

        preparation_statistics.record_total(
            time.perf_counter_ns() - profile_start_ns
        )

    return parameters


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
) -> int:
    validate_rf_output_power(output_power_dbm)
    validate_fixed_configuration()

    value = 0x14000000 | REG_R6

    value |= (
        RF_DIVIDER_TO_CODE[parameters.rf_divider]
        << R6_RF_DIVIDER_SHIFT
    )

    value |= R6_FEEDBACK_FUNDAMENTAL_MASK

    value |= (
        CP_BLEED_CURRENT_CODE
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

    value &= ~R6_RFOUTB_ENABLE_MASK

    return value


def make_register_9(parameters: SynthesizerParameters) -> int:
    """Build Register 9 with datasheet-compliant calibration timing."""
    pfd_hz = parameters.pfd_hz

    vco_band_divider = ceil_fraction(
        pfd_hz / 2_400_000
    )

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
    value = 0x00C0000A

    value |= (
        R10_ADC_ENABLE_MASK
        | R10_ADC_CONVERSION_ENABLE_MASK
    )

    value |= (
        parameters.adc_clock.adc_clk_div
        << R10_ADC_CLK_DIV_SHIFT
    )

    return value


def make_register_12() -> int:
    return (
        (1 << 16)
        | (PHASE_RESYNC_TIMEOUT << 4)
        | REG_R12
    )


def make_register_map(
    parameters: SynthesizerParameters,
    output_power_dbm: int,
    enable_rfout_a: bool,
    autocal_enabled: bool = True,
    counter_reset: bool = False,
) -> dict[int, int]:
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
        ),
        REG_R7: REGISTER_7_VALUE,
        REG_R8: 0x102D0428,
        REG_R9: make_register_9(parameters),
        REG_R10: make_register_10(parameters),
        REG_R11: 0x0061300B,
        REG_R12: make_register_12(),
    }


# ============================================================================
# Programming sequences
# ============================================================================

def make_initialization_sequence(
    parameters: SynthesizerParameters,
    output_power_dbm: int,
    enable_rfout_a: bool,
) -> list[ProgrammingStep]:
    registers = make_register_map(
        parameters,
        output_power_dbm,
        enable_rfout_a,
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


def make_frequency_update_sequence_optimized(
    parameters: SynthesizerParameters,
    output_power_dbm: int,
    enable_rfout_a: bool,
) -> list[ProgrammingStep]:
    """
    Optimized frequency-update sequence construction.

    Only R0, R1, R2, R4, and R10 are required by the update sequence.
    The previous implementation constructed two complete register maps,
    including registers that were never transmitted.
    """
    r10 = make_register_10(parameters)

    r4_reset = make_register_4(
        parameters,
        counter_reset=True,
    )

    r4_normal = make_register_4(
        parameters,
        counter_reset=False,
    )

    r2 = make_register_2(parameters)
    r1 = make_register_1(parameters)

    r0_autocal_disabled = make_register_0(
        parameters,
        autocal_enabled=False,
    )

    r0_autocal_enabled = make_register_0(
        parameters,
        autocal_enabled=True,
    )

    if parameters.pfd_hz <= HIGH_PFD_THRESHOLD_HZ:
        return [
            ProgrammingStep(REG_R10, r10, "update R10"),
            ProgrammingStep(
                REG_R4,
                r4_reset,
                "update R4, reset enabled",
            ),
            ProgrammingStep(REG_R2, r2, "update R2"),
            ProgrammingStep(REG_R1, r1, "update R1"),
            ProgrammingStep(
                REG_R0,
                r0_autocal_disabled,
                "AUTOCAL disabled",
            ),
            ProgrammingStep(
                REG_R4,
                r4_normal,
                "reset disabled",
                delay_after_ns=parameters.adc_clock.enforced_interval_ns,
            ),
            ProgrammingStep(
                REG_R0,
                r0_autocal_enabled,
                "AUTOCAL enabled",
                starts_lock_measurement=True,
            ),
        ]

    half_adc_clock = calculate_adc_clock(parameters.pfd_hz / 2)

    return [
        ProgrammingStep(REG_R10, r10, "half-PFD R10"),
        ProgrammingStep(REG_R4, r4_reset, "half-PFD R4"),
        ProgrammingStep(REG_R2, r2, "half-PFD R2"),
        ProgrammingStep(REG_R1, r1, "half-PFD R1"),
        ProgrammingStep(
            REG_R0,
            r0_autocal_disabled,
            "half-PFD AUTOCAL off",
        ),
        ProgrammingStep(
            REG_R4,
            r4_normal,
            "half-PFD reset disabled",
            delay_after_ns=half_adc_clock.enforced_interval_ns,
        ),
        ProgrammingStep(
            REG_R0,
            r0_autocal_enabled,
            "half-PFD AUTOCAL on",
        ),
        ProgrammingStep(REG_R10, r10, "final-PFD R10"),
        ProgrammingStep(REG_R4, r4_reset, "final-PFD R4"),
        ProgrammingStep(REG_R2, r2, "final-PFD R2"),
        ProgrammingStep(REG_R1, r1, "final-PFD R1"),
        ProgrammingStep(
            REG_R0,
            r0_autocal_disabled,
            "final-PFD AUTOCAL off",
        ),
        ProgrammingStep(
            REG_R4,
            r4_normal,
            "final-PFD reset disabled",
            delay_after_ns=parameters.adc_clock.enforced_interval_ns,
        ),
        ProgrammingStep(
            REG_R0,
            r0_autocal_enabled,
            "final-PFD AUTOCAL on",
            starts_lock_measurement=True,
        ),
    ]


def make_frequency_update_sequence_reference(
    parameters: SynthesizerParameters,
    output_power_dbm: int,
    enable_rfout_a: bool,
) -> list[ProgrammingStep]:
    """
    Reference implementation used only by regression tests.

    This intentionally recreates the previous full-register-map approach.
    """
    reset = make_register_map(
        parameters,
        output_power_dbm,
        enable_rfout_a,
        autocal_enabled=False,
        counter_reset=True,
    )

    normal = make_register_map(
        parameters,
        output_power_dbm,
        enable_rfout_a,
        autocal_enabled=True,
        counter_reset=False,
    )

    if parameters.pfd_hz <= HIGH_PFD_THRESHOLD_HZ:
        return [
            ProgrammingStep(REG_R10, reset[REG_R10], "update R10"),
            ProgrammingStep(
                REG_R4,
                reset[REG_R4],
                "update R4, reset enabled",
            ),
            ProgrammingStep(REG_R2, reset[REG_R2], "update R2"),
            ProgrammingStep(REG_R1, reset[REG_R1], "update R1"),
            ProgrammingStep(
                REG_R0,
                reset[REG_R0],
                "AUTOCAL disabled",
            ),
            ProgrammingStep(
                REG_R4,
                normal[REG_R4],
                "reset disabled",
                delay_after_ns=parameters.adc_clock.enforced_interval_ns,
            ),
            ProgrammingStep(
                REG_R0,
                normal[REG_R0],
                "AUTOCAL enabled",
                starts_lock_measurement=True,
            ),
        ]

    half_adc_clock = calculate_adc_clock(parameters.pfd_hz / 2)

    return [
        ProgrammingStep(REG_R10, reset[REG_R10], "half-PFD R10"),
        ProgrammingStep(REG_R4, reset[REG_R4], "half-PFD R4"),
        ProgrammingStep(REG_R2, reset[REG_R2], "half-PFD R2"),
        ProgrammingStep(REG_R1, reset[REG_R1], "half-PFD R1"),
        ProgrammingStep(
            REG_R0,
            reset[REG_R0],
            "half-PFD AUTOCAL off",
        ),
        ProgrammingStep(
            REG_R4,
            normal[REG_R4],
            "half-PFD reset disabled",
            delay_after_ns=half_adc_clock.enforced_interval_ns,
        ),
        ProgrammingStep(
            REG_R0,
            normal[REG_R0],
            "half-PFD AUTOCAL on",
        ),
        ProgrammingStep(REG_R10, reset[REG_R10], "final-PFD R10"),
        ProgrammingStep(REG_R4, reset[REG_R4], "final-PFD R4"),
        ProgrammingStep(REG_R2, reset[REG_R2], "final-PFD R2"),
        ProgrammingStep(REG_R1, reset[REG_R1], "final-PFD R1"),
        ProgrammingStep(
            REG_R0,
            reset[REG_R0],
            "final-PFD AUTOCAL off",
        ),
        ProgrammingStep(
            REG_R4,
            normal[REG_R4],
            "final-PFD reset disabled",
            delay_after_ns=parameters.adc_clock.enforced_interval_ns,
        ),
        ProgrammingStep(
            REG_R0,
            normal[REG_R0],
            "final-PFD AUTOCAL on",
            starts_lock_measurement=True,
        ),
    ]


# ============================================================================
# MUXOUT handling
# ============================================================================

def read_muxout_gpio() -> int:
    if GPIO is None:
        raise RuntimeError("RPi.GPIO is not installed")

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(MUXOUT_GPIO_BCM, GPIO.IN, pull_up_down=GPIO.PUD_OFF)

    try:
        return int(GPIO.input(MUXOUT_GPIO_BCM))
    finally:
        GPIO.cleanup(MUXOUT_GPIO_BCM)


def wait_for_digital_lock() -> None:
    if GPIO is None:
        raise RuntimeError("RPi.GPIO is not installed")

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(MUXOUT_GPIO_BCM, GPIO.IN, pull_up_down=GPIO.PUD_OFF)

    try:
        while not GPIO.input(MUXOUT_GPIO_BCM):
            time.sleep(0.001)
    finally:
        GPIO.cleanup(MUXOUT_GPIO_BCM)


class DigitalLockMonitor:
    """Measure final-R0-to-MUXOUT-digital-lock latency on GPIO25."""

    def __init__(self) -> None:
        if GPIO is None:
            raise RuntimeError("digital-lock measurement requires RPi.GPIO")

        self._edge_event = threading.Event()
        self._guard = threading.Lock()
        self._armed = False
        self._edge_timestamp_ns: Optional[int] = None

    def __enter__(self) -> "DigitalLockMonitor":
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(MUXOUT_GPIO_BCM, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
        GPIO.add_event_detect(
            MUXOUT_GPIO_BCM,
            GPIO.RISING,
            callback=self._on_rising_edge,
        )
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        GPIO.remove_event_detect(MUXOUT_GPIO_BCM)
        GPIO.cleanup(MUXOUT_GPIO_BCM)

    def _on_rising_edge(self, channel: int) -> None:
        del channel

        with self._guard:
            if not self._armed:
                return

            self._edge_timestamp_ns = time.monotonic_ns()
            self._edge_event.set()

    def arm(self) -> None:
        """Discard older edges and arm the next MUXOUT rising edge."""
        with self._guard:
            self._edge_timestamp_ns = None
            self._edge_event.clear()
            self._armed = True

    def wait_for_lock(
        self,
        final_r0_end_ns: int,
        timeout_ns: int,
    ) -> tuple[Optional[int], int]:
        """Return (lock latency, elapsed wait); latency is None on timeout."""
        deadline_ns = final_r0_end_ns + timeout_ns

        while True:
            with self._guard:
                edge_timestamp_ns = self._edge_timestamp_ns

            if edge_timestamp_ns is not None:
                with self._guard:
                    self._armed = False

                if edge_timestamp_ns >= final_r0_end_ns:
                    return (
                        edge_timestamp_ns - final_r0_end_ns,
                        time.monotonic_ns() - final_r0_end_ns,
                    )

                # An edge from before final R0 cannot represent this update.
                self.arm()

            remaining_ns = deadline_ns - time.monotonic_ns()

            if remaining_ns <= 0:
                with self._guard:
                    self._armed = False

                return None, timeout_ns

            self._edge_event.wait(remaining_ns / NS_PER_SECOND)


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
        max_speed_hz: int = DEFAULT_SPI_SPEED_HZ,
        spi=None,
        verbose: bool = False,
    ) -> None:
        if bus != self.SPI_BUS:
            raise ValueError("This driver only allows SPI0")

        if device != self.SPI_DEVICE:
            raise ValueError("This driver only allows SPI0 CE0")

        if not MIN_SPI_SPEED_HZ <= max_speed_hz <= MAX_SPI_SPEED_HZ:
            raise ValueError(
                f"SPI speed must be between "
                f"{MIN_SPI_SPEED_HZ} and {MAX_SPI_SPEED_HZ} Hz"
            )

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
        if (value & 0xF) == REG_R6:
            if value & R6_RFOUTB_ENABLE_MASK:
                raise ValueError("RFOUTB is prohibited")

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
                    / NS_PER_SECOND
                )
            else:
                time.sleep(0)

    def write_sequence(
        self,
        steps: Iterable[ProgrammingStep],
        sequence_name: str,
        command_start_ns: Optional[int] = None,
        sequence_construction_ns: int = 0,
        lock_monitor: Optional[DigitalLockMonitor] = None,
        lock_timeout_ns: int = 0,
    ) -> CommandTimingSample:
        step_list = list(steps)

        if command_start_ns is None:
            command_start_ns = time.monotonic_ns()

        previous_end_ns: Optional[int] = None
        pending_r1_end_ns: Optional[int] = None
        pending_adc_clock: Optional[ADCClockConfiguration] = None

        spi_transfer_ns = 0
        protocol_wait_ns = 0
        digital_lock_wait_ns = 0
        lock_latency_ns: Optional[int] = None
        lock_timed_out = False

        if lock_monitor is not None:
            lock_monitor.arm()

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
                wait_start_ns = time.monotonic_ns()

                self.wait_until(
                    pending_r1_end_ns
                    + pending_adc_clock.enforced_interval_ns
                )

                protocol_wait_ns += (
                    time.monotonic_ns()
                    - wait_start_ns
                )

            transfer_start_ns = time.monotonic_ns()
            start_ns, end_ns = self._write_register(step.value)
            transfer_end_ns = time.monotonic_ns()

            spi_transfer_ns += transfer_end_ns - transfer_start_ns

            if step.register == REG_R1:
                pending_r1_end_ns = end_ns
                pending_adc_clock = step.adc_clock_for_following_r0

            if step.delay_after_ns:
                wait_start_ns = time.monotonic_ns()

                self.wait_until(end_ns + step.delay_after_ns)

                protocol_wait_ns += (
                    time.monotonic_ns()
                    - wait_start_ns
                )

            if step.starts_lock_measurement and lock_monitor is not None:
                lock_latency_ns, digital_lock_wait_ns = (
                    lock_monitor.wait_for_lock(
                        end_ns,
                        lock_timeout_ns,
                    )
                )
                lock_timed_out = lock_latency_ns is None

            if self.verbose:
                start_us = (
                    start_ns - command_start_ns
                ) / 1000.0

                end_us = (
                    end_ns - command_start_ns
                ) / 1000.0

                transfer_us = (
                    end_ns - start_ns
                ) / 1000.0

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

        command_end_ns = time.monotonic_ns()
        total_ns = command_end_ns - command_start_ns

        driver_overhead_ns = max(
            0,
            total_ns
            - sequence_construction_ns
            - spi_transfer_ns
            - protocol_wait_ns
            - digital_lock_wait_ns,
        )

        return CommandTimingSample(
            total_ns=total_ns,
            sequence_construction_ns=sequence_construction_ns,
            spi_transfer_ns=spi_transfer_ns,
            protocol_wait_ns=protocol_wait_ns,
            digital_lock_wait_ns=digital_lock_wait_ns,
            driver_overhead_ns=driver_overhead_ns,
            lock_latency_ns=lock_latency_ns,
            lock_timed_out=lock_timed_out,
        )

    def program_initial_frequency(
        self,
        parameters: SynthesizerParameters,
        output_power_dbm: int,
        enable_rfout_a: bool,
    ) -> None:
        command_start_ns = time.monotonic_ns()
        construction_start_ns = time.monotonic_ns()

        steps = make_initialization_sequence(
            parameters,
            output_power_dbm,
            enable_rfout_a,
        )

        construction_duration_ns = (
            time.monotonic_ns() - construction_start_ns
        )

        self.write_sequence(
            steps,
            "Register Initialization Sequence",
            command_start_ns=command_start_ns,
            sequence_construction_ns=construction_duration_ns,
        )

    def update_frequency(
        self,
        parameters: SynthesizerParameters,
        output_power_dbm: int,
        enable_rfout_a: bool,
        command_start_ns: Optional[int] = None,
        lock_monitor: Optional[DigitalLockMonitor] = None,
        lock_timeout_ns: int = 0,
    ) -> CommandTimingSample:
        if command_start_ns is None:
            command_start_ns = time.monotonic_ns()

        construction_start_ns = time.monotonic_ns()

        steps = make_frequency_update_sequence_optimized(
            parameters,
            output_power_dbm,
            enable_rfout_a,
        )

        construction_duration_ns = (
            time.monotonic_ns() - construction_start_ns
        )

        if parameters.pfd_hz <= HIGH_PFD_THRESHOLD_HZ:
            name = "Frequency Update Sequence (fPFD <= 75 MHz)"
        else:
            name = (
                "Frequency Update Sequence "
                "(fPFD > 75 MHz; half-PFD then final-PFD)"
            )

        return self.write_sequence(
            steps,
            name,
            command_start_ns=command_start_ns,
            sequence_construction_ns=construction_duration_ns,
            lock_monitor=lock_monitor,
            lock_timeout_ns=lock_timeout_ns,
        )

    def close(self) -> None:
        if self._owns_spi:
            self.spi.close()

    def __enter__(self) -> "ADF5355":
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()


# ============================================================================
# Sweep generation and validation
# ============================================================================

def generate_sweep_frequencies(
    start_frequency: int,
    end_frequency: int,
    step_frequency: int,
):
    upward = [start_frequency]
    frequency = start_frequency

    while frequency < end_frequency:
        frequency = min(
            frequency + step_frequency,
            end_frequency,
        )
        upward.append(frequency)

    yield from upward

    while True:
        yield from reversed(upward[:-1])
        yield from upward[1:]


def iter_one_sweep_cycle(
    start_frequency: int,
    end_frequency: int,
    step_frequency: int,
):
    upward = [start_frequency]
    frequency = start_frequency

    while frequency < end_frequency:
        frequency = min(
            frequency + step_frequency,
            end_frequency,
        )
        upward.append(frequency)

    yield from upward
    yield from reversed(upward[:-1])


def validate_sweep_arguments(
    start_frequency: int,
    end_frequency: int,
    step_frequency: int,
    step_time_s: float,
    reference_hz: int,
    reference_mode: str,
    channel_spacing_hz: int,
    muxout_lock_detect: str,
    mute_till_lock: bool,
) -> None:
    if not MIN_RF_OUTPUT_HZ <= start_frequency <= MAX_RF_OUTPUT_HZ:
        raise ValueError("start_frequency is outside the RFOUTA range")

    if not MIN_RF_OUTPUT_HZ <= end_frequency <= MAX_RF_OUTPUT_HZ:
        raise ValueError("end_frequency is outside the RFOUTA range")

    if start_frequency >= end_frequency:
        raise ValueError(
            "start_frequency must be less than end_frequency"
        )

    if step_frequency <= 0:
        raise ValueError("step_frequency must be positive")

    if step_frequency > end_frequency - start_frequency:
        raise ValueError(
            "step_frequency must not exceed the sweep span"
        )

    if not isinstance(step_time_s, (int, float)):
        raise TypeError("step_time must be numeric")

    if not math.isfinite(float(step_time_s)):
        raise ValueError("step_time must be finite")

    if step_time_s <= 0:
        raise ValueError("step_time must be greater than zero")


# ============================================================================
# Absolute deadline helper
# ============================================================================

def wait_until_returning_time(deadline_ns: int) -> int:
    while True:
        remaining_ns = deadline_ns - time.monotonic_ns()

        if remaining_ns <= 0:
            return time.monotonic_ns()

        if remaining_ns > 100_000:
            time.sleep(
                (remaining_ns - 50_000)
                / NS_PER_SECOND
            )
        else:
            time.sleep(0)


# ============================================================================
# Direction reporting
# ============================================================================

def report_direction_statistics(
    direction: str,
    script_start_ns: int,
    direction_start_ns: int,
    direction_end_ns: int,
    total_steps: int,
    small_overruns: int,
    large_overruns: int,
    preparation_durations_ns: list[int],
    loop_overhead_durations_ns: list[int],
    command_durations_ns: list[int],
    preparation_statistics: ParameterPreparationStatistics,
    command_statistics: CommandTimingStatistics,
) -> None:
    absolute_elapsed_s = (
        direction_end_ns - script_start_ns
    ) / NS_PER_SECOND

    direction_duration_s = (
        direction_end_ns - direction_start_ns
    ) / NS_PER_SECOND

    preparation_mean_ns = (
        mean(preparation_durations_ns)
        if preparation_durations_ns
        else 0.0
    )

    preparation_median_ns = (
        median(preparation_durations_ns)
        if preparation_durations_ns
        else 0.0
    )

    loop_mean_ns = (
        mean(loop_overhead_durations_ns)
        if loop_overhead_durations_ns
        else 0.0
    )

    loop_median_ns = (
        median(loop_overhead_durations_ns)
        if loop_overhead_durations_ns
        else 0.0
    )

    command_mean_ns = (
        mean(command_durations_ns)
        if command_durations_ns
        else 0.0
    )

    command_median_ns = (
        median(command_durations_ns)
        if command_durations_ns
        else 0.0
    )

    active_durations_ns = [
        preparation_ns + loop_ns + command_ns
        for preparation_ns, loop_ns, command_ns in zip(
            preparation_durations_ns,
            loop_overhead_durations_ns,
            command_durations_ns,
        )
    ]

    active_mean_ns = (
        mean(active_durations_ns)
        if active_durations_ns
        else 0.0
    )

    active_median_ns = (
        median(active_durations_ns)
        if active_durations_ns
        else 0.0
    )

    profiled_mean_ns = (
        mean(preparation_statistics.total_samples_ns)
        if preparation_statistics.total_samples_ns
        else 0.0
    )

    profiled_median_ns = (
        median(preparation_statistics.total_samples_ns)
        if preparation_statistics.total_samples_ns
        else 0.0
    )

    print(
        f"[{absolute_elapsed_s:.6f} s] "
        f"One-way {direction} elapsed time: "
        f"{direction_duration_s:.6f} s",
        flush=True,
    )
    print(
        f"[{absolute_elapsed_s:.6f} s] "
        f"{direction} total steps: "
        f"{total_steps}",
        flush=True,
    )
    print(
        f"[{absolute_elapsed_s:.6f} s] "
        f"{direction} small overrun events: "
        f"{small_overruns}",
        flush=True,
    )
    print(
        f"[{absolute_elapsed_s:.6f} s] "
        f"{direction} large overrun events: "
        f"{large_overruns}",
        flush=True,
    )
    print(
        f"[{absolute_elapsed_s:.6f} s] "
        f"{direction} mean parameter-preparation time: "
        f"{preparation_mean_ns / 1000.0:.3f} usec",
        flush=True,
    )
    print(
        f"[{absolute_elapsed_s:.6f} s] "
        f"{direction} median parameter-preparation time: "
        f"{preparation_median_ns / 1000.0:.3f} usec",
        flush=True,
    )
    print(
        f"[{absolute_elapsed_s:.6f} s] "
        f"{direction} mean loop-overhead time: "
        f"{loop_mean_ns / 1000.0:.3f} usec",
        flush=True,
    )
    print(
        f"[{absolute_elapsed_s:.6f} s] "
        f"{direction} median loop-overhead time: "
        f"{loop_median_ns / 1000.0:.3f} usec",
        flush=True,
    )
    print(
        f"[{absolute_elapsed_s:.6f} s] "
        f"{direction} mean command time: "
        f"{command_mean_ns / 1000.0:.3f} usec",
        flush=True,
    )
    print(
        f"[{absolute_elapsed_s:.6f} s] "
        f"{direction} median command time: "
        f"{command_median_ns / 1000.0:.3f} usec",
        flush=True,
    )
    print(
        f"[{absolute_elapsed_s:.6f} s] "
        f"{direction} mean active step time: "
        f"{active_mean_ns / 1000.0:.3f} usec",
        flush=True,
    )
    print(
        f"[{absolute_elapsed_s:.6f} s] "
        f"{direction} median active step time: "
        f"{active_median_ns / 1000.0:.3f} usec",
        flush=True,
    )

    print(
        f"[{absolute_elapsed_s:.6f} s] "
        f"{direction} profiled preparation mean: "
        f"{profiled_mean_ns / 1000.0:.3f} usec",
        flush=True,
    )
    print(
        f"[{absolute_elapsed_s:.6f} s] "
        f"{direction} profiled preparation median: "
        f"{profiled_median_ns / 1000.0:.3f} usec",
        flush=True,
    )

    print(
        f"[{absolute_elapsed_s:.6f} s] "
        f"{direction} preparation breakdown:",
        flush=True,
    )

    section_mean_sum_ns = 0.0
    section_median_sum_ns = 0.0

    for section in PROFILE_SECTIONS:
        samples = preparation_statistics.section_samples_ns[section]

        section_mean_ns = mean(samples) if samples else 0.0
        section_median_ns = median(samples) if samples else 0.0

        section_mean_sum_ns += section_mean_ns
        section_median_sum_ns += section_median_ns

        fraction = (
            100.0 * section_mean_ns / profiled_mean_ns
            if profiled_mean_ns > 0
            else 0.0
        )

        print(
            f"  {section}: "
            f"mean={section_mean_ns / 1000.0:.3f} usec; "
            f"median={section_median_ns / 1000.0:.3f} usec; "
            f"mean_fraction={fraction:.2f}%",
            flush=True,
        )

    print(
        f"  unaccounted profiling overhead: "
        f"mean={(profiled_mean_ns - section_mean_sum_ns) / 1000.0:.3f} usec; "
        f"median={(profiled_median_ns - section_median_sum_ns) / 1000.0:.3f} usec",
        flush=True,
    )

    print(
        f"[{absolute_elapsed_s:.6f} s] "
        f"{direction} command breakdown:",
        flush=True,
    )

    total_command_samples = (
        command_statistics.section_samples_ns[
            "total command time"
        ]
    )

    # Compute denominators before calculating component percentages.
    total_command_mean_ns = (
        mean(total_command_samples)
        if total_command_samples
        else 0.0
    )

    total_command_median_ns = (
        median(total_command_samples)
        if total_command_samples
        else 0.0
    )

    command_component_mean_sum_ns = 0.0
    command_component_median_sum_ns = 0.0

    for section in COMMAND_SECTIONS:
        samples = command_statistics.section_samples_ns[section]

        section_mean_ns = mean(samples) if samples else 0.0
        section_median_ns = median(samples) if samples else 0.0

        if section != "total command time":
            command_component_mean_sum_ns += section_mean_ns
            command_component_median_sum_ns += section_median_ns

        fraction = (
            100.0 * section_mean_ns / total_command_mean_ns
            if total_command_mean_ns > 0
            else 0.0
        )

        print(
            f"  {section}: "
            f"mean={section_mean_ns / 1000.0:.3f} usec; "
            f"median={section_median_ns / 1000.0:.3f} usec; "
            f"mean_fraction={fraction:.2f}%",
            flush=True,
        )

    print(
        f"  command accounting residual: "
        f"mean={(
            total_command_mean_ns
            - command_component_mean_sum_ns
        ) / 1000.0:.3f} usec; "
        f"median={(
            total_command_median_ns
            - command_component_median_sum_ns
        ) / 1000.0:.3f} usec",
        flush=True,
    )


def report_lock_measurement_statistics(
    lock_latencies_ns: list[int],
    lock_timeouts: int,
) -> None:
    """Print final-R0-to-digital-lock measurements for one direction."""
    print("  final R0 to digital-lock:", flush=True)

    if lock_latencies_ns:
        ordered = sorted(lock_latencies_ns)
        p95_index = min(
            len(ordered) - 1,
            math.ceil(len(ordered) * 0.95) - 1,
        )
        print(
            f"    samples={len(lock_latencies_ns)}; "
            f"min={ordered[0] / 1000.0:.3f} usec; "
            f"median={median(ordered) / 1000.0:.3f} usec; "
            f"mean={mean(ordered) / 1000.0:.3f} usec; "
            f"p95={ordered[p95_index] / 1000.0:.3f} usec; "
            f"max={ordered[-1] / 1000.0:.3f} usec",
            flush=True,
        )

    if lock_timeouts:
        print(f"    timeouts={lock_timeouts}", flush=True)


# ============================================================================
# Sweep execution
# ============================================================================

def run_sweep(
    device: ADF5355,
    args: argparse.Namespace,
    output_power_dbm: int,
    enable_rfout_a: bool,
    script_start_ns: int,
    lock_monitor: Optional[DigitalLockMonitor] = None,
) -> None:
    validate_sweep_arguments(
        args.start_frequency,
        args.end_frequency,
        args.step_frequency,
        args.step_time,
        args.reference_hz,
        args.reference_mode,
        args.channel_spacing_hz,
        args.muxout_lock_detect,
        args.mute_till_lock,
    )

    if args.wait_for_lock:
        if args.muxout_lock_detect != MUXOUT_DIGITAL_LOCK_DETECT:
            raise ValueError(
                "--wait-for-lock requires digital MUXOUT"
            )

        if GPIO is None:
            raise RuntimeError(
                "--wait-for-lock requires RPi.GPIO"
            )

    if args.measure_lock_time and lock_monitor is None:
        raise RuntimeError(
            "--measure-lock-time requires a digital-lock monitor"
        )

    reference_configuration = build_reference_configuration(
        args.reference_hz,
        args.reference_mode,
    )

    start_parameters = calculate_synthesizer_parameters(
        args.start_frequency,
        args.reference_hz,
        args.reference_mode,
        args.channel_spacing_hz,
        args.muxout_lock_detect,
        args.mute_till_lock,
        args.charge_pump_current_code,
        reference_configuration=reference_configuration,
    )

    if args.verbose:
        print_n_divider_configuration(start_parameters)

    initial_command_start_ns = time.monotonic_ns()

    device.program_initial_frequency(
        start_parameters,
        output_power_dbm,
        enable_rfout_a,
    )

    initial_command_end_ns = time.monotonic_ns()

    if args.freq_step_verbose:
        print(
            f"[{(initial_command_end_ns - script_start_ns) / 1e9:.6f} s] "
            f"Current frequency commanded: "
            f"{args.start_frequency} Hz",
            flush=True,
        )
        print(
            f"[{(initial_command_end_ns - script_start_ns) / 1e9:.6f} s] "
            f"Frequency-command duration: "
            f"{(initial_command_end_ns - initial_command_start_ns) / 1000.0:.3f} usec",
            flush=True,
        )

    direction = "start-to-end"
    direction_start_ns = initial_command_end_ns

    step_period_ns = int(round(args.step_time * NS_PER_SECOND))
    next_step_deadline_ns = initial_command_end_ns + step_period_ns

    current_frequency = args.start_frequency
    first_frequency = True

    total_steps = 0
    small_overruns = 0
    large_overruns = 0

    preparation_durations_ns: list[int] = []
    loop_overhead_durations_ns: list[int] = []
    command_durations_ns: list[int] = []
    lock_latencies_ns: list[int] = []
    lock_timeouts = 0

    preparation_statistics = (
        ParameterPreparationStatistics()
    )

    command_statistics = CommandTimingStatistics()

    for frequency in generate_sweep_frequencies(
        args.start_frequency,
        args.end_frequency,
        args.step_frequency,
    ):
        if first_frequency:
            first_frequency = False
            continue

        preparation_start_ns = time.monotonic_ns()

        parameters = calculate_synthesizer_parameters(
            frequency,
            args.reference_hz,
            args.reference_mode,
            args.channel_spacing_hz,
            args.muxout_lock_detect,
            args.mute_till_lock,
            args.charge_pump_current_code,
            reference_configuration=reference_configuration,
            parameters_out=start_parameters,
            preparation_statistics=preparation_statistics,
        )

        preparation_end_ns = time.monotonic_ns()

        preparation_durations_ns.append(
            preparation_end_ns - preparation_start_ns
        )

        if args.wait_for_lock:
            wait_for_digital_lock()

        pre_deadline_work_end_ns = time.monotonic_ns()

        wake_ns = wait_until_returning_time(
            next_step_deadline_ns
        )

        if pre_deadline_work_end_ns < next_step_deadline_ns:
            wake_lateness_ns = wake_ns - next_step_deadline_ns

            if wake_lateness_ns > 0:
                if wake_lateness_ns < step_period_ns:
                    small_overruns += 1
                else:
                    large_overruns += 1

        if args.verbose:
            step_direction = (
                "up"
                if frequency > current_frequency
                else "down"
            )

            print(
                f"Sweep step {step_direction}: "
                f"{current_frequency} -> {frequency} Hz"
            )
            print_n_divider_configuration(parameters)

        deadline_wait_ns = max(
            0,
            wake_ns - pre_deadline_work_end_ns,
        )

        command_start_ns = time.monotonic_ns()

        loop_overhead_ns = max(
            0,
            command_start_ns
            - pre_deadline_work_end_ns
            - deadline_wait_ns,
        )

        command_timing = device.update_frequency(
            parameters,
            output_power_dbm,
            enable_rfout_a,
            command_start_ns=command_start_ns,
            lock_monitor=lock_monitor,
            lock_timeout_ns=int(
                round(args.lock_timeout_ms * 1_000_000)
            ),
        )

        command_end_ns = time.monotonic_ns()

        loop_overhead_durations_ns.append(loop_overhead_ns)
        command_durations_ns.append(command_timing.total_ns)

        command_statistics.record(
            sequence_construction_ns=(
                command_timing.sequence_construction_ns
            ),
            spi_transfer_ns=command_timing.spi_transfer_ns,
            protocol_wait_ns=command_timing.protocol_wait_ns,
            digital_lock_wait_ns=(
                command_timing.digital_lock_wait_ns
            ),
            driver_overhead_ns=command_timing.driver_overhead_ns,
            total_ns=command_timing.total_ns,
        )

        if args.measure_lock_time:
            if command_timing.lock_latency_ns is None:
                lock_timeouts += 1
            else:
                lock_latencies_ns.append(
                    command_timing.lock_latency_ns
                )

        total_steps += 1

        if args.freq_step_verbose:
            absolute_elapsed_s = (
                command_end_ns - script_start_ns
            ) / NS_PER_SECOND

            print(
                f"[{absolute_elapsed_s:.6f} s] "
                f"Current frequency commanded: "
                f"{frequency} Hz",
                flush=True,
            )
            print(
                f"[{absolute_elapsed_s:.6f} s] "
                f"Frequency-command duration: "
                f"{command_timing.total_ns / 1000.0:.3f} usec",
                flush=True,
            )

        if args.lock_time_verbose:
            if command_timing.lock_latency_ns is None:
                print(
                    "Final R0 to digital-lock: "
                    f"timeout after {args.lock_timeout_ms:.3f} ms",
                    flush=True,
                )
            else:
                print(
                    "Final R0 to digital-lock: "
                    f"{command_timing.lock_latency_ns / 1000.0:.3f} usec",
                    flush=True,
                )

        if (
            frequency == args.end_frequency
            and direction == "start-to-end"
        ):
            if args.chirp_time_verbose:
                report_direction_statistics(
                    direction="start-to-end",
                    script_start_ns=script_start_ns,
                    direction_start_ns=direction_start_ns,
                    direction_end_ns=command_end_ns,
                    total_steps=total_steps,
                    small_overruns=small_overruns,
                    large_overruns=large_overruns,
                    preparation_durations_ns=preparation_durations_ns,
                    loop_overhead_durations_ns=loop_overhead_durations_ns,
                    command_durations_ns=command_durations_ns,
                    preparation_statistics=preparation_statistics,
                    command_statistics=command_statistics,
                )

                if args.measure_lock_time:
                    report_lock_measurement_statistics(
                        lock_latencies_ns,
                        lock_timeouts,
                    )

            direction = "end-to-start"
            direction_start_ns = command_end_ns
            total_steps = 0
            small_overruns = 0
            large_overruns = 0
            preparation_durations_ns.clear()
            loop_overhead_durations_ns.clear()
            command_durations_ns.clear()
            lock_latencies_ns.clear()
            lock_timeouts = 0
            preparation_statistics = (
                ParameterPreparationStatistics()
            )
            command_statistics = CommandTimingStatistics()

        elif (
            frequency == args.start_frequency
            and direction == "end-to-start"
        ):
            if args.chirp_time_verbose:
                report_direction_statistics(
                    direction="end-to-start",
                    script_start_ns=script_start_ns,
                    direction_start_ns=direction_start_ns,
                    direction_end_ns=command_end_ns,
                    total_steps=total_steps,
                    small_overruns=small_overruns,
                    large_overruns=large_overruns,
                    preparation_durations_ns=preparation_durations_ns,
                    loop_overhead_durations_ns=loop_overhead_durations_ns,
                    command_durations_ns=command_durations_ns,
                    preparation_statistics=preparation_statistics,
                    command_statistics=command_statistics,
                )

                if args.measure_lock_time:
                    report_lock_measurement_statistics(
                        lock_latencies_ns,
                        lock_timeouts,
                    )

            direction = "start-to-end"
            direction_start_ns = command_end_ns
            total_steps = 0
            small_overruns = 0
            large_overruns = 0
            preparation_durations_ns.clear()
            loop_overhead_durations_ns.clear()
            command_durations_ns.clear()
            lock_latencies_ns.clear()
            lock_timeouts = 0
            preparation_statistics = (
                ParameterPreparationStatistics()
            )
            command_statistics = CommandTimingStatistics()

        current_frequency = frequency

        next_step_deadline_ns += step_period_ns

        now_ns = time.monotonic_ns()

        if next_step_deadline_ns < now_ns - step_period_ns:
            next_step_deadline_ns = now_ns + step_period_ns


# ============================================================================
# Software-only verification
# ============================================================================

class FakeSPI:
    """Minimal SPI replacement."""

    def __init__(self) -> None:
        self.transfers = []

    def xfer2(self, data) -> None:
        self.transfers.append(list(data))

    def close(self) -> None:
        pass


class FakeGPIO:
    BCM = object()
    IN = object()
    PUD_OFF = object()
    RISING = object()

    def __init__(self) -> None:
        self.callback = None

    def setmode(self, mode: object) -> None:
        del mode

    def setup(
        self,
        channel: int,
        mode: object,
        pull_up_down: object,
    ) -> None:
        del channel, mode, pull_up_down

    def add_event_detect(
        self,
        channel: int,
        edge: object,
        callback,
    ) -> None:
        del channel, edge
        self.callback = callback

    def remove_event_detect(self, channel: int) -> None:
        del channel
        self.callback = None

    def cleanup(self, channel: int) -> None:
        del channel


class LockEdgeFakeSPI(FakeSPI):
    def __init__(self, gpio: FakeGPIO) -> None:
        super().__init__()
        self.gpio = gpio

    def xfer2(self, data) -> None:
        super().xfer2(data)

        if self.gpio.callback is not None:
            threading.Timer(
                0.001,
                self.gpio.callback,
                args=(MUXOUT_GPIO_BCM,),
            ).start()


def run_digital_lock_measurement_regression_test() -> None:
    global GPIO

    original_gpio = GPIO
    fake_gpio = FakeGPIO()
    GPIO = fake_gpio

    try:
        fake_spi = LockEdgeFakeSPI(fake_gpio)

        with DigitalLockMonitor() as lock_monitor:
            device = ADF5355(spi=fake_spi)

            try:
                timing = device.write_sequence(
                    [
                        ProgrammingStep(
                            REG_R0,
                            REG_R0,
                            "test final R0",
                            starts_lock_measurement=True,
                        )
                    ],
                    "digital-lock regression test",
                    lock_monitor=lock_monitor,
                    lock_timeout_ns=10_000_000,
                )
            finally:
                device.close()

        assert timing.lock_latency_ns is not None
        assert not timing.lock_timed_out
        assert timing.digital_lock_wait_ns > 0
    finally:
        GPIO = original_gpio


def parameter_signature(
    parameters: SynthesizerParameters,
) -> tuple:
    return (
        parameters.rf_out_hz,
        parameters.reference_hz,
        parameters.reference_mode,
        parameters.channel_spacing_hz,
        parameters.muxout_lock_detect,
        parameters.mute_till_lock,
        parameters.rf_divider,
        parameters.vco_hz,
        parameters.pfd_hz,
        parameters.reference_divider,
        parameters.reference_divide_by_2,
        parameters.int_value,
        parameters.frac1,
        parameters.mod1,
        parameters.frac2,
        parameters.mod2,
        parameters.adc_clock.pfd_hz,
        parameters.adc_clock.adc_clk_div,
        parameters.adc_clock.adc_clock_hz,
        parameters.adc_clock.required_interval_ns,
        parameters.adc_clock.enforced_interval_ns,
    )


def run_sequence_construction_regression_test() -> None:
    """
    Compare optimized and reference update sequences for:

        - below/equal 75 MHz PFD;
        - above 75 MHz PFD;
        - both sweep directions;
        - multiple frequencies.
    """
    cases = [
        (
            2_000_000_000,
            125_000_000,
            REFERENCE_MODE_SINGLE_ENDED,
            100_000,
        ),
        (
            2_100_000_000,
            125_000_000,
            REFERENCE_MODE_SINGLE_ENDED,
            100_000,
        ),
        (
            2_000_000_000,
            200_000_000,
            REFERENCE_MODE_DIFFERENTIAL,
            100_000,
        ),
        (
            2_100_000_000,
            200_000_000,
            REFERENCE_MODE_DIFFERENTIAL,
            100_000,
        ),
    ]

    for (
        rf_frequency,
        reference_frequency,
        reference_mode,
        channel_spacing,
    ) in cases:
        reference_configuration = build_reference_configuration(
            reference_frequency,
            reference_mode,
        )

        parameters = calculate_synthesizer_parameters(
            rf_frequency,
            reference_hz=reference_frequency,
            reference_mode=reference_mode,
            channel_spacing_hz=channel_spacing,
            reference_configuration=reference_configuration,
        )

        optimized = make_frequency_update_sequence_optimized(
            parameters,
            output_power_dbm=5,
            enable_rfout_a=True,
        )

        reference = make_frequency_update_sequence_reference(
            parameters,
            output_power_dbm=5,
            enable_rfout_a=True,
        )

        assert optimized == reference


def run_reuse_regression_test() -> None:
    reference_configuration = build_reference_configuration(
        125_000_000,
        REFERENCE_MODE_SINGLE_ENDED,
    )

    frequencies = list(
        iter_one_sweep_cycle(
            2_000_000_000,
            2_100_000_000,
            1_000_000,
        )
    )

    reusable_parameters = calculate_synthesizer_parameters(
        frequencies[0],
        reference_hz=125_000_000,
        reference_mode=REFERENCE_MODE_SINGLE_ENDED,
        channel_spacing_hz=100_000,
        reference_configuration=reference_configuration,
    )

    for frequency in frequencies:
        fresh_parameters = calculate_synthesizer_parameters(
            frequency,
            reference_hz=125_000_000,
            reference_mode=REFERENCE_MODE_SINGLE_ENDED,
            channel_spacing_hz=100_000,
            reference_configuration=reference_configuration,
        )

        reused_parameters = calculate_synthesizer_parameters(
            frequency,
            reference_hz=125_000_000,
            reference_mode=REFERENCE_MODE_SINGLE_ENDED,
            channel_spacing_hz=100_000,
            reference_configuration=reference_configuration,
            parameters_out=reusable_parameters,
        )

        assert reused_parameters is reusable_parameters
        assert (
            parameter_signature(fresh_parameters)
            == parameter_signature(reused_parameters)
        )

        fresh_registers = make_register_map(
            fresh_parameters,
            output_power_dbm=5,
            enable_rfout_a=True,
        )

        reused_registers = make_register_map(
            reused_parameters,
            output_power_dbm=5,
            enable_rfout_a=True,
        )

        assert fresh_registers == reused_registers


def run_command_profile_regression_test() -> None:
    reference_configuration = build_reference_configuration(
        125_000_000,
        REFERENCE_MODE_SINGLE_ENDED,
    )

    parameters = calculate_synthesizer_parameters(
        2_000_000_000,
        reference_hz=125_000_000,
        reference_mode=REFERENCE_MODE_SINGLE_ENDED,
        channel_spacing_hz=100_000,
        reference_configuration=reference_configuration,
    )

    fake_spi = FakeSPI()
    device = ADF5355(spi=fake_spi)

    try:
        timing = device.update_frequency(
            parameters,
            output_power_dbm=5,
            enable_rfout_a=True,
        )
    finally:
        device.close()

    assert timing.total_ns > 0
    assert timing.sequence_construction_ns >= 0
    assert timing.spi_transfer_ns >= 0
    assert timing.protocol_wait_ns >= 0
    assert timing.digital_lock_wait_ns == 0
    assert timing.lock_latency_ns is None
    assert not timing.lock_timed_out
    assert timing.driver_overhead_ns >= 0

    accounted_ns = (
        timing.sequence_construction_ns
        + timing.spi_transfer_ns
        + timing.protocol_wait_ns
        + timing.digital_lock_wait_ns
        + timing.driver_overhead_ns
    )

    assert accounted_ns <= timing.total_ns


def run_verification() -> None:
    parameters = calculate_synthesizer_parameters(
        1_800_000_000,
        reference_hz=125_000_000,
        reference_mode=REFERENCE_MODE_SINGLE_ENDED,
        channel_spacing_hz=200_000,
    )

    registers = make_register_map(
        parameters,
        output_power_dbm=5,
        enable_rfout_a=True,
    )

    assert registers[REG_R4] == 0x3200A584
    assert registers[REG_R6] == 0x15220076
    assert registers[REG_R7] == 0x120000E7
    assert registers[REG_R9] == 0x1B1A7CC9
    assert registers[REG_R10] == 0x00C0273A
    assert registers[REG_R12] == 0x0001041C

    for charge_pump_current_code in range(16):
        charge_pump_parameters = calculate_synthesizer_parameters(
            1_800_000_000,
            charge_pump_current_code=charge_pump_current_code,
        )
        register_4 = make_register_4(charge_pump_parameters)
        assert (
            (register_4 >> R4_CHARGE_PUMP_CURRENT_SHIFT) & 0xF
            == charge_pump_current_code
        )

    differential_parameters = calculate_synthesizer_parameters(
        1_800_000_000,
        reference_hz=125_000_000,
        reference_mode=REFERENCE_MODE_DIFFERENTIAL,
        channel_spacing_hz=200_000,
    )

    differential_registers = make_register_map(
        differential_parameters,
        output_power_dbm=5,
        enable_rfout_a=True,
    )

    assert differential_registers[REG_R4] == 0x3200A784

    high_pfd_parameters = calculate_synthesizer_parameters(
        1_800_000_000,
        reference_hz=250_000_000,
        reference_mode=REFERENCE_MODE_DIFFERENTIAL,
        channel_spacing_hz=200_000,
    )

    assert high_pfd_parameters.pfd_hz == 125_000_000
    assert make_register_9(high_pfd_parameters) == 0x35347CC9

    run_sequence_construction_regression_test()
    run_reuse_regression_test()
    run_command_profile_regression_test()
    run_digital_lock_measurement_regression_test()

    fake_spi = FakeSPI()
    device = ADF5355(spi=fake_spi)

    try:
        device._write_register(
            REG_R6 | R6_RFOUTB_ENABLE_MASK
        )
    except ValueError:
        pass
    else:
        raise AssertionError("RFOUTB was not rejected")

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


def rf_output_frequency_arg(value: str) -> int:
    return bounded_integer(
        value,
        MIN_RF_OUTPUT_HZ,
        MAX_RF_OUTPUT_HZ,
        "RF output frequency",
    )


def reference_frequency_arg(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            "reference frequency must be an integer"
        ) from exc

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


# ============================================================================
# Main
# ============================================================================

def main() -> None:
    script_start_ns = time.monotonic_ns()

    parser = argparse.ArgumentParser(
        description="Continuously sweep ADF5355 RFOUTA through SPI0"
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
        "--start-frequency",
        type=rf_output_frequency_arg,
        help="sweep start frequency in Hz",
    )

    parser.add_argument(
        "--end-frequency",
        type=rf_output_frequency_arg,
        help="sweep end frequency in Hz",
    )

    parser.add_argument(
        "--step-frequency",
        type=positive_integer,
        help="sweep step frequency in Hz",
    )

    parser.add_argument(
        "--step-time",
        type=positive_float,
        help="nominal time between sweep commands in seconds",
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
            "wait for active-high digital lock detection on GPIO25 "
            "before each new sweep step"
        ),
    )

    parser.add_argument(
        "--measure-lock-time",
        action="store_true",
        help=(
            "measure final R0 to GPIO25 digital-lock rising-edge "
            "latency for every frequency update"
        ),
    )

    parser.add_argument(
        "--lock-timeout-ms",
        type=positive_float,
        default=20.0,
        help="digital-lock measurement timeout in milliseconds",
    )

    parser.add_argument(
        "--lock-time-verbose",
        action="store_true",
        help="print final-R0-to-digital-lock latency for every update",
    )

    parser.add_argument(
        "--check-muxout",
        action="store_true",
        help="read MUXOUT on GPIO25 after initial programming",
    )

    parser.add_argument(
        "--freq-step-verbose",
        action="store_true",
        help=(
            "print absolute elapsed time, commanded frequency, and "
            "individual frequency-command duration"
        ),
    )

    parser.add_argument(
        "--chirp-time-verbose",
        action="store_true",
        help=(
            "print one-way timing with preparation and command "
            "breakdowns"
        ),
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
            "Register 4 charge-pump-current code: 0 to 15 "
            "(0.3125 to 5.0000 mA)"
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
        "--max-speed-hz",
        type=spi_speed_arg,
        default=DEFAULT_SPI_SPEED_HZ,
        help=(
            "SPI0 clock rate in Hz; valid range is "
            f"{MIN_SPI_SPEED_HZ} to {MAX_SPI_SPEED_HZ} Hz"
        ),
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
        print(DIFFERENTIAL_REFERENCE_WARNING, file=sys.stderr)

    if args.wait_for_lock:
        if args.muxout_lock_detect != MUXOUT_DIGITAL_LOCK_DETECT:
            parser.error(
                "--wait-for-lock requires digital MUXOUT"
            )

        print(MUXOUT_WARNING, file=sys.stderr)

    if args.measure_lock_time:
        if args.wait_for_lock:
            parser.error(
                "--measure-lock-time cannot be combined with "
                "--wait-for-lock"
            )

        if args.muxout_lock_detect != MUXOUT_DIGITAL_LOCK_DETECT:
            parser.error(
                "--measure-lock-time requires digital MUXOUT"
            )

        print(MUXOUT_WARNING, file=sys.stderr)

    if args.lock_time_verbose and not args.measure_lock_time:
        parser.error(
            "--lock-time-verbose requires --measure-lock-time"
        )

    if args.check_muxout:
        print(MUXOUT_WARNING, file=sys.stderr)

    required = {
        "--start-frequency": args.start_frequency,
        "--end-frequency": args.end_frequency,
        "--step-frequency": args.step_frequency,
        "--step-time": args.step_time,
    }

    missing = [
        name
        for name, value in required.items()
        if value is None
    ]

    if missing:
        parser.error(
            "continuous sweep requires: "
            + ", ".join(missing)
        )

    output_power_dbm = (
        DEFAULT_RF_OUTPUT_POWER_DBM
        if args.set_rf_output_power is None
        else args.set_rf_output_power
    )

    enable_rfout_a = not args.disable_rf_output

    try:
        validate_sweep_arguments(
            args.start_frequency,
            args.end_frequency,
            args.step_frequency,
            args.step_time,
            args.reference_hz,
            args.reference_mode,
            args.channel_spacing_hz,
            args.muxout_lock_detect,
            args.mute_till_lock,
        )

        validate_rf_output_power(output_power_dbm)

    except (TypeError, ValueError) as exc:
        parser.error(str(exc))

    try:
        with ADF5355(
            max_speed_hz=args.max_speed_hz,
            verbose=args.verbose,
        ) as device:
            if args.measure_lock_time:
                with DigitalLockMonitor() as lock_monitor:
                    run_sweep(
                        device,
                        args,
                        output_power_dbm,
                        enable_rfout_a,
                        script_start_ns,
                        lock_monitor,
                    )
            else:
                run_sweep(
                    device,
                    args,
                    output_power_dbm,
                    enable_rfout_a,
                    script_start_ns,
                )

    except KeyboardInterrupt:
        print("\nSweep stopped by user.", file=sys.stderr)
        return

    except (TypeError, ValueError, RuntimeError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()

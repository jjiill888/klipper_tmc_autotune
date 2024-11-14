import math
import logging
import os
from enum import Enum
from inspect import signature
from . import tmc

# Autotune config parameters
TUNING_GOAL = 'auto'
EXTRA_HYSTERESIS = 0
TBL = 1
TOFF = 0
SGT = 1
SG4_THRS = 40
VOLTAGE = 24.0
OVERVOLTAGE_VTH = None

# Generic tuning parameters
COOLSTEP_THRS_FACTOR = 0.75
FULLSTEP_THRS_FACTOR = 1.2
MULTISTEP_FILT = True

# PWM parameters
PWM_AUTOSCALE = True  # Setup pwm autoscale even if we won't use PWM, because it
                     # gives more data about the motor and is needed for CoolStep.
PWM_AUTOGRAD = True
PWM_REG = 15
PWM_LIM = 4

# SpreadCycle parameters
TPFD = 0

# CoolStep parameters
FAST_STANDSTILL = True
SMALL_HYSTERESIS = False
SEMIN = 2
SEMAX = 4
SEUP = 3
SEDN = 2
SEIMIN = 1  # If we drop to 1/4 current, high accels don't work right
SFILT = 0
IHOLDDELAY = 12
IRUNDELAY = 0

# High speed parameters
VHIGHFS = False
VHIGHCHM = False  # Even though we are fullstepping, we want SpreadCycle control


TRINAMIC_DRIVERS = ["tmc2130", "tmc2208", "tmc2209", "tmc2240", "tmc2660", "tmc5160"]
PWM_FREQ_TARGETS = {
    "tmc2130": 55e3,
    "tmc2208": 55e3,
    "tmc2209": 55e3,
    "tmc2240": 20e3,  # 2240s run very hot at high frequencies
    "tmc2660": 55e3,
    "tmc5160": 55e3
}

AUTO_PERFORMANCE_MOTORS = {'stepper_x', 'stepper_y', 'stepper_x1', 'stepper_y1', 'stepper_a', 'stepper_b', 'stepper_c'}

class TuningGoal(str, Enum):
    AUTO = "auto"  # This is the default: automatically choose SILENT for Z and PERFORMANCE for X/Y
    AUTOSWITCH = "autoswitch"  # Experimental mode that use StealthChop at low speed and switch to SpreadCycle when needed
    SILENT = "silent"  # StealthChop at all speeds
    PERFORMANCE = "performance"  # SpreadCycle at all speeds

class AutotuneTMC:
    def __init__(self, config):
        self.printer = config.get_printer()

        # Load motor database
        pconfig = self.printer.lookup_object('configfile')
        dirname = os.path.dirname(os.path.realpath(__file__))
        filename = os.path.join(dirname, 'motor_database.cfg')
        try:
            motor_db = pconfig.read_config(filename)
        except Exception:
            raise config.error("Cannot load config '%s'" % (filename,))
        for motor in motor_db.get_prefix_sections(''):
            self.printer.load_object(motor_db, motor.get_name())

        # Now find our stepper and driver in the running Klipper config
        self.name = config.get_name().split(None, 1)[-1]
        if not config.has_section(self.name):
            raise config.error(
                "Could not find stepper config section '[%s]' required by TMC autotuning"
                % (self.name))
        self.tmc_section = None
        for driver in TRINAMIC_DRIVERS:
            driver_name = "%s %s" % (driver, self.name)
            if config.has_section(driver_name):
                self.tmc_section = config.getsection(driver_name)
                self.driver_name = driver_name
                self.driver_type = driver
                break
        if self.tmc_section is None:
            raise config.error(
                "Could not find any TMC driver config section for '%s' required by TMC autotuning"
                % (self.name))
        # TMCtstepHelper may have two signatures, let's pick an implementation
        if 'pstepper' in signature(tmc.TMCtstepHelper).parameters:
            self._set_driver_velocity_field = self._set_driver_velocity_field_new
        else:
            self._set_driver_velocity_field = self._set_driver_velocity_field_old

        # AutotuneTMC config parameters
        self.motor = config.get('motor')
        self.motor_name = "motor_constants " + self.motor
        tgoal = config.get('tuning_goal', default=TUNING_GOAL).lower()
        try:
            self.tuning_goal = TuningGoal(tgoal)
        except ValueError:
            raise config.error(
                "Tuning goal '%s' is invalid for TMC autotuning"
                % (tgoal))
        self.auto_silent = False  # Auto silent off by default
        self.tmc_object = None  # look this up at connect time
        self.tmc_cmdhelper = None  # Ditto
        self.tmc_init_registers = None  # Ditto
        self.run_current = 0.0
        self.fclk = None
        self.motor_object = None
        self.extra_hysteresis = config.getint('extra_hysteresis', default=EXTRA_HYSTERESIS,
                                              minval=0, maxval=8)
        self.tbl = config.getint('tbl', default=TBL, minval=0, maxval=3)
        self.toff = config.getint('toff', default=None, minval=1, maxval=15)
        self.tpfd = config.getint('tpfd', default=None, minval=0, maxval=15)
        self.sgt = config.getint('sgt', default=SGT, minval=-64, maxval=63)
        self.sg4_thrs = config.getint('sg4_thrs', default=SG4_THRS, minval=0, maxval=255)
        self.voltage = config.getfloat('voltage', default=VOLTAGE, minval=0.0, maxval=60.0)
        self.overvoltage_vth = config.getfloat('overvoltage_vth', default=OVERVOLTAGE_VTH,
                                              minval=0.0, maxval=60.0)
        self.pwm_freq_target = config.getfloat('pwm_freq_target',
                                               default=PWM_FREQ_TARGETS[self.driver_type],
                                               minval=10e3, maxval=100e3)
        self.printer.register_event_handler("klippy:connect",
                                            self.handle_connect)
        self.printer.register_event_handler("klippy:ready",
                                            self.handle_ready)
        # Register command
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command("AUTOTUNE_TMC", "STEPPER", self.name,
                                   self.cmd_AUTOTUNE_TMC,
                                   desc=self.cmd_AUTOTUNE_TMC_help)

    def handle_connect(self):
        self.tmc_object = self.printer.lookup_object(self.driver_name)
        self.tmc_cmdhelper = self.tmc_object.get_status.__self__
        try:
            motor = self.printer.lookup_object(self.motor_name)
        except Exception:
            raise self.printer.config_error(
                "Could not find motor definition '[%s]' required by TMC autotuning. "
                "It is not part of the database, please define it in your config!"
                % (self.motor_name))
        if self.tuning_goal == TuningGoal.AUTO:
            self.auto_silent = self.name not in AUTO_PERFORMANCE_MOTORS and motor.T > 0.3
            self.tuning_goal = TuningGoal.SILENT if self.auto_silent else TuningGoal.PERFORMANCE
        self.motor_object = self.printer.lookup_object(self.motor_name)
        #self.tune_driver()

    def handle_ready(self):
        if self.tmc_init_registers is not None:
            self.tmc_init_registers(print_time=print_time)
        try:
            self.fclk = self.tmc_object.mcu_tmc.get_tmc_frequency()
        except AttributeError:
            pass
        if self.fclk is None:
            self.fclk = 12.5e6
        self.tune_driver()

    cmd_AUTOTUNE_TMC_help = "Apply autotuning configuration to TMC stepper driver"
    def cmd_AUTOTUNE_TMC(self, gcmd):
        logging.info("AUTOTUNE_TMC %s", self.name)
        tgoal = gcmd.get('TUNING_GOAL', TUNING_GOAL).lower()
        if tgoal is not None:
            try:
                self.tuning_goal = TuningGoal(tgoal)
            except ValueError:
                pass
            if self.tuning_goal == TuningGoal.AUTO:
                self.tuning_goal = TuningGoal.SILENT if self.auto_silent else TuningGoal.PERFORMANCE
        extra_hysteresis = gcmd.get_int('EXTRA_HYSTERESIS', None)
        if extra_hysteresis is not None:
            if extra_hysteresis >= 0 or extra_hysteresis <= 8:
                self.extra_hysteresis = extra_hysteresis
        self.tune_driver()

    def tune_driver(self):
        # Here we perform the actual tuning adjustments based on the selected tuning goal
        if self.tuning_goal == TuningGoal.SILENT:
            logging.info("Tuning driver for SILENT mode")
            self.set_stealthchop_mode(self.tuning_goal)
            self.adjust_pwm_freq()
            self.smooth_acceleration()

        elif self.tuning_goal == TuningGoal.PERFORMANCE:
            logging.info("Tuning driver for PERFORMANCE mode")
            self.set_stealthchop_mode(self.tuning_goal)
            self.adjust_pwm_freq()
            self.smooth_acceleration()

    def adjust_pwm_freq(self):
        if self.tuning_goal == TuningGoal.SILENT:
            pwm_freq = max(15e3, min(self.pwm_freq_target, 20e3))
        else:
            pwm_freq = self.pwm_freq_target
        self._set_driver_field('pwm_freq', pwm_freq)

    def pwmgrad(self, fclk=20e3, steps=0, volts=24.0):
        if steps == 0:
            steps = self.S
        smooth_factor = 1.3
        return int(math.ceil(self.cbemf * 2 * math.pi * fclk * smooth_factor / (volts * 256.0 * steps)))

    def set_stealthchop_mode(self, goal):
        if goal == TuningGoal.SILENT:
            self._set_driver_field('en_pwm_mode', True)
            self._set_driver_field('en_spreadcycle', False)
        else:
            self._set_driver_field('en_pwm_mode', False)
            self._set_driver_field('en_spreadcycle', True)

    def smooth_acceleration(self):
        self._set_driver_field('semax', 3)
        self._set_driver_field('semin', 1)
        self._set_driver_field('seup', 4)
        self._set_driver_field('sedn', 2)

    def _set_driver_field(self, field, value):
        # Placeholder method for setting driver register field value
        pass

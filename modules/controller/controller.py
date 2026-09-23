from dataclasses import dataclass

from core.base_module import BaseModule


@dataclass
class MotionState:
    surge: float = 0.0
    sway: float = 0.0
    heave: float = 0.0
    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0
    boost: bool = False


# The 5-thruster ROV supports surge / yaw / heave only. LEFT/RIGHT therefore
# turn the vehicle (yaw); sway produces no motor output. PITCH/ROLL command
# slots are kept for future stabilization but are NOT mixed in Phase 1.
COMMANDS = {
    "FORWARD": ("surge", 1.0),
    "BACK": ("surge", -1.0),
    "LEFT": ("yaw", -1.0),
    "RIGHT": ("yaw", 1.0),
    "UP": ("heave", 1.0),
    "DOWN": ("heave", -1.0),
    "TURN_LEFT": ("yaw", -1.0),
    "TURN_RIGHT": ("yaw", 1.0),
    "PITCH_UP": ("pitch", 1.0),
    "PITCH_DOWN": ("pitch", -1.0),
    "ROLL_LEFT": ("roll", -1.0),
    "ROLL_RIGHT": ("roll", 1.0),
}


class ControllerModule(BaseModule):
    """Merges input sources (pad/keyboard/gamepad + link commands) into one motion target."""

    def __init__(self, config=None, telemetry=None, thrusters=None):
        super().__init__("Controller")
        self.config = config or {}
        self.telemetry = telemetry
        self.thrusters = thrusters
        self._inputs = {}
        self._killed = False
        self.last_motion = MotionState()

    def set_input(self, source, motion):
        self._inputs[source] = motion

    def kill(self):
        self._killed = True
        self.logger.warning("KILL received; motion stopped until toggle")

    def recover(self):
        self._killed = False
        self.logger.info("KILL cleared; motion re-enabled")

    def _merge_inputs(self):
        merged = MotionState()
        axes = ["surge", "sway", "heave", "yaw", "pitch", "roll"]
        for axis in axes:
            best = 0.0
            for motion in self._inputs.values():
                value = getattr(motion, axis)
                if abs(value) > abs(best):
                    best = value
            setattr(merged, axis, best)
        merged.boost = any(motion.boost for motion in self._inputs.values())
        return merged

    def _apply_command(self, command):
        tokens = str(command).upper().split()
        if not tokens:
            return
        if tokens[0] == "STOP":
            self._inputs["link"] = MotionState()
            self.logger.info("Command STOP -> all motion zero")
            return
        magnitude = 1.0
        for token in tokens[1:]:
            try:
                magnitude = float(token)
                break
            except ValueError:
                continue
        if tokens[0] in COMMANDS:
            axis, direction = COMMANDS[tokens[0]]
            state = MotionState()
            setattr(state, axis, direction * magnitude)
            self._inputs["link"] = state
            self._killed = False
            self.logger.info("Command %s -> %s %+.2f", tokens[0], axis, direction * magnitude)

    def update(self):
        if not self.running:
            return
        if self.telemetry is not None:
            for command in self.telemetry.consume_commands():
                self._apply_command(command)
        if self._killed:
            motion = MotionState()
        else:
            motion = self._merge_inputs()
        if self.thrusters is not None:
            self.thrusters.set_motion(motion)
        self.last_motion = motion

    def health_check(self):
        return self.thrusters is not None

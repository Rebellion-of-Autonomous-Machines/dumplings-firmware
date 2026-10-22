import inspect
import tkinter as tk
from tkinter import messagebox, ttk

from pymodbus.client import ModbusTcpClient


DRIVE_COUNT = 3

REG_STAT_MODE = 0
REG_STAT_ERROR = 1
REG_LOWER_TIME_MS = 7
REG_RAISE_FULL_TIME_MS = 8
REG_RAISE_BEFORE_SHAKE_MS = 9
REG_SHAKE_UP_TIME_MS = 10
REG_SHAKE_DOWN_TIME_MS = 11
REG_SHAKE_COUNT = 12
REG_VALVE_CONTROL = 13
REG_HEATER_CONTROL = 14
REG_SERVICE_COMMAND = 15
REG_COMMAND_RESULT = 16
REG_PROTOCOL_VERSION = 17
REG_COMMAND_DRIVE = 18
INPUT_UPTIME_HOURS = 0
INPUT_END_BASE = 20
INPUT_RELAY_BASE = 50
INPUT_TEMPERATURE_X10 = 70
INPUT_DALLAS_COUNT = 71
INPUT_FLOAT_SENSOR = 72
INPUT_VALVE_STATUS = 73
INPUT_HEATER_STATUS = 74
INPUT_GLOBAL_STATUS = 75
INPUT_FILL_STATE = 76
INPUT_DRAIN_STATE = 77
INPUT_HEATER_STATE = 78
INPUT_DRIVE_BASE = 80
REGS_PER_DRIVE = 5
MODE_LOWER_BASE = 11
MODE_RAISE_BASE = 14
CMD_SAVE_SETTINGS = 4
CMD_STARTUP_RAISE = 5

CMD_RESULT_TEXT = {
    0: "OK",
    1: "UNKNOWN_CMD",
    2: "BUSY",
    3: "INVALID_ARG",
}

SEQUENCE_TEXT = {
    0: "IDLE",
    1: "LOWERING",
    2: "RAISE_INITIAL",
    3: "SHAKE_UP",
    4: "SHAKE_DOWN",
    5: "RAISE_FINAL",
    6: "DONE",
}

DIRECTION_TEXT = {
    0: "STOP",
    1: "UP",
    2: "DOWN",
}

POSITION_TEXT = {
    0: "UNKNOWN",
    1: "TOP",
    2: "BOTTOM",
}

VALVE_STATE_TEXT = {0: "OPEN", 1: "CLOSED"}
HEATER_STATE_TEXT = {0: "OFF/READY", 1: "ON", 2: "OFF/BLOCKED"}


class App:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Dumplings 3-drive Modbus TCP")
        self.root.geometry("1280x640")
        self.root.minsize(1180, 560)

        self.client = None
        self.connected = False
        self.host_var = tk.StringVar(value="192.168.1.240")
        self.tcp_port_var = tk.StringVar(value="502")
        self.slave_var = tk.StringVar(value="1")
        self.status_var = tk.StringVar(value="Not connected")
        self.trace_var = tk.BooleanVar(value=False)
        self.values_var = tk.StringVar(value="-")

        self.lower_time_var = tk.StringVar(value="3500")
        self.raise_full_time_var = tk.StringVar(value="3500")
        self.raise_before_shake_var = tk.StringVar(value="1750")
        self.shake_up_var = tk.StringVar(value="250")
        self.shake_down_var = tk.StringVar(value="250")
        self.shake_count_var = tk.StringVar(value="8")

        self.timing_entries = {}
        self.drive_status_vars = [
            tk.StringVar(value="-") for _ in range(DRIVE_COUNT)
        ]
        self.valve_status_var = tk.StringVar(value="Fill=0 Drain=0")
        self.heater_status_var = tk.StringVar(value="Heater=0")
        self.lower_buttons = []
        self.raise_buttons = []
        self.raise_plain_buttons = []

        self._build_ui()
        self._tick()

    def _build_ui(self):
        frm = ttk.Frame(self.root, padding=10)
        frm.grid(sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        frm.columnconfigure(0, weight=1)
        frm.columnconfigure(1, weight=1)
        frm.rowconfigure(2, weight=1)

        conn = ttk.LabelFrame(frm, text="Connection", padding=8)
        conn.grid(row=0, column=0, columnspan=2, sticky="ew")
        conn.columnconfigure(1, weight=1)
        ttk.Label(conn, text="IP address:").grid(row=0, column=0, sticky="w")
        ttk.Entry(conn, textvariable=self.host_var, width=28).grid(
            row=0, column=1, sticky="ew"
        )
        ttk.Label(conn, text="TCP port:").grid(
            row=0, column=2, sticky="w", padx=(8, 0)
        )
        ttk.Entry(conn, textvariable=self.tcp_port_var, width=8).grid(
            row=0, column=3, sticky="w"
        )
        ttk.Label(conn, text="Unit ID:").grid(
            row=0, column=4, sticky="w", padx=(8, 0)
        )
        ttk.Entry(conn, textvariable=self.slave_var, width=8).grid(
            row=0, column=5, sticky="w"
        )
        ttk.Checkbutton(conn, text="Trace", variable=self.trace_var).grid(
            row=0, column=6, padx=(8, 0)
        )
        ttk.Button(conn, text="Connect", command=self.connect).grid(
            row=0, column=7, padx=(8, 0)
        )
        ttk.Button(conn, text="Disconnect", command=self.disconnect).grid(
            row=0, column=8, padx=(4, 0)
        )

        timing = ttk.LabelFrame(frm, text="Timings, ms", padding=8)
        timing.grid(row=1, column=0, sticky="nsew", pady=(8, 0), padx=(0, 4))
        fields = [
            ("Lower", self.lower_time_var),
            ("Raise full", self.raise_full_time_var),
            ("Raise before shake", self.raise_before_shake_var),
            ("Shake up", self.shake_up_var),
            ("Shake down", self.shake_down_var),
            ("Shake count", self.shake_count_var),
        ]
        for index, (label, var) in enumerate(fields):
            ttk.Label(timing, text=label + ":").grid(
                row=index // 3,
                column=(index % 3) * 2,
                sticky="w",
                padx=(0, 4),
                pady=3,
            )
            entry = ttk.Entry(timing, textvariable=var, width=10)
            entry.grid(
                row=index // 3,
                column=((index % 3) * 2) + 1,
                sticky="w",
                pady=3,
            )
            self.timing_entries[str(var)] = entry
        ttk.Button(
            timing, text="Write timings", command=self.write_timings
        ).grid(row=2, column=0, pady=(8, 0), sticky="w")
        ttk.Button(
            timing, text="Save to ESP32 memory", command=self.save_settings
        ).grid(row=2, column=1, pady=(8, 0), sticky="w")

        valves_frame = ttk.LabelFrame(frm, text="Valves", padding=8)
        valves_frame.grid(
            row=1, column=1, sticky="nsew", pady=(8, 0), padx=(4, 0)
        )
        ttk.Label(valves_frame, textvariable=self.valve_status_var).grid(
            row=0, column=0, columnspan=4, sticky="w"
        )
        ttk.Button(
            valves_frame,
            text="Fill ON",
            command=lambda: self.set_valve(0, True),
        ).grid(row=1, column=0, padx=(0, 4), pady=(8, 0))
        ttk.Button(
            valves_frame,
            text="Fill OFF",
            command=lambda: self.set_valve(0, False),
        ).grid(row=1, column=1, padx=4, pady=(8, 0))
        ttk.Button(
            valves_frame,
            text="Drain ON",
            command=lambda: self.set_valve(1, True),
        ).grid(row=1, column=2, padx=4, pady=(8, 0))
        ttk.Button(
            valves_frame,
            text="Drain OFF",
            command=lambda: self.set_valve(1, False),
        ).grid(row=1, column=3, padx=4, pady=(8, 0))
        ttk.Label(valves_frame, textvariable=self.heater_status_var).grid(
            row=0, column=4, padx=(20, 0), sticky="w"
        )
        ttk.Button(
            valves_frame,
            text="Heater ON",
            command=lambda: self.set_heater(True),
        ).grid(row=1, column=4, padx=(20, 4), pady=(8, 0))
        ttk.Button(
            valves_frame,
            text="Heater OFF",
            command=lambda: self.set_heater(False),
        ).grid(row=1, column=5, padx=4, pady=(8, 0))

        drives_frame = ttk.LabelFrame(frm, text="Drives", padding=8)
        drives_frame.grid(
            row=2, column=0, columnspan=2, sticky="ew", pady=(8, 0)
        )
        for drive_index in range(DRIVE_COUNT):
            panel = ttk.LabelFrame(
                drives_frame, text=f"Drive {drive_index + 1}", padding=8
            )
            panel.grid(
                row=0,
                column=drive_index,
                sticky="nsew",
                padx=(0 if drive_index == 0 else 8, 0),
            )
            drives_frame.columnconfigure(drive_index, weight=1)
            ttk.Label(
                panel,
                textvariable=self.drive_status_vars[drive_index],
                justify="left",
            ).grid(row=0, column=0, columnspan=3, sticky="w")
            lower_button = ttk.Button(
                panel,
                text="Lower",
                command=lambda idx=drive_index: self.send_drive_command(
                    MODE_LOWER_BASE, idx
                ),
            )
            raise_button = ttk.Button(
                panel,
                text="Raise",
                command=lambda idx=drive_index: self.send_drive_command(
                    MODE_RAISE_BASE, idx
                ),
            )
            raise_plain_button = ttk.Button(
                panel,
                text="Raise no shake",
                command=lambda idx=drive_index: self.raise_without_shake(idx),
            )
            lower_button.grid(row=1, column=0, padx=(0, 4), pady=(8, 0))
            raise_button.grid(row=1, column=1, padx=4, pady=(8, 0))
            raise_plain_button.grid(row=1, column=2, padx=4, pady=(8, 0))
            self.lower_buttons.append(lower_button)
            self.raise_buttons.append(raise_button)
            self.raise_plain_buttons.append(raise_plain_button)

        stat = ttk.LabelFrame(frm, text="Status", padding=8)
        stat.grid(row=3, column=0, columnspan=2, sticky="nsew", pady=(8, 0))
        stat.columnconfigure(0, weight=1)
        ttk.Label(stat, textvariable=self.status_var).grid(
            row=0, column=0, sticky="w"
        )
        ttk.Label(
            stat, textvariable=self.values_var, justify="left", anchor="w"
        ).grid(row=1, column=0, sticky="w")

    def connect(self):
        self.disconnect(update_ui=False)
        try:
            host = self.host_var.get().strip()
            if not host:
                raise RuntimeError("Enter the Ethernet IP address")
            tcp_port = int(self.tcp_port_var.get())
            if not 1 <= tcp_port <= 65535:
                raise ValueError("TCP port must be 1..65535")
            self.client = ModbusTcpClient(
                host=host,
                port=tcp_port,
                timeout=1,
                retries=0,
                trace_packet=self._trace_packet
                if self.trace_var.get()
                else None,
            )
            self.connected = bool(self.client.connect())
            if not self.connected:
                raise ConnectionError(
                    f"Cannot open TCP connection to {host}:{tcp_port}"
                )
            try:
                self.read_regs(REG_STAT_MODE, 2)
            except Exception as exc:
                raise RuntimeError(
                    f"TCP opened, but Modbus did not respond at "
                    f"{host}:{tcp_port}: {exc}"
                ) from exc
            self.status_var.set(f"Modbus connected: {host}:{tcp_port}")
        except Exception as exc:
            self.disconnect(update_ui=False)
            self.status_var.set(f"Error: {exc}")
            self._reset_runtime_ui()

    def disconnect(self, update_ui: bool = True):
        if self.client:
            try:
                self.client.close()
            except Exception:
                pass
        self.client = None
        self.connected = False
        if update_ui:
            self.status_var.set("Disconnected")
            self._reset_runtime_ui()

    def _reset_runtime_ui(self):
        self.values_var.set("-")
        self.valve_status_var.set("Fill=- Drain=-")
        self.heater_status_var.set("Heater=-")
        for drive_index in range(DRIVE_COUNT):
            self.drive_status_vars[drive_index].set("-")
            self.lower_buttons[drive_index].state(["disabled"])
            self.raise_buttons[drive_index].state(["disabled"])
            self.raise_plain_buttons[drive_index].state(["disabled"])

    def _slave(self):
        return int(self.slave_var.get())

    def _call_with_slave(self, fn, *args, **kwargs):
        params = inspect.signature(fn).parameters
        if "slave" in params:
            kwargs["slave"] = self._slave()
        elif "unit" in params:
            kwargs["unit"] = self._slave()
        elif "device_id" in params:
            kwargs["device_id"] = self._slave()
        return fn(*args, **kwargs)

    def _entry_focused(self, var: tk.StringVar) -> bool:
        try:
            return self.root.focus_get() == self.timing_entries.get(str(var))
        except Exception:
            return False

    def _set_if_not_focused(self, var: tk.StringVar, value: int):
        if not self._entry_focused(var):
            var.set(str(value))

    def write_reg(self, address: int, value: int):
        if not self.connected or not self.client:
            raise RuntimeError("Not connected")
        response = self._call_with_slave(
            self.client.write_register, address, int(value) & 0xFFFF
        )
        if response.isError():
            raise RuntimeError(str(response))

    def send_drive_command(self, command: int, drive_index: int):
        try:
            self.write_reg(REG_STAT_MODE, command + drive_index)
        except Exception as exc:
            messagebox.showerror("Modbus", str(exc))

    def raise_without_shake(self, drive_index: int):
        try:
            self.write_reg(REG_COMMAND_DRIVE, drive_index + 1)
            self.write_reg(REG_SERVICE_COMMAND, CMD_STARTUP_RAISE)
        except Exception as exc:
            messagebox.showerror("Modbus", str(exc))

    def write_timings(self):
        try:
            values = [
                (REG_LOWER_TIME_MS, int(self.lower_time_var.get())),
                (REG_RAISE_FULL_TIME_MS, int(self.raise_full_time_var.get())),
                (
                    REG_RAISE_BEFORE_SHAKE_MS,
                    int(self.raise_before_shake_var.get()),
                ),
                (REG_SHAKE_UP_TIME_MS, int(self.shake_up_var.get())),
                (REG_SHAKE_DOWN_TIME_MS, int(self.shake_down_var.get())),
                (REG_SHAKE_COUNT, int(self.shake_count_var.get())),
            ]
            for address, value in values:
                self.write_reg(address, value)
        except Exception as exc:
            messagebox.showerror("Modbus", str(exc))

    def set_valve(self, bit: int, enabled: bool):
        try:
            current = self.read_regs(REG_VALVE_CONTROL, 1)[0]
            if enabled:
                current |= 1 << bit
            else:
                current &= ~(1 << bit)
            self.write_reg(REG_VALVE_CONTROL, current)
        except Exception as exc:
            messagebox.showerror("Modbus", str(exc))

    def set_heater(self, enabled: bool):
        try:
            self.write_reg(REG_HEATER_CONTROL, 1 if enabled else 0)
        except Exception as exc:
            messagebox.showerror("Modbus", str(exc))

    def save_settings(self):
        try:
            self.write_reg(REG_SERVICE_COMMAND, CMD_SAVE_SETTINGS)
        except Exception as exc:
            messagebox.showerror("Modbus", str(exc))

    def poll(self):
        if not self.connected or not self.client:
            return

        holding = [0] * (REG_COMMAND_DRIVE + 1)
        holding[0:2] = self.read_regs(0, 2)
        holding[REG_LOWER_TIME_MS : REG_COMMAND_DRIVE + 1] = self.read_regs(
            REG_LOWER_TIME_MS, REG_COMMAND_DRIVE - REG_LOWER_TIME_MS + 1
        )
        inputs = [0] * (INPUT_DRIVE_BASE + DRIVE_COUNT * REGS_PER_DRIVE)
        inputs[INPUT_UPTIME_HOURS] = self.read_input_regs(
            INPUT_UPTIME_HOURS, 1
        )[0]
        for address, count in (
            (INPUT_END_BASE, DRIVE_COUNT * 2),
            (INPUT_RELAY_BASE, 16),
            (
                INPUT_TEMPERATURE_X10,
                INPUT_HEATER_STATE - INPUT_TEMPERATURE_X10 + 1,
            ),
            (INPUT_DRIVE_BASE, DRIVE_COUNT * REGS_PER_DRIVE),
        ):
            inputs[address : address + count] = self.read_input_regs(
                address, count
            )
        self._update_from_registers(holding, inputs)

    def read_regs(self, address: int, count: int):
        return self._read_regs(
            self.client.read_holding_registers, address, count
        )

    def read_input_regs(self, address: int, count: int):
        return self._read_regs(
            self.client.read_input_registers, address, count
        )

    def _read_regs(self, read_fn, address: int, count: int):
        params = inspect.signature(read_fn).parameters
        kwargs = {}
        if "count" in params:
            kwargs["count"] = count
            response = self._call_with_slave(read_fn, address, **kwargs)
        else:
            response = self._call_with_slave(read_fn, address, count, **kwargs)

        if response.isError():
            raise RuntimeError(str(response))
        return response.registers

    def _trace_packet(self, is_tx: bool, packet: bytes) -> bytes:
        direction = "TX" if is_tx else "RX"
        print(f"{direction}: {packet.hex(' ')}")
        return packet

    def _update_from_registers(self, holding, inputs):
        global_status = inputs[INPUT_GLOBAL_STATUS]
        command_result = holding[REG_COMMAND_RESULT]
        temperature_raw = inputs[INPUT_TEMPERATURE_X10]
        dallas_count = inputs[INPUT_DALLAS_COUNT]
        fill_state = inputs[INPUT_FILL_STATE]
        drain_state = inputs[INPUT_DRAIN_STATE]
        heater_state = inputs[INPUT_HEATER_STATE]
        self.valve_status_var.set(
            f"Fill={VALVE_STATE_TEXT.get(fill_state, 'UNKNOWN')} "
            f"Drain={VALVE_STATE_TEXT.get(drain_state, 'UNKNOWN')}"
        )
        self.heater_status_var.set(
            f"Heater={HEATER_STATE_TEXT.get(heater_state, 'UNKNOWN')}"
        )
        self._set_if_not_focused(
            self.lower_time_var, holding[REG_LOWER_TIME_MS]
        )
        self._set_if_not_focused(
            self.raise_full_time_var, holding[REG_RAISE_FULL_TIME_MS]
        )
        self._set_if_not_focused(
            self.raise_before_shake_var, holding[REG_RAISE_BEFORE_SHAKE_MS]
        )
        self._set_if_not_focused(
            self.shake_up_var, holding[REG_SHAKE_UP_TIME_MS]
        )
        self._set_if_not_focused(
            self.shake_down_var, holding[REG_SHAKE_DOWN_TIME_MS]
        )
        self._set_if_not_focused(
            self.shake_count_var, holding[REG_SHAKE_COUNT]
        )

        for drive_index in range(DRIVE_COUNT):
            base = INPUT_DRIVE_BASE + (drive_index * REGS_PER_DRIVE)
            status = inputs[base + 0]
            sequence = inputs[base + 1]
            direction = inputs[base + 2]
            remaining_ms = inputs[base + 3]
            position = inputs[base + 4]
            busy = bool(status & (1 << 0))
            top = bool(status & (1 << 2))
            bottom = bool(status & (1 << 3))
            up_relay = bool(status & (1 << 4))
            down_relay = bool(status & (1 << 5))
            self.drive_status_vars[drive_index].set(
                f"Seq: {SEQUENCE_TEXT.get(sequence, 'UNKNOWN')}\n"
                f"Dir: {DIRECTION_TEXT.get(direction, 'UNKNOWN')}\n"
                f"Pos: {POSITION_TEXT.get(position, 'UNKNOWN')}\n"
                f"Remaining: {remaining_ms} ms\n"
                f"UpRelay={int(up_relay)} DownRelay={int(down_relay)}"
            )
            mode_ready = holding[REG_STAT_MODE] == 1
            self.lower_buttons[drive_index].state(
                ["disabled"]
                if not mode_ready or busy or bottom
                else ["!disabled"]
            )
            self.raise_buttons[drive_index].state(
                ["disabled"]
                if not mode_ready or busy or top
                else ["!disabled"]
            )
            self.raise_plain_buttons[drive_index].state(
                ["disabled"]
                if not mode_ready or busy or top
                else ["!disabled"]
            )

        temperature_text = (
            "N/A"
            if temperature_raw == 0x8000
            else f"{self._to_i16(temperature_raw) / 10.0:.1f} C"
        )
        self.values_var.set(
            f"Mode={holding[REG_STAT_MODE]} Error={holding[REG_STAT_ERROR]} "
            f"Uptime={inputs[INPUT_UPTIME_HOURS]} h "
            f"Protocol={holding[REG_PROTOCOL_VERSION]}\n"
            f"GlobalStatus=0x{global_status:04X} "
            f"AnyBusy={int(bool(global_status & 1))}\n"
            f"HeaterInterlockOK={int(bool(global_status & (1 << 1)))} "
            f"HeaterBlocked={int(bool(global_status & (1 << 2)))}\n"
            f"WaterLevelOK={inputs[INPUT_FLOAT_SENSOR]} "
            f"VirtualEnds={inputs[INPUT_END_BASE:INPUT_END_BASE + 6]}\n"
            f"CommandResult={command_result} "
            f"({CMD_RESULT_TEXT.get(command_result, 'UNKNOWN')})\n"
            f"Temperature={temperature_text} DallasCount={dallas_count}"
        )

    @staticmethod
    def _to_i16(value: int) -> int:
        return value - 65536 if value > 32767 else value

    def _tick(self):
        try:
            self.poll()
        except Exception as exc:
            self.disconnect(update_ui=False)
            self._reset_runtime_ui()
            self.status_var.set(f"Error: {exc}")
        self.root.after(500, self._tick)


if __name__ == "__main__":
    root = tk.Tk()
    App(root)
    root.mainloop()

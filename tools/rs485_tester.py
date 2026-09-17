import inspect
import tkinter as tk
from tkinter import messagebox, ttk

from pymodbus.client import ModbusSerialClient
from serial.tools import list_ports


DRIVE_COUNT = 3

REG_GLOBAL_STATUS = 0
REG_COMMAND = 1
REG_COMMAND_DRIVE = 2
REG_COMMAND_RESULT = 3
REG_LOWER_TIME_MS = 4
REG_RAISE_FULL_TIME_MS = 5
REG_RAISE_BEFORE_SHAKE_MS = 6
REG_SHAKE_UP_TIME_MS = 7
REG_SHAKE_DOWN_TIME_MS = 8
REG_SHAKE_COUNT = 9
REG_DRIVE_BASE = 10
REGS_PER_DRIVE = 5
REG_VALVE_STATUS = 25
REG_VALVE_CONTROL = 26
REG_TEMPERATURE_X10 = 27
REG_DALLAS_COUNT = 28
REG_HEATER_STATUS = 29
REG_HEATER_CONTROL = 30

CMD_LOWER = 1
CMD_RAISE = 2
CMD_STOP = 3
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


class App:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Dumplings 3-drive RS-485 Modbus")
        self.root.geometry("1180x640")
        self.root.minsize(1100, 560)

        self.client = None
        self.connected = False
        self.port_display_to_device = {}

        self.port_var = tk.StringVar(value="")
        self.baud_var = tk.StringVar(value="9600")
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

        self.port_combo = None
        self.timing_entries = {}
        self.drive_status_vars = [tk.StringVar(value="-") for _ in range(DRIVE_COUNT)]
        self.valve_status_var = tk.StringVar(value="Fill=0 Drain=0")
        self.heater_status_var = tk.StringVar(value="Heater=0")
        self.lower_buttons = []
        self.raise_buttons = []
        self.stop_buttons = []

        self._build_ui()
        self.refresh_ports()
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
        ttk.Label(conn, text="COM:").grid(row=0, column=0, sticky="w")
        self.port_combo = ttk.Combobox(conn, textvariable=self.port_var, width=64, state="readonly")
        self.port_combo.grid(row=0, column=1, sticky="ew")
        ttk.Button(conn, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=(6, 0))
        ttk.Label(conn, text="Baud:").grid(row=0, column=3, sticky="w", padx=(8, 0))
        ttk.Entry(conn, textvariable=self.baud_var, width=10).grid(row=0, column=4, sticky="w")
        ttk.Label(conn, text="Slave ID:").grid(row=0, column=5, sticky="w", padx=(8, 0))
        ttk.Entry(conn, textvariable=self.slave_var, width=8).grid(row=0, column=6, sticky="w")
        ttk.Checkbutton(conn, text="Trace", variable=self.trace_var).grid(row=0, column=7, padx=(8, 0))
        ttk.Button(conn, text="Connect", command=self.connect).grid(row=0, column=8, padx=(8, 0))
        ttk.Button(conn, text="Disconnect", command=self.disconnect).grid(row=0, column=9, padx=(4, 0))

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
            ttk.Label(timing, text=label + ":").grid(row=index // 3, column=(index % 3) * 2, sticky="w", padx=(0, 4), pady=3)
            entry = ttk.Entry(timing, textvariable=var, width=10)
            entry.grid(row=index // 3, column=((index % 3) * 2) + 1, sticky="w", pady=3)
            self.timing_entries[str(var)] = entry
        ttk.Button(timing, text="Write timings", command=self.write_timings).grid(row=2, column=0, pady=(8, 0), sticky="w")
        ttk.Button(timing, text="Save to ESP32 memory", command=self.save_settings).grid(row=2, column=1, pady=(8, 0), sticky="w")

        valves_frame = ttk.LabelFrame(frm, text="Valves", padding=8)
        valves_frame.grid(row=1, column=1, sticky="nsew", pady=(8, 0), padx=(4, 0))
        ttk.Label(valves_frame, textvariable=self.valve_status_var).grid(row=0, column=0, columnspan=4, sticky="w")
        ttk.Button(valves_frame, text="Fill ON", command=lambda: self.set_valve(0, True)).grid(row=1, column=0, padx=(0, 4), pady=(8, 0))
        ttk.Button(valves_frame, text="Fill OFF", command=lambda: self.set_valve(0, False)).grid(row=1, column=1, padx=4, pady=(8, 0))
        ttk.Button(valves_frame, text="Drain ON", command=lambda: self.set_valve(1, True)).grid(row=1, column=2, padx=4, pady=(8, 0))
        ttk.Button(valves_frame, text="Drain OFF", command=lambda: self.set_valve(1, False)).grid(row=1, column=3, padx=4, pady=(8, 0))
        ttk.Label(valves_frame, textvariable=self.heater_status_var).grid(row=0, column=4, padx=(20, 0), sticky="w")
        ttk.Button(valves_frame, text="Heater ON", command=lambda: self.set_heater(True)).grid(row=1, column=4, padx=(20, 4), pady=(8, 0))
        ttk.Button(valves_frame, text="Heater OFF", command=lambda: self.set_heater(False)).grid(row=1, column=5, padx=4, pady=(8, 0))

        drives_frame = ttk.LabelFrame(frm, text="Drives", padding=8)
        drives_frame.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        for drive_index in range(DRIVE_COUNT):
            panel = ttk.LabelFrame(drives_frame, text=f"Drive {drive_index + 1}", padding=8)
            panel.grid(row=0, column=drive_index, sticky="nsew", padx=(0 if drive_index == 0 else 8, 0))
            drives_frame.columnconfigure(drive_index, weight=1)
            ttk.Label(panel, textvariable=self.drive_status_vars[drive_index], justify="left").grid(row=0, column=0, columnspan=3, sticky="w")
            lower_button = ttk.Button(panel, text="Lower", command=lambda idx=drive_index: self.send_drive_command(CMD_LOWER, idx))
            raise_button = ttk.Button(panel, text="Raise", command=lambda idx=drive_index: self.send_drive_command(CMD_RAISE, idx))
            stop_button = ttk.Button(panel, text="Stop", command=lambda idx=drive_index: self.send_drive_command(CMD_STOP, idx))
            lower_button.grid(row=1, column=0, padx=(0, 4), pady=(8, 0))
            raise_button.grid(row=1, column=1, padx=4, pady=(8, 0))
            stop_button.grid(row=1, column=2, padx=4, pady=(8, 0))
            self.lower_buttons.append(lower_button)
            self.raise_buttons.append(raise_button)
            self.stop_buttons.append(stop_button)

        stat = ttk.LabelFrame(frm, text="Status", padding=8)
        stat.grid(row=3, column=0, columnspan=2, sticky="nsew", pady=(8, 0))
        stat.columnconfigure(0, weight=1)
        ttk.Label(stat, textvariable=self.status_var).grid(row=0, column=0, sticky="w")
        ttk.Label(stat, textvariable=self.values_var, justify="left", anchor="w").grid(row=1, column=0, sticky="w")

    def refresh_ports(self):
        previous_device = self._selected_port_device()
        self.port_display_to_device = {}
        display_values = []

        for port in sorted(list_ports.comports(), key=lambda item: item.device):
            display = f"{port.device} - {port.description}"
            if port.manufacturer:
                display += f" ({port.manufacturer})"
            self.port_display_to_device[display] = port.device
            display_values.append(display)

        if self.port_combo is not None:
            self.port_combo["values"] = display_values

        if not display_values:
            self.port_var.set("")
            self.status_var.set("No COM ports found")
            return

        selected_display = None
        if previous_device:
            for display, device in self.port_display_to_device.items():
                if device == previous_device:
                    selected_display = display
                    break
        self.port_var.set(selected_display or display_values[0])

    def _selected_port_device(self):
        selected = self.port_var.get().strip()
        if not selected:
            return ""
        return self.port_display_to_device.get(selected, selected.split(" - ", 1)[0])

    def connect(self):
        self.disconnect(update_ui=False)
        try:
            port = self._selected_port_device()
            if not port:
                raise RuntimeError("No COM port selected")
            self.client = ModbusSerialClient(
                port=port,
                baudrate=int(self.baud_var.get()),
                bytesize=8,
                parity="N",
                stopbits=1,
                timeout=1,
                trace_packet=self._trace_packet if self.trace_var.get() else None,
            )
            self.connected = bool(self.client.connect())
            self.status_var.set("Connected" if self.connected else "Connection error")
            if not self.connected:
                self._reset_runtime_ui()
        except Exception as exc:
            self.connected = False
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
            self.stop_buttons[drive_index].state(["disabled"])

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
        response = self._call_with_slave(self.client.write_register, address, int(value) & 0xFFFF)
        if response.isError():
            raise RuntimeError(str(response))

    def send_drive_command(self, command: int, drive_index: int):
        try:
            self.write_reg(REG_COMMAND_DRIVE, drive_index + 1)
            self.write_reg(REG_COMMAND, command)
        except Exception as exc:
            messagebox.showerror("Modbus", str(exc))

    def write_timings(self):
        try:
            values = [
                (REG_LOWER_TIME_MS, int(self.lower_time_var.get())),
                (REG_RAISE_FULL_TIME_MS, int(self.raise_full_time_var.get())),
                (REG_RAISE_BEFORE_SHAKE_MS, int(self.raise_before_shake_var.get())),
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
            self.write_timings()
            self.write_reg(REG_COMMAND, CMD_SAVE_SETTINGS)
        except Exception as exc:
            messagebox.showerror("Modbus", str(exc))

    def poll(self):
        if not self.connected or not self.client:
            return

        regs = [0] * (REG_HEATER_CONTROL + 1)
        for address, values in (
            (0, self.read_regs(0, 10)),
            (REG_DRIVE_BASE, self.read_regs(REG_DRIVE_BASE, DRIVE_COUNT * REGS_PER_DRIVE)),
            (REG_VALVE_STATUS, self.read_regs(REG_VALVE_STATUS, 6)),
        ):
            regs[address:address + len(values)] = values
        self._update_from_registers(regs)

    def read_regs(self, address: int, count: int):
        read_fn = self.client.read_holding_registers
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

    def _update_from_registers(self, regs):
        global_status = regs[REG_GLOBAL_STATUS]
        command_result = regs[REG_COMMAND_RESULT]
        valve_status = regs[REG_VALVE_STATUS]
        temperature_raw = regs[REG_TEMPERATURE_X10]
        dallas_count = regs[REG_DALLAS_COUNT]
        heater_on = bool(regs[REG_HEATER_STATUS])
        fill_on = bool(valve_status & (1 << 0))
        drain_on = bool(valve_status & (1 << 1))
        self.valve_status_var.set(f"Fill={int(fill_on)} Drain={int(drain_on)}")
        self.heater_status_var.set(f"Heater={int(heater_on)}")
        self._set_if_not_focused(self.lower_time_var, regs[REG_LOWER_TIME_MS])
        self._set_if_not_focused(self.raise_full_time_var, regs[REG_RAISE_FULL_TIME_MS])
        self._set_if_not_focused(self.raise_before_shake_var, regs[REG_RAISE_BEFORE_SHAKE_MS])
        self._set_if_not_focused(self.shake_up_var, regs[REG_SHAKE_UP_TIME_MS])
        self._set_if_not_focused(self.shake_down_var, regs[REG_SHAKE_DOWN_TIME_MS])
        self._set_if_not_focused(self.shake_count_var, regs[REG_SHAKE_COUNT])

        for drive_index in range(DRIVE_COUNT):
            base = REG_DRIVE_BASE + (drive_index * REGS_PER_DRIVE)
            status = regs[base + 0]
            sequence = regs[base + 1]
            direction = regs[base + 2]
            remaining_ms = regs[base + 3]
            position = regs[base + 4]
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
            self.lower_buttons[drive_index].state(["disabled"] if busy or bottom else ["!disabled"])
            self.raise_buttons[drive_index].state(["disabled"] if busy or top else ["!disabled"])
            self.stop_buttons[drive_index].state(["!disabled"] if busy else ["disabled"])

        temperature_text = "N/A" if temperature_raw == 0x8000 else f"{self._to_i16(temperature_raw) / 10.0:.1f} C"
        self.values_var.set(
            f"GlobalStatus=0x{global_status:04X} AnyBusy={int(bool(global_status & 1))}\n"
            f"HeaterInterlockOK={int(bool(global_status & (1 << 1)))} "
            f"HeaterBlocked={int(bool(global_status & (1 << 2)))}\n"
            f"CommandResult={command_result} ({CMD_RESULT_TEXT.get(command_result, 'UNKNOWN')})\n"
            f"Temperature={temperature_text} DallasCount={dallas_count}"
        )

    @staticmethod
    def _to_i16(value: int) -> int:
        return value - 65536 if value > 32767 else value

    def _tick(self):
        try:
            self.poll()
        except Exception as exc:
            self.status_var.set(f"Error: {exc}")
        self.root.after(500, self._tick)


if __name__ == "__main__":
    root = tk.Tk()
    App(root)
    root.mainloop()

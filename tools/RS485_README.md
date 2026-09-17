# RS-485 / Modbus register map

Slave parameters:
- UART: `Serial2`
- RXD: `GPIO16`
- TXD: `GPIO13`
- DE/RE direction pin: not used by firmware (`-1`). The RS-485 transceiver must have auto-direction, or DE/RE must be wired for permanent receive/transmit as required by the board.
- Baudrate: `9600 8N1`
- Slave ID: `1`

Relay mapping:
- Drive 1: channel `1` up, channel `2` down
- Drive 2: channel `3` up, channel `4` down
- Drive 3: channel `5` up, channel `6` down
- Fill valve: channel `9` (first channel on second relay expander)
- Drain valve: channel `10` (second channel on second relay expander)
- Three-phase heater contactors: channels `11`, `12`, `13`
- Channels are active-low: enabled channel is pulled to ground by PCF8574.
- Firmware reverses the board output order, because the hardware is wired as `16..1`.
- Expander `0x24`: channels `1..8`
- Expander `0x25`: channels `9..16`

Input mapping:
- Input expander: `0x22`
- Float sensor: physical `P0` terminal, firmware reads PCF bit `P7` because the connector order is reversed
- Active-low: closed contact to ground means water is at the required level
- Open contact means water level is insufficient
- Current firmware only displays this input in the web interface.

Temperature:
- Dallas / DS18B20 data pin: `GPIO32`
- Current firmware displays water temperature in the web interface.

Holding registers:
- `0` `RO` `GLOBAL_STATUS`
  - bit0: any drive busy
  - bit1: heater interlock OK (water level is OK and both valves are closed)
  - bit2: heater blocked by interlock
- `1` `RW` `COMMAND` (firmware resets it to `0` after processing)
- `2` `RW` `COMMAND_DRIVE`
  - `1..3`: selected drive
  - `0`: all drives for `stop`
- `3` `RO` `COMMAND_RESULT`
  - `0` ok
  - `1` unknown command
  - `2` busy
  - `3` invalid argument
- `4` `RW` `LOWER_TIME_MS`
- `5` `RW` `RAISE_FULL_TIME_MS`
- `6` `RW` `RAISE_BEFORE_SHAKE_MS`
- `7` `RW` `SHAKE_UP_TIME_MS`
- `8` `RW` `SHAKE_DOWN_TIME_MS`
- `9` `RW` `SHAKE_COUNT`
- `25` `RO` `VALVE_STATUS`
  - bit0: fill valve on
  - bit1: drain valve on
- `26` `RW` `VALVE_CONTROL`
  - bit0: fill valve on/off
  - bit1: drain valve on/off
- `27` `RO` `TEMPERATURE_X10`
  - int16, Celsius * 10
  - `0x8000`: no Dallas data
- `28` `RO` `DALLAS_COUNT`
- `29` `RO` `HEATER_STATUS`
  - `0`: heater off
  - `1`: three phase contactor relays on
- `30` `RW` `HEATER_CONTROL`
  - `0`: heater off
  - `1`: turn on channels `11`, `12`, `13`
  - Write `1` is ignored while `GLOBAL_STATUS.bit1` is `0`.

Per-drive registers start at `10`, five registers per drive:
- Drive 1 base: `10`
- Drive 2 base: `15`
- Drive 3 base: `20`

Per-drive layout:
- `base + 0` `RO` `DRIVE_STATUS`
  - bit0: busy
  - bit1: done
  - bit2: position top
  - bit3: position bottom
  - bit4: up relay on
  - bit5: down relay on
- `base + 1` `RO` `SEQUENCE`
  - `0` idle
  - `1` lowering
  - `2` raise initial
  - `3` shake up
  - `4` shake down
  - `5` raise final
  - `6` done
- `base + 2` `RO` `DIRECTION`
  - `0` stop
  - `1` up
  - `2` down
- `base + 3` `RO` `REMAINING_MS`
- `base + 4` `RO` `POSITION`
  - `0` unknown
  - `1` top
  - `2` bottom

Commands (`reg 1`):
- `1` lower selected drive
- `2` raise selected drive with shake
- `3` stop selected drive, or all drives if `COMMAND_DRIVE=0`
- `4` save timing settings to ESP32 NVS memory
- `5` startup-style raise selected drive without shake

Default timing values:
- `LOWER_TIME_MS`: `3500`
- `RAISE_FULL_TIME_MS`: `3500`
- `RAISE_BEFORE_SHAKE_MS`: `1750`
- `SHAKE_UP_TIME_MS`: `250`
- `SHAKE_DOWN_TIME_MS`: `250`
- `SHAKE_COUNT`: `8`

Notes:
- On ESP32 startup all three drives raise for `RAISE_FULL_TIME_MS` without shake and then mark position as `top`.
- The heater can turn on only when the float sensor is active and both fill/drain valves are closed.
- If water level disappears or either valve opens while the heater is on, firmware turns the heater off automatically.
- `lower` is rejected when selected drive is already `bottom`.
- `raise` is rejected when selected drive is already `top`.
- `stop` marks selected drive position as `unknown`.
- Timing registers are applied to RAM only when all drives are idle.
- Use command `4` after writing timing registers to keep them after ESP32 restart.
- `RAISE_BEFORE_SHAKE_MS` must be less than or equal to `RAISE_FULL_TIME_MS`.

## Tkinter tester

File: `tools/rs485_tester.py`

Install dependencies:
```bash
pip install pymodbus pyserial
```

Run:
```bash
python tools/rs485_tester.py
```

Troubleshooting:
- Open the ESP32 USB monitor at `115200`; firmware prints the active Modbus pins on startup.
- Enable `Trace` in the tester before pressing `Connect`; the terminal prints Modbus `TX` and `RX` bytes.
- If `TX` appears but there is no `RX`, check `A/B` polarity, common `GND`, slave ID `1`, baudrate `9600`, and whether the RS-485 transceiver needs a DE/RE control pin.
- The tester reads registers in small blocks, so a failed poll usually means physical UART/RS-485 wiring, wrong COM port, or wrong slave settings.

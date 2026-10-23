# Dual-Leg Shunt Power Monitor

A voltage, current and power monitor for a single-phase, non-polarized load (AC or bidirectional DC). Each of the two load terminals has its own shunt and a pair of ADC taps, giving four ADC channels in total. Every tap is level-shifted to mid-rail by a resistor divider and read directly by an MCU ADC, so no amplifier IC is needed. Firmware computes load voltage and current, real/apparent/reactive power, power factor, and the imbalance between the two legs.

> **Scope:** low-voltage circuits only (about ±25 V per tap at the default scaling). The front end is not isolated and the MCU ground is tied to a reference point of the circuit. Not suitable for mains. For mains, use isolated sensing (current transformer, Hall sensor or isolated amplifier, plus a voltage transformer).

## Circuit

### Power path

```
                    Rshunt A
 SOURCE line 1 o----+--/\/\/\--+----o LOAD terminal A
                    |          |           |
                 TAP SA     TAP LA       [ LOAD ]
                                           |
                 TAP SB     TAP LB         |
                    |          |           |
 SOURCE line 2 o----+--/\/\/\--+----o LOAD terminal B
                    Rshunt B
```

- Each leg has a shunt with one tap on the source side (`SA`, `SB`) and one on the load side (`LA`, `LB`).
- On each leg, current flowing from the source side to the load side is positive. In a healthy loop the current enters the load through one terminal and returns through the other, so `I_A` is approximately `-I_B`.
- Connect the taps directly at the shunt terminals (Kelvin style) so lead and trace resistance does not add to the shunt value.
- Connect the ESP32 GND to one point of the source (a source terminal, a bus negative, or a midpoint) chosen so that all four taps stay inside the input range.

### Sensing network (identical for all four taps)

```
                                 3.3 V
                                   |
                                [Rup 10k]
                                   |
 TAP --[Rs 150k]------------------+------> ADC pin
                                   |
                           +-------+-------+
                           |               |
                        [Rdn 10k]       [C 10 nF]
                           |               |
                          GND             GND
```

- `Rs` attenuates the tap voltage; `Rup` and `Rdn` bias the node at about mid-rail so both polarities fit inside the ADC range.
- `C` is a small anti-alias filter. All channels use the same value, so voltage and current see the same phase shift and the power factor is not distorted.
- `Rs` also limits the fault current into the ESP32 pin if a tap is overdriven.

## How it works

Each ADC node follows a linear relation with its tap voltage:

```
V_adc = b + k * V_tap

k = (1/Rs) / (1/Rs + 1/Rup + 1/Rdn)
b = 3.3 V * (1/Rup) / (1/Rs + 1/Rup + 1/Rdn)
```

With identical networks the bias cancels in every difference:

```
I_A    = (V_SA - V_LA) / Rshunt_A          (tap voltages, source side minus load side)
I_B    = (V_SB - V_LB) / Rshunt_B
V_load = V_LA - V_LB
I_load = (I_A - I_B) / 2
I_leak = I_A + I_B                         (imbalance between legs, ideally 0)
P      = mean(V_load * I_load)             over a whole number of cycles
S      = Vrms * Irms
Q      = sqrt(S^2 - P^2)
PF     = P / S
```

The MCU e.g. ESP32 reads channels one after another, which would add a large error to every difference because the common-mode signal is much larger than a shunt drop. To avoid this, firmware samples in a symmetric order, `SA, LA, SB, LB, SB, LA, SA`, and averages the two readings of each repeated channel. Every channel is then estimated at the same instant, the moment of the single `LB` reading.

## Default scaling

| Parameter | Value |
|---|---|
| Rs / Rup / Rdn | 150 k / 10 k / 10 k |
| Attenuation k | about 0.0323 |
| Bias b | about 1.60 V |
| Usable tap range (ADC linear region about 0.15-2.45 V) | about ±25 V |
| Rshunt (each leg) | 0.05 ohm (adjust to the load; see limitations) |

For a different voltage range, change `Rs`: a larger value gives a wider range and a lower resolution. Firmware recomputes `k` and `b` from the resistor values at the top of the sketch.

## Bill of materials (through-hole)

| Position | Part | Notes |
|---|---|---|
| Rs (x4) | 150 k, 0.1%, metal film, 1/4 W | Must be matched across all four networks |
| Rup (x4), Rdn (x4) | 10 k, 0.1%, metal film, 1/4 W | Match within and between networks |
| Rshunt (x2) | 0.05 ohm, axial wirewound or metal film, 5 W | Matched pair; power rating at least 2x I²R of the maximum load current |
| C (x4) | 10 nF film or C0G, radial | Same type for all channels |
| Decoupling | 100 nF ceramic, radial | At the 3.3 V pin near the dividers |
| MCU | ESP32 dev board | Use ADC1 pins (GPIO32-39); ADC2 conflicts with Wi-Fi |

## Wiring

| Signal | ESP32 pin |
|---|---|
| TAP SA network output | GPIO34 (ADC1, input only) |
| TAP LA network output | GPIO35 (ADC1, input only) |
| TAP SB network output | GPIO32 (ADC1) |
| TAP LB network output | GPIO33 (ADC1) |
| Reference point | GND |
| Bias divider supply | 3V3 |

When powering the board from USB while measuring with grounded equipment, use a USB isolator or a battery to avoid ground loops.

## Firmware

[`firmware.ino`](firmware.ino) is a single-file sketch for the Arduino-ESP32 core. It uses no external libraries: only `Arduino.h` and standard C headers (`math.h`, `stdlib.h`, `string.h`, `strings.h`) that ship with the ESP32 toolchain. All settings are variables at the top (pins, resistor values, shunt values, window length, per-leg common-mode trims, mode) and can be overridden at run time over the serial port. Two modes are available:

- `measure`: prints Vrms, Irms, P, S, Q, PF, leg imbalance and the DC components once per window (default 200 ms, an integer number of cycles at both 50 Hz and 60 Hz).
- `calibrate`: prints the residual common-mode coupling of each leg and a suggested trim value.

The Arduino IDE requires the sketch to sit in a folder with the same name as the `.ino` file.

## Serial commands

Open the serial monitor at 115200 baud with line ending set to newline (or CR/LF). Commands are case-insensitive and are handled between measurement windows, so a change applies from the next window.

| Command | Effect |
|---|---|
| `help` | List commands and variables |
| `get [name]` | Show all variables, or one |
| `set <name> <value>` | Override a variable. `<name> <value>` and `<name>=<value>` also work; a bare `<name>` shows its value |
| `zero` | Measure the zero offsets now (source must be de-energized) |
| `zero calc` | Use the calculated bias instead of a measured zero |
| `read` | Run and print one window (useful with `stream off`) |
| `reboot` | Restart the ESP32 |

| Variable | Meaning | Accepted range |
|---|---|---|
| `rs`, `rup`, `rdn` | Sensing network resistors (ohm) | 1 to 1e7 |
| `vref` | Actual 3V3 rail (mV) | 2500 to 3600 |
| `rshunt_a`, `rshunt_b` | Shunt values (ohm) | 0.0001 to 100 |
| `trim_a`, `trim_b` | Common-mode trim (A per V) | -10 to 10 |
| `min_cal_v` | Minimum tap Vrms for a trim estimate (V) | 0 to 1000 |
| `window_ms` | Integration window (ms) | 10 to 10000 |
| `zero_samples` | Samples used by `zero` | 1 to 100000 |
| `pin_sa`, `pin_la`, `pin_sb`, `pin_lb` | ADC1 GPIO of each tap | 32 to 39, no duplicates |
| `mode` | `measure` or `calibrate` | |
| `stream` | `on`/`off`: print every window | |

Examples:

```
set rshunt_a 0.051
trim_b=-0.0012
mode calibrate
get
```

Notes:

- Overrides live in RAM only; after a reset the defaults compiled into the sketch apply again. Copy final values back into the sketch to keep them.
- Changing `rs`, `rup`, `rdn` or `vref` recomputes `k` and `b` and resets the zero offsets to the calculated bias; run `zero` again if you want a measured zero.
- `AUTO_ZERO_AT_BOOT` stays a compile-time constant because it only acts at boot; use the `zero` command to re-zero at any time.

## Calibration

1. **Zero offset.** With the source de-energized (all four taps at 0 V), boot the board with `AUTO_ZERO_AT_BOOT` enabled, or send `zero` at any time. Firmware averages each node and stores its own offset. If the source cannot be de-energized, send `zero calc` (or set the flag to `false`) to use the calculated bias.
2. **Common-mode trim.** Resistor mismatch converts part of a tap voltage into an apparent current. Disconnect the load (no current through either shunt), energize the source, send `mode calibrate`, and apply the suggested values with `set trim_a <value>` and `set trim_b <value>`. A leg whose taps sit at about 0 V (for example, the leg tied to GND) needs no trim and is reported as such. Then send `mode measure`.
3. **Verify** against a reference meter with a known load, then adjust the shunt values (`set rshunt_a ...`, `set rshunt_b ...`) if needed. Copy the final values into the sketch to make them permanent.

## Limitations

- Current is derived from a small difference between two attenuated signals, so resolution is limited. At the default scaling one ADC step on a tap corresponds to roughly 25 mV across a shunt. Averaging over a window improves this, but the shunt drop at full load should still be a few hundred millivolts or more. This suits loads of a few amps; for higher current or better accuracy, use a current-sense amplifier or an external ADC.
- Resistor matching matters for current. A mismatch of 0.1% between the two networks of a leg turns 25 V of tap voltage into about 25 mV of apparent shunt voltage. Use 0.1% parts and perform the common-mode trim. Load voltage is far less sensitive, since it is compared against a signal of similar size.
- The imbalance reading (`I_A + I_B`) is only indicative; it carries the error of both legs.
- ESP32 ADC linearity is limited and varies between chips. Firmware uses the factory-calibrated `analogReadMilliVolts()`, but readings should still be verified.
- Seven ADC reads per sample give an effective rate of roughly 10 kHz: suitable for 50/60 Hz and low harmonic content, not for fast PWM waveforms.
- Voltages are measured relative to the ESP32 GND, so the reference point must be chosen deliberately.
- No isolation, no overcurrent protection and no energy accumulation.

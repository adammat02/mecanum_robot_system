# Mecanum Robot System

> **Prototype** — running on [NUCLEO-L476RG](https://www.st.com/en/evaluation-tools/nucleo-l476rg.html) (STM32L476RGT6, Cortex-M4 @ 80 MHz).

4-wheel mecanum drive controller with quadrature encoders, closed-loop PID speed control, battery monitoring and a VL53L1X time-of-flight distance sensor. Built with STM32 HAL and ARM CMSIS-DSP.

---

## Table of Contents

1. [System Clock](#system-clock)
2. [Timer Configuration](#timer-configuration)
3. [Motor Control](#motor-control)
4. [Encoders](#encoders)
5. [PID Speed Controller](#pid-speed-controller)
6. [Battery Monitoring (ADC)](#battery-monitoring-adc)
7. [Time-of-Flight Sensor](#time-of-flight-sensor)
8. [UART Communication](#uart-communication)
9. [Serial Command Protocol](#serial-command-protocol)
10. [Software Timing](#software-timing)
11. [Pin Map](#pin-map)
12. [Project Structure](#project-structure)

---

## System Clock

Clock source: **HSI** internal oscillator (16 MHz) fed through the main PLL.

```
HSI (16 MHz)
  └─ PLL  PLLM=1  PLLN=10  PLLR=2
       VCO = 16/1 × 10 = 160 MHz
       SYSCLK = 160 / 2 = 80 MHz
         ├─ AHB  (HCLK)  ÷1 → 80 MHz
         ├─ APB1 (PCLK1) ÷1 → 80 MHz   (timer clock 80 MHz)
         └─ APB2 (PCLK2) ÷1 → 80 MHz   (timer clock 80 MHz)
```

Voltage regulator: `PWR_REGULATOR_VOLTAGE_SCALE1` (maximum performance).
Flash latency: 4 wait states.

---

## Timer Configuration

Formula reference:

```
f_tick   = f_clk  / (PSC + 1)
f_update = f_tick / (ARR + 1)
T_update = 1 / f_update
```

| Timer | Function | Bus / f_clk | PSC | ARR | f_tick | f_update / PWM | Period |
|-------|----------|------------|-----|-----|--------|----------------|--------|
| TIM1 | Encoder – Motor 1 | APB1 / 80 MHz | 0 | 65 535 (16-bit) | 80 MHz | — encoder mode | — |
| TIM2 | Encoder – Motor 2 | APB1 / 80 MHz | 0 | 4 294 967 295 (32-bit) | 80 MHz | — encoder mode | — |
| TIM3 | Encoder – Motor 3 | APB1 / 80 MHz | 0 | 65 535 (16-bit) | 80 MHz | — encoder mode | — |
| TIM4 | Encoder – Motor 4 | APB1 / 80 MHz | 0 | 65 535 (16-bit) | 80 MHz | — encoder mode | — |
| TIM5 | Microsecond timebase | APB1 / 80 MHz | **79** | 4 294 967 295 (32-bit) | **1 MHz** | free-running | **1 µs / tick** |
| TIM6 | ADC trigger | APB1 / 80 MHz | **7 999** | **999** | 10 kHz | **10 Hz** | **100 ms** |
| TIM8 | Motor PWM ×4 | APB2 / 80 MHz | **19** | **255** | 4 MHz | **15 625 Hz** | **64 µs** |

Defines from `Core/Inc/main.h`:

```c
#define TIM8_PSC  19
#define TIM8_ARR  255
#define TIM5_PSC  79
#define TIM6_PSC  7999
#define TIM6_ARR  999
```

### Notes

- **TIM1–TIM4** run in quadrature encoder mode (`TIM_ENCODERMODE_TI1`). The counter increments/decrements on encoder pulses — PSC and ARR are irrelevant for frequency; ARR only sets the counter wrap point.
- **TIM2** uses a 32-bit counter (max ARR) to avoid overflow at higher speeds.
- **TIM5** runs free; `micros()` reads its 32-bit counter directly — wraps after ~71 minutes.
- **TIM6** fires a TRGO (update event) that triggers ADC1 at exactly 10 Hz.
- **TIM8** is an advanced timer on APB2; APB2 prescaler = 1, so its timer clock is also 80 MHz.

---

## Motor Control

| Parameter | Value | Source |
|-----------|-------|--------|
| PWM timer | TIM8 | `Core/Src/tim.c` |
| PWM channels | CH1–CH4 | — |
| PWM pins | PC6, PC7, PC8, PC9 | `tim.c` MSP |
| PWM resolution | 8-bit (0 – 255) | `TIM8_ARR = 255` |
| PWM frequency | 15 625 Hz | see table above |
| Direction GPIOs | PB15, PB14, PB13, PB12 | `main.h` M1–M4_DIR |
| Motor sign pattern | [+1, −1, +1, −1] | `ctrl_config.h` |
| Max PWM value | 255 | `TIM8_ARR` |

Direction pin logic (in `motor_driver.c`): `GPIO_PIN_SET` → CCW, `GPIO_PIN_RESET` → CW.

Speed is written with:

```c
__HAL_TIM_SET_COMPARE(htim, channel, pwm_value);
```

---

## Encoders

| Parameter | Value | Source |
|-----------|-------|--------|
| Encoder mode | Quadrature TI1 | `tim.c` |
| Input filter | 15 (maximum) | `tim.c` IC1/IC2Filter |
| Input prescaler | DIV1 (every edge) | `tim.c` |
| Counts per revolution (CPR) | **1 940** | `ctrl_config.h` `PER_REV` |
| Motor 1 timer | TIM1 (PA8, PA9) | — |
| Motor 2 timer | TIM2 (PA15, PB3) | — |
| Motor 3 timer | TIM3 (PA6, PA7) | — |
| Motor 4 timer | TIM4 (PB6, PB7) | — |

The encoder driver (`encoder.c`) performs signed delta reading with overflow detection (threshold: 10 × PER_REV counts) to handle counter wrap-around on the 16-bit timers.

RPM calculation:

```
rpm = (delta_counts / PER_REV) / delta_time_us × 60 000 000
```

---

## PID Speed Controller

Implemented in `Core/Src/motor_pid.c` using `arm_pid_f32` from ARM CMSIS-DSP.

| Parameter | Value | Define |
|-----------|-------|--------|
| Max RPM | 160 | `MAX_RPM` |
| Kp | 3.0 | `KP` |
| Ki | 0.5 | `KI` |
| Kd | 0.2 | `KD` |
| EMA filter coefficient α | 0.5 | `ALPHA` |
| Control period | 1 000 µs (1 ms) | `CTRL_PERIOD_US` |

Speed feedback is smoothed with an exponential moving average before being fed to the PID:

```
rpm_filtered = α × rpm_raw + (1 − α) × rpm_filtered_prev
```

Feed-forward term:

```
pwm_ff = set_rpm × (TIM8_ARR / MAX_RPM)
```

Total output: `pwm = clamp(pwm_ff + pid_output, 0, TIM8_ARR)` with integral anti-windup.

---

## Battery Monitoring (ADC)

| Parameter | Value | Source |
|-----------|-------|--------|
| ADC | ADC1, Channel 1 | `Core/Src/adc.c` |
| Analog pin | PC0 | — |
| Resolution | 12-bit | — |
| Oversampling | 16× right-shift 4 | — |
| Effective resolution | 16-bit equivalent | — |
| Sampling time | 640.5 cycles | — |
| Trigger | TIM6 TRGO (update) | — |
| Sample rate | **10 Hz** | TIM6 config |
| Reference voltage (Vref) | 3.3 V | `ctrl_config.h` `V_REF` |
| Voltage divider ratio | 11.13374 | `ctrl_config.h` `DIV_RATIO` |

Voltage formula:

```
V_bat = (adc_raw / 4095) × 3.3 × 11.13374
```

---

## Time-of-Flight Sensor

VL53L1X long-distance ToF sensor connected via I2C1.

| Parameter | Value | Source |
|-----------|-------|--------|
| Sensor | VL53L1X | `Drivers/VL53L1X/` |
| Interface | I2C1 — PB8 (SCL), PB9 (SDA) | `Core/Src/i2c.c` |
| I2C speed | 400 kHz (Fast mode) | timing `0x00F12981` |
| I2C address | 0x52 | `ctrl_config.h` `TOF_ADR` |
| Timing budget | 100 ms | `TOF_TIMING_BUDGET_MS` |
| Inter-measurement period | 100 ms | `TOF_INTER_MEASUREMENT_MS` |
| Output rate | **10 Hz** | — |
| XSHUT pin | PA12 (active-high) | `main.h` `TOF_XSHUT_Pin` |
| Output unit | millimetres | — |

---

## UART Communication

| Interface | Purpose | Pins | Baud rate |
|-----------|---------|------|-----------|
| USART2 | Debug / `printf` | PA2 TX, PA3 RX | 115 200 |
| USART3 | Robot command interface | PC4 TX, PC5 RX | 230 400 |

USART3 uses interrupt-driven byte reception (`HAL_UART_Receive_IT`). Each received byte is passed to `uart_rx_byte_callback()` which assembles a line-terminated (`\n` or `\r`) command string into a 100-byte ring buffer.

---

## Serial Command Protocol

Commands are ASCII strings sent over USART3 (230 400 bps), terminated with `\n` or `\r`.

| Command | Arguments | Description |
|---------|-----------|-------------|
| `S` | `s0 s1 s2 s3` | Set target speed for each motor (RPM, signed float) |
| `P` | `kp ki kd` | Update PID gains at runtime |
| `F` | `s0 s1 s2 s3` | Set speeds and receive a full telemetry frame in response |
| `E` | — | Request current encoder positions (rotations) |
| `R` | — | Reset all encoder counters |

**Command timeout:** `CMD_TIMEOUT_US = 500 000 µs` (500 ms). If no command is received within this window, all motor speeds are set to 0.

---

## Software Timing

All timing is derived from the TIM5 `micros()` counter (1 MHz, 32-bit).

| Loop / event | Period | Frequency | Define |
|--------------|--------|-----------|--------|
| PID control update | **1 000 µs** | **1 kHz** | `CTRL_PERIOD_US` |
| Command poll | **10 000 µs** | **100 Hz** | `CMD_PERIOD_US` |
| Command timeout (watchdog) | **500 000 µs** | — | `CMD_TIMEOUT_US` |
| Battery ADC sample | 100 000 µs | 10 Hz | TIM6 hardware |
| ToF measurement | 100 000 µs | 10 Hz | sensor config |

Main loop structure (`Core/Src/main.c`):

```c
while (1) {
    if (micros() - cmd_timer > CMD_PERIOD_US) {
        controller_poll();          // parse incoming UART commands
        cmd_timer = micros();
    }
    if (micros() - ctrl_timer > CTRL_PERIOD_US) {
        controller_update();        // run PID for all 4 motors
        ctrl_timer = micros();
    }
}
```

---

## Pin Map

| Pin | Label | Direction | Function |
|-----|-------|-----------|----------|
| PA2 | USART_TX | OUT | USART2 TX (debug printf) |
| PA3 | USART_RX | IN | USART2 RX |
| PA6 | — | IN | TIM3_CH1 — Encoder M3 A |
| PA7 | — | IN | TIM3_CH2 — Encoder M3 B |
| PA8 | — | IN | TIM1_CH1 — Encoder M1 A |
| PA9 | — | IN | TIM1_CH2 — Encoder M1 B |
| PA12 | TOF_XSHUT | OUT | VL53L1X shutdown (active-high) |
| PA13 | TMS | — | SWD debug |
| PA14 | TCK | — | SWD debug |
| PA15 | — | IN | TIM2_CH1 — Encoder M2 A |
| PB3 | — | IN | TIM2_CH2 — Encoder M2 B |
| PB6 | — | IN | TIM4_CH1 — Encoder M4 A |
| PB7 | — | IN | TIM4_CH2 — Encoder M4 B |
| PB8 | — | I/O | I2C1 SCL (ToF sensor) |
| PB9 | — | I/O | I2C1 SDA (ToF sensor) |
| PB12 | M4_DIR | OUT | Motor 4 direction |
| PB13 | M3_DIR | OUT | Motor 3 direction |
| PB14 | M2_DIR | OUT | Motor 2 direction |
| PB15 | M1_DIR | OUT | Motor 1 direction |
| PC0 | — | AIN | ADC1_CH1 — battery voltage |
| PC4 | — | OUT | USART3 TX (command interface) |
| PC5 | — | IN | USART3 RX |
| PC6 | — | OUT | TIM8_CH1 — PWM Motor 1 |
| PC7 | — | OUT | TIM8_CH2 — PWM Motor 2 |
| PC8 | — | OUT | TIM8_CH3 — PWM Motor 3 |
| PC9 | — | OUT | TIM8_CH4 — PWM Motor 4 |
| PC13 | B1 | IN | User button (falling-edge EXTI) |

---

## Project Structure

```
mecanum_robot_system/
├── Core/
│   ├── Inc/
│   │   ├── main.h              # timer defines (TIM8_PSC/ARR, TIM5_PSC, TIM6_PSC/ARR)
│   │   ├── ctrl_config.h       # application constants (PID gains, PER_REV, ToF params)
│   │   ├── motor_driver.h/c    # PWM + GPIO direction output
│   │   ├── encoder.h/c         # quadrature encoder delta reading
│   │   ├── motor_pid.h/c       # CMSIS-DSP PID + EMA filter
│   │   ├── controller.h/c      # top-level state machine
│   │   ├── uart_comm.h/c       # interrupt-driven UART RX
│   │   ├── cmd_parser.h/c      # ASCII command parser
│   │   ├── battery.h/c         # ADC-based voltage measurement
│   │   ├── micros.h/c          # TIM5-based microsecond counter
│   │   └── tof_vl53l1x.h/c    # VL53L1X ToF driver wrapper
│   └── Src/
│       ├── main.c              # entry point, SystemClock_Config, main loop
│       ├── tim.c               # CubeMX-generated timer initialisation
│       ├── adc.c               # ADC1 init (oversampling, external trigger)
│       ├── i2c.c               # I2C1 init
│       ├── usart.c             # USART2/3 init
│       └── gpio.c              # GPIO init
├── Drivers/
│   ├── STM32L4xx_HAL_Driver/   # STM32 HAL
│   ├── CMSIS/                  # ARM core + DSP library
│   └── VL53L1X/                # ST VL53L1X platform driver
├── mecanum_robot_system.ioc    # STM32CubeMX project file
└── CMakeLists.txt              # CMake build system
```

# Changelog

---

## [feature/gear-warmup] — 2026-04-09

### 1. Gear Warmup Support for Geared Motors

**File:** `Inc/communication.h`

| | Before | After |
|---|---|---|
| `USBcom` enum | ended at `USBcom_ConfigTimer = 0x4D62` | added `USBcom_ConfigGear = 0x4D63` |
| `USBcomAnswer` enum | ended at `USBcomAnswer_ConfigTimer = 0x2A62` | added `USBcomAnswer_ConfigGear = 0x2A63` |
| `struct flagy` | no gear voltage field | added `uint16_t GearVoltage;` (0 = no gear warmup) |
| `struct tset` | no gear time field | added `uint16_t TimeGearRun;` (0 = no gear warmup) |

---

**File:** `Src/communication.c` — `USBDataResult()`

Before: no handler for command `0x4D63`, fell through to `default`.

After:
```c
case USBcom_ConfigGear:
    if(data[0] <= 600)  { Flag.GearVoltage  = data[0]; } else { WError = TRUE; }
    if(data[1] <= 3600) { Tset.TimeGearRun  = data[1]; } else { WError = TRUE; }
    size = USB_SendConfirmation(USB_Txbuffer, USBcomAnswer_ConfigGear, WError);
    break;
```
Bounds: voltage ≤ 60.0 V (stored as ×10), time ≤ 3600 s.

---

**File:** `Src/main.c` — `MAIN_START_REGULATOR` state

Before (single path, no gear support):
```c
ConnectVoltcraft(ON, Flag.Voltage, Flag.Current);
Set_RegulatorPWM(1000);
osDelay(1000);
Set_RegulatorPWM(Flag.Impuls);
osDelay(Tset.TimeMotorRun * 1000);
// measurement snapshot
```

After (gear path + standard path, motor never stops between phases):
```c
if (Flag.GearVoltage > 0)
{
    // GEAR WARMUP PHASE — power on at run-in voltage
    ConnectVoltcraft(ON, Flag.GearVoltage, Flag.Current);
    Set_RegulatorPWM(1000);
    osDelay(1000);
    Set_RegulatorPWM(Flag.Impuls);
    osDelay((uint32_t)Tset.TimeGearRun * 1000);

    // VOLTAGE SWITCH — I2C registers only, relay stays on, motor keeps spinning
    UpdateVoltcraftVoltage(Flag.Voltage, Flag.Current);
    osDelay(500);  // supply settles
}
else
{
    // STANDARD PATH — unchanged behaviour for non-geared motors
    ConnectVoltcraft(ON, Flag.Voltage, Flag.Current);
    Set_RegulatorPWM(1000);
    osDelay(1000);
    Set_RegulatorPWM(Flag.Impuls);
}

// MEASUREMENT PHASE — same for both paths
osDelay((uint32_t)Tset.TimeMotorRun * 1000);
// measurement snapshot
```

---

**File:** `Src/main.c` — new function `UpdateVoltcraftVoltage()`

Before: did not exist. Mid-run voltage change required calling `ConnectVoltcraft()` which re-fires relay and pre-charge circuit.

After:
```c
void UpdateVoltcraftVoltage(uint32_t voltage, uint32_t current)
```
Writes only to I2C registers 0x70 (voltage) and 0x72 (current). Safe to call while motor is running.

---

### 2. Division-by-Zero Fix in KV Calculation

**File:** `Src/main.c`

| | Before | After |
|---|---|---|
| `MotorKV` | `MotorKV = (CallValue*100/MotorUo);` | `MotorKV = (MotorUo > 0) ? (CallValue*100/MotorUo) : 0;` |

`MotorUo` is derived from ADC and can be 0 if the voltage sensor is disconnected or the supply has not stabilised. Without the guard this causes a hard fault (division by zero in integer arithmetic). With the guard, `MotorKV` reports 0 and execution continues normally.

---

### 3. CI/CD Pipeline Fixes and Artifact Expansion

**File:** `.github/workflows/build.yml`

| | Before | After |
|---|---|---|
| Trigger branches | `master` only | `master` + `feature/**` |
| APT packages | `gcc-arm-none-eabi binutils-arm-none-eabi make` | added `libnewlib-arm-none-eabi` (provides `nano.specs`) |
| CI artifacts | `StatorTester.bin` only | `.bin` + `.hex` + `.elf` + `.map` |

**File:** `Debug/makefile`

| | Before | After |
|---|---|---|
| `.hex` generation | not produced | `arm-none-eabi-objcopy -O ihex` target added |
| `post-build` | generates `.bin` only | generates `.bin` and `.hex` |
| `clean` target | removes `.bin .elf .list .map` | also removes `.hex` |
| `secondary-outputs` | `$(OBJCOPY_BIN)` | `$(OBJCOPY_BIN) $(OBJCOPY_HEX)` |

**File:** `Debug/*/subdir.mk` (all 6 files)

| | Before | After |
|---|---|---|
| Compiler flag | included `-fcyclomatic-complexity` | removed (ST-toolchain specific, not in Ubuntu `gcc-arm-none-eabi`) |

---

### Protocol Reference

| Field | Value |
|---|---|
| Command byte sent by Middleman | `0x63` |
| Full USB packet header+cmd | `[0x4D][0x63]` |
| STM32 enum match | `USBcom_ConfigGear = 0x4D63` |
| Confirmation reply | `[0x2A][0x63]` = `USBcomAnswer_ConfigGear` |
| `data[0]` | `run-in_voltage × 10` (e.g. 8.4 V → 84) |
| `data[1]` | `run-in_duration` in seconds (integer) |

# ECU STM32F405 + TLE8888 - Changelog

## v8.2.2 - Sequential Injection/Ignition + Code Cleanup (2026-01-16)

### Sequential Operation
- **Injeção sequencial**: Cada cilindro é injetado individualmente no ângulo correto
- **Ignição sequencial**: Dwell e spark calculados para cada cilindro
- **Firing order**: 1-3-4-2 (configurável em config.h)
- **Timing**: Injeção 330° BTDC (durante admissão)

### TLE8888 Driver Simplificado
- Reescrito para usar SPI padrão (16-bit frames)
- Removido código complexo de acesso direto a registros
- API simplificada: begin(), setOutput(), setIgnition()
- Diagnóstico via comando 'X'

### Lambda/Wideband Simplificado
- **Removido suporte a narrowband** (sonda de 1 fio)
- **Mantido apenas wideband**:
  - CAN: AEM X-Series, Innovate, etc. (via WB_CAN_ID)
  - Analógico: 0-5V linear (via PIN_WBO2)
- Seleção de fonte via `WB_INPUT_SOURCE` (0=CAN, 1=Analog)
- Nova função `lambdaUpdateDirect()` para entrada CAN
- Removidas funções não usadas: lambdaEnable/Disable/GetAfr/GetLambda

### Code Cleanup
**Removido de types.h:**
- Table2D, Curve1D structs (não usados, tables.h tem suas próprias)
- TABLE_SIZE_X, TABLE_SIZE_Y, CURVE_SIZE defines
- angleQ16 tipo e macros (ANGLE_TO_Q16, Q16_TO_ANGLE)

**Removido de config.h:**
- STOICH_AFR (duplicado com AFR_STOICH_E27 em lambda.h)
- SYNC_LOSS_TIMEOUT_MS (usa SYNC_LOSS_TIMEOUT_US em sync.h)
- CAN defines comentados (não implementado ainda)

**Removido de tables.cpp/h:**
- currentCalMode variável global (não utilizada, calState.mode é usado)

**Removido de lambda.h/cpp:**
- Suporte a LAMBDA_SENSOR_NARROWBAND
- Defines NB_RICH_THRESHOLD, NB_LEAN_THRESHOLD
- Código #if/#else para conversão narrowband
- Funções lambdaEnable(), lambdaDisable(), lambdaGetAfr(), lambdaGetLambda()

**Atualizado pinout.h:**
- PIN_O2 (PA5) → PIN_WBO2 (PA6) - corrigido conflito com SPI_SCK

**Total:** ~50 linhas de código morto removidas

---

## v8.2.1 - Autotune System + Bluetooth (2026-01-09)

### Major Additions

#### 1. Autotune System (autotune.h/cpp)
STFT/LTFT-based automatic fuel table learning:

**Concept:**
- **STFT (Short-Term Fuel Trim)**: Immediate correction via Lambda PID (existing)
- **LTFT (Long-Term Fuel Trim)**: Accumulated learning applied to base tables

**Modes:**
| Mode | Description |
|------|-------------|
| OFF | Autotune disabled |
| VE | Adjusts VE table (for Alpha-N mode) |
| AFR | Adjusts AFR target table |
| MAF | Adjusts MAF calibration table |

**Commands:**
| Key | Function |
|-----|----------|
| T | Cycle autotune mode (OFF→VE→AFR→MAF→OFF) |
| G | Print autotune status |
| U | Print LTFT grid (16x16) |
| J | Print MAF LTFT (24 cells) |
| Z | Reset/zero all LTFT |
| F | Force apply all accumulated LTFT |

**Learning Conditions:**
- Engine warm (CLT > 60°C)
- Lambda PID active (closed-loop)
- Steady-state: RPM delta < 100, TPS delta < 3%, Load delta < 5%
- Steady duration: 2+ seconds
- Valid lambda: 0.75 - 1.25
- Not at WOT: TPS < 85%
- RPM range: 1000 - 6000

**Parameters:**
| Parameter | Value | Description |
|-----------|-------|-------------|
| Incorporation | 0.02 | How fast LTFT accumulates |
| Apply Threshold | 3% | Min LTFT to trigger adjustment |
| Max Adjust/Cycle | 1% | Limit per application |
| Max Total LTFT | ±25% | Per-cell limit |
| Decay after Apply | 50% | Retain for convergence |

**Safety:**
- Adjustments limited to ±1% per application
- Total LTFT capped at ±25% per cell
- Only applies in steady-state
- LTFT decays after application for smooth convergence

#### 2. Bluetooth Support
Wireless tuning via HC-05/HC-06 module:

**Configuration (config.h):**
```c
#define BLUETOOTH_ENABLED    1
#define BLUETOOTH_BAUD       9600
```

**Wiring:**
| HC-05 | STM32 |
|-------|-------|
| VCC | 3.3V/5V |
| GND | GND |
| TX | PA10 (RX) |
| RX | PA9 (TX) |

**Features:**
- All status commands work via Bluetooth
- Autotune monitoring via smartphone
- Apps: Serial Bluetooth Terminal (Android)

---

## v8.2.0 - Table Consolidation + Real-Time Calibration (2026-01-08)

### Major Changes

#### 1. Complete Table Consolidation (tables.h/cpp)
All calibration tables moved to a single module:

**1D Tables (Factory Defaults) - All 8x1:**
| Table | Size | Units | Source |
|-------|------|-------|--------|
| DwellTable | 8x1 | µs | Original |
| DeadTimeTable | 8x1 | µs | Original |
| CrankingTable | 8x1 | % | Original |
| WarmupTable | 8x1 | % | Original |
| AfterstartTable | 8x1 | % | Original |
| AccelEnrichTable | 8x1 | % | Original |
| IdleRpmTable | 8x1 | RPM | Moved from idle.cpp |
| MafCalTable | 24x1 | g/s | Moved from .ino |

**2D Tables (Tunable via EEPROM):**
| Table | Size | Units | Source |
|-------|------|-------|--------|
| VeTable | 16x16 | % | Upgraded from 8x8 in .ino |
| IgnitionTable | 16x16 | °BTDC | Upgraded from 8x8 in .ino |
| AfrTargetTable | 16x16 | AFR | Upgraded from 8x8 in lambda.cpp |

#### 2. Real-Time Calibration System (calibration.h/cpp)
Serial-based tuning interface:

**Commands:**
| Key | Function |
|-----|----------|
| V | Select VE table (1% steps) |
| I | Select Ignition table (1° steps) |
| A | Select AFR table (0.1 steps) |
| m | Select MAF calibration (1% steps) |
| M | Print MAF table |
| P | Toggle Lambda PID auto-correction |
| + | Increase current cell |
| - | Decrease current cell |
| S | Save to EEPROM |
| L | Load from EEPROM |
| R | Reset to factory defaults |
| ? | Show current cell status |
| H | Help |
| 1-6 | Print various tables |

**Features:**
- Automatic cell tracking based on RPM and Load (mg/stroke)
- **Nearest-cell selection** for intuitive tuning between cells
- Interpolation weight display shows which cells affect current value
- Debounced keypress handling
- Real-time feedback of adjusted values
- EEPROM persistence with checksum validation

**Interpolation Behavior:**
When operating between cells (e.g., RPM=2200 between 2000-2500):
- ECU uses bilinear interpolation from 4 surrounding cells
- **4-cell simultaneous adjustment**: ALL 4 cells adjusted by the same step
- Math: output = Σ(wi×vi), so Δoutput = Σ(wi×step) = step×Σ(wi) = step×1.0 = step
- This ensures OUTPUT changes by exactly the nominal amount (1%, 1°, 0.1 AFR)
- Status display (?) shows all 4 cell values and weights

**Example:**
```
Position: RPM=2750 (75% toward 3000), Load=70 (50% toward 80)
Weights: [2500][60]=12%, [2500][80]=12%, [3000][60]=38%, [3000][80]=38%

Press +1% VE:
  ALL 4 cells: +1% each
  
Interpolated output change:
  = 12%×1% + 12%×1% + 38%×1% + 38%×1%
  = 1% × (12+12+38+38)% = 1% × 100% = 1.00% ✓
```

#### 3. Load Axis Changed to mg/stroke
All 2D tables now use mass-per-stroke as load axis:
- More universal than TPS%
- Normalizes load across RPM range
- Derived from MAF: `load_mg = (maf_gs * 120000) / (rpm * cylinders)`

#### 4. Module Updates

**lambda.cpp:**
- Removed local `AfrTargetTable`
- `lambdaGetTargetAfr()` now calls `getAfrTarget()` from tables.h

**idle.cpp:**
- Removed local RPM target arrays
- `idleGetTargetRpm()` now calls `getIdleTargetRpm()` from tables.h

### API Changes

**New Functions (tables.h):**
```cpp
// 2D Table Accessors
float getVe(float rpm, float loadMgStroke);
float getIgnition(float rpm, float loadMgStroke);
float getAfrTarget(float rpm, float loadMgStroke);

// MAF Calibration
float getMafFlow(float freqHz);

// Idle Target
uint16_t getIdleTargetRpm(float clt);

// Calibration
uint8_t findRpmIdx(float rpm);
uint8_t findLoadIdx(float loadMgStroke);
void adjustCell(CalibrationMode mode, uint8_t rpmIdx, uint8_t loadIdx, bool increase);

// EEPROM
bool tablesSaveToEEPROM(void);
bool tablesLoadFromEEPROM(void);
```

**Removed (from .ino):**
- `VE_TABLE[8][8]`
- `IGN_TABLE[8][8]`
- `VE_RPM_BINS[]`, `VE_TPS_BINS[]`
- `IGN_RPM_BINS[]`, `IGN_LOAD_BINS[]`
- `findBin8()`, `interpolateVE()`, `interpolateIGN()`
- MAF calibration arrays

**Removed (from lambda.cpp):**
- `AfrTargetTable afrTargetTable`
- `lambdaLoadDefaults()` table loading

**Removed (from idle.cpp):**
- `IdleCalibration idleCal`
- `idleLoadDefaults()` table loading

### Memory Usage

**CalibrationData Structure:**
```
VeTable:        16x16x4 + 32x4 = 1152 bytes
IgnitionTable:  16x16x4 + 32x4 = 1152 bytes  
AfrTargetTable: 16x16x4 + 32x4 = 1152 bytes
MafCalTable:    24x4 + 24x4    = 192 bytes
Magic + Version + Checksum     = 12 bytes
-----------------------------------------
Total CalibrationData:         ~3.7 KB
```

### File Structure v8.2

```
ECU_v82/
├── ECU_STM32F405_TLE8888.ino   # Main application
├── config.h                     # Engine constants
├── types.h                      # Data structures
├── pinout.h                     # Pin assignments
├── tables.h                     # ALL tables (NEW)
├── tables.cpp                   # Table implementation (NEW)
├── calibration.h                # Real-time tuning (NEW)
├── calibration.cpp              # Tuning implementation (NEW)
├── scheduler.h                  # Timer scheduling
├── scheduler.cpp                # Scheduler implementation
├── sync.h                       # Crank/cam sync
├── sync.cpp                     # Sync implementation
├── idle.h                       # Idle control (simplified)
├── idle.cpp                     # Idle implementation (simplified)
├── lambda.h                     # Lambda control (simplified)
├── lambda.cpp                   # Lambda implementation (simplified)
├── tle8888.h                    # Driver IC
└── tle8888.cpp                  # Driver implementation
```

### Tuning Workflow

**MAF Mode (Primary):**
- Air mass measured directly by MAF sensor
- Tune **AFR table only** to set target mixture
- Lambda closed-loop auto-corrects to hit target
- VE table NOT used in fuel calculation

**Alpha-N Mode (Fallback):**
- Air mass estimated via VE table
- Tune **VE table** for air mass accuracy
- Tune **AFR table** for target mixture
- Lambda closed-loop provides correction

| Mode | VE Table | AFR Table | Lambda PID |
|------|----------|-----------|------------|
| MAF | Not used | Target mix | Auto-correct |
| Alpha-N | Air mass | Target mix | Auto-correct |

**Ignition Tuning:**
- IGN table used in both modes
- Tune for MBT (Maximum Brake Torque)
- Watch for knock, retard if needed

**Lambda PID Control:**
- Press `P` to toggle auto-correction ON/OFF
- OFF: Correction frozen, see raw AFR from table
- ON: PID actively corrects to hit AFR target
- Use OFF mode for initial tuning, ON for fine-tuning

### Migration from v8.1.x

1. Replace all table files
2. Update includes in .ino
3. Remove local table definitions
4. Replace `interpolateVE()` → `getVe()`
5. Replace `interpolateIGN()` → `getIgnition()`
6. Replace lambda table access → `getAfrTarget()`
7. Replace idle target access → `getIdleTargetRpm()`
8. Add `calibrationInit()` and `calibrationUpdatePosition()` calls
9. Add `calibrationProcessSerial()` to main loop

---

## v8.1.5 - External Peer Review Fixes (2026-01-08)
[Previous changelog entries remain unchanged...]

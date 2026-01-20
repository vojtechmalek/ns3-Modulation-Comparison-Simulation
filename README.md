# Modulation Comparison Simulation

## Project Purpose

This project conducts a comparative performance study of three modulation schemes: **LoRa**, **WiFi HaLow**, and **DECT NR+**. The primary goal is to evaluate and compare their performance under different conditions, with particular focus on **range and noise tolerance** in line-of-sight scenarios, and **wall penetration and goodput** in non-line-of-sight scenarios.

### Why These Three Modulations?

All three modulation schemes operate in **license-exempt frequency bands**. They represent different trade-offs in frequency, data rate, and penetration capabilities:

- **LoRa (868 MHz)**:
  - Operates in 868 MHz ISM bands (but can be lowered)
  - **Low frequency** provides better **wall penetration** and long-range communication
  - **High SNR tolerance** (sensitivity down to -120 to -140 dBm) provides resilience in high-noise environments
  - **Disadvantages:** has a lower data rate (5.5 kbps at SF7), patented (commercial use requires licensing)

- **WiFi HaLow (868 MHz)**:
  - Operates in 868 MHz license-exempt bands
  - **Low frequency** provides good **wall penetration** (better than traditional 2.4/5 GHz WiFi)
  - **Higher data rate** than LoRa (150 kbps to 86.7 Mbps depending on configuration)
  - Based on IEEE 802.11ah standard (open standard)

- **DECT NR+ (1.9 GHz)**:
  - Operates at **higher frequency** (~1.9 GHz)
  - **Much higher data rate** (up to 2 Mbps) compared to LoRa and WiFi HaLow
  - **Disadvantages:** reduced wall penetration due to higher frequency, shorter range

### Research Focus

This simulation evaluates these modulations across two scenarios:
- **S1 (line-of-sight)**: Free-space propagation with variable distance
- **S2 (non-line-of-sight)**: Indoor propagation with walls to test **wall penetration**

## Overview

NS-3-based simulation system for comparing different modulation schemes (LoRa, DECT NR+, WiFi HaLow) across multiple scenarios with uniform metrics collection.

## Project Structure

```
ns-3/
├── modulation_comparison.cc      # Main simulation implementation
├── simulate.sh                   # Simulation launcher script
├── config.example.txt            # Example configuration file
├── Dockerfile                    # Dockerfile
├── DESIGN_DECISIONS.md           # Design explanation                   
└── outputs/                      # Simulation output directory
    └── YYYYMMDD_HHMMSS/          # Example timestamp directory
        ├── per_run_metrics.csv
        ├── aggregated_metrics.csv
        └── simulation_summary.txt
```

## Quick Start

### Running Simulations

**Build Docker Image**
```bash
# Build Docker image (first time only)
docker build -t ns3-modulation-comparison .
```

**Option 1: Command-Line Only**
```bash
# Run simulation with command-line arguments
./simulate.sh --scenario S1 --modulation lora --distance 20
./simulate.sh --scenario S1 --modulation dect-nr+ --distance 500
./simulate.sh --scenario S2 --modulation wifi-halow --walls 1
./simulate.sh --scenario S2 --modulation lora --walls 2 --wall-spacing 5
```

**Option 2: Configuration File**
```bash
# Create myconfig.txt
./simulate.sh --config myconfig.txt

# Override specific parameters from config file (command line arguments have priority)
./simulate.sh --config myconfig.txt --distance 50 --runs 50
```

## Configuration File

### Config File Format

- **Format**: `key=value` (one per line)
- **Comments**: Lines starting with `#` are ignored

### Example Config File

```ini
# Scenario
scenario=S1
modulation=lora
distance=20.0

# S2 Scenario Parameters (uncomment if using S2)
#scenario=S2
#walls=1
#wall-spacing=5.0
#building-type=residential
#external-wall-type=concrete-with-windows

# Traffic Parameters (specify 2 of 3)
packet-rate=6
packet-size=100
#duty-cycle=1.0  # Will be calculated if not specified

# Payload Percentage (what percentage of a packet is actual information)
payload-percentage=100.0

# Simulation
sim-time=10.0
runs=30

# Noise Parameters
temperature=290.0
noise-figure-db=10.0
background-interference-psd=4e-19

# Output directory
output-dir=outputs
```

### Configurable Parameters

| Parameter                     | Description                                            | Default                |
|-------------------------------|--------------------------------------------------------|------------------------|
| `scenario`                    | S1 or S2                                               | S1                     |
| `modulation`                  | lora, dect-nr+, wifi-halow                             | lora                   |
| `distance`                    | Distance for S1 (meters)                               | 20.0                   |
| `walls`                       | Number of walls for S2                                 | 1                      |
| `wall-spacing`                | Spacing between walls (meters)                         | 5.0                    |
| `building-type`               | Building type for S2: residential, office, commercial  | residential            |
| `external-wall-type`          | External wall type for S2: concrete-with-windows, concrete-without-windows, stone-blocks | concrete-with-windows |
| `payload-percentage`          | Percentage of packet that is actual payload            | 100.0                  |
| `packet-rate`                 | Packets per second (pps)                               | 6                      |
| `packet-size`                 | Packet size (bytes)                                    | 100                    |
| `duty-cycle`                  | Duty cycle (%)                                         | calculated             |
| `sim-time`                    | Simulation time (seconds)                              | 10.0                   |
| `runs`                        | Number of simulation runs                              | 1                      |
| `temperature`                 | Temperature (Kelvin)                                   | 290.0                  |
| `noise-figure-db`             | Noise figure (dB)                                      | 10.0                   |
| `background-interference-psd` | Background interference (W/Hz)                         | 4e-19                  |
| `output-dir`                  | Output directory                                       | outputs                |

### Priority Order
1. Command-line arguments (highest priority)
2. Custom config file (if `--config` specified)
3. `config.example.txt`
4. Hardcoded defaults (if config file is missing values)


## Scenarios

- **S1**: Line-of-Sight with variable distance
  - Uses `FriisPropagationLossModel` for free-space path loss
  - Example: `--scenario S1 --distance 20`
  
- **S2**: Indoor NLOS with variable walls
  - Uses `HybridBuildingsPropagationLossModel` for indoor propagation
  - Example: `--scenario S2 --walls 2 --wall-spacing 5`
  - Custom building: `--scenario S2 --walls 2 --building-type office --external-wall-type concrete-without-windows`


## Modulations

All modulations use NS-3 spectrum module (`AdhocAlohaNoackIdealPhy`) with SNR-based PER curves.
- **lora**: LoRa modulation (SF7)
  - Data Rate: 5.5 kbps
  - Frequency: 868 MHz
  - Bandwidth: 125 kHz
  - TX Power: 25 mW (14 dBm)
  
- **dect-nr+**: DECT NR+
  - Data Rate: 2 Mbps
  - Frequency: 1.9 GHz
  - Bandwidth: 1.728 MHz
  - TX Power: 10 mW (10 dBm)
  
- **wifi-halow**: WiFi HaLow (MCS0)
  - Data Rate: 150 kbps
  - Frequency: 868 MHz
  - Bandwidth: 1 MHz
  - TX Power: 25 mW (14 dBm)


## Output Files

- **per_run_metrics.csv**: Individual run results with all metrics (1 row per simulation run)
- **aggregated_metrics.csv**: Statistical summary (mean, std dev, min, max, percentiles: p25, p50, p75, p95) across all runs
- **simulation_summary.txt**: Human-readable simulation parameters and results summary


## Metrics

**Per-Packet Metrics:**
- Pathloss (dB)
- SNR (dB)
- RSSI (dBm)
- Latency (seconds)
- Success/Failure status

**Aggregated Metrics (per run):**
- PER (Percentage of lost packets)
- SNR (mean, min, max)
- RSSI (mean, min, max)
- Pathloss (mean, min, max)
- Goodput (bits per second)
- Latency (mean, min, max)
- Duty Cycle (percentage)
- Data Rate (bits per second)
- KPI (Composite metric combining PER, goodput, and SNR)

**Statistical Summary (across all runs):**
- Mean, standard deviation, min, max
- Percentiles: p25, p50, p75, p95

## Technical Details

### Noise Calculation

Noise power is calculated using:
- Thermal noise: `k × T × NF` (Boltzmann constant × Temperature × Noise Figure)
- Background interference (additional PSD term)
- Modulation-specific bandwidth (used to convert PSD to total noise power)

### PER Curves

Each modulation has a custom PER (Packet Error Rate) curve based on SNR:
PER curves use linear interpolation in transition intervals and exponential decay at high SNR.
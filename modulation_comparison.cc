// ============================================================================
// INCLUDES
// ============================================================================

// NS-3 core modules
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/spectrum-module.h"
#include "ns3/applications-module.h"
#include "ns3/propagation-module.h"
#include "ns3/buildings-module.h"
#include "ns3/trace-helper.h"

// Standard C++ libraries
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <map>
#include <queue>
#include <algorithm>
#include <ctime>
#include <numeric>
#include <iomanip>
#include <sstream>

// System libraries for directory operations
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <cstring>

using namespace ns3;

// ============================================================================
// LOG COMPONENT DEFINITION
// ============================================================================

NS_LOG_COMPONENT_DEFINE("ModulationComparison");

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

// Scenario setup functions
void SetupScenarioS1(NodeContainer& nodes, SpectrumChannelHelper& channelHelper, double distance, double frequencyHz);
void SetupScenarioS2(NodeContainer& nodes, SpectrumChannelHelper& channelHelper, uint32_t numWalls, double wallSpacing, const std::string& buildingType, const std::string& externalWallType, double frequencyHz);

// Scenario printing functions
void PrintScenarioS1Info(double distance);
void PrintScenarioS2Info(uint32_t numWalls, double wallSpacing, uint32_t numRooms, const std::string& buildingType, const std::string& externalWallType);

// Helper function to calculate S2 scenario distance
double CalculateS2Distance(uint32_t numWalls, double wallSpacing);

// Modulation setup functions
void SetupModulationLoRa(NodeContainer& nodes, Ptr<SpectrumChannel> channel, NetDeviceContainer& devices);
void SetupModulationDectNrPlus(NodeContainer& nodes, Ptr<SpectrumChannel> channel, NetDeviceContainer& devices);
void SetupModulationWifiHalow(NodeContainer& nodes, Ptr<SpectrumChannel> channel, NetDeviceContainer& devices);
void SetupModulationCommon(NodeContainer& nodes, Ptr<SpectrumChannel> channel, double txPowerW, const std::string& dataRate, NetDeviceContainer& devices);

// Helper functions for calculations
double CalculateNoisePowerSpectralDensity(double temperature, double noiseFigureDb, double backgroundInterferencePsd);
double ConvertWattsToDbm(double txPowerW);
double GetTxPowerWatts(const std::string& modulation);
double GetModulationDataRate(const std::string& modulation);
double GetModulationFrequencyHz(const std::string& modulation);
double GetModulationBandwidthHz(const std::string& modulation);

// Configuration file parsing functions
bool ParseConfigFile(const std::string& configPath, std::map<std::string, std::string>& config);
bool CalculateTrafficParameters(const std::string& modulation, double& packetRate, double& packetSize, double& dutyCycle);

// Helper functions to apply config file values
void ApplyConfigString(const std::map<std::string, std::string>& config, const std::string& key, std::string& value);
void ApplyConfigDouble(const std::map<std::string, std::string>& config, const std::string& key, double& value);
void ApplyConfigUint(const std::map<std::string, std::string>& config, const std::string& key, uint32_t& value);

// Helper function to write simulation parameters to an output stream
void WriteSimulationParameters(std::ostream& os, const std::string& scenario, const std::string& modulation,
                                double distance, double simulationTime, uint32_t packetSize, 
                                uint32_t packetsPerSecond, double dutyCycle, uint32_t numRuns,
                                uint32_t numWalls = 0, double wallSpacing = 0.0,
                                const std::string& buildingType = "", const std::string& externalWallType = "",
                                uint32_t seed = 0, double temperature = 0.0, double noiseFigureDb = 0.0);

// PER curve functions
double CalculatePERFromSNR_LoRa(double snrDb);
double CalculatePERFromSNR_DectNrPlus(double snrDb);
double CalculatePERFromSNR_WifiHalow(double snrDb);
double CalculatePERFromSNR(const std::string& modulation, double snrDb);

// ============================================================================
// METRICS COLLECTION STRUCTURES
// ============================================================================

// Structure to store per-packet metrics
struct PacketMetrics
{
    uint64_t packetId;   // Packet ID
    Time txTime;         // Transmission time
    Time rxTime;         // Reception time
    double pathlossDb;   // Path loss (dB)
    double snrDb;        // Signal-to-noise ratio (dB)
    double rssiDbm;      // Received signal strength indicator (dBm)
    double latency;      // Latency in seconds
    bool success;        // Success flag
    uint32_t packetSize; // Packet size in bytes
};

// Structure to store total simulation metrics
struct SimulationMetrics
{
    uint32_t totalSent;     // Total sent packets
    uint32_t totalReceived; // Total successfully received packets
    uint32_t totalLost;     // Total lost packets
    double per;             // Packet Error Rate (percentage)
    double snrMean;         // Mean SNR (dB)
    double snrMin;          // Min SNR (dB)
    double snrMax;          // Max SNR (dB)
    double rssiMean;        // Mean RSSI (dBm)
    double rssiMin;         // Min RSSI (dBm)
    double rssiMax;         // Max RSSI (dBm)
    double goodput;         // Goodput (bps)
    double latencyMean;     // Mean latency (seconds)
    double latencyMin;      // Min latency (seconds)
    double latencyMax;      // Max latency (seconds)
    double pathlossMean;    // Pathloss mean (dB)
    double pathlossMin;     // Pathloss min (dB)
    double pathlossMax;     // Pathloss max (dB)
    double dutyCycle;       // Duty cycle (percentage)
    double dataRate;        // Data rate (bps)
    double kpi;             // KPI (percentage)
};

// Statistics structure for aggregated metrics
struct Statistics {
    double mean;
    double stdDev;
    double min;
    double max;
    double p25;
    double p50;
    double p75;
    double p95;
};

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// Global metrics storage
SimulationMetrics g_simMetrics;                    // Aggregated simulation metrics
std::map<uint64_t, PacketMetrics> g_packetMetrics; // Map: packetId -> PacketMetrics (stores all packet data)
double g_pathlossDb = 0.0;                         // Pathloss in dB (updated by PathlossTrace callback)

// Per-packet pathloss storage (for matching pathloss to packets)
std::queue<double> g_pathlossQueue;   // Queue of pathloss values (one per packet in TX order)

// Global variable for modulation
std::string g_modulation = "";   // Current modulation

// Global noise parameters
double g_temperature;                // Temperature in Kelvin
double g_noiseFigureDb;              // Noise figure in dB
double g_backgroundInterferencePsd;  // Background interference PSD in W/Hz

// ============================================================================
// CONSTANTS
// ============================================================================

// Physical constants
const double BOLTZMANN_CONSTANT = 1.381e-23;   // Boltzmann's constant
const double NODE_HEIGHT = 1.5;                // Node height above ground
const double NODE_OFFSET = 2.0;                // Distance from building edge to node (for S2 scenario)

// Propagation model constants (for S1)
const double SYSTEM_LOSS = 1.0;                 // System loss factor (1.0 = no additional system loss)

// Building dimensions constants (S2 - Indoor NLOS)
const double BUILDING_DEPTH = 10.0;             // Building depth in meters (y-axis)
const double BUILDING_HEIGHT = 3.0;             // Building height in meters (z-axis)

// Modulation-specific constants
const double LORA_TX_POWER_W = 0.025;          // LoRa TX power (25 mW = 14 dBm)
const double DECT_NR_PLUS_TX_POWER_W = 0.01;   // DECT NR+ TX power (10 mW = 10 dBm)
const double WIFI_HALOW_TX_POWER_W = 0.025;    // WiFi HaLow TX power (25 mW = 14 dBm)

// ============================================================================
// DEFAULT PARAMETER VALUES
// ============================================================================
// INFO: These are used if missing from config

// Scenario defaults
const std::string DEFAULT_SCENARIO = "S1";
const double DEFAULT_S1_DISTANCE = 20.0;
const uint32_t DEFAULT_NUM_WALLS = 1;
const double DEFAULT_WALL_SPACING = 5.0;
const std::string DEFAULT_BUILDING_TYPE = "residential";
const std::string DEFAULT_EXTERNAL_WALL_TYPE = "concrete-with-windows";

// Modulation defaults
const std::string DEFAULT_MODULATION = "lora";

// Traffic defaults
const double DEFAULT_SIMULATION_TIME = 10.0;
const double DEFAULT_PACKET_SIZE = 100;
const double DEFAULT_PACKETS_PER_SECOND = 6;
const double DEFAULT_PAYLOAD_PERCENTAGE = 100.0;

// Noise defaults
const double DEFAULT_TEMPERATURE_KELVIN = 290.0;
const double DEFAULT_NOISE_FIGURE_DB = 10.0;
const double DEFAULT_BACKGROUND_INTERFERENCE_PSD = 4e-19;

// Output defaults
const std::string DEFAULT_OUTPUT_DIR = "outputs";

// Runs defaults
const uint32_t DEFAULT_NUM_RUNS = 1;

// ============================================================================
// TODO: NAME THIS HEADER OR PUT IT UNDER ANOTHER HEADER
// ============================================================================

// Function that returns the data rate for a given modulation
double GetModulationDataRate(const std::string& modulation)
{
    if (modulation == "lora") {
        return 5500.0;  // 5.5 kbps
    } else if (modulation == "dect-nr+") {
        return 2000000.0;  // 2 Mbps
    } else if (modulation == "wifi-halow") {
        return 150000.0;  // 150 kbps
    }
    return -1.0;
}

// Calculate the missing traffic parameter (packetRate, packetSize or dutyCycle)
// The user provides 2 and the 3rd is calculated
// dutyCycle = (packetSize × 8 / modulationDataRate) × packetRate × 100%
bool CalculateTrafficParameters(const std::string& modulation,
                                 double& packetRate, double& packetSize, double& dutyCycle)
{
    double modulationDataRate = GetModulationDataRate(modulation);
    
    int specifiedCount = 0; // Count of specified parameters (we need 2 out of 3)
    if (packetRate > 0.0) specifiedCount++;
    if (packetSize > 0.0) specifiedCount++;
    if (dutyCycle > 0.0) specifiedCount++;
    
    if (specifiedCount == 0) {
        // No parameters provided, use defaults
        packetRate = DEFAULT_PACKETS_PER_SECOND;
        packetSize = DEFAULT_PACKET_SIZE;
    } else if (specifiedCount == 1) {
        // One parameter provided, use default for a second one and calculate the third
        // Priority: packet-rate > packet-size > duty-cycle
        if (packetRate > 0.0) {
            // packet-rate provided: use default packet-size
            packetSize = DEFAULT_PACKET_SIZE;
        } else if (packetSize > 0.0) {
            // packet-size provided: use default packet-rate
            packetRate = DEFAULT_PACKETS_PER_SECOND;
        } else {
            // duty-cycle provided: use default packet-rate
            packetRate = DEFAULT_PACKETS_PER_SECOND;
        }
    } else if (specifiedCount == 3) {
        std::cerr << "Error: Cannot specify all 3 traffic parameters" << std::endl;
        return false;
    }

    // === CALCULATE THE MISSING PARAMETER ===
    // If packer rate and packet size are provided, calculate duty cycle
    if (packetRate > 0.0 && packetSize > 0.0) {
        double packetTxDuration = (packetSize * 8.0) / modulationDataRate;
        dutyCycle = packetTxDuration * packetRate * 100.0;
    // If packet rate and duty cycle are provided, calculate packet size
    } else if (packetRate > 0.0 && dutyCycle > 0.0) {
        packetSize = (dutyCycle / 100.0) * modulationDataRate / (packetRate * 8.0);
    // If packet size and duty cycle are provided, calculate packet rate
    } else if (packetSize > 0.0 && dutyCycle > 0.0) {
        double packetTxDuration = (packetSize * 8.0) / modulationDataRate;
        packetRate = (dutyCycle / 100.0) / packetTxDuration;
    }

    // Round packetSize and packetRate to integers ((NS-3 requires uint32_t)
    packetSize = std::round(packetSize);
    packetRate = std::round(packetRate);

    // Recalculate duty cycle from rounded values
    dutyCycle = ((packetSize * 8.0) / modulationDataRate) * packetRate * 100.0;
    
    // Validation checks
    if (packetSize < 1.0) {
        std::cerr << "Error: Packet size must be at least 1 byte (is " 
                  << static_cast<uint32_t>(packetSize) << " bytes)" << std::endl;
        return false;
    }
    
    if (packetRate <= 0.0) {
        std::cerr << "Error: Packet rate must be more than 0 (is " 
                  << static_cast<uint32_t>(packetRate) << " pps)" << std::endl;
        return false;
    }
    
    if (dutyCycle > 100.0) {
        std::cerr << "Error: Duty cycle must be less than 100% (is " 
                  << static_cast<uint32_t>(dutyCycle) << "%)" << std::endl;
        return false;
    }
    
    return true;
}

// ============================================================================
// CONFIGURATION FILE FUNCTIONS
// ============================================================================

// Parse configuration file (return key-value pairs)
bool ParseConfigFile(const std::string& configPath, std::map<std::string, std::string>& config)
{
    std::ifstream file(configPath);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open config file: " << configPath << std::endl;
        return false;
    }
    
    std::string line;
    uint32_t lineNumber = 0;
    
    while (std::getline(file, line)) {
        lineNumber++;
        
        // Remove leading whitespace
        size_t start = line.find_first_not_of(" \t");
        if (start == std::string::npos) { // Empty line
            continue;
        }
        line = line.substr(start);
        
        // Remove trailing whitespace
        size_t end = line.find_last_not_of(" \t");
        if (end != std::string::npos) {
            line = line.substr(0, end + 1);
        }
        
        // Skip comments
        if (line[0] == '#') {
            continue;
        }
        
        // Parse key=value
        size_t eqPos = line.find('=');
        if (eqPos == std::string::npos) {
            std::cerr << "Warning: Invalid line " << lineNumber << " in config file: " << line << std::endl;
            continue;
        }
        
        std::string key = line.substr(0, eqPos);
        std::string value = line.substr(eqPos + 1);
        
        // Remove whitespace from key and value
        size_t keyStart = key.find_first_not_of(" \t");
        size_t keyEnd = key.find_last_not_of(" \t");
        if (keyStart != std::string::npos && keyEnd != std::string::npos) {
            key = key.substr(keyStart, keyEnd - keyStart + 1);
        } else {
            key.clear();  // Key is all whitespace
        }
        
        size_t valStart = value.find_first_not_of(" \t");
        size_t valEnd = value.find_last_not_of(" \t");
        if (valStart != std::string::npos && valEnd != std::string::npos) {
            value = value.substr(valStart, valEnd - valStart + 1);
        } else {
            value.clear();  // Value is all whitespace
        }
        
        // Validate key (empty keys are invalid)
        if (key.empty() or value.empty()) {
            std::cerr << "Warning: empty key or value on line " << lineNumber << " in config file: " << std::endl;
            continue;
        }
        
        config[key] = value;
    }
    
    file.close();
    return true;
}

// Helper functions to apply config file values to variables
void ApplyConfigString(const std::map<std::string, std::string>& config, const std::string& key, std::string& value)
{
    if (config.count(key) > 0) {
        value = config.at(key);
    }
}

void ApplyConfigDouble(const std::map<std::string, std::string>& config, const std::string& key, double& value)
{
    if (config.count(key) > 0) {
        value = std::stod(config.at(key));
    }
}

void ApplyConfigUint(const std::map<std::string, std::string>& config, const std::string& key, uint32_t& value)
{
    if (config.count(key) > 0) {
        value = std::stoul(config.at(key));
    }
}

// Helper function to write simulation parameters
void WriteSimulationParameters(std::ostream& os, const std::string& scenario, const std::string& modulation,
                                double distance, double simulationTime, uint32_t packetSize, 
                                uint32_t packetsPerSecond, double dutyCycle, uint32_t numRuns,
                                uint32_t numWalls, double wallSpacing,
                                const std::string& buildingType, const std::string& externalWallType,
                                uint32_t seed, double temperature, double noiseFigureDb)
{
    os << "--- Scenario Parameters ---" << std::endl;
    os << "Scenario: " << scenario << std::endl;
    
    if (scenario == "S1") {
        os << "Distance: " << std::fixed << std::setprecision(2) << distance << " m" << std::endl;
    } else if (scenario == "S2") {
        os << "Number of Walls: " << numWalls << std::endl;
        os << "Wall Spacing: " << std::fixed << std::setprecision(2) << wallSpacing << " m" << std::endl;
        if (!buildingType.empty()) {
            os << "Building Type: " << buildingType << std::endl;
        }
        if (!externalWallType.empty()) {
            os << "External Wall Type: " << externalWallType << std::endl;
        }
    }
    os << std::endl;
    
    os << "--- Simulation Parameters ---" << std::endl;
    os << "Modulation: " << modulation << std::endl;
    os << "Number of Runs: " << numRuns << std::endl;
    if (seed > 0) {
        os << "Random Seed: " << seed << " (base seed)" << std::endl;
    }
    os << "Simulation Time: " << simulationTime << " s" << std::endl;
    os << std::endl;
    
    os << "--- Traffic Parameters ---" << std::endl;
    os << "Packet Size: " << packetSize << " bytes" << std::endl;
    os << "Packets Per Second: " << packetsPerSecond << " pps" << std::endl;
    os << "Data Rate: " << std::setprecision(2) << (packetsPerSecond * packetSize * 8.0 / 1000.0) << " kbps" << std::endl;
    if (dutyCycle > 0.0) {
        os << "Duty Cycle: " << std::fixed << std::setprecision(2) << dutyCycle << "%" << std::endl;
    }
    os << std::endl;
    
    os << "--- Noise Parameters ---" << std::endl;
    os << "Temperature: " << temperature << " K" << std::endl;
    os << "Noise Figure: " << noiseFigureDb << " dB" << std::endl;
    os << std::endl;
}

// ============================================================================
// TODO: NAME THIS HEADER OR PUT IT UNDER ANOTHER HEADER
// ============================================================================

// Calculate noise power spectral density
double CalculateNoisePowerSpectralDensity(double temperature, double noiseFigureDb, double backgroundInterferencePsd)
{
    const double noiseFigureLinear = std::pow(10.0, noiseFigureDb / 10.0);
    return BOLTZMANN_CONSTANT * temperature * noiseFigureLinear + backgroundInterferencePsd;
}

// Get TX power in Watts for a given modulation
double GetTxPowerWatts(const std::string& modulation)
{
    if (modulation == "lora") {
        return LORA_TX_POWER_W;
    } else if (modulation == "dect-nr+") {
        return DECT_NR_PLUS_TX_POWER_W;
    } else if (modulation == "wifi-halow") {
        return WIFI_HALOW_TX_POWER_W;
    }
    return -1.0;
}

// Get frequency in Hz for a given modulation
double GetModulationFrequencyHz(const std::string& modulation)
{
    if (modulation == "lora") {
        return 868e6;  // 868 MHz
    } else if (modulation == "dect-nr+") {
        return 1.9e9;  // 1.9 GHz
    } else if (modulation == "wifi-halow") {
        return 868e6;  // 868 MHz
    }
    return -1.0;
}

// Get bandwidth in Hz for a given modulation
double GetModulationBandwidthHz(const std::string& modulation)
{
    if (modulation == "lora") {
        return 125e3;  // 125 kHz
    } else if (modulation == "dect-nr+") {
        return 5e6;     // 5 MHz
    } else if (modulation == "wifi-halow") {
        return 1e6;     // 1 MHz
    }
    return -1.0;
}

// Convert TX power from Watts to dBm
double ConvertWattsToDbm(double txPowerW)
{
    return 10.0 * std::log10(txPowerW * 1000.0);
}

// Calculate Signal-to-Noise Ratio in dB
double CalculateSNR(double txPowerDbm, double pathlossDb, double noisePowerDbm)
{
    // Calculate received power (RX = TX - pathloss)
    double rxPowerDbm = txPowerDbm - pathlossDb;
    
    // Calculate SNR (SNR = RX - noise)
    double snrDb = rxPowerDbm - noisePowerDbm;
    
    return snrDb;
}

// ============================================================================
// PER CURVE FUNCTIONS
// ============================================================================

// Calculate Packet Error Rate for LoRa SF7 based on SNR
double CalculatePERFromSNR_LoRa(double snrDb)
{
    // Calculated as follows:
    //   100% error rate below -15 dB
    //   Linear interpolation in intervals:
    //     -15 dB to -12 dB: 100% to 10%
    //     -12 dB to -10 dB: 10% to 1%
    //     -10 dB to -7.5 dB: 1% to 0.1%
    //     -7.5 dB to -5 dB: 0.1% to 0.01%
    //   Exponential decay above -5 dB
    if (snrDb < -15.0) {
        return 1.0;
    } else if (snrDb < -12.0) {
        return 1.0 - ((snrDb + 15.0) / 3.0) * 0.9;
    } else if (snrDb < -10.0) {
        return 0.1 - ((snrDb + 12.0) / 2.0) * 0.09;
    } else if (snrDb < -7.5) {
        return 0.01 - ((snrDb + 10.0) / 2.5) * 0.009;
    } else if (snrDb < -5.0) {
        return 0.001 - ((snrDb + 7.5) / 2.5) * 0.0009;
    } else {
        return std::max(0.0, 0.0001 * std::exp(-(snrDb + 5.0) / 2.0));
    }
}

// Calculate Packet Error Rate for DECT NR+ based on SNR
double CalculatePERFromSNR_DectNrPlus(double snrDb)
{
    // Calculated as follows:
    //   100% error rate below 0 dB
    //   Linear interpolation in intervals:
    //     0 dB to 5 dB: 100% to 10%
    //     5 dB to 8 dB: 10% to 1%
    //     8 dB to 12 dB: 1% to 0.1%
    //     12 dB to 15 dB: 0.1% to 0.01%
    //   Exponential decay above 15 dB
    if (snrDb < 0.0) {
        return 1.0;
    } else if (snrDb < 5.0) {
        return 1.0 - (snrDb / 5.0) * 0.9;
    } else if (snrDb < 8.0) {
        return 0.1 - ((snrDb - 5.0) / 3.0) * 0.09;
    } else if (snrDb < 12.0) {
        return 0.01 - ((snrDb - 8.0) / 4.0) * 0.009;
    } else if (snrDb < 15.0) {
        return 0.001 - ((snrDb - 12.0) / 3.0) * 0.0009;
    } else {
        return std::max(0.0, 0.0001 * std::exp(-(snrDb - 15.0) / 3.0));
    }
}

// Calculate Packet Error Rate for WiFi HaLow based on SNR
double CalculatePERFromSNR_WifiHalow(double snrDb)
{
    // Calculated as follows:
    //   100% error rate below -2 dB
    //   Linear interpolation in intervals:
    //     -2 dB to 3 dB: 100% to 10%
    //     3 dB to 6 dB: 10% to 1%
    //     6 dB to 10 dB: 1% to 0.1%
    //     10 dB to 12 dB: 0.1% to 0.01%
    //   Exponential decay above 12 dB
    if (snrDb < -2.0) {
        return 1.0;
    } else if (snrDb < 3.0) {
        return 1.0 - ((snrDb + 2.0) / 5.0) * 0.9;
    } else if (snrDb < 6.0) {
        return 0.1 - ((snrDb - 3.0) / 3.0) * 0.09;
    } else if (snrDb < 10.0) {
        return 0.01 - ((snrDb - 6.0) / 4.0) * 0.009;
    } else if (snrDb < 12.0) {
        return 0.001 - ((snrDb - 10.0) / 2.0) * 0.0009;
    } else {
        return std::max(0.0, 0.0001 * std::exp(-(snrDb - 12.0) / 2.0));
    }
}

// Wrapper function that calls the appropriate PER curve function based on modulation type
double CalculatePERFromSNR(const std::string& modulation, double snrDb)
{
    if (modulation == "lora") {
        return CalculatePERFromSNR_LoRa(snrDb);
    } else if (modulation == "dect-nr+") {
        return CalculatePERFromSNR_DectNrPlus(snrDb);
    } else if (modulation == "wifi-halow") {
        return CalculatePERFromSNR_WifiHalow(snrDb);
    }
    return -1.0;
}

// ============================================================================
// STATISTICS FUNCTIONS
// ============================================================================

// Calculate percentile from sorted values
double CalculatePercentile(const std::vector<double>& sortedValues, double percentile)
{
    if (sortedValues.empty()) return 0.0;
    if (sortedValues.size() == 1) return sortedValues[0];
    
    double index = percentile / 100.0 * (sortedValues.size() - 1);
    size_t lower = (size_t)std::floor(index);
    size_t upper = (size_t)std::ceil(index);
    
    if (lower == upper) {
        return sortedValues[lower];
    }
    
    double weight = index - lower;
    return sortedValues[lower] * (1.0 - weight) + sortedValues[upper] * weight;
}

Statistics CalculateStatistics(const std::vector<double>& values)
{
    Statistics stats;
    
    // If no values, return 0s for all statistics
    if (values.empty()) {
        stats.mean = stats.stdDev = stats.min = stats.max = stats.p25 = stats.p50 = stats.p75 = stats.p95 = 0.0;
        return stats;
    }
    
    // Calculate mean
    stats.mean = std::accumulate(values.begin(), values.end(), 0.0) / values.size();
    
    // Calculate standard deviation
    if (values.size() == 1) {
        stats.stdDev = 0.0;
    } else {
        double variance = 0.0;
        for (double val : values) {
            variance += (val - stats.mean) * (val - stats.mean);
        }
        stats.stdDev = std::sqrt(variance / (values.size() - 1));
    }
    
    // Calculate min/max
    stats.min = *std::min_element(values.begin(), values.end());
    stats.max = *std::max_element(values.begin(), values.end());
    
    // Sort the values (for percentiles)
    std::vector<double> sortedValues = values;
    std::sort(sortedValues.begin(), sortedValues.end());
    
    // Calculate percentiles
    stats.p25 = CalculatePercentile(sortedValues, 25.0);
    stats.p50 = CalculatePercentile(sortedValues, 50.0);
    stats.p75 = CalculatePercentile(sortedValues, 75.0);
    stats.p95 = CalculatePercentile(sortedValues, 95.0);
    
    return stats;
}

// Calculate KPI from metrics
// KPI = (perFactor × 0.5 + goodputFactor × 0.4 + snrFactor × 0.1) × 100
double CalculateKPI(double per, double goodput, double dataRate, double snrMean)
{
    double perFactor = 1.0 - (per / 100.0); // PER clamped to 0-1 (1 = 0% error rate)
    double goodputFactor = (dataRate > 0) ? (goodput / dataRate) : 0.0; // Goodput clamped to 0-1 (1 = 100% goodput)
    double snrFactor = std::min(1.0, std::max(0.0, (snrMean + 20.0) / 100.0)); // SNR clamped to 0-1 (1 = 100% SNR)
    return (perFactor * 0.5 + goodputFactor * 0.4 + snrFactor * 0.1) * 100.0;
}

// ============================================================================
// OUTPUT GENERATION FUNCTIONS
// ============================================================================

// Create output directory
bool CreateOutputDirectory(const std::string& dirPath)
{
    int result = mkdir(dirPath.c_str(), 0755);
    if (result == 0 || (result == -1 && errno == EEXIST)) {
        return true;
    }
    return false; // Failed to create
}

// Write per-run metrics to CSV file
void WritePerRunMetrics(const std::string& filePath, const SimulationMetrics& metrics,
                        const std::string& scenario, const std::string& modulation,
                        double distance, uint32_t seed, uint32_t runNumber)
{
    std::ofstream file(filePath, std::ios::out | std::ios::app);  // Append mode (add new lines)
    
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file for writing: " << filePath << std::endl;
        return;
    }
    
    // Write data
    file << runNumber << ","
         << scenario << ","
         << modulation << ","
         << std::fixed << std::setprecision(2) << distance << ","
         << seed << ","
         << metrics.totalSent << ","
         << metrics.totalReceived << ","
         << metrics.totalLost << ","
         << std::setprecision(4) << metrics.per << ","
         << std::setprecision(2) << metrics.snrMean << ","
         << metrics.snrMin << ","
         << metrics.snrMax << ","
         << metrics.rssiMean << ","
         << metrics.rssiMin << ","
         << metrics.rssiMax << ","
         << std::setprecision(2) << metrics.goodput << ","
         << std::setprecision(6) << metrics.latencyMean << ","
         << metrics.latencyMin << ","
         << metrics.latencyMax << ","
         << std::setprecision(2) << metrics.dutyCycle << ","
         << std::setprecision(2) << metrics.dataRate << ","
         << std::setprecision(4) << metrics.kpi << std::endl;
    file.close();
}

// Write aggregated metrics CSV with statistics calculated from per_run_metrics.csv
void WriteAggregatedMetrics(const std::string& perRunMetricsPath, const std::string& aggregatedMetricsPath,
                            const std::string& scenario, const std::string& modulation, double distance)
{
    // Read per_run_metrics.csv
    std::ifstream inputFile(perRunMetricsPath);
    if (!inputFile.is_open()) {
        std::cerr << "Error: Could not open per_run_metrics.csv for reading: " << perRunMetricsPath << std::endl;
        return;
    }
    
    // Skip header line
    std::string header;
    std::getline(inputFile, header);
    
    // Collect values for each metric
    std::vector<double> perValues;
    std::vector<double> snrMeanValues;
    std::vector<double> rssiMeanValues;
    std::vector<double> goodputValues;
    std::vector<double> latencyMeanValues;
    std::vector<double> dutyCycleValues;
    std::vector<double> kpiValues;
    
    uint32_t numRuns = 0;
    std::string line;
    
    // Parse each data row
    while (std::getline(inputFile, line)) {
        if (line.empty() || line[0] == '#') continue;
        
        std::istringstream iss(line);
        std::string token;
        std::vector<std::string> tokens;
        
        while (std::getline(iss, token, ',')) {
            tokens.push_back(token);
        }
        
        if (tokens.size() >= 22) {
            try {
                perValues.push_back(std::stod(tokens[8]));           // per_percent
                snrMeanValues.push_back(std::stod(tokens[9]));       // snr_mean_db
                rssiMeanValues.push_back(std::stod(tokens[12]));     // rssi_mean_dbm
                goodputValues.push_back(std::stod(tokens[15]));      // goodput_bps
                latencyMeanValues.push_back(std::stod(tokens[16]));  // latency_mean_s
                dutyCycleValues.push_back(std::stod(tokens[19]));    // duty_cycle_percent
                kpiValues.push_back(std::stod(tokens[21]));          // kpi
                numRuns++; // Increment the number of runs by one
            } catch (const std::exception& e) {
                std::cerr << "Warning: Error parsing line in per_run_metrics.csv: " << line << std::endl;
            }
        }
    }
    
    inputFile.close();
    
    if (numRuns == 0) {
        std::cerr << "Warning: No valid data found in per_run_metrics.csv" << std::endl;
        return;
    }
    
    // Calculate statistics for each metric
    Statistics perStats = CalculateStatistics(perValues);
    Statistics snrStats = CalculateStatistics(snrMeanValues);
    Statistics rssiStats = CalculateStatistics(rssiMeanValues);
    Statistics goodputStats = CalculateStatistics(goodputValues);
    Statistics latencyStats = CalculateStatistics(latencyMeanValues);
    Statistics dutyCycleStats = CalculateStatistics(dutyCycleValues);
    Statistics kpiStats = CalculateStatistics(kpiValues);
    
    // Write aggregated metrics to CSV file
    std::ofstream outputFile(aggregatedMetricsPath);
    if (!outputFile.is_open()) {
        std::cerr << "Error: Could not open aggregated_metrics.csv for writing: " << aggregatedMetricsPath << std::endl;
        return;
    }
    
    // Write CSV header
    outputFile << "metric_name,scenario,modulation,distance_m,num_runs,mean,std_dev,min,max,p25,p50,p75,p95" << std::endl;
    
    // Helper function to write one row of metrics to the CSV file
    auto writeMetricRow = [&](const std::string& metricName, const Statistics& stats, int precision) { // Capture by refernce
        outputFile << std::fixed << std::setprecision(precision);
        outputFile << metricName << "," << scenario << "," << modulation << "," << distance << "," << numRuns << ","
                   << stats.mean << "," << stats.stdDev << "," << stats.min << "," << stats.max << ","
                   << stats.p25 << "," << stats.p50 << "," << stats.p75 << "," << stats.p95 << std::endl;
    };
    
    // Write one row of metrics to the CSV file for each metric
    writeMetricRow("PER_percent", perStats, 4);
    writeMetricRow("SNR_mean_db", snrStats, 2);
    writeMetricRow("RSSI_mean_dbm", rssiStats, 2);
    writeMetricRow("goodput_bps", goodputStats, 2);
    writeMetricRow("latency_mean_s", latencyStats, 6);
    writeMetricRow("duty_cycle_percent", dutyCycleStats, 2);
    writeMetricRow("kpi", kpiStats, 4);
    
    outputFile.close();
}

// Write human-readable simulation summary to text file
void WriteSimulationSummary(const std::string& aggregatedMetricsPath, const std::string& summaryPath,
                            const std::string& scenario, const std::string& modulation,
                            double distance, uint32_t seed, double simulationTime,
                            uint32_t packetSize, uint32_t packetsPerSecond, uint32_t numRuns,
                            const std::string& outputDir,
                            uint32_t numWalls = 0, double wallSpacing = 0.0,
                            const std::string& buildingType = "", const std::string& externalWallType = "",
                            double temperature = 0.0, double noiseFigureDb = 0.0)
{
    std::ofstream file(summaryPath);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file for writing: " << summaryPath << std::endl;
        return;
    }
    
    // Extract timestamp from output directory path
    std::string timestampStr = "Unknown";
    size_t lastSlash = outputDir.find_last_of("/\\");
    if (lastSlash != std::string::npos) {
        std::string dirName = outputDir.substr(lastSlash + 1);
        if (dirName.length() == 15 && dirName[8] == '_') {
            timestampStr = dirName.substr(0, 4) + "-" + dirName.substr(4, 2) + "-" + dirName.substr(6, 2) + " " +
                           dirName.substr(9, 2) + ":" + dirName.substr(11, 2) + ":" + dirName.substr(13, 2);
        }
    }
    
    // If no timestamp found in directory name, use current time
    if (timestampStr == "Unknown") {
        time_t now = time(nullptr);
        char* timeStr = ctime(&now);
        if (timeStr) {
            size_t len = strlen(timeStr);
            if (len > 0 && timeStr[len-1] == '\n') {
                timeStr[len-1] = '\0';
            }
            timestampStr = timeStr;
        }
    }
    
    // === Write metadata section ===
    // Write timestamp
    file << "=== Simulation Summary ===" << std::endl;
    file << "Timestamp: " << timestampStr << std::endl;
    file << std::endl;

    // Write simulation parameters using helper function
    WriteSimulationParameters(file, scenario, modulation, distance, simulationTime,
                             packetSize, packetsPerSecond, 0.0, numRuns,
                             numWalls, wallSpacing, buildingType, externalWallType,
                             seed, temperature, noiseFigureDb);

    // === Write simulation results summary ===
    file << "=== Simulation Results Summary ===" << std::endl;
    
    // Read aggregated metrics and write results
    std::ifstream aggFile(aggregatedMetricsPath);
    if (!aggFile.is_open()) {
        file << "Error: Could not read aggregated metrics file." << std::endl;
        file.close();
        return;
    }
    
    // Skip header
    std::string header;
    std::getline(aggFile, header);
    
    // Parse each metric
    std::map<std::string, Statistics> metricsMap;
    std::string line;
    
    while (std::getline(aggFile, line)) {
        if (line.empty()) continue;
        
        std::istringstream iss(line);
        std::string token;
        std::vector<std::string> tokens;
        
        while (std::getline(iss, token, ',')) {
            tokens.push_back(token);
        }
        
        if (tokens.size() >= 13) {
            std::string metricName = tokens[0];
            Statistics stats;
            stats.mean = std::stod(tokens[5]);
            stats.stdDev = std::stod(tokens[6]);
            stats.min = std::stod(tokens[7]);
            stats.max = std::stod(tokens[8]);
            stats.p25 = std::stod(tokens[9]);
            stats.p50 = std::stod(tokens[10]);
            stats.p75 = std::stod(tokens[11]);
            stats.p95 = std::stod(tokens[12]);
            
            metricsMap[metricName] = stats;
        }
    }
    aggFile.close();
    
    // Write key metrics section
    file << "--- Key Metrics ---" << std::endl;
    
    if (metricsMap.count("PER_percent")) {
        Statistics& s = metricsMap["PER_percent"];
        file << "PER (Packet Error Rate):     " << std::fixed << std::setprecision(2) << s.mean << "% ± " 
             << std::setprecision(2) << s.stdDev << "%  (min: " << s.min << "%, max: " << s.max << "%)" << std::endl;
    }
    
    if (metricsMap.count("SNR_mean_db")) {
        Statistics& s = metricsMap["SNR_mean_db"];
        file << "SNR (Signal-to-Noise Ratio): " << std::setprecision(2) << s.mean << " dB ± " 
             << std::setprecision(2) << s.stdDev << " dB  (min: " << s.min << " dB, max: " << s.max << " dB)" << std::endl;
    }
    
    if (metricsMap.count("goodput_bps")) {
        Statistics& s = metricsMap["goodput_bps"];
        file << "Goodput:                     " << std::setprecision(2) << (s.mean / 1000.0) << " kbps ± " 
             << std::setprecision(2) << (s.stdDev / 1000.0) << " kbps  (min: " << (s.min / 1000.0) 
             << " kbps, max: " << (s.max / 1000.0) << " kbps)" << std::endl;
    }
    
    if (metricsMap.count("kpi")) {
        Statistics& s = metricsMap["kpi"];
        file << "KPI:                         " << std::setprecision(2) << s.mean << " ± " 
             << std::setprecision(2) << s.stdDev << "  (min: " << s.min << ", max: " << s.max << ")" << std::endl;
    }
    
    if (metricsMap.count("RSSI_mean_dbm")) {
        Statistics& s = metricsMap["RSSI_mean_dbm"];
        file << "RSSI:                        " << std::setprecision(2) << s.mean << " dBm ± " 
             << std::setprecision(2) << s.stdDev << " dBm  (min: " << s.min << " dBm, max: " << s.max << " dBm)" << std::endl;
    }
    
    if (metricsMap.count("duty_cycle_percent")) {
        Statistics& s = metricsMap["duty_cycle_percent"];
        file << "Duty Cycle:                  " << std::setprecision(2) << s.mean << "% ± " 
             << std::setprecision(2) << s.stdDev << "%  (min: " << s.min << "%, max: " << s.max << "%)" << std::endl;
    }
    
    file.close();
}

// ============================================================================
// TRACE CALLBACK FUNCTIONS
// ============================================================================

// Trace callback for packet transmission end
void PhyTxEndTrace(std::string context, Ptr<const Packet> packet)
{
    g_simMetrics.totalSent++;
    uint64_t packetId = packet->GetUid();
    
    // Create PacketMetrics entry
    PacketMetrics pm;
    pm.packetId = packetId;
    pm.txTime = Simulator::Now();
    pm.packetSize = packet->GetSize();
    
    // Get pathloss for this packet from queue (PathlossTrace should have been called before this)
    // PathlossTrace is called synchronously during transmission, so it should be in the queue
    if (!g_pathlossQueue.empty()) {
        pm.pathlossDb = g_pathlossQueue.front();
        g_pathlossQueue.pop();
    } else {
        // Fallback: use global pathloss (shouldn't happen in normal operation)
        pm.pathlossDb = g_pathlossDb;
    }

    g_packetMetrics[packetId] = pm;
}

// Trace callback for successful packet reception
// Uses the PER curve to decide if the packet should be marked as successful or failed
// (so it can mark it as failed even when PHY says it was successful)
void PhyRxEndOkTrace(std::string context, Ptr<const Packet> packet)
{   
    // Get packet ID
    uint64_t packetId = packet->GetUid();
    
    // === SNR CALCULATION ===
    double snrDb = 0.0;
    double pathlossDb = 0.0;
    double rssiDbm = 0.0;
    
    if (!g_modulation.empty()) {
        // Get TX power for the current modulation
        double txPowerW = GetTxPowerWatts(g_modulation);
        double txPowerDbm = ConvertWattsToDbm(txPowerW);
        
        // Get pathloss for this packet
        pathlossDb = g_packetMetrics[packetId].pathlossDb;
        
        // Calculate noise power using modulation-specific bandwidth
        double bandwidthHz = GetModulationBandwidthHz(g_modulation);
        double noisePsdValue = CalculateNoisePowerSpectralDensity(g_temperature, g_noiseFigureDb, g_backgroundInterferencePsd);
        double noisePowerW = noisePsdValue * bandwidthHz;
        double noisePowerDbm = ConvertWattsToDbm(noisePowerW);
        
        // Calculate SNR
        snrDb = CalculateSNR(txPowerDbm, pathlossDb, noisePowerDbm);
        
        // Calculate RSSI
        rssiDbm = txPowerDbm - pathlossDb;
    }
    
    // === PER CURVE CALCULATION ===
    bool shouldFail = false;
    
    // Calculate PER from per-packet SNR using modulation-specific curve
    double per = CalculatePERFromSNR(g_modulation, snrDb);
    
    // Generate a random number between 0 and 1
    Ptr<UniformRandomVariable> randomVar = CreateObject<UniformRandomVariable>();
    randomVar->SetAttribute("Min", DoubleValue(0.0));
    randomVar->SetAttribute("Max", DoubleValue(1.0));
    double randomValue = randomVar->GetValue();
    
    // Apply PER curve chance of failure
    if (randomValue < per) {
        shouldFail = true;
    }
    
    // Update metrics
    if (shouldFail) {
        g_simMetrics.totalLost++;
    }
    else {
        g_simMetrics.totalReceived++;
    }    
    
    // Update PacketMetrics entry
    PacketMetrics& pm = g_packetMetrics[packetId];
    pm.rxTime = Simulator::Now();
    pm.success = !shouldFail;
    pm.pathlossDb = pathlossDb;
    pm.snrDb = snrDb;
    pm.rssiDbm = rssiDbm;
}

// Trace callback for failed packet reception
void PhyRxEndErrorTrace(std::string context, Ptr<const Packet> packet)
{
    g_simMetrics.totalLost++;
    
    // Get packet ID
    uint64_t packetId = packet->GetUid();
    
    // Update existing PacketMetrics entry
    g_packetMetrics[packetId].success = false;
    g_packetMetrics[packetId].rxTime = Simulator::Now();
}

// Pathloss trace callback (captures pathloss calculated by NS-3)
void PathlossTrace(std::string context,
                   Ptr<const SpectrumPhy> txPhy,
                   Ptr<const SpectrumPhy> rxPhy,
                   double lossDb)
{
    // Store latest pathloss in global variable (used as a failsafe)
    g_pathlossDb = lossDb;
    
    // Store pathloss in queue 
    g_pathlossQueue.push(lossDb);
}

// ============================================================================
// SCENARIO SETUP FUNCTIONS
// ============================================================================

// Helper function to calculate S2 scenario distance
double CalculateS2Distance(uint32_t numWalls, double wallSpacing)
{
    double node0X = 0.0;
    double node1X = NODE_OFFSET + (numWalls - 1) * wallSpacing + NODE_OFFSET;
    return node1X - node0X;
}

void SetupScenarioS1(NodeContainer& nodes, SpectrumChannelHelper& channelHelper, double distance, double frequencyHz)
{
    // === MOBILITY SETUP ===
    // Set up constant position mobility model for fixed node positions
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(nodes);
    
    // === NODE POSITIONING ===
    // Node 0 at origin, Node 1 at specified distance along x-axis
    Ptr<MobilityModel> mob0 = nodes.Get(0)->GetObject<MobilityModel>();
    Ptr<MobilityModel> mob1 = nodes.Get(1)->GetObject<MobilityModel>();
    
    mob0->SetPosition(Vector(0.0, 0.0, NODE_HEIGHT));
    mob1->SetPosition(Vector(distance, 0.0, NODE_HEIGHT));
    
    // === PROPAGATION LOSS MODEL ===
    Ptr<FriisPropagationLossModel> lossModel = CreateObject<FriisPropagationLossModel>();
    lossModel->SetFrequency(frequencyHz);
    lossModel->SetSystemLoss(SYSTEM_LOSS);
    channelHelper.AddPropagationLoss(lossModel);
}

void SetupScenarioS2(NodeContainer& nodes, SpectrumChannelHelper& channelHelper, uint32_t numWalls, double wallSpacing, const std::string& buildingType, const std::string& externalWallType, double frequencyHz)
{
    // === BUILDING DIMENSIONS ===
    // Calculate building dimensions based on number of walls and spacing
    // Number of rooms = number of walls + 1
    uint32_t numRooms = numWalls + 1;
    double roomWidth = wallSpacing;
    double buildingWidth = numRooms * roomWidth;
    
    // === BUILDING SETUP ===
    // Create building with N+1 rooms (N walls) arranged in a line along x-axis
    Ptr<Building> building = CreateObject<Building>();
    building->SetBoundaries(Box(0.0, buildingWidth, -BUILDING_DEPTH/2.0, BUILDING_DEPTH/2.0, 0.0, BUILDING_HEIGHT));
    
    // Set building type
    if (buildingType == "office") {
        building->SetBuildingType(Building::Office);
    } else if (buildingType == "commercial") {
        building->SetBuildingType(Building::Commercial);
    } else {
        building->SetBuildingType(Building::Residential);
    }
    
    // Set external wall type
    if (externalWallType == "concrete-without-windows") {
        building->SetExtWallsType(Building::ConcreteWithoutWindows);
    } else if (externalWallType == "stone-blocks") {
        building->SetExtWallsType(Building::StoneBlocks);
    } else {
        building->SetExtWallsType(Building::ConcreteWithWindows);
    }
    
    building->SetNFloors(1);
    building->SetNRoomsX(numRooms);
    building->SetNRoomsY(1);
    
    // === MOBILITY SETUP ===
    // Set up constant position mobility model for fixed node positions
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(nodes);
    
    // === NODE POSITIONING ===
    // Position nodes in first and last room along x-axis
    Ptr<MobilityModel> mob0 = nodes.Get(0)->GetObject<MobilityModel>();
    Ptr<MobilityModel> mob1 = nodes.Get(1)->GetObject<MobilityModel>();
    
    double node0X = 0.0;
    double node1X = NODE_OFFSET + (numWalls - 1) * wallSpacing + NODE_OFFSET;
    
    mob0->SetPosition(Vector(node0X, 0.0, NODE_HEIGHT));
    mob1->SetPosition(Vector(node1X, 0.0, NODE_HEIGHT));
    
    // === BUILDING ASSOCIATION ===
    // Install building information on nodes
    BuildingsHelper::Install(nodes);
    
    // === PROPAGATION LOSS MODEL ===
    // Use HybridBuildingsPropagationLossModel for indoor NLOS with wall penetration
    Ptr<HybridBuildingsPropagationLossModel> lossModel = CreateObject<HybridBuildingsPropagationLossModel>();
    lossModel->SetFrequency(frequencyHz);
    channelHelper.AddPropagationLoss(lossModel);
}

// ============================================================================
// SCENARIO PRINTING FUNCTIONS
// ============================================================================

void PrintScenarioS1Info(double distance)
{
    std::cout << "\n--- S1 Scenario Layout ---" << std::endl;
    std::cout << "Line-of-Sight: No obstacles, free space propagation" << std::endl;
    std::cout << std::endl;
    std::cout << "Node 0 --------(" << distance << "m)-------- Node 1" << std::endl;
    std::cout << std::endl;
}

void PrintScenarioS2Info(uint32_t numWalls, double wallSpacing, uint32_t numRooms, const std::string& buildingType, const std::string& externalWallType)
{
    std::cout << "\n--- S2 Building Layout ---" << std::endl;

    std::cout << "Building type: " << buildingType << std::endl;
    std::cout << "External wall type: " << externalWallType << std::endl;
    std::cout << std::endl;

    // Node 0 and distance to first wall
    std::cout << "Node 0";
    double distanceToFirstWall = NODE_OFFSET;
    std::cout << "---(" << std::fixed << std::setprecision(1) << distanceToFirstWall << " m)---";
    
    // Walls and distances between them
    for (uint32_t i = 0; i < numWalls - 1; i++) {
        std::cout << "|";
        
        // Distance between walls
        std::cout << "-----(" << std::fixed << std::setprecision(1) << wallSpacing << " m)-----";
    }

    // Node 1 and distance from last wall
    std::cout << "|---(" << std::fixed << std::setprecision(1) << NODE_OFFSET << " m)---";
    std::cout << "Node 1" << std::endl;
    std::cout << std::endl;
}

// ============================================================================
// MODULATION SETUP FUNCTIONS
// ============================================================================

// Common modulation setup code shared by all modulation schemes
// Sets up spectrum model, TX power, noise, PHY layer, and installs devices
void SetupModulationCommon(NodeContainer& nodes, Ptr<SpectrumChannel> channel, 
                           double txPowerW, const std::string& dataRate, NetDeviceContainer& devices)
{
    SpectrumValue5MhzFactory sf;
    
    // Create TX power spectral density
    Ptr<SpectrumValue> txPsd = sf.CreateTxPowerSpectralDensity(txPowerW, 1);
    
    // Calculate and create noise power spectral density
    double noisePsdValue = CalculateNoisePowerSpectralDensity(g_temperature, g_noiseFigureDb, g_backgroundInterferencePsd);
    Ptr<SpectrumValue> noisePsd = sf.CreateConstant(noisePsdValue);
    
    // Set up PHY layer
    AdhocAlohaNoackIdealPhyHelper deviceHelper;
    deviceHelper.SetChannel(channel);
    deviceHelper.SetTxPowerSpectralDensity(txPsd);
    deviceHelper.SetNoisePowerSpectralDensity(noisePsd);
    deviceHelper.SetPhyAttribute("Rate", DataRateValue(DataRate(dataRate)));
    
    // Install devices
    devices = deviceHelper.Install(nodes);
}

void SetupModulationLoRa(NodeContainer& nodes, Ptr<SpectrumChannel> channel, NetDeviceContainer& devices)
{
    SetupModulationCommon(nodes, channel, LORA_TX_POWER_W, "5.5kbps", devices);
}

void SetupModulationDectNrPlus(NodeContainer& nodes, Ptr<SpectrumChannel> channel, NetDeviceContainer& devices)
{
    SetupModulationCommon(nodes, channel, DECT_NR_PLUS_TX_POWER_W, "2Mbps", devices);
}

void SetupModulationWifiHalow(NodeContainer& nodes, Ptr<SpectrumChannel> channel, NetDeviceContainer& devices)
{
    SetupModulationCommon(nodes, channel, WIFI_HALOW_TX_POWER_W, "150kbps", devices);
}

// ============================================================================
// MAIN FUNCTION
// ============================================================================

int main(int argc, char *argv[])
{
    // ============================================================================
    // GET VALUES FROM CONFIG FILE AND COMMAND LINE ARGUMENTS
    // ============================================================================
    
    // === Initialize with default values ===
    std::string scenario = DEFAULT_SCENARIO;
    std::string modulation = DEFAULT_MODULATION;
    double distance = DEFAULT_S1_DISTANCE;
    std::string outputDir = DEFAULT_OUTPUT_DIR;
    uint32_t numRuns = DEFAULT_NUM_RUNS;
    uint32_t numWalls = DEFAULT_NUM_WALLS;
    double wallSpacing = DEFAULT_WALL_SPACING;
    std::string buildingType = DEFAULT_BUILDING_TYPE;
    std::string externalWallType = DEFAULT_EXTERNAL_WALL_TYPE;
    double payloadPercentage = DEFAULT_PAYLOAD_PERCENTAGE;
    double simulationTime = DEFAULT_SIMULATION_TIME;
    double temperature = DEFAULT_TEMPERATURE_KELVIN;
    double noiseFigureDb = DEFAULT_NOISE_FIGURE_DB;
    double backgroundInterferencePsd = DEFAULT_BACKGROUND_INTERFERENCE_PSD;
    
    // Traffic parameters (defaults handled in CalculateTrafficParameters())
    double packetRate = 0.0;
    double packetSize = 0.0;
    double dutyCycle = 0.0;
    
    std::string configFile = "";
    
    // === Parse --config first to get config file path ===
    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "--config" && i + 1 < argc) {
            configFile = argv[i + 1];
            break;
        }
    }
    
    // === Load config file (config file overrides defaults) ===
    std::map<std::string, std::string> configFileValues;
    if (!configFile.empty() && ParseConfigFile(configFile, configFileValues)) {
        ApplyConfigString(configFileValues, "scenario", scenario);
        ApplyConfigString(configFileValues, "modulation", modulation);
        ApplyConfigDouble(configFileValues, "distance", distance);
        ApplyConfigString(configFileValues, "output-dir", outputDir);
        ApplyConfigUint(configFileValues, "runs", numRuns);
        ApplyConfigUint(configFileValues, "walls", numWalls);
        ApplyConfigDouble(configFileValues, "wall-spacing", wallSpacing);
        ApplyConfigString(configFileValues, "building-type", buildingType);
        ApplyConfigString(configFileValues, "external-wall-type", externalWallType);
        ApplyConfigDouble(configFileValues, "packet-rate", packetRate);
        ApplyConfigDouble(configFileValues, "packet-size", packetSize);
        ApplyConfigDouble(configFileValues, "duty-cycle", dutyCycle);
        ApplyConfigDouble(configFileValues, "payload-percentage", payloadPercentage);
        ApplyConfigDouble(configFileValues, "sim-time", simulationTime);
        ApplyConfigDouble(configFileValues, "temperature", temperature);
        ApplyConfigDouble(configFileValues, "noise-figure-db", noiseFigureDb);
        ApplyConfigDouble(configFileValues, "background-interference-psd", backgroundInterferencePsd);
    }
    
    // === Parse command line arguments (command-line overrides config file) ===
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg.find("--") != 0 || i + 1 >= argc) continue;
        
        std::string key = arg.substr(2);
        std::string value = argv[++i];
        
        // Skip --config (already handled)
        if (key == "config") continue;
        
        // Apply values
        if (key == "scenario") scenario = value;
        else if (key == "modulation") modulation = value;
        else if (key == "distance") distance = std::stod(value);
        else if (key == "output-dir") outputDir = value;
        else if (key == "runs") numRuns = std::stoul(value);
        else if (key == "walls") numWalls = std::stoul(value);
        else if (key == "wall-spacing") wallSpacing = std::stod(value);
        else if (key == "building-type") buildingType = value;
        else if (key == "external-wall-type") externalWallType = value;
        else if (key == "packet-rate") packetRate = std::stod(value);
        else if (key == "packet-size") packetSize = std::stod(value);
        else if (key == "duty-cycle") dutyCycle = std::stod(value);
        else if (key == "payload-percentage") payloadPercentage = std::stod(value);
        else if (key == "sim-time") simulationTime = std::stod(value);
        else if (key == "temperature") temperature = std::stod(value);
        else if (key == "noise-figure-db") noiseFigureDb = std::stod(value);
        else if (key == "background-interference-psd") backgroundInterferencePsd = std::stod(value);
    }
    
    // ============================================================================
    // PARAMETER VALIDATION AND HANDLING
    // ============================================================================
    
    // Validate scenario
    if (scenario != "S1" && scenario != "S2") {
        std::cerr << "Error: Invalid scenario: " << scenario << std::endl;
        std::cerr << "Valid scenarios: S1 (LOS), S2 (NLOS with walls)" << std::endl;
        return 1;
    }
    
    // Validate modulation
    if (modulation != "lora" && modulation != "dect-nr+" && modulation != "wifi-halow") {
        std::cerr << "Error: Invalid modulation: " << modulation << std::endl;
        std::cerr << "Valid modulations: lora, dect-nr+, wifi-halow" << std::endl;
        return 1;
    }
    
    // Validate building type (only for S2, but the check will pass for S1 as it was set to default)
    if (buildingType != "residential" && buildingType != "office" && buildingType != "commercial") {
        std::cerr << "Error: Invalid building type: " << buildingType << std::endl;
        std::cerr << "Valid building types: residential, office, commercial" << std::endl;
        return 1;
    }
    
    // Validate external wall type (only for S2, but check anyway for early error detection)
    if (externalWallType != "concrete-with-windows" && 
        externalWallType != "concrete-without-windows" && 
        externalWallType != "stone-blocks") {
        std::cerr << "Error: Invalid external wall type: " << externalWallType << std::endl;
        std::cerr << "Valid external wall types: concrete-with-windows, concrete-without-windows, stone-blocks" << std::endl;
        return 1;
    }
    
    // Validate scenario-specific parameters
    if (scenario == "S1") {
        // Validate distance
        if (distance <= 0.0) {
            std::cerr << "Error: Distance for S1 scenario must be > 0 (is " << distance << " m)" << std::endl;
            return 1;
        }
    } else if (scenario == "S2") {
        // Validate number of walls
        if (numWalls < 1) {
            std::cerr << "Error: Number of walls for S2 scenario must be >= 1 (is " << numWalls << ")" << std::endl;
            return 1;
        }
        // Validate wall spacing
        if (wallSpacing <= 0.0) {
            std::cerr << "Error: Wall spacing for S2 scenario must be > 0 (is " << wallSpacing << " m)" << std::endl;
            return 1;
        }
    }
    
    // Validate simulation parameters
    if (numRuns == 0) {
        std::cerr << "Error: Number of runs must be > 0 (is " << numRuns << ")" << std::endl;
        return 1;
    }
    
    if (simulationTime <= 0.0) {
        std::cerr << "Error: Simulation time must be > 0 (is " << simulationTime << " s)" << std::endl;
        return 1;
    }
    
    // Validate noise parameters
    if (temperature <= 0.0) {
        std::cerr << "Error: Temperature must be > 0 (is " << temperature << " K)" << std::endl;
        return 1;
    }

    if (noiseFigureDb < 0.0) {
        std::cerr << "Error: Noise figure must be >= 0 (is " << noiseFigureDb << " dB)" << std::endl;
        return 1;
    }
    
    if (backgroundInterferencePsd < 0.0) {
        std::cerr << "Error: Background interference PSD must be >= 0 (is " << backgroundInterferencePsd << " W/Hz)" << std::endl;
        return 1;
    }
    
    // Calculate and validate traffic parameters
    if (!CalculateTrafficParameters(modulation, packetRate, packetSize, dutyCycle)) {
        std::cerr << "Error: Failed to calculate traffic parameters" << std::endl;
        return 1;
    }
    
    // Validate payload percentage
    if (payloadPercentage < 0.0 || payloadPercentage > 100.0) {
        std::cerr << "Error: Payload percentage must be between 0 and 100 (is " << payloadPercentage << "%)" << std::endl;
        return 1;
    }
    
    // Convert to appropriate types (already rounded in CalculateTrafficParameters())
    uint32_t packetSizeUint = static_cast<uint32_t>(packetSize);
    uint32_t packetsPerSecondUint = static_cast<uint32_t>(packetRate);
    
    // Set global noise parameters for use in modulation setup functions
    g_temperature = temperature;
    g_noiseFigureDb = noiseFigureDb;
    g_backgroundInterferencePsd = backgroundInterferencePsd;

    // ============================================================================
    // SIMULATION PARAMETERS DISPLAY
    // ============================================================================
    
    // Calculate node distance based on scenario
    double nodeDistance;
    if (scenario == "S1") {
        nodeDistance = distance;
    } else {
        nodeDistance = CalculateS2Distance(numWalls, wallSpacing);
    }
    
    std::cout << std::endl;
    std::cout << "=== Modulation Comparison Simulation ===" << std::endl;
    if (!configFile.empty()) {
        std::cout << "Config file: " << configFile << std::endl;
    }
    
    // Write simulation parameters using helper function
    WriteSimulationParameters(std::cout, scenario, modulation, nodeDistance, simulationTime,
                               packetSizeUint, packetsPerSecondUint, dutyCycle, numRuns,
                               numWalls, wallSpacing, buildingType, externalWallType,
                               0, temperature, noiseFigureDb);
    
    std::cout << "=========================================" << std::endl;
    
    // ============================================================================
    // OUTPUT FILE PATHS AND PER-RUN METRICS INITIALIZATION
    // ============================================================================

    std::string perRunMetricsPath = outputDir + "/per_run_metrics.csv";
    std::string aggregatedMetricsPath = outputDir + "/aggregated_metrics.csv";
    std::string summaryPath = outputDir + "/simulation_summary.txt";
    
    // Initialize per-run metrics file with header (will append for each run)
    std::ofstream initFile(perRunMetricsPath, std::ios::out);
    if (initFile.is_open()) {
        initFile << "run,scenario,modulation,distance_m,seed,total_sent,total_received,total_lost,"
                 << "per_percent,snr_mean_db,snr_min_db,snr_max_db,"
                 << "rssi_mean_dbm,rssi_min_dbm,rssi_max_dbm,"
                 << "goodput_bps,latency_mean_s,latency_min_s,latency_max_s,"
                 << "duty_cycle_percent,data_rate_bps,kpi" << std::endl;
        initFile.close();
    }
        
    // ============================================================================
    // SCENARIO LAYOUT PRINTING
    // ============================================================================

    if (scenario == "S1") {
        PrintScenarioS1Info(nodeDistance);
    } else if (scenario == "S2") {
        // For S2, calculate the layout info
        uint32_t numRooms = numWalls + 1;
        PrintScenarioS2Info(numWalls, wallSpacing, numRooms, buildingType, externalWallType);
    }
    
    // ============================================================================
    // MAIN SIMULATION RUNS LOOP
    // ============================================================================

    // Set global modulation once (constant across all runs)
    g_modulation = modulation;
    
    // Generate base seed from current time for reproducibility
    uint32_t baseSeed = (uint32_t)(time(nullptr));
    
    for (uint32_t run = 1; run <= numRuns; run++) {

        // ============================================================================
        // RUN SETUP
        // ============================================================================

        std::cout << "\n=== Run " << run << " of " << numRuns << " ===" << std::endl;
        
        // === SEED SETUP ===
        // Generate unique seed for this run (incremented by run number)
        uint32_t seed = baseSeed + run;
        RngSeedManager::SetSeed(seed);
        RngSeedManager::SetRun(run);
        std::cout << "Random seed: " << seed << " (run " << run << ")" << std::endl;
        
        // === NODE SETUP ===
        // Create 2 nodes
        NodeContainer nodes;
        nodes.Create(2); 
        
        // === CHANNEL SETUP ===
        // Create a spectrum channel
        SpectrumChannelHelper channelHelper;
        channelHelper.SetChannel("ns3::MultiModelSpectrumChannel");
        channelHelper.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
        
        // === SCENARIO SETUP ===
        // Get modulation-specific frequency for propagation model
        double frequencyHz = GetModulationFrequencyHz(modulation);
        if (scenario == "S1") {
            SetupScenarioS1(nodes, channelHelper, nodeDistance, frequencyHz);
        } else if (scenario == "S2") {
            SetupScenarioS2(nodes, channelHelper, numWalls, wallSpacing, buildingType, externalWallType, frequencyHz);
        }

        // === CHANNEL CREATION ===
        // Create the channel after scenario setup (which adds propagation loss models)
        Ptr<SpectrumChannel> channel = channelHelper.Create();
        
        // === MODULATION SETUP ===
        // Create net devices
        NetDeviceContainer devices;
        if (modulation == "lora") {
            SetupModulationLoRa(nodes, channel, devices);
        } else if (modulation == "dect-nr+") {
            SetupModulationDectNrPlus(nodes, channel, devices);
        } else if (modulation == "wifi-halow") {
            SetupModulationWifiHalow(nodes, channel, devices);
        }

        // ============================================================================
        // TRAFFIC GENERATION
        // ============================================================================

        // Install PacketSocket on all nodes
        // PacketSocket is used to send packets directly between nodes
        PacketSocketHelper packetSocket;
        packetSocket.Install(nodes);
        
        // Configure PacketSocket addresses
        PacketSocketAddress socket;
        socket.SetSingleDevice(devices.Get(0)->GetIfIndex());
        socket.SetPhysicalAddress(devices.Get(1)->GetAddress());
        socket.SetProtocol(1); // Arbitrary protocol number for L2 traffic
        
        // OnOffHelper for Constant Bit Rate traffic
        OnOffHelper onoff("ns3::PacketSocketFactory", Address(socket));
        onoff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]")); // ON 100% of the time
        onoff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]")); // OFF 0% of the time
        // Set it to send packets at the rate specified by the user (this schedules it using the ns3 scheduler)
        onoff.SetAttribute("DataRate", DataRateValue(DataRate(packetsPerSecondUint * packetSizeUint * 8))); // Data rate in bits per second
        onoff.SetAttribute("PacketSize", UintegerValue(packetSizeUint)); // Packet size in bytes
        
        ApplicationContainer apps = onoff.Install(nodes.Get(0));

        apps.Start(Seconds(1.0));  // Start after a short delay (1 second)
        apps.Stop(Seconds(simulationTime + 1.0));  // Stop after simulationTime seconds of actual traffic
        
        // ============================================================================
        // METRICS COLLECTION SETUP
        // ============================================================================

        g_simMetrics = SimulationMetrics(); // Initialize new metrics for this run
        g_packetMetrics.clear(); // Clear packet metrics map
        while (!g_pathlossQueue.empty()) {
            g_pathlossQueue.pop(); // Clear pathloss queue
        }
        
        // Assign callback functions
        Config::Connect("/NodeList/*/DeviceList/*/Phy/TxEnd", MakeCallback(&PhyTxEndTrace));
        Config::Connect("/NodeList/*/DeviceList/*/Phy/RxEndOk", MakeCallback(&PhyRxEndOkTrace));
        Config::Connect("/NodeList/*/DeviceList/*/Phy/RxEndError", MakeCallback(&PhyRxEndErrorTrace));
        Config::Connect("/ChannelList/*/$ns3::SpectrumChannel/PathLoss", MakeCallback(&PathlossTrace));
        
        // ============================================================================
        // SIMULATION EXECUTION
        // ============================================================================

        // Run simulation for specified duration
        Simulator::Stop(Seconds(simulationTime + 2.0));  // Stop after simulationTime + 2s (1s warm-up + 1s for traffic to complete)
        Simulator::Run();
        
        // ============================================================================
        // METRICS ALCULATIONS
        // ============================================================================

        // === PER CALCULATION ===
        g_simMetrics.totalLost = g_simMetrics.totalSent - g_simMetrics.totalReceived;
        if (g_simMetrics.totalSent > 0) {
            g_simMetrics.per = ((double)g_simMetrics.totalLost / g_simMetrics.totalSent) * 100.0;
        }
        
        // === SNR AND RSSI CALCULATION ===
        // Get TX power for the current modulation scheme
        double txPowerW = GetTxPowerWatts(modulation);
        double txPowerDbm = ConvertWattsToDbm(txPowerW);
        
        // Calculate noise power from noise PSD and bandwidth
        // Noise is calculated from physical constants (Boltzmann, temperature, noise figure)
        double bandwidthHz = GetModulationBandwidthHz(modulation);
        double noisePsdValue = CalculateNoisePowerSpectralDensity(g_temperature, g_noiseFigureDb, g_backgroundInterferencePsd);
        double noisePowerW = noisePsdValue * bandwidthHz;
        double noisePowerDbm = ConvertWattsToDbm(noisePowerW);
        
        // === PACKET-LEVEL METRICS CALCULATION ===
        std::vector<double> snrValues;
        std::vector<double> rssiValues;
        std::vector<double> pathlossValues;
        std::vector<double> latencyValues;
        
        for (auto& pair : g_packetMetrics) {
            PacketMetrics& pm = pair.second;
            
            // Calculate RSSI and SNR for this packet using its stored pathloss
            pm.rssiDbm = txPowerDbm - pm.pathlossDb;
            pm.snrDb = pm.rssiDbm - noisePowerDbm;
            
            // Collect statistics only for successfully received packets
            if (pm.success) {
                snrValues.push_back(pm.snrDb);
                rssiValues.push_back(pm.rssiDbm);
                pathlossValues.push_back(pm.pathlossDb);
                
                // Calculate latency for successfully received packets
                if (pm.txTime > Time(0) && pm.rxTime > Time(0)) {
                    double latency = (pm.rxTime - pm.txTime).GetSeconds();
                    pm.latency = latency;
                    latencyValues.push_back(latency);
                }
            }
        }
        
        // === AGGREGATE STATISTICS CALCULATION ===

        // Calculate mean, min, max for SNR
        if (!snrValues.empty()) {
            g_simMetrics.snrMean = std::accumulate(snrValues.begin(), snrValues.end(), 0.0) / snrValues.size();
            g_simMetrics.snrMin = *std::min_element(snrValues.begin(), snrValues.end());
            g_simMetrics.snrMax = *std::max_element(snrValues.begin(), snrValues.end());
        }
        
        // Calculate mean, min, max for RSSI
        if (!rssiValues.empty()) {
            g_simMetrics.rssiMean = std::accumulate(rssiValues.begin(), rssiValues.end(), 0.0) / rssiValues.size();
            g_simMetrics.rssiMin = *std::min_element(rssiValues.begin(), rssiValues.end());
            g_simMetrics.rssiMax = *std::max_element(rssiValues.begin(), rssiValues.end());
        }
        
        // Calculate mean, min, max for latency
        if (!latencyValues.empty()) {
            g_simMetrics.latencyMean = std::accumulate(latencyValues.begin(), latencyValues.end(), 0.0) / latencyValues.size();
            g_simMetrics.latencyMin = *std::min_element(latencyValues.begin(), latencyValues.end());
            g_simMetrics.latencyMax = *std::max_element(latencyValues.begin(), latencyValues.end());
        }
        
        // Calculate mean, min, max for pathloss
        if (!pathlossValues.empty()) {
            g_simMetrics.pathlossMean = std::accumulate(pathlossValues.begin(), pathlossValues.end(), 0.0) / pathlossValues.size();
            g_simMetrics.pathlossMin = *std::min_element(pathlossValues.begin(), pathlossValues.end());
            g_simMetrics.pathlossMax = *std::max_element(pathlossValues.begin(), pathlossValues.end());
        }
        
        // === THROUGHPUT METRICS CALCULATION ===

        // Calculate goodput (in bits per second)
        double payloadSizeBytes = packetSizeUint * (payloadPercentage / 100.0);
        if (simulationTime > 0) {
            g_simMetrics.goodput = (g_simMetrics.totalReceived * payloadSizeBytes * 8.0) / simulationTime;
        }
        
        // Calculate data rate (in bits per second)
        g_simMetrics.dataRate = packetsPerSecondUint * packetSizeUint * 8.0;
        
        // Calculate duty cycle in percentage
        double packetTxDuration = (packetSizeUint * 8.0) / g_simMetrics.dataRate;  // seconds per packet
        double totalTxTime = g_simMetrics.totalSent * packetTxDuration;
        g_simMetrics.dutyCycle = (totalTxTime / simulationTime) * 100.0;
        
        // === KPI CALCULATION ===
        g_simMetrics.kpi = CalculateKPI(g_simMetrics.per, g_simMetrics.goodput, g_simMetrics.dataRate, g_simMetrics.snrMean);
    
        // ============================================================================
        // PRINT RUN RESULTS (only short summary)
        // ============================================================================

        std::cout << "Run " << run << " Results: PER=" << std::fixed << std::setprecision(2) 
                  << g_simMetrics.per << "%, SNR=" << g_simMetrics.snrMean << " dB, "
                  << "Goodput=" << (g_simMetrics.goodput / 1000.0) << " kbps" << std::endl;
        
        // ============================================================================
        // WRITE PER-RUN METRICS
        // ============================================================================
        
        // Append metrics to CSV file (header already written before loop)
        WritePerRunMetrics(perRunMetricsPath, g_simMetrics, scenario, modulation, 
                          nodeDistance, seed, run);
    
        // ============================================================================
        // CLEANUP (for next run)
        // ============================================================================
        Simulator::Destroy();
    }
    
    // ============================================================================
    // FINAL OUTPUT GENERATION (after all runs)
    // ============================================================================
    WriteAggregatedMetrics(perRunMetricsPath, aggregatedMetricsPath, scenario, modulation, nodeDistance);
    WriteSimulationSummary(aggregatedMetricsPath, summaryPath, scenario, modulation, nodeDistance, 
                          baseSeed, simulationTime, packetSizeUint, packetsPerSecondUint, numRuns, outputDir,
                          numWalls, wallSpacing, buildingType, externalWallType, temperature, noiseFigureDb);
    return 0;
}


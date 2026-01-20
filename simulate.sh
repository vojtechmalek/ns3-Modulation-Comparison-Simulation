#!/bin/bash

# ============================================================================
# DEFAULT VALUES
# ============================================================================

# Default config file (can be overridden with --config)
CONFIG_FILE="config.example.txt"

# Debug mode
DEBUG=false

#============================================================================
# COMMAND LINE ARGUMENTS
#============================================================================

CMD_ARGS=""

while [[ $# -gt 0 ]]; do
    case $1 in
        --config)
            if [ $# -lt 2 ] || [ -z "$2" ]; then
                echo "Error: --config requires a file path"
                echo "Usage: --config FILE"
                exit 1
            fi
            CONFIG_FILE="$2"
            shift 2
            ;;
        --output-dir)
            if [ $# -lt 2 ] || [ -z "$2" ]; then
                echo "Error: --output-dir requires a directory path"
                echo "Usage: --output-dir DIR"
                exit 1
            fi
            OUTPUT_DIR="$2"
            shift 2
            ;;
        --debug)
            DEBUG=true
            shift
            ;;
        --help)
            echo "Usage: $0 [options]"
            echo ""
            echo "Bash-specific options:"
            echo "  --output-dir DIR       Output directory for CSV (default: auto-generated timestamped directory)"
            echo "  --config FILE          Configuration file path (default: config.example.txt)"
            echo "  --debug                Enable debug mode (default: false)"
            echo ""
            echo "Simulation parameters (To override config file values):"
            echo "  --scenario SCENARIO        Scenario: S1 (LOS), S2 (NLOS with walls) (default: S1)"
            echo "  --modulation MOD           Modulation: lora, dect-nr+, wifi-halow (default: lora)"
            echo "  --runs N                   Number of simulation runs (default: 30)"
            echo "  --distance N               Distance for S1 (LOS) in meters (default: 20)"
            echo "  --walls N                  Number of walls for S2 (default: 1)"
            echo "  --wall-spacing N           Spacing between walls for S2 in meters (default: 5)"
            echo "  --building-type TYPE       Building type for S2: residential, office, commercial (default: residential)"
            echo "  --external-wall-type TYPE  External wall type for S2: concrete-with-windows, concrete-without-windows, stone-blocks (default: concrete-with-windows)"
            echo "  --packet-size N            Packet size in bytes (default: 100)"
            echo "  --packet-rate N            Packets per second (default: 60)"
            echo "  --duty-cycle N             Duty cycle percentage (default: 1)"
            echo "  --sim-time N               Simulation time in seconds (default: 10)"
            echo "  --temperature N            Temperature in Kelvin (default: 290)"
            echo "  --noise-figure-db N        Noise figure in dB (default: 10)"
            echo "  --background-interference-psd N  Background interference PSD in W/Hz (default: 4e-19)"
            echo ""
            echo "Examples:"
            echo "  $0"
            echo "  $0 --scenario S1 --modulation lora --distance 20 (overrides default config file values)"
            echo "  $0 --config config.txt --output-dir my_outputs (runs simulation with custom config file and output directory)"
            echo "  $0 --config config.txt --modulation dect-nr+ (runs simulation with custom config file, but overrides to use DECT NR+ modulation)"
            exit 0
            ;;
        --*)
            # Pass through all other arguments to modulation_comparison.cc
            # But normalize both --arg value and --arg=value formats to space-separated
            if [[ "$1" == *"="* ]]; then
                ARG_NAME="${1%%=*}"  # Extract name
                ARG_VALUE="${1#*=}"  # Extract value
                CMD_ARGS="$CMD_ARGS $ARG_NAME $ARG_VALUE"
                shift
            else
                # Only pass through if argument has a value
                if [ $# -gt 1 ] && [[ ! "$2" =~ ^-- ]]; then
                    CMD_ARGS="$CMD_ARGS $1 $2"
                    shift 2
                else
                    echo "Warning: Argument '$1' requires a value but none provided, skipping it."
                    shift
                fi
            fi
            ;;
        *)
            echo "Error: Unexpected argument '$1'"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# ============================================================================
# OUTPUT DIRECTORY SETUP
# ============================================================================

# Create output directory (timestamped if not specified)
if [ -z "$OUTPUT_DIR" ]; then
    TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
    OUTPUT_DIR="outputs/${TIMESTAMP}"
fi
mkdir -p "$OUTPUT_DIR" # Create the output directory

# ============================================================================
# SIMULATION STARTUP MESSAGE
# ============================================================================

echo "Starting simulation..."

# ============================================================================
# DOCKER BUILD
# ============================================================================

# Check if Docker image exists, if not build it
# If debug is enabled, show the build process
if ! docker image inspect ns3-modulation-comparison >/dev/null 2>&1; then
    if [ "$DEBUG" = "true" ]; then
        echo "Docker image 'ns3-modulation-comparison' not found. Building..."
    fi
    
    if [ "$DEBUG" = "true" ]; then
        docker build -t ns3-modulation-comparison .
    else
        docker build -t ns3-modulation-comparison . >/dev/null 2>&1 # Build the Docker image silently
    fi
    
    if [ $? -ne 0 ]; then
        echo "Error: Failed to build Docker image"
        exit 1
    fi
    
    if [ "$DEBUG" = "true" ]; then
        echo "Docker image built successfully!"
    fi
fi

# ============================================================================
# DOCKER RUN
# ============================================================================

if [ "$DEBUG" = "true" ]; then
    echo "Starting simulation in Docker container..."
fi

# === BUILD COMMAND ARGUMENTS ===

# Convert OUTPUT_DIR to absolute path inside Docker container
# Add the output directory to the command arguments (normalized to space-separated format)
OUTPUT_DIR="/workspace/$OUTPUT_DIR"
CMD_ARGS="$CMD_ARGS --output-dir $OUTPUT_DIR"

# Convert CONFIG_FILE - handle both relative and absolute paths
# Store original path for copying
CONFIG_FILE_ORIG="$CONFIG_FILE"
# Get basename for the config argument
CONFIG_FILE_BASENAME=$(basename "$CONFIG_FILE")
CMD_ARGS="$CMD_ARGS --config $CONFIG_FILE_BASENAME"

# Run simulation in Docker (output streams in real-time)
docker run --rm \
    -v "$(pwd)":/workspace \
    -w /workspace \
    -e OUTPUT_DIR="/workspace/$OUTPUT_DIR" \
    -e CONFIG_FILE_ORIG="$CONFIG_FILE_ORIG" \
    ns3-modulation-comparison \
    bash -c "
        [ \"$DEBUG\" = \"true\" ] && echo 'Setting up workspace...'
        cd /workspace
        
        [ \"$DEBUG\" = \"true\" ] && echo 'Creating output directory...'
        mkdir -p $OUTPUT_DIR
        
        [ \"$DEBUG\" = \"true\" ] && echo 'Creating NS-3 scratch directory...'
        NS3_SCRATCH_DIR=\"/opt/ns-3-dev/scratch\"
        
        [ \"$DEBUG\" = \"true\" ] && echo 'Copying modulation_comparison.cc to NS-3 scratch directory...'
        cp modulation_comparison.cc \"\$NS3_SCRATCH_DIR/\"
        
        if [ -f \"/workspace/\$CONFIG_FILE_ORIG\" ]; then
            cp \"/workspace/\$CONFIG_FILE_ORIG\" /opt/ns-3-dev/ 2>/dev/null || true
            [ \"$DEBUG\" = \"true\" ] && echo \"Copied config file: \$CONFIG_FILE_ORIG to /opt/ns-3-dev/\"
        else
            echo \"ERROR: Config file \$CONFIG_FILE_ORIG not found in workspace\"
            exit 1
        fi
        

        [ \"$DEBUG\" = \"true\" ] && echo 'Building modulation comparison simulation...'
        cd /opt/ns-3-dev
        if [ \"$DEBUG\" = \"true\" ]; then
            ./ns3 build scratch_modulation_comparison
        else
            ./ns3 build scratch_modulation_comparison >/dev/null 2>&1
        fi
        
        [ \"$DEBUG\" = \"true\" ] && echo 'Running modulation comparison simulation...'
        [ \"$DEBUG\" = \"true\" ] && echo ''
        
        # Run the simulation with the command arguments
        ./ns3 run modulation_comparison -- $CMD_ARGS

    " 2>&1
DOCKER_EXIT_CODE=$?

# ============================================================================
# SIMULATION COMPLETION MESSAGE
# ============================================================================

# Check if simulation completed successfully (exit code 0 = success, non-zero = failure)
if [ $DOCKER_EXIT_CODE -eq 0 ]; then
    echo ""
    echo "================"
    echo "Simulation completed successfully!"
    echo "Output directory: $OUTPUT_DIR"
    echo "================"
else
    echo ""
    echo "=== Simulation failed! ==="
    echo "Check the error messages"
    # Clean up output directory if simulation failed
    if [ -d "$OUTPUT_DIR" ]; then
        rm -rf "$OUTPUT_DIR"
    fi
    exit 1
fi

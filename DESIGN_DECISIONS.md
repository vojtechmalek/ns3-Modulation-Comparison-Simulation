# Design Decisions: Modulation Comparison Simulation

## Scenarios

### Simple Straight-Line Configurations: S1 (LOS), S2 (NLOS)

**What:**
- **S1**: Free-space propagation scenario with configurable distance between nodes
- **S2**: Indoor propagation scenario with configurable number of walls and wall spacing

**Why:**
- **S1 (line-of-sight)**: Simple free-space scenario to test how packets and modulations behave in free-space propagation
- **S2 (non-line-of-sight)**: Indoor scenario with walls to test wall penetration capabilities
- Faster simulations

**Why not more complex scenarios:**
- NS-3 propagation models use straight-line distance calculations and do not provide ray tracing or complex path modeling

---

## Modulation

### PHY Model: AdhocAlohaNoackIdealPhy

**What:** `AdhocAlohaNoackIdealPhy` for all three modulations.

**Why:**
- Fair comparison: All modulations use the same PHY model, but can be configured with modulation-specific parameters (TX power, data rate)
- Supports direct node-to-node (P2P) communication
- Simple and fast
- Flexible: Allows modulation-specific configuration while maintaining fair comparison

**Why not other options:**
- Specific modulation models don't exist or would be too complex:
  - WiFi PHY module: This one would work for WiFi HaLow
  - LoRaWAN module: Gateway-based, doesn't allow P2P communication
  - DECT NR+ PHY module: Not available in NS-3, would require building it in its entirety

### PER Curves (SNR-Based Error Injection)

**What:** Implement SNR-based PER curves for each modulation at trace callback.

**Why:**
- Modulation-specific accuracy: Each modulation gets its own realistic PER curve
- Fair comparison: All modulations use same mechanism but with different PER curves
- Easy to implement

**How it works:**
1. PHY receives packet and decides success/failure
2. If success -> Intercept at `PhyRxEndOkTrace`
3. Calculate PER from SNR using modulation-specific curve
4. Probabilistically decide: if `random() < PER`, mark as failed
5. Update metrics

**Why not other options:**
- Specific modules would be too difficult as explained in PHY models sections
- Fixed Error Models would not be SNR-dependent

---

## Propagation Models

### S1: FriisPropagationLossModel

**What:** `FriisPropagationLossModel` for free-space path loss.

**Why:**
- Accounts for modulation-specific frequency (using Friis formula)
- Appropriate and fairly accurate for LOS scenarios

### S2: HybridBuildingsPropagationLossModel

**What:** `HybridBuildingsPropagationLossModel` for indoor propagation.

**Why:**
- Combines distance-based loss with building effects
- Accounts for number of walls, wall types (and if needed for indoor/outdoor status)
- Frequency-dependent Wall penetration loss

---

## Network Stack

### PacketSocket (L2)

**What:** PacketSocket to send packets.

**Why:**
- Direct PHY testing: Sends raw L2 packets directly from application to MAC layer which passes it directly to PHY
- Clean measurements: Only measures our generated traffic, giving accurate PER, SNR, and latency metrics
- Simple: No network layer overhead

---

## Mobility

### ConstantPositionMobilityModel

**What:** `ConstantPositionMobilityModel` for static nodes.

**Why:**
- Nodes are static: Our scenarios use fixed node positions
- Simple: No mobility overhead
- Different mobility model could easily be implemented if needed, the rest of the code should support it

### BuildingsHelper

**What:** `BuildingsHelper::Install()` to associate nodes with buildings (used in S2 scenario).

**Why:**
- Associates nodes with buildings for propagation models
- Needed for `HybridBuildingsPropagationLossModel` to calculate wall penetration correctly

---

## Channel

### MultiModelSpectrumChannel

**What:** `MultiModelSpectrumChannel` to support different propagation loss models.

**Why:**
- Supports different propagation models for different scenarios
- Required for building-aware propagation models (S2 needs it for `HybridBuildingsPropagationLossModel`)

---

## Build System

### Docker + NS-3 Scratch Directory

**What:** Build the simulation inside a Docker container using NS-3's scratch directory.

**Why Docker:**
- No local NS-3 installation required
- Consistent environment
- Easy setup, users only need Docker

**Why build inside NS-3 scratch directory:**
- Automatic build configuration and dependency linking
- No manual CMakeLists.txt needed
- Standard NS-3 workflow for custom simulations
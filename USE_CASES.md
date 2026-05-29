# Use Cases

This firmware is best suited to low-bandwidth location beacons that send
periodic GPS and battery telemetry over LoRa.

## Good Fits

### 1. Trailer and container tracking

Why it fits:

- Location updates are periodic rather than continuous.
- Battery visibility is useful for unattended deployments.
- Last Known Location is helpful when the asset moves into poor GPS coverage.

Suggested settings:

- `sleep 300` to `sleep 1800`
- `sendLKL 1`
- Choose `loraFreq` for your legal regional band plan.

### 2. Construction yard equipment beacons

Why it fits:

- Equipment often sits idle for long periods and only needs periodic presence checks.
- Deep sleep helps preserve battery between transmissions.
- GPS plus battery state is enough for many yard-level recovery workflows.

Suggested settings:

- `sleep 600` to `sleep 3600`
- `sendLKL 1`

### 3. Event infrastructure and temporary installations

Examples:

- Portable towers
- Remote power kits
- Temporary network cabinets
- Mobile broadcast or production cases

Why it fits:

- These assets are expensive, mobile, and frequently deployed in the field.
- Battery reporting helps distinguish relocation from power loss.

Suggested settings:

- `sleep 120` to `sleep 900`
- Use a descriptive `dest` per deployment or asset group.

### 4. Remote maintenance kits and service cases

Why it fits:

- Small service kits can be tracked as periodic beacons without requiring a full IP stack.
- The current payload is compact and easy to parse into CSV or another backend.

Suggested settings:

- `sleep 300` to `sleep 1800`
- `sendLKL 1`

### 5. Fixed-site proof-of-presence beacons

Examples:

- Pump stations
- Solar enclosures
- Utility cabinets
- Remote sheds

Why it fits:

- Even stationary assets benefit from periodic health and battery reporting.
- A missing or shifted location can highlight tampering or relocation.

Suggested settings:

- `sleep 900` to `sleep 7200`
- `sendLKL 0` if you prefer explicit no-fix behavior over replaying stored coordinates.

## When This Project Is Not a Good Fit

### Real-time vehicle tracking

This firmware is optimized for periodic transmissions, not second-by-second
location streams.

### Two-way command and control

The default implementation sends uplink telemetry only. It does not provide a
command channel, acknowledgements, or remote actuation workflow.

### Turnkey Reticulum deployments

Reticulum support here is a template, not a finished supported integration.
If you need production Reticulum routing, identity lifecycle handling, or LXMF,
budget for additional porting work.

### Rich sensor telemetry

The payload is intentionally focused on GPS, battery, and firmware metadata.
If you need environmental sensors, binary attachments, or larger records, you
will need to extend the packet format and corresponding receivers.

## Configuration Guidance

Use shorter sleep intervals when:

- The asset is likely to move.
- Battery life is less critical than recency.
- You need quicker confirmation after deployment.

Use longer sleep intervals when:

- The asset is mostly stationary.
- Battery life matters more than frequent updates.
- LoRa airtime needs to stay low across a fleet.

Enable `sendLKL 1` when:

- Your downstream system prefers stale-but-known coordinates over silence.
- GPS coverage is intermittent.

Disable `sendLKL` when:

- You need to distinguish a current fix from a stored historical location.
- Operators should treat no-fix conditions as a distinct event.

## Typical Deployment Pattern

1. Flash firmware and verify serial output.
2. Set `loraFreq`, `sleep`, `dest`, and `sendLKL`.
3. Bench-test payload parsing with the included Python parser.
4. Deploy the asset with a known-good antenna and power source.
5. Validate received packets in the field and tune sleep interval as needed.
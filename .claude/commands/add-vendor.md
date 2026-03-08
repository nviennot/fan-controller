# Add a new fan remote vendor

You are adding support for a new vendor to the fan-remote OOK transmitter in `fan-kareem/src/tx.rs`.

## Prerequisites

The user should provide:
- A WAV file recording of the remote pressing buttons (captured via fan-rx or other SDR tool)
- The vendor name to use in config
- The device_id of the recorded remote

## Step-by-step process

### 1. Analyze the WAV recording

Read the WAV file and analyze the OOK signal to determine:

**Timing characteristics:**
- Measure pulse and gap durations for bit=0 and bit=1
- Determine encoding type:
  - **Gap-width encoding** (like vendor_a): fixed pulse width, variable gap (short=0, long=1)
  - **Pulse-width encoding** (like vendor_b): variable pulse width, variable gap, possibly inverted
- Measure the frame gap between repeated transmissions
- Determine the carrier frequency

**Code structure:**
- All vendors use 32-bit codes: `[device_id:20][button_hi:4][rolling:4][check:4]`
- Button is 5 bits split as hi:4 (bits 8-11) and lo:1
- Determine how `lo` is encoded:
  - Vendor A style: `rolling = (lo << 3) | counter`
  - Vendor B style: `lo` XORed into bit 12 of device_id, `rolling = counter`

### 2. Decode button mappings

From multiple button recordings, extract all 32-bit codes and determine:
- The 5-bit button value for each button name
- Whether a rolling counter is used (vendor_a increments mod 8, vendor_b always 0)

### 3. Determine the check formula

The check nibble (bits 0-3) is derived from other fields:
- Vendor A: `check = rolling ^ hi ^ key`
- Vendor B: `check = hi ^ key`
- The key is derived from device_id: `key = xor_nibbles(device_id) ^ VENDOR_CONSTANT`
- Determine the vendor constant by solving: `VENDOR_CONSTANT = key ^ xor_nibbles(device_id)`

### 4. Add the vendor to tx.rs

Add the following to `fan-kareem/src/tx.rs`:

**a) Timing constants** (after existing vendor constants):
```rust
// Vendor X: <encoding-type> encoding
const VX_...: f32 = ...;
```

**b) Frequency constant:**
```rust
const FREQUENCY_VENDOR_X: f64 = ...;
```

**c) Button array** (after existing button arrays):
```rust
const BUTTONS_VENDOR_X: &[(&str, u8)] = &[
    ("toggle_light", 0x..),
    ("off", 0x..),
    // ... all buttons
];
```

**d) Match arm in `load_config()`** — add a new arm to the vendor match:
```rust
"vendor_x" => (BUTTONS_VENDOR_X, <rolling_in_check>, <key_mask>, FREQUENCY_VENDOR_X),
```

Where:
- `rolling_in_check`: `true` if check uses rolling nibble (like vendor_a), `false` otherwise
- `key_mask`: the vendor constant for key derivation

**e) If the new vendor uses a different encoding than gap-width or pulse-width**, add a new variant in `generate_press()` and `make_code()`. If it matches an existing pattern, the `rolling_in_check` flag already handles the dispatch.

**f) If the new vendor has a different `lo` bit encoding**, add a new encoding path in `make_code()`. This may require adding a field to the `Fan` struct.

### 5. Update targets_json()

Add the new vendor's button list to the merged commands in `targets_json()`.

### 6. Test

- Add a test fan to `fans.yaml` with the new vendor
- Verify the generated codes match the captured codes from the recording
- Test actual transmission with the SDR

## Reference: Existing vendor parameters

| Parameter | Vendor A | Vendor B |
|-----------|----------|----------|
| Encoding | Gap-width (fixed pulse, variable gap) | Pulse-width (variable pulse+gap, inverted) |
| Frequency | 433.899 MHz | 433.9355 MHz |
| Key mask | 0xA | 0x0 |
| Rolling in check | Yes | No |
| Counter | Increments mod 8 | Always 0 |
| Pulse durations | 260µs fixed | 390µs short / 1170µs long |
| Gap durations | 500µs short / 1500µs long | 400µs short / 1180µs long |
| Frame gap | 4300µs | 12000µs |
| Trailing pulse | Yes (after last bit) | No |

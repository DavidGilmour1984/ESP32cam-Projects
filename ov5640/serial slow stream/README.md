# ESP32 Serial Camera (Reliable Packet Streaming)

## Overview

This project uses an ESP32-CAM to capture an image, encode it as Base64, and transmit it over Serial using a reliable packet-based protocol with checksum verification.

The system is designed for:

* Serial monitoring and debugging
* Web Serial (HTML interface)
* Radio transmission (e.g. DXLR modules)
* Noisy or lossy communication links

---

## Features

* JPEG image capture (QQVGA)
* Base64 encoding for safe text transmission
* Packetised data transfer
* Checksum validation per packet
* ACK / NACK retransmission system
* Timeout-based automatic resend
* Compatible with browser-based receiver

---

## Hardware Requirements

* ESP32-CAM (M5Stack / AI Thinker compatible pinout)
* USB-to-Serial adapter (or onboard programmer)
* Optional: Serial radio module (e.g. DXLR02)

---

## Serial Settings

* Baud Rate: `115200`
* Line Ending: `Newline`

---

## How It Works

### 1. Capture

Send over Serial:

```
CAPTURE
```

ESP32:

* Captures image
* Encodes to Base64
* Splits into packets
* Sends:

```
START,<totalPackets>
```

---

### 2. Packet Format

Each packet is sent as:

```
PKT,<index>,<checksum>,<data>
```

Example:

```
PKT,0,75,/9j/4AAQSkZJRg...
```

---

### 3. Checksum

Checksum is:

```
sum of ASCII values of payload & 0xFF
```

Used to verify data integrity.

---

### 4. Receiver Response

Receiver (HTML or Serial) must reply:

#### If correct:

```
ACK,<index>,<checksum>
```

#### If incorrect:

```
NACK,<index>
```

---

### 5. Flow Control

Protocol is **stop-and-wait**:

```
ESP → PKT → WAIT → ACK → NEXT PKT
```

If no response:

* Packet is resent after timeout

---

### 6. Completion

After last packet:

```
END
```

---

## Code Structure

### Key Functions

#### `initCamera()`

Initialises ESP32-CAM with JPEG output.

#### `startCapture()`

* Captures frame
* Encodes to Base64
* Calculates packet count
* Sends first packet

#### `sendPacket()`

* Sends current packet
* Includes checksum

#### `checksum()`

* Calculates 8-bit checksum

---

## State Variables

| Variable        | Description              |
| --------------- | ------------------------ |
| `fullData`      | Full Base64 image string |
| `totalPackets`  | Number of packets        |
| `currentPacket` | Current packet index     |
| `sending`       | Transmission state       |

---

## Timeout Behaviour

If no ACK received within:

```
TIMEOUT_MS = 2000 ms
```

ESP32:

* Resends the same packet

---

## HTML Receiver

The included HTML tool:

* Connects via Web Serial
* Displays raw packet stream
* Reconstructs Base64 data
* Renders image live
* Sends ACK/NACK automatically

---

## Example Data Flow

```
CAPTURE
→ START,55

PKT,0,...
← ACK,0,75

PKT,1,...
← ACK,1,120

...

PKT,54,...
← ACK,54,154

END
```

---

## Common Issues

### Repeating packets (e.g. PKT,0 repeatedly)

Cause:

* No ACK received

Fix:

* Ensure receiver sends correct ACK format

---

### Packets duplicated

Cause:

* Timeout too short

Fix:

```
#define TIMEOUT_MS 4000
```

---

### Image not rendering

Cause:

* Missing packets or corrupted Base64

Fix:

* Ensure all packets received in order
* Verify checksum handling

---

## Optimisation Tips

For radio (DXLR):

```
#define PKT_SIZE 50
#define TIMEOUT_MS 3000
```

* Smaller packets = more reliable
* Larger timeout = fewer duplicates

---

## Future Improvements

* Adaptive packet sizing
* Compression tuning
* Forward error correction
* Binary (non-Base64) mode
* Signal strength feedback

---

## Summary

This system implements a robust:

```
Reliable Serial Image Transmission Protocol
```

with:

* Integrity checking
* Packet recovery
* Hardware compatibility
* Browser-based decoding

---

## Author Notes

Designed for:

* Embedded systems
* Remote telemetry
* Educational demonstrations
* Low-bandwidth image transmission

---

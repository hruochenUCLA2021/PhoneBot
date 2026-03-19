# NOTE: USB latency timer, Dynamixel RDT, and “granularity” (why Hz changes)

This note explains why a Dynamixel read loop often runs slower than the requested timer rate, and why changing **Return Delay Time (RDT)** and the USB‑serial **`latency_timer`** can change the measured frequency a lot.

## What you actually measure in a ROS2 motor loop

If you set a timer to \(f_{req}\) (e.g. 100 Hz, period \(T_{req}=10\) ms), the real loop rate becomes:

\[
f_{real} \approx \frac{1}{T_{work}}
\]

where \(T_{work}\) is the time taken by the callback work (BulkRead Tx/Rx + parsing + publishing + any writes).  
If \(T_{work} > T_{req}\), the timer can’t keep up and your rate drops.

In our case, the callback includes a **GroupBulkRead** that must:
- send one request packet (RPi → bus)
- receive 13 status packets (motors → bus → USB serial → Linux → your process)

So \(T_{work}\) is not just “serial baudrate”; it includes **bus wire time + motor delays + USB/driver buffering + OS scheduling**.

## Two different “latencies” that people mix up

### 1) Dynamixel **Return Delay Time (RDT)**

- RDT is a **motor-internal wait**: after the motor receives the instruction, it waits before starting to transmit its Status Packet.
- For XL/X series, the unit is typically **\(2\,\mu s\)** per RDT count:

\[
T_{RDT} \approx \text{RDT} \times 2\,\mu s
\]

This happens **once per replying motor** (so with 13 motors, it can accumulate).

Important: RDT is **not doubled** because of “send + receive”. The protocol definition is a single pre-reply delay in the motor.

### 2) USB‑serial **`latency_timer`** (host delivery granularity / batching)

For common USB‑serial chips (e.g., FTDI), incoming UART bytes are buffered and shipped to the host in USB transfers. The driver/chip typically sends data to the host when:
- a buffer threshold is hit, or
- the **latency timer expires** since the last transfer.

So `latency_timer=16` ms can cause “bursty” delivery where small/fragmented traffic is delivered to your process with ~16 ms timing, even if bytes arrived earlier on the UART.

Setting `latency_timer=1` ms reduces this **delivery batching delay** (it does not change the motor bus baudrate).

This is why we call it **granularity**: it affects *when your program sees bytes*, not how fast the motor wire transmits bits.

## Why RDT and USB granularity interact (why it can *look* like “RDT doubled”)

Even if RDT is defined as \(\text{RDT}\times 2\mu s\), your measured loop time can change by more than that because:

- High RDT spreads replies out in time.
- Spread-out replies are more likely to be delivered to userspace in **more separate USB chunks**.
- Each chunk can incur ~1 ms (or more) of buffering/delivery delay depending on USB scheduling and driver behavior.

So lowering RDT:
- directly reduces bus-side dead time, and also
- tends to “pack” replies closer together, which may reduce the number of USB delivery events.

That second effect can make the benefit look larger than the pure \(\Delta\text{RDT}\times 2\mu s\) math, especially with many motors.

## Quick serial time estimation (back-of-envelope)

Assume UART **8N1** (10 bits per byte).

### Byte time

- At **1 Mbps**: \(1{,}000{,}000\ \text{bits/s} \Rightarrow 100{,}000\ \text{bytes/s}\Rightarrow 0.01\ \text{ms/byte}\)
- At **3 Mbps**: \(300{,}000\ \text{bytes/s}\Rightarrow 0.00333\ \text{ms/byte}\)

### Packet sizes used in our motor node

We BulkRead a contiguous block: **Present Velocity(128,4) + Present Position(132,4) = 8 bytes** per motor.

Typical Protocol 2.0 sizes (approx, used for estimation):
- BulkRead request packet: \(\approx 10\) bytes overhead + **5 bytes per motor** parameter  
  \(\Rightarrow 10 + 13\times 5 = 75\) bytes
- Status reply per motor: \(\approx 11\) bytes overhead + **8 data bytes**  
  \(\Rightarrow 19\) bytes per motor, so \(13\times 19 = 247\) bytes total replies

### “Bytes-on-the-wire only” time at 1 Mbps

- Request: \(75 \times 0.01 \approx 0.75\) ms  
- Replies: \(247 \times 0.01 \approx 2.47\) ms  
- Total wire time: **~3.22 ms**

### Add RDT accumulation (example)

- If RDT=250: \(T_{RDT} \approx 250\times 2\mu s = 0.5\) ms per motor  
  13 motors ⇒ **~6.5 ms**
- Total (wire + RDT): **~9.7 ms** (still not including USB/driver/OS overhead)

If you observe ~16 ms per loop (~62 Hz), that “extra” ~6 ms is often USB/driver granularity + scheduling, not a literal doubling of RDT.

## Example estimate: baudrate 3 Mbps, RDT=50, USB latency ~1 ms

- Wire time at 3 Mbps:
  - Request: \(75\times 0.00333 \approx 0.25\) ms
  - Replies: \(247\times 0.00333 \approx 0.82\) ms
  - Wire total: **~1.07 ms**
- RDT=50: \(50\times 2\mu s = 0.1\) ms per motor ⇒ \(13\times 0.1 = 1.3\) ms
- USB delivery overhead (very rough): **~1 ms**

Total rough: **~3.4 ms** (plus small parse/scheduling overhead), which is comfortably under 10 ms ⇒ 100 Hz is feasible.

## Practical takeaway

- **Lower RDT** is one of the best ways to reduce latency and increase achievable Hz, because it helps both:
  - bus-side dead time, and
  - USB delivery/batching (granularity) by packing replies.
- **Lower `latency_timer`** helps your process see incoming bytes sooner (less batching delay).
- **Higher baudrate** reduces pure wire time for every packet.

If you want to validate whether your bottleneck is bus-side vs USB-side, the most informative test is to repeat the timing with **only 1 motor** (same baudrate, same `latency_timer`, same RDT) and compare deltas.


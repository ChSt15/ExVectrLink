# Network air link v2

Last verified against commit: feature/air-link-v2 (bench numbers of 2026-11-26)

## Problem

v2 aims at more range margin through walls than v1 (SF5, 255 B, 10.56 ms on air; not yet field-verified). It trades airtime for sensitivity: SF6 at the same bandwidth.

## Wrong-env finding (read this)

A night of experiments showed a downlink (ship to TX module) of 40 to 70 % and an uplink of about 96 %. The cause was not the link design: the ship board has an external PA and LNA, and the plain env `exvectrlink_rx_network_esp8285` does not drive them. With `exvectrlink_rx_network_esp8285_pa` the pristine v1 gives 97.5 % / 99.6 %. A second, self-inflicted failure was a 1.0 ms RX wakeup lead (see below). Always use the `_pa` env on the ship ([build-targets-and-hardware.md](build-targets-and-hardware.md)).

## Profile

`kNetworkProfileV2` (`lib/ExVectrLink/include/ExVectrLink/datalink/AirProfile.hpp`), the network firmwares' default:

| | v1 (`kNetworkProfileV1`) | v2 (`kNetworkProfileV2`) |
|---|---|---|
| LoRa | SF5, BW1600, CR 4/5, preamble 12 | SF6, BW1600, CR 4/5, preamble 12 |
| Bytes on air | 255 | 220 |
| Time on air | 10.56 ms | 15.409 ms |
| Slot, wakeup lead | 20 ms, 2.5 ms | 20 ms, 2.5 ms |
| Strict margin (slot - lead - 1 ms - airtime) | 5.9 ms | about 1.09 ms |
| Hop key / link version | 0x37 / 0x11 | 0x37 / 0x12 |
| Largest net frame / LOG_TEXT text | 240 (cap) / 203+ | 210 / 203 |

v1 and v2 never accept each other (different frame length and trailer seed). `RxRouter::log` and `NetworkRouter::log` truncate `LogText` to `kMaxLogTextActive` (derived from the active profile in `src_network/NetAirAccess.hpp`); a longer frame would be silently dropped by the aggregator. FPV is untouched (`kFpvProfile`).

**Do not shorten the RX wakeup lead.** A 1.0 ms lead before RX slots left the TX module too little time to hop and arm RX before the SF6 preamble (downlink 43 to 54 %). v2 uses one 2.5 ms lead; the 220 B frame ends 2.09 ms before it.

## Bench results (2026-11-26, correct `_pa` hardware, 220 B at 20 ms slots)

| Profile | Downlink | Uplink |
|---|---|---|
| v1, pristine Development | 97.5 % | 99.6 % |
| v2 | 96.4 % | 95.7 % |

## Rollback to v1

Build both network firmwares with `PLATFORMIO_BUILD_FLAGS=-DEXVECTRLINK_NET_PROFILE_V1` (or add the flag to the env) and flash both ends: a v1 and a v2 end do not connect.

## Field test: v1 against v2 through walls

At 10 mW (the default power) in the same place and the same walls: flash both ends with v1, run 2 minutes at each of several positions and note the LQ (`exgs` LQ view, downlink and uplink). Repeat with both ends on v2. Compare per position; v2 should keep a usable LQ one or two walls further. Keep antennas and orientation identical.

# Air link optimisation (network mode): design

> Design, 2026-11-26. Sections 1 to 7 are design only. Section 8 (build-time "v2-trial" profile) is implemented. Branch `feature/air-link-optimisation` (from `Development` a372c05).
> Workspace todo section 3, item 3. Applies to the network-mode firmwares only (Router = TX module, node 2; ship
> node = RX module, node 3). **FPV profiles and the FPV code in `src/` are not touched by anything below.**
>
> Last verified against commit: a372c05 (submodules: ExVectrLink `feature/air-link-fhss`, ExVectrNetwork de708ca).

## 0. Problem and fixed constraints

- Field result: LQ 50 to 80 % at 10 mW through two walls. Air profile v1 is LoRa SF5 / BW1600 / CR 4/5, the
  most data-rate-oriented LoRa setting the SX1280 has, so it is also the least sensitive one.
- v1 (`kNetworkProfileV1`, `lib/ExVectrLink/include/ExVectrLink/datalink/AirProfile.hpp`): 20 ms slots, 1:1
  roles, 2 slots per hop, fixed 255 B on air (253 B air packet + 2 B FHSS trailer), 12-symbol preamble, implicit
  header, CRC on. 10.56 ms on air. 25 packets/s each way.
- **The 20 ms slot interval must not grow** (owner). Power stays at 10 mW (legal limit, `NetPower.hpp`).
- Downlink load about **3.0 kB/s** (3006 B/s, ExVectrPackets `docs/telemetry-sets.md` and workspace
  `docs/network-budget.md`), uplink about **0.61 kB/s** (CMD_STATE 25 Hz plus link state).
- Capacity per direction = 25 x (frame length - 2 trailer - 3 air header - 4 CRC-32) B/s. Every frame costs its
  length + 1 (LEN). v1: 25 x 246 = 6150 B/s.

## 1. Slot budget

Within one 20 ms slot the receiver must have the whole packet and its RX_DONE before the FHSS task wakes for the
next slot. With 1:1 roles the next slot is the receiver's own TX slot, and its wakeup is `wakeupLeadNs` = 2.5 ms
before the boundary (the 253 B air packet is built and loaded into the shared FIFO then; loading while still
receiving would overwrite the received bytes). The grace window in `FHSS::taskThread()` stretches this to 0.5 ms
before the boundary, but only as a rescue. So the design budget is:

**max time on air = 20 ms - 2.5 ms lead - 1.0 ms margin (sync error, scheduler jitter, RX_DONE handling) = 16.5 ms.**

The implementation enforces it with `fhss::slotFits(lora, onAirBytes, slotIntervalNs, guard = 3.5 ms)`.

## 2. Time on air and sensitivity

### 2.1 Time on air (computed, datasheet formula)

Computed with the SX1280 datasheet formula as implemented in `fhss::loraSymbolCount()` (identical to the
driver's `SX128XLT::calcLoRaSymbolCount()`; the host test checks v1 = 10.56 ms). Preamble 12 symbols, implicit
header, CRC on. Symbol time = 2^SF / BW: SF5/BW1600 19.7 us, each SF step doubles it, each BW halving doubles it.
An SF step does *not* double the airtime of a packet: SF+1 carries one more bit per symbol, so a packet costs
about x 2 x SF/(SF+1) (SF5 -> SF6: x 1.67, SF6 -> SF7: x 1.71).

Time on air for a **255 B** frame (ms), and in brackets the **largest frame that fits 16.5 ms** (B):

| SF / BW | CR 4/5 | CR 4/6 | CR 4/7 | CR 4/8 | LI 4/6 | LI 4/8 |
|---|---|---|---|---|---|---|
| SF5 / 1600 (v1) | **10.56** (255) | 12.57 (255) | 14.58 (255) | 16.59 (253) | 12.51 (255) | 16.57 (253) |
| SF6 / 1600 | 17.77 (235) | 21.12 (196) | 24.47 (169) | 27.82 (148) | 20.96 (198) | 27.74 (148) |
| SF7 / 1600 | 30.66 (130) | 36.41 (105) | 42.16 (91) | 47.91 (81) | 36.17 (109) | 47.75 (81) |
| SF8 / 1600 | 54.23 (65) | 64.32 (53) | 74.40 (45) | 84.48 (41) | 63.68 (55) | 83.85 (41) |
| SF5 / 800 | 21.12 (195) | 25.14 (163) | 29.15 (140) | 33.17 (123) | 25.02 (164) | 33.13 (123) |
| SF6 / 800 | 35.54 (109) | 42.24 (91) | 48.94 (79) | 55.63 (67) | 41.92 (93) | 55.47 (69) |
| SF7 / 800 | 61.32 (56) | 72.82 (46) | 84.32 (39) | 95.82 (35) | 72.35 (48) | 95.51 (35) |
| SF5 / 400 | 42.24 (90) | 50.27 (75) | 58.31 (65) | 66.34 (55) | 50.04 (77) | 66.26 (57) |
| SF6 / 400 | 71.09 (46) | 84.48 (40) | 97.87 (34) | 111.26 (28) | 83.85 (41) | 110.95 (30) |

Script: the table was produced by a Python port of `loraSymbolCount()` (kept out of the repo). Cutting the
preamble from 12 to 8 symbols saves 4 symbols (0.08 ms at SF5/1600, 0.16 ms at SF6/1600, 0.31 ms at SF6/800):
a few bytes of frame, not worth the weaker preamble detection.

### 2.2 Sensitivity per step

**Not verified against the datasheet in this task**: the SX1280 datasheet PDF could not be fetched (timeout / 403).
The step gains below are from LoRa theory; the absolute numbers are from memory of the datasheet table and must be
checked before they are quoted anywhere else.

| Step | Gain (theory) | Why |
|---|---|---|
| BW halved (1600 -> 800 -> 400 -> 200) | **+3 dB** per step | noise bandwidth halves |
| SF + 1 (5 -> 6 -> 7 ...) | **about +2.5 dB** per step (datasheet steps are 2.5 to 4 dB) | demodulator SNR limit about -2.5 dB per SF |
| CR 4/5 -> 4/6 / 4/7 / 4/8 | **about 0.5 to 1.5 dB** in white noise | Hamming code; 4/7 and 4/8 correct one bit per codeword |
| long interleaving (LI) | little in white noise; helps against fading and short bursts (WiFi, multipath) | spreads codewords over more symbols |
| preamble length | none for sensitivity | only detection robustness |

Absolute values as remembered (to check): BW1625 SF5 about -99 dBm, BW812 SF5 about -105 dBm (ELRS quotes -105 dBm
for its 500 Hz mode, SF5/BW812), SF7/BW812 about -112 dBm. If BW1625 really is -99 dBm, halving to BW812 gains
more than the theoretical 3 dB (the 1625 kHz bandwidth has an extra implementation loss). The field test (section
5) settles it.

Rule of thumb: +3 dB is x 1.4 range in free space, or about one light interior wall's worth of loss at 2.4 GHz
(walls are typically 3 to 15 dB each).

## 3. Which options fit

### 3.1 Candidates (frame length chosen to fit 16.5 ms with a small margin)

Capacity = 25 x (frame - 9) B/s. Downlink need 3006 B/s; the "worst slot" (every downlink frame due at once) is
292 B and is already deferred today (latest-wins, lowest priority waits one slot).

| Id | Modulation | Frame | ToA | Gain vs v1 (theory) | Capacity per direction | Downlink load | Verdict |
|---|---|---|---|---|---|---|---|
| P0 | SF5 / 1600 / 4/5 (v1) | 255 | 10.56 | 0 | 6150 | 49 % | baseline |
| P1 | SF5 / 1600 / 4/7 | 255 | 14.58 | about +1 dB | 6150 | 49 % | fits, cheap, small gain |
| P2 | SF5 / 1600 / LI 4/8 | 250 | 16.25 | about +1.5 dB, plus burst robustness | 6025 | 50 % | fits, small gain |
| P3 | **SF6 / 1600 / 4/5** | **224** | **15.80** | **about +2.5 to +4 dB** | **5375** | **56 %** | **fits: recommended v2** |
| P4 | SF5 / 800 / 4/5 | 192 | 16.20 | +3 dB (possibly up to +6 dB, see 2.2) | 4575 | 66 % | fits, strongest candidate if the datasheet penalty at BW1625 is real |
| P5 | SF6 / 1600 / LI 4/6 | 192 | 16.00 | about +3 to +5 dB | 4575 | 66 % | fits |
| P6 | SF5 / 800 / LI 4/6 | 160 | 16.04 | about +3.5 to +7 dB | 3775 | 80 % | fits on paper, too little headroom for growth |
| X1 | SF6 / 800 / 4/5 | 104 | 15.85 | about +5.5 to +7 dB | 2375 | 127 % | **does not fit the downlink** |
| X2 | SF7 / 1600 / 4/5 | 128 | 16.48 | about +5 to +7 dB | 2975 | 101 % | does not fit the downlink |

Conclusion: **with a 20 ms slot and 3 kB/s of downlink, modulation can buy about 3 to 4 dB (maybe 6 dB with
BW800), not more.** Anything beyond needs less downlink data or a longer interval.

### 3.2 Smaller frames and reduced payload

- Smaller frames do not improve sensitivity by themselves; they only make room for a slower modulation inside the
  16.5 ms. A shorter packet is also a slightly smaller target for a burst, so LQ in a WiFi-busy band rises a little.
- Splitting one slot into two short frames (for example two 100 B frames) gains nothing: same bits, same
  modulation, two preambles.
- Payload savings from `docs/network-budget.md` that would move the downlink down one capacity class: P2
  (`LINK_STATS` 2 Hz on air, -78 B/s), P3/P4 (`VEHICLE_STATUS` time and bits, -40 B/s), P8 (air header
  compression, -96 B/s down). Together about 2.8 kB/s, which makes P6 (80 % -> 74 %) usable but still does not
  reach X1/X2 (that needs about 2.3 kB/s or less: drop `AIR_ACTUATORS` to 12.5 Hz, -560 B/s, owner decision).

### 3.3 Asymmetric profile (uplink more robust than downlink)

The uplink carries only 0.61 kB/s, and it is the **safety-relevant direction** (CMD_STATE; the vehicle fails
safe when it stops). It can use X1 (SF6 / BW800 / 4/5, 104 B, 2375 B/s, 3.9 x the need) while the downlink uses
P3 or P4. The uplink then gains about +5.5 to +7 dB over v1 and about +3 dB over the downlink.

- Cost: the radio must switch modulation and fixed length per slot role (SetModulationParams + SetPacketParams
  before each slot, a few tens of us over SPI; the driver already applies changes lazily via `modParamsChanged`).
  FHSS needs a per-role frame length (TX side sends uplink length, RX side downlink length) and both sides must
  compute the RX_DONE time-on-air correction with the modulation of the packet received.
- Uplink worst slot: CMD_STATE 21 + LINK_CONFIG 9 + LINK_STATS 26 + heartbeat 16 + REQUEST 19 = 91 B against 95 B
  of entries. Fits, CMD_STATE first (CONTROL class).
- Sync (ship node) runs on uplink packets, so lock and hold also become more robust.
- Recommended as **phase 2**, after the symmetric field test shows which downlink modulation is best.

### 3.4 FHSS and diversity interaction

- **FHSS:** the hop sequence (20 channels, 2425 to 2475 MHz, 2.63 MHz apart), the 2 slots per hop and the slot grid
  do not change. A narrower bandwidth (BW800) fits the channel spacing better (less adjacent overlap). WiFi on a
  channel blocks it in every profile; longer packets are hit by a burst a little more often, which LI partly
  offsets. The Searching receiver stretches slots to 1.9 x (38 ms) and listens continuously, so every candidate
  is found while searching. Frequency error is not a concern: 2 x 20 ppm at 2.45 GHz is about 100 kHz, inside the
  LoRa tolerance of about +-BW/4 for both 1625 and 812 kHz.
- **Sync timing:** the follower takes the phase from RX_DONE minus time on air. The driver computes that from its
  current settings, so it follows a runtime change automatically, but the switch must change both radios (and the
  driver's notion of the length) in the same slot. Must be covered by a host test.
- **Diversity:** the Router has true diversity (receive on both radios, best wins; TX on the best fresh radio).
  The ship node on ESP8285 has one radio. Through walls (multipath) diversity often gives more than the 3 to 4 dB
  available from modulation. The existing `exvectrlink_rx_network_genericdiversity` env (ESP32 + 2 radios) would
  give the ship node RX diversity: a hardware choice for the owner, outside this design.
- **Duty cycle:** each end transmits one slot in two: 26 % (v1) rises to about 40 % (P3/P4). Power is unchanged.
  The owner should confirm that this is fine under the local 2.4 GHz rules at 10 mW (no change of EIRP).

## 4. Runtime air profile switch

Goal: the owner compares profiles in the field without reflashing. Both ends must change in the same slot; a lost
link in the middle must end on a known profile without help.

### 4.1 Parameters and validation

- `AirProfile` gets runtime fields for the modulation: `lora.sf` (5..8), `lora.bwHz` (1625000 / 812500 / 406250),
  `lora.cr` (1..4, 5..7 LI), and `fixedFrameLength` (64..255). Slot interval, slots per hop, role period, hop key,
  link version and wakeup lead **stay fixed** (the grid and hop sequence must continue through a switch).
- A pure function `validateNetProfile(params)` (host-tested) accepts a set only if the values are in range,
  `slotFits(lora, frameLength, 20 ms, 3.5 ms)` holds, and capacity = 25 x (frameLength - 9) >= 3500 B/s (downlink
  need plus margin; constant in one place). Both ends run it; a set either end refuses is never used.
- Packed form (`uint32_t`, used in REQUEST.arg_i and LINK_CONFIG): bits 0-3 SF, 4-5 BW index (0 = 1625, 1 = 812,
  2 = 406 kHz), 6-8 CR register value, 9-16 frame length, 17-23 reserved 0. Value 0 means "baseline (v1)".
- Compiled-in table of named candidates (P0 to P6 of section 3.1) for the Lua menu; the REQUEST accepts any valid
  packed set.

### 4.2 Who commands it

- **Router is the configuration master**, as for power. Two entry points:
  - REQUEST action **25 `LINK_SET_AIR_PROFILE`**, target 2, `arg_i` = packed set (new enum value, ExVectrPackets).
    REQUEST_ACK: `IN_PROGRESS` while the switch runs (retries with the same id get the same answer), `DONE` after
    probation passed, `FAILED` detail 1 = reverted (link lost), `REJECTED_INVALID` detail 1 = set refused by
    validation, `REJECTED_STATE` detail 1 = link not Synced or a switch already running.
  - Lua menu entry "Air profile" (TEXT_SELECTION over the candidate table) with a confirmation step, like "Reboot
    ship node". Lets the owner test with the Pocket alone.
- The ship node never initiates a switch; it only follows LINK_CONFIG.
- **Safety:** a failed switch silences the link for up to about 10 s (4.4). CMD_STATE stops, and a committed vehicle
  fails safe (1 s motors off, 2 s latched). The modules cannot know the vehicle state, so the senders guard it, as
  for `LINK_REBOOT`: GS only behind a danger confirmation while the vehicle is on the ground and disarmed; Lua only
  through the confirmation step. The flight software ignores action 25.

### 4.3 Switch protocol (continuously repeated state, no one-shot event)

State lives in `LINK_CONFIG` (port 0x04, Router -> ship node), extended with two optional fields (older senders
decode as 0 = no switch):

- `air_profile` (u32, packed): the profile the Router wants.
- `switch_sfc` (u8): superframe counter value at which `air_profile` becomes active.

and in `LINK_STATS` from the ship node, extended with `air_profile` (u32, packed, the profile it runs) and
`pending_profile` (u32, the one it has armed, 0 = none). The superframe counter (SFC) is the 8-bit counter in every
air packet header: the master increments it per own TX slot (40 ms, so it wraps after 10.24 s); the ship node
extrapolates it by +1 per role period when a packet is missed.

Sequence:

1. **Arm.** Router validates the new set N, keeps the current set C as "last good", picks
   `switch_sfc = SFC_now + 50` (2 s ahead) and sends LINK_CONFIG {N, switch_sfc} **in every uplink packet** (25 Hz,
   instead of 2 Hz) until the switch. At LQ 50 % the chance that all 50 copies are lost is negligible.
2. **Confirm.** The ship node validates N; if valid it arms it for `switch_sfc` and reports
   `pending_profile = N` in LINK_STATS (sent at 5 Hz, now also every downlink packet while armed). If the Router has
   not seen the confirmation by `switch_sfc - 10` (0.4 s before), it **cancels**: it sends LINK_CONFIG
   {C, no switch} (the ship node disarms on that) and answers `FAILED` detail 2. If the ship node armed but missed
   the cancel, it switches alone and recovers through step 4.
3. **Switch.** Both ends change modulation and fixed length at the start of the slot with SFC = `switch_sfc`, before
   that slot's TX prep or RX arm (FHSS task, same place `applyProfile` values are used). The slot grid and hop
   sequence are untouched, so the ship node stays Synced if packets keep arriving.
4. **Probation and revert.** For 5 s after the switch each end independently checks: if it receives no valid
   packet for **2 s**, it reverts to C (its own "last good") at its next slot boundary. If one end reverted
   alone, the other also sees silence and reverts within another 2 s, so a split always converges on C within
   about 4 s. After 5 s with packets on both ends (the Router sees the ship node's LINK_STATS with
   `air_profile = N`), the Router answers `DONE` and N becomes "last good" (RAM only).
5. **Lost after probation (backstop).** If FHSS on the ship node falls back to Searching (3 s without packets), it
   searches alternately on its current profile and on the **baseline v1** (switching every full hop cycle,
   20 hops x 40 ms = 0.8 s). The Router, when it has received nothing for 5 s, returns to v1 and stays there. So
   after any failure both ends meet on v1 within about 10 s, without anyone pressing anything.
6. **Reboot.** Every boot starts on the compiled-in default on both ends (v1 until the owner fixes v2). The
   runtime choice is never stored: one end rebooting alone must not leave the two on different stored profiles.
   The ship node's Searching ladder (step 5) also covers a ship node that rebooted while the Router runs N:
   the Router sees silence and returns to v1.

LINK_CONFIG keeps its 10 s rule for the power; the profile fields have no timeout of their own because the ship node
applies them only through the armed switch at `switch_sfc`, never "level-triggered". A LINK_CONFIG with
`air_profile` equal to the running profile is a no-op.

Notes:
- LINK_CONFIG is unauthenticated on air (known issue). A forged switch can at worst force a revert cycle; it cannot
  select an invalid profile (validation on the ship node).
- Extending LINK_CONFIG and LINK_STATS and adding action 25 changes the ExVectrPackets schema hash: a cross-project
  change (ground station decoder, flight software pin). Alternative without a schema change: carry the profile in
  the 2 reserved bits of LINK_CONFIG.flags and a fixed profile index (2 bits = 4 candidates only). See open
  decisions.
- Air protocol: v1 and v2 units still talk (both start on v1); the air packet format does not change. No
  `OTA_VERSION` bump (that constant is FPV's). The network link version (0x11) stays, so the trailer is the same in
  every candidate.

## 5. Recommended v2 and field test

### 5.1 Recommendation

- **Default v2 (symmetric): P3 = SF6 / BW1600 / CR 4/5, fixed frame 224 B** (15.80 ms, 0.7 ms more margin than the
  16.5 ms budget, 5375 B/s per direction, downlink 56 %). Expected about +2.5 to +4 dB both ways.
- **Close contender: P4 = SF5 / BW800 / CR 4/5, 192 B** (16.20 ms, 4575 B/s, 66 %). If the field test shows it
  more than about 1.5 dB better than P3 (the BW1625 penalty is real), take P4.
- **Phase 2:** asymmetric uplink X1 (SF6 / BW800, 104 B) for the command path, about +3 dB more on the uplink.
- The owner decides the final v2 from field numbers; it is then compiled in as the default of both network
  firmwares (`kNetworkProfileV2`), and v1 stays in the table as the recovery profile.

### 5.2 Field test procedure (owner)

Setup: same place as the 50 to 80 % result (two walls), Router in the Pocket on the GS PC, ship node on the vehicle
(disarmed, motors electrically disconnected), 10 dBm both ends, antennas and positions fixed and photographed.
GS recording running for the whole test.

1. On v1, record 2 minutes. Note from LINK_STATS (both nodes): rx LQ (mean and minimum over 1 s windows), RSSI, SNR,
   `air_rx_bad`, `frames_expired`, desync to Searching (count).
2. Switch to P1 (Lua menu or GS REQUEST). Confirm the ship node's LINK_STATS shows the new profile. Record 2 min.
3. Repeat for P2, P3, P4, P5, then back to v1 for 2 min (checks that conditions did not drift).
4. Repeat the whole sequence at a second, harder position (one more wall or further away), where v1 is near 30 %.
5. Compare: mean LQ and the 10th percentile LQ per profile and node. **SNR difference at the same RSSI is the
   sensitivity gain** (RSSI is a power reading and does not change with modulation; the LoRa demodulator's SNR
   margin above the SF limit does). Also note how often each profile fell back.
6. Fallback check (once, on the bench): start a switch to P4 and shield the ship node's antenna right after the
   ACK `IN_PROGRESS`. Expected: both ends back on v1 within about 10 s, LQ returns, Router answers `FAILED`.
7. Decision rule (proposal): pick the profile with the best 10th-percentile LQ on the uplink at the hard position;
   on a tie, the one with more capacity.

The GS can summarise this from the recording (LINK_STATS per profile segment); a small `exgs` script is optional.

## 6. Implementation steps (each sized for one Sonnet run)

Order matters; every step keeps FPV unchanged (`pio test -e native_fhss`, golden trace) and builds
`exvectrlink_tx_module_network`, `exvectrlink_rx_network_esp8285` and one FPV env.

1. **ExVectrLink lib, pure maths (S).** `NetProfile` packed codec, `validateNetProfile()`, candidate table P0..P6
   and X1 in `AirProfile.hpp`; budget constant 3.5 ms guard. Host tests: ToA values of section 3.1 against the
   table, every candidate valid, out-of-range and over-budget sets refused, round trip of the packing.
2. **ExVectrPackets (S, cross-project, main session first).** Action 25 `LINK_SET_AIR_PROFILE`; optional fields
   `LINK_CONFIG.air_profile`, `LINK_CONFIG.switch_sfc`, `LINK_STATS.air_profile`, `LINK_STATS.pending_profile`;
   regenerate C++ and Python; new hash. Then pin it in ExVectrLink (and the GS decoder in its own task).
3. **FHSS: scheduled profile switch (M).** `FHSS::scheduleModulationSwitch(params, atSlot)` applied at a slot start
   before TX prep / RX arm, changing the fixed length and calling a radio-config callback for both radios. Default
   path unchanged (nothing scheduled in FPV). Host tests in `native_fhss`: switch on both fake ends in the same slot
   keeps Synced with no lost packet; mismatched switch loses the link; RX_DONE time-on-air correction uses the new
   modulation; FPV golden trace still identical.
4. **Network mode: switch state machines (M).** Pure `ProfileSwitchMaster` (Router) and `ProfileSwitchFollower`
   (ship node) with injected time and SFC: arm, confirm, cancel, switch, probation, 2 s revert, 5 s Router return
   to v1, ship-node Searching ladder. `NetRadio.cpp` configures radios from the active profile instead of
   hard-coded SF5/BW1600/CR4/5; `NetAirAccess::kMaxAir` and the aggregator `maxLen` follow the frame length.
   Host tests: the sequences of 4.3 including every loss case (arm lost, confirm lost, cancel lost, one end
   reverts alone, ship node reboot) converge on one profile within the stated times; end-to-end test with the two
   routers over simulated slots.
5. **Entry points (S).** REQUEST action 25 handler on the Router (ACK states of 4.2), Lua "Air profile" entry with
   confirmation, LINK_STATS fields filled, LINK_CONFIG at 25 Hz while armed. Host tests for the handler and the
   Lua entry.
6. **Docs (S).** `network-mode.md` (profile switch, bench steps), `air-protocol.md` pointer, this file updated to
   "implemented", `known-issues.md` if something is found. GS side (separate task, ground-station agent): show
   the active profile per node, a guarded "Air profile" control, and the per-profile summary for the field test.
7. **After the field test (S).** `kNetworkProfileV2` with the owner's choice as the compiled default; v1 kept as
   the recovery profile. Optional phase 2: asymmetric per-role modulation (M, needs its own short design check of
   the per-slot reconfiguration time on the ESP8285).

## 7. Open decisions for the owner

1. Extend LINK_CONFIG / LINK_STATS and add REQUEST action 25 (schema hash change, cross-project), or the minimal
   variant with 4 fixed candidates in the reserved LINK_CONFIG flag bits and no GS control?
2. Entry points: GS REQUEST, Lua menu, or both?
3. Accept the duty cycle rise from 26 % to about 40 % per end at unchanged 10 mW.
4. Candidate list for the field test (proposal P0 to P5) and the decision rule in 5.2 step 7.
5. Phase 2 asymmetric uplink: wanted?
6. Ship-node RX diversity (ESP32 diversity board, existing env): consider as a hardware option?
7. Payload reductions (network-budget P2, P3/P4, P8; `AIR_ACTUATORS` rate) if more than about 4 dB is needed.

## 8. Trial: SF6 / BW1600 / CR 4/5 with 255 B frames ("v2-trial", implemented 2026-11-26)

Owner decision: try SF6 / BW1600 / CR 4/5 with the full 255 B frame on the Starship link, although it is above the
16.5 ms budget of section 1. This is an experiment, selected at **build time only** (no runtime switch). v1 stays the
default; FPV is untouched (the FPV golden-trace test passes unchanged).

### 8.1 Exact timing (repo formula `fhss::loraSymbolCount` / `loraTimeOnAirUs`, 255 B, preamble 12, implicit header, CRC on)

| | SF5 (v1) | SF6 (v2-trial) |
|---|---|---|
| symbols | 536.25 | 451.25 |
| symbol time | 19.69 us | 39.38 us |
| time on air | 10.560 ms | **17.772 ms** (the 17.9 ms estimate was 0.13 ms high) |

Slot code facts (`FHSS::taskThread` / `timingControl`): the task wakes `wakeupLeadNs` = 2.5 ms before each boundary;
in that wakeup it processes the **upcoming** slot (hop, TX prep, FIFO load) and busy-waits to the boundary. The grace
window waits for an in-flight reception until 0.5 ms before the boundary when the next slot is TX. The 20 ms slot,
1:1 roles, 2.5 ms lead, 255 B frame, hop key and link version are identical to v1; only `lora.sf` differs (plus the RX
lead below).

Margins of v2-trial (all computed by `netSlotTiming()` and checked by tests):

| Quantity | Value |
|---|---|
| Strict budget (20 - 2.5 - 1.0 = 16.5 ms) | **fails by 1.27 ms** |
| Frame end vs the TX wakeup (17.5 ms) | frame still in flight by 0.27 ms: **every frame goes through the grace path** |
| Frame end vs the grace cutoff (19.5 ms) | 1.73 ms |
| Frame end vs 19.5 ms minus the 1.0 ms margin | **0.73 ms remaining margin** |
| Time left after RX_DONE for RX fetch + TX prep + busy-wait | 2.23 ms (v1: 2.5 ms of prep, after an RX that ended long before) |

### 8.2 Finding: the plain 255 B SF6 frame does not work with the v1 slot code

Two defects showed up in the FHSS simulation (`native_fhss`), both only when the frame is longer than
`slot - wakeupLead` = 17.5 ms. Neither exists at SF5:

1. **The own TX is cut.** In a 1:1 link the slot after an own TX is an RX slot. Its wakeup (17.5 ms after the TX
   start) hops the channel and arms RX, but the own 17.77 ms frame is still on air for 0.27 ms. `setRx` aborts a
   running TX: the frame arrives truncated and fails the CRC, in both directions. The doc's section 1 budget only
   looked at the receiving end and missed this.
2. **The TX module (master) never locks.** The grace window is gated on `fhssState != Searching`, but the TX
   side's state is derived from "a packet in the last 1 s", so before the first downlink it is Searching and the
   window is skipped: the master aborts every downlink frame at its wakeup, forever.

Fixes (FHSS lib, active only for profiles where `wakeupSlackNs < 0`, i.e. v2-trial; v1 and FPV paths are
bit-identical, the FPV golden trace and all v1 network tests pass unchanged):

- `AirProfile::wakeupLeadRxNs` (new field, default 0 = same as `wakeupLeadNs`). v2-trial sets 1.0 ms: the wakeup
  before an RX slot happens 1.0 ms before the boundary (19.0 ms), 1.23 ms after the own TX has ended. The wakeup
  before a TX slot keeps the 2.5 ms lead (TX prep).
- `FHSS::rxInFlightAtWakeup` (set by `applyProfile` from `netSlotTiming`): the TX side gets the grace window in
  `Searching` too.
- `netSlotTiming(profile)` (AirProfile.hpp) derives all of the above from the profile; `fitsWithGrace` requires
  both the grace budget (>= 1 ms before the cutoff minus 0.5 ms) and the own-TX-done slack (>= 1 ms). The firmware
  (`NetRadio.cpp`) falls back to v1 if the active profile does not satisfy it. `NetAirAccess::kMaxAir` and the
  frame length come from the profile (both 255, static_assert).

### 8.3 Build instructions

Both ends must be built with the same profile. Plain envs = v1 (default):

```
pio run -e exvectrlink_tx_module_network_v2trial      # TX module (Pocket), -DEXVECTRLINK_NET_PROFILE_V2_TRIAL
pio run -e exvectrlink_rx_network_esp8285_v2trial     # ship node (ESP8285); OTA upload as the v1 env
pio run -e exvectrlink_tx_module_network               # back to v1
pio run -e exvectrlink_rx_network_esp8285
```

A v1 unit and a v2-trial unit cannot talk (different modulation); re-flash both to go back. The PA variant and the
diversity ship node can get a `_v2trial` env the same way (`extends` plus the flag), not added yet.

### 8.4 Tests

- `native` `test_air_link`: `test_v2_trial_profile_timing` (differs from v1 only by SF and RX lead; ToA 17.772 ms;
  strict budget refused, margins above; usual-lead variant, SF7 and a 10 ms slot refused),
  `test_active_profile_selection_and_frame_limits` (v1 is the default).
- `native_fhss` `test_fhss`: `test_net_v2_trial_locks_and_holds_with_17_8ms_frames` (20 s, crystal mismatch, 500/500
  frames each way, no own frame cut), `test_net_v2_trial_with_single_lead_cuts_own_frames` (negative control: the
  fake radio now models "TX aborted by RX arm or hop"). `test_fpv_trace_is_unchanged` unchanged.

### 8.5 Bench test for the owner (hardware, not done)

1. Flash both `_v2trial` envs. Watch lock time, LQ, `graceRescueCount` / `graceExpireCount` / `missedSlotCounter` on
   the ship node (ESP8285 at 160 MHz). `graceRescueCount` rising 1:1 with received frames is expected;
   `graceExpireCount` or missed slots rising means RX fetch + TX prep do not fit the 2.23 ms left: stop, the
   trial profile is too tight for that hardware.
2. Compare mean and 10th-percentile LQ and SNR at the same RSSI against v1 at the two field positions (5.2).
3. Confirm the ESP8285 SPI time for a 255 B FIFO read plus write is well below 2 ms (the simulation uses no
   hardware latency for it).

# ExVectrLink: Developer Documentation

These docs are versioned with the code. **Keep them current:** a change isn't finished until the docs it affects are updated (see [development-workflow.md](development-workflow.md#documentation-rule)).

## Start here

| Document | Read it for |
|---|---|
| [development-workflow.md](development-workflow.md) | Branch model, submodule rules, the second working copy, compatibility and version bumps, documentation rule |
| [codebase-structure.md](codebase-structure.md) | Repo layout, `src/` vs the `lib/ExVectrLink` library, what each submodule is for, runtime model (scheduler, tasks, interrupts) |
| [build-targets-and-hardware.md](build-targets-and-hardware.md) | Every `[env:...]`: board, role, flags, pin maps, PSRAM note, build status |
| [air-protocol.md](air-protocol.md) | FHSS slots/hops/sync, radio settings and presets, OTA frame and trailer, packet types, RC packets, data, link telemetry, diversity, dynamic power, bind, WiFi upload mode, sync failsafe |
| [serial-protocol.md](serial-protocol.md) | Note: the host ↔ module serial protocol (`SerialTelecoms`) was removed, and why |
| [network-mode.md](network-mode.md) | **ExVectrNetwork mode** firmwares: TX module (node 2) and vehicle receiver (node 3) routers, FHSS air link with air profile v1, sync, diversity, power, filters, WiFi upload, host tests, bench plan |
| [crsf.md](crsf.md) | CRSF input from the handset, Lua parameter menu, telemetry relay, CRSF/SBUS output to flight controllers |
| [settings.md](settings.md) | What is persisted in EEPROM, and how to add a setting |
| [known-issues.md](known-issues.md) | Problems found while documenting (not fixed) |
| [roadmap.md](roadmap.md) | Planned work: link modes (protocol / rate / hopping), TX-module USB bridge and network mode (air link done, runtime mode switch open), design notes |

## Doc conventions

- Every doc states `Last verified against commit:` (parent repo commit, plus the submodule pins where relevant). If the code has moved past it, treat the doc as possibly stale and re-check the parts you rely on.
- Code is cited as `path:line`, relative to the repo root (`src/...`, `lib/ExVectrLink/...`). Line numbers drift, so re-check them when editing.
- `feature/gs-requests-and-upload` changed `src_network/` (egress filters: only the ground-station heartbeat and RC_STATE are dropped over radio; new `LinkUploadCommand`), the network-mode tests and two pins (ExVectrNetwork `addSrcPortDrop`, ExVectrPackets with `LINK_UPLOAD_MODE`). Nothing in `src/` changed.
- `feature/air-link-fhss` changed the FHSS library (runtime parameters, FPV defaults unchanged and checked by a golden-trace test), `src_network/`, the network envs and tests, and two ExVectrNetwork additions. Nothing in `src/` changed.
- `feature/tx-network-router` changed only what the network-mode docs and the files listed there describe (new `src_network/`, new envs, library pins); the FPV code in `src/` is untouched, and docs about it keep their older `Last verified` commit.
- `feature/remove-serial` deleted code from `src/main.cpp`, `src/DatalinkCommandManager.cpp` and `src/WifiUpload.cpp`. Where those docs cite line numbers in those files, the numbers were not all re-checked and may be off. Their `Last verified` commit is deliberately left at the pre-removal commit.
- **Network-mode names:** Router = TX module = node 2 = the Radiomaster Pocket module; ship radio node (ship node) = RX module = node 3 (glossary in [network-mode.md](network-mode.md)). User-facing text says Router / ship node; code identifiers and env names keep TX module / RX module.
- `feature/network-power-cap` added to `src_network/` the Router Lua menu (`NetLuaMenu.hpp`), the one manual power setting (`NetPower.hpp`), `LINK_CONFIG` (ExVectrPackets `feature/link-config`) and `ShipRebootRequester.hpp`; dynamic power is off in network mode. Nothing in `src/` changed.
- `feature/cmd-uplink` changed `src_network/` (Router and ship node filters: CMD_STATE 0x13 node 0 -> vehicle only, RC_STATE off the air), the network-mode tests, and the ExVectrPackets pin (49d89c6). Nothing in `src/` changed.
- "TX side" means the handset end (`IS_TX_SIDE`). "RX side" means the vehicle end. "Uplink" is TX → RX, "downlink" is RX → TX.

Last verified against commit: ce2b498 + `feature/gs-requests-and-upload` (submodules: ExVectrNetwork de708ca, ExVectrPackets 4ca8fb1 local). Docs that this branch did not touch keep their own, older commit.

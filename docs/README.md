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
| [crsf.md](crsf.md) | CRSF input from the handset, Lua parameter menu, telemetry relay, CRSF/SBUS output to flight controllers |
| [settings.md](settings.md) | What is persisted in EEPROM, and how to add a setting |
| [known-issues.md](known-issues.md) | Problems found while documenting (not fixed) |
| [roadmap.md](roadmap.md) | Planned work: link modes (protocol / rate / hopping), TX-module USB bridge (to be designed), design notes |

## Doc conventions

- Every doc states `Last verified against commit:` (parent repo commit, plus the submodule pins where relevant). If the code has moved past it, treat the doc as possibly stale and re-check the parts you rely on.
- Code is cited as `path:line`, relative to the repo root (`src/...`, `lib/ExVectrLink/...`). Line numbers drift, so re-check them when editing.
- `feature/remove-serial` deleted code from `src/main.cpp`, `src/DatalinkCommandManager.cpp` and `src/WifiUpload.cpp`. Where those docs cite line numbers in those files, the numbers were not all re-checked and may be off. Their `Last verified` commit is deliberately left at the pre-removal commit.
- "TX side" means the handset end (`IS_TX_SIDE`). "RX side" means the vehicle end. "Uplink" is TX → RX, "downlink" is RX → TX.

Last verified against commit: 4830874 (submodules: ExVectrLink 37ab8e2, ExVectrPackets 46286bd, ExVectrNetwork 7cb0e79, ExVectrCore 18bbb09)

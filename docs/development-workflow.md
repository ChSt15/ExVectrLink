# Development Workflow

> How branches, submodules and docs are handled in ExVectrLink. These rules are binding, including for Claude sessions. The model matches the flight software's workflow (`../flight-software/docs/development-workflow.md` in the workspace).

Last verified against commit: 4830874

## Branch model

| Branch | Role | Who writes to it |
|---|---|---|
| `master` | Released state: the firmware flashed on devices in use (FPV and Starship). Must always build. | Only merges from `Development`, after user approval. |
| `Development` | Integration. Finished features merged together and verified on hardware. Must always build. | Merges from feature branches only (plus trivial doc or pin fixes). |
| `feature/<name>` | One feature, fix or experiment, cut from `Development`. | Free to commit. |

Older remote branches (`MultiAux`, `PermanentSettings`, `TxAntennaSwitching`) predate this model. Treat them as archived.

### Feature lifecycle

1. Start from an up-to-date `Development`:
   ```sh
   git checkout Development && git pull
   git submodule update --init --recursive
   git checkout -b feature/<name>
   ```
2. Implement, build at least one RX and one TX env, and bench-test both ends on hardware.
3. Merge the newest `Development` into the branch, rebuild and re-test.
4. Merge into `Development` with `--no-ff`. In the ExVectrStarship workspace this is done by the main session after user approval.
5. Update `docs/` in the same branch.

## The second working copy

The user also develops in `~/Development/PlatformIO/ExVectrLink`. The copy inside the ExVectrStarship workspace and that copy are **separate clones that sync only through GitHub**.
- Never modify the other copy from here.
- Before starting work, `git fetch` and check whether `Development` moved.
- Expect the other copy to have unpushed work. Coordinate through the user before rebasing or force-pushing anything shared.
- Machine-local files differ between the copies. For example `src/WifiCredentials.cpp` is gitignored, and `lib/SBUS` is pinned to a commit that may not be on its remote.

## Submodules

`lib/ExVectrArduinoPlatform`, `lib/ExVectrCore`, `lib/ExVectrData`, `lib/ExVectrHAL`, `lib/ExVectrLink`, `lib/ExVectrNetwork`, `lib/ExVectrPackets` and `lib/SBUS` are git submodules. What each is used for is in [codebase-structure.md](codebase-structure.md#submodules).

- **Library changes belong in the library repo.** Create a branch with the same name in the submodule, cut from the library's `Development`. Commit there, then `git add lib/<Lib>` in the parent to pin it.
- **Merging:**
  1. Merge the library feature branch into the library's `Development` first, and push it.
  2. Re-pin the parent to that merge commit.
  3. Then merge the parent feature branch.
- **Never pin an unpushed submodule commit** on `Development` or `master`:
  ```sh
  git submodule foreach 'git branch -r --contains HEAD | grep -q . || echo "UNPUSHED: $name"'
  ```
  Known exception: `lib/SBUS` (see [known-issues.md](known-issues.md)).
- **No dirty submodules** when committing the parent (`git status` must not show ` m lib/...`).
- The ExVectr libraries are shared with the flight software, which pins its own versions. ExVectrLink's ExVectrNetwork pin (7cb0e79) is 29 commits ahead of the flight software's (cabddce) and has a different datalink API.

## Compatibility and version bumps

- **FPV first:** existing FPV setups (TX module → RX with CRSF to Betaflight/INAV) must keep working with the default settings. New behaviour is opt-in at runtime.
- **Air protocol** (frame layout, trailer, type bytes, timing): both ends must run compatible firmware. On an incompatible change, bump `FHSS::OTA_VERSION` (`lib/ExVectrLink/include/ExVectrLink/datalink/FHSS.hpp:184`). It seeds the trailer checksum, so mismatched ends simply reject each other's packets instead of misreading them.
- **Serial protocol:** removed (see [serial-protocol.md](serial-protocol.md)); `ExVectrLinkVersion` no longer exists. A future host interface needs its own version rule, agreed when it is designed.
- **EEPROM layout:** give a settings struct a new `kStorageKey` when its layout changes ([settings.md](settings.md)).

## Commit conventions

- Imperative subject of at most 72 characters. The body explains why.
- One logical change per commit.
- Bumping a submodule pin: `Pin ExVectrLink: <what changed>`.
- Never commit `.pio/`, `src/WifiCredentials.cpp` or other secrets.

## Documentation rule

- Any change to behaviour, interfaces, timing, wiring or workflow updates the relevant doc **in the same branch**.
- Bugs found but not fixed go into [known-issues.md](known-issues.md). Remove them when they are fixed.
- Update `Last verified against commit:` when you re-verify a doc.

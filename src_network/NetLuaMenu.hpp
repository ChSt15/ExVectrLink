#pragma once
// EdgeTX / ELRS Lua parameter menu of the network-mode Router (node 2, Radiomaster Pocket module). Pure and
// host-tested: no Arduino, no heap. The wire format is the CRSF parameter protocol the FPV firmware also speaks
// (src/CRSFRcInput.cpp: ParameterPing -> DeviceInfo, ParameterRead -> entry 0x2B, ParameterWrite). Entries are
// built here with their own encoder (not the 16/32 byte legacy packet classes) so labels can be longer; every
// entry is one chunk and at most 64 bytes.
//
// Entries (all in the root folder):
//   1 Power (10mW legal)   TEXT_SELECTION  dBm levels up to the board maximum; RAM only, boot value 10 dBm
//   2 Power (mW)           INFO            the selected level in mW, so the label is readable
//   3 TX antenna           TEXT_SELECTION  Auto / A / B (RAM only)
//   4 Link                 INFO            air state and LQ
//   5 Ship power           INFO            the ship radio node's reported tx_power (its LINK_STATS), "no data" when stale
//   6 Reboot router        COMMAND         with confirmation step
//   7 Reboot ship node     COMMAND         with confirmation step (REQUEST LINK_REBOOT target 3, repeated until acked)
//
// Time is bounded: every call builds at most one frame from a few snprintf()s and does no I/O.
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ExVectrPackets/gen/all.hpp"

#include "NetCrsf.hpp"
#include "NetPower.hpp"

namespace VCTR::ExVectrLink::netmode {

/// What the menu does to the firmware. Implemented by the application (and by a recorder in the host tests).
class LuaMenuActions {
public:
  virtual ~LuaMenuActions() = default;
  virtual void setTxPower(int8_t dbm) = 0;
  /// 0 = Auto (diversity), 1 = antenna A, 2 = antenna B.
  virtual void setTxAntenna(uint8_t selection) = 0;
  virtual void rebootRouter() = 0;
  virtual void rebootShipNode() = 0;
};

/// Read-only snapshot the application supplies for each call.
struct LuaMenuStatus {
  int8_t txPowerDbm = 10;     ///< the Router's power variable (what it applies and broadcasts)
  uint8_t txAntennaSel = 0;   ///< 0 Auto, 1 A, 2 B
  Packets::AirState airState = Packets::AirState::kOff;
  uint8_t rxLq = 0;           ///< percent
  bool shipValid = false;     ///< a fresh LINK_STATS from node 3 exists
  int8_t shipPowerDbm = 0;    ///< its tx_power
};

class NetLuaMenu {
public:
  static constexpr uint8_t kParamPower = 1;
  static constexpr uint8_t kParamPowerMw = 2;
  static constexpr uint8_t kParamAntenna = 3;
  static constexpr uint8_t kParamLink = 4;
  static constexpr uint8_t kParamShipPower = 5;
  static constexpr uint8_t kParamRebootRouter = 6;
  static constexpr uint8_t kParamRebootShip = 7;
  static constexpr uint8_t kParamCount = 7;

  static constexpr uint8_t kAddrBroadcast = 0x00;
  static constexpr uint8_t kAddrHandset = 0xEA;
  static constexpr uint8_t kAddrModule = 0xEE;
  static constexpr uint8_t kAddrElrsLua = 0xEF;

  static constexpr int64_t kConfirmTimeoutNs = 10000000000LL; // 10 s to confirm
  static constexpr int64_t kProgressShowNs = 2000000000LL;    // "done" shown for 2 s

  static constexpr int8_t kLevels[] = {0, 5, 10, 13, 17, 20, 23, 27};
  static constexpr uint8_t kNumLevels = sizeof(kLevels) / sizeof(kLevels[0]);

  explicit NetLuaMenu(int boardMaxDbm) : boardMax_(boardMaxDbm) {}

  // ---- power level table (pure helpers) -----------------------------------------------------------------------
  /// Number of levels offered: those <= the board maximum (at least the first).
  uint8_t levelCount() const {
    uint8_t n = 0;
    for (uint8_t i = 0; i < kNumLevels; i++)
      if (kLevels[i] <= boardMax_) n++;
    return n ? n : 1;
  }
  /// dBm of option `index` (clamped to the offered range).
  int8_t levelAt(uint8_t index) const {
    const uint8_t n = levelCount();
    return kLevels[index < n ? index : n - 1];
  }
  /// Index of the highest offered level <= dbm (0 if below all).
  uint8_t indexFor(int dbm) const {
    uint8_t idx = 0;
    const uint8_t n = levelCount();
    for (uint8_t i = 0; i < n; i++)
      if (kLevels[i] <= dbm) idx = i;
    return idx;
  }
  /// Power in milliwatts for a dBm value, rounded (10 dBm -> 10, 27 dBm -> 501).
  static uint32_t milliwatts(int dbm) {
    return static_cast<uint32_t>(pow(10.0, dbm / 10.0) + 0.5);
  }
  /// Option list "0;5;10;..." (fits the 64 byte frame together with the 18 character label).
  void buildPowerOptions(char *out, size_t size) const {
    size_t pos = 0;
    out[0] = '\0';
    for (uint8_t i = 0; i < levelCount(); i++) {
      const int w = snprintf(out + pos, size - pos, i ? ";%d" : "%d", kLevels[i]);
      if (w < 0 || static_cast<size_t>(w) >= size - pos) break;
      pos += static_cast<size_t>(w);
    }
  }

  // ---- ping -----------------------------------------------------------------------------------------------------
  /// True if a ParameterPing to `dest` is for this module (broadcast or the module address).
  static bool pingIsForUs(uint8_t dest) { return dest == kAddrBroadcast || dest == kAddrModule; }
  /// ELRS Lua replies are addressed to the handset (EdgeTX only routes a reply back to the script that way).
  static uint8_t replyDest(uint8_t origin) { return origin == kAddrElrsLua ? kAddrHandset : origin; }

  // ---- read / write ---------------------------------------------------------------------------------------------
  /// ParameterRead of `param` from `origin`. Fills `out` and returns true if the module answers (unknown numbers,
  /// including 0, are not answered).
  bool buildRead(uint8_t param, uint8_t origin, const LuaMenuStatus &st, int64_t nowNs, CrsfFrame &out) {
    expireCommands(nowNs);
    return buildEntry(param, replyDest(origin), st, out);
  }

  /// ParameterWrite of `value` to `param`. Applies it through `act`. Returns true and fills `out` if a reply
  /// frame is due (command entries only; selections just apply, the Lua re-reads them).
  bool handleWrite(uint8_t param, uint8_t value, uint8_t origin, const LuaMenuStatus &st, int64_t nowNs,
                   LuaMenuActions &act, CrsfFrame &out) {
    expireCommands(nowNs);
    const uint8_t dest = replyDest(origin);
    switch (param) {
    case kParamPower:
      act.setTxPower(levelAt(value));
      return false;
    case kParamAntenna:
      act.setTxAntenna(value > 2 ? 0 : value);
      return false;
    case kParamRebootRouter:
    case kParamRebootShip:
      return handleCommand(param, value, dest, st, nowNs, act, out);
    default:
      return false;
    }
  }

  /// Command state of entry `param` (tests): Ready, ConfirmationNeeded or Progress.
  uint8_t commandStep(uint8_t param) const {
    const Command *c = cmd(param);
    return c ? c->step : 0;
  }

private:
  enum : uint8_t { kReady = 0, kStart = 1, kProgress = 2, kConfirmNeeded = 3, kConfirm = 4, kCancel = 5, kPoll = 6 };
  struct Command {
    uint8_t step = kReady;
    int64_t sinceNs = 0;
    char info[24] = "";
  };

  Command *cmd(uint8_t param) {
    return param == kParamRebootRouter ? &cmds_[0] : (param == kParamRebootShip ? &cmds_[1] : nullptr);
  }
  const Command *cmd(uint8_t param) const {
    return param == kParamRebootRouter ? &cmds_[0] : (param == kParamRebootShip ? &cmds_[1] : nullptr);
  }

  /// A confirmation not given within 10 s lapses; "running" is shown for 2 s.
  void expireCommands(int64_t nowNs) {
    for (Command &c : cmds_) {
      if (c.step == kConfirmNeeded && nowNs - c.sinceNs > kConfirmTimeoutNs) setStep(c, kReady, "", nowNs);
      else if (c.step == kProgress && nowNs - c.sinceNs > kProgressShowNs) setStep(c, kReady, "", nowNs);
    }
  }
  static void setStep(Command &c, uint8_t step, const char *info, int64_t nowNs) {
    c.step = step;
    c.sinceNs = nowNs;
    snprintf(c.info, sizeof c.info, "%s", info);
  }

  bool handleCommand(uint8_t param, uint8_t value, uint8_t dest, const LuaMenuStatus &st, int64_t nowNs,
                     LuaMenuActions &act, CrsfFrame &out) {
    Command &c = *cmd(param);
    const bool ship = param == kParamRebootShip;
    if (value == kStart) {
      // Click: ask for confirmation first (a reboot of the ship node in flight stops RC for seconds).
      setStep(c, kConfirmNeeded, ship ? "Reboot ship node?" : "Reboot router?", nowNs);
    } else if (value == kConfirm) {
      if (c.step == kProgress) {
        // already executed: a repeated confirm changes nothing
      } else if (c.step != kConfirmNeeded) {
        setStep(c, kReady, "", nowNs);
      } else {
        setStep(c, kProgress, ship ? "Request sent" : "Rebooting", nowNs);
        if (ship) act.rebootShipNode();
        else act.rebootRouter();
      }
    } else if (value == kCancel) {
      setStep(c, kReady, "", nowNs);
    } else if (value == kPoll) {
      if (c.step == kProgress) return false; // keep showing it until it lapses (same as the FPV menu)
    } else {
      return false;
    }
    return buildEntry(param, dest, st, out);
  }

  // ---- frame encoding ---------------------------------------------------------------------------------------------
  struct Writer {
    uint8_t buf[kCrsfMaxFrame];
    size_t n = 0;
    bool ok = true;
    void u8(uint8_t v) {
      if (n < sizeof buf) buf[n++] = v;
      else ok = false;
    }
    void str(const char *s) {
      while (*s) u8(static_cast<uint8_t>(*s++));
      u8(0);
    }
  };
  static constexpr uint8_t kFrameParamEntry = 0x2B;
  static constexpr uint8_t kTypeTextSelection = 0x09, kTypeInfo = 0x0C, kTypeCommand = 0x0D;

  static void header(Writer &w, uint8_t dest, uint8_t param, uint8_t dataType) {
    w.u8(dest);
    w.u8(kAddrModule);
    w.u8(param);
    w.u8(0); // chunks remaining: one chunk
    w.u8(0); // parent folder: root
    w.u8(dataType);
  }
  static bool finish(const Writer &w, CrsfFrame &out) { return w.ok && crsfBuildFrame(out, kFrameParamEntry, w.buf, w.n); }

  static bool selection(CrsfFrame &out, uint8_t dest, uint8_t param, const char *name, const char *options,
                        uint8_t value, uint8_t max, uint8_t def, const char *unit) {
    Writer w;
    header(w, dest, param, kTypeTextSelection);
    w.str(name);
    w.str(options);
    w.u8(value);
    w.u8(0);
    w.u8(max);
    w.u8(def);
    w.str(unit);
    return finish(w, out);
  }
  static bool info(CrsfFrame &out, uint8_t dest, uint8_t param, const char *name, const char *text) {
    Writer w;
    header(w, dest, param, kTypeInfo);
    w.str(name);
    w.str(text);
    return finish(w, out);
  }
  static bool command(CrsfFrame &out, uint8_t dest, uint8_t param, const char *name, const Command &c) {
    Writer w;
    header(w, dest, param, kTypeCommand);
    w.str(name);
    w.u8(c.step);
    w.u8(200); // timeout in 100 ms units (2 s), as the FPV commands
    w.str(c.info);
    return finish(w, out);
  }

  static const char *airStateName(Packets::AirState s) {
    switch (s) {
    case Packets::AirState::kSearching: return "Searching";
    case Packets::AirState::kSyncing: return "Syncing";
    case Packets::AirState::kSynced: return "Synced";
    default: return "Off";
    }
  }

  bool buildEntry(uint8_t param, uint8_t dest, const LuaMenuStatus &st, CrsfFrame &out) const {
    char text[32];
    switch (param) {
    case kParamPower: {
      char opts[40];
      buildPowerOptions(opts, sizeof opts);
      return selection(out, dest, param, "Power (10mW legal)", opts, indexFor(st.txPowerDbm),
                       static_cast<uint8_t>(levelCount() - 1), indexFor(kDefaultDbm), "dBm");
    }
    case kParamPowerMw:
      snprintf(text, sizeof text, "%lu mW", static_cast<unsigned long>(milliwatts(st.txPowerDbm)));
      return info(out, dest, param, "Power (mW)", text);
    case kParamAntenna:
      return selection(out, dest, param, "TX antenna", "Auto;A;B", st.txAntennaSel > 2 ? 0 : st.txAntennaSel, 2, 0, "");
    case kParamLink:
      snprintf(text, sizeof text, "%s LQ %u%%", airStateName(st.airState), static_cast<unsigned>(st.rxLq));
      return info(out, dest, param, "Link", text);
    case kParamShipPower:
      if (st.shipValid) snprintf(text, sizeof text, "%d dBm", static_cast<int>(st.shipPowerDbm));
      else snprintf(text, sizeof text, "no data");
      return info(out, dest, param, "Ship power", text);
    case kParamRebootRouter:
      return command(out, dest, param, "Reboot router", cmds_[0]);
    case kParamRebootShip:
      return command(out, dest, param, "Reboot ship node", cmds_[1]);
    default:
      return false;
    }
  }

  static constexpr int kDefaultDbm = kNetTxPowerDefaultDbm;
  int boardMax_;
  Command cmds_[2];
};

} // namespace VCTR::ExVectrLink::netmode

//
//    openSAM: manage DGS and jetways for X Plane
//
//    Copyright (C) 2024, 2025, 2026  Holger Teutsch
//
//    This library is free software; you can redistribute it and/or
//    modify it under the terms of the GNU Lesser General Public
//    License as published by the Free Software Foundation; either
//    version 2.1 of the License, or (at your option) any later version.
//
//    This library is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//    Lesser General Public License for more details.
//
//    You should have received a copy of the GNU Lesser General Public
//    License along with this library; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
//    USA
//

#include <cassert>
#include "opensam.h"
#include "os_plane.h"
#include "samjw.h"
#include "log_msg.h"

int OsPlane::id_base_;

static const float kAnimInterval = -1;  // s for debugging or -1 for frame loop

const char* const OsPlane::state_str_[] = {"DISABLED", "IDLE",   "PARKED",    "SELECT_JWS", "CAN_DOCK",
                                         "DOCKING",  "DOCKED", "UNDOCKING", "CANT_DOCK"};

OsPlane::~OsPlane() {
    LogMsg("pid=%02d, Plane destructor, state: %s, active_jws: %d", id_, state_str_[state_], (int)active_jws_.size());
    if (kIdle <= state_) {
        for (int ajw_idx : active_jws_)
            nearest_jws_[ajw_idx].ResetJw();
    }

    LogMsg("pid=%02d, Plane destructor finished", id_);
}

static bool FitsDoor(const JwCtrl& njw, bool allow_soft) {
    const SamJw* jw = njw.jw_;
    if (!is_between(njw.docked_rot3_, jw->min_rot3, jw->max_rot3))
        return false;
    if (is_between(njw.docked_rot1_, jw->min_rot1, jw->max_rot1) && is_between(njw.docked_rot2_, jw->min_rot2, jw->max_rot2) &&
        is_between(njw.docked_extent_, jw->min_extent, jw->max_extent))
        return true;
    return allow_soft && njw.docked_extent_ - jw->max_extent < 10.0f;
}

static bool DockedPathsCross(const JwCtrl& a, const JwCtrl& b) {
    auto side = [](float ax, float az, float bx, float bz, float cx, float cz) {
        return (bx - ax) * (cz - az) - (bz - az) * (cx - ax);
    };
    float d1 = side(b.x_, b.z_, b.docked_x_, b.docked_z_, a.x_, a.z_);
    float d2 = side(b.x_, b.z_, b.docked_x_, b.docked_z_, a.docked_x_, a.docked_z_);
    float d3 = side(a.x_, a.z_, a.docked_x_, a.docked_z_, b.x_, b.z_);
    float d4 = side(a.x_, a.z_, a.docked_x_, a.docked_z_, b.docked_x_, b.docked_z_);
    bool cross = ((d1 > 0) != (d2 > 0)) && ((d3 > 0) != (d4 > 0));
    if (cross)
        LogMsg("docked paths of %s and %s cross", a.name(), b.name());
    return cross;
}

static bool AssignDoors(std::vector<JwCtrl>& jws, const std::vector<DoorInfo>& doors, unsigned n_doors, bool allow_soft,
                        std::vector<int>& chosen) {
    if (chosen.size() == n_doors)
        return true;

    const DoorInfo& door = doors[chosen.size()];
    for (int j = 0; j < static_cast<int>(jws.size()); j++) {
        bool taken = false;
        for (int c : chosen)
            if (c == j || jws[c].jw_ == jws[j].jw_) {
                taken = true;
                break;
            }
        if (taken)
            continue;

        jws[j].SetupForDoor(door);
        if (!FitsDoor(jws[j], allow_soft))
            continue;

        bool collides = false;
        for (int c : chosen)
            if (DockedPathsCross(jws[j], jws[c])) {
                collides = true;
                break;
            }
        if (collides)
            continue;

        chosen.push_back(j);
        if (AssignDoors(jws, doors, n_doors, allow_soft, chosen))
            return true;
        chosen.pop_back();
    }

    return false;
}

// auto select active jetways: all doors when every door gets a jetway, otherwise the first two doors, then one
void OsPlane::AutoSelectJws() {
    if (door_info_.empty())
        return;

    std::vector<int> chosen;
    bool done = false;
    int n_all = static_cast<int>(door_info_.size());
    for (int n = n_all; !done && n >= 1; n--) {
        if (n < n_all && n > 2)
            continue;
        for (bool allow_soft : {false, true}) {
            chosen.clear();
            if (AssignDoors(nearest_jws_, door_info_, n, allow_soft, chosen)) {
                done = true;
                break;
            }
        }
    }

    for (unsigned i_door = 0; i_door < chosen.size(); i_door++) {
        JwCtrl& jw = nearest_jws_[chosen[i_door]];
        jw.SetupForDoor(door_info_[i_door]);
        jw.door_ = i_door;
        jw.selected_ = true;
        active_jws_.push_back(chosen[i_door]);
        LogMsg("active jetway for door %d: %s", i_door, jw.name());
    }

    if (active_jws_.empty())
        LogMsg("Oh no, no active jetways left in AutoSelectJws()!");
}

// the state machine called from the flight loop
float OsPlane::JwStateMachine() {
    if (state_ == kDisabled) {
        state_machine_next_ts_ = ::now + 2.0f;
        return 2.0f;
    }

    if (state_machine_next_ts_ > ::now)  // action is not due
        return state_machine_next_ts_ - ::now;

    State new_state{state_};

    // only MyPlane has a meaningful CheckTeleportation() implementation
    if (state_ > kIdle && CheckParkedTeleportation()) {
        LogMsg("teleportation detected!");
        prev_state_ = state_;
        state_ = new_state = kIdle;
        state_change_ts_ = now;
        LogMsg("pid=%02d, new state: IDLE after teleportation", id_);
        // FALLTHROUGH into IDLE below
    }

    unsigned n_done;

    switch (state_) {
        case kIdle:
            if (prev_state_ != kIdle) {
                // from anywhere to idle nullifies all selections and locks
                for (int ajw_idx : active_jws_)
                    nearest_jws_[ajw_idx].ResetJw();
                active_jws_.clear();

                nearest_jws_.clear();
            }

            if (on_ground_ && !beacon_on_) {
                // memorize position teleportation detection
                MemorizeParkedPos();

                // reset stale command invocations
                dock_requested();
                undock_requested();
                toggle_requested();

                new_state = kParked;
            }
            break;

        case kParked:
            if (!JwCtrl::FindNearestJetways({id_, x_, y_, z_, psi_, door_info_}, nearest_jws_)) {
                new_state = kCantDock;
                break;
            }

            // falling through adds much concurrency to the MP case as unused jetways get unlocked much earlier
            nearest_jws_seqno_++;  // for detecting changes in the nearest jetway list by the UI
            new_state = state_ = kSelectJws;
            LogMsg("pid=%02d, new state: SELECT_JWS", id_);
            // FALLTHROUGH

        case kSelectJws:
            if (beacon_on_) {
                LogMsg("SELECT_JWS and beacon goes on");
                new_state = kIdle;
                break;
            }

            if (auto_mode()) {
                AutoSelectJws();
                if (active_jws_.empty()) {  // e.g. collisions, locked jws
                    nearest_jws_.clear();
                    new_state = kCantDock;
                    break;
                }
            }

            // or wait for GUI selection
            if (active_jws_.size()) {
                for (int ajw_idx : active_jws_) {
                    JwCtrl& ajw = nearest_jws_[ajw_idx];
                    LogMsg("pid=%02d, setting up active jw for door: %d", id_, ajw.door_);
                    ajw.SetupForDoor(door_info_[ajw.door_]);

                    if (ajw.door_ == 0)  // slightly slant towards the nose cone for door LF1
                        ajw.docked_rot2_ += 3.0f;
                }

                // unlock jws that were not selected as active jw
                for (auto& njw : nearest_jws_)
                    if (!njw.selected_)
                        njw.UnlockJw();

                new_state = kCanDock;
            }
            break;

        case kCanDock:
            if (beacon_on_) {
                LogMsg("CAN_DOCK and beacon goes on");
                new_state = kIdle;
            }

            // mp planes always dock directly
            if (dock_requested() || toggle_requested()) {
                LogMsg("pid=%02d, docking requested", id_);

                float start_ts = now + active_jws_.size() * 5.0f;
                for (int ajw_idx : active_jws_) {
                    // staggered start for docking high to low
                    start_ts -= 5.0f;
                    nearest_jws_[ajw_idx].SetupDockUndock(start_ts, with_alert_sound());
                }

                new_state = kDocking;
            }
            break;

        case kCantDock:
            if (!on_ground_ || beacon_on_) {
                new_state = kIdle;
                break;
            }
            break;

        case kDocking:
            n_done = 0;
            for (int ajw_idx : active_jws_) {
                if (nearest_jws_[ajw_idx].DockDrive())
                    n_done++;
            }

            if (n_done == active_jws_.size()) {
                if (call_pre_post_dock_cmd()) {
                    XPLMCommandRef cmdr = XPLMFindCommand("openSAM/post_dock");
                    if (cmdr)
                        XPLMCommandOnce(cmdr);
                }
                new_state = kDocked;
            } else {
                state_machine_next_ts_ = 0.0f;
                return kAnimInterval;
            }
            break;

        case kDocked:
            if (!on_ground_) {
                new_state = kIdle;
                break;
            }

            if (beacon_on_)
                LogMsg("pid=%d, DOCKED and beacon goes on", id_);

            if (beacon_on_ || undock_requested() || toggle_requested()) {
                LogMsg("undocking requested");

                float start_ts = now + active_jws_.size() * 5.0f;
                for (int ajw_idx : active_jws_) {
                    // staggered start for undocking high to low
                    start_ts -= 5.0f;
                    nearest_jws_[ajw_idx].SetupDockUndock(start_ts, with_alert_sound());
                }

                if (call_pre_post_dock_cmd()) {
                    XPLMCommandRef cmdr = XPLMFindCommand("openSAM/pre_undock");
                    if (cmdr)
                        XPLMCommandOnce(cmdr);
                }

                new_state = kUndocking;
            }
            break;

        case kUndocking:
            n_done = 0;
            for (int ajw_idx : active_jws_) {
                if (nearest_jws_[ajw_idx].UndockDrive())
                    n_done++;
            }

            if (n_done == active_jws_.size())
                new_state = kIdle;
            else {
                state_machine_next_ts_ = 0.0f;
                return kAnimInterval;
            }
            break;

        default:
            LogMsg("Bad state %d", state_);
            new_state = kDisabled;
            break;
    }

    prev_state_ = state_;

    if (new_state != state_) {
        state_change_ts_ = now;
        LogMsg("pid=%02d, jw state transition, %s -> %s, beacon: %d", id_, state_str_[state_], state_str_[new_state],
               beacon_on_);
        state_ = new_state;
        state_machine_next_ts_ = 0.0f;
        return -1;  // see you on next frame
    }

    state_machine_next_ts_ = ::now + 0.5f;
    return 0.5f;
}

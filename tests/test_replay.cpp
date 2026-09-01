#include "garuda/core/simulation_world.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace garuda;

int main() {
    std::cout << "[TEST] Running test_replay...\n";

    // 1. Record Simulation
    SimulationWorld w_record(9999, 0.0025);
    auto* d_rec = w_record.add_drone("GARUDA-01", {}, {0.0, 3.0, 0.0});
    d_rec->arm();
    w_record.start_recording();

    for (int tick = 0; tick < 500; ++tick) {
        double thr = 0.584 + 0.20 * std::sin(tick * 0.01);
        double pitch = 0.10 * std::sin(tick * 0.02);
        d_rec->set_attitude_setpoint(0.0, pitch, 0.0, thr);

        ReplayActionFrame frame{};
        frame.tick = tick;
        frame.drone_id = "GARUDA-01";
        frame.armed = true;
        frame.pitch_rad = pitch;
        frame.throttle = thr;
        w_record.replay().record_action(frame);

        w_record.step();
    }

    uint64_t record_final_hash = w_record.compute_world_state_hash();
    auto manifest = w_record.replay().manifest();
    w_record.finish_recording();

    std::cout << "  Recorded 500 ticks. Final Hash: 0x" << std::hex << record_final_hash << std::dec << "\n";

    // 2. Playback Simulation
    SimulationWorld w_play(9999, 0.0025);
    w_play.add_drone("GARUDA-01", {}, {0.0, 3.0, 0.0});
    w_play.replay().start_playback(manifest);

    for (int tick = 0; tick < 500; ++tick) {
        w_play.step();
    }

    uint64_t play_final_hash = w_play.compute_world_state_hash();
    std::cout << "  Replayed 500 ticks. Final Hash: 0x" << std::hex << play_final_hash << std::dec << "\n";

    auto rec_pos = d_rec->physics_state().position;
    auto play_pos = w_play.get_drone("GARUDA-01")->physics_state().position;
    std::cout << "  Record End Pos: (" << rec_pos.x << ", " << rec_pos.y << ", " << rec_pos.z << ")\n";
    std::cout << "  Play End Pos:   (" << play_pos.x << ", " << play_pos.y << ", " << play_pos.z << ")\n";

    assert(record_final_hash == play_final_hash && "Replay final state hash must exactly match recorded run");
    std::cout << "[TEST] test_replay: ALL CHECKS PASSED.\n";
    return 0;
}

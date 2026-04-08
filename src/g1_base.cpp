#include "g1_base.hpp"

#include <chrono>
#include <cmath>

#include <viam/sdk/log/logging.hpp>

namespace viam_unitree {

G1Base::G1Base(const viam::sdk::Dependencies& deps, const viam::sdk::ResourceConfig& cfg)
    : Base(cfg.name()) {
    auto attrs = cfg.attributes();
    std::string network_interface = "eth0";

    auto it = attrs.find("network_interface");
    if (it != attrs.end()) {
        if (auto* val = it->second.get<std::string>()) {
            network_interface = *val;
        }
    }

    VIAM_SDK_LOG(info) << "Initializing G1Base with network interface: " << network_interface;

    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
    loco_client_.Init();
    loco_client_.SetTimeout(10.f);

    VIAM_SDK_LOG(info) << "G1Base initialized successfully";
}

G1Base::~G1Base() {
    cancel_movement();
    loco_client_.StopMove();
}

std::vector<std::string> G1Base::validate(const viam::sdk::ResourceConfig& cfg) {
    return {};
}

void G1Base::cancel_movement() {
    {
        std::lock_guard<std::mutex> lock(mu_);
        cancelled_ = true;
    }
    cancel_cv_.notify_all();
}

void G1Base::stop(const viam::sdk::ProtoStruct& extra) {
    cancel_movement();
    loco_client_.StopMove();
    moving_ = false;
}

void G1Base::move_straight(int64_t distance_mm,
                           double mm_per_sec,
                           const viam::sdk::ProtoStruct& extra) {
    if (distance_mm == 0 || mm_per_sec == 0) {
        return;
    }

    double speed_mps = std::abs(mm_per_sec) / 1000.0;
    double duration_sec = std::abs(static_cast<double>(distance_mm)) / std::abs(mm_per_sec);
    double direction = (distance_mm > 0) ? 1.0 : -1.0;
    float vx = static_cast<float>(direction * speed_mps);

    {
        std::lock_guard<std::mutex> lock(mu_);
        cancelled_ = false;
    }
    moving_ = true;

    auto end_time =
        std::chrono::steady_clock::now() + std::chrono::duration<double>(duration_sec);

    while (std::chrono::steady_clock::now() < end_time) {
        {
            std::unique_lock<std::mutex> lock(mu_);
            if (cancelled_) {
                moving_ = false;
                return;
            }
        }

        loco_client_.Move(vx, 0, 0);

        {
            std::unique_lock<std::mutex> lock(mu_);
            cancel_cv_.wait_for(lock, std::chrono::milliseconds(100), [this] {
                return cancelled_;
            });
            if (cancelled_) {
                moving_ = false;
                return;
            }
        }
    }

    loco_client_.StopMove();
    moving_ = false;
}

void G1Base::spin(double angle_deg,
                  double degs_per_sec,
                  const viam::sdk::ProtoStruct& extra) {
    if (angle_deg == 0 || degs_per_sec == 0) {
        return;
    }

    double duration_sec = std::abs(angle_deg) / std::abs(degs_per_sec);
    double direction = (angle_deg > 0) ? 1.0 : -1.0;
    float vyaw = static_cast<float>(direction * std::abs(degs_per_sec) * M_PI / 180.0);

    {
        std::lock_guard<std::mutex> lock(mu_);
        cancelled_ = false;
    }
    moving_ = true;

    auto end_time =
        std::chrono::steady_clock::now() + std::chrono::duration<double>(duration_sec);

    while (std::chrono::steady_clock::now() < end_time) {
        {
            std::unique_lock<std::mutex> lock(mu_);
            if (cancelled_) {
                moving_ = false;
                return;
            }
        }

        loco_client_.Move(0, 0, vyaw);

        {
            std::unique_lock<std::mutex> lock(mu_);
            cancel_cv_.wait_for(lock, std::chrono::milliseconds(100), [this] {
                return cancelled_;
            });
            if (cancelled_) {
                moving_ = false;
                return;
            }
        }
    }

    loco_client_.StopMove();
    moving_ = false;
}

void G1Base::set_power(const viam::sdk::Vector3& linear,
                       const viam::sdk::Vector3& angular,
                       const viam::sdk::ProtoStruct& extra) {
    // Map power (-1 to 1) to reasonable velocities for G1.
    // G1 max forward speed ~1.5 m/s, max rotation ~1.0 rad/s.
    constexpr float kMaxLinearVel = 1.5f;
    constexpr float kMaxAngularVel = 1.0f;

    float vx = static_cast<float>(linear.x) * kMaxLinearVel;
    float vy = static_cast<float>(linear.y) * kMaxLinearVel;
    float vyaw = static_cast<float>(angular.z) * kMaxAngularVel;

    moving_ = (vx != 0 || vy != 0 || vyaw != 0);
    loco_client_.Move(vx, vy, vyaw);
}

void G1Base::set_velocity(const viam::sdk::Vector3& linear,
                          const viam::sdk::Vector3& angular,
                          const viam::sdk::ProtoStruct& extra) {
    // Viam: linear in mm/s, angular in deg/s.
    // Unitree: linear in m/s, angular in rad/s.
    float vx = static_cast<float>(linear.x / 1000.0);
    float vy = static_cast<float>(linear.y / 1000.0);
    float vyaw = static_cast<float>(angular.z * M_PI / 180.0);

    moving_ = (vx != 0 || vy != 0 || vyaw != 0);
    loco_client_.Move(vx, vy, vyaw);
}

bool G1Base::is_moving() {
    return moving_;
}

viam::sdk::ProtoStruct G1Base::get_status() {
    viam::sdk::ProtoStruct status;
    status["is_moving"] = moving_.load();
    return status;
}

viam::sdk::Base::properties G1Base::get_properties(const viam::sdk::ProtoStruct& extra) {
    // G1 humanoid: ~0.45m wide, not wheeled.
    return {0.45, 0, 0};
}

viam::sdk::ProtoStruct G1Base::do_command(const viam::sdk::ProtoStruct& command) {
    auto it = command.find("command");
    if (it == command.end()) {
        return {};
    }

    auto* cmd = it->second.get<std::string>();
    if (!cmd) {
        return {};
    }

    viam::sdk::ProtoStruct result;
    int32_t rc = 0;

    if (*cmd == "stand_up") {
        rc = loco_client_.StandUp();
    } else if (*cmd == "sit") {
        rc = loco_client_.Sit();
    } else if (*cmd == "squat") {
        rc = loco_client_.Squat();
    } else if (*cmd == "high_stand") {
        rc = loco_client_.HighStand();
    } else if (*cmd == "low_stand") {
        rc = loco_client_.LowStand();
    } else if (*cmd == "balance_stand") {
        rc = loco_client_.BalanceStand();
    } else if (*cmd == "damp") {
        rc = loco_client_.Damp();
    } else if (*cmd == "zero_torque") {
        rc = loco_client_.ZeroTorque();
    } else if (*cmd == "wave_hand") {
        rc = loco_client_.WaveHand();
    } else if (*cmd == "start") {
        rc = loco_client_.Start();
    } else if (*cmd == "stop_move") {
        rc = loco_client_.StopMove();
    } else {
        result["error"] = std::string("unknown command: " + *cmd);
        return result;
    }

    result["rc"] = static_cast<double>(rc);
    return result;
}

std::vector<viam::sdk::GeometryConfig> G1Base::get_geometries(
    const viam::sdk::ProtoStruct& extra) {
    return {};
}

}  // namespace viam_unitree

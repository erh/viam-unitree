#pragma once

#include <atomic>
#include <condition_variable>
#include <mutex>

#include <viam/sdk/common/proto_value.hpp>
#include <viam/sdk/components/base.hpp>
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/module/service.hpp>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>

namespace viam_unitree {

class G1Base : public viam::sdk::Base {
   public:
    G1Base(const viam::sdk::Dependencies& deps, const viam::sdk::ResourceConfig& cfg);
    ~G1Base();

    static std::vector<std::string> validate(const viam::sdk::ResourceConfig& cfg);

    void stop(const viam::sdk::ProtoStruct& extra) override;

    void move_straight(int64_t distance_mm,
                       double mm_per_sec,
                       const viam::sdk::ProtoStruct& extra) override;

    void spin(double angle_deg,
              double degs_per_sec,
              const viam::sdk::ProtoStruct& extra) override;

    void set_power(const viam::sdk::Vector3& linear,
                   const viam::sdk::Vector3& angular,
                   const viam::sdk::ProtoStruct& extra) override;

    void set_velocity(const viam::sdk::Vector3& linear,
                      const viam::sdk::Vector3& angular,
                      const viam::sdk::ProtoStruct& extra) override;

    bool is_moving() override;

    viam::sdk::ProtoStruct get_status() override;

    viam::sdk::Base::properties get_properties(const viam::sdk::ProtoStruct& extra) override;

    viam::sdk::ProtoStruct do_command(const viam::sdk::ProtoStruct& command) override;

    std::vector<viam::sdk::GeometryConfig> get_geometries(
        const viam::sdk::ProtoStruct& extra) override;

   private:
    void cancel_movement();

    unitree::robot::g1::LocoClient loco_client_;
    std::atomic<bool> moving_{false};
    std::mutex mu_;
    std::condition_variable cancel_cv_;
    bool cancelled_{false};
};

}  // namespace viam_unitree

#pragma once

#include <viam/sdk/common/proto_value.hpp>
#include <viam/sdk/components/camera.hpp>
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/module/service.hpp>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/video/video_client.hpp>

namespace viam_unitree {

class G1Camera : public viam::sdk::Camera {
   public:
    G1Camera(const viam::sdk::Dependencies& deps, const viam::sdk::ResourceConfig& cfg);

    static std::vector<std::string> validate(const viam::sdk::ResourceConfig& cfg);

    viam::sdk::Camera::image_collection get_images(
        std::vector<std::string> filter_source_names,
        const viam::sdk::ProtoStruct& extra) override;

    viam::sdk::Camera::point_cloud get_point_cloud(
        std::string mime_type,
        const viam::sdk::ProtoStruct& extra) override;

    viam::sdk::Camera::properties get_properties() override;

    viam::sdk::ProtoStruct get_status() override;

    viam::sdk::ProtoStruct do_command(const viam::sdk::ProtoStruct& command) override;

    std::vector<viam::sdk::GeometryConfig> get_geometries(
        const viam::sdk::ProtoStruct& extra) override;

   private:
    unitree::robot::go2::VideoClient video_client_;
};

}  // namespace viam_unitree

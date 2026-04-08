#include "g1_camera.hpp"

#include <chrono>
#include <stdexcept>

#include <viam/sdk/log/logging.hpp>

namespace viam_unitree {

G1Camera::G1Camera(const viam::sdk::Dependencies& deps, const viam::sdk::ResourceConfig& cfg)
    : Camera(cfg.name()) {
    auto attrs = cfg.attributes();
    std::string network_interface = "eth0";

    auto it = attrs.find("network_interface");
    if (it != attrs.end()) {
        if (auto* val = it->second.get<std::string>()) {
            network_interface = *val;
        }
    }

    VIAM_SDK_LOG(info) << "Initializing G1Camera with network interface: " << network_interface;

    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
    video_client_.Init();

    VIAM_SDK_LOG(info) << "G1Camera initialized successfully";
}

std::vector<std::string> G1Camera::validate(const viam::sdk::ResourceConfig& cfg) {
    return {};
}

viam::sdk::Camera::image_collection G1Camera::get_images(
    std::vector<std::string> filter_source_names,
    const viam::sdk::ProtoStruct& extra) {
    std::vector<uint8_t> image_data;
    int32_t rc = video_client_.GetImageSample(image_data);
    if (rc != 0) {
        throw std::runtime_error("GetImageSample failed with rc=" + std::to_string(rc));
    }

    viam::sdk::Camera::raw_image img;
    img.mime_type = "image/jpeg";
    img.bytes = std::vector<unsigned char>(image_data.begin(), image_data.end());
    img.source_name = name();

    viam::sdk::Camera::image_collection collection;
    collection.images = {std::move(img)};
    collection.metadata.captured_at =
        std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());

    return collection;
}

viam::sdk::Camera::point_cloud G1Camera::get_point_cloud(std::string mime_type,
                                                         const viam::sdk::ProtoStruct& extra) {
    throw std::runtime_error("point cloud not supported");
}

viam::sdk::Camera::properties G1Camera::get_properties() {
    viam::sdk::Camera::properties props;
    props.supports_pcd = false;
    props.intrinsic_parameters = {};
    props.distortion_parameters = {};
    props.mime_types = {"image/jpeg"};
    props.frame_rate = 30.0f;
    return props;
}

viam::sdk::ProtoStruct G1Camera::get_status() {
    return {};
}

viam::sdk::ProtoStruct G1Camera::do_command(const viam::sdk::ProtoStruct& command) {
    return {};
}

std::vector<viam::sdk::GeometryConfig> G1Camera::get_geometries(
    const viam::sdk::ProtoStruct& extra) {
    return {};
}

}  // namespace viam_unitree

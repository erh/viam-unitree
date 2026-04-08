#include "g1_base.hpp"
#include "g1_camera.hpp"

#include <iostream>
#include <memory>
#include <vector>

#include <viam/sdk/common/exception.hpp>
#include <viam/sdk/common/instance.hpp>
#include <viam/sdk/log/logging.hpp>
#include <viam/sdk/registry/registry.hpp>

int main(int argc, char** argv) try {
    viam::sdk::Instance inst;

    VIAM_SDK_LOG(info) << "Starting up viam-unitree module";

    auto base_mr = std::make_shared<viam::sdk::ModelRegistration>(
        viam::sdk::API::get<viam::sdk::Base>(),
        viam::sdk::Model("erh", "viam-unitree", "g1-base"),
        [](viam::sdk::Dependencies deps, viam::sdk::ResourceConfig cfg) {
            return std::make_unique<viam_unitree::G1Base>(deps, cfg);
        },
        &viam_unitree::G1Base::validate);

    auto camera_mr = std::make_shared<viam::sdk::ModelRegistration>(
        viam::sdk::API::get<viam::sdk::Camera>(),
        viam::sdk::Model("erh", "viam-unitree", "g1-camera"),
        [](viam::sdk::Dependencies deps, viam::sdk::ResourceConfig cfg) {
            return std::make_unique<viam_unitree::G1Camera>(deps, cfg);
        },
        &viam_unitree::G1Camera::validate);

    std::vector<std::shared_ptr<viam::sdk::ModelRegistration>> mrs = {base_mr, camera_mr};
    auto my_mod = std::make_shared<viam::sdk::ModuleService>(argc, argv, mrs);
    my_mod->serve();

    return EXIT_SUCCESS;
} catch (const viam::sdk::Exception& ex) {
    std::cerr << "main failed with exception: " << ex.what() << "\n";
    return EXIT_FAILURE;
}

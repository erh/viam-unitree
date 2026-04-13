#include "unitree_capi.h"

#include <cstring>
#include <vector>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>
#include <unitree/robot/go2/video/video_client.hpp>

extern "C" {

int unitree_channel_init(int domain_id, const char* network_interface) {
    try {
        unitree::robot::ChannelFactory::Instance()->Init(
            domain_id, std::string(network_interface));
        return 0;
    } catch (...) {
        return -1;
    }
}

// --- Loco Client ---

struct loco_wrapper {
    unitree::robot::g1::LocoClient client;
};

unitree_loco_client_t unitree_loco_new(void) {
    try {
        return new loco_wrapper();
    } catch (...) {
        return nullptr;
    }
}

void unitree_loco_free(unitree_loco_client_t c) {
    delete static_cast<loco_wrapper*>(c);
}

int unitree_loco_init(unitree_loco_client_t c) {
    try {
        static_cast<loco_wrapper*>(c)->client.Init();
        return 0;
    } catch (...) {
        return -1;
    }
}

void unitree_loco_set_timeout(unitree_loco_client_t c, float timeout_sec) {
    static_cast<loco_wrapper*>(c)->client.SetTimeout(timeout_sec);
}

int unitree_loco_move(unitree_loco_client_t c, float vx, float vy, float vyaw) {
    try {
        return static_cast<loco_wrapper*>(c)->client.Move(vx, vy, vyaw);
    } catch (...) {
        return -1;
    }
}

int unitree_loco_stop_move(unitree_loco_client_t c) {
    try {
        return static_cast<loco_wrapper*>(c)->client.StopMove();
    } catch (...) {
        return -1;
    }
}

int unitree_loco_stand_up(unitree_loco_client_t c) {
    try {
        return static_cast<loco_wrapper*>(c)->client.StandUp();
    } catch (...) {
        return -1;
    }
}

int unitree_loco_sit(unitree_loco_client_t c) {
    try {
        return static_cast<loco_wrapper*>(c)->client.Sit();
    } catch (...) {
        return -1;
    }
}

int unitree_loco_squat(unitree_loco_client_t c) {
    try {
        return static_cast<loco_wrapper*>(c)->client.Squat();
    } catch (...) {
        return -1;
    }
}

int unitree_loco_high_stand(unitree_loco_client_t c) {
    try {
        return static_cast<loco_wrapper*>(c)->client.HighStand();
    } catch (...) {
        return -1;
    }
}

int unitree_loco_low_stand(unitree_loco_client_t c) {
    try {
        return static_cast<loco_wrapper*>(c)->client.LowStand();
    } catch (...) {
        return -1;
    }
}

int unitree_loco_balance_stand(unitree_loco_client_t c) {
    try {
        return static_cast<loco_wrapper*>(c)->client.BalanceStand();
    } catch (...) {
        return -1;
    }
}

int unitree_loco_damp(unitree_loco_client_t c) {
    try {
        return static_cast<loco_wrapper*>(c)->client.Damp();
    } catch (...) {
        return -1;
    }
}

int unitree_loco_zero_torque(unitree_loco_client_t c) {
    try {
        return static_cast<loco_wrapper*>(c)->client.ZeroTorque();
    } catch (...) {
        return -1;
    }
}

int unitree_loco_wave_hand(unitree_loco_client_t c) {
    try {
        return static_cast<loco_wrapper*>(c)->client.WaveHand();
    } catch (...) {
        return -1;
    }
}

int unitree_loco_start(unitree_loco_client_t c) {
    try {
        return static_cast<loco_wrapper*>(c)->client.Start();
    } catch (...) {
        return -1;
    }
}

// --- Video Client ---

struct video_wrapper {
    unitree::robot::go2::VideoClient client;
};

unitree_video_client_t unitree_video_new(void) {
    try {
        return new video_wrapper();
    } catch (...) {
        return nullptr;
    }
}

void unitree_video_free(unitree_video_client_t c) {
    delete static_cast<video_wrapper*>(c);
}

int unitree_video_init(unitree_video_client_t c) {
    try {
        static_cast<video_wrapper*>(c)->client.Init();
        return 0;
    } catch (...) {
        return -1;
    }
}

int unitree_video_get_image(unitree_video_client_t c, uint8_t** data, int* size) {
    try {
        std::vector<uint8_t> img;
        int32_t rc = static_cast<video_wrapper*>(c)->client.GetImageSample(img);
        if (rc != 0) {
            return rc;
        }
        *size = static_cast<int>(img.size());
        *data = static_cast<uint8_t*>(malloc(img.size()));
        if (*data == nullptr) {
            return -1;
        }
        memcpy(*data, img.data(), img.size());
        return 0;
    } catch (...) {
        return -1;
    }
}

void unitree_image_free(uint8_t* data) {
    free(data);
}

}  // extern "C"

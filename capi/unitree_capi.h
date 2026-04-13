#ifndef UNITREE_CAPI_H
#define UNITREE_CAPI_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize the DDS channel factory. Must be called before creating clients.
// Returns 0 on success, -1 on error.
int unitree_channel_init(int domain_id, const char* network_interface);

// --- Loco Client ---

typedef void* unitree_loco_client_t;

unitree_loco_client_t unitree_loco_new(void);
void unitree_loco_free(unitree_loco_client_t c);
int unitree_loco_init(unitree_loco_client_t c);
void unitree_loco_set_timeout(unitree_loco_client_t c, float timeout_sec);

int unitree_loco_move(unitree_loco_client_t c, float vx, float vy, float vyaw);
int unitree_loco_stop_move(unitree_loco_client_t c);
int unitree_loco_stand_up(unitree_loco_client_t c);
int unitree_loco_sit(unitree_loco_client_t c);
int unitree_loco_squat(unitree_loco_client_t c);
int unitree_loco_high_stand(unitree_loco_client_t c);
int unitree_loco_low_stand(unitree_loco_client_t c);
int unitree_loco_balance_stand(unitree_loco_client_t c);
int unitree_loco_damp(unitree_loco_client_t c);
int unitree_loco_zero_torque(unitree_loco_client_t c);
int unitree_loco_wave_hand(unitree_loco_client_t c);
int unitree_loco_start(unitree_loco_client_t c);

// --- Video Client ---

typedef void* unitree_video_client_t;

unitree_video_client_t unitree_video_new(void);
void unitree_video_free(unitree_video_client_t c);
int unitree_video_init(unitree_video_client_t c);

// Capture a JPEG frame. Caller must free *data with unitree_image_free().
// Returns 0 on success.
int unitree_video_get_image(unitree_video_client_t c, uint8_t** data, int* size);
void unitree_image_free(uint8_t* data);

#ifdef __cplusplus
}
#endif

#endif

#ifndef LIVOX_MID360_H
#define LIVOX_MID360_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Start Livox-SDK2 using a config JSON file path. Returns 0 on success. */
int livox_mid360_start(const char *config_path);

/* Stop Livox-SDK2 and free frame buffers. Safe to call multiple times. */
void livox_mid360_stop(void);

/*
 * Wait up to timeout_ms for a assembled point-cloud frame.
 * On success writes up to max_points into xyz (meters, length 3*n) and
 * optional intensity (length n; may be NULL), sets *n_out, returns 0.
 * Returns non-zero on timeout / not started.
 *
 * If invert_mount is non-zero, applies G1 head mount correction (x, -y, -z).
 */
int livox_mid360_take_cloud(float *xyz, uint8_t *intensity, int max_points,
                            int *n_out, int timeout_ms, int invert_mount);

/* Frame assembly window in milliseconds (default 100 ≈ 10Hz). */
void livox_mid360_set_frame_ms(int frame_ms);

#ifdef __cplusplus
}
#endif

#endif /* LIVOX_MID360_H */

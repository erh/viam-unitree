#include "livox_mid360.h"

#include "livox_lidar_api.h"
#include "livox_lidar_def.h"

#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <vector>

namespace {

constexpr int kDefaultFrameMs = 100;
constexpr int kMaxPoints = 200000;

struct CloudBuf {
  std::vector<float> xyz; // x,y,z interleaved, meters
  std::vector<uint8_t> intensity;
  void clear() {
    xyz.clear();
    intensity.clear();
  }
  int n() const { return static_cast<int>(intensity.size()); }
};

pthread_mutex_t g_mu = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t g_cv = PTHREAD_COND_INITIALIZER;
bool g_started = false;
int g_frame_ms = kDefaultFrameMs;
CloudBuf g_accum;
CloudBuf g_ready;
bool g_have_ready = false;
int64_t g_frame_start_ms = 0;

int64_t now_ms() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1000 + ts.tv_nsec / 1000000;
}

void append_high(const LivoxLidarCartesianHighRawPoint *pts, uint32_t n) {
  for (uint32_t i = 0; i < n; i++) {
    if (g_accum.n() >= kMaxPoints) {
      break;
    }
    // High cartesian is millimeters.
    g_accum.xyz.push_back(pts[i].x * 0.001f);
    g_accum.xyz.push_back(pts[i].y * 0.001f);
    g_accum.xyz.push_back(pts[i].z * 0.001f);
    g_accum.intensity.push_back(pts[i].reflectivity);
  }
}

void append_low(const LivoxLidarCartesianLowRawPoint *pts, uint32_t n) {
  for (uint32_t i = 0; i < n; i++) {
    if (g_accum.n() >= kMaxPoints) {
      break;
    }
    // Low cartesian is centimeters.
    g_accum.xyz.push_back(pts[i].x * 0.01f);
    g_accum.xyz.push_back(pts[i].y * 0.01f);
    g_accum.xyz.push_back(pts[i].z * 0.01f);
    g_accum.intensity.push_back(pts[i].reflectivity);
  }
}

void maybe_rotate_frame(int64_t t) {
  if (g_frame_start_ms == 0) {
    g_frame_start_ms = t;
    return;
  }
  if (t - g_frame_start_ms < g_frame_ms) {
    return;
  }
  if (g_accum.n() > 0) {
    g_ready.xyz.swap(g_accum.xyz);
    g_ready.intensity.swap(g_accum.intensity);
    g_accum.clear();
    g_have_ready = true;
    pthread_cond_broadcast(&g_cv);
  }
  g_frame_start_ms = t;
}

void PointCloudCallback(uint32_t /*handle*/, const uint8_t /*dev_type*/,
                        LivoxLidarEthernetPacket *data, void * /*client_data*/) {
  if (data == nullptr || data->dot_num == 0) {
    return;
  }

  pthread_mutex_lock(&g_mu);
  if (!g_started) {
    pthread_mutex_unlock(&g_mu);
    return;
  }

  const int64_t t = now_ms();
  maybe_rotate_frame(t);

  if (data->data_type == kLivoxLidarCartesianCoordinateHighData) {
    append_high(reinterpret_cast<LivoxLidarCartesianHighRawPoint *>(data->data),
                data->dot_num);
  } else if (data->data_type == kLivoxLidarCartesianCoordinateLowData) {
    append_low(reinterpret_cast<LivoxLidarCartesianLowRawPoint *>(data->data),
               data->dot_num);
  }
  // Spherical / dual-echo ignored for now.

  pthread_mutex_unlock(&g_mu);
}

void WorkModeCallback(livox_status /*status*/, uint32_t /*handle*/,
                      LivoxLidarAsyncControlResponse * /*response*/,
                      void * /*client_data*/) {}

void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo *info,
                             void * /*client_data*/) {
  if (info == nullptr) {
    return;
  }
  // Bring the unit into Normal streaming mode (same as Livox quick start).
  SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeCallback, nullptr);
}

} // namespace

extern "C" void livox_mid360_set_frame_ms(int frame_ms) {
  if (frame_ms < 20) {
    frame_ms = 20;
  }
  if (frame_ms > 1000) {
    frame_ms = 1000;
  }
  pthread_mutex_lock(&g_mu);
  g_frame_ms = frame_ms;
  pthread_mutex_unlock(&g_mu);
}

extern "C" int livox_mid360_start(const char *config_path) {
  if (config_path == nullptr) {
    return -1;
  }

  pthread_mutex_lock(&g_mu);
  if (g_started) {
    pthread_mutex_unlock(&g_mu);
    return 0;
  }
  pthread_mutex_unlock(&g_mu);

  DisableLivoxSdkConsoleLogger();

  if (!LivoxLidarSdkInit(config_path, "", nullptr)) {
    LivoxLidarSdkUninit();
    return -2;
  }

  SetLivoxLidarPointCloudCallBack(PointCloudCallback, nullptr);
  SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);

  if (!LivoxLidarSdkStart()) {
    LivoxLidarSdkUninit();
    return -3;
  }

  pthread_mutex_lock(&g_mu);
  g_accum.clear();
  g_ready.clear();
  g_have_ready = false;
  g_frame_start_ms = 0;
  g_started = true;
  pthread_mutex_unlock(&g_mu);
  return 0;
}

extern "C" void livox_mid360_stop(void) {
  pthread_mutex_lock(&g_mu);
  if (!g_started) {
    pthread_mutex_unlock(&g_mu);
    return;
  }
  g_started = false;
  g_have_ready = false;
  g_accum.clear();
  g_ready.clear();
  pthread_cond_broadcast(&g_cv);
  pthread_mutex_unlock(&g_mu);

  LivoxLidarSdkUninit();
}

extern "C" int livox_mid360_take_cloud(float *xyz, uint8_t *intensity,
                                       int max_points, int *n_out,
                                       int timeout_ms, int invert_mount) {
  if (xyz == nullptr || n_out == nullptr || max_points <= 0) {
    return -1;
  }
  *n_out = 0;

  pthread_mutex_lock(&g_mu);
  if (!g_started) {
    pthread_mutex_unlock(&g_mu);
    return -2;
  }

  if (!g_have_ready) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += timeout_ms / 1000;
    ts.tv_nsec += static_cast<long>(timeout_ms % 1000) * 1000000L;
    if (ts.tv_nsec >= 1000000000L) {
      ts.tv_sec += 1;
      ts.tv_nsec -= 1000000000L;
    }
    while (!g_have_ready && g_started) {
      int rc = pthread_cond_timedwait(&g_cv, &g_mu, &ts);
      if (rc != 0) {
        break;
      }
    }
  }

  if (!g_have_ready) {
    pthread_mutex_unlock(&g_mu);
    return -3;
  }

  const int n = g_ready.n() < max_points ? g_ready.n() : max_points;
  for (int i = 0; i < n; i++) {
    float x = g_ready.xyz[static_cast<size_t>(i) * 3 + 0];
    float y = g_ready.xyz[static_cast<size_t>(i) * 3 + 1];
    float z = g_ready.xyz[static_cast<size_t>(i) * 3 + 2];
    if (invert_mount) {
      // G1 head Mid-360 is mounted upside down.
      y = -y;
      z = -z;
    }
    xyz[i * 3 + 0] = x;
    xyz[i * 3 + 1] = y;
    xyz[i * 3 + 2] = z;
    if (intensity != nullptr) {
      intensity[i] = g_ready.intensity[static_cast<size_t>(i)];
    }
  }
  *n_out = n;
  g_have_ready = false;
  g_ready.clear();
  pthread_mutex_unlock(&g_mu);
  return 0;
}

#ifndef DDS_UNITREE_H
#define DDS_UNITREE_H

#include <dds/dds.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Flat C structs matching the CDR wire layout of the Unitree SDK2 RPC types.
 * These are compatible with unitree_api::msg::dds_::Request_ and Response_.
 *
 * Request_ layout (CDR order):
 *   RequestHeader_.RequestIdentity_.id      (int64)
 *   RequestHeader_.RequestIdentity_.api_id  (int64)
 *   RequestHeader_.RequestLease_.id         (int64)
 *   RequestHeader_.RequestPolicy_.priority  (int32)
 *   RequestHeader_.RequestPolicy_.noreply   (bool)
 *   parameter                               (string)
 *   binary                                  (sequence<uint8>)
 *
 * Response_ layout (CDR order):
 *   ResponseHeader_.RequestIdentity_.id     (int64)
 *   ResponseHeader_.RequestIdentity_.api_id (int64)
 *   ResponseHeader_.ResponseStatus_.code    (int32)
 *   data                                    (string)
 *   binary                                  (sequence<uint8>)
 */

typedef struct {
    uint32_t _maximum;
    uint32_t _length;
    uint8_t *_buffer;
} unitree_seq_uint8_t;

typedef struct {
    int64_t  identity_id;
    int64_t  identity_api_id;
    int64_t  lease_id;
    int32_t  policy_priority;
    uint8_t  policy_noreply;
    char    *parameter;
    unitree_seq_uint8_t binary;
} unitree_request_t;

typedef struct {
    int64_t  identity_id;
    int64_t  identity_api_id;
    int32_t  status_code;
    char    *data;
    unitree_seq_uint8_t binary;
} unitree_response_t;

/* ROS2 sensor_msgs::msg::dds_::PointCloud2_ - flattened to match CDR layout. */
typedef struct {
    char    *name;
    uint32_t offset;
    uint8_t  datatype;
    uint32_t count;
} unitree_point_field_t;

typedef struct {
    uint32_t _maximum;
    uint32_t _length;
    unitree_point_field_t *_buffer;
} unitree_seq_point_field_t;

typedef struct {
    /* Header.stamp */
    int32_t  stamp_sec;
    uint32_t stamp_nanosec;
    /* Header.frame_id */
    char    *frame_id;
    uint32_t height;
    uint32_t width;
    unitree_seq_point_field_t fields;
    uint8_t  is_bigendian;
    uint32_t point_step;
    uint32_t row_step;
    unitree_seq_uint8_t data;
    uint8_t  is_dense;
} unitree_pointcloud2_t;

extern const dds_topic_descriptor_t unitree_request_desc;
extern const dds_topic_descriptor_t unitree_response_desc;
extern const dds_topic_descriptor_t unitree_pointcloud2_desc;

/* Initialize the DDS participant. Returns 0 on success. */
int unitree_dds_init(int domain_id, const char *network_interface);

/* Create a writer+reader pair for an RPC service (e.g. "sport", "videohub").
   Returns 0 on success. writer_out/reader_out receive DDS entity handles. */
int unitree_dds_create_rpc(const char *service_name,
                           dds_entity_t *writer_out,
                           dds_entity_t *reader_out);

/* Publish a Request_ message on the given writer. Returns 0 on success. */
int unitree_dds_write_request(dds_entity_t writer,
                              int64_t req_id, int64_t api_id,
                              const char *params_json);

/* Read a Response_ from the given reader (blocking up to timeout_ms).
   On success, sets *out to the response. Caller must call unitree_response_free(). */
int unitree_dds_read_response(dds_entity_t reader, int timeout_ms,
                              unitree_response_t *out);

/* Free memory allocated inside a response by DDS deserialization. */
void unitree_response_free(unitree_response_t *resp);

/* Create a subscriber for a streaming (non-RPC) topic.
   topic_type indicates which topic descriptor to use:
     0 = PointCloud2
   Returns 0 on success and writes the reader handle to *reader_out. */
int unitree_dds_subscribe(const char *topic_name, int topic_type,
                          dds_entity_t *reader_out);

/* Take the latest PointCloud2 from a reader. Returns 0 on success.
   On success, *out is populated and caller must call unitree_pointcloud2_free(). */
int unitree_dds_take_pointcloud2(dds_entity_t reader, int timeout_ms,
                                 unitree_pointcloud2_t *out);

/* Free memory allocated inside a PointCloud2 by DDS deserialization. */
void unitree_pointcloud2_free(unitree_pointcloud2_t *pc);

/* Delete a subscriber created by unitree_dds_subscribe. */
void unitree_dds_close_subscriber(dds_entity_t reader);

/* Delete a writer/reader pair created by unitree_dds_create_rpc. */
void unitree_dds_close_rpc(dds_entity_t writer, dds_entity_t reader);

/* Shut down the DDS participant. */
void unitree_dds_shutdown(void);

#ifdef __cplusplus
}
#endif

#endif

#include "dds_unitree.h"
#include <dds/ddsc/dds_opcodes.h>
#include <stdlib.h>
#include <string.h>

/*
 * CycloneDDS topic descriptors for the Unitree RPC message types.
 * These match the CDR encoding of unitree_api::msg::dds_::Request_ and Response_.
 *
 * The ops arrays describe how to serialize/deserialize the flat C structs to/from CDR.
 * Fields are listed in CDR wire order (matching the original nested struct field order).
 */

/* --- Request_ descriptor --- */

static const uint32_t unitree_request_ops[] = {
    /* identity.id (int64, signed) */
    DDS_OP_ADR | DDS_OP_TYPE_8BY | DDS_OP_FLAG_SGN,
    offsetof(unitree_request_t, identity_id),
    /* identity.api_id (int64, signed) */
    DDS_OP_ADR | DDS_OP_TYPE_8BY | DDS_OP_FLAG_SGN,
    offsetof(unitree_request_t, identity_api_id),
    /* lease.id (int64, signed) */
    DDS_OP_ADR | DDS_OP_TYPE_8BY | DDS_OP_FLAG_SGN,
    offsetof(unitree_request_t, lease_id),
    /* policy.priority (int32, signed) */
    DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN,
    offsetof(unitree_request_t, policy_priority),
    /* policy.noreply (bool) */
    DDS_OP_ADR | DDS_OP_TYPE_BLN,
    offsetof(unitree_request_t, policy_noreply),
    /* parameter (string) */
    DDS_OP_ADR | DDS_OP_TYPE_STR,
    offsetof(unitree_request_t, parameter),
    /* binary (sequence<uint8>) */
    DDS_OP_ADR | DDS_OP_TYPE_SEQ | DDS_OP_SUBTYPE_1BY,
    offsetof(unitree_request_t, binary),
    DDS_OP_RTS
};

const dds_topic_descriptor_t unitree_request_desc = {
    .m_size = sizeof(unitree_request_t),
    .m_align = sizeof(int64_t),
    .m_flagset = DDS_TOPIC_NO_OPTIMIZE,
    .m_nkeys = 0u,
    .m_typename = "unitree_api::msg::dds_::Request_",
    .m_keys = NULL,
    .m_nops = 8u, /* 7 field ops + RTS */
    .m_ops = unitree_request_ops,
    .m_meta = ""
};

/* --- Response_ descriptor --- */

static const uint32_t unitree_response_ops[] = {
    /* identity.id (int64, signed) */
    DDS_OP_ADR | DDS_OP_TYPE_8BY | DDS_OP_FLAG_SGN,
    offsetof(unitree_response_t, identity_id),
    /* identity.api_id (int64, signed) */
    DDS_OP_ADR | DDS_OP_TYPE_8BY | DDS_OP_FLAG_SGN,
    offsetof(unitree_response_t, identity_api_id),
    /* status.code (int32, signed) */
    DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN,
    offsetof(unitree_response_t, status_code),
    /* data (string) */
    DDS_OP_ADR | DDS_OP_TYPE_STR,
    offsetof(unitree_response_t, data),
    /* binary (sequence<uint8>) */
    DDS_OP_ADR | DDS_OP_TYPE_SEQ | DDS_OP_SUBTYPE_1BY,
    offsetof(unitree_response_t, binary),
    DDS_OP_RTS
};

const dds_topic_descriptor_t unitree_response_desc = {
    .m_size = sizeof(unitree_response_t),
    .m_align = sizeof(int64_t),
    .m_flagset = DDS_TOPIC_NO_OPTIMIZE,
    .m_nkeys = 0u,
    .m_typename = "unitree_api::msg::dds_::Response_",
    .m_keys = NULL,
    .m_nops = 6u, /* 5 field ops + RTS */
    .m_ops = unitree_response_ops,
    .m_meta = ""
};

/* --- PointCloud2_ descriptor ---
 *
 * sensor_msgs::msg::dds_::PointCloud2_ contains a sequence of PointField
 * structs. The CycloneDDS opcode for "sequence of struct" is:
 *   [ADR|SEQ|STU, offset, sizeof(elem), (next_insn<<16) | elem_insn]
 * where next_insn / elem_insn are offsets in uint32 units measured from
 * the start of the SEQ opcode.
 *
 * The PointField element ops are inlined within the same array, between
 * the SEQ opcode and the next field's opcode.
 */
static const uint32_t unitree_pointcloud2_ops[] = {
    /* 0  */ DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof(unitree_pointcloud2_t, stamp_sec),
    /* 2  */ DDS_OP_ADR | DDS_OP_TYPE_4BY,                   offsetof(unitree_pointcloud2_t, stamp_nanosec),
    /* 4  */ DDS_OP_ADR | DDS_OP_TYPE_STR,                   offsetof(unitree_pointcloud2_t, frame_id),
    /* 6  */ DDS_OP_ADR | DDS_OP_TYPE_4BY,                   offsetof(unitree_pointcloud2_t, height),
    /* 8  */ DDS_OP_ADR | DDS_OP_TYPE_4BY,                   offsetof(unitree_pointcloud2_t, width),
    /* 10 */ DDS_OP_ADR | DDS_OP_TYPE_SEQ | DDS_OP_SUBTYPE_STU, offsetof(unitree_pointcloud2_t, fields),
    /* 12 */ sizeof(unitree_point_field_t),
    /* 13 */ (14u << 16) | 5u,  /* next_insn=14 (skip to op 24), elem_insn=5 (jump to op 15) */
    /* 14 */ DDS_OP_RTS,         /* return-to-sender for sequence (terminates element ops) */
    /* 15 */ DDS_OP_ADR | DDS_OP_TYPE_STR,                   offsetof(unitree_point_field_t, name),
    /* 17 */ DDS_OP_ADR | DDS_OP_TYPE_4BY,                   offsetof(unitree_point_field_t, offset),
    /* 19 */ DDS_OP_ADR | DDS_OP_TYPE_1BY,                   offsetof(unitree_point_field_t, datatype),
    /* 21 */ DDS_OP_ADR | DDS_OP_TYPE_4BY,                   offsetof(unitree_point_field_t, count),
    /* 23 */ DDS_OP_RTS,         /* end of element ops */
    /* 24 */ DDS_OP_ADR | DDS_OP_TYPE_BLN,                   offsetof(unitree_pointcloud2_t, is_bigendian),
    /* 26 */ DDS_OP_ADR | DDS_OP_TYPE_4BY,                   offsetof(unitree_pointcloud2_t, point_step),
    /* 28 */ DDS_OP_ADR | DDS_OP_TYPE_4BY,                   offsetof(unitree_pointcloud2_t, row_step),
    /* 30 */ DDS_OP_ADR | DDS_OP_TYPE_SEQ | DDS_OP_SUBTYPE_1BY, offsetof(unitree_pointcloud2_t, data),
    /* 32 */ DDS_OP_ADR | DDS_OP_TYPE_BLN,                   offsetof(unitree_pointcloud2_t, is_dense),
    /* 34 */ DDS_OP_RTS
};

const dds_topic_descriptor_t unitree_pointcloud2_desc = {
    .m_size = sizeof(unitree_pointcloud2_t),
    .m_align = sizeof(uint32_t),
    .m_flagset = DDS_TOPIC_NO_OPTIMIZE,
    .m_nkeys = 0u,
    .m_typename = "sensor_msgs::msg::dds_::PointCloud2_",
    .m_keys = NULL,
    .m_nops = sizeof(unitree_pointcloud2_ops) / sizeof(uint32_t),
    .m_ops = unitree_pointcloud2_ops,
    .m_meta = ""
};

/* --- unitree_hg::msg::dds_::LowCmd_ descriptor ---
 *
 * CDR layout (in field order):
 *   uint8 mode_pr
 *   uint8 mode_machine
 *   MotorCmd_ motor_cmd[35]    -- fixed-length array of struct
 *   uint32 reserve[4]          -- fixed-length array of uint32
 *   uint32 crc
 *
 * MotorCmd_ inner layout:
 *   uint8 mode
 *   float q, dq, tau, kp, kd
 *   uint32 reserve
 *
 * CycloneDDS ARR|SUBTYPE_STU slot layout (verified against
 * cyclonedds-0.10.2 src/core/ddsi/src/ddsi_cdrstream.c and the test
 * descriptor in src/core/ddsc/tests/cdrstream.c:940):
 *   [ADR|ARR|SUBTYPE_STU, offset, count, jmp, elem_size]
 * where jmp = (next_insn << 16) | elem_insn, both measured in
 * uint32-units from the ARR opcode's own position. The element ops
 * for the sub-struct live at elem_insn and are terminated with
 * DDS_OP_RTS; processing resumes at next_insn after the array is done.
 */
static const uint32_t unitree_hg_lowcmd_ops[] = {
    /* --- outer unitree_hg_lowcmd_t --- */
    /*  0 */ DDS_OP_ADR | DDS_OP_TYPE_1BY, offsetof(unitree_hg_lowcmd_t, mode_pr),
    /*  2 */ DDS_OP_ADR | DDS_OP_TYPE_1BY, offsetof(unitree_hg_lowcmd_t, mode_machine),
    /*  4 */ DDS_OP_ADR | DDS_OP_TYPE_ARR | DDS_OP_SUBTYPE_STU,
                offsetof(unitree_hg_lowcmd_t, motor_cmd),
                UNITREE_HG_NUM_MOTOR,
                (5u << 16) | 11u,  /* next_insn=5 → pos 9; elem_insn=11 → pos 15 */
                sizeof(unitree_hg_motor_cmd_t),
    /*  9 */ DDS_OP_ADR | DDS_OP_TYPE_ARR | DDS_OP_SUBTYPE_4BY,
                offsetof(unitree_hg_lowcmd_t, reserve), 4u,
    /* 12 */ DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof(unitree_hg_lowcmd_t, crc),
    /* 14 */ DDS_OP_RTS,
    /* --- element ops for unitree_hg_motor_cmd_t --- */
    /* 15 */ DDS_OP_ADR | DDS_OP_TYPE_1BY, offsetof(unitree_hg_motor_cmd_t, mode),
    /* 17 */ DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof(unitree_hg_motor_cmd_t, q),
    /* 19 */ DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof(unitree_hg_motor_cmd_t, dq),
    /* 21 */ DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof(unitree_hg_motor_cmd_t, tau),
    /* 23 */ DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof(unitree_hg_motor_cmd_t, kp),
    /* 25 */ DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof(unitree_hg_motor_cmd_t, kd),
    /* 27 */ DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof(unitree_hg_motor_cmd_t, reserve),
    /* 29 */ DDS_OP_RTS
};

const dds_topic_descriptor_t unitree_hg_lowcmd_desc = {
    .m_size = sizeof(unitree_hg_lowcmd_t),
    .m_align = sizeof(uint32_t),
    .m_flagset = DDS_TOPIC_NO_OPTIMIZE,
    .m_nkeys = 0u,
    .m_typename = "unitree_hg::msg::dds_::LowCmd_",
    .m_keys = NULL,
    .m_nops = sizeof(unitree_hg_lowcmd_ops) / sizeof(uint32_t),
    .m_ops = unitree_hg_lowcmd_ops,
    .m_meta = ""
};

/* --- unitree_hg::msg::dds_::LowState_ descriptor ---
 *
 * Same ARR|STU layout convention as LowCmd above. Element ops for the
 * motor_state sub-struct live after the outer RTS.
 */
static const uint32_t unitree_hg_lowstate_ops[] = {
    /* --- outer unitree_hg_lowstate_t --- */
    /*  0 */ DDS_OP_ADR | DDS_OP_TYPE_ARR | DDS_OP_SUBTYPE_4BY,
                offsetof(unitree_hg_lowstate_t, version), 2u,
    /*  3 */ DDS_OP_ADR | DDS_OP_TYPE_1BY, offsetof(unitree_hg_lowstate_t, mode_pr),
    /*  5 */ DDS_OP_ADR | DDS_OP_TYPE_1BY, offsetof(unitree_hg_lowstate_t, mode_machine),
    /*  7 */ DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof(unitree_hg_lowstate_t, tick),
    /*  9 */ DDS_OP_ADR | DDS_OP_TYPE_ARR | DDS_OP_SUBTYPE_4BY,
                offsetof(unitree_hg_lowstate_t, imu_state.quaternion), 4u,
    /* 12 */ DDS_OP_ADR | DDS_OP_TYPE_ARR | DDS_OP_SUBTYPE_4BY,
                offsetof(unitree_hg_lowstate_t, imu_state.gyroscope), 3u,
    /* 15 */ DDS_OP_ADR | DDS_OP_TYPE_ARR | DDS_OP_SUBTYPE_4BY,
                offsetof(unitree_hg_lowstate_t, imu_state.accelerometer), 3u,
    /* 18 */ DDS_OP_ADR | DDS_OP_TYPE_ARR | DDS_OP_SUBTYPE_4BY,
                offsetof(unitree_hg_lowstate_t, imu_state.rpy), 3u,
    /* 21 */ DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN,
                offsetof(unitree_hg_lowstate_t, imu_state.temperature),
    /* 23 */ DDS_OP_ADR | DDS_OP_TYPE_ARR | DDS_OP_SUBTYPE_STU,
                offsetof(unitree_hg_lowstate_t, motor_state),
                UNITREE_HG_NUM_MOTOR,
                (5u << 16) | 34u,  /* next_insn=5 → pos 28; elem_insn=34 → pos 57 */
                sizeof(unitree_hg_motor_state_t),
    /* 28 */ DDS_OP_ADR | DDS_OP_TYPE_1BY, offsetof(unitree_hg_lowstate_t, bms_version_high),
    /* 30 */ DDS_OP_ADR | DDS_OP_TYPE_1BY, offsetof(unitree_hg_lowstate_t, bms_version_low),
    /* 32 */ DDS_OP_ADR | DDS_OP_TYPE_1BY, offsetof(unitree_hg_lowstate_t, bms_status),
    /* 34 */ DDS_OP_ADR | DDS_OP_TYPE_1BY, offsetof(unitree_hg_lowstate_t, bms_soc),
    /* 36 */ DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN,
                offsetof(unitree_hg_lowstate_t, bms_current),
    /* 38 */ DDS_OP_ADR | DDS_OP_TYPE_2BY, offsetof(unitree_hg_lowstate_t, bms_cycle),
    /* 40 */ DDS_OP_ADR | DDS_OP_TYPE_ARR | DDS_OP_SUBTYPE_1BY | DDS_OP_FLAG_SGN,
                offsetof(unitree_hg_lowstate_t, bms_bq_ntc), 2u,
    /* 43 */ DDS_OP_ADR | DDS_OP_TYPE_ARR | DDS_OP_SUBTYPE_1BY | DDS_OP_FLAG_SGN,
                offsetof(unitree_hg_lowstate_t, bms_mcu_ntc), 2u,
    /* 46 */ DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof(unitree_hg_lowstate_t, bms_vol),
    /* 48 */ DDS_OP_ADR | DDS_OP_TYPE_ARR | DDS_OP_SUBTYPE_1BY,
                offsetof(unitree_hg_lowstate_t, wireless_remote), 40u,
    /* 51 */ DDS_OP_ADR | DDS_OP_TYPE_ARR | DDS_OP_SUBTYPE_4BY,
                offsetof(unitree_hg_lowstate_t, reserve), 4u,
    /* 54 */ DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof(unitree_hg_lowstate_t, crc),
    /* 56 */ DDS_OP_RTS,
    /* --- element ops for unitree_hg_motor_state_t --- */
    /* 57 */ DDS_OP_ADR | DDS_OP_TYPE_1BY, offsetof(unitree_hg_motor_state_t, mode),
    /* 59 */ DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof(unitree_hg_motor_state_t, q),
    /* 61 */ DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof(unitree_hg_motor_state_t, dq),
    /* 63 */ DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof(unitree_hg_motor_state_t, ddq),
    /* 65 */ DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof(unitree_hg_motor_state_t, tau_est),
    /* 67 */ DDS_OP_ADR | DDS_OP_TYPE_ARR | DDS_OP_SUBTYPE_4BY,
                offsetof(unitree_hg_motor_state_t, temperature), 2u,
    /* 70 */ DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof(unitree_hg_motor_state_t, vol),
    /* 72 */ DDS_OP_ADR | DDS_OP_TYPE_ARR | DDS_OP_SUBTYPE_4BY,
                offsetof(unitree_hg_motor_state_t, sensor), 2u,
    /* 75 */ DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof(unitree_hg_motor_state_t, motorstate),
    /* 77 */ DDS_OP_ADR | DDS_OP_TYPE_ARR | DDS_OP_SUBTYPE_4BY,
                offsetof(unitree_hg_motor_state_t, reserve), 4u,
    /* 80 */ DDS_OP_RTS
};

const dds_topic_descriptor_t unitree_hg_lowstate_desc = {
    .m_size = sizeof(unitree_hg_lowstate_t),
    .m_align = sizeof(uint32_t),
    .m_flagset = DDS_TOPIC_NO_OPTIMIZE,
    .m_nkeys = 0u,
    .m_typename = "unitree_hg::msg::dds_::LowState_",
    .m_keys = NULL,
    .m_nops = sizeof(unitree_hg_lowstate_ops) / sizeof(uint32_t),
    .m_ops = unitree_hg_lowstate_ops,
    .m_meta = ""
};

/* --- DDS infrastructure --- */

static dds_entity_t g_participant = 0;

int unitree_dds_init(int domain_id, const char *network_interface) {
    if (g_participant > 0)
        return 0; /* already initialized */

    /* Configure network interface via DDS environment variable if specified. */
    if (network_interface && network_interface[0] != '\0') {
        /* CycloneDDS uses CYCLONEDDS_URI for configuration.
           Set the network interface via the config XML. */
        char cfg[512];
        snprintf(cfg, sizeof(cfg),
            "<CycloneDDS><Domain><General>"
            "<Interfaces><NetworkInterface name=\"%s\"/></Interfaces>"
            "</General></Domain></CycloneDDS>",
            network_interface);
        g_participant = dds_create_participant(domain_id, NULL, NULL);
        if (g_participant < 0) {
            /* Try with config */
            setenv("CYCLONEDDS_URI", cfg, 1);
            g_participant = dds_create_participant(domain_id, NULL, NULL);
        }
    } else {
        g_participant = dds_create_participant(domain_id, NULL, NULL);
    }

    return (g_participant > 0) ? 0 : -1;
}

int unitree_dds_create_rpc(const char *service_name,
                           dds_entity_t *writer_out,
                           dds_entity_t *reader_out) {
    if (g_participant <= 0)
        return -1;

    char req_topic_name[128], resp_topic_name[128];
    snprintf(req_topic_name, sizeof(req_topic_name), "rt/api/%s/request", service_name);
    snprintf(resp_topic_name, sizeof(resp_topic_name), "rt/api/%s/response", service_name);

    dds_entity_t req_topic = dds_create_topic(
        g_participant, &unitree_request_desc, req_topic_name, NULL, NULL);
    if (req_topic < 0)
        return -1;

    dds_entity_t resp_topic = dds_create_topic(
        g_participant, &unitree_response_desc, resp_topic_name, NULL, NULL);
    if (resp_topic < 0)
        return -1;

    /* Create writer with reliable QoS */
    dds_qos_t *wqos = dds_create_qos();
    dds_qset_reliability(wqos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
    *writer_out = dds_create_writer(g_participant, req_topic, wqos, NULL);
    dds_delete_qos(wqos);
    if (*writer_out < 0)
        return -1;

    /* Create reader with reliable QoS */
    dds_qos_t *rqos = dds_create_qos();
    dds_qset_reliability(rqos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
    *reader_out = dds_create_reader(g_participant, resp_topic, rqos, NULL);
    dds_delete_qos(rqos);
    if (*reader_out < 0)
        return -1;

    return 0;
}

int unitree_dds_write_request(dds_entity_t writer,
                              int64_t req_id, int64_t api_id,
                              const char *params_json) {
    unitree_request_t req;
    memset(&req, 0, sizeof(req));
    req.identity_id = req_id;
    req.identity_api_id = api_id;
    req.lease_id = 0;
    req.policy_priority = 0;
    req.policy_noreply = 0;
    req.parameter = (char *)params_json;
    req.binary._maximum = 0;
    req.binary._length = 0;
    req.binary._buffer = NULL;

    dds_return_t rc = dds_write(writer, &req);
    return (rc == DDS_RETCODE_OK) ? 0 : -1;
}

int unitree_dds_read_response(dds_entity_t reader, int timeout_ms,
                              unitree_response_t *out) {
    void *samples[1] = { NULL };
    dds_sample_info_t infos[1];

    dds_duration_t timeout = DDS_MSECS(timeout_ms);
    dds_entity_t ws = dds_create_waitset(g_participant);
    dds_entity_t rc_cond = dds_create_readcondition(reader, DDS_ANY_STATE);
    dds_waitset_attach(ws, rc_cond, 0);

    dds_return_t rc = dds_waitset_wait(ws, NULL, 0, timeout);
    dds_waitset_detach(ws, rc_cond);
    dds_delete(rc_cond);
    dds_delete(ws);

    if (rc <= 0)
        return -1; /* timeout or error */

    rc = dds_take(reader, samples, infos, 1, 1);
    if (rc <= 0 || !infos[0].valid_data) {
        if (rc > 0)
            dds_return_loan(reader, samples, rc);
        return -1;
    }

    /* Copy the response data out */
    unitree_response_t *resp = (unitree_response_t *)samples[0];
    memset(out, 0, sizeof(*out));
    out->identity_id = resp->identity_id;
    out->identity_api_id = resp->identity_api_id;
    out->status_code = resp->status_code;
    out->data = resp->data ? strdup(resp->data) : NULL;
    if (resp->binary._length > 0 && resp->binary._buffer) {
        out->binary._length = resp->binary._length;
        out->binary._maximum = resp->binary._length;
        out->binary._buffer = (uint8_t *)malloc(resp->binary._length);
        memcpy(out->binary._buffer, resp->binary._buffer, resp->binary._length);
    }

    dds_return_loan(reader, samples, 1);
    return 0;
}

void unitree_response_free(unitree_response_t *resp) {
    if (resp->data) {
        free(resp->data);
        resp->data = NULL;
    }
    if (resp->binary._buffer) {
        free(resp->binary._buffer);
        resp->binary._buffer = NULL;
    }
}

int unitree_dds_subscribe(const char *topic_name, int topic_type,
                          dds_entity_t *reader_out) {
    if (g_participant <= 0)
        return -1;

    const dds_topic_descriptor_t *desc = NULL;
    switch (topic_type) {
        case 0: desc = &unitree_pointcloud2_desc; break;
        case 1: desc = &unitree_hg_lowstate_desc; break;
        default: return -1;
    }

    dds_entity_t topic = dds_create_topic(g_participant, desc, topic_name, NULL, NULL);
    if (topic < 0)
        return -1;

    /* Sensor data: BEST_EFFORT, KEEP_LAST(1) so we always get the freshest sample. */
    dds_qos_t *qos = dds_create_qos();
    dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, 0);
    dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);
    *reader_out = dds_create_reader(g_participant, topic, qos, NULL);
    dds_delete_qos(qos);
    if (*reader_out < 0)
        return -1;

    return 0;
}

int unitree_dds_take_pointcloud2(dds_entity_t reader, int timeout_ms,
                                 unitree_pointcloud2_t *out) {
    void *samples[1] = { NULL };
    dds_sample_info_t infos[1];

    dds_duration_t timeout = DDS_MSECS(timeout_ms);
    dds_entity_t ws = dds_create_waitset(g_participant);
    dds_entity_t cond = dds_create_readcondition(reader, DDS_ANY_STATE);
    dds_waitset_attach(ws, cond, 0);

    dds_return_t rc = dds_waitset_wait(ws, NULL, 0, timeout);
    dds_waitset_detach(ws, cond);
    dds_delete(cond);
    dds_delete(ws);

    if (rc <= 0)
        return -1;

    rc = dds_take(reader, samples, infos, 1, 1);
    if (rc <= 0 || !infos[0].valid_data) {
        if (rc > 0) dds_return_loan(reader, samples, rc);
        return -1;
    }

    /* Deep-copy out so caller can free the loan. */
    unitree_pointcloud2_t *src = (unitree_pointcloud2_t *)samples[0];
    memset(out, 0, sizeof(*out));
    out->stamp_sec     = src->stamp_sec;
    out->stamp_nanosec = src->stamp_nanosec;
    out->frame_id      = src->frame_id ? strdup(src->frame_id) : NULL;
    out->height        = src->height;
    out->width         = src->width;
    out->is_bigendian  = src->is_bigendian;
    out->point_step    = src->point_step;
    out->row_step      = src->row_step;
    out->is_dense      = src->is_dense;

    if (src->fields._length > 0 && src->fields._buffer) {
        out->fields._length  = src->fields._length;
        out->fields._maximum = src->fields._length;
        out->fields._buffer = (unitree_point_field_t *)calloc(
            src->fields._length, sizeof(unitree_point_field_t));
        for (uint32_t i = 0; i < src->fields._length; i++) {
            out->fields._buffer[i].name = src->fields._buffer[i].name
                ? strdup(src->fields._buffer[i].name) : NULL;
            out->fields._buffer[i].offset   = src->fields._buffer[i].offset;
            out->fields._buffer[i].datatype = src->fields._buffer[i].datatype;
            out->fields._buffer[i].count    = src->fields._buffer[i].count;
        }
    }

    if (src->data._length > 0 && src->data._buffer) {
        out->data._length  = src->data._length;
        out->data._maximum = src->data._length;
        out->data._buffer  = (uint8_t *)malloc(src->data._length);
        memcpy(out->data._buffer, src->data._buffer, src->data._length);
    }

    dds_return_loan(reader, samples, 1);
    return 0;
}

void unitree_pointcloud2_free(unitree_pointcloud2_t *pc) {
    if (pc->frame_id) { free(pc->frame_id); pc->frame_id = NULL; }
    if (pc->fields._buffer) {
        for (uint32_t i = 0; i < pc->fields._length; i++) {
            if (pc->fields._buffer[i].name) free(pc->fields._buffer[i].name);
        }
        free(pc->fields._buffer);
        pc->fields._buffer = NULL;
        pc->fields._length = 0;
    }
    if (pc->data._buffer) {
        free(pc->data._buffer);
        pc->data._buffer = NULL;
        pc->data._length = 0;
    }
}

int unitree_dds_create_lowcmd_writer(const char *topic_name,
                                     dds_entity_t *writer_out) {
    if (g_participant <= 0)
        return -1;

    dds_entity_t topic = dds_create_topic(
        g_participant, &unitree_hg_lowcmd_desc, topic_name, NULL, NULL);
    if (topic < 0)
        return -1;

    /* arm_sdk topic: best-effort, KEEP_LAST(1) - matches Unitree examples. */
    dds_qos_t *qos = dds_create_qos();
    dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, 0);
    dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);
    *writer_out = dds_create_writer(g_participant, topic, qos, NULL);
    dds_delete_qos(qos);
    if (*writer_out < 0)
        return -1;
    return 0;
}

int unitree_dds_publish_lowcmd(dds_entity_t writer, const unitree_hg_lowcmd_t *cmd) {
    dds_return_t rc = dds_write(writer, cmd);
    return (rc == DDS_RETCODE_OK) ? 0 : -1;
}

int unitree_dds_take_lowstate(dds_entity_t reader, int timeout_ms,
                              unitree_hg_lowstate_t *out) {
    void *samples[1] = { NULL };
    dds_sample_info_t infos[1];

    dds_duration_t timeout = DDS_MSECS(timeout_ms);
    dds_entity_t ws = dds_create_waitset(g_participant);
    dds_entity_t cond = dds_create_readcondition(reader, DDS_ANY_STATE);
    dds_waitset_attach(ws, cond, 0);

    dds_return_t rc = dds_waitset_wait(ws, NULL, 0, timeout);
    dds_waitset_detach(ws, cond);
    dds_delete(cond);
    dds_delete(ws);

    if (rc <= 0)
        return -1;

    rc = dds_take(reader, samples, infos, 1, 1);
    if (rc <= 0 || !infos[0].valid_data) {
        if (rc > 0) dds_return_loan(reader, samples, rc);
        return -1;
    }

    /* Plain-old-data: shallow copy is sufficient (no embedded pointers). */
    memcpy(out, samples[0], sizeof(*out));
    dds_return_loan(reader, samples, 1);
    return 0;
}

void unitree_dds_close_writer(dds_entity_t writer) {
    dds_entity_t topic = (writer > 0) ? dds_get_topic(writer) : 0;
    if (writer > 0) dds_delete(writer);
    if (topic > 0) dds_delete(topic);
}

void unitree_dds_close_subscriber(dds_entity_t reader) {
    dds_entity_t topic = (reader > 0) ? dds_get_topic(reader) : 0;
    if (reader > 0) dds_delete(reader);
    if (topic > 0) dds_delete(topic);
}

void unitree_dds_close_rpc(dds_entity_t writer, dds_entity_t reader) {
    /* Capture the topic handles before deleting the writers/readers.
       Topics are reference counted; we delete them so they don't accumulate
       across module reconfigurations. */
    dds_entity_t writer_topic = (writer > 0) ? dds_get_topic(writer) : 0;
    dds_entity_t reader_topic = (reader > 0) ? dds_get_topic(reader) : 0;

    if (writer > 0) dds_delete(writer);
    if (reader > 0) dds_delete(reader);
    if (writer_topic > 0) dds_delete(writer_topic);
    if (reader_topic > 0 && reader_topic != writer_topic) dds_delete(reader_topic);
}

void unitree_dds_shutdown(void) {
    if (g_participant > 0) {
        /* dds_delete on a participant cascades to all child entities and
           sends DDS "participant gone" so remote endpoints don't wait for
           the lease to time out. */
        dds_delete(g_participant);
        g_participant = 0;
    }
}

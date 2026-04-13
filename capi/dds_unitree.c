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

void unitree_dds_shutdown(void) {
    if (g_participant > 0) {
        dds_delete(g_participant);
        g_participant = 0;
    }
}

#pragma once

#include <inttypes.h>

#include "interface.h"

struct euicc_ctx
{
    struct
    {
        struct euicc_apdu_interface apdu;
        struct euicc_es9p_interface es9p;
    } interface;
    uint8_t g_apdu_request_buf[EUICC_INTERFACE_BUFSZ];
    uint8_t g_asn1_der_request_buf[256];
    int es10x_logic_channel;
};

#include "es9p.h"
#include "es10x.h"

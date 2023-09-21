/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "RSPDefinitions"
 * 	found in "../../../asn1/rsp.asn"
 * 	`asn1c -fwide-types -fcompound-names -fincludes-quoted -no-gen-example`
 */

#ifndef	_DeviceInfo_H_
#define	_DeviceInfo_H_


#include "asn_application.h"

/* Including external dependencies */
#include "Octet8.h"
#include "DeviceCapabilities.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* DeviceInfo */
typedef struct DeviceInfo {
	Octet8_t	 tac;
	DeviceCapabilities_t	 deviceCapabilities;
	Octet8_t	*imei	/* OPTIONAL */;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} DeviceInfo_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_DeviceInfo;
extern asn_SEQUENCE_specifics_t asn_SPC_DeviceInfo_specs_1;
extern asn_TYPE_member_t asn_MBR_DeviceInfo_1[3];

#ifdef __cplusplus
}
#endif

#endif	/* _DeviceInfo_H_ */
#include "asn_internal.h"

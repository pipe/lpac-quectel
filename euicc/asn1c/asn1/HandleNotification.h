/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "RSPDefinitions"
 * 	found in "../../../asn1/rsp.asn"
 * 	`asn1c -fwide-types -fcompound-names -fincludes-quoted -no-gen-example`
 */

#ifndef	_HandleNotification_H_
#define	_HandleNotification_H_


#include "asn_application.h"

/* Including external dependencies */
#include "PendingNotification.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* HandleNotification */
typedef struct HandleNotification {
	PendingNotification_t	 pendingNotification;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} HandleNotification_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_HandleNotification;
extern asn_SEQUENCE_specifics_t asn_SPC_HandleNotification_specs_1;
extern asn_TYPE_member_t asn_MBR_HandleNotification_1[1];

#ifdef __cplusplus
}
#endif

#endif	/* _HandleNotification_H_ */
#include "asn_internal.h"

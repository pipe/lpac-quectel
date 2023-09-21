/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "RSPDefinitions"
 * 	found in "../../../asn1/rsp.asn"
 * 	`asn1c -fwide-types -fcompound-names -fincludes-quoted -no-gen-example`
 */

#ifndef	_SuccessResult_H_
#define	_SuccessResult_H_


#include "asn_application.h"

/* Including external dependencies */
#include "OCTET_STRING.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* SuccessResult */
typedef struct SuccessResult {
	OCTET_STRING_t	 aid;
	OCTET_STRING_t	 simaResponse;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SuccessResult_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_SuccessResult;
extern asn_SEQUENCE_specifics_t asn_SPC_SuccessResult_specs_1;
extern asn_TYPE_member_t asn_MBR_SuccessResult_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _SuccessResult_H_ */
#include "asn_internal.h"

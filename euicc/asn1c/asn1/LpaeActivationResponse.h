/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "RSPDefinitions"
 * 	found in "../../../asn1/rsp.asn"
 * 	`asn1c -fwide-types -fcompound-names -fincludes-quoted -no-gen-example`
 */

#ifndef	_LpaeActivationResponse_H_
#define	_LpaeActivationResponse_H_


#include "asn_application.h"

/* Including external dependencies */
#include "INTEGER.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum LpaeActivationResponse__lpaeActivationResult {
	LpaeActivationResponse__lpaeActivationResult_ok	= 0,
	LpaeActivationResponse__lpaeActivationResult_notSupported	= 1
} e_LpaeActivationResponse__lpaeActivationResult;

/* LpaeActivationResponse */
typedef struct LpaeActivationResponse {
	INTEGER_t	 lpaeActivationResult;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} LpaeActivationResponse_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_LpaeActivationResponse;

#ifdef __cplusplus
}
#endif

#endif	/* _LpaeActivationResponse_H_ */
#include "asn_internal.h"

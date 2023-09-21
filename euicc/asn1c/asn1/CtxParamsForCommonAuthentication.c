/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "RSPDefinitions"
 * 	found in "../../../asn1/rsp.asn"
 * 	`asn1c -fwide-types -fcompound-names -fincludes-quoted -no-gen-example`
 */

#include "CtxParamsForCommonAuthentication.h"

asn_TYPE_member_t asn_MBR_CtxParamsForCommonAuthentication_1[] = {
	{ ATF_POINTER, 1, offsetof(struct CtxParamsForCommonAuthentication, matchingId),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_UTF8String,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"matchingId"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct CtxParamsForCommonAuthentication, deviceInfo),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_DeviceInfo,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"deviceInfo"
		},
};
static const int asn_MAP_CtxParamsForCommonAuthentication_oms_1[] = { 0 };
static const ber_tlv_tag_t asn_DEF_CtxParamsForCommonAuthentication_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_CtxParamsForCommonAuthentication_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* matchingId */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* deviceInfo */
};
asn_SEQUENCE_specifics_t asn_SPC_CtxParamsForCommonAuthentication_specs_1 = {
	sizeof(struct CtxParamsForCommonAuthentication),
	offsetof(struct CtxParamsForCommonAuthentication, _asn_ctx),
	asn_MAP_CtxParamsForCommonAuthentication_tag2el_1,
	2,	/* Count of tags in the map */
	asn_MAP_CtxParamsForCommonAuthentication_oms_1,	/* Optional members */
	1, 0,	/* Root/Additions */
	2,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_CtxParamsForCommonAuthentication = {
	"CtxParamsForCommonAuthentication",
	"CtxParamsForCommonAuthentication",
	&asn_OP_SEQUENCE,
	asn_DEF_CtxParamsForCommonAuthentication_tags_1,
	sizeof(asn_DEF_CtxParamsForCommonAuthentication_tags_1)
		/sizeof(asn_DEF_CtxParamsForCommonAuthentication_tags_1[0]), /* 1 */
	asn_DEF_CtxParamsForCommonAuthentication_tags_1,	/* Same as above */
	sizeof(asn_DEF_CtxParamsForCommonAuthentication_tags_1)
		/sizeof(asn_DEF_CtxParamsForCommonAuthentication_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_CtxParamsForCommonAuthentication_1,
	2,	/* Elements count */
	&asn_SPC_CtxParamsForCommonAuthentication_specs_1	/* Additional specs */
};


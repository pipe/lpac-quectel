/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "RSPDefinitions"
 * 	found in "../../../asn1/rsp.asn"
 * 	`asn1c -fwide-types -fcompound-names -fincludes-quoted -no-gen-example`
 */

#include "ReplaceSessionKeysRequest.h"

static asn_TYPE_member_t asn_MBR_ReplaceSessionKeysRequest_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct ReplaceSessionKeysRequest, initialMacChainingValue),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_OCTET_STRING,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"initialMacChainingValue"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct ReplaceSessionKeysRequest, ppkEnc),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_OCTET_STRING,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"ppkEnc"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct ReplaceSessionKeysRequest, ppkCmac),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_OCTET_STRING,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"ppkCmac"
		},
};
static const ber_tlv_tag_t asn_DEF_ReplaceSessionKeysRequest_tags_1[] = {
	(ASN_TAG_CLASS_CONTEXT | (38 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_ReplaceSessionKeysRequest_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* initialMacChainingValue */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* ppkEnc */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 } /* ppkCmac */
};
static asn_SEQUENCE_specifics_t asn_SPC_ReplaceSessionKeysRequest_specs_1 = {
	sizeof(struct ReplaceSessionKeysRequest),
	offsetof(struct ReplaceSessionKeysRequest, _asn_ctx),
	asn_MAP_ReplaceSessionKeysRequest_tag2el_1,
	3,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	3,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_ReplaceSessionKeysRequest = {
	"ReplaceSessionKeysRequest",
	"ReplaceSessionKeysRequest",
	&asn_OP_SEQUENCE,
	asn_DEF_ReplaceSessionKeysRequest_tags_1,
	sizeof(asn_DEF_ReplaceSessionKeysRequest_tags_1)
		/sizeof(asn_DEF_ReplaceSessionKeysRequest_tags_1[0]) - 1, /* 1 */
	asn_DEF_ReplaceSessionKeysRequest_tags_1,	/* Same as above */
	sizeof(asn_DEF_ReplaceSessionKeysRequest_tags_1)
		/sizeof(asn_DEF_ReplaceSessionKeysRequest_tags_1[0]), /* 2 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_ReplaceSessionKeysRequest_1,
	3,	/* Elements count */
	&asn_SPC_ReplaceSessionKeysRequest_specs_1	/* Additional specs */
};


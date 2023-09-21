/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "RSPDefinitions"
 * 	found in "../../../asn1/rsp.asn"
 * 	`asn1c -fwide-types -fcompound-names -fincludes-quoted -no-gen-example`
 */

#include "EuiccSigned1.h"

asn_TYPE_member_t asn_MBR_EuiccSigned1_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct EuiccSigned1, transactionId),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_TransactionId,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"transactionId"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct EuiccSigned1, serverAddress),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_UTF8String,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"serverAddress"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct EuiccSigned1, serverChallenge),
		(ASN_TAG_CLASS_CONTEXT | (4 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_Octet16,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"serverChallenge"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct EuiccSigned1, euiccInfo2),
		(ASN_TAG_CLASS_CONTEXT | (34 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_EUICCInfo2,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"euiccInfo2"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct EuiccSigned1, ctxParams1),
		-1 /* Ambiguous tag (CHOICE?) */,
		0,
		&asn_DEF_CtxParams1,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"ctxParams1"
		},
};
static const ber_tlv_tag_t asn_DEF_EuiccSigned1_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_EuiccSigned1_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 1 }, /* transactionId */
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 4, -1, 0 }, /* ctxParamsForCommonAuthentication */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 1, 0, 0 }, /* serverAddress */
    { (ASN_TAG_CLASS_CONTEXT | (4 << 2)), 2, 0, 0 }, /* serverChallenge */
    { (ASN_TAG_CLASS_CONTEXT | (34 << 2)), 3, 0, 0 } /* euiccInfo2 */
};
asn_SEQUENCE_specifics_t asn_SPC_EuiccSigned1_specs_1 = {
	sizeof(struct EuiccSigned1),
	offsetof(struct EuiccSigned1, _asn_ctx),
	asn_MAP_EuiccSigned1_tag2el_1,
	5,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	5,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_EuiccSigned1 = {
	"EuiccSigned1",
	"EuiccSigned1",
	&asn_OP_SEQUENCE,
	asn_DEF_EuiccSigned1_tags_1,
	sizeof(asn_DEF_EuiccSigned1_tags_1)
		/sizeof(asn_DEF_EuiccSigned1_tags_1[0]), /* 1 */
	asn_DEF_EuiccSigned1_tags_1,	/* Same as above */
	sizeof(asn_DEF_EuiccSigned1_tags_1)
		/sizeof(asn_DEF_EuiccSigned1_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_EuiccSigned1_1,
	5,	/* Elements count */
	&asn_SPC_EuiccSigned1_specs_1	/* Additional specs */
};


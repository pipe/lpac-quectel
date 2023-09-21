/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "RSPDefinitions"
 * 	found in "../../../asn1/rsp.asn"
 * 	`asn1c -fwide-types -fcompound-names -fincludes-quoted -no-gen-example`
 */

#include "DeleteProfileRequest.h"

static asn_oer_constraints_t asn_OER_type_DeleteProfileRequest_constr_1 CC_NOTUSED = {
	{ 0, 0 },
	-1};
static asn_per_constraints_t asn_PER_type_DeleteProfileRequest_constr_1 CC_NOTUSED = {
	{ APC_CONSTRAINED | APC_EXTENSIBLE,  1,  1,  0,  1 }	/* (0..1,...) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_TYPE_member_t asn_MBR_DeleteProfileRequest_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct DeleteProfileRequest, choice.isdpAid),
		(ASN_TAG_CLASS_APPLICATION | (15 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_OctetTo16,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"isdpAid"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct DeleteProfileRequest, choice.iccid),
		(ASN_TAG_CLASS_APPLICATION | (26 << 2)),
		0,
		&asn_DEF_Iccid,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"iccid"
		},
};
static const ber_tlv_tag_t asn_DEF_DeleteProfileRequest_tags_1[] = {
	(ASN_TAG_CLASS_CONTEXT | (51 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_DeleteProfileRequest_tag2el_1[] = {
    { (ASN_TAG_CLASS_APPLICATION | (15 << 2)), 0, 0, 0 }, /* isdpAid */
    { (ASN_TAG_CLASS_APPLICATION | (26 << 2)), 1, 0, 0 } /* iccid */
};
static asn_CHOICE_specifics_t asn_SPC_DeleteProfileRequest_specs_1 = {
	sizeof(struct DeleteProfileRequest),
	offsetof(struct DeleteProfileRequest, _asn_ctx),
	offsetof(struct DeleteProfileRequest, present),
	sizeof(((struct DeleteProfileRequest *)0)->present),
	asn_MAP_DeleteProfileRequest_tag2el_1,
	2,	/* Count of tags in the map */
	0, 0,
	2	/* Extensions start */
};
asn_TYPE_descriptor_t asn_DEF_DeleteProfileRequest = {
	"DeleteProfileRequest",
	"DeleteProfileRequest",
	&asn_OP_CHOICE,
	asn_DEF_DeleteProfileRequest_tags_1,
	sizeof(asn_DEF_DeleteProfileRequest_tags_1)
		/sizeof(asn_DEF_DeleteProfileRequest_tags_1[0]), /* 1 */
	asn_DEF_DeleteProfileRequest_tags_1,	/* Same as above */
	sizeof(asn_DEF_DeleteProfileRequest_tags_1)
		/sizeof(asn_DEF_DeleteProfileRequest_tags_1[0]), /* 1 */
	{ &asn_OER_type_DeleteProfileRequest_constr_1, &asn_PER_type_DeleteProfileRequest_constr_1, CHOICE_constraint },
	asn_MBR_DeleteProfileRequest_1,
	2,	/* Elements count */
	&asn_SPC_DeleteProfileRequest_specs_1	/* Additional specs */
};


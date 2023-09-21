/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "RSPDefinitions"
 * 	found in "../../../asn1/rsp.asn"
 * 	`asn1c -fwide-types -fcompound-names -fincludes-quoted -no-gen-example`
 */

#include "LoadCRLResponseOk.h"

static int
memb_number_constraint_3(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	const INTEGER_t *st = (const INTEGER_t *)sptr;
	long value;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	/* Check if the sign bit is present */
	value = st->buf ? ((st->buf[0] & 0x80) ? -1 : 1) : 0;
	
	if((value >= 0)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static asn_oer_constraints_t asn_OER_memb_number_constr_4 CC_NOTUSED = {
	{ 0, 1 }	/* (0..MAX) */,
	-1};
static asn_per_constraints_t asn_PER_memb_number_constr_4 CC_NOTUSED = {
	{ APC_SEMI_CONSTRAINED,	-1, -1,  0,  0 }	/* (0..MAX) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_TYPE_member_t asn_MBR_Member_3[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct LoadCRLResponseOk__missingParts__Member, number),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_INTEGER,
		0,
		{ &asn_OER_memb_number_constr_4, &asn_PER_memb_number_constr_4,  memb_number_constraint_3 },
		0, 0, /* No default value */
		"number"
		},
};
static const ber_tlv_tag_t asn_DEF_Member_tags_3[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_Member_tag2el_3[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 } /* number */
};
static asn_SEQUENCE_specifics_t asn_SPC_Member_specs_3 = {
	sizeof(struct LoadCRLResponseOk__missingParts__Member),
	offsetof(struct LoadCRLResponseOk__missingParts__Member, _asn_ctx),
	asn_MAP_Member_tag2el_3,
	1,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	1,	/* First extension addition */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_Member_3 = {
	"SEQUENCE",
	"SEQUENCE",
	&asn_OP_SEQUENCE,
	asn_DEF_Member_tags_3,
	sizeof(asn_DEF_Member_tags_3)
		/sizeof(asn_DEF_Member_tags_3[0]), /* 1 */
	asn_DEF_Member_tags_3,	/* Same as above */
	sizeof(asn_DEF_Member_tags_3)
		/sizeof(asn_DEF_Member_tags_3[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_Member_3,
	1,	/* Elements count */
	&asn_SPC_Member_specs_3	/* Additional specs */
};

static asn_TYPE_member_t asn_MBR_missingParts_2[] = {
	{ ATF_POINTER, 0, 0,
		(ASN_TAG_CLASS_UNIVERSAL | (16 << 2)),
		0,
		&asn_DEF_Member_3,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		""
		},
};
static const ber_tlv_tag_t asn_DEF_missingParts_tags_2[] = {
	(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static asn_SET_OF_specifics_t asn_SPC_missingParts_specs_2 = {
	sizeof(struct LoadCRLResponseOk__missingParts),
	offsetof(struct LoadCRLResponseOk__missingParts, _asn_ctx),
	0,	/* XER encoding is XMLDelimitedItemList */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_missingParts_2 = {
	"missingParts",
	"missingParts",
	&asn_OP_SEQUENCE_OF,
	asn_DEF_missingParts_tags_2,
	sizeof(asn_DEF_missingParts_tags_2)
		/sizeof(asn_DEF_missingParts_tags_2[0]) - 1, /* 1 */
	asn_DEF_missingParts_tags_2,	/* Same as above */
	sizeof(asn_DEF_missingParts_tags_2)
		/sizeof(asn_DEF_missingParts_tags_2[0]), /* 2 */
	{ 0, 0, SEQUENCE_OF_constraint },
	asn_MBR_missingParts_2,
	1,	/* Single element */
	&asn_SPC_missingParts_specs_2	/* Additional specs */
};

asn_TYPE_member_t asn_MBR_LoadCRLResponseOk_1[] = {
	{ ATF_POINTER, 1, offsetof(struct LoadCRLResponseOk, missingParts),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		0,
		&asn_DEF_missingParts_2,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"missingParts"
		},
};
static const int asn_MAP_LoadCRLResponseOk_oms_1[] = { 0 };
static const ber_tlv_tag_t asn_DEF_LoadCRLResponseOk_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_LoadCRLResponseOk_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 } /* missingParts */
};
asn_SEQUENCE_specifics_t asn_SPC_LoadCRLResponseOk_specs_1 = {
	sizeof(struct LoadCRLResponseOk),
	offsetof(struct LoadCRLResponseOk, _asn_ctx),
	asn_MAP_LoadCRLResponseOk_tag2el_1,
	1,	/* Count of tags in the map */
	asn_MAP_LoadCRLResponseOk_oms_1,	/* Optional members */
	1, 0,	/* Root/Additions */
	1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_LoadCRLResponseOk = {
	"LoadCRLResponseOk",
	"LoadCRLResponseOk",
	&asn_OP_SEQUENCE,
	asn_DEF_LoadCRLResponseOk_tags_1,
	sizeof(asn_DEF_LoadCRLResponseOk_tags_1)
		/sizeof(asn_DEF_LoadCRLResponseOk_tags_1[0]), /* 1 */
	asn_DEF_LoadCRLResponseOk_tags_1,	/* Same as above */
	sizeof(asn_DEF_LoadCRLResponseOk_tags_1)
		/sizeof(asn_DEF_LoadCRLResponseOk_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_LoadCRLResponseOk_1,
	1,	/* Elements count */
	&asn_SPC_LoadCRLResponseOk_specs_1	/* Additional specs */
};


/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "RSPDefinitions"
 * 	found in "../../../asn1/rsp.asn"
 * 	`asn1c -fwide-types -fcompound-names -fincludes-quoted -no-gen-example`
 */

#include "OctetTo16.h"

int
OctetTo16_constraint(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	const OCTET_STRING_t *st = (const OCTET_STRING_t *)sptr;
	size_t size;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	size = st->size;
	
	if((size >= 1 && size <= 16)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

/*
 * This type is implemented using OCTET_STRING,
 * so here we adjust the DEF accordingly.
 */
static asn_oer_constraints_t asn_OER_type_OctetTo16_constr_1 CC_NOTUSED = {
	{ 0, 0 },
	-1	/* (SIZE(1..16)) */};
asn_per_constraints_t asn_PER_type_OctetTo16_constr_1 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED,	 4,  4,  1,  16 }	/* (SIZE(1..16)) */,
	0, 0	/* No PER value map */
};
static const ber_tlv_tag_t asn_DEF_OctetTo16_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (4 << 2))
};
asn_TYPE_descriptor_t asn_DEF_OctetTo16 = {
	"OctetTo16",
	"OctetTo16",
	&asn_OP_OCTET_STRING,
	asn_DEF_OctetTo16_tags_1,
	sizeof(asn_DEF_OctetTo16_tags_1)
		/sizeof(asn_DEF_OctetTo16_tags_1[0]), /* 1 */
	asn_DEF_OctetTo16_tags_1,	/* Same as above */
	sizeof(asn_DEF_OctetTo16_tags_1)
		/sizeof(asn_DEF_OctetTo16_tags_1[0]), /* 1 */
	{ &asn_OER_type_OctetTo16_constr_1, &asn_PER_type_OctetTo16_constr_1, OctetTo16_constraint },
	0, 0,	/* No members */
	&asn_SPC_OCTET_STRING_specs	/* Additional specs */
};


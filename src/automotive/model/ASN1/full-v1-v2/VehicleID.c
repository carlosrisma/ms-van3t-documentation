/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/mnt/EVO/ASN1-C-ITS/DSRC.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example`
 */

#include "VehicleID.h"

static asn_oer_constraints_t asn_OER_type_VehicleID_constr_1 CC_NOTUSED = {
	{ 0, 0 },
	-1};
asn_per_constraints_t asn_PER_type_VehicleID_constr_1 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 1,  1,  0,  1 }	/* (0..1) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
asn_TYPE_member_t asn_MBR_VehicleID_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct VehicleID, choice.entityID),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_TemporaryID,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"entityID"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct VehicleID, choice.stationID),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_StationID,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"stationID"
		},
};
static const asn_TYPE_tag2member_t asn_MAP_VehicleID_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* entityID */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* stationID */
};
asn_CHOICE_specifics_t asn_SPC_VehicleID_specs_1 = {
	sizeof(struct VehicleID),
	offsetof(struct VehicleID, _asn_ctx),
	offsetof(struct VehicleID, present),
	sizeof(((struct VehicleID *)0)->present),
	asn_MAP_VehicleID_tag2el_1,
	2,	/* Count of tags in the map */
	0, 0,
	-1	/* Extensions start */
};
asn_TYPE_descriptor_t asn_DEF_VehicleID = {
	"VehicleID",
	"VehicleID",
	&asn_OP_CHOICE,
	0,	/* No effective tags (pointer) */
	0,	/* No effective tags (count) */
	0,	/* No tags (pointer) */
	0,	/* No tags (count) */
	{ &asn_OER_type_VehicleID_constr_1, &asn_PER_type_VehicleID_constr_1, CHOICE_constraint },
	asn_MBR_VehicleID_1,
	2,	/* Elements count */
	&asn_SPC_VehicleID_specs_1	/* Additional specs */
};

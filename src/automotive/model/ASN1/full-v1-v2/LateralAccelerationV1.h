/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/mnt/EVO/ASN1-C-ITS/ITS-Container.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example`
 */

#ifndef	_LateralAccelerationV1_H_
#define	_LateralAccelerationV1_H_


#include "asn_application.h"

/* Including external dependencies */
#include "LateralAccelerationValueV1.h"
#include "AccelerationConfidenceV1.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* LateralAccelerationV1 */
typedef struct LateralAccelerationV1 {
	LateralAccelerationValueV1_t	 lateralAccelerationValue;
	AccelerationConfidenceV1_t	 lateralAccelerationConfidence;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} LateralAccelerationV1_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_LateralAccelerationV1;
extern asn_SEQUENCE_specifics_t asn_SPC_LateralAccelerationV1_specs_1;
extern asn_TYPE_member_t asn_MBR_LateralAccelerationV1_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _LateralAccelerationV1_H_ */
#include "asn_internal.h"
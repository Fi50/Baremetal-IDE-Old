#ifndef __LL_PLIC_H
#define __LL_PLIC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rv_common.h"


/* Peripheral Struct Definition */
typedef struct {
  __IO uint32_t priority_threshold;
  __IO uint32_t claim_complete;
} PLIC_ContextControlEntry_TypeDef;

typedef struct {
  __IO uint32_t priorities[1024];
  __I  uint32_t pendings[1024];
  __IO uint32_t enables[1024];
} PLIC_TypeDef;

// because the maximum struct size is 65535, we need to split PLIC content
typedef struct {
  PLIC_ContextControlEntry_TypeDef context_controls[1024];
} PLIC_ContextControl_TypeDef;


#ifdef __cplusplus
}
#endif

#endif /* __LL_PLIC_H */
/**
 * @file hal_plic.h
 * @author -T.K.- / t_k_233@outlook.com
 * @brief 
 * @version 0.1
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef __HAL_PLIC_H
#define __HAL_PLIC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rv_common.h"
#include "ll_plic.h"


void HAL_PLIC_disable(PLIC_TypeDef *PLICx, PLIC_ContextControl_TypeDef *PLIC_CCx, uint32_t hart_id, uint32_t irq_id);

void HAL_PLIC_enable(PLIC_TypeDef *PLICx, PLIC_ContextControl_TypeDef *PLIC_CCx, uint32_t hart_id, uint32_t irq_id);

void HAL_PLIC_setPriority(PLIC_TypeDef *PLICx, PLIC_ContextControl_TypeDef *PLIC_CCx, uint32_t irq_id, uint32_t priority);

void HAL_PLIC_setPriorityThreshold(PLIC_TypeDef *PLICx, PLIC_ContextControl_TypeDef *PLIC_CCx, uint32_t hart_id, uint32_t priority);

uint32_t HAL_PLIC_claimIRQ(PLIC_TypeDef *PLICx, PLIC_ContextControl_TypeDef *PLIC_CCx, uint32_t hart_id);

void HAL_PLIC_completeIRQ(PLIC_TypeDef *PLICx, PLIC_ContextControl_TypeDef *PLIC_CCx, uint32_t hart_id, uint32_t irq_id);

#ifdef __cplusplus
}
#endif

#endif /* __HAL_PLIC_H */
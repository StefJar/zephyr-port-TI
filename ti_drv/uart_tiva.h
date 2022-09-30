/*
 * Copyright (c) 2021 Stefan Jaritz, TIER SE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SRC_DRV_TIVA_UART_TIVA_H_
#define SRC_DRV_TIVA_UART_TIVA_H_

void uart_tiva_blockSend(const struct device *dev, uint8_t * const src, const uint32_t N);

#endif /* SRC_DRV_TIVA_UART_TIVA_H_ */

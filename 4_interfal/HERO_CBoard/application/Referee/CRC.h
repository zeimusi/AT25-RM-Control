#ifndef _CRC_H_
#define _CRC_H_

#include "stdint.h"
#include "stddef.h"

#ifdef __cplusplus
extern "C" {
#endif
/**
*   @brief  计算CRC8校验和
*   @param[in] *pchMessage 数据包
*   @param[in]  dwLength   数据包的长度（减去CRC8的长度）
*   @return   数据包的校验和
*/
uint8_t Get_CRC8_Check_Sum(uint8_t *pchMessage, uint8_t dwLength);
/**
*   @brief  计算CRC16校验和
*   @param[in] *pchMessage 数据包
*   @param[in]  dwLength   数据包的长度（减去CRC16的长度）
*   @return   数据包的校验和
*/
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint8_t dwLength);

#ifdef __cplusplus
}
#endif

#endif

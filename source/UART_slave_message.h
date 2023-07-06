/*******************************************************************************
* File Name: UART_slave_message.h
*
* Description:
*  This file provides the source code to the API for the UART slave
*  packet protocol example
*
* Note:
*  Protocol description
*  Master(A) slave(B) communication:
*    |---|    --- Request send --->    |---|
*    | A |    {Wait for respond}       | B |
*    |---|    <--- Respond send ---    |--
*  Packet format:
*  <STX><ADDR><DL><[Data bytes length equal DL]><CRCH><CRCL>
*  STX        - 0x02 begin packet marker. Unique byte of start packet
*  ADDR    - device address
*  DL        - data length in bytes [1..255]
*  CRCH    - MSB of CRC-16 that calculated from ADDR to last data byte
*  CRCL    - LSB of CRC-16    that calculated from ADDR to last data byte
*  If there is a byte <ADDR> <DL> <[Data]> or <[CRC]> that equals STX
*  then it's exchanged with two byte sequence <ESC><STX+1>
*  If there is a byte <ADDR> <DL> <[Data]> or <[CRC]> that equals ESC
*  then it's exchanged with two byte sequence <ESC><ESC+1>
*
********************************************************************************
* Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
********************************************************************************/


#if !defined(UART_slave_message_H)
    #define UART_slave_message_H


/*******************************************************************************
* Function Prototypes
*******************************************************************************/

void UartMesSlave_Init(CySCB_Type* uart_base, uint8_t address);
uint8_t UartMesSlave_Respond(uint8_t * txd, uint8_t tlen);
uint8_t UartMesSlave_State(void);
uint8_t UartMesSlave_GetDataSize(void);
volatile uint8_t * UartMesSlave_GetDataPtr(void);
void UartMesSlave_Msg_ISR(void);


/*******************************************************************************
* Enumerated Types and Parameters
*******************************************************************************/

#define RX_TEST_SIZE        (10u)
#define RX_BUFF_SIZE        (16u)

#define UM_IDLE             (0u)
#define UM_PACKREADY        (1u)
#define UM_RESPOND          (2u)


#endif /* End __UART_slave_message_H */


/* [] END OF FILE */

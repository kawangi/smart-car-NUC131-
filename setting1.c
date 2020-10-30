/****************************************************************************
 * @file     setting1.c
 * @version  V1.09
 * @Date     2018/04/03-16:23:46 
 * @brief    NuMicro generated code file
 *
 * Copyright (C) 2013-2018 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

/********************
MCU: NUC131LD2AE(LQFP48)
Pin Configuration:
Pin 3: PWM1_CH4
Pin 29: ICE_DAT
Pin 30: ICE_CLK
Pin 32: ADC_CH0
Pin 33: PWM0_CH5
Pin 37: UART3_RXD
Pin 38: UART3_TXD
********************/

#include "NUC131.h";
/*
 * @brief This function provides the configued MFP registers
 * @param None
 * @return None
 */
void SYS_Init(void)
{
    //SYS->ALT_MFP = 0x00000000;
    //SYS->ALT_MFP2 = 0x00000000;
    //SYS->ALT_MFP3 = 0x00000420;
    //SYS->ALT_MFP4 = 0x00000030;
    //SYS->GPA_MFP = 0x00000063;
    //SYS->GPB_MFP = 0x00000000;
    //SYS->GPC_MFP = 0x00000000;
    //SYS->GPD_MFP = 0x00000000;
    //SYS->GPF_MFP = 0x000000D0;

    //If the defines do not exist in your project, please refer to the related sys.h in the sys_h folder appended to the tool package.
    SYS->ALT_MFP = 0x00000000;
    SYS->ALT_MFP2 = 0x00000000;
    SYS->ALT_MFP3 = SYS_ALT_MFP3_PF4_PWM1_CH4 | SYS_ALT_MFP3_PA1_PWM0_CH5;
    SYS->ALT_MFP4 = SYS_ALT_MFP4_PA6_UART3_TXD | SYS_ALT_MFP4_PA5_UART3_RXD;
    SYS->GPA_MFP = SYS_GPA_MFP_PA6_UART3_TXD | SYS_GPA_MFP_PA5_UART3_RXD | SYS_GPA_MFP_PA1_PWM0_CH5 | SYS_GPA_MFP_PA0_ADC0;
    SYS->GPB_MFP = 0x00000000;
    SYS->GPC_MFP = 0x00000000;
    SYS->GPD_MFP = 0x00000000;
    SYS->GPF_MFP = SYS_GPF_MFP_PF7_ICE_DAT | SYS_GPF_MFP_PF6_ICE_CLK | SYS_GPF_MFP_PF4_PWM1_CH4;

    return;
}

/*** (C) COPYRIGHT 2013-2018 Nuvoton Technology Corp. ***/
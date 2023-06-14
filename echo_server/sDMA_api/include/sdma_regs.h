#pragma once

/* ----------------------------------------------------------------------------
   -- SDMAARM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SDMAARM_Register_Masks SDMAARM Register Masks
 * @{
 */

/*! @name MC0PTR - Arm platform Channel 0 Pointer */
/*! @{ */

#define SDMAARM_MC0PTR_MC0PTR_MASK               (0xFFFFFFFFU)
#define SDMAARM_MC0PTR_MC0PTR_SHIFT              (0U)
#define SDMAARM_MC0PTR_MC0PTR(x)                 (((uint32_t)(((uint32_t)(x)) << SDMAARM_MC0PTR_MC0PTR_SHIFT)) & SDMAARM_MC0PTR_MC0PTR_MASK)
/*! @} */

/*! @name INTR - Channel Interrupts */
/*! @{ */

#define SDMAARM_INTR_HI_MASK                     (0xFFFFFFFFU)
#define SDMAARM_INTR_HI_SHIFT                    (0U)
#define SDMAARM_INTR_HI(x)                       (((uint32_t)(((uint32_t)(x)) << SDMAARM_INTR_HI_SHIFT)) & SDMAARM_INTR_HI_MASK)
/*! @} */

/*! @name STOP_STAT - Channel Stop/Channel Status */
/*! @{ */

#define SDMAARM_STOP_STAT_HE_MASK                (0xFFFFFFFFU)
#define SDMAARM_STOP_STAT_HE_SHIFT               (0U)
#define SDMAARM_STOP_STAT_HE(x)                  (((uint32_t)(((uint32_t)(x)) << SDMAARM_STOP_STAT_HE_SHIFT)) & SDMAARM_STOP_STAT_HE_MASK)
/*! @} */

/*! @name HSTART - Channel Start */
/*! @{ */

#define SDMAARM_HSTART_HSTART_HE_MASK            (0xFFFFFFFFU)
#define SDMAARM_HSTART_HSTART_HE_SHIFT           (0U)
#define SDMAARM_HSTART_HSTART_HE(x)              (((uint32_t)(((uint32_t)(x)) << SDMAARM_HSTART_HSTART_HE_SHIFT)) & SDMAARM_HSTART_HSTART_HE_MASK)
/*! @} */

/*! @name EVTOVR - Channel Event Override */
/*! @{ */

#define SDMAARM_EVTOVR_EO_MASK                   (0xFFFFFFFFU)
#define SDMAARM_EVTOVR_EO_SHIFT                  (0U)
#define SDMAARM_EVTOVR_EO(x)                     (((uint32_t)(((uint32_t)(x)) << SDMAARM_EVTOVR_EO_SHIFT)) & SDMAARM_EVTOVR_EO_MASK)
/*! @} */

/*! @name DSPOVR - Channel BP Override */
/*! @{ */

#define SDMAARM_DSPOVR_DO_MASK                   (0xFFFFFFFFU)
#define SDMAARM_DSPOVR_DO_SHIFT                  (0U)
/*! DO
 *  0b00000000000000000000000000000000..- Reserved
 *  0b00000000000000000000000000000001..- Reset value.
 */
#define SDMAARM_DSPOVR_DO(x)                     (((uint32_t)(((uint32_t)(x)) << SDMAARM_DSPOVR_DO_SHIFT)) & SDMAARM_DSPOVR_DO_MASK)
/*! @} */

/*! @name HOSTOVR - Channel Arm platform Override */
/*! @{ */

#define SDMAARM_HOSTOVR_HO_MASK                  (0xFFFFFFFFU)
#define SDMAARM_HOSTOVR_HO_SHIFT                 (0U)
#define SDMAARM_HOSTOVR_HO(x)                    (((uint32_t)(((uint32_t)(x)) << SDMAARM_HOSTOVR_HO_SHIFT)) & SDMAARM_HOSTOVR_HO_MASK)
/*! @} */

/*! @name EVTPEND - Channel Event Pending */
/*! @{ */

#define SDMAARM_EVTPEND_EP_MASK                  (0xFFFFFFFFU)
#define SDMAARM_EVTPEND_EP_SHIFT                 (0U)
#define SDMAARM_EVTPEND_EP(x)                    (((uint32_t)(((uint32_t)(x)) << SDMAARM_EVTPEND_EP_SHIFT)) & SDMAARM_EVTPEND_EP_MASK)
/*! @} */

/*! @name RESET - Reset Register */
/*! @{ */

#define SDMAARM_RESET_RESET_MASK                 (0x1U)
#define SDMAARM_RESET_RESET_SHIFT                (0U)
#define SDMAARM_RESET_RESET(x)                   (((uint32_t)(((uint32_t)(x)) << SDMAARM_RESET_RESET_SHIFT)) & SDMAARM_RESET_RESET_MASK)

#define SDMAARM_RESET_RESCHED_MASK               (0x2U)
#define SDMAARM_RESET_RESCHED_SHIFT              (1U)
#define SDMAARM_RESET_RESCHED(x)                 (((uint32_t)(((uint32_t)(x)) << SDMAARM_RESET_RESCHED_SHIFT)) & SDMAARM_RESET_RESCHED_MASK)
/*! @} */

/*! @name EVTERR - DMA Request Error Register */
/*! @{ */

#define SDMAARM_EVTERR_CHNERR_MASK               (0xFFFFFFFFU)
#define SDMAARM_EVTERR_CHNERR_SHIFT              (0U)
#define SDMAARM_EVTERR_CHNERR(x)                 (((uint32_t)(((uint32_t)(x)) << SDMAARM_EVTERR_CHNERR_SHIFT)) & SDMAARM_EVTERR_CHNERR_MASK)
/*! @} */

/*! @name INTRMASK - Channel Arm platform Interrupt Mask */
/*! @{ */

#define SDMAARM_INTRMASK_HIMASK_MASK             (0xFFFFFFFFU)
#define SDMAARM_INTRMASK_HIMASK_SHIFT            (0U)
#define SDMAARM_INTRMASK_HIMASK(x)               (((uint32_t)(((uint32_t)(x)) << SDMAARM_INTRMASK_HIMASK_SHIFT)) & SDMAARM_INTRMASK_HIMASK_MASK)
/*! @} */

/*! @name PSW - Schedule Status */
/*! @{ */

#define SDMAARM_PSW_CCR_MASK                     (0xFU)
#define SDMAARM_PSW_CCR_SHIFT                    (0U)
#define SDMAARM_PSW_CCR(x)                       (((uint32_t)(((uint32_t)(x)) << SDMAARM_PSW_CCR_SHIFT)) & SDMAARM_PSW_CCR_MASK)

#define SDMAARM_PSW_CCP_MASK                     (0xF0U)
#define SDMAARM_PSW_CCP_SHIFT                    (4U)
/*! CCP
 *  0b0000..No running channel
 *  0b0001..Active channel priority
 */
#define SDMAARM_PSW_CCP(x)                       (((uint32_t)(((uint32_t)(x)) << SDMAARM_PSW_CCP_SHIFT)) & SDMAARM_PSW_CCP_MASK)

#define SDMAARM_PSW_NCR_MASK                     (0x1F00U)
#define SDMAARM_PSW_NCR_SHIFT                    (8U)
#define SDMAARM_PSW_NCR(x)                       (((uint32_t)(((uint32_t)(x)) << SDMAARM_PSW_NCR_SHIFT)) & SDMAARM_PSW_NCR_MASK)

#define SDMAARM_PSW_NCP_MASK                     (0xE000U)
#define SDMAARM_PSW_NCP_SHIFT                    (13U)
/*! NCP
 *  0b000..No running channel
 *  0b001..Active channel priority
 */
#define SDMAARM_PSW_NCP(x)                       (((uint32_t)(((uint32_t)(x)) << SDMAARM_PSW_NCP_SHIFT)) & SDMAARM_PSW_NCP_MASK)
/*! @} */

/*! @name EVTERRDBG - DMA Request Error Register */
/*! @{ */

#define SDMAARM_EVTERRDBG_CHNERR_MASK            (0xFFFFFFFFU)
#define SDMAARM_EVTERRDBG_CHNERR_SHIFT           (0U)
#define SDMAARM_EVTERRDBG_CHNERR(x)              (((uint32_t)(((uint32_t)(x)) << SDMAARM_EVTERRDBG_CHNERR_SHIFT)) & SDMAARM_EVTERRDBG_CHNERR_MASK)
/*! @} */

/*! @name CONFIG - Configuration Register */
/*! @{ */

#define SDMAARM_CONFIG_CSM_MASK                  (0x3U)
#define SDMAARM_CONFIG_CSM_SHIFT                 (0U)
/*! CSM
 *  0b00..static
 *  0b01..dynamic low power
 *  0b10..dynamic with no loop
 *  0b11..dynamic
 */
#define SDMAARM_CONFIG_CSM(x)                    (((uint32_t)(((uint32_t)(x)) << SDMAARM_CONFIG_CSM_SHIFT)) & SDMAARM_CONFIG_CSM_MASK)

#define SDMAARM_CONFIG_ACR_MASK                  (0x10U)
#define SDMAARM_CONFIG_ACR_SHIFT                 (4U)
/*! ACR
 *  0b0..Arm platform DMA interface frequency equals twice core frequency
 *  0b1..Arm platform DMA interface frequency equals core frequency
 */
#define SDMAARM_CONFIG_ACR(x)                    (((uint32_t)(((uint32_t)(x)) << SDMAARM_CONFIG_ACR_SHIFT)) & SDMAARM_CONFIG_ACR_MASK)

#define SDMAARM_CONFIG_RTDOBS_MASK               (0x800U)
#define SDMAARM_CONFIG_RTDOBS_SHIFT              (11U)
/*! RTDOBS
 *  0b0..RTD pins disabled
 *  0b1..RTD pins enabled
 */
#define SDMAARM_CONFIG_RTDOBS(x)                 (((uint32_t)(((uint32_t)(x)) << SDMAARM_CONFIG_RTDOBS_SHIFT)) & SDMAARM_CONFIG_RTDOBS_MASK)

#define SDMAARM_CONFIG_DSPDMA_MASK               (0x1000U)
#define SDMAARM_CONFIG_DSPDMA_SHIFT              (12U)
/*! DSPDMA
 *  0b0..- Reset Value
 *  0b1..- Reserved
 */
#define SDMAARM_CONFIG_DSPDMA(x)                 (((uint32_t)(((uint32_t)(x)) << SDMAARM_CONFIG_DSPDMA_SHIFT)) & SDMAARM_CONFIG_DSPDMA_MASK)
/*! @} */

/*! @name SDMA_LOCK - SDMA LOCK */
/*! @{ */

#define SDMAARM_SDMA_LOCK_LOCK_MASK              (0x1U)
#define SDMAARM_SDMA_LOCK_LOCK_SHIFT             (0U)
/*! LOCK
 *  0b0..LOCK disengaged.
 *  0b1..LOCK enabled.
 */
#define SDMAARM_SDMA_LOCK_LOCK(x)                (((uint32_t)(((uint32_t)(x)) << SDMAARM_SDMA_LOCK_LOCK_SHIFT)) & SDMAARM_SDMA_LOCK_LOCK_MASK)

#define SDMAARM_SDMA_LOCK_SRESET_LOCK_CLR_MASK   (0x2U)
#define SDMAARM_SDMA_LOCK_SRESET_LOCK_CLR_SHIFT  (1U)
/*! SRESET_LOCK_CLR
 *  0b0..Software Reset does not clear the LOCK bit.
 *  0b1..Software Reset clears the LOCK bit.
 */
#define SDMAARM_SDMA_LOCK_SRESET_LOCK_CLR(x)     (((uint32_t)(((uint32_t)(x)) << SDMAARM_SDMA_LOCK_SRESET_LOCK_CLR_SHIFT)) & SDMAARM_SDMA_LOCK_SRESET_LOCK_CLR_MASK)
/*! @} */

/*! @name ONCE_ENB - OnCE Enable */
/*! @{ */

#define SDMAARM_ONCE_ENB_ENB_MASK                (0x1U)
#define SDMAARM_ONCE_ENB_ENB_SHIFT               (0U)
#define SDMAARM_ONCE_ENB_ENB(x)                  (((uint32_t)(((uint32_t)(x)) << SDMAARM_ONCE_ENB_ENB_SHIFT)) & SDMAARM_ONCE_ENB_ENB_MASK)
/*! @} */

/*! @name ONCE_DATA - OnCE Data Register */
/*! @{ */

#define SDMAARM_ONCE_DATA_DATA_MASK              (0xFFFFFFFFU)
#define SDMAARM_ONCE_DATA_DATA_SHIFT             (0U)
#define SDMAARM_ONCE_DATA_DATA(x)                (((uint32_t)(((uint32_t)(x)) << SDMAARM_ONCE_DATA_DATA_SHIFT)) & SDMAARM_ONCE_DATA_DATA_MASK)
/*! @} */

/*! @name ONCE_INSTR - OnCE Instruction Register */
/*! @{ */

#define SDMAARM_ONCE_INSTR_INSTR_MASK            (0xFFFFU)
#define SDMAARM_ONCE_INSTR_INSTR_SHIFT           (0U)
#define SDMAARM_ONCE_INSTR_INSTR(x)              (((uint32_t)(((uint32_t)(x)) << SDMAARM_ONCE_INSTR_INSTR_SHIFT)) & SDMAARM_ONCE_INSTR_INSTR_MASK)
/*! @} */

/*! @name ONCE_STAT - OnCE Status Register */
/*! @{ */

#define SDMAARM_ONCE_STAT_ECDR_MASK              (0x7U)
#define SDMAARM_ONCE_STAT_ECDR_SHIFT             (0U)
/*! ECDR
 *  0b000..1 matched addra_cond
 *  0b001..1 matched addrb_cond
 *  0b010..1 matched data_cond
 */
#define SDMAARM_ONCE_STAT_ECDR(x)                (((uint32_t)(((uint32_t)(x)) << SDMAARM_ONCE_STAT_ECDR_SHIFT)) & SDMAARM_ONCE_STAT_ECDR_MASK)

#define SDMAARM_ONCE_STAT_MST_MASK               (0x80U)
#define SDMAARM_ONCE_STAT_MST_SHIFT              (7U)
/*! MST
 *  0b0..The JTAG interface controls the OnCE.
 *  0b1..The Arm platform peripheral interface controls the OnCE.
 */
#define SDMAARM_ONCE_STAT_MST(x)                 (((uint32_t)(((uint32_t)(x)) << SDMAARM_ONCE_STAT_MST_SHIFT)) & SDMAARM_ONCE_STAT_MST_MASK)

#define SDMAARM_ONCE_STAT_SWB_MASK               (0x100U)
#define SDMAARM_ONCE_STAT_SWB_SHIFT              (8U)
#define SDMAARM_ONCE_STAT_SWB(x)                 (((uint32_t)(((uint32_t)(x)) << SDMAARM_ONCE_STAT_SWB_SHIFT)) & SDMAARM_ONCE_STAT_SWB_MASK)

#define SDMAARM_ONCE_STAT_ODR_MASK               (0x200U)
#define SDMAARM_ONCE_STAT_ODR_SHIFT              (9U)
#define SDMAARM_ONCE_STAT_ODR(x)                 (((uint32_t)(((uint32_t)(x)) << SDMAARM_ONCE_STAT_ODR_SHIFT)) & SDMAARM_ONCE_STAT_ODR_MASK)

#define SDMAARM_ONCE_STAT_EDR_MASK               (0x400U)
#define SDMAARM_ONCE_STAT_EDR_SHIFT              (10U)
#define SDMAARM_ONCE_STAT_EDR(x)                 (((uint32_t)(((uint32_t)(x)) << SDMAARM_ONCE_STAT_EDR_SHIFT)) & SDMAARM_ONCE_STAT_EDR_MASK)

#define SDMAARM_ONCE_STAT_RCV_MASK               (0x800U)
#define SDMAARM_ONCE_STAT_RCV_SHIFT              (11U)
#define SDMAARM_ONCE_STAT_RCV(x)                 (((uint32_t)(((uint32_t)(x)) << SDMAARM_ONCE_STAT_RCV_SHIFT)) & SDMAARM_ONCE_STAT_RCV_MASK)

#define SDMAARM_ONCE_STAT_PST_MASK               (0xF000U)
#define SDMAARM_ONCE_STAT_PST_SHIFT              (12U)
/*! PST
 *  0b0000..Program
 *  0b0001..Data
 *  0b0010..Change of Flow
 *  0b0011..Change of Flow in Loop
 *  0b0100..Debug
 *  0b0101..Functional Unit
 *  0b0110..Sleep
 *  0b0111..Save
 *  0b1000..Program in Sleep
 *  0b1001..Data in Sleep
 *  0b0010..Change of Flow in Sleep
 *  0b0011..Change Flow in Loop in Sleep
 *  0b1100..Debug in Sleep
 *  0b1101..Functional Unit in Sleep
 *  0b1110..Sleep after Reset
 *  0b1111..Restore
 */
#define SDMAARM_ONCE_STAT_PST(x)                 (((uint32_t)(((uint32_t)(x)) << SDMAARM_ONCE_STAT_PST_SHIFT)) & SDMAARM_ONCE_STAT_PST_MASK)
/*! @} */

/*! @name ONCE_CMD - OnCE Command Register */
/*! @{ */

#define SDMAARM_ONCE_CMD_CMD_MASK                (0xFU)
#define SDMAARM_ONCE_CMD_CMD_SHIFT               (0U)
/*! CMD
 *  0b0000..rstatus
 *  0b0001..dmov
 *  0b0010..exec_once
 *  0b0011..run_core
 *  0b0100..exec_core
 *  0b0101..debug_rqst
 *  0b0110..rbuffer
 */
#define SDMAARM_ONCE_CMD_CMD(x)                  (((uint32_t)(((uint32_t)(x)) << SDMAARM_ONCE_CMD_CMD_SHIFT)) & SDMAARM_ONCE_CMD_CMD_MASK)
/*! @} */

/*! @name ILLINSTADDR - Illegal Instruction Trap Address */
/*! @{ */

#define SDMAARM_ILLINSTADDR_ILLINSTADDR_MASK     (0x3FFFU)
#define SDMAARM_ILLINSTADDR_ILLINSTADDR_SHIFT    (0U)
#define SDMAARM_ILLINSTADDR_ILLINSTADDR(x)       (((uint32_t)(((uint32_t)(x)) << SDMAARM_ILLINSTADDR_ILLINSTADDR_SHIFT)) & SDMAARM_ILLINSTADDR_ILLINSTADDR_MASK)
/*! @} */

/*! @name CHN0ADDR - Channel 0 Boot Address */
/*! @{ */

#define SDMAARM_CHN0ADDR_CHN0ADDR_MASK           (0x3FFFU)
#define SDMAARM_CHN0ADDR_CHN0ADDR_SHIFT          (0U)
#define SDMAARM_CHN0ADDR_CHN0ADDR(x)             (((uint32_t)(((uint32_t)(x)) << SDMAARM_CHN0ADDR_CHN0ADDR_SHIFT)) & SDMAARM_CHN0ADDR_CHN0ADDR_MASK)

#define SDMAARM_CHN0ADDR_SMSZ_MASK               (0x4000U)
#define SDMAARM_CHN0ADDR_SMSZ_SHIFT              (14U)
/*! SMSZ
 *  0b0..24 words per context
 *  0b1..32 words per context
 */
#define SDMAARM_CHN0ADDR_SMSZ(x)                 (((uint32_t)(((uint32_t)(x)) << SDMAARM_CHN0ADDR_SMSZ_SHIFT)) & SDMAARM_CHN0ADDR_SMSZ_MASK)
/*! @} */

/*! @name EVT_MIRROR - DMA Requests */
/*! @{ */

#define SDMAARM_EVT_MIRROR_EVENTS_MASK           (0xFFFFFFFFU)
#define SDMAARM_EVT_MIRROR_EVENTS_SHIFT          (0U)
/*! EVENTS
 *  0b00000000000000000000000000000000..DMA request event not pending
 *  0b00000000000000000000000000000001..DMA request event pending
 */
#define SDMAARM_EVT_MIRROR_EVENTS(x)             (((uint32_t)(((uint32_t)(x)) << SDMAARM_EVT_MIRROR_EVENTS_SHIFT)) & SDMAARM_EVT_MIRROR_EVENTS_MASK)
/*! @} */

/*! @name EVT_MIRROR2 - DMA Requests 2 */
/*! @{ */

#define SDMAARM_EVT_MIRROR2_EVENTS_MASK          (0xFFFFU)
#define SDMAARM_EVT_MIRROR2_EVENTS_SHIFT         (0U)
/*! EVENTS
 *  0b0000000000000000..- DMA request event not pending
 */
#define SDMAARM_EVT_MIRROR2_EVENTS(x)            (((uint32_t)(((uint32_t)(x)) << SDMAARM_EVT_MIRROR2_EVENTS_SHIFT)) & SDMAARM_EVT_MIRROR2_EVENTS_MASK)
/*! @} */

/*! @name XTRIG_CONF1 - Cross-Trigger Events Configuration Register 1 */
/*! @{ */

#define SDMAARM_XTRIG_CONF1_NUM0_MASK            (0x3FU)
#define SDMAARM_XTRIG_CONF1_NUM0_SHIFT           (0U)
#define SDMAARM_XTRIG_CONF1_NUM0(x)              (((uint32_t)(((uint32_t)(x)) << SDMAARM_XTRIG_CONF1_NUM0_SHIFT)) & SDMAARM_XTRIG_CONF1_NUM0_MASK)

#define SDMAARM_XTRIG_CONF1_CNF0_MASK            (0x40U)
#define SDMAARM_XTRIG_CONF1_CNF0_SHIFT           (6U)
/*! CNF0
 *  0b0..channel
 *  0b1..DMA request
 */
#define SDMAARM_XTRIG_CONF1_CNF0(x)              (((uint32_t)(((uint32_t)(x)) << SDMAARM_XTRIG_CONF1_CNF0_SHIFT)) & SDMAARM_XTRIG_CONF1_CNF0_MASK)

#define SDMAARM_XTRIG_CONF1_NUM1_MASK            (0x3F00U)
#define SDMAARM_XTRIG_CONF1_NUM1_SHIFT           (8U)
#define SDMAARM_XTRIG_CONF1_NUM1(x)              (((uint32_t)(((uint32_t)(x)) << SDMAARM_XTRIG_CONF1_NUM1_SHIFT)) & SDMAARM_XTRIG_CONF1_NUM1_MASK)

#define SDMAARM_XTRIG_CONF1_CNF1_MASK            (0x4000U)
#define SDMAARM_XTRIG_CONF1_CNF1_SHIFT           (14U)
/*! CNF1
 *  0b0..channel
 *  0b1..DMA request
 */
#define SDMAARM_XTRIG_CONF1_CNF1(x)              (((uint32_t)(((uint32_t)(x)) << SDMAARM_XTRIG_CONF1_CNF1_SHIFT)) & SDMAARM_XTRIG_CONF1_CNF1_MASK)

#define SDMAARM_XTRIG_CONF1_NUM2_MASK            (0x3F0000U)
#define SDMAARM_XTRIG_CONF1_NUM2_SHIFT           (16U)
#define SDMAARM_XTRIG_CONF1_NUM2(x)              (((uint32_t)(((uint32_t)(x)) << SDMAARM_XTRIG_CONF1_NUM2_SHIFT)) & SDMAARM_XTRIG_CONF1_NUM2_MASK)

#define SDMAARM_XTRIG_CONF1_CNF2_MASK            (0x400000U)
#define SDMAARM_XTRIG_CONF1_CNF2_SHIFT           (22U)
/*! CNF2
 *  0b0..channel
 *  0b1..DMA request
 */
#define SDMAARM_XTRIG_CONF1_CNF2(x)              (((uint32_t)(((uint32_t)(x)) << SDMAARM_XTRIG_CONF1_CNF2_SHIFT)) & SDMAARM_XTRIG_CONF1_CNF2_MASK)

#define SDMAARM_XTRIG_CONF1_NUM3_MASK            (0x3F000000U)
#define SDMAARM_XTRIG_CONF1_NUM3_SHIFT           (24U)
#define SDMAARM_XTRIG_CONF1_NUM3(x)              (((uint32_t)(((uint32_t)(x)) << SDMAARM_XTRIG_CONF1_NUM3_SHIFT)) & SDMAARM_XTRIG_CONF1_NUM3_MASK)

#define SDMAARM_XTRIG_CONF1_CNF3_MASK            (0x40000000U)
#define SDMAARM_XTRIG_CONF1_CNF3_SHIFT           (30U)
/*! CNF3
 *  0b0..channel
 *  0b1..DMA request
 */
#define SDMAARM_XTRIG_CONF1_CNF3(x)              (((uint32_t)(((uint32_t)(x)) << SDMAARM_XTRIG_CONF1_CNF3_SHIFT)) & SDMAARM_XTRIG_CONF1_CNF3_MASK)
/*! @} */

/*! @name XTRIG_CONF2 - Cross-Trigger Events Configuration Register 2 */
/*! @{ */

#define SDMAARM_XTRIG_CONF2_NUM4_MASK            (0x3FU)
#define SDMAARM_XTRIG_CONF2_NUM4_SHIFT           (0U)
#define SDMAARM_XTRIG_CONF2_NUM4(x)              (((uint32_t)(((uint32_t)(x)) << SDMAARM_XTRIG_CONF2_NUM4_SHIFT)) & SDMAARM_XTRIG_CONF2_NUM4_MASK)

#define SDMAARM_XTRIG_CONF2_CNF4_MASK            (0x40U)
#define SDMAARM_XTRIG_CONF2_CNF4_SHIFT           (6U)
/*! CNF4
 *  0b0..channel
 *  0b1..DMA request
 */
#define SDMAARM_XTRIG_CONF2_CNF4(x)              (((uint32_t)(((uint32_t)(x)) << SDMAARM_XTRIG_CONF2_CNF4_SHIFT)) & SDMAARM_XTRIG_CONF2_CNF4_MASK)

#define SDMAARM_XTRIG_CONF2_NUM5_MASK            (0x3F00U)
#define SDMAARM_XTRIG_CONF2_NUM5_SHIFT           (8U)
#define SDMAARM_XTRIG_CONF2_NUM5(x)              (((uint32_t)(((uint32_t)(x)) << SDMAARM_XTRIG_CONF2_NUM5_SHIFT)) & SDMAARM_XTRIG_CONF2_NUM5_MASK)

#define SDMAARM_XTRIG_CONF2_CNF5_MASK            (0x4000U)
#define SDMAARM_XTRIG_CONF2_CNF5_SHIFT           (14U)
/*! CNF5
 *  0b0..channel
 *  0b1..DMA request
 */
#define SDMAARM_XTRIG_CONF2_CNF5(x)              (((uint32_t)(((uint32_t)(x)) << SDMAARM_XTRIG_CONF2_CNF5_SHIFT)) & SDMAARM_XTRIG_CONF2_CNF5_MASK)

#define SDMAARM_XTRIG_CONF2_NUM6_MASK            (0x3F0000U)
#define SDMAARM_XTRIG_CONF2_NUM6_SHIFT           (16U)
#define SDMAARM_XTRIG_CONF2_NUM6(x)              (((uint32_t)(((uint32_t)(x)) << SDMAARM_XTRIG_CONF2_NUM6_SHIFT)) & SDMAARM_XTRIG_CONF2_NUM6_MASK)

#define SDMAARM_XTRIG_CONF2_CNF6_MASK            (0x400000U)
#define SDMAARM_XTRIG_CONF2_CNF6_SHIFT           (22U)
/*! CNF6
 *  0b0..channel
 *  0b1..DMA request
 */
#define SDMAARM_XTRIG_CONF2_CNF6(x)              (((uint32_t)(((uint32_t)(x)) << SDMAARM_XTRIG_CONF2_CNF6_SHIFT)) & SDMAARM_XTRIG_CONF2_CNF6_MASK)

#define SDMAARM_XTRIG_CONF2_NUM7_MASK            (0x3F000000U)
#define SDMAARM_XTRIG_CONF2_NUM7_SHIFT           (24U)
#define SDMAARM_XTRIG_CONF2_NUM7(x)              (((uint32_t)(((uint32_t)(x)) << SDMAARM_XTRIG_CONF2_NUM7_SHIFT)) & SDMAARM_XTRIG_CONF2_NUM7_MASK)

#define SDMAARM_XTRIG_CONF2_CNF7_MASK            (0x40000000U)
#define SDMAARM_XTRIG_CONF2_CNF7_SHIFT           (30U)
/*! CNF7
 *  0b0..channel
 *  0b1..DMA request
 */
#define SDMAARM_XTRIG_CONF2_CNF7(x)              (((uint32_t)(((uint32_t)(x)) << SDMAARM_XTRIG_CONF2_CNF7_SHIFT)) & SDMAARM_XTRIG_CONF2_CNF7_MASK)
/*! @} */

/*! @name SDMA_CHNPRI - Channel Priority Registers */
/*! @{ */

#define SDMAARM_SDMA_CHNPRI_CHNPRIn_MASK         (0x7U)
#define SDMAARM_SDMA_CHNPRI_CHNPRIn_SHIFT        (0U)
#define SDMAARM_SDMA_CHNPRI_CHNPRIn(x)           (((uint32_t)(((uint32_t)(x)) << SDMAARM_SDMA_CHNPRI_CHNPRIn_SHIFT)) & SDMAARM_SDMA_CHNPRI_CHNPRIn_MASK)
/*! @} */

/* The count of SDMAARM_SDMA_CHNPRI */
#define SDMAARM_SDMA_CHNPRI_COUNT                (32U)

/*! @name CHNENBL - Channel Enable RAM */
/*! @{ */

#define SDMAARM_CHNENBL_ENBLn_MASK               (0xFFFFFFFFU)
#define SDMAARM_CHNENBL_ENBLn_SHIFT              (0U)
#define SDMAARM_CHNENBL_ENBLn(x)                 (((uint32_t)(((uint32_t)(x)) << SDMAARM_CHNENBL_ENBLn_SHIFT)) & SDMAARM_CHNENBL_ENBLn_MASK)
/*! @} */

/* The count of SDMAARM_CHNENBL */
#define SDMAARM_CHNENBL_COUNT                    (48U)

/*! @name DONE0_CONFIG - SDMA DONE0 Configuration */
/*! @{ */

#define SDMAARM_DONE0_CONFIG_CH_SEL0_MASK        (0x1FU)
#define SDMAARM_DONE0_CONFIG_CH_SEL0_SHIFT       (0U)
#define SDMAARM_DONE0_CONFIG_CH_SEL0(x)          (((uint32_t)(((uint32_t)(x)) << SDMAARM_DONE0_CONFIG_CH_SEL0_SHIFT)) & SDMAARM_DONE0_CONFIG_CH_SEL0_MASK)

#define SDMAARM_DONE0_CONFIG_SW_DONE_DIS0_MASK   (0x40U)
#define SDMAARM_DONE0_CONFIG_SW_DONE_DIS0_SHIFT  (6U)
/*! SW_DONE_DIS0
 *  0b0..Enable
 *  0b1..Disable
 */
#define SDMAARM_DONE0_CONFIG_SW_DONE_DIS0(x)     (((uint32_t)(((uint32_t)(x)) << SDMAARM_DONE0_CONFIG_SW_DONE_DIS0_SHIFT)) & SDMAARM_DONE0_CONFIG_SW_DONE_DIS0_MASK)

#define SDMAARM_DONE0_CONFIG_DONE_SEL0_MASK      (0x80U)
#define SDMAARM_DONE0_CONFIG_DONE_SEL0_SHIFT     (7U)
/*! DONE_SEL0
 *  0b0..HW
 *  0b1..SW
 */
#define SDMAARM_DONE0_CONFIG_DONE_SEL0(x)        (((uint32_t)(((uint32_t)(x)) << SDMAARM_DONE0_CONFIG_DONE_SEL0_SHIFT)) & SDMAARM_DONE0_CONFIG_DONE_SEL0_MASK)

#define SDMAARM_DONE0_CONFIG_CH_SEL1_MASK        (0x1F00U)
#define SDMAARM_DONE0_CONFIG_CH_SEL1_SHIFT       (8U)
#define SDMAARM_DONE0_CONFIG_CH_SEL1(x)          (((uint32_t)(((uint32_t)(x)) << SDMAARM_DONE0_CONFIG_CH_SEL1_SHIFT)) & SDMAARM_DONE0_CONFIG_CH_SEL1_MASK)

#define SDMAARM_DONE0_CONFIG_SW_DONE_DIS1_MASK   (0x4000U)
#define SDMAARM_DONE0_CONFIG_SW_DONE_DIS1_SHIFT  (14U)
/*! SW_DONE_DIS1
 *  0b0..Enable
 *  0b1..Disable
 */
#define SDMAARM_DONE0_CONFIG_SW_DONE_DIS1(x)     (((uint32_t)(((uint32_t)(x)) << SDMAARM_DONE0_CONFIG_SW_DONE_DIS1_SHIFT)) & SDMAARM_DONE0_CONFIG_SW_DONE_DIS1_MASK)

#define SDMAARM_DONE0_CONFIG_DONE_SEL1_MASK      (0x8000U)
#define SDMAARM_DONE0_CONFIG_DONE_SEL1_SHIFT     (15U)
/*! DONE_SEL1
 *  0b0..HW
 *  0b1..SW
 */
#define SDMAARM_DONE0_CONFIG_DONE_SEL1(x)        (((uint32_t)(((uint32_t)(x)) << SDMAARM_DONE0_CONFIG_DONE_SEL1_SHIFT)) & SDMAARM_DONE0_CONFIG_DONE_SEL1_MASK)

#define SDMAARM_DONE0_CONFIG_CH_SEL2_MASK        (0x1F0000U)
#define SDMAARM_DONE0_CONFIG_CH_SEL2_SHIFT       (16U)
#define SDMAARM_DONE0_CONFIG_CH_SEL2(x)          (((uint32_t)(((uint32_t)(x)) << SDMAARM_DONE0_CONFIG_CH_SEL2_SHIFT)) & SDMAARM_DONE0_CONFIG_CH_SEL2_MASK)

#define SDMAARM_DONE0_CONFIG_SW_DONE_DIS2_MASK   (0x400000U)
#define SDMAARM_DONE0_CONFIG_SW_DONE_DIS2_SHIFT  (22U)
/*! SW_DONE_DIS2
 *  0b0..Enable
 *  0b1..Disable
 */
#define SDMAARM_DONE0_CONFIG_SW_DONE_DIS2(x)     (((uint32_t)(((uint32_t)(x)) << SDMAARM_DONE0_CONFIG_SW_DONE_DIS2_SHIFT)) & SDMAARM_DONE0_CONFIG_SW_DONE_DIS2_MASK)

#define SDMAARM_DONE0_CONFIG_DONE_SEL2_MASK      (0x800000U)
#define SDMAARM_DONE0_CONFIG_DONE_SEL2_SHIFT     (23U)
/*! DONE_SEL2
 *  0b0..HW
 *  0b1..SW
 */
#define SDMAARM_DONE0_CONFIG_DONE_SEL2(x)        (((uint32_t)(((uint32_t)(x)) << SDMAARM_DONE0_CONFIG_DONE_SEL2_SHIFT)) & SDMAARM_DONE0_CONFIG_DONE_SEL2_MASK)

#define SDMAARM_DONE0_CONFIG_CH_SEL3_MASK        (0x1F000000U)
#define SDMAARM_DONE0_CONFIG_CH_SEL3_SHIFT       (24U)
#define SDMAARM_DONE0_CONFIG_CH_SEL3(x)          (((uint32_t)(((uint32_t)(x)) << SDMAARM_DONE0_CONFIG_CH_SEL3_SHIFT)) & SDMAARM_DONE0_CONFIG_CH_SEL3_MASK)

#define SDMAARM_DONE0_CONFIG_SW_DONE_DIS3_MASK   (0x40000000U)
#define SDMAARM_DONE0_CONFIG_SW_DONE_DIS3_SHIFT  (30U)
/*! SW_DONE_DIS3
 *  0b0..Enable
 *  0b1..Disable
 */
#define SDMAARM_DONE0_CONFIG_SW_DONE_DIS3(x)     (((uint32_t)(((uint32_t)(x)) << SDMAARM_DONE0_CONFIG_SW_DONE_DIS3_SHIFT)) & SDMAARM_DONE0_CONFIG_SW_DONE_DIS3_MASK)

#define SDMAARM_DONE0_CONFIG_DONE_SEL3_MASK      (0x80000000U)
#define SDMAARM_DONE0_CONFIG_DONE_SEL3_SHIFT     (31U)
/*! DONE_SEL3
 *  0b0..HW
 *  0b1..SW
 */
#define SDMAARM_DONE0_CONFIG_DONE_SEL3(x)        (((uint32_t)(((uint32_t)(x)) << SDMAARM_DONE0_CONFIG_DONE_SEL3_SHIFT)) & SDMAARM_DONE0_CONFIG_DONE_SEL3_MASK)
/*! @} */

/*! @name DONE1_CONFIG - SDMA DONE1 Configuration */
/*! @{ */

#define SDMAARM_DONE1_CONFIG_CH_SEL4_MASK        (0x1FU)
#define SDMAARM_DONE1_CONFIG_CH_SEL4_SHIFT       (0U)
#define SDMAARM_DONE1_CONFIG_CH_SEL4(x)          (((uint32_t)(((uint32_t)(x)) << SDMAARM_DONE1_CONFIG_CH_SEL4_SHIFT)) & SDMAARM_DONE1_CONFIG_CH_SEL4_MASK)

#define SDMAARM_DONE1_CONFIG_SW_DONE_DIS4_MASK   (0x40U)
#define SDMAARM_DONE1_CONFIG_SW_DONE_DIS4_SHIFT  (6U)
/*! SW_DONE_DIS4
 *  0b0..Enable
 *  0b1..Disable
 */
#define SDMAARM_DONE1_CONFIG_SW_DONE_DIS4(x)     (((uint32_t)(((uint32_t)(x)) << SDMAARM_DONE1_CONFIG_SW_DONE_DIS4_SHIFT)) & SDMAARM_DONE1_CONFIG_SW_DONE_DIS4_MASK)

#define SDMAARM_DONE1_CONFIG_DONE_SEL4_MASK      (0x80U)
#define SDMAARM_DONE1_CONFIG_DONE_SEL4_SHIFT     (7U)
/*! DONE_SEL4
 *  0b0..HW
 *  0b1..SW
 */
#define SDMAARM_DONE1_CONFIG_DONE_SEL4(x)        (((uint32_t)(((uint32_t)(x)) << SDMAARM_DONE1_CONFIG_DONE_SEL4_SHIFT)) & SDMAARM_DONE1_CONFIG_DONE_SEL4_MASK)

#define SDMAARM_DONE1_CONFIG_CH_SEL5_MASK        (0x1F00U)
#define SDMAARM_DONE1_CONFIG_CH_SEL5_SHIFT       (8U)
#define SDMAARM_DONE1_CONFIG_CH_SEL5(x)          (((uint32_t)(((uint32_t)(x)) << SDMAARM_DONE1_CONFIG_CH_SEL5_SHIFT)) & SDMAARM_DONE1_CONFIG_CH_SEL5_MASK)

#define SDMAARM_DONE1_CONFIG_SW_DONE_DIS5_MASK   (0x4000U)
#define SDMAARM_DONE1_CONFIG_SW_DONE_DIS5_SHIFT  (14U)
/*! SW_DONE_DIS5
 *  0b0..Enable
 *  0b1..Disable
 */
#define SDMAARM_DONE1_CONFIG_SW_DONE_DIS5(x)     (((uint32_t)(((uint32_t)(x)) << SDMAARM_DONE1_CONFIG_SW_DONE_DIS5_SHIFT)) & SDMAARM_DONE1_CONFIG_SW_DONE_DIS5_MASK)

#define SDMAARM_DONE1_CONFIG_DONE_SEL5_MASK      (0x8000U)
#define SDMAARM_DONE1_CONFIG_DONE_SEL5_SHIFT     (15U)
/*! DONE_SEL5
 *  0b0..HW
 *  0b1..SW
 */
#define SDMAARM_DONE1_CONFIG_DONE_SEL5(x)        (((uint32_t)(((uint32_t)(x)) << SDMAARM_DONE1_CONFIG_DONE_SEL5_SHIFT)) & SDMAARM_DONE1_CONFIG_DONE_SEL5_MASK)

#define SDMAARM_DONE1_CONFIG_CH_SEL6_MASK        (0x1F0000U)
#define SDMAARM_DONE1_CONFIG_CH_SEL6_SHIFT       (16U)
#define SDMAARM_DONE1_CONFIG_CH_SEL6(x)          (((uint32_t)(((uint32_t)(x)) << SDMAARM_DONE1_CONFIG_CH_SEL6_SHIFT)) & SDMAARM_DONE1_CONFIG_CH_SEL6_MASK)

#define SDMAARM_DONE1_CONFIG_SW_DONE_DIS6_MASK   (0x400000U)
#define SDMAARM_DONE1_CONFIG_SW_DONE_DIS6_SHIFT  (22U)
/*! SW_DONE_DIS6
 *  0b0..Enable
 *  0b1..Disable
 */
#define SDMAARM_DONE1_CONFIG_SW_DONE_DIS6(x)     (((uint32_t)(((uint32_t)(x)) << SDMAARM_DONE1_CONFIG_SW_DONE_DIS6_SHIFT)) & SDMAARM_DONE1_CONFIG_SW_DONE_DIS6_MASK)

#define SDMAARM_DONE1_CONFIG_DONE_SEL6_MASK      (0x800000U)
#define SDMAARM_DONE1_CONFIG_DONE_SEL6_SHIFT     (23U)
/*! DONE_SEL6
 *  0b0..HW
 *  0b1..SW
 */
#define SDMAARM_DONE1_CONFIG_DONE_SEL6(x)        (((uint32_t)(((uint32_t)(x)) << SDMAARM_DONE1_CONFIG_DONE_SEL6_SHIFT)) & SDMAARM_DONE1_CONFIG_DONE_SEL6_MASK)

#define SDMAARM_DONE1_CONFIG_CH_SEL7_MASK        (0x1F000000U)
#define SDMAARM_DONE1_CONFIG_CH_SEL7_SHIFT       (24U)
#define SDMAARM_DONE1_CONFIG_CH_SEL7(x)          (((uint32_t)(((uint32_t)(x)) << SDMAARM_DONE1_CONFIG_CH_SEL7_SHIFT)) & SDMAARM_DONE1_CONFIG_CH_SEL7_MASK)

#define SDMAARM_DONE1_CONFIG_SW_DONE_DIS7_MASK   (0x40000000U)
#define SDMAARM_DONE1_CONFIG_SW_DONE_DIS7_SHIFT  (30U)
/*! SW_DONE_DIS7
 *  0b0..Enable
 *  0b1..Disable
 */
#define SDMAARM_DONE1_CONFIG_SW_DONE_DIS7(x)     (((uint32_t)(((uint32_t)(x)) << SDMAARM_DONE1_CONFIG_SW_DONE_DIS7_SHIFT)) & SDMAARM_DONE1_CONFIG_SW_DONE_DIS7_MASK)

#define SDMAARM_DONE1_CONFIG_DONE_SEL7_MASK      (0x80000000U)
#define SDMAARM_DONE1_CONFIG_DONE_SEL7_SHIFT     (31U)
/*! DONE_SEL7
 *  0b0..HW
 *  0b1..SW
 */
#define SDMAARM_DONE1_CONFIG_DONE_SEL7(x)        (((uint32_t)(((uint32_t)(x)) << SDMAARM_DONE1_CONFIG_DONE_SEL7_SHIFT)) & SDMAARM_DONE1_CONFIG_DONE_SEL7_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group SDMAARM_Register_Masks */


/* SDMAARM - Peripheral instance base addresses */
/** Peripheral SDMAARM1 base address */
#define SDMAARM1_BASE                            (0x30BD0000u)
/** Peripheral SDMAARM1 base pointer */
#define SDMAARM1                                 ((SDMAARM_Type *)SDMAARM1_BASE)
/** Peripheral SDMAARM2 base address */
#define SDMAARM2_BASE                            (0x302C0000u)
/** Peripheral SDMAARM2 base pointer */
#define SDMAARM2                                 ((SDMAARM_Type *)SDMAARM2_BASE)
/** Peripheral SDMAARM3 base address */
#define SDMAARM3_BASE                            (0x302B0000u)
/** Peripheral SDMAARM3 base pointer */
#define SDMAARM3                                 ((SDMAARM_Type *)SDMAARM3_BASE)
/** Array initializer of SDMAARM peripheral base addresses */
#define SDMAARM_BASE_ADDRS                       { SDMAARM1_BASE, SDMAARM2_BASE, SDMAARM3_BASE }
/** Array initializer of SDMAARM peripheral base pointers */
#define SDMAARM_BASE_PTRS                        { SDMAARM1, SDMAARM2, SDMAARM3 }
/** Interrupt vectors for the SDMAARM peripheral type */
#define SDMAARM_IRQS      { SDMA1_IRQn, SDMA2_IRQn, SDMA3_IRQn }
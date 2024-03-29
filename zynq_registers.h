/*
 * Zynq_registers.h
 *
 *  Originally created on: 28.5.2018
 *  Modified on: 11.9.2021
 *
 *  Reference: https://www.xilinx.com/support/documentation/user_guides/ug585-Zynq-7000-TRM.pdf
 *
 */

#include "xil_types.h"

#ifndef SRC_ZYNQ_REGISTERS_H_
#define SRC_ZYNQ_REGISTERS_H_

/*
 * POINTER_TO_REGISTER is a macro that simplifies the creation of memory address pointers
 * by automatically casting the supplied register as a volatile unsigned 32-bit integer pointer.
 */

#define POINTER_TO_REGISTER(REG)		( *((volatile u32*)(REG))) //u32 (xil_types.h) data type is declared as uint32_t (stdint.h)

/*
 * The following register addresses have been obtained from system.hdf
 * Each AXI GPIO data register has two channels but with LEDs,
 * only the first channel has anything useful.
 * */

#define AXI_LED_BASE_ADDRESS			0x41220000
#define AXI_BTNSW_BASE_ADDRESS			0x41210000
/* This one can be used to control an AXI GPIO register, that's
 * connected to six different pins in the Pmod connector JD */
#define AXI_PMOD_GPIO_BASE_ADDRESS		0x41240000

#define AXI_LED_DATA_ADDRESS			( AXI_LED_BASE_ADDRESS + 0x00000000 )
#define AXI_LED_TRI_ADDRESS				( AXI_LED_BASE_ADDRESS + 0x00000004 )

#define AXI_SW_DATA_ADDRESS				( AXI_BTNSW_BASE_ADDRESS + 0x00000000 )
#define AXI_SW_TRI_ADDRESS				( AXI_BTNSW_BASE_ADDRESS + 0x00000004 )
#define AXI_BTN_DATA_ADDRESS			( AXI_BTNSW_BASE_ADDRESS + 0x00000008 )
#define AXI_BTN_TRI_ADDRESS				( AXI_BTNSW_BASE_ADDRESS + 0x0000000C )

#define AXI_PMOD_GPIO_DATA_ADDRESS		(AXI_PMOD_GPIO_BASE_ADDRESS + 0x00000000)
#define AXI_PMOD_GPIO_TRI_ADDRESS		(AXI_PMOD_GPIO_BASE_ADDRESS + 0x00000004)

/* The following registers are used to control the on-board LEDs.
 * Tri register sets the data register as either input or output.
 * The tri register takes 0 to mean output and 1 to mean input.
 * When tri register is set as output, the data register
 * can be used to turn the on-board LEDs on/off.
 * */

#define AXI_LED_DATA					( POINTER_TO_REGISTER(AXI_LED_DATA_ADDRESS) )
#define AXI_LED_TRI						( POINTER_TO_REGISTER(AXI_LED_TRI_ADDRESS) )

/*
 * The following registers are used to control the on-board
 * switches and buttons. The tri registers need to be set
 * as inputs, to read their state.
 * */

#define AXI_SW_DATA						( POINTER_TO_REGISTER(AXI_SW_DATA_ADDRESS) )
#define AXI_SW_TRI						( POINTER_TO_REGISTER(AXI_SW_TRI_ADDRESS) )
#define AXI_BTN_DATA					( POINTER_TO_REGISTER(AXI_BTN_DATA_ADDRESS) )
#define AXI_BTN_TRI						( POINTER_TO_REGISTER(AXI_BTN_TRI_ADDRESS) )

#define AXI_PMOD_GPIO_DATA				( POINTER_TO_REGISTER(AXI_PMOD_GPIO_DATA_ADDRESS ))
#define AXI_PMOD_GPIO_TRI				( POINTER_TO_REGISTER(AXI_PMOD_GPIO_TRI_ADDRESS ))

/* The following macro creates pointers specifically to the TTC registers.
 * The TMR parameter designates which of the two TTC blocks is used;
 * notice how it changes from 0 to 1 when creating pointers to
 * the registers associated with TTC1.
 * */
#define POINTER_TO_TTC_REGISTER(TMR, REG)( *((volatile u32*)(XPS_TTC##TMR##_BASEADDR + REG)))

/* TTC registers can be found on page 1734 of the TRM. */
#define TTC0_CLK_CNTRL					( POINTER_TO_TTC_REGISTER(0, XTTCPS_CLK_CNTRL_OFFSET) )
#define TTC0_CLK_CNTRL2					( POINTER_TO_TTC_REGISTER(0, 0x00000004) )
#define TTC0_CLK_CNTRL3					( POINTER_TO_TTC_REGISTER(0, 0x00000008) )
#define TTC0_CNT_CNTRL					( POINTER_TO_TTC_REGISTER(0, XTTCPS_CNT_CNTRL_OFFSET) )
#define TTC0_CNT_CNTRL2					( POINTER_TO_TTC_REGISTER(0, 0x00000010) )
#define TTC0_CNT_CNTRL3					( POINTER_TO_TTC_REGISTER(0, 0x00000014) )
#define TTC0_COUNT_VALUE				( POINTER_TO_TTC_REGISTER(0, XTTCPS_COUNT_VALUE_OFFSET) )
#define TTC0_COUNT_VALUE2				( POINTER_TO_TTC_REGISTER(0, 0x0000001C) )
#define TTC0_COUNT_VALUE3				( POINTER_TO_TTC_REGISTER(0, 0x00000020) )
#define TTC0_INTERVAL_VAL				( POINTER_TO_TTC_REGISTER(0, XTTCPS_INTERVAL_VAL_OFFSET) )
#define TTC0_INTERVAL_VAL2				( POINTER_TO_TTC_REGISTER(0, 0x00000028) )
#define TTC0_INTERVAL_VAL3				( POINTER_TO_TTC_REGISTER(0, 0x0000002C) )
#define TTC0_MATCH_0					( POINTER_TO_TTC_REGISTER(0, XTTCPS_MATCH_0_OFFSET) )
#define TTC0_MATCH_1_COUNTER_2			( POINTER_TO_TTC_REGISTER(0, 0x00000034) )
#define TTC0_MATCH_1_COUNTER_3			( POINTER_TO_TTC_REGISTER(0, 0x00000038) )
#define TTC0_MATCH_1					( POINTER_TO_TTC_REGISTER(0, XTTCPS_MATCH_1_OFFSET) )
#define TTC0_MATCH_2_COUNTER_2			( POINTER_TO_TTC_REGISTER(0, 0x00000040) )
#define TTC0_MATCH_2_COUNTER_3			( POINTER_TO_TTC_REGISTER(0, 0x00000044) )
#define TTC0_MATCH_2					( POINTER_TO_TTC_REGISTER(0, XTTCPS_MATCH_2_OFFSET) )
#define TTC0_MATCH_3_COUNTER_2			( POINTER_TO_TTC_REGISTER(0, 0x0000004C) )
#define TTC0_MATCH_3_COUNTER_3			( POINTER_TO_TTC_REGISTER(0, 0x00000050) )
#define TTC0_ISR						( POINTER_TO_TTC_REGISTER(0, XTTCPS_ISR_OFFSET) )
#define TTC0_ISR_2						( POINTER_TO_TTC_REGISTER(0, 0x00000058))
#define TTC0_ISR_3						( POINTER_TO_TTC_REGISTER(0, 0x0000005C))
#define TTC0_IER						( POINTER_TO_TTC_REGISTER(0, XTTCPS_IER_OFFSET) )
#define TTC0_IER_2						( POINTER_TO_TTC_REGISTER(0, 0x00000064) )
#define TTC0_IER_3						( POINTER_TO_TTC_REGISTER(0, 0x00000068) )

#define TTC1_CLK_CNTRL					( POINTER_TO_TTC_REGISTER(1, XTTCPS_CLK_CNTRL_OFFSET) )
#define TTC1_CLK_CNTRL2					( POINTER_TO_TTC_REGISTER(1, 0x00000004) )
#define TTC1_CLK_CNTRL3					( POINTER_TO_TTC_REGISTER(1, 0x00000008) )
#define TTC1_CNT_CNTRL					( POINTER_TO_TTC_REGISTER(1, XTTCPS_CNT_CNTRL_OFFSET) )
#define TTC1_CNT_CNTRL2					( POINTER_TO_TTC_REGISTER(1, 0x00000010) )
#define TTC1_CNT_CNTRL3					( POINTER_TO_TTC_REGISTER(1, 0x00000014) )
#define TTC1_COUNT_VALUE				( POINTER_TO_TTC_REGISTER(1, XTTCPS_COUNT_VALUE_OFFSET) )
#define TTC1_COUNT_VALUE2				( POINTER_TO_TTC_REGISTER(1, 0x0000001C) )
#define TTC1_COUNT_VALUE3				( POINTER_TO_TTC_REGISTER(1, 0x00000020) )
#define TTC1_INTERVAL_VAL				( POINTER_TO_TTC_REGISTER(1, XTTCPS_INTERVAL_VAL_OFFSET) )
#define TTC1_INTERVAL_VAL2				( POINTER_TO_TTC_REGISTER(1, 0x00000028) )
#define TTC1_INTERVAL_VAL3				( POINTER_TO_TTC_REGISTER(1, 0x0000002C) )
#define TTC1_MATCH_0					( POINTER_TO_TTC_REGISTER(1, XTTCPS_MATCH_0_OFFSET) )
#define TTC1_MATCH_1_COUNTER_2			( POINTER_TO_TTC_REGISTER(1, 0x00000034) )
#define TTC1_MATCH_1_COUNTER_3			( POINTER_TO_TTC_REGISTER(1, 0x00000038) )
#define TTC1_MATCH_1					( POINTER_TO_TTC_REGISTER(1, XTTCPS_MATCH_1_OFFSET) )
#define TTC1_MATCH_2_COUNTER_2			( POINTER_TO_TTC_REGISTER(1, 0x00000040) )
#define TTC1_MATCH_2_COUNTER_3			( POINTER_TO_TTC_REGISTER(1, 0x00000044) )
#define TTC1_MATCH_2					( POINTER_TO_TTC_REGISTER(1, XTTCPS_MATCH_2_OFFSET) )
#define TTC1_MATCH_3_COUNTER_2			( POINTER_TO_TTC_REGISTER(1, 0x0000004C) )
#define TTC1_MATCH_3_COUNTER_3			( POINTER_TO_TTC_REGISTER(1, 0x00000050) )
#define TTC1_ISR						( POINTER_TO_TTC_REGISTER(1, XTTCPS_ISR_OFFSET) )
#define TTC1_ISR_2						( POINTER_TO_TTC_REGISTER(1, 0x00000058))
#define TTC1_ISR_3						( POINTER_TO_TTC_REGISTER(1, 0x0000005C))
#define TTC1_IER						( POINTER_TO_TTC_REGISTER(1, XTTCPS_IER_OFFSET) )
#define TTC1_IER_2						( POINTER_TO_TTC_REGISTER(1, 0x00000064) )
#define TTC1_IER_3						( POINTER_TO_TTC_REGISTER(1, 0x00000068) )

/* Used to enable different interrupts.
 * The interrupt IDs for shared peripheral interrupts start from 32 and then work
 * their way up. IDs between 32-63 go in ICDISER1 and IDs above that go in ICDISER2.
 * The listing for different IDs can be found from page 232 of the TRM.
 * */
#define ICDISER1_ADDRESS				( XPS_SCU_PERIPH_BASE + 0x00001104 )
#define ICDISER1						( POINTER_TO_REGISTER(ICDISER1_ADDRESS) )
#define ICDISER2_ADDRESS				( XPS_SCU_PERIPH_BASE + 0x00001108 )
#define ICDISER2						( POINTER_TO_REGISTER(ICDISER2_ADDRESS) )

#define XDCFG_UNLOCK_OFFSET_ADDRESS 	( XDCFG_BASE_ADDRESS + XADCPS_UNLK_OFFSET )

#define XDCFG_UNLOCK_OFFSET				( POINTER_TO_REGISTER(XADCIF_CFG_ADDRESS) )

#define XADCIF_CFG_ADDRESS				( XPAR_PS7_XADC_0_BASEADDR + XADCPS_CFG_OFFSET )
#define XADCIF_INT_STS_ADDRESS			( XPAR_PS7_XADC_0_BASEADDR + XADCPS_INT_STS_OFFSET )
#define XADCIF_INT_MASK_ADDRESS			( XPAR_PS7_XADC_0_BASEADDR + XADCPS_INT_MASK_OFFSET )
#define XADCIF_MSTS_ADDRESS				( XPAR_PS7_XADC_0_BASEADDR + XADCPS_MSTS_OFFSET )
#define XADCIF_CMDFIFO_ADDRESS			( XPAR_PS7_XADC_0_BASEADDR + XADCPS_CMDFIFO_OFFSET )
#define XADCIF_RDFIFO_ADDRESS			( XPAR_PS7_XADC_0_BASEADDR + XADCPS_RDFIFO_OFFSET )
#define XADCIF_MCTL_ADDRESS				( XPAR_PS7_XADC_0_BASEADDR + XADCPS_MCTL_OFFSET )

#define XADCIF_CFG						( POINTER_TO_REGISTER(XADCIF_CFG_ADDRESS) )
#define XADCIF_INT_STS					( POINTER_TO_REGISTER(XADCIF_INT_STS_ADDRESS) )
#define XADCIF_INT_MASK					( POINTER_TO_REGISTER(XADCIF_INT_MASK_ADDRESS) )
#define XADCIF_MSTS						( POINTER_TO_REGISTER(XADCIF_MSTS_ADDRESS) )
#define XADCIF_CMDFIFO					( POINTER_TO_REGISTER(XADCIF_CMDFIFO_ADDRESS) )
#define XADCIF_RDFIFO					( POINTER_TO_REGISTER(XADCIF_RDFIFO_ADDRESS) )
#define XADCIF_MCTL						( POINTER_TO_REGISTER(XADCIF_MCTL_ADDRESS) )

/* UART registers can be found on page 1755 of the TRM. */
#define UART_BASE 						XPS_UART1_BASEADDR // Base Address
#define UART_CTRL 						POINTER_TO_REGISTER(UART_BASE + XUARTPS_CR_OFFSET) // Control Register
#define UART_MODE 						POINTER_TO_REGISTER(UART_BASE + XUARTPS_MR_OFFSET) // Mode Register
#define UART_BAUD_GEN 					POINTER_TO_REGISTER(UART_BASE + XUARTPS_BAUDGEN_OFFSET) // Baud Rate Generator "CD"
#define UART_BAUD_DIV 					POINTER_TO_REGISTER(UART_BASE + XUARTPS_BAUDDIV_OFFSET) // Baud Rate Divider "BDIV"
#define UART_FIFO 						POINTER_TO_REGISTER(UART_BASE + XUARTPS_FIFO_OFFSET) // FIFO
#define UART_STATUS 					POINTER_TO_REGISTER(UART_BASE + XUARTPS_SR_OFFSET) // Channel Status

#define UART_IER     					POINTER_TO_REGISTER(UART_BASE + XUARTPS_IER_OFFSET) // Interrupt Enable Register
#define UART_RXWM     					POINTER_TO_REGISTER(UART_BASE + XUARTPS_RXWM_OFFSET) // Receiver FIFO Trigger Level Register
#define UART_ISR     					POINTER_TO_REGISTER(UART_BASE + XUARTPS_ISR_OFFSET) // Interrupt Status Register
#define UART_IMR     					POINTER_TO_REGISTER(UART_BASE + XUARTPS_IMR_OFFSET) // Interrupt Status Register
#define UART_IDR     					POINTER_TO_REGISTER(UART_BASE + XUARTPS_IDR_OFFSET) // Interrupt Status Register



#endif /* SRC_ZYNQ_REGISTERS_H_ */


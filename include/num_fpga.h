#ifndef _NUM_FPGA_H
#define _NUM_FPGA_H

#define ADR_BASE_CPUBUS                0x48000000       

//------------------------------------------------------------------------
// CPU interface sub-block base addresses
//------------------------------------------------------------------------
#define ADR_BASE_TSTAMP                 (ADR_BASE_CPUBUS + 0x00000000)
#define ADR_BASE_TRIG                   (ADR_BASE_CPUBUS + 0x00004000)
#define ADR_BASE_TPAT                   (ADR_BASE_CPUBUS + 0x00008000)      
#define ADR_BASE_MISC                   (ADR_BASE_CPUBUS + 0x0000C000)

// Tstamp manager regs
#define ADR_TSTAMP_CTRL                 ADR_BASE_TSTAMP + ( 0 * 4)  // Self-clearing Bit-0 resets timestamp
#define ADR_TSTAMP_CNT                  ADR_BASE_TSTAMP + ( 1 * 4)  // Current timestamp
#define ADR_TSTAMP_DIVISOR              ADR_BASE_TSTAMP + ( 2 * 4)  // Read only clock freq in MHz 

// Trigger manager regs
#define ADR_TRIG_SW_IN                  ADR_BASE_TRIG   + ( 0 * 4) 
#define ADR_TRIG_HW_IN_ENABLE           ADR_BASE_TRIG   + ( 2 * 4) 
#define ADR_TRIG_HW_IN_PARAM            ADR_BASE_TRIG   + ( 3 * 4) 
#define ADR_TRIG_HW_OUT_ENABLE          ADR_BASE_TRIG   + ( 4 * 4) 
#define ADR_TRIG_HW_OUT_PARAM           ADR_BASE_TRIG   + ( 5 * 4) 
#define ADR_TRIG_HW_OUT_DLY             ADR_BASE_TRIG   + ( 6 * 4) 
#define ADR_BASE_TRIG_SW_OUT_ENABLE     ADR_BASE_TRIG   + (32 * 4) 
#define ADR_BASE_TRIG_SW_OUT_DLY        ADR_BASE_TRIG   + (64 * 4)

// Trigger IDs as set in hardware design (s/w trig out wiring)
#define TRIG_ID_TPAT_GENERATOR			0
#define TRIG_ID_LED_DAUGHT_3			0
#define TRIG_ID_LED_CARR1_RED			1
#define TRIG_ID_LED_CARR1_GRN			2
#define TRIG_ID_LED_CARR1_BLU			3

// Test pattern generator regs
#define ADR_TPAT_CTRL                   ADR_BASE_TPAT   + ( 0 * 4) 
#define ADR_TPAT_STAT                   ADR_BASE_TPAT   + ( 1 * 4) 
#define ADR_TPAT_DATA                   ADR_BASE_TPAT   + ( 2 * 4) 
#define ADR_TPAT_TSTAMP                 ADR_BASE_TPAT   + ( 3 * 4)
#define ADR_TPAT_SW_TRIG_MASK			ADR_BASE_TRIG_SW_OUT_ENABLE \
									  		+ ( TRIG_ID_TPAT_GENERATOR * 4)
#define ADR_TPAT_SW_TRIG_DLY			ADR_BASE_TRIG_SW_OUT_DLY \
											+ ( TRIG_ID_TPAT_GENERATOR * 4)


//------------------------------------------------------------------------
// Register value extraction
//------------------------------------------------------------------------
#define TPAT_STAT_GET_DONE_BIT(val)		((val) & 0x1)
#define TPAT_STAT_GET_OUTPUT_CNT(val)	(((val) & 0xFFF) >> 8)											

#endif //_NUM_FPGA_H


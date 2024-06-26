/*
author: Katelyn Mak

This header file defines the register addresses and values to control the MCDMA.

*/
#ifndef AXI_MCDMA
#define AXI_MCDMA

#include <stdint.h>
#include <stdlib.h>
#include "misc.h"

/*
    AXI MCDMA Configuration
*/

typedef struct axi_mcdma axi_mcdma_t;
typedef struct axi_mcdma_channel axi_mcdma_channel_t;
typedef struct axi_mcdma_bd axi_mcdma_bd_t;

int32_t axi_mcdma_init(axi_mcdma_t *device, uint32_t baseaddr, uint32_t src_addr, uint32_t dst_addr, uint32_t mm2s_bd_addr, uint32_t s2mm_bd_addr, uint32_t size);
int axi_mcdma_haru_query_transfer(axi_mcdma_t *device, int channel_idx, uint32_t src_len, uint32_t dst_len);

void axi_mcdma_channel_init(axi_mcdma_t *device, int channel_idx, uint32_t src_addr_offset, uint32_t dst_addr_offset, int buf_size);
void axi_mcdma_mm2s_bd_init(axi_mcdma_t *device, int channel_idx, uint32_t transfer_size, uint32_t bd_addr_offset);
void axi_mcdma_s2mm_bd_init(axi_mcdma_t *device, int channel_idx, uint32_t transfer_size, uint32_t bd_addr_offset);
int axi_mcdma_mm2s_transfer(axi_mcdma_t *device);
int axi_mcdma_s2mm_transfer(axi_mcdma_t *device);
void axi_mcdma_release(axi_mcdma_t *device);
void axi_mcdma_free(axi_mcdma_t *device);

void mcdma_config_mm2s_channel(axi_mcdma_t *device, int channel_idx);
void mcdma_config_s2mm_channel(axi_mcdma_t *device, int channel_idx);
int mcdma_mm2s_busy_wait(axi_mcdma_t *device);
int mcdma_s2mm_busy_wait(axi_mcdma_t *device);
void mcdma_mm2s_stop(axi_mcdma_t *device);
void mcdma_s2mm_stop(axi_mcdma_t *device);
void mcdma_reset(axi_mcdma_t *device);
void mcdma_mm2s_program_tail_bd(axi_mcdma_t *device, int channel_idx);
void mcdma_s2mm_program_tail_bd(axi_mcdma_t *device, int channel_idx);
void mcdma_mm2s_start(axi_mcdma_t *device);
void mcdma_s2mm_start(axi_mcdma_t *device);
void mm2s_clear_channel_status(axi_mcdma_t *device);

void mm2s_common_status(axi_mcdma_t *device);
void mm2s_channel_status(axi_mcdma_t *device);
void mm2s_bd_status(axi_mcdma_channel_t *channel);
void s2mm_common_status(axi_mcdma_t *device);
void s2mm_channel_status(axi_mcdma_t *device);
void s2mm_bd_status(axi_mcdma_channel_t *channel);
/*
    AXI MCDMA Device Config
*/
#define NUM_CHANNELS 1

/* mcdma device */
struct axi_mcdma {
    // device info
    uint32_t p_baseaddr;
    uint32_t *v_baseaddr;
    int size; // size of device space in bytes

    // buffer addresses
    uint32_t p_buffer_src_addr;
    uint32_t *v_buffer_src_addr;
    uint32_t p_buffer_dst_addr;
    uint32_t *v_buffer_dst_addr;

    // bd addresses
    uint32_t p_mm2s_bd_addr;
    uint32_t *v_mm2s_bd_addr;
    uint32_t p_s2mm_bd_addr;
    uint32_t *v_s2mm_bd_addr;

    // channels
    uint32_t channel_en;
    axi_mcdma_channel_t *channels[NUM_CHANNELS];

};

/* mcdma channel */
struct axi_mcdma_channel {
    int channel_id;
    uint32_t p_buf_src_addr;
    uint32_t *v_buf_src_addr;
    uint32_t p_buf_dst_addr;
    uint32_t *v_buf_dst_addr;
    uint32_t buf_size;
    uint32_t mm2s_curr_bd_addr; // only counts bits 31:6
    uint32_t mm2s_tail_bd_addr; // only counts bits 31:6
    uint32_t s2mm_curr_bd_addr; // only counts bits 31:6
    uint32_t s2mm_tail_bd_addr; // only counts bits 31:6

    // buffer descriptor chain 
    axi_mcdma_bd_t *mm2s_bd_chain;
    axi_mcdma_bd_t *s2mm_bd_chain;
};

/* mcdma buffer descriptor */
struct axi_mcdma_bd {
    uint32_t p_bd_addr;
    uint32_t *v_bd_addr;
    axi_mcdma_bd_t *next_mcdma_bd;

    uint32_t next_bd_addr; // only counts bits 31:6
    uint32_t buffer_addr;
    uint32_t buffer_length; // only counts bits 0:25
    int sof;
    int eof;
    uint8_t tid;
    uint8_t tdest;
};

/*
    AXI MCDMA Buffer Address Space
*/
#define AXI_MCDMA_BD_OFFSET                         0x1000
#define AXI_MCDMA_CH_OFFSET                         0x040

#define AXI_MCDMA_BUF_INIT_ERROR     0x01

/*
    AXI Multichannel DMA MM2S
*/
// MM2S Address Space
#define AXI_MCDMA_MM2S_CCR                          0x000   // Common Control Register
#define AXI_MCDMA_MM2S_CSR                          0x004   // Common Status Register
#define AXI_MCDMA_MM2S_CHEN                         0x008   // Channel Enable/Disable
#define AXI_MCDMA_MM2S_CHSER                        0x00C   // Channel in Progress REgister
#define AXI_MCDMA_MM2S_ERR                          0x010   // Error Register
#define AXI_MCDMA_MM2S_CH_SCHD_TYPE                 0x014   // Channel Queue Scheduler type
#define AXI_MCDMA_MM2S_WWR_REG1                     0x018   // Weight of each channel (CH1-8)
#define AXI_MCDMA_MM2S_WWR_REG2                     0x01C   // Weight of each channel (CH9-16)
#define AXI_MCDMA_MM2S_CHANNELS_SERVICE             0x020   // MM2S Channels Completed Register
#define AXI_MCDMA_MM2S_ARCACHE_ARUSER               0x024   // Set the ARCHACHE and ARUSER values for AXI4 read
#define AXI_MCDMA_MM2S_INTR_STATUS                  0x028   // MM2S Channel Interrupt Monitor Register
#define AXI_MCDMA_MM2S_CHCR                         0x040   // CH1 Control Register

// MM2S Channel 1 Address Space
#define AXI_MCDMA_MM2S_CHSR                        0x044   // CH1 Status Register
#define AXI_MCDMA_MM2S_CHCURDESC_LSB               0x048   // CH1 Current Descriptor (LSB)
#define AXI_MCDMA_MM2S_CHCURDESC_MSB               0x04C   // CH1 Current Descriptor (MSB)
#define AXI_MCDMA_MM2S_CHTAILDESC_LSB              0x050   // CH1 Tail Descriptor (LSB)
#define AXI_MCDMA_MM2S_CHTAILDESC_MSB              0x054   // CH1 Tail Descriptor (MSB)
#define AXI_MCDMA_MM2S_CHPKTCOUNT_STAT             0x058   // CH1 Packet Processed count

// MM2S Common Control Register
#define AXI_MCDMA_MM2S_RS                           0x001   // Run = 1, Stop = 0
#define AXI_MCDMA_MM2S_RESET                        0x004   // Reset in progress = 1, Reset not in progress = 0

// MM2S Common Status Register
#define AXI_MCDMA_MM2S_HALTED                       0x001   // Halted = 1, Running = 0
#define AXI_MCDMA_MM2S_IDLE                         0x002   // Idle = = 1, Not Idle = 0

// MM2S Channel Control Register
#define AXI_MCDMA_MM2S_CHRS                        0x001   // Run = 1

// MM2S Channel Status Register
#define AXI_MCDMA_MM2S_CHIDLE                       0x001   // Idle = = 1, Not Idle = 0

// MM2S Error Register
#define AXI_MCDMA_MM2S_DMA_INTR_ERR                 0x01    // MCDMA Internal Error
#define AXI_MCDMA_MM2S_DMA_SLV_ERR                  0x02    // MCDMA Slave Error
#define AXI_MCDMA_MM2S_DMA_DEC_ERR                  0x04    // MCDMA Decode Error
#define AXI_MCDMA_MM2S_SG_INT_ERR                   0x10    // Scatter gather Internal Error
#define AXI_MCDMA_MM2S_SG_SLV_ERR                   0x20    // Scatter gather Slave Error
#define AXI_MCDMA_MM2S_SG_DEC_ERR                   0x40    // Scatter gather Decode Error

/*
    AXI Multichannel DMA S2MM
*/
// S2MM Address Space
#define AXI_MCDMA_S2MM_CCR                          0x500   // Common Control Register
#define AXI_MCDMA_S2MM_CSR                          0x504   // Common Status Register
#define AXI_MCDMA_S2MM_CHEN                         0x508   // Channel Enable/Disable
#define AXI_MCDMA_S2MM_CHSER                        0x50C   // Channel in Progress REgister
#define AXI_MCDMA_S2MM_ERR                          0x510   // Error Register
#define AXI_MCDMA_S2MM_PKTDROP                      0x514   // S2MM Packet Drop Stat
#define AXI_MCDMA_S2MM_CHANNELS_SERVICE             0x518   // S2MM Channels Completed Register
#define AXI_MCDMA_S2MM_AWCACHE_AWUSER               0x51C   // Set the values for awcache and awuser ports
#define AXI_MCDMA_S2MM_INTR_STATUS                  0x520   // S2MM Channel Interrupt Monitor Register

// S2MM Channel 1 Address Space
#define AXI_MCDMA_S2MM_CHCR                        0x540   // CH1 Status Register
#define AXI_MCDMA_S2MM_CHSR                        0x544   // CH1 Status Register
#define AXI_MCDMA_S2MM_CHCURDESC_LSB               0x548   // CH1 Current Descriptor (LSB)
#define AXI_MCDMA_S2MM_CHCURDESC_MSB               0x54C   // CH1 Current Descriptor (MSB)
#define AXI_MCDMA_S2MM_CHTAILDESC_LSB              0x550   // CH1 Tail Descriptor (LSB)
#define AXI_MCDMA_S2MM_CHTAILDESC_MSB              0x554   // CH1 Tail Descriptor (MSB)
#define AXI_MCDMA_S2MM_CHPKTDROP_STAT              0x558   // CH1 Packet Drop Stat
#define AXI_MCDMA_S2MM_CHPKTCOUNT_STAT             0x55C   // CH1 Packet Processed count

// S2MM Common Control Register
#define AXI_MCDMA_S2MM_RS                           0x001   // Run = 1, Stop = 0
#define AXI_MCDMA_S2MM_RESET                        0x004   // Reset in progress = 1, Reset not in progress = 0

// S2MM Common Status Register
#define AXI_MCDMA_S2MM_HALTED                       0x001   // Halted = 1, Running = 0
#define AXI_MCDMA_S2MM_IDLE                         0x002   // Idle = = 1, Not Idle = 0

// S2MM Channel Control Register
#define AXI_MCDMA_S2MM_CHRS                        0x001   // Run = 1

// S2MM Channel Status Register
#define AXI_MCDMA_S2MM_CHIDLE                       0x001   // Idle = = 1, Not Idle = 0

// S2MM Error Register
#define AXI_MCDMA_S2MM_DMA_INTR_ERR                 0x01    // MCDMA Internal Error
#define AXI_MCDMA_S2MM_DMA_SLV_ERR                  0x02    // MCDMA Slave Error
#define AXI_MCDMA_S2MM_DMA_DEC_ERR                  0x04    // MCDMA Decode Error
#define AXI_MCDMA_S2MM_SG_INT_ERR                   0x10    // Scatter gather Internal Error
#define AXI_MCDMA_S2MM_SG_SLV_ERR                   0x20    // Scatter gather Slave Error
#define AXI_MCDMA_S2MM_SG_DEC_ERR                   0x40    // Scatter gather Decode Error

// Channel status register values
#define AXI_MCDMA_CH_IDLE                           0x01 // Channel idle (queue empty)
#define AXI_MCDMA_CH_ERR_OTH_CH                     0x08 // Channel error on other channel
#define AXI_MCDMA_CH_IOC_IRQ                        0x20
#define AXI_MCDMA_CH_DLY_IRQ                        0x40
#define AXI_MCDMA_CH_ERR_IRQ                        0x80
// todo: irq thresholds and statuses

/*
    Scatter Gather Buffer Descriptor Addresses
*/
#define AXI_MCDMA_MM2S_BD_NEXT_DESC_LSB             0x00 // Next descriptor pointer
#define AXI_MCDMA_MM2S_BD_NEXT_DESC_MSB             0x04 // Next descriptor pointer
#define AXI_MCDMA_MM2S_BD_BUF_ADDR_LSB              0x08 // Buffer descriptor address
#define AXI_MCDMA_MM2S_BD_BUF_ADDR_MSB              0x08 // Buffer descriptor address
#define AXI_MCDMA_MM2S_BD_CONTROL                   0x14 // Control Information for BD
#define AXI_MCDMA_MM2S_BD_CONTROL_SIDEBAND          0x18 // Control Information for BD
#define AXI_MCDMA_MM2S_BD_STATUS                    0x1C // Control Information for BD

#define AXI_MCDMA_S2MM_BD_NEXT_DESC_LSB             0x00 // Next descriptor pointer
#define AXI_MCDMA_S2MM_BD_NEXT_DESC_MSB             0x04 // Next descriptor pointer
#define AXI_MCDMA_S2MM_BD_BUF_ADDR_LSB              0x08 // Buffer descriptor address
#define AXI_MCDMA_S2MM_BD_BUF_ADDR_MSB              0x08 // Buffer descriptor address
#define AXI_MCDMA_S2MM_BD_CONTROL                   0x14 // Control Information for BD
#define AXI_MCDMA_S2MM_BD_STATUS                    0x18 // Control Information for BD
#define AXI_MCDMA_S2MM_BD_SIDEBAND_STATUS           0x1C // Control Information for BD

#define AXI_MCDMA_MM2S_BD_SBYTE_MASK                0x01ffffff
#define AXI_MCDMA_MM2S_BD_DMA_INT_ERR               0x1 << 28
#define AXI_MCDMA_MM2S_BD_DMA_SLV_ERR               0x1 << 29
#define AXI_MCDMA_MM2S_BD_DMA_DEC_ERR               0x1 << 30
#define AXI_MCDMA_MM2S_BD_DMA_COMPLETED             0x1 << 31

#define AXI_MCDMA_S2MM_BD_SBYTE_MASK                0x01ffffff
#define AXI_MCDMA_S2MM_BD_DMA_INT_ERR               0x1 << 28
#define AXI_MCDMA_S2MM_BD_DMA_SLV_ERR               0x1 << 29
#define AXI_MCDMA_S2MM_BD_DMA_DEC_ERR               0x1 << 30
#define AXI_MCDMA_S2MM_BD_DMA_COMPLETED             0x1 << 31
#define AXI_MCDMA_S2MM_BD_DMA_RXSOF                 0x1 << 27
#define AXI_MCDMA_S2MM_BD_DMA_RXEOF                 0x1 << 28

#endif
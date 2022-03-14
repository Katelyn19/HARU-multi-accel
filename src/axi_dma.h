// https://www.xilinx.com/support/documentation/ip_documentation/axi_dma/v7_1/pg021_axi_dma.pdf

#ifndef AXI_DMA
#define AXI_DMA

#include <stdint.h>

// AXI DMA register map (Direct Register Mode)
#define AXI_DMA_MM2S_CR                 0x00
#define AXI_DMA_MM2S_SR                 0x04
#define AXI_DMA_MM2S_SRC_ADDR           0x18
#define AXI_DMA_MM2S_SRC_ADDR_MSB       0x1C
#define AXI_DMA_MM2S_LENGTH             0x28

#define AXI_DMA_S2MM_CR                 0x30
#define AXI_DMA_S2MM_SR                 0x34
#define AXI_DMA_S2MM_DST_ADDR           0x48
#define AXI_DMA_S2MM_DST_ADDR_MSB       0x4C
#define AXI_DMA_S2MM_LENGTH             0x58

// CR register bits
#define AXI_DMA_CR_RS                   0x00 // Run Stop. Default = 0
#define AXI_DMA_CR_RESET                0x02 // Reset. Default = 0
#define AXI_DMA_CR_IOC_IRQ_EN           0x012 // Interrupt on Completion IRQ Enable. Default = 0
#define AXI_DMA_CR_DLY_IRQ_EN           0x013 // Delay on Timer Interrupt Enable. Default = 0
#define AXI_DMA_CR_ERR_IRQ_EN           0x014 // Error Interrupt Enable. Default = 0

// SR register bits
#define AXI_DMA_SR_HALTED               0x00 // Channel halted. 0 = Running, 1 = Halted
#define AXI_DMA_SR_IDLE                 0x01 // Channel idle. 0 = Active, 1 = Idle
#define AXI_DMA_SR_SG_ACT               0x03 // Scatter/Gather Active. 0 = Inactive, 1 = Active
#define AXI_DMA_SR_DMA_INT_ERR          0x04 // DMA Internal Error. 0 = No error, 1 = Internal error
#define AXI_DMA_SR_DMA_SLV_ERR          0x05 // DMA Slave Error. 0 = No error, 1 = Slave error
#define AXI_DMA_SR_DMA_DEC_ERR          0x06 // DMA Decode Error. 0 = No error, 1 = Decode error
#define AXI_DMA_SR_SG_ERR               0x07 // DMA Scatter/Gather Error. 0 = No error, 1 = Scatter/Gather error
#define AXI_DMA_SR_SG_INT_ERR           0x08 // DMA Scatter/Gather Internal Error. 0 = No error, 1 = Error
#define AXI_DMA_SR_SG_SLV_ERR           0x09 // DMA Scatter/Gather Slave Error. 0 = No error, 1 = Slave error
#define AXI_DMA_SR_SG_DEC_ERR           0x0A // DMA Scatter/Gather Decode Error. 0 = No error, 1 = Decode error
#define AXI_DMA_SR_IOC_IRQ              0x0C // Interrupt on Completion IRQ. 0 = No IRQ, 1 = IRQ
#define AXI_DMA_SR_DLY_IRQ              0x0D // Delay on Timer IRQ. 0 = No IRQ, 1 = IRQ
#define AXI_DMA_SR_ERR_IRQ              0x0E // Error IRQ. 0 = No IRQ, 1 = IRQ

/* AXI DMA type */
typedef struct {
    uint32_t v_baseaddr;    // Memory mapped virtual base address
    uint32_t p_baseaddr;    // Physical base address
    uint32_t size;          // Size of device
} axi_dma_t;

uint32_t axi_dma_init(axi_dma_t *device, uint32_t baseaddr, uint32_t size);
void axi_dma_release(axi_dma_t *device);

void dma_mm2s_reset(axi_dma_t *device);
void dma_mm2s_run(axi_dma_t *device);
void dma_mm2s_stop(axi_dma_t *device);
void dma_mm2s_busy(axi_dma_t *device);
void dma_mm2s_IOC_IRQ_EN(axi_dma_t *device);
void dma_mm2s_IOC_IRQ_DIS(axi_dma_t *device);
void dma_mm2s_DLY_IRO_EN(axi_dma_t *device);
void dma_mm2s_DLY_IRO_DIS(axi_dma_t *device);
void dma_mm2s_ERR_IRQ_EN(axi_dma_t *device);
void dma_mm2s_ERR_IRQ_DIS(axi_dma_t *device);
void dma_mm2s_set_src_addr(axi_dma_t *device, uint32_t addr);
void dma_mm2s_set_src_addr_msb(axi_dma_t *device, uint32_t addr);
void dma_mm2s_set_length(axi_dma_t *device, uint32_t length);

void dma_s2mm_reset(axi_dma_t *device);
void dma_s2mm_run(axi_dma_t *device);
void dma_s2mm_stop(axi_dma_t *device);
void dma_s2mm_busy(axi_dma_t *device);
void dma_s2mm_IOC_IRQ_EN(axi_dma_t *device);
void dma_s2mm_IOC_IRQ_DIS(axi_dma_t *device);
void dma_s2mm_DLY_IRO_EN(axi_dma_t *device);
void dma_s2mm_DLY_IRO_DIS(axi_dma_t *device);
void dma_s2mm_ERR_IRQ_EN(axi_dma_t *device);
void dma_s2mm_ERR_IRQ_DIS(axi_dma_t *device);

void axi_dma_mm2s_transfer(axi_dma_t *device, uint32_t src_addr, uint32_t size);
void axi_dma_s2mm_transfer(axi_dma_t *device, uint32_t dst_addr, uint32_t size);

#endif
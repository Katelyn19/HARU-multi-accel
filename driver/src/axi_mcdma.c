
/*
author: Katelyn Mak
date created: 15/4/2024

This module is used to control the AXI MCDMA.

*/
#include "axi_mcdma.h"

#include <stdio.h> // todo: remove this once debugging is finished
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <string.h>

int32_t axi_mcdma_init(axi_mcdma_t *device, uint32_t baseaddr, uint32_t src_addr, uint32_t dst_addr, uint32_t mm2s_bd_addr, uint32_t s2mm_bd_addr, uint32_t size) {
	/*** Memory map address space ***/
	// Open /dev/mem for memory mapping
	int32_t dev_fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (dev_fd < 0) {
		HARU_ERROR("%s", "Failed to open /dev/mem.");
		return -1;
	}

	// initialise the axi dma control space
	device->size = size;
	device->p_baseaddr = baseaddr;
	device->v_baseaddr = (uint32_t *) mmap(NULL, (uint32_t) size, PROT_READ | PROT_WRITE, MAP_SHARED, dev_fd, (uint32_t) baseaddr); 
	if (device->v_baseaddr == MAP_FAILED) {
		HARU_ERROR("%s", "control space map failed.");
		close(dev_fd);
		return -1;
	}

	// Reset device
	mcdma_reset(device);

	// intialise mm2s buffer space
	device->p_buffer_src_addr = src_addr;
	device->v_buffer_src_addr = (uint32_t *) mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, dev_fd, src_addr); 
	if (device->v_buffer_src_addr == MAP_FAILED) {
		HARU_ERROR("%s", "buffer src address map failed.");
		close(dev_fd);
		return -1;
	}

	// initialise s2mm buffer space
	device->p_buffer_dst_addr = dst_addr;
	device->v_buffer_dst_addr = (uint32_t *) mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, dev_fd, dst_addr); 
	if (device->v_buffer_dst_addr == MAP_FAILED) {
		HARU_ERROR("%s", "buffer dst address map failed.");
		close(dev_fd);
		return -1;
	}

	// initialise mm2s bd chain space
	device->p_mm2s_bd_addr = mm2s_bd_addr;
	device->v_mm2s_bd_addr = (uint32_t *) mmap(NULL, AXI_MCDMA_BD_OFFSET, PROT_READ | PROT_WRITE, MAP_SHARED, dev_fd, mm2s_bd_addr); 
	if (device->v_mm2s_bd_addr == MAP_FAILED) {
		HARU_ERROR("%s", "mm2s bd chain address map failed.");
		close(dev_fd);
		return -1;
	}

	// initialise s2mm bd chain space
	device->p_s2mm_bd_addr = s2mm_bd_addr;
	device->v_s2mm_bd_addr = (uint32_t *) mmap(NULL, AXI_MCDMA_BD_OFFSET, PROT_READ | PROT_WRITE, MAP_SHARED, dev_fd, s2mm_bd_addr); 
	if (device->v_s2mm_bd_addr == MAP_FAILED) {
		HARU_ERROR("%s", "s2mm bd chain address map failed.");
		close(dev_fd);
		return -1;
	}

	// Disable all channels
	device->channel_en = 0x0000;
	for (int i = 0; i < NUM_CHANNELS; i++) {
		device->channels[i] = NULL;
	}

	close(dev_fd);

	return 0;
}

/*
	Initialises a single mcdma channel struct. The registers are not written until _ function. 
	Configures the following:
		- source buffer address
		- destination buffer address
		- source and destination buffer sizes
		- channel enable
*/
void axi_mcdma_channel_init(axi_mcdma_t *device, int channel_idx, uint32_t src_addr_offset, uint32_t dst_addr_offset, int buf_size) {
	axi_mcdma_channel_t *channel;
	if (device->channels[channel_idx] == NULL) {
		channel = (axi_mcdma_channel_t *) malloc(sizeof( axi_mcdma_channel_t ));
		HARU_MALLOC_CHK(channel);
		device->channels[channel_idx] = channel;
	} else {
		channel = device->channels[channel_idx];
	}

	device->channel_en = device->channel_en | (1 << channel_idx);

	channel->channel_id = channel_idx;
	channel->p_buf_src_addr = device->p_buffer_src_addr + src_addr_offset;
	channel->v_buf_src_addr = device->v_buffer_src_addr + src_addr_offset;
	channel->p_buf_dst_addr = device->p_buffer_dst_addr + dst_addr_offset;
	channel->v_buf_dst_addr = device->v_buffer_dst_addr + dst_addr_offset;
	channel->buf_size = buf_size;

	HARU_LOG("Configuring channel %d struct", channel_idx);
	HARU_LOG("ch%d_p_buf_src_addr : 0x%08x", channel_idx, channel->p_buf_src_addr);
	HARU_LOG("ch%d_p_buf_dst_addr : 0x%08x", channel_idx, channel->p_buf_dst_addr);
	HARU_LOG("ch%d_size : 0x%08x", channel_idx, channel->buf_size);
}

/*
	Initialises a single buffer descriptor in memory space. Overwites existing buffer descriptor if there is one.
	This buffer descriptor sets up a transfer with "transfer_size" number of bytes from the channel's source buffer.
*/
void axi_mcdma_mm2s_bd_init(axi_mcdma_t *device, int channel_idx, uint32_t transfer_size, uint32_t bd_addr_offset) {
	HARU_LOG("Configuring bd chain for channel %d", channel_idx);
	axi_mcdma_channel_t *channel = device->channels[channel_idx];

	if (transfer_size > channel->buf_size) {
		HARU_ERROR("Transfer size (0x%08x) greater than buffer size (0x%08x)", transfer_size, channel->buf_size);
	}

	axi_mcdma_bd_t *mm2s_bd = channel->mm2s_bd_chain;
	if (mm2s_bd == NULL) {
		mm2s_bd = (axi_mcdma_bd_t *) malloc(sizeof( axi_mcdma_bd_t ));
		HARU_MALLOC_CHK(mm2s_bd);
		channel->mm2s_bd_chain = mm2s_bd;
	}

	channel->mm2s_curr_bd_addr = device->p_mm2s_bd_addr + bd_addr_offset;
	channel->mm2s_tail_bd_addr = device->p_mm2s_bd_addr + bd_addr_offset;
	mm2s_bd->p_bd_addr = device->p_mm2s_bd_addr + bd_addr_offset;
	mm2s_bd->v_bd_addr = device->v_mm2s_bd_addr + bd_addr_offset;

	mm2s_bd->next_mcdma_bd = NULL;
	mm2s_bd->next_bd_addr = channel->mm2s_tail_bd_addr;
	mm2s_bd->buffer_addr = channel->p_buf_src_addr;
	mm2s_bd->buffer_length = transfer_size;
	mm2s_bd->sof = 1;
	mm2s_bd->eof = 1;
	mm2s_bd->tid = 0;

	HARU_LOG("%s", "mm2s_bd fields:");
	HARU_LOG("> next_bd_addr: 0x%08x", mm2s_bd->next_bd_addr);
	HARU_LOG("> buffer_addr: 0x%08x", mm2s_bd->buffer_addr);
	HARU_LOG("> buffer_length: 0x%08x", mm2s_bd->buffer_length);
	HARU_LOG("> sof: %d", mm2s_bd->sof);
	HARU_LOG("> eof: %d", mm2s_bd->eof);
	HARU_LOG("> tid: %d", mm2s_bd->tid);

	_reg_set(mm2s_bd->v_bd_addr, AXI_MCDMA_MM2S_BD_NEXT_DESC_LSB, mm2s_bd->next_bd_addr);
	_reg_set(mm2s_bd->v_bd_addr, AXI_MCDMA_MM2S_BD_BUF_ADDR_LSB, mm2s_bd->buffer_addr);
	uint32_t control = (uint32_t) (mm2s_bd->sof << 31) | (uint32_t) (mm2s_bd->eof << 30) | mm2s_bd->buffer_length;
	_reg_set(mm2s_bd->v_bd_addr, AXI_MCDMA_MM2S_BD_CONTROL, control);
	uint32_t control_sideband = (uint32_t) (mm2s_bd->tid << 31);
	_reg_set(mm2s_bd->v_bd_addr, AXI_MCDMA_MM2S_BD_CONTROL_SIDEBAND, control_sideband);
	_reg_set(mm2s_bd->v_bd_addr, AXI_MCDMA_MM2S_BD_STATUS, 0x00000000);

}

void axi_mcdma_mm2s_transfer(axi_mcdma_t *device) {
	// Clearup
	mcdma_reset(device);
	mcdma_mm2s_stop(device);

	// Config and start
	_reg_set(device->v_baseaddr, AXI_MCDMA_MM2S_CCR, AXI_MCDMA_MM2S_RS);
	HARU_LOG("Start mm2s @ 0x%03x", AXI_MCDMA_MM2S_CCR);

	_reg_set(device->v_baseaddr, AXI_MCDMA_MM2S_CHEN, device->channel_en);
	HARU_LOG("reg@0x%03x : 0x%08x (channel enable)", AXI_MCDMA_MM2S_CHEN, device->channel_en);

	// Program mm2s current and tail descriptor
	for (int i = 0; i < NUM_CHANNELS; i++) {
		if (device->channel_en & (1 << i)) {
			config_and_start_mcdma_mm2s_channel(device, i);
		}
	}

	mcdma_mm2s_busy_wait(device);
	HARU_LOG("%s", "mm2s transfer done.");
	
}

void config_and_start_mcdma_mm2s_channel(axi_mcdma_t *device, int channel_idx) {
	// Set current descriptor
	_reg_set(device->v_baseaddr, (AXI_MCDMA_MM2S_CHCURDESC_LSB + AXI_MCDMA_CH_OFFSET*channel_idx), device->channels[channel_idx]->mm2s_curr_bd_addr);
	// Channel fetch bit
	_reg_set(device->v_baseaddr, (AXI_MCDMA_MM2S_CHCR + AXI_MCDMA_CH_OFFSET*channel_idx), AXI_MCDMA_MM2S_CHRS);

	HARU_LOG("Writing mm2s configuration to addr 0x%08x", device->p_baseaddr + AXI_MCDMA_CH_OFFSET*channel_idx);
	HARU_LOG("reg@0x%03x : 0x%08x (current bd)", AXI_MCDMA_MM2S_CHCURDESC_LSB + AXI_MCDMA_CH_OFFSET*channel_idx, device->channels[channel_idx]->mm2s_curr_bd_addr);
	HARU_LOG("reg@0x%03x : 0x%08x (channel %d fetch)", AXI_MCDMA_MM2S_CHCR + AXI_MCDMA_CH_OFFSET*channel_idx, AXI_MCDMA_MM2S_CHRS, channel_idx);

	_reg_set(device->v_baseaddr, (AXI_MCDMA_MM2S_CHTAILDESC_LSB + AXI_MCDMA_CH_OFFSET*channel_idx), device->channels[channel_idx]->mm2s_tail_bd_addr);
	HARU_LOG("Programmed mm2s tail bd 0x%03x : 0x%08x", AXI_MCDMA_MM2S_CHTAILDESC_LSB + AXI_MCDMA_CH_OFFSET*channel_idx, device->channels[channel_idx]->mm2s_tail_bd_addr);
}

void mcdma_mm2s_busy_wait(axi_mcdma_t *device) {
	// Busy wait
	uint32_t mm2s_sr = _reg_get(device->v_baseaddr, AXI_MCDMA_MM2S_CSR);
	while (!(mm2s_sr & AXI_MCDMA_MM2S_IDLE)) {
		mm2s_sr = _reg_get(device->v_baseaddr, AXI_MCDMA_MM2S_CSR);
	}
}

int get_bd_complete(uint32_t *bd_v_addr) {
	uint32_t bd_status = _reg_get(bd_v_addr, AXI_MCDMA_S2MM_BD_STATUS);
	return (bd_status & AXI_MCDMA_MM2S_BD_DMA_COMPLETED);
}
/*
	Resets AXI MCDMA with the mm2s common control register. Resetting with the mm2s ccr resets both mm2s and s2mm.
*/
void mcdma_reset(axi_mcdma_t *device) {
	HARU_LOG("%s", "Reset MCDMA.");
	_reg_set(device->v_baseaddr, AXI_MCDMA_MM2S_CCR, AXI_MCDMA_MM2S_RESET);

	// wait for reset
	uint32_t mm2s_ccr = _reg_get(device->v_baseaddr, AXI_MCDMA_MM2S_CCR);
	while (mm2s_ccr & AXI_MCDMA_MM2S_RESET) {
		mm2s_ccr = _reg_get(device->v_baseaddr, AXI_MCDMA_MM2S_CCR);
	}
}

void mcdma_mm2s_stop(axi_mcdma_t *device) {
	HARU_LOG("%s", "Stop mm2s MCDMA operations.");
	_reg_set(device->v_baseaddr, AXI_MCDMA_MM2S_CCR, AXI_MCDMA_MM2S_RS);
}
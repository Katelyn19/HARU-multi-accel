
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
	mm2s_common_status(device);
	mm2s_channel_status(device);
	s2mm_common_status(device);
	s2mm_channel_status(device);

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
	device->v_mm2s_bd_addr = (uint32_t *) mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, dev_fd, mm2s_bd_addr); 
	if (device->v_mm2s_bd_addr == MAP_FAILED) {
		HARU_ERROR("%s", "mm2s bd chain address map failed.");
		close(dev_fd);
		return -1;
	}

	// initialise s2mm bd chain space
	device->p_s2mm_bd_addr = s2mm_bd_addr;
	device->v_s2mm_bd_addr = (uint32_t *) mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, dev_fd, s2mm_bd_addr); 
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
	channel->mm2s_bd_chain = NULL;
	channel->s2mm_bd_chain = NULL;

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
	HARU_LOG("Configuring mm2s bd chain for channel %d", channel_idx);
	axi_mcdma_channel_t *channel = device->channels[channel_idx];

	if (transfer_size > channel->buf_size) {
		HARU_ERROR("Transfer size (0x%08x) greater than buffer size (0x%08x)", transfer_size, channel->buf_size);
	} else {
		HARU_LOG("Setting up BD for %d bytes.", transfer_size);
	}

	axi_mcdma_bd_t *mm2s_bd = channel->mm2s_bd_chain;
	if (mm2s_bd == NULL) {
        HARU_LOG("allocating struct of size: %ld bytes", sizeof(axi_mcdma_bd_t));
		mm2s_bd = (axi_mcdma_bd_t *) malloc(sizeof( axi_mcdma_bd_t ));
		HARU_MALLOC_CHK(mm2s_bd);
		channel->mm2s_bd_chain = mm2s_bd;
	}

	HARU_LOG("mm2s_bd_chain address: %p", channel->mm2s_bd_chain);
	HARU_LOG("mm2s_bd address: %p", mm2s_bd);

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

	// HARU_LOG("%s", "mm2s_bd fields:");
	// HARU_LOG("> next_bd_addr: 0x%08x", mm2s_bd->next_bd_addr);
	// HARU_LOG("> buffer_addr: 0x%08x", mm2s_bd->buffer_addr);
	// HARU_LOG("> buffer_length: 0x%08x", mm2s_bd->buffer_length);
	// HARU_LOG("> sof: %d", mm2s_bd->sof);
	// HARU_LOG("> eof: %d", mm2s_bd->eof);
	// HARU_LOG("> tid: %d", mm2s_bd->tid);

	_reg_set(mm2s_bd->v_bd_addr, AXI_MCDMA_MM2S_BD_NEXT_DESC_LSB, mm2s_bd->next_bd_addr);
	_reg_set(mm2s_bd->v_bd_addr, AXI_MCDMA_MM2S_BD_BUF_ADDR_LSB, mm2s_bd->buffer_addr);
	uint32_t control = (uint32_t) (mm2s_bd->sof << 31) | (uint32_t) (mm2s_bd->eof << 30) | mm2s_bd->buffer_length;
	_reg_set(mm2s_bd->v_bd_addr, AXI_MCDMA_MM2S_BD_CONTROL, control);
	uint32_t control_sideband = (uint32_t) (mm2s_bd->tid << 31);
	_reg_set(mm2s_bd->v_bd_addr, AXI_MCDMA_MM2S_BD_CONTROL_SIDEBAND, control_sideband);
	_reg_set(mm2s_bd->v_bd_addr, AXI_MCDMA_MM2S_BD_STATUS, 0x00000000);

	HARU_LOG("Writing mm2s_bd fields to addr 0x%08x:", mm2s_bd->p_bd_addr);
	HARU_LOG("reg@0x%03x : 0x%08x (next bd)", mm2s_bd->p_bd_addr + AXI_MCDMA_MM2S_BD_NEXT_DESC_LSB, mm2s_bd->next_bd_addr);
	HARU_LOG("reg@0x%03x : 0x%08x (buf addr)", mm2s_bd->p_bd_addr + AXI_MCDMA_MM2S_BD_BUF_ADDR_LSB, mm2s_bd->buffer_addr);
	HARU_LOG("reg@0x%03x : 0x%08x (bd sof, eof, buf length)", mm2s_bd->p_bd_addr + AXI_MCDMA_MM2S_BD_CONTROL, control);
	HARU_LOG("reg@0x%03x : 0x%08x (bd tid)", mm2s_bd->p_bd_addr + AXI_MCDMA_MM2S_BD_CONTROL_SIDEBAND, control_sideband);
	HARU_LOG("reg@0x%03x : 0x%08x (bd status)", mm2s_bd->p_bd_addr + AXI_MCDMA_MM2S_BD_STATUS, 0x00000000);
}

void axi_mcdma_s2mm_bd_init(axi_mcdma_t *device, int channel_idx, uint32_t transfer_size, uint32_t bd_addr_offset) {
	HARU_LOG("Configuring s2mm bd chain for channel %d", channel_idx);
	axi_mcdma_channel_t *channel = device->channels[channel_idx];

	if (transfer_size > channel->buf_size) {
		HARU_ERROR("Transfer size (0x%08x) greater than buffer size (0x%08x)", transfer_size, channel->buf_size);
	} else {
		HARU_LOG("Setting up BD for %d bytes.", transfer_size);
	}


	axi_mcdma_bd_t *s2mm_bd = channel->s2mm_bd_chain;
	if (s2mm_bd == NULL) {
		s2mm_bd = (axi_mcdma_bd_t *) malloc(sizeof( axi_mcdma_bd_t ));
		HARU_MALLOC_CHK(s2mm_bd);
		channel->s2mm_bd_chain = s2mm_bd;
	}

	channel->s2mm_curr_bd_addr = device->p_s2mm_bd_addr + bd_addr_offset;
	channel->s2mm_tail_bd_addr = device->p_s2mm_bd_addr + bd_addr_offset;
	s2mm_bd->p_bd_addr = device->p_s2mm_bd_addr + bd_addr_offset;
	s2mm_bd->v_bd_addr = device->v_s2mm_bd_addr;
	*s2mm_bd->v_bd_addr += bd_addr_offset;

	s2mm_bd->next_mcdma_bd = NULL;
	s2mm_bd->next_bd_addr = channel->s2mm_tail_bd_addr;
	s2mm_bd->buffer_addr = channel->p_buf_dst_addr;
	s2mm_bd->buffer_length = transfer_size;

	// HARU_LOG("%s", "s2mm_bd fields:");
	// HARU_LOG("> next_bd_addr: 0x%08x", s2mm_bd->next_bd_addr);
	// HARU_LOG("> buffer_addr: 0x%08x", s2mm_bd->buffer_addr);
	// HARU_LOG("> buffer_length: 0x%08x", s2mm_bd->buffer_length);

	_reg_set(s2mm_bd->v_bd_addr, AXI_MCDMA_S2MM_BD_NEXT_DESC_LSB, s2mm_bd->next_bd_addr);
	_reg_set(s2mm_bd->v_bd_addr, AXI_MCDMA_S2MM_BD_BUF_ADDR_LSB, s2mm_bd->buffer_addr);
	uint32_t control = (uint32_t) (s2mm_bd->sof << 31) | (uint32_t) (s2mm_bd->eof << 30) | s2mm_bd->buffer_length;
	_reg_set(s2mm_bd->v_bd_addr, AXI_MCDMA_S2MM_BD_CONTROL, control);
	_reg_set(s2mm_bd->v_bd_addr, AXI_MCDMA_S2MM_BD_STATUS, 0x00000000);

	HARU_LOG("Writing s2mm_bd fields to addr 0x%08x:", s2mm_bd->p_bd_addr);
	HARU_LOG("reg@0x%03x : 0x%08x (next bd)", s2mm_bd->p_bd_addr + AXI_MCDMA_S2MM_BD_NEXT_DESC_LSB, s2mm_bd->next_bd_addr);
	HARU_LOG("reg@0x%03x : 0x%08x (buf addr)", s2mm_bd->p_bd_addr + AXI_MCDMA_S2MM_BD_BUF_ADDR_LSB, s2mm_bd->buffer_addr);
	HARU_LOG("reg@0x%03x : 0x%08x (buf length)", s2mm_bd->p_bd_addr + AXI_MCDMA_S2MM_BD_CONTROL, s2mm_bd->buffer_length);
	HARU_LOG("reg@0x%03x : 0x%08x (bd status)", s2mm_bd->p_bd_addr + AXI_MCDMA_S2MM_BD_STATUS, 0x00000000);
}

int axi_mcdma_mm2s_transfer(axi_mcdma_t *device) {
	// Clearup
	mcdma_reset(device);
	mcdma_s2mm_stop(device);
	mcdma_mm2s_stop(device);

	// Config and start
	_reg_set(device->v_baseaddr, AXI_MCDMA_MM2S_CHEN, device->channel_en);
	HARU_LOG("reg@0x%03x : 0x%08x (channel enable)", AXI_MCDMA_MM2S_CHEN, device->channel_en);


	// Program mm2s current and tail descriptor
	for (int i = 0; i < NUM_CHANNELS; i++) {
		if (device->channel_en & (1 << i)) {
			mcdma_config_mm2s_channel(device, i);
		}
	}

	mcdma_mm2s_start(device);

	for (int i = 0; i < NUM_CHANNELS; i++) {
		if (device->channel_en & (1 << i)) {
			mcdma_mm2s_program_tail_bd(device, i);
		}
	}

	int res;
	res = mcdma_mm2s_busy_wait(device);
	if (res) {
		HARU_ERROR("%s", "mm2s transfer failed.");
		return -1;
	}

	HARU_LOG("%s", "mm2s transfer done.");
	mm2s_common_status(device);
	mm2s_channel_status(device);
	mm2s_bd_status(device->channels[0]);

	return 0;
}

int axi_mcdma_s2mm_transfer(axi_mcdma_t *device) {
	// Config and start
	_reg_set(device->v_baseaddr, AXI_MCDMA_S2MM_CHEN, device->channel_en);
	HARU_LOG("reg@0x%03x : 0x%08x (channel enable)", AXI_MCDMA_S2MM_CHEN, device->channel_en);


	// Program s2mm current and tail descriptor
	for (int i = 0; i < NUM_CHANNELS; i++) {
		if (device->channel_en & (1 << i)) {
			mcdma_config_s2mm_channel(device, i);
			mcdma_mm2s_start(device);
			mcdma_s2mm_program_tail_bd(device, i);
		}
	}

	int res;
	res = mcdma_s2mm_busy_wait(device);
	if (res) {
		HARU_ERROR("%s", "s2mm transfer failed.");
		return -1;
	}

	HARU_LOG("%s", "s2mm transfer done.");
	s2mm_common_status(device);
	s2mm_channel_status(device);
	s2mm_bd_status(device->channels[0]);

	return 0;
}

void axi_mcdma_release(axi_mcdma_t *device) {
	munmap(device->v_baseaddr, device->size);
	munmap(device->v_buffer_src_addr, device->size);
	munmap(device->v_buffer_dst_addr, device->size);
	munmap(device->v_mm2s_bd_addr, device->size);
	munmap(device->v_s2mm_bd_addr, device->size);
}

void axi_mcdma_free(axi_mcdma_t *device) {
	axi_mcdma_channel_t *channel;
	axi_mcdma_bd_t *curr_bd;
	axi_mcdma_bd_t *next_bd;
	for (int i = 0; i < NUM_CHANNELS; i++) {
		channel = device->channels[i];
		if (channel != NULL) {
			HARU_LOG("%s", "channel not null.");
			curr_bd = channel->mm2s_bd_chain;
			while (curr_bd != NULL) {
				HARU_LOG("freeing pointer @ %p", curr_bd);
				next_bd = curr_bd->next_mcdma_bd;
				free(curr_bd);
				curr_bd = next_bd;
			}
			HARU_LOG("%s", "Freed mm2s bd chain.");

			curr_bd = channel->s2mm_bd_chain;
			while (curr_bd != NULL) {
				HARU_LOG("freeing pointer @ %p", curr_bd);
				next_bd = curr_bd->next_mcdma_bd;
				free(curr_bd);
				curr_bd = next_bd;
			}
			HARU_LOG("%s", "Freed s2mm bd chain.");

			HARU_LOG("%s", "Freed channel");
		} else {
			HARU_LOG("%s", "channel is null.");
		}
	}

	for (int i = 0; i < NUM_CHANNELS; i++) {
		free(device->channels[i]);
	}
}

int axi_mcdma_haru_query_transfer(axi_mcdma_t *device, int channel_idx, uint32_t src_len, uint32_t dst_len) {
	// Clearup
	mcdma_reset(device);
	mcdma_s2mm_stop(device);
	mcdma_mm2s_stop(device);

	// mm2s_clear_channel_status(device);
	mm2s_common_status(device);
	mm2s_channel_status(device);
	s2mm_common_status(device);
	s2mm_channel_status(device);

	/* s2mm setup */
	// s2mm bd config
	axi_mcdma_s2mm_bd_init(device, channel_idx, dst_len, 0);
	_reg_set(device->v_baseaddr, AXI_MCDMA_S2MM_CHEN, device->channel_en);
	HARU_LOG("reg@0x%03x : 0x%08x (channel enable)", AXI_MCDMA_S2MM_CHEN, device->channel_en);
	mcdma_config_s2mm_channel(device, channel_idx);

	// s2mm start
	mcdma_s2mm_start(device);
	mcdma_s2mm_program_tail_bd(device, channel_idx);

	/* mm2s setup */
	// mm2s bd config
	axi_mcdma_mm2s_bd_init(device, 0, src_len, 0);
	_reg_set(device->v_baseaddr, AXI_MCDMA_MM2S_CHEN, device->channel_en);
	HARU_LOG("reg@0x%03x : 0x%08x (channel enable)", AXI_MCDMA_MM2S_CHEN, device->channel_en);
	mcdma_config_mm2s_channel(device, channel_idx);

	// mm2s start
	mcdma_mm2s_start(device);
	mcdma_mm2s_program_tail_bd(device, channel_idx);

	int res;
	res = mcdma_mm2s_busy_wait(device);
	if (res) {
		HARU_ERROR("%s", "mm2s query transfer failed.");
		return -1;
	}
	HARU_LOG("%s", "mm2s query transfer done.");
	mm2s_common_status(device);
	mm2s_channel_status(device);
	mm2s_bd_status(device->channels[0]);

	res = mcdma_s2mm_busy_wait(device);
	if (res) {
		HARU_ERROR("%s", "s2mm query transfer failed.");
		return -1;
	}
	HARU_LOG("%s", "s2mm query transfer done.");
	s2mm_common_status(device);
	s2mm_channel_status(device);
	s2mm_bd_status(device->channels[0]);

	return 0;
}


void mcdma_config_mm2s_channel(axi_mcdma_t *device, int channel_idx) {
	// Set current descriptor
	// _reg_set(device->v_baseaddr, (AXI_MCDMA_MM2S_CHCURDESC_LSB + AXI_MCDMA_CH_OFFSET*channel_idx), device->channels[channel_idx]->mm2s_curr_bd_addr);
	_reg_set(device->v_baseaddr, AXI_MCDMA_MM2S_CHCURDESC_LSB, device->channels[channel_idx]->mm2s_curr_bd_addr);
	// Channel fetch bit
	_reg_set(device->v_baseaddr, (AXI_MCDMA_MM2S_CHCR + AXI_MCDMA_CH_OFFSET*channel_idx), AXI_MCDMA_MM2S_CHRS);

	HARU_LOG("Writing mm2s configuration to addr 0x%08x", device->p_baseaddr + AXI_MCDMA_CH_OFFSET*channel_idx);
	HARU_LOG("reg@0x%03x : 0x%08x (current bd)", AXI_MCDMA_MM2S_CHCURDESC_LSB + AXI_MCDMA_CH_OFFSET*channel_idx, device->channels[channel_idx]->mm2s_curr_bd_addr);
	HARU_LOG("reg@0x%03x : 0x%08x (channel %d fetch)", AXI_MCDMA_MM2S_CHCR + AXI_MCDMA_CH_OFFSET*channel_idx, AXI_MCDMA_MM2S_CHRS, channel_idx);
}

void mcdma_config_s2mm_channel(axi_mcdma_t *device, int channel_idx) {
	// Set current descriptor
	_reg_set(device->v_baseaddr, (AXI_MCDMA_S2MM_CHCURDESC_LSB + AXI_MCDMA_CH_OFFSET*channel_idx), device->channels[channel_idx]->s2mm_curr_bd_addr);
	// Channel fetch bit
	_reg_set(device->v_baseaddr, (AXI_MCDMA_S2MM_CHCR + AXI_MCDMA_CH_OFFSET*channel_idx), AXI_MCDMA_S2MM_CHRS);

	HARU_LOG("Writing s2mm configuration to addr 0x%08x", device->p_baseaddr + AXI_MCDMA_CH_OFFSET*channel_idx);
	HARU_LOG("reg@0x%03x : 0x%08x (current bd)", AXI_MCDMA_S2MM_CHCURDESC_LSB + AXI_MCDMA_CH_OFFSET*channel_idx, device->channels[channel_idx]->s2mm_curr_bd_addr);
	HARU_LOG("reg@0x%03x : 0x%08x (channel %d fetch)", AXI_MCDMA_S2MM_CHCR + AXI_MCDMA_CH_OFFSET*channel_idx, AXI_MCDMA_S2MM_CHRS, channel_idx);
}
void mcdma_mm2s_start(axi_mcdma_t *device) {
	_reg_set(device->v_baseaddr, AXI_MCDMA_MM2S_CCR, AXI_MCDMA_MM2S_RS);
	HARU_LOG("reg@0x%03x : 0x%08x (run)", AXI_MCDMA_MM2S_CCR, AXI_MCDMA_MM2S_RS);

	uint32_t mm2s_sr = _reg_get(device->v_baseaddr, AXI_MCDMA_MM2S_CSR);
	while (mm2s_sr & AXI_MCDMA_S2MM_HALTED) {
		mm2s_sr = _reg_get(device->v_baseaddr, AXI_MCDMA_MM2S_CSR);
	}

}

void mcdma_s2mm_start(axi_mcdma_t *device) {
	_reg_set(device->v_baseaddr, AXI_MCDMA_S2MM_CCR, AXI_MCDMA_S2MM_RS);
	HARU_LOG("reg@0x%03x : 0x%08x (run)", AXI_MCDMA_S2MM_CCR, AXI_MCDMA_S2MM_RS);

	uint32_t s2mm_sr = _reg_get(device->v_baseaddr, AXI_MCDMA_S2MM_CSR);
	while (s2mm_sr & AXI_MCDMA_S2MM_HALTED) {
		s2mm_sr = _reg_get(device->v_baseaddr, AXI_MCDMA_S2MM_CSR);
	}
}

void mcdma_mm2s_program_tail_bd(axi_mcdma_t *device, int channel_idx) {
	_reg_set(device->v_baseaddr, (AXI_MCDMA_MM2S_CHTAILDESC_LSB + AXI_MCDMA_CH_OFFSET*channel_idx), device->channels[channel_idx]->mm2s_tail_bd_addr);
	HARU_LOG("reg@0x%03x : 0x%08x (mm2s tail bd)", AXI_MCDMA_MM2S_CHTAILDESC_LSB + AXI_MCDMA_CH_OFFSET*channel_idx, device->channels[channel_idx]->mm2s_tail_bd_addr);
}

void mcdma_s2mm_program_tail_bd(axi_mcdma_t *device, int channel_idx) {
	_reg_set(device->v_baseaddr, (AXI_MCDMA_S2MM_CHTAILDESC_LSB + AXI_MCDMA_CH_OFFSET*channel_idx), device->channels[channel_idx]->s2mm_tail_bd_addr);
	HARU_LOG("reg@0x%03x : 0x%08x (s2mm tail bd)", AXI_MCDMA_S2MM_CHTAILDESC_LSB + AXI_MCDMA_CH_OFFSET*channel_idx, device->channels[channel_idx]->s2mm_tail_bd_addr);
}

int mcdma_mm2s_busy_wait(axi_mcdma_t *device) {
	// Busy wait
	HARU_LOG("%s", "Waiting for mm2s to go idle.");
	uint32_t mm2s_sr = _reg_get(device->v_baseaddr, AXI_MCDMA_MM2S_CSR);
	while (!(mm2s_sr & AXI_MCDMA_MM2S_IDLE) && !(mm2s_sr & AXI_MCDMA_MM2S_HALTED)) {
		mm2s_sr = _reg_get(device->v_baseaddr, AXI_MCDMA_MM2S_CSR);
	}
	
	if (mm2s_sr & AXI_MCDMA_MM2S_HALTED) {
		mm2s_common_status(device);
		mm2s_channel_status(device);
		mm2s_bd_status(device->channels[0]);

		return -1;
	}

	return 0;
}

int mcdma_s2mm_busy_wait(axi_mcdma_t *device) {
	// Busy wait
	HARU_LOG("%s", "Waiting for s2mm to go idle.");
	uint32_t s2mm_sr = _reg_get(device->v_baseaddr, AXI_MCDMA_S2MM_CSR);
	while (!(s2mm_sr & AXI_MCDMA_S2MM_IDLE) && !(s2mm_sr & AXI_MCDMA_S2MM_HALTED)) {
		s2mm_sr = _reg_get(device->v_baseaddr, AXI_MCDMA_S2MM_CSR);
	}

	if (s2mm_sr & AXI_MCDMA_S2MM_HALTED) {
		s2mm_common_status(device);
		s2mm_channel_status(device);
		s2mm_bd_status(device->channels[0]);

		return -1;
	}

	return 0;
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
	_reg_set(device->v_baseaddr, AXI_MCDMA_S2MM_CCR, AXI_MCDMA_S2MM_RESET);

	// wait for reset
	uint32_t mm2s_ccr = _reg_get(device->v_baseaddr, AXI_MCDMA_MM2S_CCR);
	while (mm2s_ccr & AXI_MCDMA_MM2S_RESET) {
		mm2s_ccr = _reg_get(device->v_baseaddr, AXI_MCDMA_MM2S_CCR);
	}

	HARU_LOG("%s", "mm2s reset done.");

	uint32_t s2mm_ccr = _reg_get(device->v_baseaddr, AXI_MCDMA_S2MM_CCR);
	while (s2mm_ccr & AXI_MCDMA_S2MM_RESET) {
		s2mm_ccr = _reg_get(device->v_baseaddr, AXI_MCDMA_S2MM_CCR);
	}

	HARU_LOG("%s", "s2mm reset done.");

}

void mcdma_mm2s_stop(axi_mcdma_t *device) {
	HARU_LOG("%s", "Stop mm2s MCDMA operations.");
	_reg_set(device->v_baseaddr, AXI_MCDMA_MM2S_CCR, !AXI_MCDMA_MM2S_RS);

	uint32_t mm2s_sr = _reg_get(device->v_baseaddr, AXI_MCDMA_MM2S_CSR);
	while (!(mm2s_sr & AXI_MCDMA_MM2S_HALTED)) {
		mm2s_sr = _reg_get(device->v_baseaddr, AXI_MCDMA_MM2S_CSR);
	}

}

void mcdma_s2mm_stop(axi_mcdma_t *device) {
	HARU_LOG("%s", "Stop s2mm MCDMA operations.");
	_reg_set(device->v_baseaddr, AXI_MCDMA_S2MM_CCR, !AXI_MCDMA_S2MM_RS);

	uint32_t s2mm_sr = _reg_get(device->v_baseaddr, AXI_MCDMA_S2MM_CSR);
	while (!(s2mm_sr & AXI_MCDMA_S2MM_HALTED)) {
		s2mm_sr = _reg_get(device->v_baseaddr, AXI_MCDMA_S2MM_CSR);
	}
}

void mm2s_common_status(axi_mcdma_t *device) {
	uint32_t mm2s_common = _reg_get(device->v_baseaddr, AXI_MCDMA_MM2S_CSR);
	// HARU_STATUS("mm2s common status @ 0x%08x = 0x%08x", device->p_baseaddr, mm2s_common);
	if (mm2s_common & AXI_MCDMA_MM2S_HALTED) {
		HARU_STATUS("%s", "mm2s_common: halted");
	}
	if (mm2s_common & AXI_MCDMA_MM2S_IDLE) {
		HARU_STATUS("%s", "mm2s_common: idle");
	}

	uint32_t mm2s_ch_prog = _reg_get(device->v_baseaddr, AXI_MCDMA_MM2S_CHSER);
	HARU_STATUS("mm2s_ch_prog: 0x%08x", mm2s_ch_prog);

	uint32_t mm2s_error = _reg_get(device->v_baseaddr, AXI_MCDMA_MM2S_ERR);
	// HARU_STATUS("mm2s error status = 0x%08x", mm2s_error);
	if (mm2s_error & AXI_MCDMA_MM2S_SG_DEC_ERR) {
		HARU_ERROR("%s", "mm2s_err: SGDecErr");
	}
	if (mm2s_error & AXI_MCDMA_MM2S_SG_INT_ERR) {
		HARU_ERROR("%s", "mm2s_err: SGIntErr");
	}
	if (mm2s_error & AXI_MCDMA_MM2S_SG_SLV_ERR) {
		HARU_ERROR("%s", "mm2s_err: SGSlvErr");
	}
	if (mm2s_error & AXI_MCDMA_MM2S_DMA_DEC_ERR) {
		HARU_ERROR("%s", "mm2s_err:  DMA Dec Err ");
	}
	if (mm2s_error & AXI_MCDMA_MM2S_DMA_SLV_ERR) {
		HARU_ERROR("%s", "mm2s_err: DMA SLv Err");
	}
	if (mm2s_error & AXI_MCDMA_MM2S_DMA_INTR_ERR) {
		HARU_ERROR("%s", "mm2s_err: DMA Intr Err");
	}
}

void s2mm_common_status(axi_mcdma_t *device) {
	uint32_t s2mm_common = _reg_get(device->v_baseaddr, AXI_MCDMA_S2MM_CSR);
	// HARU_STATUS("s2mm common status = 0x%08x", s2mm_common);
	if (s2mm_common & AXI_MCDMA_S2MM_HALTED) {
		HARU_STATUS("%s", "s2mm_common: halted");
	}
	if (s2mm_common & AXI_MCDMA_S2MM_IDLE) {
		HARU_STATUS("%s", "s2mm_common: idle");
	}

	uint32_t s2mm_error = _reg_get(device->v_baseaddr, AXI_MCDMA_S2MM_ERR);
	// HARU_STATUS("s2mm error status = 0x%08x", s2mm_error);
	if (s2mm_error & AXI_MCDMA_S2MM_SG_DEC_ERR) {
		HARU_ERROR("%s", "s2mm_err: SGDecErr");
	}
	if (s2mm_error & AXI_MCDMA_S2MM_SG_INT_ERR) {
		HARU_ERROR("%s", "s2mm_err: SGIntErr");
	}
	if (s2mm_error & AXI_MCDMA_S2MM_SG_SLV_ERR) {
		HARU_ERROR("%s", "s2mm_err: SGSlvErr");
	}
	if (s2mm_error & AXI_MCDMA_S2MM_DMA_DEC_ERR) {
		HARU_ERROR("%s", "s2mm_err: DMA Dec Err ");
	}
	if (s2mm_error & AXI_MCDMA_S2MM_DMA_SLV_ERR) {
		HARU_ERROR("%s", "s2mm_err: DMA SLv Err");
	}
	if (s2mm_error & AXI_MCDMA_S2MM_DMA_INTR_ERR) {
		HARU_ERROR("%s", "s2mm_err: DMA Intr Err");
	}

	uint32_t s2mm_ch_prog = _reg_get(device->v_baseaddr, AXI_MCDMA_S2MM_CHSER);
	HARU_STATUS("s2mm_ch_prog: 0x%08x", s2mm_ch_prog);
}

void mm2s_channel_status(axi_mcdma_t *device) {
	for (int i = 0; i < NUM_CHANNELS; i ++) {
		uint32_t ch_mm2s_status = _reg_get(device->v_baseaddr, (AXI_MCDMA_MM2S_CHSR + AXI_MCDMA_CH_OFFSET*i));
		// HARU_STATUS("ch1_mm2s_status = 0x%08x", ch1_mm2s_status);
		if (ch_mm2s_status & AXI_MCDMA_CH_IDLE) {
			HARU_STATUS("ch%d_mm2s_status: Idle (Queue Empty) ", i);
		}
		if (ch_mm2s_status & AXI_MCDMA_CH_ERR_OTH_CH) {
			HARU_ERROR("ch%d_mm2s_status: Err_on_other_ch_irq ", i);
		}
		if (ch_mm2s_status & AXI_MCDMA_CH_IOC_IRQ) {
			HARU_STATUS("ch%d_mm2s_status: IOC_Irq", i);
		}
		if (ch_mm2s_status & AXI_MCDMA_CH_DLY_IRQ) {
			HARU_STATUS("ch%d_mm2s_status: DlyIrq", i);
		}
		if (ch_mm2s_status & AXI_MCDMA_CH_ERR_IRQ) {
			HARU_ERROR("ch%d_mm2s_status: Err Irq ", i);
		}
	}
}

void mm2s_clear_channel_status(axi_mcdma_t *device) {
	uint32_t mm2s_sr = _reg_get(device->v_baseaddr, AXI_MCDMA_MM2S_CHSR);
	_reg_set(device->v_baseaddr, AXI_MCDMA_MM2S_CHSR, (mm2s_sr & 0x17));
	HARU_LOG("reg@0x%03x : 0x%08x (clear channel status)", AXI_MCDMA_MM2S_CHSR, (mm2s_sr & 0x17));
}

void s2mm_channel_status(axi_mcdma_t *device) {
	for (int i = 0; i < NUM_CHANNELS; i ++) {
		uint32_t ch_s2mm_status = _reg_get(device->v_baseaddr, (AXI_MCDMA_S2MM_CHSR + AXI_MCDMA_CH_OFFSET*i));
		// HARU_STATUS("ch1_s2mm_status = 0x%08x", ch1_mm2s_status);
		if (ch_s2mm_status & AXI_MCDMA_CH_IDLE) {
			HARU_STATUS("ch%d_s2mm_status: Idle (Queue Empty) ", i);
		}
		if (ch_s2mm_status & AXI_MCDMA_CH_ERR_OTH_CH) {
			HARU_ERROR("ch%d_s2mm_status: Err_on_other_ch_irq ", i);
		}
		if (ch_s2mm_status & AXI_MCDMA_CH_IOC_IRQ) {
			HARU_STATUS("ch%d_s2mm_status: IOC_Irq", i);
		}
		if (ch_s2mm_status & AXI_MCDMA_CH_DLY_IRQ) {
			HARU_STATUS("ch%d_s2mm_status: DlyIrq", i);
		}
		if (ch_s2mm_status & AXI_MCDMA_CH_ERR_IRQ) {
			HARU_ERROR("ch%d_s2mm_status: Err Irq ", i);
		}
	}
}

void mm2s_bd_status(axi_mcdma_channel_t *channel) {
	uint32_t mm2s_bd_status = _reg_get(channel->mm2s_bd_chain->v_bd_addr, AXI_MCDMA_MM2S_BD_STATUS);
	// HARU_STATUS("mm2s bd status = 0x%08x", mm2s_bd_status);
	HARU_STATUS("ch%d_mm2s_bd_status: %d bytes transferred", channel->channel_id, mm2s_bd_status & AXI_MCDMA_MM2S_BD_SBYTE_MASK);
	if (mm2s_bd_status & AXI_MCDMA_MM2S_BD_DMA_INT_ERR) {
		HARU_ERROR("ch%d_mm2s_bd_status: DMA Int Err", channel->channel_id);
	}
	if (mm2s_bd_status & AXI_MCDMA_MM2S_BD_DMA_SLV_ERR) {
		HARU_ERROR("ch%d_mm2s_bd_status: DMA Slave Err ", channel->channel_id);
	}
	if (mm2s_bd_status & AXI_MCDMA_MM2S_BD_DMA_DEC_ERR) {
		HARU_ERROR("ch%d_mm2s_bd_status: DMA Dec Err", channel->channel_id);
	}
	if (mm2s_bd_status & AXI_MCDMA_MM2S_BD_DMA_COMPLETED) {
		HARU_STATUS("ch%d_mm2s_bd_status: Completed", channel->channel_id);
	}
}

void s2mm_bd_status(axi_mcdma_channel_t *channel) {
	uint32_t s2mm_bd_status = _reg_get(channel->s2mm_bd_chain->v_bd_addr, AXI_MCDMA_S2MM_BD_STATUS);
	// HARU_STATUS("s2mm bd status = 0x%08x", S2MM_bd_status);
	HARU_STATUS("ch%d_s2mm_bd_status: %d bytes transferred", channel->channel_id, s2mm_bd_status & AXI_MCDMA_S2MM_BD_SBYTE_MASK);
	if (s2mm_bd_status & AXI_MCDMA_S2MM_BD_DMA_INT_ERR) {
		HARU_ERROR("ch%d_s2mm_bd_status: DMA Int Err", channel->channel_id);
	}
	if (s2mm_bd_status & AXI_MCDMA_S2MM_BD_DMA_SLV_ERR) {
		HARU_ERROR("ch%d_s2mm_bd_status: DMA Slave Err ", channel->channel_id);
	}
	if (s2mm_bd_status & AXI_MCDMA_S2MM_BD_DMA_DEC_ERR) {
		HARU_ERROR("ch%d_s2mm_bd_status: DMA Dec Err", channel->channel_id);
	}
	if (s2mm_bd_status & AXI_MCDMA_S2MM_BD_DMA_COMPLETED) {
		HARU_STATUS("ch%d_s2mm_bd_status: Completed", channel->channel_id);
	}
	if (s2mm_bd_status & AXI_MCDMA_S2MM_BD_DMA_RXSOF) {
		HARU_STATUS("ch%d_s2mm_bd_status: SOF", channel->channel_id);
	}
	if (s2mm_bd_status & AXI_MCDMA_S2MM_BD_DMA_RXEOF) {
		HARU_STATUS("ch%d_s2mm_bd_status: EOF", channel->channel_id);
	}
}

/*
author: Katelyn Mak
date created: 15/4/2024

This module is used to control the AXI MCDMA.

*/
#include "misc.h"
#include "error.h"
#include "axi_mcdma.h"
#include "mcdma_test.h"

#include <stdio.h> // todo: remove this once debugging is finished
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <string.h>

int32_t axi_mcdma_init(axi_mcdma_t *device, uint32_t baseaddr, uint32_t src_addr, uint32_t dst_addr, uint32_t size) {
	/*** Memory map address space ***/
	// Open /dev/mem for memory mapping
	int32_t dev_fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (dev_fd < 0) {
		ERROR("%s", "Failed to open /dev/mem.");
		return -1;
	}

	// initialise the axi dma control space
	device->p_baseaddr = baseaddr;
	device->v_baseaddr = (uint32_t *) mmap(NULL, (uint32_t) size, PROT_READ | PROT_WRITE, MAP_SHARED, dev_fd, (uint32_t) baseaddr); 
	if (device->v_baseaddr == MAP_FAILED) {
		ERROR("%s", "control space map failed.");
		close(dev_fd);
		return -1;
	}

	// Reset device
	mcdma_reset(device);

	// intialise mm2s buffer space
	device->p_buffer_src_addr = src_addr;
	device->v_buffer_src_addr = (uint32_t *) mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, dev_fd, src_addr); 
	if (device->v_buffer_src_addr == MAP_FAILED) {
		ERROR("%s", "buffer src address map failed.");
		close(dev_fd);
		return -1;
	}

	// initialise s2mm buffer space
	device->p_buffer_dst_addr = dst_addr;
	device->v_buffer_dst_addr = (uint32_t *) mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, dev_fd, dst_addr); 
	if (device->v_buffer_dst_addr == MAP_FAILED) {
		ERROR("%s", "buffer dst address map failed.");
		close(dev_fd);
		return -1;
	}

	close(dev_fd);

	/*** Configure channel struct ***/
	device->num_channels = NUM_CHANNELS;
	device->size = size;
	mcdma_channel_t *channel;
	for (int i = 0; i < NUM_CHANNELS; i++) {
		channel = (mcdma_channel_t *) malloc(sizeof( mcdma_channel_t ));
		MALLOC_CHK(channel);
		device->channels[i] = channel;

		// Initialise channel
		// Each channel is given an equal and fixed portion of the source/destination buffers.
		uint32_t buf_size = (uint32_t) size/NUM_CHANNELS;
		channel->channel_id = i;
		channel->p_buf_src_addr = device->p_buffer_src_addr + buf_size*i;
		channel->v_buf_src_addr = device->v_buffer_src_addr + buf_size*i;
		channel->p_buf_dst_addr = device->p_buffer_dst_addr + buf_size*i;
		channel->v_buf_dst_addr = device->v_buffer_dst_addr + buf_size*i;
		channel->buf_size = buf_size;

		LOG_DEBUG("Configuring channel %d struct", i);
		LOG_DEBUG("ch%d_p_buf_src_addr : 0x%08x", i, channel->p_buf_src_addr);
		LOG_DEBUG("ch%d_p_buf_dst_addr : 0x%08x", i, channel->p_buf_dst_addr);
		LOG_DEBUG("ch%d_size : 0x%08x", i, channel->buf_size);
	}
}

/*
	Resets AXI MCDMA with the mm2s common control register. Resetting with the mm2s ccr resets both mm2s and s2mm.
*/
void mcdma_reset(axi_mcdma_t *device) {
	LOG_DEBUG("%s", "Reset MCDMA.");
	_reg_set(device->v_baseaddr, AXI_MCDMA_MM2S_CCR, AXI_MCDMA_MM2S_RESET);

	// wait for reset
	uint32_t mm2s_ccr = _reg_get(device->v_baseaddr, AXI_MCDMA_MM2S_CCR);
	while (mm2s_ccr & AXI_MCDMA_MM2S_RESET) {
		mm2s_ccr = _reg_get(device->v_baseaddr, AXI_MCDMA_MM2S_CCR);
	}
}
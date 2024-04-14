/*
author: Katelyn Mak
date created: 7/4/2024

This module writes and reads to the AXI MCDMA IP module a random payload.
The AXI MCDMA module should have the master mm2s axi stream connected to the slave mm2s axi stream in a loop.

*/
#include "misc.h"
#include "axi_mcdma.h"
#include "mcdma_test.h"

#include <stdio.h> // todo: remove this once debugging is finished
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <string.h>

int mcdma_test(int payload_length);
int config_mcdma(mcdma_device_t *device, int payload_length);
int config_mcdma_bd_chain(mcdma_channel_t *channel, uint32_t mm2s_bd_addr, uint32_t s2mm_bd_addr);
int config_mcdma_mm2s(mcdma_device_t *device);
int config_mcdma_s2mm(mcdma_device_t *device);
int run_mcdma(mcdma_device_t *device);
int verify_payload(mcdma_device_t *device, int payload_length);
void free_device(mcdma_device_t *device);

// For one channel, one bd
void mm2s_common_status(mcdma_device_t *device);
void mm2s_channel_status(mcdma_device_t *device);
void mm2s_bd_status(mcdma_channel_t *channel);
void s2mm_common_status(mcdma_device_t *device);
void s2mm_channel_status(mcdma_device_t *device);
void s2mm_bd_status(mcdma_channel_t *channel);

int mcdma_test (int payload_length) {
	fprintf(stderr, "============================= MCDMA FIFO TEST =============================\n");

	// Create device struct
	mcdma_device_t *device = malloc(sizeof( mcdma_device_t ));
	MALLOC_CHK(device);

	int res;
	res = config_mcdma(device, payload_length);
	if (res) {
		ERROR("%s", "Could not config MCDMA.");
		free_device(device);
		return -1;
	}

	res = run_mcdma(device);
	if (res) {
		ERROR("%s", "Could not run MCDMA.\n");
		free_device(device);
		return -1;
	}

	res = verify_payload(device, payload_length);
	if (res) {
		ERROR("%s", "Could not verify payload.\n");
	}

	free_device(device);

	INFO("%s", "we chillin");
	return 0;
}

int config_mcdma(mcdma_device_t * device, int payload_length) {
	/*** Memory map address space ***/
	// Open /dev/mem for memory mapping
	int32_t dev_fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (dev_fd < 0) {
		ERROR("%s", "Failed to open /dev/mem.");
		return -1;
	}

	// initialise the axi dma control space
	device->p_baseaddr = AXI_MCDMA_BUF_ADDR_BASE;
	device->v_baseaddr = (uint32_t *) mmap(NULL, (uint32_t) AXI_MCDMA_BUF_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, dev_fd, (uint32_t) AXI_MCDMA_BUF_ADDR_BASE); 
	if (device->v_baseaddr == MAP_FAILED) {
		ERROR("%s", "control space map failed.");
		close(dev_fd);
		return -1;
	}

	// intialise mm2s buffer space
	device->p_buffer_src_addr = AXI_MCDMA_BUF_SRC_ADDR;
	device->v_buffer_src_addr = (uint32_t *) mmap(NULL, AXI_MCDMA_BUF_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, dev_fd, AXI_MCDMA_BUF_SRC_ADDR); 
	if (device->v_buffer_src_addr == MAP_FAILED) {
		ERROR("%s", "buffer src address map failed.");
		close(dev_fd);
		return -1;
	}

	// initialise s2mm buffer space
	device->p_buffer_dst_addr = AXI_MCDMA_BUF_DST_ADDR;
	device->v_buffer_dst_addr = (uint32_t *) mmap(NULL, AXI_MCDMA_BUF_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, dev_fd, AXI_MCDMA_BUF_DST_ADDR); 
	if (device->v_buffer_dst_addr == MAP_FAILED) {
		ERROR("%s", "buffer dst address map failed.");
		close(dev_fd);
		return -1;
	}

	close(dev_fd);

	/*** Configure channel struct ***/
	device->num_channels = NUM_CHANNELS;
	device->size = AXI_MCDMA_BUF_SIZE;
	mcdma_channel_t *channel;
	for (int i = 0; i < NUM_CHANNELS; i++) {
		channel = (mcdma_channel_t *) malloc(sizeof( mcdma_channel_t ));
		MALLOC_CHK(channel);
		device->channels[i] = channel;

		// Initialise channel
		uint32_t buf_size = (uint32_t) (payload_length * sizeof ( uint32_t ));
		channel->channel_id = i;
		channel->p_buf_src_addr = device->p_buffer_src_addr + buf_size*i;
		channel->v_buf_src_addr = device->v_buffer_src_addr + buf_size*i;
		channel->p_buf_dst_addr = device->p_buffer_dst_addr + buf_size*i;
		channel->v_buf_dst_addr = device->v_buffer_dst_addr + buf_size*i;
		channel->buf_size = buf_size;

		INFO("Configuring channel %d struct", i);
		INFO("ch%d_p_buf_src_addr : 0x%08x", i, channel->p_buf_src_addr);
		INFO("ch%d_p_buf_dst_addr : 0x%08x", i, channel->p_buf_dst_addr);
		INFO("ch%d_size : 0x%08x", i, channel->buf_size);
	}

	/*** Initialise buffer space ***/
	INFO("%s", "Creating random payload:");
	// Create NUM_CHANNEL random payloads
	srand(PAYLOAD_SEED);
	uint32_t payload[NUM_CHANNELS][payload_length];
	for (int i = 0; i < NUM_CHANNELS; i++) {
		INFO("Channel %d payload:", i);
		for (int j = 0; j < payload_length; j++) {
			payload[i][j] = (uint32_t) rand();
			INFO("%d: 0x%08x", j, payload[i][j]);
		}
	}

	// Copy payload to mm2s buffer space
	memcpy(device->v_buffer_src_addr, (void *) payload, payload_length * sizeof(uint32_t) * NUM_CHANNELS);
	INFO("%s", "Copied payload to src buffer.");

	// Clear dst buffer
	memset(device->v_buffer_dst_addr, 0, payload_length * sizeof(uint32_t) * NUM_CHANNELS);

	// Initialise bd chain buffer space
	int res;

	for (int i = 0; i < NUM_CHANNELS; i++) {
		res = config_mcdma_bd_chain(device->channels[i], AXI_MCDMA_MM2S_BD_CHAIN_ADDR + AXI_MCDMA_BD_OFFSET*i, AXI_MCDMA_S2MM_BD_CHAIN_ADDR + AXI_MCDMA_BD_OFFSET*i);
		if (res) {
			ERROR("%s", "failed to config bd chain.");
			return -1;
		}
	}

	INFO("configd %d channels.", NUM_CHANNELS);

	return 0;
}

int config_mcdma_bd_chain(mcdma_channel_t *channel, uint32_t mm2s_bd_addr, uint32_t s2mm_bd_addr) {
	INFO("Configuring bd chain for channel %d", channel->channel_id);
	// Open /dev/mem for memory mapping
	int32_t dev_fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (dev_fd < 0) {
		ERROR("%s", "Failed to open /dev/mem.");
		return -1;
	}

	mcdma_bd_t *mm2s_bd = (mcdma_bd_t *) malloc(sizeof( mcdma_bd_t ));
	MALLOC_CHK(mm2s_bd);
	channel->mm2s_bd_chain = mm2s_bd;

	mcdma_bd_t *s2mm_bd = (mcdma_bd_t *) malloc(sizeof( mcdma_bd_t ));
	MALLOC_CHK(s2mm_bd);
	channel->s2mm_bd_chain = s2mm_bd;

	channel->mm2s_curr_bd_addr = mm2s_bd_addr;
	channel->mm2s_tail_bd_addr = mm2s_bd_addr;

	// initialise mm2s bd chain space
	mm2s_bd->p_bd_addr = mm2s_bd_addr;
	mm2s_bd->v_bd_addr = (uint32_t *) mmap(NULL, AXI_MCDMA_BD_OFFSET, PROT_READ | PROT_WRITE, MAP_SHARED, dev_fd, mm2s_bd_addr); 
	if (mm2s_bd->v_bd_addr == MAP_FAILED) {
		ERROR("%s", "mm2s bd chain address map failed.");
		close(dev_fd);
		return -1;
	}

	mm2s_bd->next_mcdma_bd = NULL;
	mm2s_bd->next_bd_addr = channel->mm2s_tail_bd_addr;
	mm2s_bd->buffer_addr = channel->p_buf_src_addr;
	mm2s_bd->buffer_length = channel->buf_size;
	mm2s_bd->sof = 1;
	mm2s_bd->eof = 1;
	mm2s_bd->tid = 0;

	INFO("%s", "mm2s_bd fields:");
	INFO("> next_bd_addr: 0x%08x", mm2s_bd->next_bd_addr);
	INFO("> buffer_addr: 0x%08x", mm2s_bd->buffer_addr);
	INFO("> buffer_length: 0x%08x", mm2s_bd->buffer_length);
	INFO("> sof: %d", mm2s_bd->sof);
	INFO("> eof: %d", mm2s_bd->eof);
	INFO("> tid: %d", mm2s_bd->tid);

	_reg_set(mm2s_bd->v_bd_addr, AXI_MCDMA_MM2S_BD_NEXT_DESC_LSB, mm2s_bd->next_bd_addr);
	_reg_set(mm2s_bd->v_bd_addr, AXI_MCDMA_MM2S_BD_BUF_ADDR_LSB, mm2s_bd->buffer_addr);
	uint32_t control = (uint32_t) (mm2s_bd->sof << 31) | (uint32_t) (mm2s_bd->eof << 30) | mm2s_bd->buffer_length;
	_reg_set(mm2s_bd->v_bd_addr, AXI_MCDMA_MM2S_BD_CONTROL, control);
	uint32_t control_sideband = (uint32_t) (mm2s_bd->tid << 31);
	_reg_set(mm2s_bd->v_bd_addr, AXI_MCDMA_MM2S_BD_CONTROL_SIDEBAND, control_sideband);
	_reg_set(mm2s_bd->v_bd_addr, AXI_MCDMA_MM2S_BD_STATUS, 0x00000000);

	INFO("Writing mm2s_bd fields to addr 0x%08x:", mm2s_bd->p_bd_addr);
	INFO("reg@0x%03x : 0x%08x (next bd)", mm2s_bd->p_bd_addr + AXI_MCDMA_MM2S_BD_NEXT_DESC_LSB, mm2s_bd->next_bd_addr);
	INFO("reg@0x%03x : 0x%08x (buf addr)", mm2s_bd->p_bd_addr + AXI_MCDMA_MM2S_BD_BUF_ADDR_LSB, mm2s_bd->buffer_addr);
	INFO("reg@0x%03x : 0x%08x (bd sof, eof, buf length)", mm2s_bd->p_bd_addr + AXI_MCDMA_MM2S_BD_CONTROL, control);
	INFO("reg@0x%03x : 0x%08x (bd tid)", mm2s_bd->p_bd_addr + AXI_MCDMA_MM2S_BD_CONTROL_SIDEBAND, control_sideband);
	INFO("reg@0x%03x : 0x%08x (bd status)", mm2s_bd->p_bd_addr + AXI_MCDMA_MM2S_BD_STATUS, 0x00000000);

	// initialise s2mm bd chain space
	s2mm_bd->p_bd_addr = s2mm_bd_addr;
	s2mm_bd->v_bd_addr = (uint32_t *) mmap(NULL, AXI_MCDMA_BD_OFFSET, PROT_READ | PROT_WRITE, MAP_SHARED, dev_fd, s2mm_bd_addr); 
	if (s2mm_bd->v_bd_addr == MAP_FAILED) {
		ERROR("%s", "s2mm bd chain address map failed.");
		close(dev_fd);
		return -1;
	}

	channel->s2mm_curr_bd_addr = s2mm_bd_addr;
	channel->s2mm_tail_bd_addr = s2mm_bd_addr;

	s2mm_bd->next_mcdma_bd = NULL;
	s2mm_bd->next_bd_addr = s2mm_bd->p_bd_addr;
	s2mm_bd->buffer_addr = channel->p_buf_dst_addr;
	s2mm_bd->buffer_length = (uint32_t) 2*channel->buf_size;

	INFO("%s", "s2mm_bd fields:");
	INFO("> next_bd_addr: 0x%08x", s2mm_bd->next_bd_addr);
	INFO("> buffer_addr: 0x%08x", s2mm_bd->buffer_addr);
	INFO("> buffer_length: 0x%08x", s2mm_bd->buffer_length);

	_reg_set(s2mm_bd->v_bd_addr, AXI_MCDMA_S2MM_BD_NEXT_DESC_LSB, s2mm_bd->next_bd_addr);
	_reg_set(s2mm_bd->v_bd_addr, AXI_MCDMA_S2MM_BD_BUF_ADDR_LSB, s2mm_bd->buffer_addr);
	_reg_set(s2mm_bd->v_bd_addr, AXI_MCDMA_S2MM_BD_CONTROL, s2mm_bd->buffer_length);
	_reg_set(s2mm_bd->v_bd_addr, AXI_MCDMA_S2MM_BD_STATUS, 0x00000000);

	INFO("Writing s2mm_bd fields to addr 0x%08x:", s2mm_bd->p_bd_addr);
	INFO("reg@0x%03x : 0x%08x (next bd)", s2mm_bd->p_bd_addr + AXI_MCDMA_S2MM_BD_NEXT_DESC_LSB, s2mm_bd->next_bd_addr);
	INFO("reg@0x%03x : 0x%08x (buf addr)", s2mm_bd->p_bd_addr + AXI_MCDMA_S2MM_BD_BUF_ADDR_LSB, s2mm_bd->buffer_addr);
	INFO("reg@0x%03x : 0x%08x (buf length)", s2mm_bd->p_bd_addr + AXI_MCDMA_S2MM_BD_CONTROL, s2mm_bd->buffer_length);
	INFO("reg@0x%03x : 0x%08x (bd status)", s2mm_bd->p_bd_addr + AXI_MCDMA_S2MM_BD_STATUS, 0x00000000);

	close(dev_fd);

	return 0;
}

int config_mcdma_mm2s(mcdma_device_t *device) {
	// enable channels
	uint32_t channel_en_reg = 0x0;
	for (int i = 0; i < device->num_channels; i++) {
		channel_en_reg |= 0x1 << i;
	}
	_reg_set(device->v_baseaddr, AXI_MCDMA_MM2S_CHEN, channel_en_reg);
	INFO("reg@0x%03x : 0x%08x (channel enable)", AXI_MCDMA_MM2S_CHEN, channel_en_reg);

	for (int i = 0; i < NUM_CHANNELS; i++) {
		// Set current descriptor
		_reg_set(device->v_baseaddr, (AXI_MCDMA_MM2S_CHCURDESC_LSB + AXI_MCDMA_CH_OFFSET*i), device->channels[i]->mm2s_curr_bd_addr);
		// Channel fetch bit
		_reg_set(device->v_baseaddr, (AXI_MCDMA_MM2S_CHCR + AXI_MCDMA_CH_OFFSET*i), AXI_MCDMA_MM2S_CHRS);

		INFO("Writing mm2s configuration to addr 0x%08x", device->p_baseaddr + AXI_MCDMA_CH_OFFSET*i);
		INFO("reg@0x%03x : 0x%08x (current bd)", AXI_MCDMA_MM2S_CHCURDESC_LSB + AXI_MCDMA_CH_OFFSET*i, device->channels[i]->mm2s_curr_bd_addr);
		INFO("reg@0x%03x : 0x%08x (channel %d fetch)", AXI_MCDMA_MM2S_CHCR + AXI_MCDMA_CH_OFFSET*i, AXI_MCDMA_MM2S_CHRS, i);
	}
	
	return 0;
}

int config_mcdma_s2mm(mcdma_device_t *device) {
	// reset mcdma s2mm
	// INFO("%s", "Reset s2mm.");
	// _reg_set(device->v_baseaddr, AXI_MCDMA_S2MM_CCR, AXI_MCDMA_S2MM_RESET);

	// wait for reset
	// uint32_t s2mm_ccr = _reg_get(device->v_baseaddr, AXI_MCDMA_S2MM_CCR);
	// while (s2mm_ccr & AXI_MCDMA_S2MM_RESET) {
	// 	s2mm_ccr = _reg_get(device->v_baseaddr, AXI_MCDMA_S2MM_CCR);
	// }

	// enable channels
	uint32_t channel_en_reg = 0x0;
	for (int i = 0; i < device->num_channels; i++) {
		channel_en_reg |= 0x1 << i;
	}
	_reg_set(device->v_baseaddr, AXI_MCDMA_S2MM_CHEN, channel_en_reg);

	for (int i = 0; i < NUM_CHANNELS; i++) {
		// Set current descriptor
		_reg_set(device->v_baseaddr, (AXI_MCDMA_S2MM_CHCURDESC_LSB + AXI_MCDMA_CH_OFFSET*i), device->channels[i]->s2mm_curr_bd_addr);
		// Channel fetch bit
		_reg_set(device->v_baseaddr, (AXI_MCDMA_S2MM_CHCR + AXI_MCDMA_CH_OFFSET*i), AXI_MCDMA_S2MM_CHRS);

		INFO("Writing s2mm configuration to addr 0x%08x", device->p_baseaddr + AXI_MCDMA_CH_OFFSET*i);
		INFO("reg@0x%03x : 0x%08x (channel enable)", AXI_MCDMA_S2MM_CHEN, channel_en_reg);
		INFO("reg@0x%03x : 0x%08x (current bd)", AXI_MCDMA_S2MM_CHCURDESC_LSB + AXI_MCDMA_CH_OFFSET*i, device->channels[i]->s2mm_curr_bd_addr);
		INFO("reg@0x%03x : 0x%08x (channel %d fetch)", AXI_MCDMA_S2MM_CHCR + AXI_MCDMA_CH_OFFSET*i, AXI_MCDMA_S2MM_CHRS, i);
	}
	return 0;
}

/*
	Programming sequence of transfer:
	1. Reset MCDMA
	2. Program s2mm
	3. Start s2mm
	4. Program mm2s
	5. Start mm2s
	6. Wait for mm2s transfer to complete
	7. Wait for s2mm transfer to complete
*/
int run_mcdma(mcdma_device_t *device) {
	int res;

	// reset mcdmad
	INFO("%s", "Reset MCDMA.");
	_reg_set(device->v_baseaddr, AXI_MCDMA_MM2S_CCR, AXI_MCDMA_MM2S_RESET);

	// wait for reset
	uint32_t mm2s_ccr = _reg_get(device->v_baseaddr, AXI_MCDMA_MM2S_CCR);
	while (mm2s_ccr & AXI_MCDMA_MM2S_RESET) {
		mm2s_ccr = _reg_get(device->v_baseaddr, AXI_MCDMA_MM2S_CCR);
	}

	// program s2mm
	res = config_mcdma_s2mm(device);
	if (res) {
		ERROR("%s", "failed to config s2mm.");
		return -1;
	}
	INFO("%s", "configd s2mm.");

	// Start s2mm
	_reg_set(device->v_baseaddr, AXI_MCDMA_S2MM_CCR, AXI_MCDMA_S2MM_RS);
	INFO("Start s2mm @ 0x%03x", AXI_MCDMA_S2MM_CCR);

	// Program s2mm tail descriptor
	for (int i = 0; i < NUM_CHANNELS; i++) {
		_reg_set(device->v_baseaddr, (AXI_MCDMA_S2MM_CHTAILDESC_LSB + AXI_MCDMA_CH_OFFSET*i), device->channels[i]->s2mm_tail_bd_addr);
		INFO("Programmed s2mm tail bd 0x%03x : 0x%08x", AXI_MCDMA_S2MM_CHTAILDESC_LSB + AXI_MCDMA_CH_OFFSET*i, device->channels[i]->s2mm_tail_bd_addr);
	}

	// program mm2s
	res = config_mcdma_mm2s(device);
	if (res) {

	}
	INFO("%s", "configd mm2s.");

	// Start mm2
	_reg_set(device->v_baseaddr, AXI_MCDMA_MM2S_CCR, AXI_MCDMA_MM2S_RS);
	INFO("Start mm2s @ 0x%03x", AXI_MCDMA_MM2S_CCR);

	// Program mm2s tail descriptor
	for (int i = 0; i < NUM_CHANNELS; i++) {
		_reg_set(device->v_baseaddr, (AXI_MCDMA_MM2S_CHTAILDESC_LSB + AXI_MCDMA_CH_OFFSET*i), device->channels[i]->mm2s_tail_bd_addr);
		INFO("Programmed mm2s tail bd 0x%03x : 0x%08x", AXI_MCDMA_MM2S_CHTAILDESC_LSB + AXI_MCDMA_CH_OFFSET*i, device->channels[i]->mm2s_tail_bd_addr);
	}

	// Busy wait
	uint32_t mm2s_sr = _reg_get(device->v_baseaddr, AXI_MCDMA_MM2S_CSR);
	while (!(mm2s_sr & AXI_MCDMA_MM2S_IDLE)) {
		mm2s_sr = _reg_get(device->v_baseaddr, AXI_MCDMA_MM2S_CSR);
	}
	INFO("%s", "mm2s transfer done.");

	mm2s_common_status(device);
	mm2s_channel_status(device);
	for (int i = 0; i < NUM_CHANNELS; i++) {
		mm2s_bd_status(device->channels[i]);
	}

	// Busy wait
	uint32_t s2mm_sr = _reg_get(device->v_baseaddr, AXI_MCDMA_S2MM_CSR);
	while (!(s2mm_sr & AXI_MCDMA_S2MM_IDLE)) {
		s2mm_sr = _reg_get(device->v_baseaddr, AXI_MCDMA_S2MM_CSR);
	}
	INFO("%s", "s2mm transfer done.");

	s2mm_common_status(device);
	s2mm_channel_status(device);
	for (int i = 0; i < NUM_CHANNELS; i++) {
		s2mm_bd_status(device->channels[i]);
	}

	return 0;
}

int verify_payload(mcdma_device_t *device, int payload_length) {
	// check the loaded data is correct
	int error = 0;
	uint32_t src_data, dst_data;
	for (int i = 0; i < payload_length*NUM_CHANNELS; i++) {
		src_data = _reg_get(device->v_buffer_src_addr, (uint32_t) i*4);
		dst_data = _reg_get(device->v_buffer_dst_addr, (uint32_t) i*4);
		if (src_data != dst_data) {
			error++;
			ERROR("data mismatch @ payload[%d]: 0x%08x 0x%08x", i, src_data, dst_data);
		}
	}

	if (error == 0) {
		INFO("0/%d data mistaches, Success!", payload_length*NUM_CHANNELS);
	} else {
		ERROR("Oh no! %d/%d data mistaches", error, payload_length*NUM_CHANNELS);
	}

	return 0;

}

void free_device(mcdma_device_t *device) {

	INFO("%s", "Freeing device.");

	munmap(device->v_baseaddr, device->size);
	INFO("%s", "unmapped device.");

	munmap(device->v_buffer_src_addr, device->size);
	INFO("%s", "unmapped src buffer.");

	munmap(device->v_buffer_dst_addr, device->size);
	INFO("%s", "unmapped dst buffer.");

	for (int i = 0; i < NUM_CHANNELS; i++) {
		munmap(device->channels[i]->mm2s_bd_chain->v_bd_addr, AXI_MCDMA_BUF_SIZE);
		munmap(device->channels[i]->s2mm_bd_chain->v_bd_addr, AXI_MCDMA_BUF_SIZE);
	}
	INFO("%s", "unmapped bds.");

	for (int i = 0; i < NUM_CHANNELS; i++) {
		free(device->channels[i]->mm2s_bd_chain);
		free(device->channels[i]->s2mm_bd_chain);
		free(device->channels[i]);
	}
	INFO("%s", "freed bd chains and channels.");

	free(device);
	INFO("%s", "freed device.");
}

void mm2s_common_status(mcdma_device_t *device) {
	uint32_t mm2s_common = _reg_get(device->v_baseaddr, AXI_MCDMA_MM2S_CSR);
	// STATUS("mm2s common status @ 0x%08x = 0x%08x", device->p_baseaddr, mm2s_common);
	if (mm2s_common & AXI_MCDMA_MM2S_HALTED) {
		STATUS("%s", "mm2s_common: halted");
	}
	if (mm2s_common & AXI_MCDMA_MM2S_IDLE) {
		STATUS("%s", "mm2s_common: idle");
	}

	uint32_t mm2s_ch_prog = _reg_get(device->v_baseaddr, AXI_MCDMA_MM2S_CHSER);
	STATUS("mm2s_ch_prog: 0x%08x", mm2s_ch_prog);

	uint32_t mm2s_error = _reg_get(device->v_baseaddr, AXI_MCDMA_MM2S_ERR);
	// STATUS("mm2s error status = 0x%08x", mm2s_error);
	if (mm2s_error & AXI_MCDMA_MM2S_SG_DEC_ERR) {
		ERROR("%s", "mm2s_err: SGDecErr");
	}
	if (mm2s_error & AXI_MCDMA_MM2S_SG_INT_ERR) {
		ERROR("%s", "mm2s_err: SGIntErr");
	}
	if (mm2s_error & AXI_MCDMA_MM2S_SG_SLV_ERR) {
		ERROR("%s", "mm2s_err: SGSlvErr");
	}
	if (mm2s_error & AXI_MCDMA_MM2S_DMA_DEC_ERR) {
		ERROR("%s", "mm2s_err:  DMA Dec Err ");
	}
	if (mm2s_error & AXI_MCDMA_MM2S_DMA_SLV_ERR) {
		ERROR("%s", "mm2s_err: DMA SLv Err");
	}
	if (mm2s_error & AXI_MCDMA_MM2S_DMA_INTR_ERR) {
		ERROR("%s", "mm2s_err: DMA Intr Err");
	}
}

void s2mm_common_status(mcdma_device_t *device) {
	uint32_t s2mm_common = _reg_get(device->v_baseaddr, AXI_MCDMA_S2MM_CSR);
	// STATUS("s2mm common status = 0x%08x", s2mm_common);
	if (s2mm_common & AXI_MCDMA_S2MM_HALTED) {
		STATUS("%s", "s2mm_common: halted");
	}
	if (s2mm_common & AXI_MCDMA_S2MM_IDLE) {
		STATUS("%s", "s2mm_common: idle");
	}

	uint32_t s2mm_error = _reg_get(device->v_baseaddr, AXI_MCDMA_S2MM_ERR);
	// STATUS("s2mm error status = 0x%08x", s2mm_error);
	if (s2mm_error & AXI_MCDMA_S2MM_SG_DEC_ERR) {
		ERROR("%s", "s2mm_err: SGDecErr");
	}
	if (s2mm_error & AXI_MCDMA_S2MM_SG_INT_ERR) {
		ERROR("%s", "s2mm_err: SGIntErr");
	}
	if (s2mm_error & AXI_MCDMA_S2MM_SG_SLV_ERR) {
		ERROR("%s", "s2mm_err: SGSlvErr");
	}
	if (s2mm_error & AXI_MCDMA_S2MM_DMA_DEC_ERR) {
		ERROR("%s", "s2mm_err: DMA Dec Err ");
	}
	if (s2mm_error & AXI_MCDMA_S2MM_DMA_SLV_ERR) {
		ERROR("%s", "s2mm_err: DMA SLv Err");
	}
	if (s2mm_error & AXI_MCDMA_S2MM_DMA_INTR_ERR) {
		ERROR("%s", "s2mm_err: DMA Intr Err");
	}

	uint32_t s2mm_ch_prog = _reg_get(device->v_baseaddr, AXI_MCDMA_S2MM_CHSER);
	STATUS("s2mm_ch_prog: 0x%08x", s2mm_ch_prog);
}

void mm2s_channel_status(mcdma_device_t *device) {
	for (int i = 0; i < NUM_CHANNELS; i ++) {
		uint32_t ch_mm2s_status = _reg_get(device->v_baseaddr, (AXI_MCDMA_MM2S_CHSR + AXI_MCDMA_CH_OFFSET*i));
		// STATUS("ch1_mm2s_status = 0x%08x", ch1_mm2s_status);
		if (ch_mm2s_status & AXI_MCDMA_CH_IDLE) {
			STATUS("ch%d_mm2s_status: Idle (Queue Empty) ", i);
		}
		if (ch_mm2s_status & AXI_MCDMA_CH_ERR_OTH_CH) {
			ERROR("ch%d_mm2s_status: Err_on_other_ch_irq ", i);
		}
		if (ch_mm2s_status & AXI_MCDMA_CH_IOC_IRQ) {
			STATUS("ch%d_mm2s_status: IOC_Irq", i);
		}
		if (ch_mm2s_status & AXI_MCDMA_CH_DLY_IRQ) {
			STATUS("ch%d_mm2s_status: DlyIrq", i);
		}
		if (ch_mm2s_status & AXI_MCDMA_CH_ERR_IRQ) {
			ERROR("ch%d_mm2s_status: Err Irq ", i);
		}
	}
}

void s2mm_channel_status(mcdma_device_t *device) {
	for (int i = 0; i < NUM_CHANNELS; i ++) {
		uint32_t ch_s2mm_status = _reg_get(device->v_baseaddr, (AXI_MCDMA_S2MM_CHSR + AXI_MCDMA_CH_OFFSET*i));
		// STATUS("ch1_s2mm_status = 0x%08x", ch1_mm2s_status);
		if (ch_s2mm_status & AXI_MCDMA_CH_IDLE) {
			STATUS("ch%d_s2mm_status: Idle (Queue Empty) ", i);
		}
		if (ch_s2mm_status & AXI_MCDMA_CH_ERR_OTH_CH) {
			ERROR("ch%d_s2mm_status: Err_on_other_ch_irq ", i);
		}
		if (ch_s2mm_status & AXI_MCDMA_CH_IOC_IRQ) {
			STATUS("ch%d_s2mm_status: IOC_Irq", i);
		}
		if (ch_s2mm_status & AXI_MCDMA_CH_DLY_IRQ) {
			STATUS("ch%d_s2mm_status: DlyIrq", i);
		}
		if (ch_s2mm_status & AXI_MCDMA_CH_ERR_IRQ) {
			ERROR("ch%d_s2mm_status: Err Irq ", i);
		}
	}
}

void mm2s_bd_status(mcdma_channel_t *channel) {
	uint32_t mm2s_bd_status = _reg_get(channel->mm2s_bd_chain->v_bd_addr, AXI_MCDMA_MM2S_BD_STATUS);
	// STATUS("mm2s bd status = 0x%08x", mm2s_bd_status);
	STATUS("ch%d_mm2s_bd_status: %d bytes transferred", channel->channel_id, mm2s_bd_status & AXI_MCDMA_MM2S_BD_SBYTE_MASK);
	if (mm2s_bd_status & AXI_MCDMA_MM2S_BD_DMA_INT_ERR) {
		ERROR("ch%d_mm2s_bd_status: DMA Int Err", channel->channel_id);
	}
	if (mm2s_bd_status & AXI_MCDMA_MM2S_BD_DMA_SLV_ERR) {
		ERROR("ch%d_mm2s_bd_status: DMA Slave Err ", channel->channel_id);
	}
	if (mm2s_bd_status & AXI_MCDMA_MM2S_BD_DMA_DEC_ERR) {
		ERROR("ch%d_mm2s_bd_status: DMA Dec Err", channel->channel_id);
	}
	if (mm2s_bd_status & AXI_MCDMA_MM2S_BD_DMA_COMPLETED) {
		STATUS("ch%d_mm2s_bd_status: Completed", channel->channel_id);
	}
}

void s2mm_bd_status(mcdma_channel_t *channel) {
	uint32_t s2mm_bd_status = _reg_get(channel->s2mm_bd_chain->v_bd_addr, AXI_MCDMA_S2MM_BD_STATUS);
	// STATUS("s2mm bd status = 0x%08x", S2MM_bd_status);
	STATUS("ch%d_s2mm_bd_status: %d bytes transferred", channel->channel_id, s2mm_bd_status & AXI_MCDMA_S2MM_BD_SBYTE_MASK);
	if (s2mm_bd_status & AXI_MCDMA_S2MM_BD_DMA_INT_ERR) {
		ERROR("ch%d_s2mm_bd_status: DMA Int Err", channel->channel_id);
	}
	if (s2mm_bd_status & AXI_MCDMA_S2MM_BD_DMA_SLV_ERR) {
		ERROR("ch%d_s2mm_bd_status: DMA Slave Err ", channel->channel_id);
	}
	if (s2mm_bd_status & AXI_MCDMA_S2MM_BD_DMA_DEC_ERR) {
		ERROR("ch%d_s2mm_bd_status: DMA Dec Err", channel->channel_id);
	}
	if (s2mm_bd_status & AXI_MCDMA_S2MM_BD_DMA_COMPLETED) {
		STATUS("ch%d_s2mm_bd_status: Completed", channel->channel_id);
	}
	if (s2mm_bd_status & AXI_MCDMA_S2MM_BD_DMA_RXSOF) {
		STATUS("ch%d_s2mm_bd_status: SOF", channel->channel_id);
	}
	if (s2mm_bd_status & AXI_MCDMA_S2MM_BD_DMA_RXEOF) {
		STATUS("ch%d_s2mm_bd_status: EOF", channel->channel_id);
	}
}
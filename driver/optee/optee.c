/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021, Bootlin
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "common.h"
#include "debug.h"
#include "optee.h"
#include "types.h"

#define OPTEE_MAGIC             0x4554504f
#define OPTEE_VERSION           1
#define OPTEE_ARCH_ARM32        0
#define OPTEE_ARCH_ARM64        1

struct optee_header {
        u32 magic;
        u8 version;
        u8 arch;
        u16 flags;
        u32 init_size;
        u32 init_load_addr_hi;
        u32 init_load_addr_lo;
        u32 init_mem_usage;
        u32 paged_size;
};

void optee_start(void *pager, void *arg1, void *dtb_addr, void *ns_entry_addr);

void optee_image_init(struct image_info *image)
{
	image->dest = (unsigned char *) OPTEE_JUMP_ADDR;
	image->dest -= sizeof(struct optee_header);

#if defined(CONFIG_DATAFLASH) || defined(CONFIG_NANDFLASH) || defined(CONFIG_FLASH)
	image->length = OPTEE_IMG_SIZE;
	image->offset = get_image_load_offset(OPTEE_IMG_ADDRESS);
#endif

#ifdef CONFIG_SDCARD
	image->filename = "optee.bin";
#endif
}

int optee_image_check(struct image_info *image, void **pagestore)
{
	struct optee_header *hdr = (struct optee_header *) image->dest;
	u32 optee_size;

	dbg_info("OP-TEE hdr info:\n");
	dbg_info("      magic=%x\n", hdr->magic);
	dbg_info("      version=%x\n", hdr->version);
	dbg_info("      arch=%x\n", hdr->arch);
	dbg_info("      flags=%x\n", hdr->flags);
	dbg_info("      load_addr=%x\n", hdr->init_load_addr_lo);
	dbg_info("      init_size=%x\n", hdr->init_size);

	if (hdr->magic != OPTEE_MAGIC ||
	    hdr->version != OPTEE_VERSION ||
	    hdr->arch != OPTEE_ARCH_ARM32 ||
	    hdr->init_load_addr_lo != OPTEE_JUMP_ADDR ||
	    hdr->init_load_addr_hi != 0)
		return 1;

	optee_size = hdr->init_size + hdr->paged_size +
		     sizeof(struct optee_header);

	*pagestore = (void *) (image->dest + optee_size);

	return 0;
}

void optee_load(void)
{
	int ret;
	void *of_addr = NULL;
	void *page_store = 0;
	struct image_info image = {0};
	load_function load_func = get_image_load_func();

	optee_image_init(&image);

	ret = load_func(&image);
	if (ret) {
		dbg_info("Failed to load OP-TEE\n");
		while(1);
	}

	ret = optee_image_check(&image, &page_store);
	if (ret) {
		dbg_info("Invalid OP-TEE image\n");
		while(1);
	}

	dbg_info("Starting OP-TEE, Run at %x\n", OPTEE_JUMP_ADDR);

#ifdef CONFIG_OF_LIBFDT
	of_addr = OF_ADDRESS;
#endif
	optee_start(page_store, NULL, of_addr, (void *) JUMP_ADDR);

	/* We should never reach this point */
	while(1);
}

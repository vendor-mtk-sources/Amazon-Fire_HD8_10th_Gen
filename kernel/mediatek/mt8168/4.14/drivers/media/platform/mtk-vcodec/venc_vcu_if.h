/*
 * Copyright (c) 2016 MediaTek Inc.
 * Author: PoChun Lin <pochun.lin@mediatek.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _VENC_VCU_IF_H_
#define _VENC_VCU_IF_H_

#include "mtk_vcu.h"
#include "mtk_vcodec_drv.h"
#include "venc_ipi_msg.h"

/*
 * struct venc_vcu_inst - encoder VCU driver instance
 * @wq_hd: wait queue used for vcu cmd trigger then wait vcu interrupt done
 * @signaled: flag used for checking vcu interrupt done
 * @failure: flag to show vcu cmd succeeds or not
 * @state: enum venc_ipi_msg_enc_state
 * @bs_size: bitstream size for skip frame case usage
 * @is_key_frm: key frame flag
 * @inst_addr: VCU instance addr
 * @vsi: driver structure allocated by VCU side and shared to AP side for
 *       control and info share
 * @id: the id of inter-processor interrupt
 * @ctx: context for v4l2 layer integration
 * @dev: device for v4l2 layer integration
 */
struct venc_vcu_inst {
	wait_queue_head_t wq_hd;
	int signaled;
	int failure;
	int bs_size;
	int is_key_frm;
	unsigned int inst_addr;
	void *vsi;
	enum ipi_id id;
	struct mtk_vcodec_ctx *ctx;
	struct platform_device *dev;
	bool abort;
};

int vcu_enc_init(struct venc_vcu_inst *vcu);
int vcu_enc_query_cap(struct venc_vcu_inst *vcu, unsigned int id, void *out);
int vcu_enc_set_param(struct venc_vcu_inst *vcu,
					  enum venc_set_param_type id,
					  struct venc_enc_param *param);
int vcu_enc_encode(struct venc_vcu_inst *vcu, unsigned int bs_mode,
				   struct venc_frm_buf *frm_buf,
				   struct mtk_vcodec_mem *bs_buf,
				   unsigned int *bs_size);
int vcu_enc_deinit(struct venc_vcu_inst *vcu);
int vcu_enc_set_ctx_for_gce(struct venc_vcu_inst *vcu);

#endif

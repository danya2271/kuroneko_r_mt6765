// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include "ion_drv.h"
#include "mtk_ion.h"
#include <ion_priv.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/file.h>
#include <linux/printk.h>

#ifdef CONFIG_MTK_M4U
#include "m4u.h"
#endif

#include "mtk_sync.h"
#include "ddp_ovl.h"
#include "mtkfb_fence.h"
#include "ddp_path.h"
#include "disp_drv_platform.h"
#include "primary_display.h"
#include "mtk_disp_mgr.h"

static struct ion_client *ion_client;

#define MTK_FB_NO_ION_FD        ((int)(~0U>>1))
#define DISP_SESSION_TYPE(id) (((id)>>16)&0xff)

static LIST_HEAD(info_pool_head);
static DEFINE_MUTEX(_disp_fence_mutex);
static DEFINE_MUTEX(fence_buffer_mutex);

struct disp_session_sync_info _disp_fence_context[MAX_SESSION_COUNT];

static struct disp_session_sync_info *_get_session_sync_info(unsigned int session_id)
{
	int i = 0, j = 0;
	struct disp_session_sync_info *session_info = NULL;
	struct disp_sync_info *layer_info = NULL;
	char name[32];
	const char *prefix = "tl";

	if (DISP_SESSION_TYPE(session_id) != DISP_SESSION_PRIMARY &&
	    DISP_SESSION_TYPE(session_id) != DISP_SESSION_MEMORY &&
	    DISP_SESSION_TYPE(session_id) != DISP_SESSION_EXTERNAL) {
		pr_err("invalid session id:0x%08x\n", session_id);
		return NULL;
	}

	mutex_lock(&_disp_fence_mutex);
	for (i = 0; i < ARRAY_SIZE(_disp_fence_context); i++) {
		if (session_id == _disp_fence_context[i].session_id) {
			session_info = &(_disp_fence_context[i]);
			goto done;
		}
	}

	for (i = 0; i < ARRAY_SIZE(_disp_fence_context); i++) {
		if (_disp_fence_context[i].session_id != 0xffffffff)
			continue;

		_disp_fence_context[i].session_id = session_id;
		session_info = &(_disp_fence_context[i]);

		for (j = 0; j < ARRAY_SIZE(session_info->session_layer_info); j++) {
			if (DISP_SESSION_TYPE(session_id) == DISP_SESSION_PRIMARY)
				snprintf(name, sizeof(name), "%s-p-%d-%d", prefix, DISP_SESSION_DEV(session_id), j);
			else if (DISP_SESSION_TYPE(session_id) == DISP_SESSION_EXTERNAL)
				snprintf(name, sizeof(name), "%s-e-%d-%d", prefix, DISP_SESSION_DEV(session_id), j);
			else
				snprintf(name, sizeof(name), "%s-m-%d-%d", prefix, DISP_SESSION_DEV(session_id), j);

			layer_info = &(session_info->session_layer_info[j]);
			mutex_init(&(layer_info->sync_lock));
			layer_info->layer_id = j;
			layer_info->fence_idx = 0;
			layer_info->timeline_idx = 0;
			layer_info->inc = 0;
			layer_info->cur_idx = 0;
			layer_info->inited = 1;
			layer_info->timeline = timeline_create(name);
			INIT_LIST_HEAD(&layer_info->buf_list);
		}
		goto done;
	}

done:
	mutex_unlock(&_disp_fence_mutex);
	return session_info;
}

struct disp_session_sync_info *disp_get_session_sync_info_for_debug(unsigned int session_id)
{
	return _get_session_sync_info(session_id);
}

struct disp_sync_info *_get_sync_info(unsigned int session_id, unsigned int timeline_id)
{
	struct disp_sync_info *layer_info = NULL;
	struct disp_session_sync_info *session_info = _get_session_sync_info(session_id);

	mutex_lock(&_disp_fence_mutex);

	if (!session_info || timeline_id >= ARRAY_SIZE(session_info->session_layer_info))
		goto done;

	layer_info = &(session_info->session_layer_info[timeline_id]);

	if (layer_info && !layer_info->inited)
		layer_info = NULL;

done:
	mutex_unlock(&_disp_fence_mutex);
	return layer_info;
}

#if defined(MTK_FB_ION_SUPPORT)
static void mtkfb_ion_init(void)
{
	if (!ion_client && g_ion_device)
		ion_client = ion_client_create(g_ion_device, "display");
}

static struct ion_handle *mtkfb_ion_import_handle(struct ion_client *client, int fd)
{
	struct ion_handle *handle = NULL;

	if (fd == MTK_FB_NO_ION_FD || !ion_client || fd == MTK_FB_INVALID_ION_FD)
		return handle;

	handle = ion_import_dma_buf_fd(client, fd);
	if (IS_ERR(handle)) return NULL;

	return handle;
}

static void mtkfb_ion_free_handle(struct ion_client *client, struct ion_handle *handle)
{
	if (ion_client && handle)
		ion_free(client, handle);
}

static size_t mtkfb_ion_phys_mmu_addr(struct ion_client *client, struct ion_handle *handle, unsigned int *mva)
{
	size_t size;
	struct ion_mm_data mm_data;

	if (!ion_client || !handle || !handle->buffer || !handle->buffer->heap)
		return 0;

	memset(&mm_data, 0, sizeof(mm_data));
	if (handle->buffer->heap->type == ION_HEAP_TYPE_MULTIMEDIA) {
		mm_data.mm_cmd = ION_MM_GET_IOVA;
		mm_data.config_buffer_param.kernel_handle = handle;
		mm_data.config_buffer_param.module_id = 0;
		if (ion_kernel_ioctl(ion_client, ION_CMD_MULTIMEDIA, (unsigned long)&mm_data)) return 0;

		*mva = (unsigned int)mm_data.get_phys_param.phy_addr;
		size = (size_t)mm_data.get_phys_param.len;
	} else {
		ion_phys_addr_t phy_addr = 0;

		mm_data.mm_cmd = ION_MM_CONFIG_BUFFER;
		mm_data.config_buffer_param.kernel_handle = handle;
		mm_data.config_buffer_param.module_id = 0;
		mm_data.config_buffer_param.security = 0;
		mm_data.config_buffer_param.coherent = 0;
		if (ion_kernel_ioctl(ion_client, ION_CMD_MULTIMEDIA, (unsigned long)&mm_data)) return 0;

		ion_phys(client, handle, &phy_addr, &size);
		*mva = (unsigned int)phy_addr;
	}

	return size;
}

static void mtkfb_ion_cache_flush(struct ion_client *client, struct ion_handle *handle, unsigned int size)
{
	struct ion_sys_data sys_data;
	void *va = NULL;

	if (!ion_client || !handle) return;

	va = ion_map_kernel(client, handle);
	sys_data.sys_cmd = ION_SYS_CACHE_SYNC;
	sys_data.cache_sync_param.kernel_handle = handle;
	sys_data.cache_sync_param.va = va;
	sys_data.cache_sync_param.size = size;
	sys_data.cache_sync_param.sync_type = ION_CACHE_FLUSH_BY_RANGE;

	ion_kernel_ioctl(client, ION_CMD_SYSTEM, (unsigned long)&sys_data);
	ion_unmap_kernel(client, handle);
}
#endif

unsigned int mtkfb_query_buf_mva(unsigned int session_id, unsigned int layer_id, unsigned int idx)
{
	struct mtkfb_fence_buf_info *buf = NULL;
	unsigned int mva = 0x0;
	struct disp_sync_info *layer_info = _get_sync_info(session_id, layer_id);

	if (!layer_info) return 0;

	mutex_lock(&layer_info->sync_lock);
	list_for_each_entry(buf, &layer_info->buf_list, list) {
		if (buf->idx == idx) {
			mva = buf->mva;
			buf->buf_state = reg_configed;
			break;
		}
	}
	mutex_unlock(&layer_info->sync_lock);

	if (mva) {
		if (buf->cache_sync) mtkfb_ion_cache_flush(ion_client, buf->hnd, buf->size);
	}

	return mva;
}

unsigned int mtkfb_query_buf_va(unsigned int session_id, unsigned int layer_id, unsigned int idx)
{
	struct mtkfb_fence_buf_info *buf;
	unsigned int va = 0x0;
	struct disp_session_sync_info *session_info;
	struct disp_sync_info *layer_info;

	session_info = _get_session_sync_info(session_id);
	if (!session_info || layer_id >= DISP_SESSION_TIMELINE_COUNT) return 0;

	layer_info = &(session_info->session_layer_info[layer_id]);

	mutex_lock(&layer_info->sync_lock);
	list_for_each_entry(buf, &layer_info->buf_list, list) {
		if (buf->idx == idx) {
			va = buf->va;
			break;
		}
	}
	mutex_unlock(&layer_info->sync_lock);

	return va;
}

unsigned int mtkfb_query_release_idx(unsigned int session_id, unsigned int layer_id, unsigned long phy_addr)
{
	struct mtkfb_fence_buf_info *buf, *pre_buf = NULL;
	unsigned int idx = 0x0;
	struct disp_session_sync_info *session_info = _get_session_sync_info(session_id);
	struct disp_sync_info *layer_info;

	if (!session_info) return 0;
	layer_info = &(session_info->session_layer_info[layer_id]);

	mutex_lock(&layer_info->sync_lock);
	list_for_each_entry(buf, &layer_info->buf_list, list) {
		if (((buf->mva + buf->mva_offset) == phy_addr) &&
		    (buf->buf_state < reg_updated && buf->buf_state > create)) {
			buf->buf_state = reg_updated;
		} else if (((buf->mva + buf->mva_offset) != phy_addr) && (buf->buf_state == reg_updated)) {
			buf->buf_state = read_done;
		} else if ((phy_addr == 0) && (buf->buf_state > create)) {
			buf->buf_state = read_done;
		}

		if (pre_buf && ((pre_buf->mva + pre_buf->mva_offset) == (buf->mva + buf->mva_offset)) &&
		    (pre_buf->buf_state == reg_updated)) {
			pre_buf->buf_state = read_done;
			idx = pre_buf->idx;
		}

		if (buf->buf_state == read_done) idx = buf->idx;
		pre_buf = buf;
	}
	mutex_unlock(&layer_info->sync_lock);
	return idx;
}

unsigned int mtkfb_update_buf_ticket(unsigned int session_id, unsigned int layer_id, unsigned int idx, unsigned int ticket)
{
	struct mtkfb_fence_buf_info *buf;
	struct disp_session_sync_info *session_info = _get_session_sync_info(session_id);
	struct disp_sync_info *layer_info;

	if (!session_info || layer_id >= DISP_SESSION_TIMELINE_COUNT) return 0;
	layer_info = &(session_info->session_layer_info[layer_id]);

	mutex_lock(&layer_info->sync_lock);
	list_for_each_entry(buf, &layer_info->buf_list, list) {
		if (buf->idx == idx) {
			buf->trigger_ticket = ticket;
			break;
		}
	}
	mutex_unlock(&layer_info->sync_lock);

	return 0;
}

int mtkfb_query_idx_by_ticket(unsigned int session_id, unsigned int layer_id, unsigned int ticket)
{
	struct mtkfb_fence_buf_info *buf;
	int idx = -1;
	struct disp_session_sync_info *session_info = _get_session_sync_info(session_id);
	struct disp_sync_info *layer_info;

	if (!session_info) return idx;
	layer_info = &(session_info->session_layer_info[layer_id]);

	mutex_lock(&layer_info->sync_lock);
	list_for_each_entry(buf, &layer_info->buf_list, list) {
		if (buf->trigger_ticket == ticket) idx = buf->idx;
	}
	mutex_unlock(&layer_info->sync_lock);

	return idx;
}

bool mtkfb_update_buf_info_new(unsigned int session_id, unsigned int mva_offset, struct disp_input_config *buf_info)
{
	struct mtkfb_fence_buf_info *buf;
	struct disp_session_sync_info *session_info = _get_session_sync_info(session_id);
	struct disp_sync_info *layer_info;

	if (!session_info || buf_info->layer_id >= DISP_SESSION_TIMELINE_COUNT) return false;
	layer_info = &(session_info->session_layer_info[buf_info->layer_id]);

	mutex_lock(&layer_info->sync_lock);
	list_for_each_entry(buf, &layer_info->buf_list, list) {
		if (buf->idx == buf_info->next_buff_idx) {
			buf->layer_type = buf_info->layer_type;
			break;
		}
	}
	mutex_unlock(&layer_info->sync_lock);

	return false;
}

unsigned int mtkfb_query_buf_info(unsigned int session_id, unsigned int layer_id, unsigned long phy_addr, int query_type)
{
	struct mtkfb_fence_buf_info *buf;
	struct disp_session_sync_info *session_info = _get_session_sync_info(session_id);
	struct disp_sync_info *layer_info;
	int query_info = 0;

	if (!session_info) return 0;
	layer_info = &(session_info->session_layer_info[layer_id]);

	mutex_lock(&layer_info->sync_lock);
	list_for_each_entry(buf, &layer_info->buf_list, list) {
		if ((buf->mva + buf->mva_offset) == phy_addr) {
			query_info = buf->layer_type;
			mutex_unlock(&layer_info->sync_lock);
			return query_info;
		}
	}
	mutex_unlock(&layer_info->sync_lock);

	return query_info;
}

bool mtkfb_update_buf_info(unsigned int session_id, unsigned int layer_id, unsigned int idx, unsigned int mva_offset, unsigned int seq)
{
	struct mtkfb_fence_buf_info *buf;
	bool ret = false;
	struct disp_sync_info *layer_info = _get_sync_info(session_id, layer_id);

	if (!layer_info) return false;

	mutex_lock(&layer_info->sync_lock);
	list_for_each_entry(buf, &layer_info->buf_list, list) {
		if (buf->idx != idx) continue;
		buf->mva_offset = mva_offset;
		buf->seq = seq;
		ret = true;
		break;
	}
	mutex_unlock(&layer_info->sync_lock);

	return ret;
}

unsigned int mtkfb_query_frm_seq_by_addr(unsigned int session_id, unsigned int layer_id, unsigned long phy_addr)
{
	struct mtkfb_fence_buf_info *buf;
	unsigned int frm_seq = 0x0;
	struct disp_session_sync_info *session_info;
	struct disp_sync_info *layer_info;

	if (session_id <= 0) return 0;
	session_info = _get_session_sync_info(session_id);
	if (!session_info) return 0;

	layer_info = &(session_info->session_layer_info[layer_id]);

	mutex_lock(&layer_info->sync_lock);
	list_for_each_entry(buf, &layer_info->buf_list, list) {
		if (phy_addr > 0) {
			if ((buf->mva + buf->mva_offset) == phy_addr) {
				frm_seq = buf->seq;
				break;
			}
		} else {
			if (buf->seq < frm_seq) break;
			frm_seq = buf->seq;
		}
	}
	mutex_unlock(&layer_info->sync_lock);

	return frm_seq;
}

int disp_sync_init(void)
{
	int i;
	struct disp_session_sync_info *session_info;

	memset((void *)&_disp_fence_context, 0, sizeof(_disp_fence_context));

	for (i = 0; i < ARRAY_SIZE(_disp_fence_context) ; i++) {
		session_info = &_disp_fence_context[i];
		session_info->session_id = 0xffffffff;
	}

#ifdef MTK_FB_ION_SUPPORT
	mtkfb_ion_init();
#endif
	return 0;
}

struct mtkfb_fence_buf_info *mtkfb_init_buf_info(struct mtkfb_fence_buf_info *buf)
{
	INIT_LIST_HEAD(&buf->list);
	buf->fence = MTK_FB_INVALID_FENCE_FD;
	buf->hnd = NULL;
	buf->idx = 0;
	buf->mva = 0;
	buf->cache_sync = 0;
	buf->layer_type = 0;
	return buf;
}

static struct mtkfb_fence_buf_info *mtkfb_get_buf_info(void)
{
	struct mtkfb_fence_buf_info *info;

	mutex_lock(&fence_buffer_mutex);
	if (!list_empty(&info_pool_head)) {
		info = list_first_entry(&info_pool_head, struct mtkfb_fence_buf_info, list);
		list_del_init(&info->list);
		mtkfb_init_buf_info(info);
		mutex_unlock(&fence_buffer_mutex);
	} else {
		info = kzalloc(sizeof(struct mtkfb_fence_buf_info), GFP_KERNEL);
		mtkfb_init_buf_info(info);
		mutex_unlock(&fence_buffer_mutex);
	}

	return info;
}

void mtkfb_release_fence(unsigned int session_id, unsigned int layer_id, int fence)
{
	struct mtkfb_fence_buf_info *buf, *n;
	int num_fence = 0;
	struct disp_sync_info *layer_info = _get_sync_info(session_id, layer_id);

	if (!layer_info || !layer_info->timeline) return;

	mutex_lock(&layer_info->sync_lock);
	num_fence = fence - layer_info->timeline_idx;

	if (num_fence <= 0) {
		mutex_unlock(&layer_info->sync_lock);
		return;
	}

	timeline_inc(layer_info->timeline, num_fence);
	layer_info->timeline_idx = fence;

	list_for_each_entry_safe(buf, n, &layer_info->buf_list, list) {
		if (buf->idx > fence) continue;

		layer_info->fence_fd = buf->fence;
		list_del_init(&buf->list);

#ifdef MTK_FB_ION_SUPPORT
		if (buf->va && ((DISP_SESSION_TYPE(session_id) > DISP_SESSION_PRIMARY)))
			ion_unmap_kernel(ion_client, buf->hnd);
		if (buf->hnd)
			mtkfb_ion_free_handle(ion_client, buf->hnd);
#endif

		mutex_lock(&fence_buffer_mutex);
		list_add_tail(&buf->list, &info_pool_head);
		mutex_unlock(&fence_buffer_mutex);
	}

	mutex_unlock(&layer_info->sync_lock);
}

void mtkfb_release_layer_fence(unsigned int session_id, unsigned int layer_id)
{
	struct disp_sync_info *layer_info = _get_sync_info(session_id, layer_id);
	int fence = 0;

	if (!layer_info) return;

	mutex_lock(&layer_info->sync_lock);
	fence = layer_info->fence_idx;
	mutex_unlock(&layer_info->sync_lock);

	mtkfb_release_fence(session_id, layer_id, fence);
}

void mtkfb_release_session_fence(unsigned int session_id)
{
	struct disp_session_sync_info *session_sync_info = _get_session_sync_info(session_id);
	int i;

	if (!session_sync_info) return;
	for (i = 0; i < ARRAY_SIZE(session_sync_info->session_layer_info); i++)
		mtkfb_release_layer_fence(session_id, i);
}

void mtkfb_release_present_fence(unsigned int session_id, unsigned int fence_idx)
{
	struct disp_sync_info *layer_info = _get_sync_info(session_id, disp_sync_get_present_timeline_id());
	int fence_increment = 0;

	if (!layer_info) return;

	mutex_lock(&layer_info->sync_lock);
	fence_increment = fence_idx - layer_info->timeline->value;

	if (fence_increment > 0)
		timeline_inc(layer_info->timeline, fence_increment);

	mutex_unlock(&layer_info->sync_lock);
}

int disp_sync_get_ovl_timeline_id(int layer_id) { return DISP_SESSION_OVL_TIMELINE_ID(layer_id); }
int disp_sync_get_output_timeline_id(void) { return DISP_SESSION_OUTPUT_TIMELINE_ID; }
int disp_sync_get_output_interface_timeline_id(void) { return DISP_SESSION_OUTPUT_INTERFACE_TIMELINE_ID; }
int disp_sync_get_present_timeline_id(void) { return DISP_SESSION_PRESENT_TIMELINE_ID; }

int disp_sync_get_cached_layer_info(unsigned int session_id, unsigned int timeline_idx, unsigned int *layer_en, unsigned long *addr, unsigned int *fence_idx)
{
	struct disp_sync_info *layer_info = _get_sync_info(session_id, timeline_idx);

	if (!layer_info) return -1;

	mutex_lock(&(layer_info->sync_lock));
	if (layer_en && addr && fence_idx) {
		*layer_en = layer_info->cached_config.layer_en;
		*addr = layer_info->cached_config.addr;
		*fence_idx = layer_info->cached_config.buff_idx;
		mutex_unlock(&(layer_info->sync_lock));
		return 0;
	}
	mutex_unlock(&(layer_info->sync_lock));
	return -1;
}

int disp_sync_put_cached_layer_info(unsigned int session_id, unsigned int timeline_idx, struct disp_input_config *src, unsigned long mva)
{
	struct disp_sync_info *layer_info = _get_sync_info(session_id, timeline_idx);

	if (!layer_info) return -1;

	mutex_lock(&(layer_info->sync_lock));
	if (src) {
		disp_sync_convert_input_to_fence_layer_info(src, &(layer_info->cached_config), mva);
		mutex_unlock(&(layer_info->sync_lock));
		return 0;
	}
	mutex_unlock(&(layer_info->sync_lock));
	return -1;
}

int disp_sync_convert_input_to_fence_layer_info(struct disp_input_config *src, struct FENCE_LAYER_INFO *dst, unsigned long dst_mva)
{
	if (src && dst) {
		dst->layer = src->layer_id;
		dst->addr = dst_mva;
		if (src->next_buff_idx == 0) {
			dst->layer_en = 0;
		} else {
			dst->layer_en = src->layer_enable;
			dst->buff_idx = src->next_buff_idx;
		}
		return 0;
	}
	return -1;
}

int disp_sync_put_cached_layer_info_v2(unsigned int session_id, unsigned int timeline_idx, unsigned int fence_id, int layer_en, unsigned long mva)
{
	struct disp_sync_info *layer_info = _get_sync_info(session_id, timeline_idx);

	if (!layer_info) return -1;

	mutex_lock(&(layer_info->sync_lock));
	layer_info->cached_config.layer = timeline_idx;
	layer_info->cached_config.addr = mva;
	if (fence_id == 0) {
		layer_info->cached_config.layer_en = 0;
	} else {
		layer_info->cached_config.layer_en = layer_en;
		layer_info->cached_config.buff_idx = fence_id;
	}
	mutex_unlock(&(layer_info->sync_lock));

	return 0;
}

static int prepare_ion_buf(struct disp_buffer_info *buf, struct mtkfb_fence_buf_info *buf_info)
{
	unsigned int mva = 0x0;
	struct ion_handle *handle = NULL;

#if defined(MTK_FB_ION_SUPPORT)
	handle = mtkfb_ion_import_handle(ion_client, buf->ion_fd);
	if (handle) buf_info->size = mtkfb_ion_phys_mmu_addr(ion_client, handle, &mva);
#endif

	buf_info->hnd = handle;
	buf_info->mva = mva;
	buf_info->va = 0;
	return 0;
}

struct mtkfb_fence_buf_info *disp_sync_prepare_buf(struct disp_buffer_info *buf)
{
	struct mtkfb_fence_buf_info *buf_info = NULL;
	struct mtk_sync_create_fence_data data;
	struct disp_sync_info *layer_info;

	if (!buf) return NULL;

	layer_info = _get_sync_info(buf->session_id, buf->layer_id);
	if (!layer_info || !layer_info->inited) return NULL;

	buf_info = mtkfb_get_buf_info();

	mutex_lock(&layer_info->sync_lock);
	data.fence = MTK_FB_INVALID_FENCE_FD;
	data.value = ++(layer_info->fence_idx);
	mutex_unlock(&(layer_info->sync_lock));

	snprintf(data.name, sizeof(data.name), "disp-S%x-L%d-%d", buf->session_id, buf->layer_id, data.value);
	fence_create(layer_info->timeline, &data);

	buf_info->fence = data.fence;
	buf_info->idx = data.value;

	if (buf->ion_fd >= 0)
		if (prepare_ion_buf(buf, buf_info) < 0) return NULL;

	buf_info->mva_offset = 0;
	buf_info->trigger_ticket = 0;
	buf_info->buf_state = create;
	buf_info->cache_sync = buf->cache_sync;

	mutex_lock(&layer_info->sync_lock);
	list_add_tail(&buf_info->list, &layer_info->buf_list);
	mutex_unlock(&layer_info->sync_lock);

	return buf_info;
}

int disp_sync_find_fence_idx_by_addr(unsigned int session_id, unsigned int timeline_id, unsigned long phy_addr)
{
	struct mtkfb_fence_buf_info *buf;
	int idx = -1;
	unsigned int layer_en = 0, fence_idx = -1;
	unsigned long addr = 0;
	struct disp_sync_info *layer_info = _get_sync_info(session_id, timeline_id);

	if (!layer_info) return -1;
	if (layer_info->fence_idx == 0) return -2;

	disp_sync_get_cached_layer_info(session_id, timeline_id, &layer_en, &addr, &fence_idx);

	if (phy_addr) {
		mutex_lock(&layer_info->sync_lock);
		list_for_each_entry(buf, &layer_info->buf_list, list) {
			if (buf->idx > fence_idx) continue;
			if ((buf->mva + buf->mva_offset) == phy_addr) idx = buf->idx - 1;
		}
		mutex_unlock(&layer_info->sync_lock);
	} else {
		idx = layer_en == 0 ? fence_idx : fence_idx - 1;
	}

	return idx;
}

unsigned int disp_sync_buf_cache_sync(unsigned int session_id, unsigned int timeline_id, unsigned int idx)
{
	struct mtkfb_fence_buf_info *buf;
	struct disp_sync_info *layer_info = _get_sync_info(session_id, timeline_id);

	if (!layer_info) return -1;

	mutex_lock(&layer_info->sync_lock);
	list_for_each_entry(buf, &layer_info->buf_list, list) {
		if (buf->idx != idx) continue;
		if (buf->cache_sync) mtkfb_ion_cache_flush(ion_client, buf->hnd, buf->size);
		break;
	}
	mutex_unlock(&layer_info->sync_lock);

	return 0;
}

static unsigned int __disp_sync_query_buf_info(unsigned int session_id, unsigned int timeline_id, unsigned int idx, unsigned long *mva, unsigned int *size, int need_sync)
{
	struct mtkfb_fence_buf_info *buf;
	unsigned long dst_mva = 0;
	uint32_t dst_size = 0;
	struct disp_sync_info *layer_info = _get_sync_info(session_id, timeline_id);

	if (!layer_info || !mva || !size) return 0;

	mutex_lock(&layer_info->sync_lock);
	list_for_each_entry(buf, &layer_info->buf_list, list) {
		if (buf->idx == idx) {
			dst_mva = buf->mva;
			dst_size = buf->size;
			break;
		}
	}
	mutex_unlock(&layer_info->sync_lock);

	if (dst_mva) {
		*mva = dst_mva;
		*size = dst_size;
		if (buf->cache_sync && need_sync) mtkfb_ion_cache_flush(ion_client, buf->hnd, buf->size);
	}

	return 0;
}

unsigned int disp_sync_query_buf_info(unsigned int session_id, unsigned int timeline_id, unsigned int idx, unsigned long *mva, unsigned int *size)
{
	return __disp_sync_query_buf_info(session_id, timeline_id, idx, mva, size, 1);
}

unsigned int disp_sync_query_buf_info_nosync(unsigned int session_id, unsigned int timeline_id, unsigned int idx, unsigned long *mva, unsigned int *size)
{
	return __disp_sync_query_buf_info(session_id, timeline_id, idx, mva, size, 0);
}

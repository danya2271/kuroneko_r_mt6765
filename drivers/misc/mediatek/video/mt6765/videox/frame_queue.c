// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#define pr_fmt(fmt) "disp_frame_queue: " fmt

#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/printk.h>
#include <uapi/linux/sched/types.h>

#include "disp_drv_platform.h"
#include "frame_queue.h"
#include "mtk_disp_mgr.h"

static struct frame_queue_head_t frame_q_head[MAX_SESSION_COUNT];
DEFINE_MUTEX(frame_q_head_lock);

#ifdef DISP_SYNC_ENABLE
static int _do_wait_fence(struct sync_fence **src_fence)
{
	int ret;

	if (!*src_fence)
		return 0;

	ret = sync_fence_wait(*src_fence, 1000);

	if (ret == -ETIME)
		pr_err("display fence wait timeout for 1000ms.\n");
	else if (ret != 0)
		pr_err("display fence wait status error. ret=%d\n", ret);

	sync_fence_put(*src_fence);
	*src_fence = NULL;
	return ret;
}
#endif

static int frame_wait_all_fence(struct disp_frame_cfg_t *cfg)
{
#ifdef DISP_SYNC_ENABLE
	int i, ret = 0;

	/* wait present fence */
	if (cfg->prev_present_fence_struct) {
		if (_do_wait_fence((struct sync_fence **)&cfg->prev_present_fence_struct)) {
			pr_err("wait present fence fail!\n");
			ret = -1;
		}
	}

	/* wait input fences */
	for (i = 0; i < cfg->input_layer_num; i++) {
		if (!cfg->input_cfg[i].src_fence_struct)
			continue;

		if (_do_wait_fence((struct sync_fence **)&cfg->input_cfg[i].src_fence_struct)) {
			dump_input_cfg_info(&cfg->input_cfg[i], cfg->session_id, 1);
			ret = -1;
		}
	}

	/* wait output fence */
	if (cfg->output_en && cfg->output_cfg.src_fence_struct) {
		if (_do_wait_fence((struct sync_fence **)&cfg->output_cfg.src_fence_struct)) {
			pr_err("wait output fence fail!\n");
			ret = -1;
		}
	}
	return ret;
#else
	return 0;
#endif
}

static int fence_wait_worker_func(void *data);

static int frame_queue_head_init(struct frame_queue_head_t *head, int session_id)
{
	WARN_ON(head->inited);

	INIT_LIST_HEAD(&head->queue);
	mutex_init(&head->lock);
	head->session_id = session_id;
	init_waitqueue_head(&head->wq);

	head->worker = kthread_run(fence_wait_worker_func, head, "disp_queue_%s%d",
				   DISP_SESSION_DEV(session_id));

	if (IS_ERR_OR_NULL(head->worker)) {
		pr_err("create fence thread fail! ret=%ld\n", PTR_ERR(head->worker));
		head->worker = NULL;
		return -ENOMEM;
	}

	head->inited = 1;
	return 0;
}

struct frame_queue_head_t *get_frame_queue_head(int session_id)
{
	int i, ret;
	struct frame_queue_head_t *head = NULL;
	struct frame_queue_head_t *unused_head = NULL;

	mutex_lock(&frame_q_head_lock);

	for (i = 0; i < ARRAY_SIZE(frame_q_head); i++) {
		if (frame_q_head[i].session_id == session_id) {
			head = &frame_q_head[i];
			break;
		}
		if (!frame_q_head[i].inited && !unused_head)
			unused_head = &frame_q_head[i];
	}

	if (head)
		goto out;

	if (unused_head) {
		ret = frame_queue_head_init(unused_head, session_id);
		if (!ret)
			head = unused_head;
		goto out;
	}

	pr_err("cannot find frame_q_head!! session_id=0x%x ===>\n", session_id);
	for (i = 0; i < ARRAY_SIZE(frame_q_head); i++)
		pr_info("0x%x,", frame_q_head[i].session_id);
	pr_info("\n");

out:
	mutex_unlock(&frame_q_head_lock);
	return head;
}

static int frame_queue_size(struct frame_queue_head_t *head)
{
	int cnt = 0;
	struct list_head *list;

	list_for_each(list, &head->queue)
		cnt++;

	return cnt;
}

struct frame_queue_t *frame_queue_node_create(void)
{
	struct frame_queue_t *node;

	node = kzalloc(sizeof(struct frame_queue_t), GFP_KERNEL);
	if (!node) {
		pr_err("fail to kzalloc %zu of frame_queue\n", sizeof(*node));
		return ERR_PTR(-ENOMEM);
	}

	INIT_LIST_HEAD(&node->link);
	return node;
}

void frame_queue_node_destroy(struct frame_queue_t *node)
{
	disp_input_free_dirty_roi(&node->frame_cfg);
	kfree(node);
}

static int fence_wait_worker_func(void *data)
{
	struct frame_queue_head_t *head = data;
	struct frame_queue_t *node;
	struct list_head *list;
	struct disp_frame_cfg_t *frame_cfg;
	struct sched_param param = {.sched_priority = 94 };

	sched_setscheduler(current, SCHED_RR, &param);

	while (1) {
		wait_event_interruptible(head->wq, !list_empty(&head->queue));

		mutex_lock(&head->lock);
		if (list_empty(&head->queue)) {
			mutex_unlock(&head->lock);
			goto next;
		}

		list = head->queue.next;
		mutex_unlock(&head->lock);

		node = list_entry(list, struct frame_queue_t, link);
		frame_cfg = &node->frame_cfg;

		frame_wait_all_fence(frame_cfg);

		if (node->do_frame_cfg)
			node->do_frame_cfg(node);

		mutex_lock(&head->lock);
		list_del(list);
		mutex_unlock(&head->lock);

		frame_queue_node_destroy(node);

next:
		wake_up(&head->wq);
		if (kthread_should_stop())
			break;
	}
	return 0;
}

int frame_queue_push(struct frame_queue_head_t *head, struct frame_queue_t *node)
{
	int frame_queue_sz;

	mutex_lock(&head->lock);
	frame_queue_sz = frame_queue_size(head);
	if (frame_queue_sz >= 5) {
		mutex_unlock(&head->lock);
		pr_err("block HWC because jobs=%d >=5\n", frame_queue_sz);
		wait_event_killable(head->wq, list_empty(&head->queue));
		mutex_lock(&head->lock);
	}
	list_add_tail(&node->link, &head->queue);
	mutex_unlock(&head->lock);
	wake_up(&head->wq);

	return 0;
}

int frame_queue_wait_all_jobs_done(struct frame_queue_head_t *head)
{
	int ret = 0;

	mutex_lock(&head->lock);
	while (!list_empty(&head->queue)) {
		mutex_unlock(&head->lock);
		ret = wait_event_killable(head->wq, list_empty(&head->queue));
		mutex_lock(&head->lock);

		if (ret == -ERESTARTSYS)
			break;
	}
	mutex_unlock(&head->lock);

	return ret;
}

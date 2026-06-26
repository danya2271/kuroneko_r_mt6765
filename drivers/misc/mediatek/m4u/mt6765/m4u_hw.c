// SPDX-License-Identifier: GPL-2.0
/*
 *
 *   Copyright (c) 2020 MediaTek Inc.
 */

#include <linux/slab.h>
#include <linux/interrupt.h>
#include "m4u_priv.h"
#include "m4u_hw.h"
#include "m4u_platform.h"
#include <linux/kmemleak.h>
#include <linux/of.h>
#include <linux/of_address.h>

#ifdef CONFIG_MTK_SMI_EXT
#include "smi_public.h"
#endif

static struct m4u_domain gM4uDomain;
static unsigned long gM4UBaseAddr[TOTAL_M4U_NUM];
static unsigned long gLarbBaseAddr[SMI_LARB_NR];
static unsigned long gPericfgBaseAddr;
static unsigned int gM4UTagCount[] = { 64 };

static struct m4u_range_des gM4U0_seq[M4U0_SEQ_NR] = { {0} };
static struct m4u_range_des *gM4USeq[] = { gM4U0_seq, NULL };
static DEFINE_MUTEX(gM4u_seq_mutex);
static struct m4u_prog_dist gM4U0_prog_pfh[M4U0_PROG_PFH_NR] = { {0} };
static struct m4u_prog_dist *gM4UProgPfh[] = { gM4U0_prog_pfh, NULL };
static DEFINE_MUTEX(gM4u_prog_pfh_mutex);

#define TF_PROTECT_BUFFER_SIZE 128L
static u8 *tf_protect_buffer;

int gM4U_L2_enable = 1;
int gM4U_4G_DRAM_Mode;

static spinlock_t gM4u_reg_lock;
int gM4u_port_num = M4U_PORT_UNKNOWN;
static DEFINE_MUTEX(m4u_larb0_mutex);

int m4u_invalid_tlb(int m4u_id, int L2_en, int isInvAll, unsigned int mva_start, unsigned int mva_end)
{
	unsigned int reg = F_MMU_INV_EN_L1;
	unsigned long m4u_base = gM4UBaseAddr[m4u_id];

	if (mva_start >= mva_end) isInvAll = 1;
	if (L2_en) reg |= F_MMU_INV_EN_L2;

	if (!isInvAll) {
		mva_start = round_down(mva_start, SZ_4K);
		mva_end = round_up(mva_end, SZ_4K);
	}

	spin_lock(&gM4u_reg_lock);
	M4U_WriteReg32(m4u_base, REG_INVLID_SEL, reg);

	if (isInvAll) {
		M4U_WriteReg32(m4u_base, REG_MMU_INVLD, F_MMU_INV_ALL);
	} else {
		writel_relaxed(mva_start, (void __iomem *)(m4u_base + REG_MMU_INVLD_SA));
		writel_relaxed(mva_end, (void __iomem *)(m4u_base + REG_MMU_INVLD_EA));
		M4U_WriteReg32(m4u_base, REG_MMU_INVLD, F_MMU_INV_RANGE);

		while (!M4U_ReadReg32(m4u_base, REG_MMU_CPE_DONE))
			cpu_relax();
		M4U_WriteReg32(m4u_base, REG_MMU_CPE_DONE, 0);
	}
	spin_unlock(&gM4u_reg_lock);
	return 0;

}

static void m4u_invalid_tlb_all(int m4u_id)
{
	m4u_invalid_tlb(m4u_id, gM4U_L2_enable, 1, 0, 0);
}

void m4u_invalid_tlb_by_range(struct m4u_domain *m4u_domain, unsigned int mva_start, unsigned int mva_end)
{
	int i;
	for (i = 0; i < TOTAL_M4U_NUM; i++)
		m4u_invalid_tlb(i, gM4U_L2_enable, 0, mva_start, mva_end);
}

static inline void m4u_clear_intr(unsigned int m4u_id)
{
	m4uHw_set_field_by_mask(gM4UBaseAddr[m4u_id], REG_MMU_INT_L2_CONTROL, F_INT_L2_CLR_BIT, F_INT_L2_CLR_BIT);
}

static inline void m4u_enable_intr(unsigned int m4u_id)
{
	M4U_WriteReg32(gM4UBaseAddr[m4u_id], REG_MMU_INT_L2_CONTROL, 0x6f);
	M4U_WriteReg32(gM4UBaseAddr[m4u_id], REG_MMU_INT_MAIN_CONTROL, 0xffffffff);
}

static inline void m4u_disable_intr(unsigned int m4u_id)
{
	M4U_WriteReg32(gM4UBaseAddr[m4u_id], REG_MMU_INT_L2_CONTROL, 0);
	M4U_WriteReg32(gM4UBaseAddr[m4u_id], REG_MMU_INT_MAIN_CONTROL, 0);
}

static inline void m4u_intr_modify_all(unsigned long enable)
{
	int i;
	for (i = 0; i < TOTAL_M4U_NUM; i++) {
		if (enable) m4u_enable_intr(i);
		else m4u_disable_intr(i);
	}
}

int mau_start_monitor(int m4u_id, int m4u_slave_id, int mau_set,
					  int wr, int vir, int io, int bit32, unsigned int start,
					  unsigned int end, unsigned int port_mask, unsigned int larb_mask)
{
	unsigned long m4u_base = gM4UBaseAddr[m4u_id];
	if (!m4u_base) return -1;

	writel_relaxed(start, (void __iomem *)(m4u_base + REG_MMU_MAU_START(m4u_slave_id, mau_set)));
	writel_relaxed(!!(bit32), (void __iomem *)(m4u_base + REG_MMU_MAU_START_BIT32(m4u_slave_id, mau_set)));
	writel_relaxed(end, (void __iomem *)(m4u_base + REG_MMU_MAU_END(m4u_slave_id, mau_set)));
	writel_relaxed(!!(bit32), (void __iomem *)(m4u_base + REG_MMU_MAU_END_BIT32(m4u_slave_id, mau_set)));
	writel_relaxed(port_mask, (void __iomem *)(m4u_base + REG_MMU_MAU_PORT_EN(m4u_slave_id, mau_set)));

	m4uHw_set_field_by_mask(m4u_base, REG_MMU_MAU_LARB_EN(m4u_slave_id), F_MAU_LARB_MSK(mau_set), F_MAU_LARB_VAL(mau_set, larb_mask));
	m4uHw_set_field_by_mask(m4u_base, REG_MMU_MAU_IO(m4u_slave_id), F_MAU_BIT_VAL(1, mau_set), F_MAU_BIT_VAL(io, mau_set));
	m4uHw_set_field_by_mask(m4u_base, REG_MMU_MAU_RW(m4u_slave_id), F_MAU_BIT_VAL(1, mau_set), F_MAU_BIT_VAL(wr, mau_set));
	m4uHw_set_field_by_mask(m4u_base, REG_MMU_MAU_VA(m4u_slave_id), F_MAU_BIT_VAL(1, mau_set), F_MAU_BIT_VAL(vir, mau_set));
	return 0;

}

unsigned int m4u_get_main_descriptor(int m4u_id, int m4u_slave_id, int idx)
{
	unsigned int regValue;
	unsigned long m4u_base = gM4UBaseAddr[m4u_id];

	regValue = F_READ_ENTRY_EN | F_READ_ENTRY_MMx_MAIN(m4u_slave_id) | F_READ_ENTRY_MAIN_IDX(idx);
	M4U_WriteReg32(m4u_base, REG_MMU_READ_ENTRY, regValue);
	while (M4U_ReadReg32(m4u_base, REG_MMU_READ_ENTRY) & F_READ_ENTRY_EN)
		cpu_relax();
	return M4U_ReadReg32(m4u_base, REG_MMU_DES_RDATA);

}

unsigned int m4u_get_main_tag(int m4u_id, int m4u_slave_id, int idx)
{
	return M4U_ReadReg32(gM4UBaseAddr[m4u_id], REG_MMU_MAIN_TAG(m4u_slave_id, idx));
}

unsigned int m4u_get_pfh_tag(int m4u_id, int set, int page, int way)
{
	unsigned int regValue;
	unsigned long m4u_base = gM4UBaseAddr[m4u_id];

	regValue = F_READ_ENTRY_EN | F_READ_ENTRY_PFH | F_READ_ENTRY_PFH_IDX(set) | F_READ_ENTRY_PFH_PAGE_IDX(page) | F_READ_ENTRY_PFH_WAY(way);
	M4U_WriteReg32(m4u_base, REG_MMU_READ_ENTRY, regValue);
	while (M4U_ReadReg32(m4u_base, REG_MMU_READ_ENTRY) & F_READ_ENTRY_EN)
		cpu_relax();

	return M4U_ReadReg32(m4u_base, REG_MMU_PFH_TAG_RDATA);

}

static unsigned int imu_pfh_tag_to_va(int mmu, int set, int way, unsigned int tag)
{
	unsigned int va = F_PFH_TAG_VA_GET(mmu, tag);
	if (tag & F_PFH_TAG_LAYER_BIT)
		return va | (set << 15);

	return (va & F_MMU_PFH_TAG_VA_LAYER0_MSK(mmu)) | (set << 23);
}

int m4u_confirm_main_range_invalidated(int m4u_index, int m4u_slave_id, unsigned int MVAStart, unsigned int MVAEnd)
{
	unsigned int i, regval;
	unsigned int sa = MVAStart & PAGE_MASK;
	unsigned int ea = MVAEnd | ~PAGE_MASK;

	for (i = 0; i < gM4UTagCount[m4u_index]; i++) {
		regval = m4u_get_main_tag(m4u_index, m4u_slave_id, i);

		if (regval & F_MAIN_TLB_VALID_BIT) {
			unsigned int tag_s = regval & F_MAIN_TLB_VA_MSK;
			unsigned int tag_e;

			if (regval & F_MAIN_TLB_LAYER_BIT) {
				tag_e = tag_s + ((regval & F_MAIN_TLB_16X_BIT) ? MMU_LARGE_PAGE_SIZE : PAGE_SIZE) - 1;
				if (!((tag_e < sa) || (tag_s > ea))) return -1;
			} else {
				tag_e = tag_s + ((regval & F_MAIN_TLB_16X_BIT) ? MMU_SUPERSECTION_SIZE : MMU_SECTION_SIZE) - 1;
				if ((tag_s >= sa) && (tag_e <= ea)) return -1;
			}
		}
	}
	return 0;

}

int m4u_confirm_range_invalidated(int m4u_index, unsigned int MVAStart, unsigned int MVAEnd)
{
	unsigned int regval, tag_s, tag_e, sa, ea, unit_sz;
	unsigned long m4u_base = gM4UBaseAddr[m4u_index];
	int set_nr = MMU_SET_NR(m4u_index);
	int way, set, i;


	if (m4u_confirm_main_range_invalidated(m4u_index, 0, MVAStart, MVAEnd) < 0) return -1;
	if (m4u_index == 0 && m4u_confirm_main_range_invalidated(m4u_index, 1, MVAStart, MVAEnd) < 0) return -1;

	sa = MVAStart & PAGE_MASK;
	ea = MVAEnd | ~PAGE_MASK;

	for (way = 0; way < MMU_WAY_NR; way++) {
		for (set = 0; set < set_nr; set++) {
			regval = M4U_ReadReg32(m4u_base, REG_MMU_PFH_VLD(m4u_index, set, way));
			if (regval & F_MMU_PFH_VLD_BIT(set, way)) {
				unsigned int tag = m4u_get_pfh_tag(m4u_index, set, 0, way);
				int layer = tag & F_PFH_TAG_LAYER_BIT;
				int large = tag & F_PFH_TAG_16X_BIT;

				tag_s = imu_pfh_tag_to_va(m4u_index, set, way, tag);

				if (layer && large) unit_sz = MMU_LARGE_PAGE_SIZE;
				else if (layer && !large) unit_sz = PAGE_SIZE;
				else if (!layer && large) unit_sz = MMU_SUPERSECTION_SIZE;
				else unit_sz = MMU_SECTION_SIZE;

				tag_e = tag_s + unit_sz * 8 - 1;
				if (!((tag_e < sa) || (tag_s > ea))) return -1;
			}
		}
	}
	return 0;

}

int m4u_power_on(int m4u_index) { return 0; }
int m4u_power_off(int m4u_index) { return 0; }
static int m4u_clock_on(void) { return 0; }

char *smi_clk_name[] = {
	"m4u_smi_larb0", "m4u_smi_larb1", "m4u_smi_larb2", "m4u_smi_larb3"
};

int larb_clock_on(int larb, bool config_mtcmos)
{
	#ifdef CONFIG_MTK_SMI_EXT
	if (larb < ARRAY_SIZE(smi_clk_name))
		smi_bus_prepare_enable(larb, smi_clk_name[larb]);
	#endif
	return 0;
}

int larb_clock_off(int larb, bool config_mtcmos)
{
	#ifdef CONFIG_MTK_SMI_EXT
	if (larb < ARRAY_SIZE(smi_clk_name))
		smi_bus_disable_unprepare(larb, smi_clk_name[larb]);
	#endif
	return 0;
}

int m4u_enable_prog_dist_by_id(int port, int id)
{
	unsigned long m4u_base = gM4UBaseAddr[m4u_port_2_m4u_id(port)];
	spin_lock(&gM4u_reg_lock);
	m4uHw_set_field_by_mask(m4u_base, REG_MMU_PROG_DIST(id), F_PF_EN(1), 1);
	spin_unlock(&gM4u_reg_lock);
	return 0;
}

int m4u_disable_prog_dist_by_id(int port, int id)
{
	unsigned long m4u_base = gM4UBaseAddr[m4u_port_2_m4u_id(port)];
	spin_lock(&gM4u_reg_lock);
	m4uHw_set_field_by_mask(m4u_base, REG_MMU_PROG_DIST(id), F_PF_EN(1), 0);
	spin_unlock(&gM4u_reg_lock);
	return 0;
}

int m4u_config_prog_dist(M4U_PORT_ID port, int dir, int dist, int en, int mm_id, int sel)
{
	int i, free_id = -1;
	int m4u_index = m4u_port_2_m4u_id(port);
	unsigned long m4u_base = gM4UBaseAddr[m4u_index];
	unsigned int larb = m4u_port_2_larb_id(port);
	unsigned int larb_port = m4u_port_2_larb_port(port);
	struct m4u_prog_dist *pProgPfh = gM4UProgPfh[m4u_index];


	if (unlikely(larb >= SMI_LARB_NR)) return -1;

	mutex_lock(&gM4u_prog_pfh_mutex);
	for (i = 0; i < M4U_PROG_PFH_NUM(m4u_index); i++) {
		if (pProgPfh[i].Enabled == 1) {
			if (port == pProgPfh[i].port && (sel == 0 || pProgPfh[i].sel == 0)) {
				free_id = i;
				break;
			}
		} else {
			free_id = i;
			break;
		}
	}

	if (free_id == -1) {
		mutex_unlock(&gM4u_prog_pfh_mutex);
		return -1;
	}

	pProgPfh[free_id].Enabled = 1;
	pProgPfh[free_id].port = port;
	pProgPfh[free_id].mm_id = mm_id;
	pProgPfh[free_id].dir = dir;
	pProgPfh[free_id].dist = dist;
	pProgPfh[free_id].en = en;
	pProgPfh[free_id].sel = sel;
	mutex_unlock(&gM4u_prog_pfh_mutex);

	spin_lock(&gM4u_reg_lock);
	m4uHw_set_field_by_mask(m4u_base, REG_MMU_PROG_DIST(free_id), F_PF_ID_COMP_SEL(1), F_PF_ID_COMP_SEL(!!(sel)));
	m4uHw_set_field_by_mask(m4u_base, REG_MMU_PROG_DIST(free_id), F_PF_DIR(1), F_PF_DIR(!!(dir)));
	m4uHw_set_field(m4u_base, REG_MMU_PROG_DIST(free_id), F_PF_DIST_MSB - F_PF_DIST_LSB + 1, F_PF_DIST_LSB, dist);
	m4uHw_set_field(m4u_base, REG_MMU_PROG_DIST(free_id), F_PF_ID_MSB - F_PF_ID_LSB + 1, F_PF_ID_LSB, F_PF_ID(larb, larb_port, mm_id));
	m4uHw_set_field_by_mask(m4u_base, REG_MMU_PROG_DIST(free_id), F_PF_EN(1), F_PF_EN(!!(en)));
	spin_unlock(&gM4u_reg_lock);

	return free_id;

}

int m4u_invalid_prog_dist_by_id(int port)
{
	int i;
	int m4u_index = m4u_port_2_m4u_id(port);
	int m4u_slave_id = m4u_port_2_m4u_slave_id(port);
	unsigned long m4u_base = gM4UBaseAddr[m4u_index];
	struct m4u_prog_dist *pProgPfh = gM4UProgPfh[m4u_index] + M4U_PROG_PFH_NUM(m4u_index) * m4u_slave_id;


	mutex_lock(&gM4u_prog_pfh_mutex);
	for (i = 0; i < M4U_PROG_PFH_NUM(m4u_index); i++) {
		if (pProgPfh[i].Enabled == 1 && port == pProgPfh[i].port && pProgPfh[i].sel == 0) {
			pProgPfh[i].Enabled = 0;
			break;
		}
	}
	mutex_unlock(&gM4u_prog_pfh_mutex);

	if (i == M4U_PROG_PFH_NUM(m4u_index)) return -1;

	spin_lock(&gM4u_reg_lock);
	M4U_WriteReg32(m4u_base, REG_MMU_PROG_DIST(i), 0x800);
	spin_unlock(&gM4u_reg_lock);

	return 0;

}

int m4u_insert_seq_range(M4U_PORT_ID port, unsigned int MVAStart, unsigned int MVAEnd)
{
	int i, free_id = -1;
	unsigned int m4u_index = m4u_port_2_m4u_id(port);
	unsigned int m4u_slave_id = m4u_port_2_m4u_slave_id(port);
	struct m4u_range_des *pSeq = gM4USeq[m4u_index] + M4U_SEQ_NUM(m4u_index) * m4u_slave_id;


	if (MVAEnd - MVAStart < PAGE_SIZE) return free_id;

	MVAStart &= ~M4U_SEQ_ALIGN_MSK;
	MVAEnd |= M4U_SEQ_ALIGN_MSK;

	mutex_lock(&gM4u_seq_mutex);
	for (i = 0; i < M4U_SEQ_NUM(m4u_index); i++) {
		if (pSeq[i].Enabled == 1) {
			if (!(MVAEnd < pSeq[i].MVAStart || MVAStart > pSeq[i].MVAEnd)) {
				mutex_unlock(&gM4u_seq_mutex);
				return -1;
			}
		} else free_id = i;
	}

	if (free_id == -1) {
		mutex_unlock(&gM4u_seq_mutex);
		return -1;
	}

	pSeq[free_id].Enabled = 1;
	pSeq[free_id].port = port;
	pSeq[free_id].MVAStart = MVAStart;
	pSeq[free_id].MVAEnd = MVAEnd;
	mutex_unlock(&gM4u_seq_mutex);

	MVAStart &= F_SQ_VA_MASK;
	MVAStart |= F_SQ_EN_BIT;
	MVAEnd |= ~F_SQ_VA_MASK;

	spin_lock(&gM4u_reg_lock);
	M4U_WriteReg32(gM4UBaseAddr[m4u_index], REG_MMU_SQ_START(m4u_slave_id, free_id), MVAStart);
	M4U_WriteReg32(gM4UBaseAddr[m4u_index], REG_MMU_SQ_END(m4u_slave_id, free_id), MVAEnd);
	spin_unlock(&gM4u_reg_lock);

	return free_id;

}

int m4u_invalid_seq_range_by_id(int port, int seq_id)
{
	int m4u_index = m4u_port_2_m4u_id(port);
	int m4u_slave_id = m4u_port_2_m4u_slave_id(port);
	unsigned long m4u_base = gM4UBaseAddr[m4u_index];
	struct m4u_range_des *pSeq = gM4USeq[m4u_index] + M4U_SEQ_NUM(m4u_index) * m4u_slave_id;


	mutex_lock(&gM4u_seq_mutex);
	pSeq[seq_id].Enabled = 0;
	mutex_unlock(&gM4u_seq_mutex);

	spin_lock(&gM4u_reg_lock);
	M4U_WriteReg32(m4u_base, REG_MMU_SQ_START(m4u_slave_id, seq_id), 0);
	M4U_WriteReg32(m4u_base, REG_MMU_SQ_END(m4u_slave_id, seq_id), 0);
	spin_unlock(&gM4u_reg_lock);

	return 0;

}

static int _m4u_config_port(int port, int virt, int sec, int dis, int dir)
{
	int m4u_index = m4u_port_2_m4u_id(port);
	unsigned long larb_base;
	unsigned int larb, larb_port;


	if (dir != 0 || dis != 1) m4u_config_prog_dist(port, dir, dis, 1, 0, 0);
	else m4u_invalid_prog_dist_by_id(port);
	if (virt == 0) m4u_invalid_prog_dist_by_id(port);

	spin_lock(&gM4u_reg_lock);
	if (m4u_index == 0) {
		larb = m4u_port_2_larb_id(port);
		larb_port = m4u_port_2_larb_port(port);

		if (likely(larb < SMI_LARB_NR)) {
			larb_base = gLarbBaseAddr[larb];
			m4uHw_set_field_by_mask(larb_base, SMI_LARB_NON_SEC_CONx(larb_port), F_SMI_MMU_EN, !!(virt));

			#ifdef M4U_GZ_SERVICE_ENABLE
			if (virt == 1 && sec == 0)
				m4uHw_set_field_by_mask(larb_base, SMI_LARB_NON_SEC_CONx(larb_port), F_SMI_BIT32, 0);
			#endif
		}
	} else {
		larb_port = m4u_port_2_larb_port(port);
		m4uHw_set_field_by_mask(gPericfgBaseAddr, REG_PERIAXI_BUS_CTL3, F_PERI_MMU_EN(larb_port, 1), F_PERI_MMU_EN(larb_port, !!(virt)));
	}
	spin_unlock(&gM4u_reg_lock);
	return 0;
}

int m4u_config_port(struct m4u_port_config_struct *pM4uPort)
{
	M4U_PORT_ID PortID = (pM4uPort->ePortID);
	unsigned int larb = m4u_port_2_larb_id(PortID);


	if (unlikely(larb >= SMI_LARB_NR)) return -1;

	if (m4u_port_2_m4u_id(PortID) == 0) larb_clock_on(larb, 1);

	#if !defined(M4U_GZ_SERVICE_ENABLE) && defined(M4U_TEE_SERVICE_ENABLE)
	if (m4u_tee_en) m4u_config_port_tee(pM4uPort);
	else
		#endif
		_m4u_config_port(PortID, pM4uPort->Virtuality, pM4uPort->Security, pM4uPort->Distance, pM4uPort->Direction);


	if (m4u_port_2_m4u_id(PortID) == 0) larb_clock_off(larb, 1);
	return 0;

}

int m4u_config_port_ext(struct m4u_port_config_struct *pM4uPort)
{
	return m4u_config_port(pM4uPort);
}

void m4u_port_array_init(struct m4u_port_array *port_array)
{
	memset(port_array, 0, sizeof(struct m4u_port_array));
}

int m4u_port_array_add(struct m4u_port_array *port_array, int port, int m4u_en, int secure)
{
	if (port >= M4U_PORT_NR || port < 0) return -1;
	port_array->ports[port] = M4U_PORT_ATTR_EN | (m4u_en ? M4U_PORT_ATTR_VIRTUAL : 0) | (secure ? M4U_PORT_ATTR_SEC : 0);
	return 0;
}

int m4u_config_port_array(struct m4u_port_array *port_array)
{
	unsigned int port, larb, larb_port;
	unsigned int config_larb[SMI_LARB_NR] = { 0 };
	unsigned int regNew[SMI_LARB_NR][32] = { {0} };


	for (port = 0; port < M4U_PORT_NR; port++) {
		if (port_array->ports[port] && M4U_PORT_ATTR_EN != 0) {
			larb = m4u_port_2_larb_id(port);
			larb_port = m4u_port_2_larb_port(port);
			if (likely(larb < SMI_LARB_NR)) {
				config_larb[larb] |= (1 << larb_port);
				regNew[larb][larb_port] = !!port_array->ports[port] && !!M4U_PORT_ATTR_VIRTUAL;
			}
		}
	}

	for (larb = 0; larb < SMI_LARB_NR; larb++)
		if (config_larb[larb] != 0) larb_clock_on(larb, 1);

		for (port = 0; port < gM4u_port_num; port++) {
			if ((port_array->ports[port] && M4U_PORT_ATTR_EN) == 0) continue;
			if (m4u_port_2_m4u_id(port) == 0) {
				larb = m4u_port_2_larb_id(port);
				larb_port = m4u_port_2_larb_port(port);
				if (likely(larb < SMI_LARB_NR)) {
					unsigned int orig_value = m4uHw_get_field_by_mask(gLarbBaseAddr[larb], SMI_LARB_NON_SEC_CONx(larb_port), F_SMI_NON_SEC_MMU_EN(1));
					if (orig_value != regNew[larb][larb_port]) {
						spin_lock(&gM4u_reg_lock);
						m4uHw_set_field_by_mask(gLarbBaseAddr[larb], SMI_LARB_NON_SEC_CONx(larb_port), F_SMI_MMU_EN, F_SMI_NON_SEC_MMU_EN(!!(regNew[larb][larb_port])));
						spin_unlock(&gM4u_reg_lock);
					}
				}
			}
		}

		for (larb = 0; larb < SMI_LARB_NR; larb++)
			if (config_larb[larb] != 0) larb_clock_off(larb, 1);

			return 0;

}

int m4u_monitor_start(int m4u_id)
{
	unsigned long m4u_base = gM4UBaseAddr[m4u_id];
	m4uHw_set_field_by_mask(m4u_base, REG_MMU_CTRL_REG, F_MMU_CTRL_MONITOR_CLR(1), F_MMU_CTRL_MONITOR_CLR(1));
	m4uHw_set_field_by_mask(m4u_base, REG_MMU_CTRL_REG, F_MMU_CTRL_MONITOR_CLR(1), F_MMU_CTRL_MONITOR_CLR(0));
	m4uHw_set_field_by_mask(m4u_base, REG_MMU_CTRL_REG, F_MMU_CTRL_MONITOR_EN(1), F_MMU_CTRL_MONITOR_EN(1));
	return 0;
}

int m4u_monitor_stop(int m4u_id)
{
	unsigned long m4u_base = gM4UBaseAddr[m4u_id];
	m4uHw_set_field_by_mask(m4u_base, REG_MMU_CTRL_REG, F_MMU_CTRL_MONITOR_EN(1), F_MMU_CTRL_MONITOR_EN(0));
	return 0;
}

#define M4U_REG_BACKUP_SIZE (100*sizeof(unsigned int))
static unsigned int *pM4URegBackUp;

#define __M4U_BACKUP(base, reg, back) ((back) = M4U_ReadReg32(base, reg))
#define __M4U_RESTORE(base, reg, back) writel_relaxed(back, (void __iomem *)((base) + (reg)))

int m4u_reg_backup(void)
{
	unsigned int *pReg = pM4URegBackUp;
	unsigned long m4u_base;
	int m4u_id, m4u_slave, seq, mau, dist;


	for (m4u_id = 0; m4u_id < TOTAL_M4U_NUM; m4u_id++) {
		m4u_base = gM4UBaseAddr[m4u_id];
		__M4U_BACKUP(m4u_base, REG_MMUg_PT_BASE, *(pReg++));
		__M4U_BACKUP(m4u_base, REG_MMUg_PT_BASE_SEC, *(pReg++));
		__M4U_BACKUP(m4u_base, REG_MMU_SEC_ABORT_INFO, *(pReg++));
		__M4U_BACKUP(m4u_base, REG_MMU_STANDARD_AXI_MODE, *(pReg++));
		__M4U_BACKUP(m4u_base, REG_MMU_PRIORITY, *(pReg++));
		__M4U_BACKUP(m4u_base, REG_MMU_DCM_DIS, *(pReg++));
		__M4U_BACKUP(m4u_base, REG_MMU_WR_LEN, *(pReg++));
		__M4U_BACKUP(m4u_base, REG_MMU_HW_DEBUG, *(pReg++));
		__M4U_BACKUP(m4u_base, REG_MMU_NON_BLOCKING_DIS, *(pReg++));
		__M4U_BACKUP(m4u_base, REG_MMU_LEGACY_4KB_MODE, *(pReg++));
		for (dist = 0; dist < MMU_TOTAL_PROG_DIST_NR; dist++)
			__M4U_BACKUP(m4u_base, REG_MMU_PROG_DIST(dist), *(pReg++));
		__M4U_BACKUP(m4u_base, REG_MMU_CTRL_REG, *(pReg++));
		__M4U_BACKUP(m4u_base, REG_MMU_IVRP_PADDR, *(pReg++));
		__M4U_BACKUP(m4u_base, REG_MMU_INT_L2_CONTROL, *(pReg++));
		__M4U_BACKUP(m4u_base, REG_MMU_INT_MAIN_CONTROL, *(pReg++));

		for (m4u_slave = 0; m4u_slave < M4U_SLAVE_NUM(m4u_id); m4u_slave++) {
			for (seq = 0; seq < M4U_SEQ_NUM(m4u_id); seq++) {
				__M4U_BACKUP(m4u_base, REG_MMU_SQ_START(m4u_slave, seq), *(pReg++));
				__M4U_BACKUP(m4u_base, REG_MMU_SQ_END(m4u_slave, seq), *(pReg++));
			}
			for (mau = 0; mau < MAU_NR_PER_M4U_SLAVE; mau++) {
				__M4U_BACKUP(m4u_base, REG_MMU_MAU_START(m4u_slave, mau), *(pReg++));
				__M4U_BACKUP(m4u_base, REG_MMU_MAU_START_BIT32(m4u_slave, mau), *(pReg++));
				__M4U_BACKUP(m4u_base, REG_MMU_MAU_END(m4u_slave, mau), *(pReg++));
				__M4U_BACKUP(m4u_base, REG_MMU_MAU_END_BIT32(m4u_slave, mau), *(pReg++));
				__M4U_BACKUP(m4u_base, REG_MMU_MAU_PORT_EN(m4u_slave, mau), *(pReg++));
			}
			__M4U_BACKUP(m4u_base, REG_MMU_MAU_LARB_EN(m4u_slave), *(pReg++));
			__M4U_BACKUP(m4u_base, REG_MMU_MAU_IO(m4u_slave), *(pReg++));
			__M4U_BACKUP(m4u_base, REG_MMU_MAU_RW(m4u_slave), *(pReg++));
			__M4U_BACKUP(m4u_base, REG_MMU_MAU_VA(m4u_slave), *(pReg++));
		}
	}
	return 0;

}

int m4u_reg_restore(void)
{
	unsigned int *pReg = pM4URegBackUp;
	unsigned long m4u_base;
	int m4u_id, m4u_slave, seq, mau, dist;


	for (m4u_id = 0; m4u_id < TOTAL_M4U_NUM; m4u_id++) {
		m4u_base = gM4UBaseAddr[m4u_id];
		__M4U_RESTORE(m4u_base, REG_MMUg_PT_BASE, *(pReg++));
		__M4U_RESTORE(m4u_base, REG_MMUg_PT_BASE_SEC, *(pReg++));
		__M4U_RESTORE(m4u_base, REG_MMU_SEC_ABORT_INFO, *(pReg++));
		__M4U_RESTORE(m4u_base, REG_MMU_STANDARD_AXI_MODE, *(pReg++));
		__M4U_RESTORE(m4u_base, REG_MMU_PRIORITY, *(pReg++));
		__M4U_RESTORE(m4u_base, REG_MMU_DCM_DIS, *(pReg++));
		__M4U_RESTORE(m4u_base, REG_MMU_WR_LEN, *(pReg++));
		__M4U_RESTORE(m4u_base, REG_MMU_HW_DEBUG, *(pReg++));
		__M4U_RESTORE(m4u_base, REG_MMU_NON_BLOCKING_DIS, *(pReg++));
		__M4U_RESTORE(m4u_base, REG_MMU_LEGACY_4KB_MODE, *(pReg++));
		for (dist = 0; dist < MMU_TOTAL_PROG_DIST_NR; dist++)
			__M4U_RESTORE(m4u_base, REG_MMU_PROG_DIST(dist), *(pReg++));

		__M4U_RESTORE(m4u_base, REG_MMU_CTRL_REG, *(pReg++));
		__M4U_RESTORE(m4u_base, REG_MMU_IVRP_PADDR, *(pReg++));
		__M4U_RESTORE(m4u_base, REG_MMU_INT_L2_CONTROL, *(pReg++));
		__M4U_RESTORE(m4u_base, REG_MMU_INT_MAIN_CONTROL, *(pReg++));

		for (m4u_slave = 0; m4u_slave < M4U_SLAVE_NUM(m4u_id); m4u_slave++) {
			for (seq = 0; seq < M4U_SEQ_NUM(m4u_id); seq++) {
				__M4U_RESTORE(m4u_base, REG_MMU_SQ_START(m4u_slave, seq), *(pReg++));
				__M4U_RESTORE(m4u_base, REG_MMU_SQ_END(m4u_slave, seq), *(pReg++));
			}
			for (mau = 0; mau < MAU_NR_PER_M4U_SLAVE; mau++) {
				__M4U_RESTORE(m4u_base, REG_MMU_MAU_START(m4u_slave, mau), *(pReg++));
				__M4U_RESTORE(m4u_base, REG_MMU_MAU_START_BIT32(m4u_slave, mau), *(pReg++));
				__M4U_RESTORE(m4u_base, REG_MMU_MAU_END(m4u_slave, mau), *(pReg++));
				__M4U_RESTORE(m4u_base, REG_MMU_MAU_END_BIT32(m4u_slave, mau), *(pReg++));
				__M4U_RESTORE(m4u_base, REG_MMU_MAU_PORT_EN(m4u_slave, mau), *(pReg++));
			}
			__M4U_RESTORE(m4u_base, REG_MMU_MAU_LARB_EN(m4u_slave), *(pReg++));
			__M4U_RESTORE(m4u_base, REG_MMU_MAU_IO(m4u_slave), *(pReg++));
			__M4U_RESTORE(m4u_base, REG_MMU_MAU_RW(m4u_slave), *(pReg++));
			__M4U_RESTORE(m4u_base, REG_MMU_MAU_VA(m4u_slave), *(pReg++));
		}
		m4uHw_set_field_by_mask(m4u_base, REG_MMU_DUMMY, F_REG_MMU_IDLE_ENABLE, 0);
	}
	wmb();
	return 0;
}

static unsigned int larb_reg_backup_buf[SMI_LARB_NR][64];

void m4u_larb_backup(unsigned int larb_idx)
{
	unsigned long larb_base = gLarbBaseAddr[larb_idx];
	unsigned int i;
	if (larb_idx >= SMI_LARB_NR) return;
	for (i = 0; i < 32; i++)
		__M4U_BACKUP(larb_base, SMI_LARB_NON_SEC_CONx(i), larb_reg_backup_buf[larb_idx][i]);
}

void m4u_larb_restore(unsigned int larb_idx)
{
	unsigned long larb_base = gLarbBaseAddr[larb_idx];
	unsigned int i;
	if (larb_idx >= SMI_LARB_NR) return;
	for (i = 0; i < 32; i++)
		__M4U_RESTORE(larb_base, SMI_LARB_NON_SEC_CONx(i), larb_reg_backup_buf[larb_idx][i]);
	wmb();
}

static unsigned int larb0_cnt;

void m4u_larb0_enable(char *name)
{
	mutex_lock(&m4u_larb0_mutex);
	larb_clock_on(0, 1);
	if (larb0_cnt == 0) m4u_larb_restore(0);
	larb0_cnt++;
	mutex_unlock(&m4u_larb0_mutex);
}

void m4u_larb0_disable(char *name)
{
	mutex_lock(&m4u_larb0_mutex);
	larb0_cnt--;
	if (larb0_cnt == 0) m4u_larb_backup(0);
	larb_clock_off(0, 1);
	mutex_unlock(&m4u_larb0_mutex);
}

int m4u_register_reclaim_callback(int port, m4u_reclaim_mva_callback_t *fn, void *data)
{
	if ((unsigned int)port >= M4U_PORT_UNKNOWN) return -1;
	gM4uPort[port].reclaim_fn = fn;
	gM4uPort[port].reclaim_data = data;
	return 0;
}

int m4u_unregister_reclaim_callback(int port)
{
	if ((unsigned int)port >= M4U_PORT_UNKNOWN) return -1;
	gM4uPort[port].reclaim_fn = NULL;
	gM4uPort[port].reclaim_data = NULL;
	return 0;
}

int m4u_reclaim_notify(int port, unsigned int mva, unsigned int size)
{
	int i;
	for (i = 0; i < M4U_PORT_UNKNOWN; i++) {
		if (gM4uPort[i].reclaim_fn)
			gM4uPort[i].reclaim_fn(port, mva, size, gM4uPort[i].reclaim_data);
	}
	return 0;
}

int m4u_register_fault_callback(int port, m4u_fault_callback_t *fn, void *data)
{
	if ((unsigned int)port >= M4U_PORT_UNKNOWN) return -1;
	gM4uPort[port].fault_fn = fn;
	gM4uPort[port].fault_data = data;
	return 0;
}

int m4u_unregister_fault_callback(int port)
{
	if ((unsigned int)port >= M4U_PORT_UNKNOWN) return -1;
	gM4uPort[port].fault_fn = NULL;
	gM4uPort[port].fault_data = NULL;
	return 0;
}

int m4u_enable_tf(int port, bool fgenable)
{
	if ((unsigned int)port >= M4U_PORT_UNKNOWN) return -1;
	gM4uPort[port].enable_tf = fgenable;
	return 0;
}

static struct timer_list m4u_isr_pause_timer;

static void m4u_isr_restart(struct timer_list *unused)
{
	m4u_intr_modify_all(1);
}

static int m4u_isr_pause_timer_init(void)
{
	timer_setup(&m4u_isr_pause_timer, m4u_isr_restart, 0);
	return 0;
}

static int m4u_isr_pause(int delay)
{
	m4u_intr_modify_all(0);
	m4u_isr_pause_timer.expires = jiffies + delay * HZ;
	add_timer(&m4u_isr_pause_timer);
	return 0;
}

static void m4u_isr_record(void)
{
	static int m4u_isr_cnt;
	static unsigned long first_jiffies;

	if (!m4u_isr_cnt || time_after(jiffies, first_jiffies + m4u_isr_cnt * HZ)) {
		m4u_isr_cnt = 1;
		first_jiffies = jiffies;
	} else if (++m4u_isr_cnt >= 5) {
		m4u_isr_pause(10);
		m4u_isr_cnt = 0;
	}
}

irqreturn_t MTK_M4U_isr(int irq, void *dev_id)
{
	unsigned long m4u_base = gM4UBaseAddr[0];
	unsigned int IntrSrc, fault_mva;
	int m4u_slave_id, m4u_port;


	if (unlikely(irq != gM4uDev->irq_num[0])) return IRQ_NONE;

	M4U_ReadReg32(m4u_base, REG_MMU_L2_FAULT_ST);

	IntrSrc = M4U_ReadReg32(m4u_base, REG_MMU_MAIN_FAULT_ST);
	if (unlikely(IntrSrc)) {
		m4u_slave_id = (IntrSrc & (F_INT_MMU1_MAIN_MSK | F_INT_MMU1_MAU_MSK)) ? 1 : 0;
		fault_mva = M4U_ReadReg32(m4u_base, REG_MMU_FAULT_VA(m4u_slave_id));
		m4u_port = m4u_get_port_by_tf_id(0, M4U_ReadReg32(m4u_base, REG_MMU_INT_ID(m4u_slave_id)));

		if (IntrSrc & F_INT_TRANSLATION_FAULT(m4u_slave_id)) {
			int bypass_DISP_TF = 0;

			if (m4u_port == M4U_PORT_DISP_OVL0) {
				unsigned int valid_mva = 0, valid_size = 0;
				m4u_query_mva_info(fault_mva - 1, 0, &valid_mva, &valid_size);
				if (valid_mva && valid_size && fault_mva < (valid_mva + valid_size + SZ_4K))
					bypass_DISP_TF = 1;
			}

			if (m4u_port < gM4u_port_num && gM4uPort[m4u_port].enable_tf && !bypass_DISP_TF) {
				if (gM4uPort[m4u_port].fault_fn) {
					gM4uPort[m4u_port].fault_fn(m4u_port, fault_mva & F_MMU_FAULT_VA_MSK, gM4uPort[m4u_port].fault_data);
				}
			}
		}
		m4u_clear_intr(0);
		m4u_isr_record();
	}
	return IRQ_HANDLED;

}

struct m4u_domain *m4u_get_domain_by_port(M4U_PORT_ID port) { return &gM4uDomain; }
struct m4u_domain *m4u_get_domain_by_id(int id) { return &gM4uDomain; }
int m4u_get_domain_nr(void) { return 1; }

int m4u_reg_init(struct m4u_domain *m4u_domain, unsigned long ProtectPA, int m4u_id)
{
	unsigned int regval;
	int i;
	m4u_clock_on();


	if (m4u_id == 0) {
		struct device_node *node = NULL;
		for (i = 0; i < SMI_LARB_NR; i++) {
			node = of_find_compatible_node(NULL, NULL, gM4U_SMILARB[i]);
			if (node) {
				gLarbBaseAddr[i] = (unsigned long)of_iomap(node, 0);
				of_node_put(node);
			}
		}
	}
	if (m4u_id == 1) {
		struct device_node *node = of_find_compatible_node(NULL, NULL, "mediatek,pericfg");
		if (node) {
			gPericfgBaseAddr = (unsigned long)of_iomap(node, 0);
			of_node_put(node);
		}
	}

	M4U_WriteReg32(gM4UBaseAddr[m4u_id], REG_MMUg_PT_BASE, (unsigned int)m4u_domain->pgd_pa);
	M4U_WriteReg32(gM4UBaseAddr[m4u_id], REG_MMUg_PT_BASE_SEC, (unsigned int)m4u_domain->pgd_pa);

	regval = M4U_ReadReg32(gM4UBaseAddr[m4u_id], REG_MMU_CTRL_REG);
	if (m4u_id == 0) {
		regval = regval | F_MMU_CTRL_PFH_DIS(0) | F_MMU_CTRL_MONITOR_EN(0) | F_MMU_CTRL_MONITOR_CLR(0) | F_MMU_CTRL_TF_PROTECT_SEL(2) | F_MMU_CTRL_INT_HANG_EN(0);
	}
	M4U_WriteReg32(gM4UBaseAddr[m4u_id], REG_MMU_CTRL_REG, regval);
	M4U_WriteReg32(gM4UBaseAddr[m4u_id], REG_MMU_MMU_COHERENCE_EN, 0x3);
	M4U_WriteReg32(gM4UBaseAddr[m4u_id], REG_MMU_MMU_TABLE_WALK_DIS, 0);

	m4u_enable_intr(m4u_id);
	M4U_WriteReg32(gM4UBaseAddr[m4u_id], REG_MMU_IVRP_PADDR, (unsigned int)F_MMU_IVRP_PA_SET(ProtectPA));
	M4U_WriteReg32(gM4UBaseAddr[m4u_id], REG_MMU_DCM_DIS, 0);
	m4u_invalid_tlb_all(m4u_id);

	if (m4u_id == 0) {
		unsigned long m4u_base = gM4UBaseAddr[0];

		#ifdef CONFIG_MTK_SMI_EXT
		M4U_WriteReg32(m4u_base, REG_MMU_IN_ORDER_WR_EN, 0);
		#endif
		M4U_WriteReg32(m4u_base, REG_MMU_STANDARD_AXI_MODE, 0);
		m4uHw_set_field_by_mask(m4u_base, REG_MMU_WR_LEN, F_MMU_WR_THROT_DIS(3), F_MMU_WR_THROT_DIS(0));
		m4uHw_set_field_by_mask(m4u_base, REG_MMU_DUMMY, F_REG_MMU_IDLE_ENABLE, 0);
	}
	return 0;
}

int m4u_domain_init(struct m4u_device *m4u_dev, void *priv_reserve)
{
	memset(&gM4uDomain, 0, sizeof(gM4uDomain));
	gM4uDomain.pgsize_bitmap = M4U_PGSIZES;
	mutex_init(&gM4uDomain.pgtable_mutex);
	m4u_pgtable_init(m4u_dev, &gM4uDomain);
	m4u_mvaGraph_init(priv_reserve);
	return 0;
}

int m4u_reset(int m4u_id)
{
	m4u_invalid_tlb_all(m4u_id);
	m4u_clear_intr(m4u_id);
	return 0;
}

int m4u_hw_init(struct m4u_device *m4u_dev, int m4u_id)
{
	phys_addr_t ProtectPA;

	#ifdef M4U_4GBDRAM
	gM4U_4G_DRAM_Mode = enable_4G();
	#endif


	gM4UBaseAddr[m4u_id] = m4u_dev->m4u_base[m4u_id];
	if (!tf_protect_buffer) {
		tf_protect_buffer = kzalloc(TF_PROTECT_BUFFER_SIZE * 2, GFP_KERNEL);
		if (!tf_protect_buffer) return -ENOMEM;
	}
	ProtectPA = virt_to_phys((void *)ALIGN((unsigned long)tf_protect_buffer, TF_PROTECT_BUFFER_SIZE));

	if (!pM4URegBackUp) {
		pM4URegBackUp = kmalloc(M4U_REG_BACKUP_SIZE, GFP_KERNEL | __GFP_ZERO);
		if (!pM4URegBackUp) {
			kfree(tf_protect_buffer);
			tf_protect_buffer = NULL;
			return -ENOMEM;
		}
	}

	spin_lock_init(&gM4u_reg_lock);
	m4u_reg_init(&gM4uDomain, ProtectPA, m4u_id);

	if (request_irq(m4u_dev->irq_num[m4u_id], MTK_M4U_isr, IRQF_TRIGGER_NONE, "m4u", NULL))
		return -ENODEV;

	m4u_isr_pause_timer_init();
	m4u_monitor_start(m4u_id);

	mau_start_monitor(0, 0, 0, 1, 1, 0, 0, 0x0, 0xfffff, 0xffffffff, 0xffffffff);
	mau_start_monitor(0, 0, 1, 0, 1, 0, 0, 0x0, 0xfffff, 0xffffffff, 0xffffffff);

	if (m4u_id == 0) {
		struct m4u_port_config_struct port;
		port.Direction = 0;
		port.Distance = 1;
		port.domain = 0;
		port.Security = 0;
		port.Virtuality = 1;

		port.ePortID = M4U_PORT_MDP_RDMA0;
		m4u_config_port(&port);
		port.ePortID = M4U_PORT_MDP_WROT0;
		m4u_config_port(&port);
	}
	return 0;

}

int m4u_hw_deinit(struct m4u_device *m4u_dev, int m4u_id)
{
	free_irq(m4u_dev->irq_num[m4u_id], NULL);
	kfree(pM4URegBackUp);
	pM4URegBackUp = NULL;
	kfree(tf_protect_buffer);
	tf_protect_buffer = NULL;
	return 0;
}

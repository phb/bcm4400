
/******************************************************************************/
/*                                                                            */
/* Broadcom BCM4400 Linux Network Driver, Copyright (c) 2002 Broadcom         */
/* Corporation.                                                               */
/* All rights reserved.                                                       */
/*                                                                            */
/* This program is free software; you can redistribute it and/or modify       */
/* it under the terms of the GNU General Public License as published by       */
/* the Free Software Foundation, located in the file LICENSE.                 */
/*                                                                            */
/******************************************************************************/

#ifndef B44MM_H
#define B44MM_H


#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <inttypes.h>
#include <sys/slog.h>
#include <hw/inout.h>
#include <hw/pci.h>
#include <gulliver.h> //for endianness
#include <drvr/support.h>
#include <drvr/eth.h>
#include <drvr/mdi.h>
#include <sys/io-net.h>
#include <sys/pm.h>
#include <net/if_types.h>
#include <net/if_dl.h>
#include <net/if.h>
#include <sys/mman.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/dcmd_io-net.h>
#include <sys/neutrino.h>
#include <atomic.h>
#include <sys/slogcodes.h>
#include <assert.h>


#define NIC_INTERRUPT_EVENT 1
#define NIC_TIMER_EVENT 2
#define NIC_INTERRUPT_PRIORITY 5 

//12 upper bits is of the drivers use
#define BCM4400_TEMP_PACKET 0x79c00000

#define BCM_WOL 1

#ifdef __BIG_ENDIAN
#define BIG_ENDIAN_HOST 1
#endif

//XXX: is this correct?
#define MM_SWAP_LE16(x) ENDIAN_LE16(x)

#define MM_MEMWRITEL(ptr, val) (*(volatile unsigned int *)ptr = (val))
#define MM_MEMREADL(ptr) (*(volatile unsigned int *)ptr)

typedef volatile unsigned MM_ATOMIC_T;

#define MM_ATOMIC_SET(ptr, val) atomic_set(ptr, val)
#define MM_ATOMIC_READ(ptr) *ptr 
#define MM_ATOMIC_INC(ptr) atomic_add(ptr,1)
#define MM_ATOMIC_ADD(ptr, val) atomic_add(ptr,val )
#define MM_ATOMIC_DEC(ptr) atomic_sub(ptr,1)
#define MM_ATOMIC_SUB(ptr, val) atomic_sub(ptr,val)

//Memory barrier, not needed on QNX??
#define MM_MB() mem_barrier()
#define MM_WMB() mem_barrier()

#include "b44lm.h"
#include "b44queue.h"
#include "b44.h"

#if DBG
#define STATIC
#else
#define STATIC static
#endif

extern int b44_Packet_Desc_Size;

#define B44_MM_PACKET_DESC_SIZE b44_Packet_Desc_Size

DECLARE_QUEUE_TYPE(UM_RX_PACKET_Q, MAX_RX_PACKET_DESC_COUNT+1);

#define MAX_MEM 16

typedef uint32_t dma_addr_t;

typedef struct {
	nic_options_t nic;

	//bcm4400 specific options
	uint32_t flowcontrol;
} bcm4400_options_t;


typedef struct _UM_DEVICE_BLOCK {
	LM_DEVICE_BLOCK lm_dev;
	void *pci_handle;
	struct pci_dev_info pciinfo;
	io_net_self_t *ion;
	int reg_hdlp;
	uint16_t cell;
	int chid;
	int coid;
	int tid;
	int iid;
	timer_t timer;
	Nic_t *nic;
	NicTxRxCount_t bytecount;
	bcm4400_options_t options;
	pthread_spinlock_t	spin;
	pmd_attr_t	pmd;
/*	struct net_device *dev;
	struct pci_dev *pdev;
	struct net_device *next_module; */
	void *mem_list[MAX_MEM];
	dma_addr_t dma_list[MAX_MEM];
	int mem_size_list[MAX_MEM];
	int mem_list_num;
	char *name;
	int index;
	int opened;
	int delayed_link_ind; /* Delay link status during initial load */
	int timer_interval; //in nanoseconds.
	int link_interval; //how often we should update stats etc, every 10th timerev.
	int tx_full;
	int tx_queued;
	int line_speed;		/* in Mbps, 0 if link is down */
	unsigned maxpkts;
	UM_RX_PACKET_Q rx_out_of_buf_q; //packages which's buffer is "in io-net" is placed here, moved back to the real queue @ tx_complete.
	int rx_buf_repl_thresh;
	int rx_buf_repl_panic_thresh;
//	struct timer_list timer;
//	spinlock_t phy_lock;
	MM_ATOMIC_T interrupt;
	MM_ATOMIC_T intr_sem;
	MM_ATOMIC_T	asleep;
//	struct tasklet_struct tasklet;
//	struct net_device_stats stats;
} UM_DEVICE_BLOCK, *PUM_DEVICE_BLOCK;

typedef struct _UM_PACKET {
	LM_PACKET lm_packet;
	npkt_t	*npkt;
} UM_PACKET, *PUM_PACKET;

static inline void b44_MM_MapRxDma(PLM_DEVICE_BLOCK pDevice,
	struct _LM_PACKET *pPacket,
	LM_UINT32 *paddr)
{
PUM_PACKET pUmPacket = (PUM_PACKET)pPacket;
npkt_t * npkt = pUmPacket->npkt;
net_buf_t *nb = (net_buf_t *)TAILQ_FIRST(&npkt->buffers);

*paddr =(LM_UINT32) nb->net_iov->iov_phys;
}

static inline void b44_MM_MapTxDma(PLM_DEVICE_BLOCK pDevice,
	struct _LM_PACKET *pPacket,
	LM_UINT32 *paddr, LM_UINT32 *len, int frag)
{
	PUM_PACKET pUmPacket = (PUM_PACKET)pPacket;
	npkt_t *npkt = pUmPacket->npkt;
	net_buf_t *nb;
	net_iov_t *iov;
	int i;
	int fragid=0;

for (nb = TAILQ_FIRST (&npkt -> buffers); nb;
                        nb = TAILQ_NEXT (nb, ptrs)) {
    for (i = 0, iov = nb -> net_iov; i < nb->niov; i++, iov++) {
				if(fragid == frag) { 
					*paddr =(LM_UINT32) iov->iov_phys;
					*len = iov->iov_len; 
					return;
				}
				fragid++;
    }
}

}

#define B44_MM_PTR(_ptr)   ((unsigned long) (_ptr))

#if (BITS_PER_LONG == 64)
#define B44_MM_GETSTATS(_Ctr) \
	(unsigned long) (_Ctr).Low + ((unsigned long) (_Ctr).High << 32)
#else
#define B44_MM_GETSTATS(_Ctr) \
	(unsigned long) (_Ctr).Low
#endif


#define printf(fmt, args...) slogf(_SLOGC_NETWORK_NEXTFREE,_SLOG_DEBUG1,fmt,##args)
//was : printk(KERN_DEBUG fmt, ##args)

#define DbgPrint(fmt, arg...) slogf(_SLOGC_NETWORK_NEXTFREE,_SLOG_WARNING,fmt,##arg)

#define DbgRawPrint(fmt,size) slogb(_SLOGC_NETWORK_NEXTFREE,_SLOG_DEBUG1,fmt,size)
//was : printk(KERN_WARNING fmt, ##arg)

#if defined(CONFIG_X86)
#define DbgBreakPoint() __asm__("int $129")
#else
#define DbgBreakPoint()
#endif
//was udelay
#define b44_MM_Wait(time) usleep(time) 

//slogf
#define ASSERT(expr)							\
	if (!(expr)) {							\
		slogf(_SLOGC_NETWORK_NEXTFREE,_SLOG_DEBUG1,"ASSERT failed: %s\n",#expr); \
	}
//io-net callbacks : 
int bcm4400_rxfill(PUM_DEVICE_BLOCK pUmDevice);
int bcm4400_allocatenpkt(PUM_DEVICE_BLOCK pUmDevice,PUM_PACKET pUmPacket);
void bcm4400_setmcast(PLM_DEVICE_BLOCK pDevice,io_net_msg_join_mcast_t *mc);
void bcm4400_free_remaining_rx_bufs(PUM_DEVICE_BLOCK pUmDevice);
int bcm4400_freemem(PUM_DEVICE_BLOCK pUmDevice);
int bcm4400_shutdown1(int, void *hdl);
int bcm4400_shutdown2(int, void *hdl);
int bcm4400_devctl(void *hdl, int dcmd, void *data, size_t size, union _io_net_dcmd_ret_cred *ret);
int bcm4400_receive(Nic_t *nic);
int bcm4400_receive_complete(npkt_t *npkt, void *, void *);
int bcm4400_transmit_packets(npkt_t *npkt, void *);
int bcm4400_transmit(Nic_t *nic);
void *bcm4400_event_handler(void *data);
int bcm4400_collect_tx_packets(Nic_t *nic);
int bcm4400_reset(Nic_t *nic);
int bcm4400_advertise(int reg_hdl, void *func_hdl);
int bcm4400_flush(int, void *hdl);
int bcm4400_init(void *, dispatch_t *, io_net_self_t *, char *options);
void bcm4400_mii_init(Nic_t *nic);
int bcm4400_nettrap_detect(int *count);
void *bcm4400_event_handler(void *data);
int bcm4400_setpower(void *data,pm_power_mode_t mode,unsigned flags);
void bcm4400_powermgmt_init(Nic_t *nic);
#endif

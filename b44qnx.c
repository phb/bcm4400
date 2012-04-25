#include "b44mm.h"

/* exported symbol for io-net */
io_net_dll_entry_t io_net_dll_entry = 
{
	2, 
	bcm4400_init, 
	NULL 
};

/* callback functions for io-net to use to interface with driver */
io_net_registrant_funcs_t bcm4400Funcs = 
{
	8,
	NULL, 
	bcm4400_transmit_packets, //rx_down 
	bcm4400_receive_complete, //tx_done
	bcm4400_shutdown1,
	bcm4400_shutdown2,
	bcm4400_advertise,
	bcm4400_devctl, 
	bcm4400_flush, 
	NULL
};

/* io-net configuration structure, used during io-net registration */ 
io_net_registrant_t bcm4400Entry = 
{
	_REG_PRODUCER_UP,
	"devn-bcm4400",
	"en",
	NULL,
	NULL,
	&bcm4400Funcs,
	0
};
int b44_Packet_Desc_Size = sizeof(UM_PACKET);

#define BCM4400_OFFSET(a) offsetof(bcm4400_options_t,a)
static drvr_subopt_tbl_t bcm4400OptionsTable[] = 
{
	{"flowcontrol",BCM4400_OFFSET(flowcontrol),DRVR_OPT_FLAG_VAL_UINT32,0},
};

int bcm4400_init (void *dll_hdl, dispatch_t *dpp, io_net_self_t *ion,
         char *options)
{
	PUM_DEVICE_BLOCK handle;
	struct itimerspec tvalue;
	struct sigevent event;
	uint16_t lan;
	pthread_attr_t	pattr;
	struct sched_param	param;
	Nic_t *nic;
	int ret;
	drvr_subopt_tbl_chain_t link;




	if(pci_attach(0) == -1 ) {
		DbgPrint("Can't attach to the pci server! %d",errno);
		return errno;
	}

	/* let's alloc a nic structure and put our PUM_DEVICE_BLOCK inside it */
	nic = nic_create_dev(sizeof(UM_DEVICE_BLOCK));
	handle=nic->ext;
	memset(handle,0x0,sizeof(UM_DEVICE_BLOCK));
	handle->nic = nic; //so we can access the nic struct from the callbacks from the platform independant code.

	//setup a few defaults
	handle->options.nic.promiscuous = -1;
	handle->options.nic.nomulticast = -1;
	link.next = NULL;
	link.table = bcm4400OptionsTable;
	link.table_size = sizeof(bcm4400OptionsTable) / sizeof(drvr_subopt_tbl_t);
	if((ret = nic_parse_subopts(&handle->options,"devn-bcm4400",options,&link)) != EOK) {
		DbgPrint("Failure parsing commandline options");
		errno=ret;
		goto err1;
	}


	
	handle->pciinfo.DeviceId = (handle->options.nic.drvr.busdevice == 0) ? 0x4401 : handle->options.nic.drvr.busdevice;
	handle->pciinfo.VendorId = (handle->options.nic.drvr.busvendor == 0) ? 0x14e4 : handle->options.nic.drvr.busvendor;

	handle->pci_handle = pci_attach_device(NULL,PCI_INIT_ALL|PCI_SEARCH_VENDEV|PCI_MASTER_ENABLE,0,&handle->pciinfo);


	if(handle->pci_handle == NULL) {
		DbgPrint("Can't find any supported device! %s\n",strerror(errno));
		goto err1;
	}

	if(b44_LM_GetAdapterInfo(&handle->lm_dev) != LM_STATUS_SUCCESS) {
		DbgPrint("GetAdapterinfo failed\n");
		goto err2;
	}

	/* init Config params */
	nic->cfg.Device_ID.DevID = (uint32_t)handle->pciinfo.VendorId|(handle->pciinfo.DeviceId << 16);
	nic->cfg.BusAccess.PCIAccess.BusNumber = handle->pciinfo.BusNumber;
	nic->cfg.BusAccess.PCIAccess.DevFuncNumber = handle->pciinfo.DevFunc;
	nic->cfg.NumIRQs = 1;
	nic->cfg.IRQRegisters[0] = handle->pciinfo.Irq;
	strcpy(nic->cfg.Description,"BMC4400");
	/*initialize the card and setup interrupt handling and such */
	handle->chid = ChannelCreate(_NTO_CHF_DISCONNECT | _NTO_CHF_UNBLOCK);
	if(handle->chid == -1) {
		goto err3;
	}

	handle->coid = ConnectAttach(0,0,handle->chid,_NTO_SIDE_CHANNEL,0);
	if(handle->coid == -1) {
		goto err4;
	}

	event.sigev_notify = SIGEV_PULSE;
	event.sigev_coid = handle->coid;
	event.sigev_code = NIC_INTERRUPT_EVENT;
	event.sigev_priority = 15;

	handle->iid = InterruptAttachEvent(handle->pciinfo.Irq,&event,_NTO_INTR_FLAGS_TRK_MSK);
	if(handle->iid == -1) {
		goto err5;
	}
	pthread_attr_init(&pattr);
	pthread_attr_setschedpolicy(&pattr,SCHED_RR);
	param.sched_priority = 15;
	pthread_attr_setschedparam(&pattr,&param);	
	pthread_attr_setinheritsched(&pattr, PTHREAD_EXPLICIT_SCHED);	
	event.sigev_code = NIC_TIMER_EVENT;
	tvalue.it_value.tv_sec = 0; 
	tvalue.it_interval.tv_sec = 0;
	tvalue.it_value.tv_nsec = tvalue.it_interval.tv_nsec = handle->timer_interval;

	if(timer_create(CLOCK_REALTIME,&event,&handle->timer) == -1) {
		goto err6;
	}
	timer_settime(handle->timer,0,&tvalue,0);

	bcm4400Entry.func_hdl = nic;
	handle->ion = ion;
	handle->delayed_link_ind = 4; //wait to check link status until the 4th timer ev.
	if(b44_LM_InitializeAdapter(&handle->lm_dev) != LM_STATUS_SUCCESS) {
		DbgPrint("InitAdapter FAIIILED!!!");
		goto err7;
	}

	pthread_spin_init(&handle->spin,PTHREAD_PROCESS_PRIVATE);

	nic->lan = lan;
	QQ_InitQueue(&handle->rx_out_of_buf_q.Container, MAX_RX_PACKET_DESC_COUNT);
	handle->opened=1;
	pthread_create(&handle->tid,&pattr,bcm4400_event_handler,handle);
	MM_ATOMIC_SET(&handle->intr_sem,0);

	bcm4400_powermgmt_init(nic);
	bcm4400Entry.flags |= _REG_POWER_MNGR;
	bcm4400Entry.pmd_attrp = &handle->pmd;

	if(ion->reg(dll_hdl,&bcm4400Entry,&handle->reg_hdlp,&handle->cell,&lan) < 0) {
		goto err7;
	}

	handle->maxpkts = 100;
	ion->devctl(handle->reg_hdlp,DCMD_IO_NET_MAX_QUEUE,&handle->maxpkts,sizeof(handle->maxpkts),NULL);

	b44_LM_EnableInterrupt(&handle->lm_dev);
	bcm4400_advertise(handle->reg_hdlp,bcm4400Entry.func_hdl);
return EOK;

//errors...
err7:
b44_LM_Halt(&handle->lm_dev);
err6:
pthread_attr_destroy(&pattr);
timer_delete(handle->timer);
err5:
InterruptDetach(handle->iid);
err4:
ConnectDetach(handle->coid);
err3:
ChannelDestroy(handle->chid);
err2:
munmap_device_memory(handle->lm_dev.pMappedMemBase,sizeof(bcmenetregs_t)+128); //allocated from  b44_LM_GetAdapterInfo..
pci_detach_device(handle->pci_handle);
err1:
free(nic->ext);
free(nic);

return ENODEV;
}

//IO-net callbacks:
int bcm4400_transmit_packets(npkt_t *npkt,
                void *func_hdl)
{
	Nic_t *nic = (Nic_t *)func_hdl;
	PUM_DEVICE_BLOCK pUmDevice = (PUM_DEVICE_BLOCK)nic->ext;
	PLM_DEVICE_BLOCK pDevice = (PLM_DEVICE_BLOCK) pUmDevice;
	PLM_PACKET pPacket;
	PUM_PACKET pUmPacket;
	io_net_msg_join_mcast_t *msg;
	net_buf_t		*buf;

	if(npkt->flags & _NPKT_MSG) {
		buf = TAILQ_FIRST(&npkt->buffers);
		if(buf != NULL && NULL != (msg = (io_net_msg_join_mcast_t*)buf->net_iov->iov_base)) 
		{
			bcm4400_setmcast(pDevice,msg);
		}
		pUmDevice->ion->tx_done(pUmDevice->reg_hdlp,npkt);
		return TX_DOWN_OK;
	}
	if((pDevice->LinkStatus == LM_STATUS_LINK_DOWN) || !pDevice->InitDone || npkt->next) {
		DbgPrint("Link is down/init not done/multiple npkts.");
		npkt->flags |= _NPKT_NOT_TXED;
		pUmDevice->ion->tx_done(pUmDevice->reg_hdlp,npkt);
		return TX_DOWN_FAILED;
	}

	if(MM_ATOMIC_READ(&pUmDevice->asleep)) {
		npkt->flags |= _NPKT_NOT_TXED;
		pUmDevice->ion->tx_done(pUmDevice->reg_hdlp,npkt);
		errno = ENETDOWN;
		return TX_DOWN_FAILED;
	}
		pPacket = (PLM_PACKET) QQ_PopHead(&pDevice->TxPacketFreeQ.Container);
		if(pPacket == 0) {
			DbgPrint("SQUEEEK, no free packages!");
			npkt->flags |= _NPKT_NOT_TXED;
			pUmDevice->ion->tx_done(pUmDevice->reg_hdlp,npkt);
			return TX_DOWN_FAILED;
		}

	if(MM_ATOMIC_READ(&pDevice->SendDescLeft) == 0) {
		pUmDevice->tx_full=1;
		DbgPrint("NOOOO, no senddesc left!");
		pUmDevice->ion->tx_done(pUmDevice->reg_hdlp,npkt);
		return TX_DOWN_FAILED;
	}

	pUmPacket = (PUM_PACKET)pPacket;
	pUmPacket->npkt = npkt;
	pPacket->u.Tx.FragCount = npkt->tot_iov;
	pUmDevice->bytecount.tx_bytes+=npkt->framelen;
	pUmDevice->nic->nstats.gstats.xmit_ok++;
	if(pUmDevice->options.nic.drvr.verbose > 3)
		DbgPrint("transmitted a package of %d bytes",npkt->framelen);

	pthread_spin_lock(&pUmDevice->spin);
	b44_LM_SendPacket(pDevice,pPacket);
	pthread_spin_unlock(&pUmDevice->spin);

return TX_DOWN_OK;
}

int bcm4400_receive_complete(npkt_t *npkt,
                void *done_hdl,
                void *func_hdl) 
{
	Nic_t *nic = (Nic_t *)func_hdl;
	PUM_DEVICE_BLOCK pUmDevice = (PUM_DEVICE_BLOCK)nic->ext;
	PLM_DEVICE_BLOCK pDevice = (PLM_DEVICE_BLOCK)nic->ext;
	PUM_PACKET pUmPacket;
	PLM_PACKET pPacket;
	net_buf_t *nb;

	if(npkt->flags & BCM4400_TEMP_PACKET ) { //this is a package that was allocated either because of a reaching a panic-low rxpkg cnt , or a package from the advert function.
			pUmDevice->ion->free(npkt->org_data);
			pUmDevice->ion->free(npkt);
	} else {
		pUmPacket = QQ_PopHead(&pUmDevice->rx_out_of_buf_q.Container);
		if(pUmPacket == NULL) {
			DbgPrint("HOW DID THIS HAPPEN?!");
			pUmDevice->ion->free(npkt->org_data);
			pUmDevice->ion->free(npkt);
			return 0;
		}
		nb = TAILQ_FIRST(&npkt->buffers);
		nb->net_iov->iov_base = npkt->org_data;
		nb->net_iov->iov_phys = pUmDevice->ion->mphys(nb->net_iov->iov_base); 
		pPacket = (PLM_PACKET)pUmPacket;
		pUmPacket->npkt = npkt;
		pPacket->u.Rx.pRxBufferVirt = npkt->org_data;
		QQ_PushTail(&pDevice->RxPacketFreeQ.Container,pPacket);
		b44_LM_QueueRxPackets(pDevice);
	}
return 0;
}


int bcm4400_shutdown1(int registrant_hdl,
                  void *func_hdl) {
DbgPrint("shutdown 1");
return EOK;
}



int bcm4400_shutdown2(int registrant_hdl,
                  void *func_hdl) {
Nic_t	*nic = (Nic_t *)func_hdl;
PUM_DEVICE_BLOCK pUmDevice = (PUM_DEVICE_BLOCK) nic->ext;
PLM_DEVICE_BLOCK pDevice = (PLM_DEVICE_BLOCK) nic->ext;

pUmDevice->opened = 0;

b44_LM_DisableInterrupt(pDevice);

InterruptDetach(pUmDevice->iid);
timer_delete(pUmDevice->timer);
ConnectDetach(pUmDevice->coid);
ChannelDestroy(pUmDevice->chid);
pthread_join(pUmDevice->tid, NULL);

b44_LM_Halt(pDevice);
pDevice->InitDone = 0;

munmap_device_memory(pDevice->pMappedMemBase,sizeof(bcmenetregs_t)+128);
pci_detach_device(pUmDevice->pci_handle);
bcm4400_free_remaining_rx_bufs(pUmDevice);
bcm4400_freemem(pUmDevice);
free(nic->ext);
free(nic);

return 0;
}


int bcm4400_advertise(int registrant_hdl,
                  void *func_hdl) {

	  npkt_t                  *npkt;
		net_buf_t               *nb;
    net_iov_t               *iov;
    io_net_msg_dl_advert_t  *ap;
		Nic_t										*nic = (Nic_t *)func_hdl;
		PUM_DEVICE_BLOCK handle = (PUM_DEVICE_BLOCK )nic->ext;

		if(handle->options.nic.drvr.verbose > 4) {
			DbgPrint("Advert networkdriver!");
		}
    // 1) Allocate a packet; we'll use this for communications
    //    with io-net.
    if ((npkt = handle->ion->alloc_up_npkt (sizeof (*nb) + sizeof (*iov),
                                (void **) &nb)) == NULL) {
        return (0);
    }

    // 2) Allocate room for the advertisement message.
    if ((ap = handle->ion->alloc (sizeof (*ap), 0)) == NULL) {
        handle->ion->free (npkt);
        return (0);
    }

    // 3) Set up the packet into the queue.
    TAILQ_INSERT_HEAD (&npkt -> buffers, nb, ptrs);

    iov = (net_iov_t *) (nb + 1);

    nb -> niov = 1;
    nb -> net_iov = iov;
    iov -> iov_base = ap;
    iov -> iov_len = sizeof (*ap);

    // 4) Generate the info for the advertisement message.
    memset (ap, 0x00, sizeof (*ap));
    ap -> type          = _IO_NET_MSG_DL_ADVERT;
    ap -> iflags        = (IFF_SIMPLEX |IFF_RUNNING| IFF_BROADCAST);
		if(handle->options.nic.nomulticast == -1)
			ap->iflags |= IFF_MULTICAST;
		if(handle->options.nic.promiscuous != -1)
			ap->iflags |= IFF_PROMISC;

    ap -> mtu_min       = 0;
    ap -> mtu_max       = MAX_ETHERNET_PACKET_SIZE_NO_CRC;
    ap -> mtu_preferred = MAX_ETHERNET_PACKET_SIZE_NO_CRC;

		strcpy(ap->up_type,"en");
		itoa(nic->lan,ap->up_type+2,10);
   
		 strcpy (ap -> dl.sdl_data, ap -> up_type);

    ap -> dl.sdl_len = sizeof (struct sockaddr_dl);
    ap -> dl.sdl_family = AF_LINK;
    ap -> dl.sdl_index  = nic->lan;
    ap -> dl.sdl_type = IFT_ETHER;

    // Not terminated:
    ap -> dl.sdl_nlen = strlen (ap -> dl.sdl_data); 
    ap -> dl.sdl_alen = 6;
    memcpy (ap -> dl.sdl_data + ap -> dl.sdl_nlen,
            handle->lm_dev.NodeAddress, 6);

    // 5) Bind the advertisement message to the packet; note
    //    the use of the _NPKT_MSG flag to indicate to the 
    //    upper modules that this is a message intended for
    //    them. It isn't just a "regular" packet.
    npkt -> org_data = ap;
    npkt -> flags |= _NPKT_MSG | BCM4400_TEMP_PACKET;
    npkt -> iface = 0;
    npkt -> framelen = sizeof (*ap);
		npkt -> tot_iov = 1;
		npkt->next=NULL;

		if(handle->ion->reg_tx_done(handle->reg_hdlp,npkt,NULL) == -1 ) {
				DbgPrint("reg_tx_done @ advertise failed");
        handle->ion->free (ap);
        handle->ion->free (npkt);
        return (0);
    }
		if(handle->ion->tx_up(handle->reg_hdlp,npkt,0,0,handle->cell,nic->lan,0) <= 0) 		{
			DbgPrint("Advertise went wrong! %d: %s",errno,strerror(errno));
			handle->ion->tx_done(handle->reg_hdlp,npkt); 
		}
return 0;
}
int bcm4400_devctl(void *func_hdl, int dcmd, void *data, size_t size, union _io_net_dcmd_ret_cred *ret) 
{
	Nic_t *nic = (Nic_t *)func_hdl;
	PLM_DEVICE_BLOCK pDevice = (PLM_DEVICE_BLOCK)nic->ext;
	PUM_DEVICE_BLOCK pUmDevice = (PUM_DEVICE_BLOCK)pDevice;
	int promisc;
	int status=0;
	io_net_msg_join_mcast_t	*mc;

	pthread_spin_lock(&pUmDevice->spin);
	switch(dcmd) {
		case DCMD_IO_NET_NICINFO:
			nic->flags = 0;
			if(pDevice->DuplexMode == LM_DUPLEX_MODE_FULL)
					nic->flags |= NIC_FLAG_FDX; 
			if(pUmDevice->options.nic.promiscuous != -1)
				nic->flags |= NIC_FLAG_PROMISCUOUS;
			if(pUmDevice->options.nic.nomulticast == -1) 
				nic->flags |= NIC_FLAG_MULTICAST;

			nic->flags |= NIC_FLAG_UTP|NIC_FLAG_BMSTR|NIC_FLAG_BROADCAST;
			if(pDevice->LinkStatus != LM_STATUS_LINK_ACTIVE)
				nic->flags |= NIC_FLAG_LINK_DOWN;
			
			nic->media = NIC_MEDIA_802_3;
			nic->media_rate = pUmDevice->line_speed;
			nic->mac_length = 6;
			nic->mtu = MAX_ETHERNET_PACKET_SIZE_NO_CRC;
			//	nic->phy = PHY_NOT_INSTALLED;
			//		nic->nstats.gstats
			//add Multicast/Broadcast stats!!	
			nic->nstats.un.estats.rcv_alignment_error = pDevice->rx_align_errs;
//			nic->nstats.un.estats.xmit_collisions = pDevice->total_cols;
			nic->nstats.un.estats.rcv_overrun = 0;
//		nic->nstats.un.estats.xmit_underrun = pDevice->tx_underrun;
//		nic->nstats.un.estats.xmit_crs_lost =
			nic->nstats.un.estats.rcv_crc_error = pDevice->rx_crc_errs;
			nic->nstats.un.estats.rcv_length_error = pDevice->rx_oversize_pkts;
			

			memcpy(nic->permanent_address,pDevice->PermanentNodeAddress,6);	
			memcpy(nic->current_address,pDevice->NodeAddress,6);
			memcpy(data,nic,min(size,sizeof(Nic_t)));
		break;
		case DCMD_IO_NET_TXRX_COUNT:
			memcpy(data,&pUmDevice->bytecount,min(size,sizeof(NicTxRxCount_t)));
		break;
		case DCMD_IO_NET_PROMISCUOUS:
			promisc = *(int *)data;
			if(promisc) {
				nic->flags |= NIC_FLAG_PROMISCUOUS;
				b44_LM_SetReceiveMask(pDevice,pDevice->ReceiveMask | LM_PROMISCUOUS_MODE);
			}
			else {
				nic->flags &= ~NIC_FLAG_PROMISCUOUS;
				b44_LM_SetReceiveMask(pDevice,pDevice->ReceiveMask & ~LM_PROMISCUOUS_MODE);
			}
		break;
		case DCMD_IO_NET_CHANGE_MCAST:
			mc = (io_net_msg_join_mcast_t *)data;
			bcm4400_setmcast(pDevice,mc);
		break;
		case DCMD_IO_NET_TX_FLUSH:
			if(pUmDevice->options.nic.drvr.verbose > 6) //higher then 'max', really really useless info which clutters the output...
				DbgPrint("DCMD_IO_NET_TX_FLUSH");
		default:
			if(pUmDevice->options.nic.drvr.verbose > 6)
				DbgPrint("devctl not supported!! %x",dcmd);
			status = ENOTSUP;
		break;
	}
	pthread_spin_unlock(&pUmDevice->spin);
return status; 
}

int bcm4400_flush(int registrant_hdl,
              void *func_hdl) {
DbgPrint("bcm4400_flush --- WHEN ARE WE CALLED???");

return 0;
}

void *bcm4400_event_handler(void *data){

	PUM_DEVICE_BLOCK pUmDevice = (PUM_DEVICE_BLOCK)data;
	PLM_DEVICE_BLOCK pDevice = (PLM_DEVICE_BLOCK)pUmDevice;
	int rcvid;
	struct _pulse pulse;
	iov_t	iov;

	SETIOV(&iov,&pulse,sizeof(pulse));
	while(1) {
		rcvid = MsgReceivev(pUmDevice->chid,&iov,1,NULL);
		if(rcvid == -1) {
			if(errno == ESRCH)
				pthread_exit(NULL);
			continue;
		}
		if(!pUmDevice->opened) 
			break;

		pthread_spin_lock(&pUmDevice->spin);
		switch(pulse.code) {
			case NIC_INTERRUPT_EVENT:
			if(!pDevice->InitDone) {
				DbgPrint("Init not done!");
				break;
			}
			if(MM_ATOMIC_READ(&pUmDevice->asleep))
				break;
			b44_LM_ServiceInterrupts(&pUmDevice->lm_dev);

			if(QQ_GetEntryCnt(&pUmDevice->rx_out_of_buf_q.Container) > pUmDevice->rx_buf_repl_panic_thresh ) {
				//allocate new ones??
				bcm4400_rxfill(pUmDevice);
			}
			if(QQ_GetEntryCnt(&pDevice->RxPacketFreeQ.Container))
				b44_LM_QueueRxPackets(pDevice);	
			break;
			case NIC_TIMER_EVENT:
				if(pUmDevice->intr_sem || MM_ATOMIC_READ(&pUmDevice->asleep)) //timer will automatically rearm.
					break;
				if(pUmDevice->delayed_link_ind > 0) {
					pUmDevice->delayed_link_ind--;
					if(pUmDevice->delayed_link_ind == 0) {
						b44_MM_IndicateStatus(pDevice,pDevice->LinkStatus);
					}
				}
				if(REG_RD(pDevice,intstatus) & I_XI) { //no idea why we do this.. ???
					REG_WR(pDevice,gptimer,2); 

			/*XXX:	if((QQ_GetEntryCnt(&pDevice->TxPacketFreeQ.Container) != pDevice->TxPacketDescCnt) && ()) {
					DbgPrint("Warning, Tx hung!");
					bcm4400_tx_timeout(pUmDevice);
				}*/
				}
				if(QQ_GetEntryCnt(&pUmDevice->rx_out_of_buf_q.Container) > pUmDevice->rx_buf_repl_panic_thresh) {
				//generate interrupt.
					REG_WR(pDevice,gptimer,2);
				}
				if(pUmDevice->link_interval == 0) {
					b44_LM_PollLink(pDevice);
					if(pDevice->LinkStatus == LM_STATUS_LINK_ACTIVE)
						b44_LM_StatsUpdate(pDevice);
					pUmDevice->link_interval = 10; //pUmDevice->timer_interval;
				} else {
					pUmDevice->link_interval--;
				}
				b44_LM_GetStats(pDevice);
				break;
			default:
				if(rcvid)
					MsgReplyv (rcvid,ENOTSUP,&iov,1);
			break;
		}
		pthread_spin_unlock(&pUmDevice->spin);
		if(pulse.code == NIC_INTERRUPT_EVENT) 
			InterruptUnmask(pUmDevice->pciinfo.Irq,pUmDevice->iid);
	}
return 0;
}

int bcm4400_rxfill(PUM_DEVICE_BLOCK pUmDevice)
{
	PLM_PACKET pPacket;
	PUM_PACKET pUmPacket;
	PLM_DEVICE_BLOCK pDevice = (PLM_DEVICE_BLOCK) pUmDevice;
	int queue_rx = 0;


	if(pUmDevice->options.nic.drvr.verbose > 4) 
		DbgPrint("Entering rxfill, there is %d packages without buffer (%d in the freeQ)",QQ_GetEntryCnt(&pUmDevice->rx_out_of_buf_q.Container),QQ_GetEntryCnt(&pDevice->RxPacketFreeQ.Container));

	if (!pUmDevice->opened)
		return 0;

	while ((pUmPacket = (PUM_PACKET)
		QQ_PopHead(&pUmDevice->rx_out_of_buf_q.Container)) != 0) {
		pPacket = (PLM_PACKET) pUmPacket;
		if (pUmPacket->npkt) {
			/* reuse an old npkt */
			QQ_PushTail(&pDevice->RxPacketFreeQ.Container, pPacket);
			queue_rx = 1;
			continue;
		}
		if(bcm4400_allocatenpkt(pUmDevice,pUmPacket)==-1){
			QQ_PushTail(&pUmDevice->rx_out_of_buf_q.Container,pPacket);	
			continue;
		}
		//indicate we don't want to keep this package..
		pUmPacket->npkt->flags |= BCM4400_TEMP_PACKET;
		QQ_PushTail(&pDevice->RxPacketFreeQ.Container, pPacket);
		queue_rx = 1;
	}
	if (queue_rx) {
		b44_LM_QueueRxPackets(pDevice);
	}
	return 0;
}

int bcm4400_allocatenpkt(PUM_DEVICE_BLOCK pUmDevice,PUM_PACKET pUmPacket) 
{
	npkt_t *npkt;
	net_buf_t	*nb;
	net_iov_t *iov;
	void *data;
	PLM_PACKET pPacket = (PLM_PACKET)pUmPacket;
	npkt = pUmDevice->ion->alloc_up_npkt(sizeof(*nb) + sizeof(*iov),(void **)&nb);
	if(npkt == NULL) {
		DbgPrint("Can't allocate a new package, errno=%d",errno);
		return -1;
	}
	data = pUmDevice->ion->alloc(pPacket->u.Rx.RxBufferSize,0);
	if(data == NULL) {
		DbgPrint("Can't allocate npkt_t data, errno=%d",errno);
		pUmDevice->ion->free(npkt);
		return -1;
	}
	TAILQ_INSERT_HEAD(&npkt->buffers,nb,ptrs);
	iov = (net_iov_t *)(nb+1);
	nb->niov=1;
	nb->net_iov = iov;
	iov->iov_base = data;
	iov->iov_phys = (paddr_t)pUmDevice->ion->mphys(iov->iov_base);
	iov->iov_len = pPacket->u.Rx.RxBufferSize;
	npkt->org_data = data;
	npkt->next = NULL;
	npkt->tot_iov = 1;
	pUmPacket->npkt = npkt;
	pPacket->u.Rx.pRxBufferVirt = iov->iov_base;
return 0;
}
int bcm4400_freemem(PUM_DEVICE_BLOCK pUmDevice) {
	int i;
	for (i = 0; i < pUmDevice->mem_list_num; i++) {
		if (pUmDevice->mem_size_list[i] == 0) {
			free(pUmDevice->mem_list[i]);
		}
		else {
			pUmDevice->ion->free(pUmDevice->mem_list[i]);
		}
	}
	pUmDevice->mem_list_num = 0;
	return 0;

}
void
bcm4400_free_remaining_rx_bufs(PUM_DEVICE_BLOCK pUmDevice)
{
	LM_DEVICE_BLOCK *pDevice = &pUmDevice->lm_dev;
	UM_PACKET *pUmPacket;
	int cnt, i;
	cnt = QQ_GetEntryCnt(&pUmDevice->rx_out_of_buf_q.Container);
	for (i = 0; i < cnt; i++) {
		if ((pUmPacket = QQ_PopHead(&pUmDevice->rx_out_of_buf_q.Container)) != 0) {
			b44_MM_FreeRxBuffer(pDevice, &pUmPacket->lm_packet);
			QQ_PushTail(&pDevice->RxPacketFreeQ.Container, pUmPacket);
		}
	}
}

void bcm4400_setmcast(PLM_DEVICE_BLOCK pDevice,io_net_msg_join_mcast_t *mc)
{
	switch(mc->type) {
		case _IO_NET_JOIN_MCAST:
			if(mc->flags & _IO_NET_MCAST_ALL)
				b44_LM_SetReceiveMask(pDevice,pDevice->ReceiveMask | LM_ACCEPT_ALL_MULTICAST);
			else {
			b44_LM_MulticastAdd(pDevice,mc->mc_max.addr_dl.sdl_data);
			}
		break;
		case _IO_NET_REMOVE_MCAST:
			if(mc->flags & _IO_NET_MCAST_ALL) 
				b44_LM_MulticastClear(pDevice);
			else {
		b44_LM_MulticastDel(pDevice,mc->mc_max.addr_dl.sdl_data);
			}
		break;
		default:
			DbgPrint("Unsupported mcast mode! %x",mc->type);
	}
return;
}

void bcm4400_powermgmt_init(Nic_t *nic)
{
	PUM_DEVICE_BLOCK	pUmDevice = (PUM_DEVICE_BLOCK)nic->ext;
	pmd_attr_t *dev_pmd = &pUmDevice->pmd;
	static pm_power_mode_t	dev_modes[] = {PM_MODE_OFF,PM_MODE_ACTIVE};

	pmd_attr_init(dev_pmd);
	pmd_attr_setmodes(dev_pmd,PM_MODE_ACTIVE,dev_modes,2);
	pmd_attr_setpower(dev_pmd,bcm4400_setpower,nic);
}

int bcm4400_setpower(void *data,pm_power_mode_t mode,unsigned flags) 
{
	Nic_t *nic = (Nic_t *)data;
	PUM_DEVICE_BLOCK pUmDevice = nic->ext;
	PLM_DEVICE_BLOCK pDevice = (PLM_DEVICE_BLOCK) pUmDevice;

	DbgPrint("Changing powermgmt mode into %d\n",mode);
	pthread_spin_lock(&pUmDevice->spin);
	switch(mode) {
		case PM_MODE_ACTIVE:
			if(pUmDevice->pmd.pm_attr.cur_mode == PM_MODE_ACTIVE) {
				pthread_spin_unlock(&pUmDevice->spin);
				return EOK;
			}
		b44_LM_InitializeAdapter(pDevice);
		b44_LM_EnableInterrupt(pDevice);
		MM_ATOMIC_SET(&pUmDevice->asleep,0);
		break;
		case PM_MODE_OFF:
			if(pUmDevice->pmd.pm_attr.cur_mode == PM_MODE_OFF) {
				pthread_spin_unlock(&pUmDevice->spin);
				return EOK;
			}
			b44_LM_DisableInterrupt(&pUmDevice->lm_dev);
			b44_LM_Halt(pDevice);
			bcm4400_free_remaining_rx_bufs(pUmDevice);
			bcm4400_freemem(pUmDevice);
			b44_LM_PowerDownPhy(pDevice);
			MM_ATOMIC_SET(&pUmDevice->asleep,1);
		break;
		default:
			return EINVAL;
	}

pthread_spin_unlock(&pUmDevice->spin);
return EOK;
}

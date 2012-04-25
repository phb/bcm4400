#include "b44mm.h"


LM_STATUS b44_MM_ReadConfig16(PLM_DEVICE_BLOCK pDevice, LM_UINT32 Offset, LM_UINT16 *pValue16) 
{
	PUM_DEVICE_BLOCK pUmDevice = (PUM_DEVICE_BLOCK )pDevice;
	pci_read_config16(pUmDevice->pciinfo.BusNumber,pUmDevice->pciinfo.DevFunc,Offset,1,pValue16);
	return LM_STATUS_SUCCESS;
}

LM_STATUS b44_MM_WriteConfig16(PLM_DEVICE_BLOCK pDevice, LM_UINT32 Offset, LM_UINT16 Value16) 
{
	PUM_DEVICE_BLOCK pUmDevice = (PUM_DEVICE_BLOCK )pDevice;
	pci_write_config16(pUmDevice->pciinfo.BusNumber,pUmDevice->pciinfo.DevFunc,Offset,1,&Value16);
	return LM_STATUS_SUCCESS;
}

LM_STATUS b44_MM_ReadConfig32(PLM_DEVICE_BLOCK pDevice, LM_UINT32 Offset, LM_UINT32 *pValue32) 
{

	PUM_DEVICE_BLOCK pUmDevice = (PUM_DEVICE_BLOCK )pDevice;
	pci_read_config32(pUmDevice->pciinfo.BusNumber,pUmDevice->pciinfo.DevFunc,Offset,1,pValue32);
	return LM_STATUS_SUCCESS;
}

LM_STATUS b44_MM_WriteConfig32(PLM_DEVICE_BLOCK pDevice, LM_UINT32 Offset, LM_UINT32 Value32)
{
	PUM_DEVICE_BLOCK pUmDevice = (PUM_DEVICE_BLOCK )pDevice;
	pci_write_config32(pUmDevice->pciinfo.BusNumber,pUmDevice->pciinfo.DevFunc,Offset,1,&Value32);
	return LM_STATUS_SUCCESS;
}

LM_STATUS b44_MM_MapMemBase(PLM_DEVICE_BLOCK pDevice)
{
	PUM_DEVICE_BLOCK pUmDevice = (PUM_DEVICE_BLOCK)pDevice;

	pDevice->pMappedMemBase = (void *)mmap_device_memory(0,sizeof(bcmenetregs_t) + 128,PROT_READ|PROT_WRITE|PROT_NOCACHE,0,PCI_MEM_ADDR(pUmDevice->pciinfo.CpuBaseAddress[0]));
	if(pDevice->pMappedMemBase == MAP_FAILED)
	{
		DbgPrint("mmap_device_memory failed: %s\n",strerror(errno));
	}
	pUmDevice->nic->cfg.NumMemWindows=1;
	pUmDevice->nic->cfg.MemBase[0] = (ulong_t)pDevice->pMappedMemBase;
	pUmDevice->nic->cfg.MemLength[0] = sizeof(bcmenetregs_t)+128;
//	pUmDevice->cfg.MemAttrib[0] = ATTR_MEM_SHARED; //??
	return LM_STATUS_SUCCESS;
}

/********/

LM_STATUS b44_MM_IndicateRxPackets(PLM_DEVICE_BLOCK pDevice)
{
	PUM_DEVICE_BLOCK pUmDevice = (PUM_DEVICE_BLOCK)pDevice;
	PLM_PACKET pPacket;
	PUM_PACKET pUmPacket;
	int size;
	npkt_t * npkt;
	net_buf_t *nb;
	net_iov_t *iov;

	while(1) {
		pPacket = (PLM_PACKET)QQ_PopHead(&pDevice->RxPacketReceivedQ.Container);
		if(pPacket == 0) { 
			break;
		}
		pUmPacket = (PUM_PACKET)pPacket;
		if((pPacket->PacketStatus != LM_STATUS_SUCCESS) || ((size = pPacket->PacketSize) > MAX_ETHERNET_PACKET_SIZE_NO_CRC)) { //error.. but reuse the packet.
			DbgPrint("packet is failure :c");
			QQ_PushTail(&pDevice->RxPacketFreeQ.Container,pPacket);
			pUmDevice->nic->nstats.gstats.rcv_error++;
			continue;
		}

		npkt = pUmPacket->npkt;
		nb = (net_buf_t *)TAILQ_FIRST(&npkt->buffers);
		iov = nb->net_iov;

		iov->iov_phys += pDevice->rxoffset;
		iov->iov_base += pDevice->rxoffset;
		//Reset the packet into a working condition
		npkt->next = NULL;
		npkt->num_complete=1;
		npkt->req_complete=0;
		npkt->ref_cnt = 1;
		npkt->flags = _NPKT_UP | _NPKT_NOT_TXED;
		npkt->tot_iov = 1;

		iov->iov_len = size; 
		npkt->framelen = size;
		npkt->cell = pUmDevice->cell;
		npkt->iface = 0;
		npkt->endpoint = pUmDevice->nic->lan; 
	
		pUmPacket->npkt = 0;

		QQ_PushTail(&pUmDevice->rx_out_of_buf_q.Container,pPacket); //once this packet is processed, it'll be put back onto the Receivequeue...

		pUmDevice->bytecount.rx_bytes+=size;
		pUmDevice->nic->nstats.gstats.rcv_ok++;
	
		if(pUmDevice->options.nic.drvr.verbose>4) {
			DbgPrint("Received a package of %d bytes",npkt->framelen);
		}
	if(	pUmDevice->ion->tx_up_start(pUmDevice->reg_hdlp,npkt,0,0,pUmDevice->cell,pUmDevice->nic->lan,0,	pUmDevice->nic) != 0) {
		DbgPrint("GAwww, couldn't send package to io-net!!");
		pUmDevice->ion->tx_done(pUmDevice->reg_hdlp,npkt);	
	} 

	}
return LM_STATUS_SUCCESS;
}

LM_STATUS b44_MM_IndicateTxPackets(PLM_DEVICE_BLOCK pDevice)
{
	PUM_DEVICE_BLOCK pUmDevice = (PUM_DEVICE_BLOCK)pDevice;
	PLM_PACKET pPacket;
	PUM_PACKET pUmPacket;
	npkt_t *npkt;

	while(1){
		pPacket = (PLM_PACKET)QQ_PopHead(&pDevice->TxPacketXmittedQ.Container);
		if(pPacket == 0) 
			break;
		pUmPacket = (PUM_PACKET)pPacket;
		npkt = pUmPacket->npkt;
		if(npkt) 
			pUmDevice->ion->tx_done(pUmDevice->reg_hdlp,npkt);

		pUmPacket->npkt = NULL;
		QQ_PushTail(&pDevice->TxPacketFreeQ.Container,pPacket);
	}
return LM_STATUS_SUCCESS;
}
LM_STATUS b44_MM_AllocateMemory(PLM_DEVICE_BLOCK pDevice, LM_UINT32 BlockSize, PLM_VOID *pMemoryBlockVirt)
{ 
	PUM_DEVICE_BLOCK pUmDevice = (PUM_DEVICE_BLOCK) pDevice;
	PLM_VOID pvirt;

	pvirt = malloc(BlockSize);
	pUmDevice->mem_list[pUmDevice->mem_list_num] = pvirt;
	pUmDevice->dma_list[pUmDevice->mem_list_num] = 0;
	pUmDevice->mem_size_list[pUmDevice->mem_list_num++] = 0;
	/* mem_size_list[i] == 0 indicates that the memory should be freed */
	memset(pvirt,0,BlockSize);
	*pMemoryBlockVirt = pvirt;

return LM_STATUS_SUCCESS;
}
LM_STATUS b44_MM_AllocateSharedMemory(PLM_DEVICE_BLOCK pDevice, LM_UINT32 BlockSize, PLM_VOID *pMemoryBlockVirt, PLM_PHYSICAL_ADDRESS pMemoryBlockPhy)
{
	PLM_VOID pvirt;
	PUM_DEVICE_BLOCK pUmDevice = (PUM_DEVICE_BLOCK)pDevice;

	pvirt = pUmDevice->ion->alloc(BlockSize,0);

	pUmDevice->mem_list[pUmDevice->mem_list_num] = pvirt;
	pUmDevice->dma_list[pUmDevice->mem_list_num] = pUmDevice->ion->mphys(pvirt);
	pUmDevice->mem_size_list[pUmDevice->mem_list_num++] = BlockSize;
	*pMemoryBlockVirt = (PLM_VOID)pvirt;
	*pMemoryBlockPhy = (LM_PHYSICAL_ADDRESS)pUmDevice->ion->mphys(pvirt);

	return LM_STATUS_SUCCESS;
}

LM_STATUS b44_MM_GetConfig(PLM_DEVICE_BLOCK pDevice)
{
	PUM_DEVICE_BLOCK pUmDevice = (PUM_DEVICE_BLOCK)pDevice;

	if(pUmDevice->options.nic.speed != 0)
		pDevice->DisableAutoNeg = TRUE;
	else
		pDevice->DisableAutoNeg = FALSE;

	if(pUmDevice->options.nic.speed == 10)
		pDevice->RequestedLineSpeed = LM_LINE_SPEED_10MBPS;
	else if(pUmDevice->options.nic.speed == 100)
		pDevice->RequestedLineSpeed = LM_LINE_SPEED_100MBPS;
	else
		pDevice->RequestedLineSpeed = LM_LINE_SPEED_AUTO;

	if(pUmDevice->options.nic.fullduplex == 1) 
		pDevice->RequestedDuplexMode = LM_DUPLEX_MODE_HALF;
	else if(pUmDevice->options.nic.fullduplex == 2)
		pDevice->RequestedDuplexMode = LM_DUPLEX_MODE_FULL;
	else 
		pDevice->RequestedDuplexMode = LM_DUPLEX_MODE_UNKNOWN;
	
	pDevice->FlowControlCap = 0;

	pUmDevice->timer_interval = 1000000; //0.1 seconds polling time..
	pUmDevice->link_interval =  10; //every 10th time, update stats etc.

	if(pUmDevice->options.nic.transmit != 0 && pUmDevice->options.nic.transmit < MAX_TX_PACKET_DESC_COUNT) { 
		pDevice->TxPacketDescCnt = pUmDevice->options.nic.transmit;
	} else
			pDevice->TxPacketDescCnt = DEFAULT_TX_PACKET_DESC_COUNT;
	if(pUmDevice->options.nic.receive != 0 && pUmDevice->options.nic.receive < MAX_RX_PACKET_DESC_COUNT) {
		pDevice->RxPacketDescCnt = pUmDevice->options.nic.receive;
	} else
		pDevice->RxPacketDescCnt = DEFAULT_RX_PACKET_DESC_COUNT;

	if(memcmp(pUmDevice->options.nic.mac,"\0\0\0\0\0\0",6)!= 0){
		b44_LM_SetMacAddress(pDevice,pUmDevice->options.nic.mac);
	}
	if(pUmDevice->options.nic.nomulticast != -1)
		b44_LM_SetReceiveMask(pDevice,pDevice->ReceiveMask & ~LM_ACCEPT_MULTICAST);
	else
		b44_LM_SetReceiveMask(pDevice,pDevice->ReceiveMask & LM_ACCEPT_MULTICAST);
	if(pUmDevice->options.nic.promiscuous != -1) 
		b44_LM_SetReceiveMask(pDevice,pDevice->ReceiveMask & LM_PROMISCUOUS_MODE);
	else 
		b44_LM_SetReceiveMask(pDevice,pDevice->ReceiveMask & ~LM_PROMISCUOUS_MODE);

//Add support for flowcontrol
return LM_STATUS_SUCCESS;
}

LM_STATUS b44_MM_IndicateStatus(PLM_DEVICE_BLOCK pDevice, LM_STATUS Status)
{
	PUM_DEVICE_BLOCK pUmDevice = (PUM_DEVICE_BLOCK) pDevice;
	
	if(!pUmDevice->opened) 
		return LM_STATUS_SUCCESS;

	if(pUmDevice->delayed_link_ind > 0) 
		return LM_STATUS_SUCCESS;
	else  {
		if(Status == LM_STATUS_LINK_DOWN) {
			pUmDevice->line_speed = 0;
		}
		else if(Status == LM_STATUS_LINK_ACTIVE) {
		//netif_carrier_on.
		}
	}
	if(Status == LM_STATUS_LINK_ACTIVE) {
		if(pDevice->LineSpeed == LM_LINE_SPEED_100MBPS)
			pUmDevice->line_speed = 100000;
		else if(pDevice->LineSpeed == LM_LINE_SPEED_10MBPS)
			pUmDevice->line_speed = 10000;
	}
	
return LM_STATUS_SUCCESS;
}

LM_STATUS b44_MM_InitializeUmPackets(PLM_DEVICE_BLOCK pDevice){
	int i;
	PUM_DEVICE_BLOCK pUmDevice = (PUM_DEVICE_BLOCK) pDevice;
	PUM_PACKET pUmPacket;
	PLM_PACKET pPacket;

	for (i = 0; i < pDevice->RxPacketDescCnt; i++) {
		pPacket = QQ_PopHead(&pDevice->RxPacketFreeQ.Container);
		pUmPacket = (PUM_PACKET) pPacket;
		if (pPacket == 0) {
			DbgPrint("Bad RxPacketFreeQ\n");
		}
		if(pUmPacket->npkt) {
			ASSERT(0);
		}

		if(bcm4400_allocatenpkt(pUmDevice,pUmPacket) == -1) { 
			QQ_PushTail(&pUmDevice->rx_out_of_buf_q.Container,pPacket);
			continue;
		}
		QQ_PushTail(&pDevice->RxPacketFreeQ.Container, pPacket);
	}
	pUmDevice->rx_buf_repl_thresh = pDevice->RxPacketDescCnt / 4;
	pUmDevice->rx_buf_repl_panic_thresh = pDevice->RxPacketDescCnt * 3 / 4;

return LM_STATUS_SUCCESS;
}

LM_STATUS b44_MM_FreeRxBuffer(PLM_DEVICE_BLOCK pDevice, PLM_PACKET pPacket)
{
	PUM_PACKET pUmPacket;
	PUM_DEVICE_BLOCK pUmDevice = (PUM_DEVICE_BLOCK)pDevice;
	npkt_t *npkt;
	if(pPacket == NULL)
		return LM_STATUS_SUCCESS;

	pUmPacket = (PUM_PACKET)pPacket;
	if((npkt = pUmPacket->npkt))
	pUmDevice->ion->free(npkt->org_data);	
	pUmDevice->ion->free(npkt);
	pUmPacket->npkt = 0;
	return LM_STATUS_SUCCESS;
}

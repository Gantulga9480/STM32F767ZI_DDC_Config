/* Includes ------------------------------------------------------------------*/
#include "udp_server.h"
#include "string.h"

struct udp_pcb my_upcb;

ip_addr_t my_addr, local_addr;
#ifdef __POST_SEND__
char post_data_buffer[POST_DATA_BUFFER_SIZE];
uint16_t post_data_size = 0;
#endif

uint8_t is_con = 0;

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initialize the server application.
  * @param  ip4 IP4 address of client computer
  * @param  sendPort STM32 to PC UDP send port
  * @param  receivPort PC to STM32 UDP receive port
  * @return value None
  */
void USR_UDP_Init(struct IP4_Container ip4, uint16_t sendPort, uint16_t receivePort)
{
	my_addr.addr = (ip4.IP4 << 24) + (ip4.IP3 << 16) + (ip4.IP2 << 8) + ip4.IP1;
	my_upcb.local_port = sendPort;
	my_upcb.remote_port = 0;
	my_upcb.remote_ip = local_addr;
	my_upcb.local_ip = local_addr;
	my_upcb.next = &my_upcb;
	my_upcb.ttl = 255;
	my_upcb.netif_idx = 0;

	err_t err;

	struct udp_pcb *send_upcb, *recv_upcb;

	/* Create a new UDP send control block  */
	send_upcb = udp_new();
	if (send_upcb)
	{
		/* Bind the upcb to the UDP_PORT port */
		/* Using IP_ADDR_ANY allow the upcb to be used by any local interface */
		err = udp_bind(send_upcb, IP_ADDR_ANY, sendPort);
		if(err != ERR_OK)
		{
			udp_remove(send_upcb);
		}
		udp_connect(&my_upcb, &my_addr, sendPort);
	}

	/* Create a new UDP receive control block  */
	recv_upcb = udp_new();
	if (recv_upcb)
	{
		/* Bind the upcb to the UDP_PORT port */
		/* Using IP_ADDR_ANY allow the upcb to be used by any local interface */
		err = udp_bind(recv_upcb, IP_ADDR_ANY, receivePort);

		if(err == ERR_OK)
		{
			/* Set a receive callback for the upcb */
			#ifdef __UDP_TEST__
				udp_recv(recv_upcb, USR_UDP_Server_EchoCallback, NULL);
			#else
				udp_recv(recv_upcb, USR_UDP_Server_ReceiveCallback, NULL);
			#endif
		}
		else
		{
			udp_remove(recv_upcb);
		}
	}
}

USR_StatusTypeDef USR_UDP_Send(uint16_t udp_send_port, uint8_t *buff, uint16_t len)
{
	struct pbuf *p_tx;

	p_tx = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);

	if(p_tx != NULL)
	{
		pbuf_take(p_tx, buff, len);

		udp_connect(&my_upcb, &my_addr, udp_send_port);

		udp_send(&my_upcb, p_tx);

		udp_disconnect(&my_upcb);

		pbuf_free(p_tx);

		return USR_OK;
	}
	return USR_ERR;
}

/**
  * @brief This function is called when an UDP datagrm has been received on the port UDP_PORT.
  * @param arg user supplied argument (udp_pcb.recv_arg)
  * @param pcb the udp_pcb which received data
  * @param p the packet buffer that was received
  * @param addr the remote IP address from which the packet was received
  * @param port the remote port from which the packet was received
  * @retval None
  */
#ifdef __UDP_TEST__
void USR_UDP_Server_EchoCallback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
	struct pbuf *p_tx;

	/* allocate pbuf from RAM*/
	p_tx = pbuf_alloc(PBUF_TRANSPORT,p->len, PBUF_RAM);

	if(p_tx != NULL)
	{
		pbuf_take(p_tx, (char*)p->payload, p->len);

		/* Connect to the remote client */
		udp_connect(upcb, addr, port);

		/* Tell the client that we have accepted it */
		udp_send(upcb, p_tx);

		/* free the UDP connection, so we can accept new clients */
		udp_disconnect(upcb);

		/* Free the p_tx buffer */
		pbuf_free(p_tx);

		/* Free the p buffer */
		pbuf_free(p);
	}
}
#else
void USR_UDP_Server_ReceiveCallback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
	/* Handle received packet */
	USR_UDP_ReceiveCallback(p, addr->addr, port);

	/* Send response */
#ifdef __POST_SEND__
	if (post_data_size != 0)
	{
		struct pbuf *p_tx;

		/* allocate pbuf from RAM */
		p_tx = pbuf_alloc(PBUF_TRANSPORT, post_data_size, PBUF_RAM);

		if(p_tx != NULL)
		{
			pbuf_take(p_tx, (uint8_t *)post_data_buffer, post_data_size);

			/* Connect to the remote client */
			udp_connect(upcb, addr, port);

			/* Tell the client that we have accepted it */
			udp_send(upcb, p_tx);

			/* free the UDP connection, so we can accept new clients */
			udp_disconnect(upcb);

			/* Free the p_tx buffer */
			pbuf_free(p_tx);

			/* Free post data buffer */
			strcpy(post_data_buffer, "");
			post_data_size = 0;
		}
	}
#endif

	/* Free the p buffer */
	pbuf_free(p);
}
#endif

#ifdef __POST_SEND__
USR_StatusTypeDef USR_UDP_InsertPostDataCh(char data, uint16_t index)
{
	if (index >= POST_DATA_BUFFER_SIZE)
		return USR_ERR;
	post_data_buffer[index] = data;
	return USR_OK;
}

USR_StatusTypeDef USR_UDP_InsertPostData(char *data, uint16_t len)
{
	if ((len >= POST_DATA_BUFFER_SIZE) || ((post_data_size + len) >= POST_DATA_BUFFER_SIZE))
		return USR_ERR;
	strcpy(post_data_buffer, data);
	post_data_size += len;
	return USR_OK;
}
#endif

__weak void USR_UDP_ReceiveCallback(struct pbuf *p, const uint32_t addr, const uint16_t port)
{
	__NOP();
}

struct IP4_Container toIP4(uint32_t ip_value)
{
	struct IP4_Container ip;
	ip.IP4 = (ip_value & 0xFF000000) >> 24;
	ip.IP3 = (ip_value & 0x00FF0000) >> 16;
	ip.IP2 = (ip_value & 0x0000FF00) >> 8;
	ip.IP1 = (ip_value & 0x000000FF);
	return ip;
}

#ifndef __ECHO_H__
#define __ECHO_H__

#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "usr.h"

struct IP4_Container {
	uint8_t IP1;
	uint8_t IP2;
	uint8_t IP3;
	uint8_t IP4;
};

/* Echo callback */
// #define __UDP_TEST__

/* Response callback */
#define __POST_SEND__

#ifdef __POST_SEND__
#define POST_DATA_BUFFER_SIZE 128 // bytes
#endif

void USR_UDP_Init(struct IP4_Container ip4, uint16_t sendPort, uint16_t receivePort);

USR_StatusTypeDef USR_UDP_Send(uint16_t udp_send_port, uint8_t *buff, uint16_t len);

void USR_UDP_ReceiveCallback(struct pbuf *p, const uint32_t addr, const uint16_t port);

#ifdef __POST_SEND__
USR_StatusTypeDef USR_UDP_InsertPostDataCh(char data, uint16_t index);
USR_StatusTypeDef USR_UDP_InsertPostData(char *data, uint16_t len);
#endif

struct IP4_Container toIP4(uint32_t ip);

#ifdef __UDP_TEST__
  void USR_UDP_Server_EchoCallback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);
#else
  void USR_UDP_Server_ReceiveCallback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);
#endif

#endif /* __MINIMAL_ECHO_H */

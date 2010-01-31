
#include "socketapp.h"
#include "uip.h"
#include <string.h>

static int handle_connection(struct socket_app_state *s);

void socket_app_init(void)
{
  uip_listen(HTONS(2451));
}

void socket_app_appcall(void)
{
  struct socket_app_state *s = &(uip_conn->appstate);
  if(uip_connected()) 
  {
    PSOCK_INIT(&s->p, s->inputbuffer, sizeof(s->inputbuffer));
  }

  handle_connection(s);
}

extern char packetData[20];
extern int dataWaiting;

static int handle_connection(struct socket_app_state *s)
{
  PSOCK_BEGIN(&s->p);
  
  PSOCK_READTO(&s->p, '\n');
  
  memcpy(packetData, s->inputbuffer, sizeof(packetData)); 
  dataWaiting = 1;
  
  memset(s->inputbuffer, 0x00, sizeof(s->inputbuffer));
  
  PSOCK_END(&s->p);
}
/*---------------------------------------------------------------------------*/

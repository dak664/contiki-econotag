/*
 * Copyright (c) 2002, Adam Dunkels.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * This file is part of the Contiki OS
 *
 * $Id: contiki-main.c,v 1.14 2011/01/21 14:19:57 nvt-se Exp $
 *
 */

#include <stdio.h>
#include <unistd.h>
#include <sys/select.h>
#include <string.h>

#include "contiki.h"
#include "contiki-net.h"
#include "net/netstack.h"

#include "dev/serial-line.h"

#include "net/uip.h"
#ifdef __CYGWIN__
#include "net/wpcap-drv.h"
#else /* __CYGWIN__ */
#include "net/tapdev-drv.h"
#endif /* __CYGWIN__ */


#include "dev/button-sensor.h"
#include "dev/pir-sensor.h"
#include "dev/vib-sensor.h"

PROCINIT(&etimer_process);

SENSORS(&pir_sensor, &vib_sensor, &button_sensor);

/*---------------------------------------------------------------------------*/
int
main(void)
{
  rimeaddr_t addr;

  printf("Starting Contiki\n");
  process_init();
  ctimer_init();

  netstack_init();
  
  procinit_init();

  serial_line_init();

#ifdef __CYGWIN__
  addr.u8[0] = 0x00;
  addr.u8[1] = 0x50;
  addr.u8[2] = 0xc2;
  addr.u8[3] = 0xff;
  addr.u8[4] = 0xfe;
  addr.u8[5] = 0xa8;
  addr.u8[6] = 0xcd;
  addr.u8[7] = 0x1a;
#else
//00:50:C2:A8:C6:C8:3E:99
  addr.u8[0] = 0;
  addr.u8[1] = 0x50;
  addr.u8[2] = 0xc2;
  addr.u8[3] = 0xa8;
  addr.u8[4] = 0xc6;
  addr.u8[5] = 0xc8;
  addr.u8[6] = 0x3e;
  addr.u8[7] = 0x99;
#endif
  rimeaddr_set_node_addr(&addr);
  memcpy(&uip_lladdr.addr, &addr.u8, sizeof(uip_lladdr.addr));

  process_start(&tcpip_process, NULL);

#ifdef __CYGWIN__
  process_start(&wpcap_process, NULL);
#endif
  
  autostart_start(autostart_processes);

  
  /* Make standard output unbuffered. */
  setvbuf(stdout, (char *)NULL, _IONBF, 0);
  
  while(1) {
    fd_set fds;
    int n;
    struct timeval tv;
    
    n = process_run();
#if 0
    tv.tv_sec = 0;
    tv.tv_usec = 1;

    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    if(select(1, &fds, NULL, NULL, &tv) < 0) {
      perror("select");
    } else if(FD_ISSET(STDIN_FILENO, &fds)) {
      char c;
      if(read(STDIN_FILENO, &c, 1) > 0) {
	serial_line_input_byte(c);
      }
    }
#endif
    
void slip_arch_read(void);
    slip_arch_read();
#ifdef __CYGWIN__
#else
    tun_arch_read();
#endif

    etimer_request_poll();
  }
  
  return 0;
}
/*---------------------------------------------------------------------------*/
void
log_message(char *m1, char *m2)
{
  printf("%s%s\n", m1, m2);
}
/*---------------------------------------------------------------------------*/
void
uip_log(char *m)
{
  printf("%s\n", m);
}
/*---------------------------------------------------------------------------*/

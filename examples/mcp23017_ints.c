#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <errno.h>

#include <wiringPi.h>
#include <mcp23017.h>

#define	AF_BASE		300
#define	MCP23X17_INTA	0

void mcp23x17_interrupt(void) {
  int values, pins = AF_BASE;

  interruptRead (&pins, &values);
  printf("mcp23x17_interrupt() pins=%x values=%x\n", pins, values);
}

int main (int argc, char *argv[])
{
  int i;
  char cmd[64];
  wiringPiSetup() ;
  mcp23017Setup (AF_BASE, 0x20) ;

  // 1 - OR'd AB ints, 0 - disable opendrain, LOW - active_low ints
  if (mcp23017SetupInts (AF_BASE, 1, 1, LOW) < 0) {
    printf ("Unable to setup mcp23017SetupInts: %s\n", strerror (errno));
    return -1;
  }

  for(i=0; i<16; i++) {
    pinMode (AF_BASE+i, INPUT) ;
    pullUpDnControl (AF_BASE+i, PUD_UP);
    pinIntPolarity (AF_BASE+i, INT_EDGE_BOTH) ;
  }

  pinMode(MCP23X17_INTA, INPUT);
  pullUpDnControl(MCP23X17_INTA, PUD_UP);

  if (wiringPiISR (MCP23X17_INTA, INT_EDGE_FALLING, &mcp23x17_interrupt) < 0 ) {
    printf ("Unable to setup MCP ISR: %s\n", strerror (errno));
    return -1;
  }

  mcp23x17_interrupt();

  while (1) {
#if 1
    if (++i > 7) i = 0;
    printf("Trying i=%x\n", ~(1<<i)&0xff);
    sprintf(cmd, "/usr/sbin/i2cset -y 1 0x20 0xc 0x%x", ~(1<<i)&0xff);
    system(cmd);
    sleep(16);
    printf("NOW i=%x\n", 0xff);
    sprintf(cmd, "/usr/sbin/i2cset -y 1 0x20 0xc 0x%x", 0xff);
    system(cmd);
#endif
    sleep(2);
  }

  return 0 ;
}

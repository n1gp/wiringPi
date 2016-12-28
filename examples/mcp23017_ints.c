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
#define	AF_BUTTON1	(AF_BASE + 0)
#define	AF_BUTTON2	(AF_BASE + 2)
#define	AF_BUTTON3	(AF_BASE + 10)
#define	MCP23X17_INTA	0

void mcp23x17_interrupt(void) {
  int prev, values, pins = AF_BASE;

  interruptRead (&pins, &values);
  prev = digitalRead(MCP23X17_INTA);
  printf("mcp23x17_interrupt() pins=%x values=%x prev=%x\n", pins, values, prev);
}

int main (int argc, char *argv[])
{
  int i;
  wiringPiSetup() ;
  mcp23017Setup (AF_BASE, 0x20) ;

#if 1
  // 1 - OR'd AB ints, 0 - disable opendrain, LOW - active_low ints
  if (mcp23017SetupInts (AF_BASE, 1, 0, LOW) < 0) {
    printf ("Unable to setup mcp23017SetupInts: %s\n", strerror (errno));
    return -1;
  }
#endif

  pinMode (AF_BUTTON1, INPUT) ;
  pinMode (AF_BUTTON2, INPUT) ;
  pinMode (AF_BUTTON3, INPUT) ;
  pullUpDnControl (AF_BUTTON1, PUD_UP);
  pullUpDnControl (AF_BUTTON2, PUD_UP);
  pullUpDnControl (AF_BUTTON3, PUD_UP);

#if 1
  pinMode(MCP23X17_INTA, INPUT);
  pullUpDnControl(MCP23X17_INTA, PUD_UP);

  if (wiringPiISR (MCP23X17_INTA, INT_EDGE_FALLING, &mcp23x17_interrupt) < 0 ) {
    printf ("Unable to setup MCP ISR: %s\n", strerror (errno));
    return -1;
  }

  pinIntPolarity (AF_BUTTON1, INT_EDGE_BOTH) ;
  pinIntPolarity (AF_BUTTON2, INT_EDGE_BOTH) ;
  pinIntPolarity (AF_BUTTON3, INT_EDGE_BOTH) ;
#endif

  mcp23x17_interrupt();

  while (1) {
#if 0
    // digitalRead clears INTF registers!
    if (digitalRead (AF_BUTTON) == LOW)	// Pushed
    {
      printf("RRK, LOW\n");
    }
    else
    {
      printf("RRK, HIGH\n");
    }
#endif
    sleep(1);
  }

  return 0 ;
}

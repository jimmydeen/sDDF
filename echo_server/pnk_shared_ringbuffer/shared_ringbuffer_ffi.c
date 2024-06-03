#include <stdio.h>
#include <stdlib.h>
#include <microkit.h>
#include "fence.h"

/*
 *
 * FFIs needed for shared_ringbuffer implementation
 *
*/

/* Simple wrapper for thread memory fencing */

void ffiTHREAD_MEMORY_RELEASE(unsigned char *c, long clen, unsigned char *a, long alen) {

  THREAD_MEMORY_RELEASE();
  
}

/* FFI to call functions from functions pointers in Pancake 
 * @param c - the function to call
 */

void ffifun_call(unsigned char *c, long clen, unsigned char *a, long alen) {

  if (clen != 1) {

    microkit_dbg_puts("ld_hw: There are no arguments supplied when args are expected")
    c[0] = 0;
    return;
   
  }
  
  void (*func)(void) = (void (*)(void))c;
  func();

}


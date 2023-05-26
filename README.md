# sDDF
seL4 Device Driver Framework

The sDDF aims to provide interfaces and protocols for writing and
porting device drivers to run as seL4 user level programs. It
currently supports a network device running on iMX8 hardware, reaching
near wire speed.  It has been built on top of [seL4 Core
Platform](https://github.com/BreakawayConsulting/sel4cp) and requires
[this pull
request](https://github.com/BreakawayConsulting/sel4cp/pull/11). The
seL4 Core Platform binaries can be built separately and handed to the
echo_server makefile.

## Building the sDDF

The Pancake compiler is included within this repo. Extract the supplied file, and run ```make cake``` within the unzipped repository 
to build the bootstrapped comipler.

Note that while any ARM GCC toolchain should work, all testing and
benchmarking so far has been done with the ARM GCC toolchain version 10.2-2020.11.

If you wish to use the default toolchain you can download it from here:
https://developer.arm.com/-/media/Files/downloads/gnu-a/10.2-2020.11/binrel/gcc-arm-10.2-2020.11-x86_64-aarch64-none-elf.tar.xz?revision=79f65c42-1a1b-43f2-acb7-a795c8427085&hash=61BBFB526E785D234C5D8718D9BA8E61.

Otherwise, you can change the Makefile to accept another toolchain or pass the prefix
to the Makefile using the argument `TOOLCHAIN=<PREFIX>`.

```
    $ cd echo_server
    $ make BUILD_DIR=<path/to/build> \
        SEL4CP_SDK=<path/to/core/platform/sdk> \
        SEL4CP_BOARD=imx8mm_evk \
	CAKE_COMPILER=<path/to/cake/compiler> \
	SEL4CP_CONFIG=(benchmark/release/debug)
```

## Benchmarking

In order to run the benchmarks, set `SEL4CP_CONFIG=benchmark`. The
system has been designed to interact with
[ipbench](https://sourceforge.net/projects/ipbench/) to take
measurements.

Checks to make before benchmarking:
* Turn off all debug prints.
* Turn off all sDDF related asserts (pass `NO_ASSERT` in Makefile).
* Run with LWIP asserts turned off as well (`LWIP_NOASSERT`).
* Make sure compiler optimisations are enabled.

[Benchmarks for Pancake mux.](https://bit.ly/3oHrgZ4)

## Building with Pancake

In the build directory there will be two .S files outputted, mux_tx.S and mux_rx.S. Add the following changes into both .S files (overwrite the existing sections), and run the make command again. This is to make Pancake return to where ```cml_main()``` was called, without it we will never return back to the core platform's handler loop.
```
cdecl(cml_main):
     _ldrel x0, cake_main            /* arg1: entry address */
     _ldrel x1, cdecl(cml_heap)      /* arg2: first address of heap */
     ldr    x1,[x1]
     _ldrel x2, cake_bitmaps
     str    x2,[x1]                  /* store bitmap pointer */
     _ldrel x2, cdecl(cml_stack)     /* arg3: first address of stack */
     ldr    x2,[x2]
     _ldrel x3, cdecl(cml_stackend)  /* arg4: first address past the stack */
     ldr    x3,[x3]
	 str x30, [sp, #-32]!
     b      cake_main
     .ltorg
```

And, 

```
cake_exit:
     ldr x30, [sp], #32
	 ret
     .p2align 4

```

## Supported Boards

### iMX8MM-EVK


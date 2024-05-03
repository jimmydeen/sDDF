# Brief Intro
This is a learning exercise to get acquainted with the sDDF. A basic serial driver has been implemented in pancake, with printf, getchar and scanf.

## Building the sDDF

    $ cd echo_server
    $ make BUILD_DIR=<path/to/build> microkit_SDK=<path/to/core/platform/sdk> CAKE_COMPILER=<path/to/cake/compiler/binary> microkit_BOARD=imx8mm microkit_CONFIG=(release/debug)

## Notes on building Pancake
We will need to modify an output file, and recompile. This is because of the auto-genrated assembly code outputted by the cake compiler wishing to call
to a `cml_exit` function rather than return from where it was called, which is the behaviour we want.

In `serial.S`, please replace `cdecl (cml_main)` with:

```
cdecl ( cml_main ) :
    _ldrel x0 , cake_main /* arg1 : entry
    address */
    _ldrel x1 , cdecl ( cml_heap ) /* arg2 : first
    address of heap */
    ldr x1 ,[ x1 ]
    _ldrel x2 , cake_bitmaps
    str x2 ,[ x1 ] /* store bitmap
    pointer */
    _ldrel x2 , cdecl ( cml_stack ) /* arg3 : first
    address of stack */
    ldr x2 ,[ x2 ]
    _ldrel x3 , cdecl ( cml_stackend ) /* arg4 : first
    address past the stack */
    ldr x3 ,[ x3 ]
    str x30 , [ sp , # -32]!
    b cake_main
```

And please replace `cake_exit` with:

```
cake_exit :
    ldr x30 , [ sp ] , #32
    ret
    .p2align 4
```
## Supported Boards

### iMX8MM-EVK


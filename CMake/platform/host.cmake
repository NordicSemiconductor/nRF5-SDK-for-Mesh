set(host_ARCH x86)
set(host_DEFINES
    -DHOST
    -DSVCALL_AS_NORMAL_FUNCTION)

set(host_INCLUDE_DIRS
    "${SDK_ROOT}/modules/nrfx/"
    "${SDK_ROOT}/modules/nrfx/mdk"
    "${SDK_ROOT}/components/toolchain/cmsis/include")

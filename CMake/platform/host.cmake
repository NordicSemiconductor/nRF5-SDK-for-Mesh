set(host_ARCH x86)
set(host_DEFINES
    -DHOST
    -DSVCALL_AS_NORMAL_FUNCTION)

set(host_INCLUDE_DIRS
    "${SDK_ROOT}/components/device"
    "${SDK_ROOT}/components/toolchain"
    "${SDK_ROOT}/components/toolchain/cmsis/include")

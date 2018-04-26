set(pca10031_DEFINES
    -DBOARD_PCA10031
    -DBSP_LED_3=BSP_LED_0  # Pretend to have four LEDs
    -DCONFIG_GPIO_AS_PINRESET)
set(pca10031_INCLUDE_DIRS
    "${SDK_ROOT}/components/boards")

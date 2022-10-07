make distclean
make imx_alientek_emmc_defconfig
bear make all -j4
cp arch/arm/boot/zImage ~/linux/tftpboot/ -f
cp arch/arm/boot/dts/imx6ull-alientek-emmc.dtb ~/linux/tftpboot/ -f

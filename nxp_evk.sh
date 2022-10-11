make distclean
make imx_v7_mfg_defconfig
bear make all -j4
cp arch/arm/boot/zImage ~/linux/tftpboot/ -f
cp arch/arm/boot/dts/imx6ull-14x14-evk.dtb ~/linux/tftpboot/ -f

#!/bin/bash

CPUs=`sed -n "N;/processor/p" /proc/cpuinfo|wc -l`
DEFCONFIG=
#########################################################################################################################################
#        model           flag    arch  chip      uart       dtb                           multi  image           defconfig		
#########################################################################################################################################
ID_MODEL=1
ID_FLAG=2
ID_ARCH=3
ID_CHIP=4
ID_UART=5
ID_DTB=6
ID_MULTI=7
ID_IMAGE=8
ID_CONFIG=9
model_arm64=(
	"TB-RK3568C1-C   TC031C arm64 RK3568    0xff660000 rk3568-toybrick-core-linux       1     Image.rockchip  rockchip_linux_defconfig"
	"TB-RK3568C0-C   TC030C arm64 RK3568    0xff660000 rk3568-toybrick-core-linux       1     Image.rockchip  rockchip_linux_defconfig"
	"TB-RK3568C0-D   TC030D arm64 RK3568    0xff660000 rk3568-toybrick-core-linux       1     Image.rockchip  rockchip_linux_defconfig"
	"TB-RK3568X0     TX0356 arm64 RK3568    0xff660000 rk3568-toybrick-core-linux-x0    0     Image.rockchip  rockchip_linux_defconfig"
	"TB-RK3568Xs0    TXs356 arm64 RK3568    0xff660000 rk3568-toybrick-core-linux       1     Image.rockchip  rockchip_linux_defconfig"
	"TB-RK3568X1-C   TX031C arm64 RK3568    0xff660000 rk3568-toybrick-core-linux-x0    0     Image.rockchip  rockchip_linux_defconfig"
	"TB-RK3568X0-C   TX030C arm64 RK3568    0xff660000 rk3568-toybrick-core-linux-x0    0     Image.rockchip  rockchip_linux_defconfig"
	"TB-RK3568X0-D   TX030D arm64 RK3568    0xff660000 rk3568-toybrick-core-linux-x0    0     Image.rockchip  rockchip_linux_defconfig"
	"TB-RK3399ProD   TD0331 arm64 RK3399Pro 0xff1a0000 rk3399pro-toybrick-prod-linux    0     Image.rockchip  rockchip_linux_defconfig"
	"TB-RK3399ProDs  TDs331 arm64 RK3399pro 0xff1a0000 rk3399pro-toybrick-prod-linux    0     Image.rockchip  rockchip_linux_defconfig"
	"TB-RK3399ProP   TP0331 arm64 RK3399Pro 0xff1a0000 rk3399pro-toybrick-prop-linux    0     Image.rockchip  rockchip_linux_defconfig"
	"TB-RK3399ProPs  TPs331 arm64 RK3399Pro 0xff1a0000 rk3399pro-toybrick-prop-linux    0     Image.rockchip  rockchip_linux_defconfig"
	"TB-RK3399ProX0  TX0331 arm64 RK3399Pro 0xff1a0000 rk3399pro-toybrick-prox-linux-x0 0     Image.rockchip  rockchip_linux_defconfig"
	"TB-RK3399ProXs0 TXs331 arm64 RK3399Pro 0xff1a0000 rk3399pro-toybrick-prox-linux-x0 0     Image.rockchip  rockchip_linux_defconfig"
	"TB-RK3399ProX1  TX0331 arm64 RK3399Pro 0xff1a0000 rk3399pro-toybrick-prox-linux-x1 0     Image.rockchip  rockchip_linux_defconfig"
	"TB-RK3399ProXs1 TXs331 arm64 RK3399Pro 0xff1a0000 rk3399pro-toybrick-prox-linux-x1 0     Image.rockchip  rockchip_linux_defconfig"
	#"TB-RK1808M0     TM0180 arm64 RK1808    0xff550000 rk1808-toybrick-m0               0     Image.RK1808    RK1808_linux_defconfig"
	)

model_arm=(
	"TB-RV1126D     TR0112 arm   rv1126    0xff570000  rv1126-toybrick-linux            0     zImage.rv1126   rv1126_defconfig"
	"TB-RV1126Ds    TRs112 arm   rv1126    0xff570000  rv1126-toybrick-linux            0     zImage.rv1126   rv1126_defconfig"
	"TB-ToyT01      TRt112 arm   rv1126    0xff570000  rv1126-toybrick-ToyT01           0     zImage.rv1126   rv1126_defconfig"
	)

model_ubifs=(
	"TB-RV1126P0-A  TP020A arm   rv1126    0xff570000  rv1126-toybrick-mi               0     zImage.rv1126   rv1126_defconfig"
	)

###########################################################################################################################################

UBIFS_KERNEL_SIZE=8
EXT4_KERNEL_SIZE=64
BLOCKS=4096
MAX_BOARDID=32

function help()
{
	echo "Usage: ./make-linux.sh {arm|arm64|ubifs}"
	echo "e.g."
	echo "  ./make-linux.sh arm"
	echo "  ./make-linux.sh arm64"
	echo
	echo "Usage: ./make-linux.sh {MODEL}"
	echo "e.g."
	for i in "${model_arm64[@]}"; do
		echo "  ./make-linux.sh $(echo $i | awk '{print $1}')"
	done
	for i in "${model_arm[@]}"; do
		echo "  ./make-linux.sh $(echo $i | awk '{print $1}')"
	done
	for i in "${model_ubifs[@]}"; do
		echo "  ./make-linux.sh $(echo $i | awk '{print $1}')"
	done
}

# Make all toybrick dts:
function make_toybrick_dtb()
{
	arch=$1
	if [ ${arch} == "arm" ]; then
		dts_path=arch/arm/boot/dts
	else
		dts_path=arch/arm64/boot/dts/rockchip
	fi
	rm -rf ${dts_path}/*.dtb
	dts_list=`ls ${dts_path}/*-toybrick*.dts | awk -F'.' '{print $1}'`
	for d in ${dts_list}; do
		make -f ./scripts/Makefile.build obj=${dts_path} ${d}.dtb srctree=./ objtree=./ > /dev/null
	done
	cp ${dts_path}/*toybrick*.dtb boot_linux/extlinux/
}

function make_toybrick_logo()
{
    cp -f logo*.bmp boot_linux/extlinux/
}

function make_extlinux_conf_one()
{
	flag=$1
	index=$2
	dtb_name=$3
	image=$4
	fs=$5
	uart=$6
	
	file_path=boot_linux/extlinux

	if [ ${flag} == "none" ]; then
		src_dtb_file=${dtb_name}.dtb
		src_dtb_file_factory=${dtb_name}-factory.dtb
		dst_dtb_file=toybrick.dtb
		conf_file=${file_path}/extlinux.conf
	elif [ ${index} -eq -1 ]; then
		src_dtb_file=${dtb_name}.dtb
		src_dtb_file_factory=${dtb_name}-factory.dtb
		dst_dtb_file=toybrick.dtb.${flag}
		conf_file=${file_path}/extlinux.conf.${flag}
	else
		src_dtb_file=${dtb_name}-x${index}.dtb
		src_dtb_file_factory=${dtb_name}-x${index}-factory.dtb
		dst_dtb_file=toybrick.dtb.${flag}.${index}
		conf_file=${file_path}/extlinux.conf.${flag}.${index}
	fi

	cp ${file_path}/${src_dtb_file} ${file_path}/${dst_dtb_file}
	if [ -f ${file_path}/${src_dtb_file_factory} ]; then
		cp ${file_path}/${src_dtb_file_factory} ${file_path}/${dst_dtb_file}.factory
	fi

	echo "label rockchip-kernel-4.19" > ${conf_file}
	echo "	kernel /extlinux/${image}" >> ${conf_file}
	echo "	fdt /extlinux/${dst_dtb_file}" >> ${conf_file}
	case ${fs} in
	ext4)
		echo "	append earlycon=uart8250,mmio32,${uart} root=PARTUUID=614e0000-0000-4b53-8000-1d28000054a9 rw rootwait rootfstype=ext4" >> ${conf_file}
		;;
	ubifs)
		echo "	append earlycon=uart8250,mmio32,${uart} printk.time=1 ubi.mtd=3 root=ubi0.rootfs rw rootwait rootfstype=ubifs initcall_debug=1" >> ${conf_file}
		;;
	*)
		echo Unsupproted filesystem ...
		exit 1
		;;
	esac
}

function make_extlinux_conf_legacy()
{
	ID_FS=$#
	fs=${!ID_FS}
	dtb_name=${!ID_DTB}
	image=${!ID_IMAGE}
	uart=${!ID_UART}
	make_extlinux_conf_one none -1 ${dtb_name} ${image} ${fs} ${uart}
}

function make_extlinux_conf()
{
	ID_FS=$#
	fs=${!ID_FS}
	dtb_name=${!ID_DTB}
	image=${!ID_IMAGE}
	multi=${!ID_MULTI}
	flag=${!ID_FLAG}
	uart=${!ID_UART}

	make_extlinux_conf_one ${flag} -1 ${dtb_name} ${image} ${fs} ${uart}
	if [ ${multi} -eq 1 ]; then
		for i in $(seq 0 ${MAX_BOARDID})
		do
			if [ -f boot_linux/extlinux/${dtb_name}-x$i.dtb ]; then
				make_extlinux_conf_one ${flag} $i ${dtb_name} ${image} ${fs} ${uart}
			fi
		done
	fi
}

function make_kernel_image()
{
	config=${!ID_CONFIG}
	dtb_name=${!ID_DTB}
	arch=${!ID_ARCH}
	image=${!ID_IMAGE}

	if [ "${config}" != "${DEFCONFIG}" ]; then
		make ARCH=${arch} ${config}
		make ARCH=${arch} ${dtb_name}.img -j${CPUs}
		cp arch/${arch}/boot/`echo ${image} | awk -F'.' '{print $1}'` boot_linux/extlinux/${image} 
		DEFCONFIG=${config}
	fi
	make_toybrick_dtb ${arch}
    make_toybrick_logo
}

function make_boot_linux()
{
	fs=$1
	if [ "${fs}" == "ubifs" ]; then
		size_m=${UBIFS_KERNEL_SIZE}
	else
		size_m=${EXT4_KERNEL_SIZE}
	fi
	blocks=${BLOCKS}
	block_size=$((${size_m} * 1024 * 1024 / ${blocks}))

	if [ "`uname -i`" == "aarch64" ]; then
		echo y | mke2fs -b ${block_size} -d boot_linux -i 8192 -t ext2 boot_linux.img ${blocks}
	else
		genext2fs -B ${blocks} -b ${block_size} -d boot_linux -i 8192 -U boot_linux.img
	fi
}

rm -rf boot_linux
mkdir -p boot_linux/extlinux
touch boot_linux/extlinux/extlinux.conf
case $1 in
	arm)
		for i in "${model_arm[@]}"; do
			make_kernel_image $i
			make_extlinux_conf $i ext4
		done
		make_boot_linux ext4
		;;
	arm64)
		for i in "${model_arm64[@]}"; do
			make_kernel_image $i
			make_extlinux_conf $i ext4
		done
		make_boot_linux ext4
		;;
	ubifs)
		for i in "${model_ubifs[@]}"; do
			make_kernel_image $i
			make_extlinux_conf $i ubifs
		done
		make_boot_linux ubifs
		;;
	*)
		if [ $# -eq 1 ]; then
			found=0
			for i in "${model_arm64[@]}"; do
				if [ "$(echo $i | awk '{print $1}')" == "$1" ]; then
					make_kernel_image $i
					make_extlinux_conf_legacy $i ext4
					make_boot_linux ext4
					found=1
				fi
			done
			for i in "${model_arm[@]}"; do
				if [ "$(echo $i | awk '{print $1}')" == "$1" ]; then
					make_kernel_image $i
					make_extlinux_conf_legacy $i ext4
					make_boot_linux ext4
					found=1
				fi
			done
			for i in "${model_ubifs[@]}"; do
				if [ "$(echo $i | awk '{print $1}')" == "$1" ]; then
					make_kernel_image $i
					make_extlinux_conf_legacy $i ubifs
					make_boot_linux ubifs
					found=1
				fi
			done
			if [ ${found} -eq 0 ]; then
				help
				exit 1
			fi
		else
			help
			exit 1
		fi
esac

rm -rf boot_linux/extlinux/*toybrick-*.dtb

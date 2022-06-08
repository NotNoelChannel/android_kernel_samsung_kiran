#!/bin/bash
#
JOBS=`grep -c processor /proc/cpuinfo`
let JOBS=${JOBS}*2
JOBS="-j${JOBS}"
RELEASE_DATE=`date +%Y%m%d`
COMMIT_ID=`git log --pretty=format:'%h' -n 1`
BOOT_PATH="arch/arm/boot"
DZIMAGE="dzImage"

if [ "${1}" = "" ]; then
	echo "Warnning: failed to get machine id."
	echo "ex)./release.sh	Product_name		Model"
	echo "ex)------------	---------------		------"
	echo "ex)./release.sh	tizen_young23gdtv	<Young2 3G DTV>"
	echo "ex)./release.sh	tizen_kiran		<Kiran>"
	exit
fi
make ARCH=arm ${1}_defconfig
if [ "$?" != "0" ]; then
	echo "Failed to make defconfig :"$ARCH
	exit 1
fi

make $JOBS zImage ARCH=arm
if [ "$?" != "0" ]; then
        echo "Failed to make zImage"
        exit 1
fi

DTC_PATH="scripts/dtc/"

rm $BOOT_PATH/dts/*.dtb -f

make ARCH=arm dtbs
if [ "$?" != "0" ]; then
        echo "Failed to make dtbs"
        exit 1
fi

dtbtool -o $BOOT_PATH/merged-dtb -p $DTC_PATH -v $BOOT_PATH/dts/
if [ "$?" != "0" ]; then
	echo "Failed to make merged-dtb"
	exit 1
fi

mkdzimage -o $BOOT_PATH/$DZIMAGE -k $BOOT_PATH/zImage -d $BOOT_PATH/merged-dtb
if [ "$?" != "0" ]; then
	echo "Failed to make mkdzImage"
	exit 1
fi

if [ "${2}" != "" ]; then
	RELEASE_IMAGE=System_${1}_${RELEASE_DATE}_${2}.tar
else
	RELEASE_IMAGE=System_${1}_${RELEASE_DATE}-${COMMIT_ID}.tar
fi

tar cf $RELEASE_IMAGE -C $BOOT_PATH $DZIMAGE
if [ "$?" != "0" ]; then
	echo "Failed to tar $DZIMAGE"
	exit 1
fi

echo $RELEASE_IMAGE

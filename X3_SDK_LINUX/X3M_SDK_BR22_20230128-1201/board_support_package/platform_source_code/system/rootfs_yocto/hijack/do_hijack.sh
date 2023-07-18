#!/bin/bash

srcdir=$1
dstdir=$2

function copy_files()
{
  cp -a --remove-destination $1 $2
}

echo "Start Hijacking $2"

# make sure dstdir has the target directory structure
mkdir -p $dstdir/usr/bin
mkdir -p $dstdir/sbin/
mkdir -p $dstdir/etc/init.d
mkdir -p $dstdir/etc/rc5.d

# this is a example
copy_files "$srcdir/usr/bin/*" "$dstdir/usr/bin/"
# example end

# core R5 firmware
copy_files "$srcdir/lib/firmware/x3-cr5-rpmsg-service.axf" "$dstdir/lib/firmware/"

# init init-ad setprop reboot poweroff
copy_files "$srcdir/sbin/*" "$dstdir/sbin/"

# adbd
copy_files "$srcdir/adbd/bin/adbd" "$dstdir/usr/bin"
copy_files "$srcdir/adbd/adbd" "$dstdir/etc/init.d"
cd $dstdir
ln -sf ./etc/init.d/adbd ./etc/rc5.d/S50adbd

# asound.conf
copy_files "$srcdir/etc/asound.conf" "$dstdir/etc/asound.conf"

# usb-gadget ... not enabled as not ready
copy_files "$srcdir/usb/gadget/usb-gadget.sh" "$dstdir/etc/init.d/"
copy_files "$srcdir/usb/gadget/.usb" "$dstdir/etc/init.d/"
copy_files "$srcdir/usb/gadget/uvc-uac2-rndis.sh" "$dstdir/usr/bin/"
copy_files "$srcdir/usb/gadget/uvc-uac2-ecm.sh" "$dstdir/usr/bin/"
copy_files "$srcdir/usb/gadget/uac-demo.sh" "$dstdir/usr/bin/"
copy_files "$srcdir/usb/gadget/uac-driver.sh" "$dstdir/usr/bin/"
#ln -s  $dstdir/etc/init.d/usb-gadget.sh $dstdir/etc/rc5.d/S51usb-gadget

# secip
copy_files "$srcdir/secip/bin/*" "$dstdir/usr/bin/"
copy_files "$srcdir/secip/lib/*" "$dstdir/usr/lib/"

# etc/init.d
copy_files "${srcdir}/etc/init.d/1mkfs.sh" "${dstdir}/etc/init.d/"
copy_files "${srcdir}/etc/init.d/load_ko.sh" "${dstdir}/etc/init.d/"

# defaultip.sh
copy_files "${srcdir}/usr/bin/defaultip.sh" "${dstdir}/usr/bin/"

# This marks the end of ramfs hijack
if [ "$3" = "true" ];then
  echo "Hijack $2 done!"
  exit 0
fi

# bluetooth,  wifi start scripts and bluez
copy_files "${srcdir}/ap6212/etc/*" "${dstdir}/etc/"
copy_files "${srcdir}/ap6212/sbin/*" "${dstdir}/sbin/"
copy_files "${srcdir}/ap6212/lib/*" "${dstdir}/lib/"
copy_files "${srcdir}/ap6212/usr/*" "${dstdir}/usr/"

# FDE
if [ "${HR_FDE_HIJACK}" = "true" ];then
  copy_files "${srcdir}/etc/init.d/fde_default.bin" "${dstdir}/etc/init.d/"
  cat ${srcdir}/etc/fde_userfstab >${dstdir}/etc/userfstab
  cat ${srcdir}/etc/fde_fstab >${dstdir}/etc/fstab
  cat ${srcdir}/etc/fde_crypttab >${dstdir}/etc/crypttab
fi

# SELINUX
# Since init-ad has been updated, copy selinux dependencies to root
# by default including libselinux.
copy_files "$srcdir/selinux_bins/lib/libselinux.so.1" "${dstdir}/lib/"
if [ "$HR_SELINUX" = "true" ]; then
  echo "Hijacking for SELinux!"
  copy_files "$srcdir/selinux_bins/*" "${dstdir}/"
fi

echo "Hijack $2 done!"

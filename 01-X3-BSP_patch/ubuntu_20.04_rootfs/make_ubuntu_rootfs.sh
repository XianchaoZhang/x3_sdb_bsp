#!/bin/bash

set -e

LOCAL_DIR=$(realpath $(cd $(dirname $0); pwd))

# RELEASE=bionic
RELEASE="focal"
ARCH=arm64
DEBOOTSTRAP_COMPONENTS="main,universe"
# apt_mirror="http://ports.ubuntu.com/"
UBUNTU_MIRROR="mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/"

# apt_mirror="http://${UBUNTU_MIRROR}"
# To use a local proxy to cache apt packages, you need to install apt-cacher-ng
apt_mirror="http://localhost:3142/${UBUNTU_MIRROR}"


CONSOLE_CHAR="UTF-8"
DEST_LANG="en_US.UTF-8"

SUN_USERNAME="sunrise"
ROOTPWD="root"
SUN_PWD="sunrise"
HOST="ubuntu"

#network
ethdev="eth0"
address="192.168.1.10"
netmask="255.255.255.0"
gateway="192.168.1.1"

NAMESERVER="114.114.114.114"

COMMON_PACKAGE_LIST=" "
PYTHON_PACKAGE_LIST="numpy opencv-python pySerial i2cdev spidev matplotlib pillow \
websocket websockets lark-parser netifaces google protobuf==3.20.1 "

#DEBOOTSTRAP_LIST="systemd sudo locales apt-utils openssh-server ssh dbus init module-init-tools \
DEBOOTSTRAP_LIST="systemd sudo vim locales apt-utils openssh-server ssh dbus init \
strace kmod init udev bash-completion netbase network-manager \
ifupdown ethtool net-tools iputils-ping "

BASE_PACKAGE_LIST="file openssh-server ssh bsdmainutils whiptail device-tree-compiler \
bzip2 htop rsyslog parted python3 python3-pip console-setup fake-hwclock \
ncurses-term gcc g++ toilet sysfsutils rsyslog tzdata u-boot-tools \
libcjson1 libcjson-dev db-util diffutils e2fsprogs libc6 xterm \
libcrypt1 libcrypto++6 libdevmapper1.02.1 libedit2 libgcc-s1-arm64-cross libgcrypt20 libgpg-error0 \
libion0 libjsoncpp1 libkcapi1 libmenu-cache3 libnss-db libpcap0.8 libpcre3 \
libstdc++-10-dev libvorbis0a libzmq5 lvm2 makedev mtd-utils ncurses-term ncurses-base nettle-bin \
nfs-common openssl perl-base perl tftpd-hpa tftp-hpa tzdata watchdog \
wpasupplicant alsa-utils base-files cryptsetup diffutils dosfstools \
dropbear e2fsprogs ethtool exfat-utils ffmpeg i2c-tools iperf3 \
libaio1 libasound2 libattr1 libavcodec58 libavdevice58 libavfilter7 libavformat58 libavutil56 \
libblkid1 libc6 libc6-dev libcap2 libcom-err2 libcrypt-dev libdbus-1-3 libexpat1 libext2fs2 libflac8 \
libgcc1 libgdbm-compat4 libgdbm-dev libgdbm6 libgmp10 libgnutls30 libidn2-0 libjson-c4 libkmod2 \
liblzo2-2 libmount1 libncurses5 libncursesw5 libnl-3-200 libnl-genl-3-200 libogg0 libpopt0 \
libpostproc55 libreadline8 libsamplerate0 libsndfile1 libss2 libssl1.1 libstdc++6 libswresample3 \
libswscale5 libtinfo5 libtirpc3 libudev1 libunistring2 libusb-1.0-0 libuuid1 libwrap0 libx11-6 \
libxau6 libxcb1 libxdmcp6 libxext6 libxv1 libz-dev libz1 lrzsz lvm2 mtd-utils net-tools \
netbase openssh-sftp-server openssl rpcbind screen sysstat tcpdump libgl1-mesa-glx \
thin-provisioning-tools trace-cmd tzdata usbutils watchdog libturbojpeg libturbojpeg0-dev \
base-passwd libasound2-dev libavcodec-dev libavformat-dev libavutil-dev libcrypto++-dev \
libjsoncpp-dev libssl-dev libswresample-dev libzmq3-dev perl sed \
symlinks libunwind8 libperl-dev devmem2 ifmetric v4l-utils python3-dev \
build-essential libbullet-dev libasio-dev libtinyxml2-dev iotop htop iw wireless-tools \
bluetooth bluez blueman sqlite3 libsqlite3-dev libeigen3-dev liblog4cxx-dev libcurl4-openssl-dev \
libboost-dev libboost-date-time-dev libboost-thread-dev "

SERVER_PACKAGE_LIST="file openssh-server ssh bsdmainutils whiptail device-tree-compiler \
bzip2 htop rsyslog make cmake parted python3 python3-pip console-setup fake-hwclock \
ncurses-term gcc g++ toilet sysfsutils rsyslog tzdata u-boot-tools \
libcjson1 libcjson-dev db-util diffutils e2fsprogs iptables libc6 xterm \
libcrypt1 libcrypto++6 libdevmapper1.02.1 libedit2 libgcc-s1-arm64-cross libgcrypt20 libgpg-error0 \
libion0 libjsoncpp1 libkcapi1 libmenu-cache3 libnss-db libpcap0.8 libpcre3 \
libstdc++-10-dev libvorbis0a libzmq5 lvm2 makedev mtd-utils ncurses-term ncurses-base nettle-bin \
nfs-common openssl perl-base perl tftpd-hpa tftp-hpa tzdata watchdog \
wpasupplicant alsa-utils base-files cryptsetup diffutils dosfstools \
dropbear e2fsprogs ethtool exfat-utils ffmpeg file gdb gdbserver i2c-tools iperf3 iptables \
libaio1 libasound2 libattr1 libavcodec58 libavdevice58 libavfilter7 libavformat58 libavutil56 \
libblkid1 libc6 libc6-dev libcap2 libcom-err2 libcrypt-dev libdbus-1-3 libexpat1 libext2fs2 libflac8 \
libgcc1 libgdbm-compat4 libgdbm-dev libgdbm6 libgmp10 libgnutls30 libidn2-0 libjson-c4 libkmod2 \
liblzo2-2 libmount1 libncurses5 libncursesw5 libnl-3-200 libnl-genl-3-200 libogg0 libpopt0 \
libpostproc55 libreadline8 libsamplerate0 libsndfile1 libss2 libssl1.1 libstdc++6 libswresample3 \
libswscale5 libtinfo5 libtirpc3 libudev1 libunistring2 libusb-1.0-0 libuuid1 libwrap0 libx11-6 \
libxau6 libxcb1 libxdmcp6 libxext6 libxv1 libz-dev libz1 lrzsz lvm2 mtd-utils net-tools \
netbase openssh-sftp-server openssl rpcbind screen sysstat tcpdump libgl1-mesa-glx \
thin-provisioning-tools trace-cmd tzdata usbutils watchdog libturbojpeg libturbojpeg0-dev \
base-passwd libasound2-dev libavcodec-dev libavformat-dev libavutil-dev libcrypto++-dev \
libjsoncpp-dev libssl-dev libswresample-dev libzmq3-dev perl sed \
symlinks libunwind8 libperl-dev devmem2 tree unzip ifmetric v4l-utils python3-dev \
wget curl gnupg2 lsb-release lshw lsof memstat aptitude apt-show-versions \
build-essential libbullet-dev libasio-dev libtinyxml2-dev iotop htop iw wireless-tools \
bluetooth bluez blueman sqlite3 libsqlite3-dev libeigen3-dev liblog4cxx-dev libcurl4-openssl-dev \
libboost-dev libboost-date-time-dev libboost-thread-dev \
python3-wstool ninja-build stow \
libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev \
lua5.2 liblua5.2-dev libluabind-dev libprotobuf-dev protobuf-compiler libcairo2-dev \
hostapd isc-dhcp-server x11vnc "

DESKTOP_PACKAGE_LIST="xubuntu-desktop xserver-xorg-video-fbdev policykit-1-gnome notification-daemon \
tightvncserver network-manager-gnome gnome-terminal tightvncserver firefox firefox-locale-zh-hans \
gedit \
fonts-beng \
fonts-beng-extra fonts-deva fonts-deva-extra fonts-freefont-ttf fonts-gargi fonts-gubbi fonts-gujr fonts-gujr-extra fonts-guru fonts-guru-extra \
fonts-indic fonts-kacst fonts-kacst-one fonts-kalapi fonts-khmeros-core fonts-knda fonts-lao fonts-liberation fonts-lklug-sinhala \
fonts-lohit-beng-assamese fonts-lohit-beng-bengali fonts-lohit-deva fonts-lohit-gujr fonts-lohit-guru fonts-lohit-knda fonts-lohit-mlym \
fonts-lohit-orya fonts-lohit-taml fonts-lohit-taml-classical fonts-lohit-telu fonts-mlym fonts-nakula fonts-navilu fonts-noto-cjk \
fonts-noto-core fonts-noto-hinted fonts-noto-ui-core fonts-orya fonts-orya-extra fonts-pagul fonts-sahadeva fonts-samyak-deva fonts-samyak-gujr \
fonts-samyak-mlym fonts-samyak-taml fonts-sarai fonts-sil-abyssinica fonts-sil-padauk fonts-smc fonts-smc-anjalioldlipi fonts-smc-chilanka \
fonts-smc-dyuthi fonts-smc-gayathri fonts-smc-karumbi fonts-smc-keraleeyam fonts-smc-manjari fonts-smc-meera fonts-smc-rachana \
fonts-smc-raghumalayalamsans fonts-smc-suruma fonts-smc-uroob fonts-symbola fonts-taml fonts-telu fonts-telu-extra fonts-thai-tlwg \
fonts-tibetan-machine fonts-tlwg-garuda fonts-tlwg-garuda-ttf fonts-tlwg-kinnari fonts-tlwg-kinnari-ttf fonts-tlwg-laksaman \
fonts-tlwg-laksaman-ttf fonts-tlwg-loma fonts-tlwg-loma-ttf fonts-tlwg-mono fonts-tlwg-mono-ttf fonts-tlwg-norasi fonts-tlwg-norasi-ttf \
fonts-tlwg-purisa fonts-tlwg-purisa-ttf fonts-tlwg-sawasdee fonts-tlwg-sawasdee-ttf fonts-tlwg-typewriter fonts-tlwg-typewriter-ttf \
fonts-tlwg-typist fonts-tlwg-typist-ttf fonts-tlwg-typo fonts-tlwg-typo-ttf fonts-tlwg-umpush fonts-tlwg-umpush-ttf fonts-tlwg-waree \
fonts-tlwg-waree-ttf fonts-ubuntu fonts-yrsa-rasa \
smplayer pavucontrol pulseaudio "

# The default version is Ubuntu Server
ADD_PACKAGE_LIST="${SERVER_PACKAGE_LIST} "

ubuntufs_src="${LOCAL_DIR}/ubuntu_src"
ubuntufs_dst="${LOCAL_DIR}/system"

# Ubuntu Desktop
if [[ $1 == *"d"*  ]] ; then
    desktop="true"
    ADD_PACKAGE_LIST="$ADD_PACKAGE_LIST $DESKTOP_PACKAGE_LIST "
    ubuntufs_src="${LOCAL_DIR}/ubuntu_src_desktop"
fi

# Ubuntu Base
if [[ $1 == *"b"*  ]] ; then
    ADD_PACKAGE_LIST="${BASE_PACKAGE_LIST} "
    ubuntufs_src="${LOCAL_DIR}/ubuntu_src_base"
fi

root_path=${ubuntufs_src}/${RELEASE}-xj3-${ARCH}
tar_file=${root_path}.tar.xz

# Release specific packages
case $RELEASE in
    bionic)
        # Dependent debootstarp packages
        DEBOOTSTRAP_COMPONENTS="main,universe"
        DEBOOTSTRAP_LIST+=" module-init-tools"
        ADD_PACKAGE_LIST+=" android-tools-adbd"
    ;;
    focal)
        # Dependent debootstarp packages
        DEBOOTSTRAP_COMPONENTS="main,universe"
        DEBOOTSTRAP_LIST+=""
        ADD_PACKAGE_LIST+=""
    ;;
esac


log_out()
{
    # log function parameters to install.log
    local tmp=""
    [[ -n $2 ]] && tmp="[\e[0;33m $2 \x1B[0m]"

    case $3 in
        err)
        echo -e "[\e[0;31m error \x1B[0m] $1 $tmp"
        ;;

        wrn)
        echo -e "[\e[0;35m warn \x1B[0m] $1 $tmp"
        ;;

        ext)
        echo -e "[\e[0;32m o.k. \x1B[0m] \e[1;32m$1\x1B[0m $tmp"
        ;;

        info)
        echo -e "[\e[0;32m o.k. \x1B[0m] $1 $tmp"
        ;;

        *)
        echo -e "[\e[0;32m .... \x1B[0m] $1 $tmp"
        ;;
    esac
}

# mount_chroot <target>
#
# helper to reduce code duplication
#
mount_chroot()
{
    local target=$1
    log_out "Mounting" "$target" "info"
    mount -t proc chproc "${target}"/proc
    mount -t sysfs chsys "${target}"/sys
    mount -t devtmpfs chdev "${target}"/dev || mount --bind /dev "${target}"/dev
    mount -t devpts chpts "${target}"/dev/pts
} 

# unmount_on_exit <target>
#
# helper to reduce code duplication
#
unmount_on_exit()
{
    local target=$1
    trap - INT TERM EXIT
    umount_chroot "${target}/"
    rm -rf ${target}
}


# umount_chroot <target>
#
# helper to reduce code duplication
#
umount_chroot()
{
    local target=$1
    log_out "Unmounting" "$target" "info"
    while grep -Eq "${target}.*(dev|proc|sys)" /proc/mounts
    do
        umount -l --recursive "${target}"/dev >/dev/null 2>&1
        umount -l "${target}"/proc >/dev/null 2>&1
        umount -l "${target}"/sys >/dev/null 2>&1
        sleep 5
    done
} 

create_base_sources_list()
{
    local release=$1
    local basedir=$2
    [[ -z $basedir ]] && log_out "No basedir passed to create_base_sources_list" " " "err"
    # cp /etc/apt/sources.list "${basedir}"/etc/apt/sources.list
    cat <<-EOF > "${basedir}"/etc/apt/sources.list
# See http://help.ubuntu.com/community/UpgradeNotes for how to upgrade to
# newer versions of the distribution.
deb http://${UBUNTU_MIRROR} $release main restricted universe multiverse
#deb-src http://${UBUNTU_MIRROR} $release main restricted universe multiverse
EOF
}


create_sources_list()
{
    local release=$1
    local basedir=$2
    [[ -z $basedir ]] && log_out "No basedir passed to create_sources_list" " " "err"
    # cp /etc/apt/sources.list "${basedir}"/etc/apt/sources.list
    cat <<-EOF > "${basedir}"/etc/apt/sources.list
# See http://help.ubuntu.com/community/UpgradeNotes for how to upgrade to
# newer versions of the distribution.
deb http://${UBUNTU_MIRROR} $release main restricted universe multiverse
#deb-src http://${UBUNTU_MIRROR} $release main restricted universe multiverse

deb http://${UBUNTU_MIRROR} ${release}-security main restricted universe multiverse
#deb-src http://${UBUNTU_MIRROR} ${release}-security main restricted universe multiverse

deb http://${UBUNTU_MIRROR} ${release}-updates main restricted universe multiverse
#deb-src http://${UBUNTU_MIRROR} ${release}-updates main restricted universe multiverse

deb http://${UBUNTU_MIRROR} ${release}-backports main restricted universe multiverse
#deb-src http://${UBUNTU_MIRROR} ${release}-backports main restricted universe multiverse
EOF
}

end_debootstrap()
{
    local target=$1
    # remove service start blockers
    rm -f "${target}"/sbin/initctl "${target}"/sbin/start-stop-daemon
    chroot "${target}" /bin/bash -c "dpkg-divert --quiet --local --rename --remove /sbin/initctl"
    chroot "${target}" /bin/bash -c "dpkg-divert --quiet --local --rename --remove /sbin/start-stop-daemon"
    rm -f "${target}"/usr/sbin/policy-rc.d
}

extract_base_root() {
    local tar_file=$1
    local dest_dir=$2
    rm -rf $dest_dir
    mkdir -p $dest_dir
    if [ ! -f "${tar_file}0" ];then
        log_out "File is not exist!" "${tar_file}0" "err"
        exit -1
    fi
    if [ ! -d $dest_dir ];then
        log_out "Dir is not exist!" "${dest_dir}" "err"
        exit -1
    fi
    log_out "Start extract" "$tar_file to $dest_dir" "info"
    cat ${tar_file}[0-9] | tar -Jxf - -C $dest_dir --exclude='./dev/*' --exclude='./proc/*' --exclude='./run/*' --exclude='./tmp/*' --exclude='./sys/*'
    mkdir -p ${dest_dir}/{dev,proc,tmp,run,proc,sys,userdata,app}
}

compress_base_root() {
    local tar_file=$1
    local src_dir=$2
    if [ ! -d $src_dir ];then
        log_out "Dir is not exist!" "${src_dir}" "err"
        exit -1
    fi
    log_out "Start compress" "${tar_file} from ${src_dir}" "info"
    tar -Jcf - -C $src_dir/ --exclude='./dev/*' --exclude='./proc/*' --exclude='./run/*' --exclude='./tmp/*' --exclude='./sys/*' --exclude='./usr/lib/aarch64-linux-gnu/dri/*' .  | split -b 99m -d -a 1 - ${tar_file}
    # rm -rf $src_dir
}

check_ret(){
    ret=$1
    if [ ${ret} -ne 0 ];then
        log_out "return value:" "${ret}" "err"
        exit -1
    fi
}

make_base_root() {
    local dst_dir=$1
    rm -rf $dst_dir
    mkdir -p $dst_dir
    trap "unmount_on_exit ${dst_dir}" INT TERM EXIT
    log_out "Installing base system : " "Stage 2/1" "info"
    debootstrap --variant=minbase \
        --include=${DEBOOTSTRAP_LIST// /,} \
        --arch=${ARCH} \
        --components=${DEBOOTSTRAP_COMPONENTS} \
        --foreign ${RELEASE} \
        $dst_dir \
        $apt_mirror
    if [[ $? -ne 0 ]] || [[ ! -f $dst_dir/debootstrap/debootstrap ]];then 
        log_out "Debootstrap base system first stage failed" "err"
        exit -1
    fi
    if [ ! -f /usr/bin/qemu-aarch64-static ];then
        log_out "File is not exist!" "Please install qemu-user-static with apt first" "err"
        exit -1
    else
        log_out "Copy qemu-aarch64-static to" "$dst_dir/usr/bin" "info"
        cp /usr/bin/qemu-aarch64-static $dst_dir/usr/bin
    fi
    mkdir -p $dst_dir/usr/share/keyrings/
    log_out "Copy .gpg files to" "$dst_dir/usr/share/keyrings/" "info"
    cp -a /usr/share/keyrings/*-archive-keyring.gpg $dst_dir/usr/share/keyrings/
    log_out "Installing base system : " "Stage 2/2" "info"
    chroot ${dst_dir} /bin/bash -c "/debootstrap/debootstrap --second-stage"
    if [[ $? -ne 0 ]] || [[ ! -f $dst_dir/bin/bash ]];then
        log_out "Debootstrap base system second stage failed" "err"
        exit -1
    fi
    mount_chroot ${dst_dir}

    printf '#!/bin/sh\nexit 101' > $dst_dir/usr/sbin/policy-rc.d
    chroot $dst_dir /bin/bash -c "dpkg-divert --quiet --local --rename --add /sbin/initctl"
    chroot $dst_dir /bin/bash -c "dpkg-divert --quiet --local --rename --add /sbin/start-stop-daemon"
    printf '#!/bin/sh\necho "Warning: Fake start-stop-daemon called, doing nothing"' > $dst_dir/sbin/start-stop-daemon
    printf '#!/bin/sh\necho "Warning: Fake initctl called, doing nothing"' > $dst_dir/sbin/initctl
    chmod 755 $dst_dir/usr/sbin/policy-rc.d
    chmod 755 $dst_dir/sbin/initctl
    chmod 755 $dst_dir/sbin/start-stop-daemon

    log_out "Configuring locales" "$DEST_LANG" "info"
    if [ -f ${dst_dir}/etc/locale.gen ];then
        sed -i "s/^# $DEST_LANG/$DEST_LANG/" $dst_dir/etc/locale.gen
    fi
    eval 'LC_ALL=C LANG=C chroot $dst_dir /bin/bash -c "locale-gen $DEST_LANG"'
    eval 'LC_ALL=C LANG=C chroot $dst_dir /bin/bash -c "update-locale LANG=$DEST_LANG LANGUAGE=$DEST_LANG LC_MESSAGES=$DEST_LANG"'

    if [[ -f ${dst_dir}/etc/default/console-setup ]]; then
            sed -e 's/CHARMAP=.*/CHARMAP="UTF-8"/' -e 's/FONTSIZE=.*/FONTSIZE="8x16"/' \
                -e 's/CODESET=.*/CODESET="guess"/' -i ${dst_dir}/etc/default/console-setup
            eval 'LC_ALL=C LANG=C chroot ${dst_dir} /bin/bash -c "setupcon --save"'
    fi

    # this should fix resolvconf installation failure in some cases
    chroot ${dst_dir} /bin/bash -c 'echo "resolvconf resolvconf/linkify-resolvconf boolean false" | debconf-set-selections'

    local apt_extra="-o Acquire::http::Proxy=\"http://localhost:3142\""
    # base for gcc 9.3
    create_base_sources_list ${RELEASE} ${dst_dir}
    log_out "Updating base packages" "${dst_dir}" "info"
    eval 'LC_ALL=C LANG=C chroot ${dst_dir} /bin/bash -c "apt-get -q -y $apt_extra update"'
    [[ $? -ne 0 ]] && exit -1
    log_out "Upgrading base packages" "${dst_dir}" "info"
    eval 'LC_ALL=C LANG=C chroot ${dst_dir} /bin/bash -c "apt-get -q -y $apt_extra upgrade"'
    [[ $? -ne 0 ]] && exit -1
    log_out "Installing base packages" "${dst_dir}" "info"
    eval 'LC_ALL=C LANG=C chroot ${dst_dir} /bin/bash -c "DEBIAN_FRONTEND=noninteractive apt-get -y -q $apt_extra --no-install-recommends install $ADD_PACKAGE_LIST"'
    [[ $? -ne 0 ]] && exit -1
    chroot ${dst_dir} /bin/bash -c "dpkg --get-selections" | grep -v deinstall | awk '{print $1}' | cut -f1 -d':' > ${tar_file}.info

    # Fixed GCC version: 9.3.0
    chroot ${dst_dir} /bin/bash -c "apt-mark hold cpp-9 g++-9 gcc-9-base gcc-9 libasan5 libgcc-9-dev libstdc++-9-dev"

    # upgrade packages
    create_sources_list ${RELEASE} ${dst_dir}
    log_out "Updating focal-updates and focal-security packages" "${dst_dir}" "info"
    eval 'LC_ALL=C LANG=C chroot ${dst_dir} /bin/bash -c "apt-get -q -y $apt_extra update"'
    [[ $? -ne 0 ]] && exit -1
    log_out "Upgrading base packages" "${dst_dir}" "info"
    eval 'LC_ALL=C LANG=C chroot ${dst_dir} /bin/bash -c "apt-get -q -y $apt_extra upgrade"'
    [[ $? -ne 0 ]] && exit -1
    log_out "Installing base packages" "${dst_dir}" "info"
    eval 'LC_ALL=C LANG=C chroot ${dst_dir} /bin/bash -c "DEBIAN_FRONTEND=noninteractive apt-get -y -q $apt_extra --no-install-recommends install $ADD_PACKAGE_LIST"'
    [[ $? -ne 0 ]] && exit -1
    chroot ${dst_dir} /bin/bash -c "dpkg --get-selections" | grep -v deinstall | awk '{print $1}' | cut -f1 -d':' > ${tar_file}.info

    log_out "Configure hostname" "$HOST" "info"
    echo "$HOST" > ${dst_dir}/etc/hostname

    log_out "Configure resolv" ""${dst_dir}"/etc/resolv.conf" "info"
    rm ${dst_dir}/etc/resolv.conf
    echo "nameserver $NAMESERVER" > ${dst_dir}/etc/resolv.conf
    chroot ${dst_dir} /bin/bash -c "pip3 config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple"
    chroot ${dst_dir} /bin/bash -c "pip3 config set install.trusted-host https://pypi.tuna.tsinghua.edu.cn"
    chroot ${dst_dir} /bin/bash -c "pip3 install $PYTHON_PACKAGE_LIST"

    chroot ${dst_dir} /bin/bash -c "apt clean"

    log_out "Configure udev" ""${dst_dir}"/lib/udev/rules.d/" "info"

    # fixed camera sensor bug and fixed auto load audio ko bug
    # sed -i 's/ENV{MODALIAS}/# ENV{MODALIAS}/' "${dst_dir}"/lib/udev/rules.d/80-drivers.rules
    sed -i 's/IMPORT{program}/# IMPORT{program}/' "${dst_dir}"/lib/udev/rules.d/60-persistent-v4l.rules

    log_out "Configure udev" "${dst_dir}/lib/systemd/system/udev.service" "info"
    mkdir "${dst_dir}"/etc/systemd/system/systemd-udevd.service.d
    cat <<-EOF >> "${dst_dir}"/etc/systemd/system/systemd-udevd.service.d/00-hobot-private.conf
[Service]
PrivateMounts=no
EOF

    log_out "Configure ssh" ""${dst_dir}"/etc/ssh/sshd_config" "info"
    # permit root login via SSH for the first boot
    sed -i 's/#\?PermitRootLogin .*/PermitRootLogin yes/' "${dst_dir}"/etc/ssh/sshd_config
    # enable PubkeyAuthentication
    sed -i 's/#\?PubkeyAuthentication .*/PubkeyAuthentication yes/' "${dst_dir}"/etc/ssh/sshd_config

    # # console fix due to Debian bug
    sed -e 's/CHARMAP=".*"/CHARMAP="'$CONSOLE_CHAR'"/g' -i "${dst_dir}"/etc/default/console-setup

    # add the /dev/urandom path to the rng config file
    echo "HRNGDEVICE=/dev/urandom" >> "${dst_dir}"/etc/default/rng-tools

    cat <<-EOF >> "${dst_dir}"/etc/hosts
127.0.0.1   localhost
127.0.0.1   $HOST
EOF
    log_out "Setup net IP =" "${address}" "info"
    cat <<-EOF >> "${dst_dir}"/etc/network/interfaces
auto ${ethdev}
iface ${ethdev} inet static
        address ${address}
        netmask ${netmask}
        #network ${network}
        #broadcast ${broadcast}
        gateway ${gateway}
        metric 700
EOF


    cp "${dst_dir}"/lib/systemd/system/serial-getty@.service "${dst_dir}/lib/systemd/system/serial-getty@ttyS0.service"
    sed -i "s/--keep-baud 115200/--keep-baud 921600,115200/"  "${dst_dir}/lib/systemd/system/serial-getty@ttyS0.service"
    log_out "Enabling serial console" "ttyS0" "info"
    chroot "${dst_dir}" /bin/bash -c "systemctl daemon-reload"
    chroot "${dst_dir}" /bin/bash -c "systemctl --no-reload enable serial-getty@ttyS0.service"

    chroot "${dst_dir}" /bin/bash -c "(echo $ROOTPWD;echo $ROOTPWD;) | passwd root"
    chroot "${dst_dir}" /bin/bash -c "adduser --quiet --disabled-password --shell /bin/bash --ingroup sudo --home /home/${SUN_USERNAME} --gecos ${SUN_USERNAME} ${SUN_USERNAME}"
    chroot "${dst_dir}" /bin/bash -c "(echo ${SUN_PWD};echo ${SUN_PWD};) | passwd ${SUN_USERNAME}"
    chroot "${dst_dir}" /bin/bash -c "addgroup --quiet ${SUN_USERNAME}"

    # configure network manager
    sed "s/managed=\(.*\)/managed=true/g" -i "${dst_dir}"/etc/NetworkManager/NetworkManager.conf

    # most likely we don't need to wait for nm to get online
    chroot "${dst_dir}" /bin/bash -c "systemctl disable NetworkManager-wait-online.service"
    chroot "${dst_dir}" /bin/bash -c "systemctl disable hostapd.service"

    # Just regular DNS and maintain /etc/resolv.conf as a file
    sed "/dns/d" -i "${dst_dir}"/etc/NetworkManager/NetworkManager.conf
    sed "s/\[main\]/\[main\]\ndns=default\nrc-manager=file/g" -i "${dst_dir}"/etc/NetworkManager/NetworkManager.conf
    if [[ -n $NM_IGNORE_DEVICES ]]; then
        mkdir -p "${dst_dir}"/etc/NetworkManager/conf.d/
        cat <<-EOF > "${dst_dir}"/etc/NetworkManager/conf.d/10-ignore-interfaces.conf
[keyfile]
unmanaged-devices=$NM_IGNORE_DEVICES
EOF
    fi

    chroot ${dst_dir} /bin/bash -c "chmod a=rx,u+ws /usr/bin/sudo"

     #/etc/apt/source.list
    chroot ${dst_dir} /bin/bash -c "apt clean"

    chroot ${dst_dir} /bin/bash -c "rm -f /var/lib/apt/lists/mirrors*"
    chroot ${dst_dir} /bin/bash -c "rm -rf /home/hobot"

    log_out "Configure tzdata" "${dst_dir}/etc/timezone" "info"
    echo "`cat /etc/timezone`" > "${dst_dir}"/etc/timezone
    chroot "${dst_dir}" /bin/bash -c "dpkg-reconfigure -f noninteractive tzdata >/dev/null 2>&1"

    # initial date for fake-
    log_out "Configure fake-hwclock" "date :`date '+%Y-%m-%d %H:%M:%S'`" "info"
    date '+%Y-%m-%d %H:%M:%S' > "${dst_dir}"/etc/fake-hwclock.data

    umount_chroot ${dst_dir}
    end_debootstrap ${dst_dir}
    chmod 777 ${dst_dir}/home/ -R

    # enable few bash aliases enabled in Ubuntu by default to make it even
    sed "s/#alias ll='ls -l'/alias ll='ls -l'/" -i "${dst_dir}"/etc/skel/.bashrc
    sed "s/#alias la='ls -A'/alias la='ls -A'/" -i "${dst_dir}"/etc/skel/.bashrc
    sed "s/#alias l='ls -CF'/alias l='ls -CF'/" -i "${dst_dir}"/etc/skel/.bashrc

    echo "/usr/bin/resize > /dev/null" >> "${dst_dir}"/etc/skel/.bashrc

    # root user is already there. Copy bashrc there as well
    cp "${dst_dir}"/etc/skel/.bashrc "${dst_dir}"/root

    if [ x$"desktop" == x"true" ];then
        log_out "Recreating Synaptic search index" "Please wait" "info"
        chroot ${dst_dir} /bin/bash -c "/usr/sbin/update-apt-xapian-index -u"

        # fix for gksu in Xenial
        touch ${dst_dir}/home/${SUN_USERNAME}/.Xauthority
        chroot "${dst_dir}" /bin/bash -c "chown ${SUN_USERNAME}:${SUN_USERNAME} /home/${SUN_USERNAME}/.Xauthority"
        # set up profile sync daemon on desktop systems
        chroot "${dst_dir}" /bin/bash -c "which psd >/dev/null 2>&1"
        if [ $? -eq 0 ]; then
            echo -e "${SUN_USERNAME} ALL=(ALL) NOPASSWD: /usr/bin/psd-overlay-helper" >> ${dst_dir}/etc/sudoers
            touch ${dst_dir}/home/${SUN_USERNAME}/.activate_psd
            chroot "${dst_dir}" /bin/bash -c "chown $SUN_USERNAME:$SUN_USERNAME /home/${SUN_USERNAME}/.activate_psd"
        fi
    fi

    #store size of dst_dir to ${tar_file}.info
    local dusize=`du -sh ${dst_dir} 2> /dev/null |awk '{print $1}'`
    echo "DIR_DU_SIZE ${dusize%%M}" >> ${tar_file}.info
    trap - INT TERM EXIT
}

log_out "Build ubuntu base" "root_path=$root_path tar_file=$tar_file" "info"
log_out "Start build" "ubuntu base :${RELEASE}-xj3-${ARCH}" "info"

mkdir -p ${ubuntufs_dst}
if [ ! -f "${tar_file}0" ];then
    make_base_root "${root_path}"
    sync
    # Compression takes longer
    compress_base_root "${tar_file}" "${root_path}" s
    extract_base_root "${tar_file}" "${ubuntufs_dst}"
    sync
else
    extract_base_root "${tar_file}" "${ubuntufs_dst}"
    sync
fi

log_out "End build ubuntu base" "${ubuntufs_dst}" "info"
exit 0

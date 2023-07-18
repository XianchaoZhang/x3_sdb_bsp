#!/bin/sh
#
# forcefde   force encrypt userdata partition
#
#

BOOTUP=color
SETCOLOR_SUCCESS="echo -en \\033[1;32m"
SETCOLOR_FAILURE="echo -en \\033[1;31m"

echo_success() {
    [ "$BOOTUP" = "color" ] && $MOVE_TO_COL
    echo -n "["
    [ "$BOOTUP" = "color" ] && $SETCOLOR_SUCCESS
    echo -n "$1  OK  "
    [ "$BOOTUP" = "color" ] && $SETCOLOR_NORMAL
    echo -n "]"
    echo -ne "\r"
    return 0
}

echo_failure() {
    [ "$BOOTUP" = "color" ] && $MOVE_TO_COL
    echo -n "[" > /dev/ttyS0
    [ "$BOOTUP" = "color" ] && $SETCOLOR_FAILURE
    echo -n "$1 FAILED" > /dev/ttyS0
    [ "$BOOTUP" = "color" ] && $SETCOLOR_NORMAL
    echo -n "]" > /dev/ttyS0
    echo -ne "\r" > /dev/ttyS0
    return 1
}

# Log that something succeeded
success() {
    [ "$BOOTUP" != "verbose" -a -z "${LSB:-}" ] && echo_success $1
    return 0
}

# Log that something failed
failure() {
    local rc=$?
    [ "$BOOTUP" != "verbose" -a -z "${LSB:-}" ] && echo_failure $1 > /dev/ttyS0
    return $rc
}

key_is_random() {
    [ "$1" = "/dev/urandom" -o "$1" = "/dev/hw_random" \
        -o "$1" = "/dev/random" ]
}

find_crypto_mount_point() {
    local fs_spec fs_file fs_vfstype remaining_fields
    local fs
    while read fs_spec fs_file remaining_fields; do
        if [ "$fs_spec" = "/dev/mapper/$1" ]; then
            echo $fs_file
            break
        fi
    done </etc/userfstab
}

# Because of a chicken/egg problem, init_crypto must be run twice.  /var may be
# encrypted but /var/lib/random-seed is needed to initialize swap.
init_crypto() {
    local have_random dst src key opt mode owner params makeswap skip arg opt
    local param value rc ret mke2fs mdir mount_point

    [ -z "$(mount | grep /run/cryptsetup)" ] && mount -t tmpfs tmpfs /run/cryptsetup/
    ret=0
    have_random=$1
    while read dst src opt key if_force; do
        [ -z "$dst" -o "${dst#\#}" != "$dst" ] && continue
        [ -b "/dev/mapper/$dst" ] && continue

        if [ "$have_random" = 0 ] && key_is_random "$key"; then
            continue
        fi

        if [ -n "$key" -a "x$key" != "xnone" ]; then
            if test -e "$key"; then
                mode=$(ls -l "$key" | cut -c 5-10)
                owner=$(ls -l $key | awk '{ print $3 }')
                if [ "$mode" != "------" ] && ! key_is_random "$key"; then
                    echo "PARAM: INSECURE MODE FOR $key"
                fi

                if [ "$owner" != root ]; then
                    echo "PARAM: INSECURE OWNER FOR $key"
                fi
            else
                echo "PARAM: Key file for $dst not found"
            fi
        else
            key=""
        fi

        # limit memory usage
        params="--pbkdf-memory 256 --progress-frequency 2"
        makeswap=""
        mke2fs=""
        skip=""
        # Parse the src field for UUID= and convert to real device names
        if [ "${src%%=*}" == "UUID" ]; then
            src=$(/sbin/blkid -t "$src" -l -o device)
        elif [ "${src/^\/dev\/disk\/by-uuid\//}" != "$src" ]; then
            src=$(__readlink $src)
        fi

        # Is it a block device?
        [ -b "$src" ] || continue

        # Is it already a device mapper slave? (this is gross)
        devesc=${src##/dev/}
        devesc=${devesc//\//!}
        for d in /sys/block/dm-*/slaves; do
            [ -e $d/$devesc ] && continue 2
        done

        # Parse the options field, convert to cryptsetup parameters
        # and contruct the command line
        while [ -n "$opt" ]; do
            arg=${opt%%,*}
            opt=${opt##$arg}
            opt=${opt##,}
            param=${arg%%=*}
            value=${arg##$param=}

            case "$param" in
            cipher)
                echo "PARAM: CRYPTSETUP: cipher value: $value"
                params="$params -c $value"
                if [ -z "$value" ]; then
                    echo $"PARAM: $dst: no value for cipher option, skipping"
                    skip="yes"
                fi
                ;;
            size)
                params="$params -s $value"
                if [ -z "$value" ]; then
                    echo $"PARAM: $dst: no value for size option, skipping"
                    skip="yes"
                fi
                ;;
            hash)
                params="$params -h $value"
                if [ -z "$value" ]; then
                    echo $"PARAM: $dst: no value for hash option, skipping\n"
                    skip="yes"
                fi
                ;;
            verify)
                params="$params -y"
                ;;
            swap)
                makeswap=yes
                ;;
            tmp)
                mke2fs=yes
                ;;
            esac
        done

        if [ "$skip" = "yes" ]; then
            ret=1
            continue
        fi

        echo "CRYPTSETUP: $src"
        if cryptsetup isLuks "$src" 2>/dev/null; then
            if key_is_random "$key"; then
                echo "CRYPTSETUP-Open: $dst: LUKS requires non-random key, skipping\n"
                ret=1
                continue
            fi
            echo "CRYPTSETUP-Open: $params"
            #echo "CRYPTSETUP: key:$key src:$src dst:$dst"
            if test -e "$key"; then
                /sbin/cryptsetup $params luksOpen --test-passphrase ${key:+-d $key} "$src"
                rc=$?
            else
                rc=1
            fi
            if [ $rc -ne 0 ]; then
                if test -e "$key"; then
                    rm $key
                fi
                /sbin/cryptsetup $params luksOpen "$src" "$dst" <&1
                rc=$?
                if [ $rc -ne 0 ]; then
                    ret=1
                    return $ret
                fi
            else
                /sbin/cryptsetup ${key:+-d $key} $params -v luksOpen "$src" "$dst" <&1
                rc=$?
                if [ $rc -ne 0 ]; then
                    ret=1
                    return $ret
                fi
            fi
        else
            echo "CRYPTSETUP-Format: key:$key src:$src dst:$dst"
            #/sbin/cryptsetup $params ${key:+-d $key} -v create "$dst" "$src" <&1 2>/dev/null
            #source /etc/init.d/forceencrypt.sh $src $dst $mount_point "$key" "$params"
            e2fsck -y $src > /dev/null 2>&1
            fsck_res=$?
            if [ "$if_force" = "forceencrypt" ];then
                echo "CRYPTSETUP-Format: \"forceencrypt\" detected! Keeping all content in $src" > /dev/ttyS0
                if [ ${fsck_res} -le 7 ];then
                    # try to reserve enough space for cryptsetup while there is filesystem in $src
                    resize2fs -f -M -p /dev/mmcblk0p12
                fi
                # Even raw contents will be kept and encrypted
                cd /run/cryptsetup/
                cryptsetup reencrypt --encrypt --reduce-device-size 32M $params -q --type luks2 $src -d $key -v
                cd -
            else
                [ ! -z "$(mount | grep $src)" ] && umount $src
                cryptsetup $params -q luksFormat --type luks2 $src -d $key -v
            fi
            rc=$?
            if [ $rc -ne 0 ]; then
                echo "CRYPTSETUP-Format: format with param: $params fail" > /dev/ttyS0
                return $rc
            else
                cryptsetup $params luksOpen --type luks2 $src $dst -d $key -v
                e2fsck -y /dev/mapper/$dst > /dev/null 2>&1
                fsck_res=$?
                if [ "$if_force" = "forceencrypt" -a ${fsck_res} -le 7 ];then
                    resize2fs -f /dev/mapper/$dst
                else
                    mkfs.ext4 /dev/mapper/$dst
                fi
            fi

            rc=$?
            if [ $rc -ne 0 ]; then
                echo "encrypt $src<or $mount_point> fail" > /dev/ttyS0
                return $rc
            fi
        fi

        echo "CRYPTSETUP-Mount: $dst"
        if [ -b "/dev/mapper/$dst" ]; then
            if [ "$makeswap" = "yes" ]; then
                echo "CRYPTSETUP-Mount: mkswap $dst"
                mkswap "/dev/mapper/$dst" 2>/dev/null >/dev/null
            fi
            mount_point="$(find_crypto_mount_point $dst)"
            echo "CRYPTSETUP-Mount: mount $dst to $mount_point"
            mount "/dev/mapper/$dst" "$mount_point"
            rc=$?
            if [ $rc -ne 0 ]; then
                echo "encrypt $dst fail" > /dev/ttyS0
                return $rc
            fi
        fi
    done </etc/crypttab
    return $ret
}

if [ -f /etc/crypttab -a "$(cat /sys/class/socinfo/boot_mode)" = "0" ];then
    s="FDE Init:"
    echo $s
    mount -t tmpfs tmpfs /run
    mkdir -p /run/cryptsetup
    init_crypto 0
    ret=$?
    echo "Encryption reault $ret"
    if [ $ret -ne 0 ]; then
        echo "[$s FAIL]" > /dev/ttyS0
    else
        echo "[$s OK]"
    fi
fi

setprop fde.ready 1

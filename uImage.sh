#!/bin/bash -ex
#make vmlinux $@
mipsel-openwrt-linux-objcopy -O binary -R .reginfo -R .notes -R .note -R .comment -R .mdebug -R .note.gnu.build-id -S ./vmlinux ./vmlinux.stripped
gzip -9 -c ./vmlinux.stripped > ./vmlinux.stripped.gz
mkimage -A mips -O linux -T kernel -a 0x80010000 -C gzip -e 0xffffffff80366ec0 -n 'MIPS OpenWrt Linux-3.3.8' -d ./vmlinux.stripped.gz openwrt-xburst-qi_lb60-uImage.bin

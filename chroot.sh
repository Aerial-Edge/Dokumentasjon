#!/bin/bash

mount /dev/sda2 /mnt/raspbian

mount --bind /dev /mnt/raspbian/dev/
mount --bind /sys /mnt/raspbian/sys/
mount --bind /proc /mnt/raspbian/proc
mount --bind /dev/pts /mnt/raspbian/dev/pts

chroot /mnt/raspbian qemu-aarch64-static /bin/bash

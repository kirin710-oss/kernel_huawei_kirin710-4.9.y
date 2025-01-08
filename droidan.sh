#!/bin/bash

# Create debian directory and its subdirectories
mkdir -p debian/source

# Create debian/control
cat > debian/control << 'EOF'
Source: linux-android-huawei-kirin710
Section: kernel
Priority: optional
Maintainer: Your Name <you@example.com>
Build-Depends: debhelper (>= 13), bc, kmod, cpio, rsync
Standards-Version: 4.1.0

Package: linux-image-huawei-kirin710
Architecture: arm64
Depends: ${misc:Depends}
Description: Linux kernel image for Huawei Kirin 710 devices
 This package contains the Linux kernel image for Huawei devices
 based on the Kirin 710 platform.
EOF

# Create debian/kernel-info.mk
cat > debian/kernel-info.mk << 'EOF'
# Device name
DEVICE_NAME := Huawei Kirin 710

# Kernel base version
KERNEL_BASE_VERSION := 4.9

# Defconfig
KERNEL_DEFCONFIG := merge_kirin710_defconfig
EOF

# Create debian/rules
cat > debian/rules << 'EOF'
#!/usr/bin/make -f

export KBUILD_BUILD_VERSION = 1
export DEB_HOST_ARCH = arm64

%:
	dh $@
EOF

# Make rules executable
chmod +x debian/rules

# Create debian/compat
echo "13" > debian/compat

# Create debian/source/format
echo "3.0 (native)" > debian/source/format

# Git commands to commit changes
#git add debian/
#git commit -m "Add Debian packaging for Droidian kernel build"

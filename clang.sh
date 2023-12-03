#!/bin/bash
make CC=$(pwd)/clang/bin/clang-15 \
LD=$(pwd)/clang/bin/ld.lld \
NM=$(pwd)/clang/bin/llvm-nm \
AR=$(pwd)/clang/bin/llvm-ar \
OBJCOPY=$(pwd)/clang/bin/llvm-objcopy \
OBJDUMP=$(pwd)/clang/bin/llvm-objdump \
STRIP=$(pwd)/clang/bin/llvm-strip \
CROSS_COMPILE_ARM32=$(pwd)/gcc-linaro-13.0.0-2022.10-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf- \
CROSS_COMPILE=$(pwd)/gcc-linaro-13.0.0-2022.10-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu- \
O=out ARCH=arm64 -j$(($(nproc)+1)) $@

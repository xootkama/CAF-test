#!/usr/bin/bash 

if [[ -d "out" ]]
then 
cd out && make clean && make distclean && make mrproper && cd  ..
else 
  mkdir -p out
  fi
  
  make O=out ARCH=arm64 realme_sdm710_defconfig
sattire=`find / -type d -name "g" 2>/dev/null`

PATH="$sattire/bin:$PATH" \
 
  make                O=out \
                      ARCH=arm64 \
                      CC=clang \
                      CROSS_COMPILE=aarch64-linux-gnu- \
                      CROSS_COMPILE_ARM32=arm-linux-gnueabi- \
                      -j8
                      

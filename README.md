# cgminer-hash

make distclean

rm compat/jansson-2.9/jansson_private_config.h.in
rm config.h.in

./autogen.sh

//default: T2 hash, simplex mode spi
./configure --enable-bitmine_A1 --without-curses --host=arm-linux-gnueabi --build=x86_64-pc-linux-gnu

make -j2

# cgminer-hash

make distclean

rm compat/jansson-2.9/jansson_private_config.h.in
rm config.h.in

./autogen.sh

//default: T2 hash, simplex mode spi
./configure --enable-bitmine_A1 --without-curses --host=arm-linux-gnueabi --build=x86_64-pc-linux-gnu
./configured --enable-bitmine_T2 --without-curses --host=arm-linux-gnueabi --build=x86_64-pc-linux-gnu CPPFLAGS=-I./compat/jansson-2.9/src

make -j2

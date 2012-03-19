all: libpointmatcher

libpointmatcher:
	mkdir -p build
	cd build && cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -DNABO_INCLUDE_DIR=`rospack cflags-only-I libnabo` -DNABO_LIBRARY=`rospack libs-only-l libnabo` -DCMAKE_INSTALL_PREFIX=.. -DPYTHON_CUSTOM_TARGET=src ../upstream_src
	cd build && make
	cd build && make install
	touch libpointmatcher

clean:
	-rm -rf build libpointmatcher include lib share src
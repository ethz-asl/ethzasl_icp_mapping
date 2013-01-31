all: libpointmatcher

libpointmatcher: $(wildcard upstream_src/*/*.cpp) $(wildcard upstream_src/*/*.h)
	mkdir -p build
	cd build && cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_FLAGS="-fPIC" -DNABO_INCLUDE_DIR=`rospack cflags-only-I libnabo` -DCMAKE_LIBRARY_PATH=`rospack libs-only-L libnabo` -DCMAKE_INSTALL_PREFIX=.. -DPYTHON_CUSTOM_TARGET=src ../upstream_src
	cd build && make
	cd build && make install
	touch libpointmatcher

clean:
	-rm -rf build libpointmatcher include lib share src

all: libnabo

libnabo: $(wildcard upstream_src/*/*.cpp) $(wildcard upstream_src/*/*.h)
	mkdir -p build
	cd build && cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_FLAGS="-fPIC" -DCMAKE_INSTALL_PREFIX=.. -DPYTHON_CUSTOM_TARGET=src ../upstream_src
	cd build && make
	cd build && make install
	touch libnabo

clean:
	-rm -rf build libnabo include lib share src

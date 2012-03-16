all: libnabo

libnabo:
	mkdir -p build
	cd build && cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX=.. ../src 
	cd build && make
	cd build && make install
	touch libnabo

clean:
	-rm -rf build libnabo include lib share src
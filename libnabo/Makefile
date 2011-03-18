all: libnabo

FILENAME    = libnabo-git-4a43d8ee6d72070c3e91.tar.gz
TARBALL     = build/$(FILENAME)
TARBALL_URL = https://code.ros.org/svn/release/download/thirdparty/$(FILENAME)
SOURCE_DIR  = build/libnabo
UNPACK_CMD  = tar xzf
MD5SUM_FILE = $(FILENAME).md5sum

include $(shell rospack find mk)/download_unpack_build.mk

EIGEN_INCLUDE_DIR = $(shell rospack find eigen)/include

libnabo: installed

installed: $(SOURCE_DIR)/unpacked
	mkdir -p $(SOURCE_DIR)/build
	cd $(SOURCE_DIR)/build && cmake -DEIGEN_INCLUDE_DIR=${EIGEN_INCLUDE_DIR} -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX=../.. ..
	cd $(SOURCE_DIR)/build && make install
	touch installed

clean:
	rm -rf $(SOURCE_DIR) installed

wipe: clean
	rm -rf build *~

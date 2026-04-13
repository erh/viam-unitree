.PHONY: all build-wrapper build-go clean module

BUILD_DIR := build
BIN_DIR := bin

ARCH := $(shell uname -m)
SDK_LIB := $(BUILD_DIR)/_deps/unitree_sdk2-src/lib/$(ARCH)
DDS_LIB := $(BUILD_DIR)/_deps/unitree_sdk2-src/thirdparty/lib/$(ARCH)

all: module

# Build the C++ wrapper library and unitree_sdk2
build-wrapper:
	mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && cmake .. -DCMAKE_BUILD_TYPE=Release
	cd $(BUILD_DIR) && cmake --build . -j$$(nproc)

# Build the Go binary (requires wrapper to be built first)
build-go: build-wrapper
	mkdir -p $(BIN_DIR)
	CGO_ENABLED=1 \
	CGO_CFLAGS="-I$(CURDIR)" \
	CGO_LDFLAGS="-L$(CURDIR)/$(BUILD_DIR) -lunitree_capi \
		-L$(CURDIR)/$(SDK_LIB) -lunitree_sdk2 \
		-L$(CURDIR)/$(DDS_LIB) -lddsc -lddscxx \
		-lstdc++ -lm -lpthread \
		-Wl,-rpath,\$$ORIGIN/lib" \
	go build -o $(BIN_DIR)/viam-unitree .

# Package as module tarball (includes DDS shared libs)
module: build-go
	mkdir -p $(BIN_DIR)/lib
	cp -a $(DDS_LIB)/libddsc.so* $(BIN_DIR)/lib/
	cp -a $(DDS_LIB)/libddscxx.so* $(BIN_DIR)/lib/
	cd $(BIN_DIR) && tar -czf ../$(BUILD_DIR)/module.tar.gz viam-unitree lib/
	cp meta.json $(BUILD_DIR)/

clean:
	rm -rf $(BUILD_DIR) $(BIN_DIR)

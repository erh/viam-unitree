.PHONY: all build-wrapper build-go clean module

BUILD_DIR := build
BIN_DIR := bin

all: module

# Build the C/C++ DDS + Livox wrappers and their dependencies
build-wrapper:
	mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5
	cd $(BUILD_DIR) && cmake --build . -j$${NPROC:-$$(command -v nproc >/dev/null && nproc || sysctl -n hw.ncpu)}

# Build the Go binary (requires wrapper to be built first)
build-go: build-wrapper
	mkdir -p $(BIN_DIR)
	CGO_ENABLED=1 \
	CGO_CFLAGS="-I$(CURDIR)/capi \
		-I$(CURDIR)/$(BUILD_DIR)/_deps/cyclonedds-src/src/core/ddsc/include \
		-I$(CURDIR)/$(BUILD_DIR)/_deps/cyclonedds-src/src/ddsrt/include \
		-I$(CURDIR)/$(BUILD_DIR)/_deps/cyclonedds-build/src/core/include \
		-I$(CURDIR)/$(BUILD_DIR)/_deps/cyclonedds-build/src/ddsrt/include \
		-I$(CURDIR)/$(BUILD_DIR)/_deps/livox_sdk2-src/include" \
	CGO_CXXFLAGS="-std=c++11 -I$(CURDIR)/capi -I$(CURDIR)/$(BUILD_DIR)/_deps/livox_sdk2-src/include" \
	CGO_LDFLAGS="-L$(CURDIR)/$(BUILD_DIR) -L$(CURDIR)/$(BUILD_DIR)/_deps/livox_sdk2-build/sdk_core \
		-llivox_mid360 -llivox_lidar_sdk_static -ldds_unitree \
		-L$(CURDIR)/$(BUILD_DIR)/lib -lddsc \
		-lstdc++ -lm -lpthread \
		-Wl,-rpath,\$$ORIGIN/lib" \
	go build -o $(BIN_DIR)/viam-unitree .

# Package as module tarball
module: build-go
	mkdir -p $(BIN_DIR)/lib
	cp -a $(BUILD_DIR)/lib/libddsc.so* $(BIN_DIR)/lib/
	cd $(BIN_DIR) && tar -czf ../$(BUILD_DIR)/module.tar.gz viam-unitree lib/
	cp meta.json $(BUILD_DIR)/

clean:
	rm -rf $(BUILD_DIR) $(BIN_DIR)

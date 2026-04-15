.PHONY: all build-wrapper build-go clean module

BUILD_DIR := build
BIN_DIR := bin

all: module

# Build the C DDS wrapper and CycloneDDS
build-wrapper:
	mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && cmake .. -DCMAKE_BUILD_TYPE=Release
	cd $(BUILD_DIR) && cmake --build . -j$$(nproc)

# Build the Go binary (requires wrapper to be built first)
build-go: build-wrapper
	mkdir -p $(BIN_DIR)
	CGO_ENABLED=1 \
	CGO_CFLAGS="-I$(CURDIR)/capi -I$(CURDIR)/$(BUILD_DIR)/_deps/cyclonedds-src/src/core/ddsc/include -I$(CURDIR)/$(BUILD_DIR)/_deps/cyclonedds-src/src/ddsrt/include -I$(CURDIR)/$(BUILD_DIR)/_deps/cyclonedds-build/src/core/include -I$(CURDIR)/$(BUILD_DIR)/_deps/cyclonedds-build/src/ddsrt/include" \
	CGO_LDFLAGS="-L$(CURDIR)/$(BUILD_DIR) -ldds_unitree \
		-L$(CURDIR)/$(BUILD_DIR)/lib -lddsc \
		-lm -lpthread \
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

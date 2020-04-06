#!bin/bash

ESP32CS_TARGET=$1

RUN_DIR=$PWD
export IDF_PATH=${RUN_DIR}/esp-idf
TOOLCHAIN_DIR=${RUN_DIR}/toolchain
BUILD_DIR=${RUN_DIR}/build
BINARIES_DIR=${RUN_DIR}/binaries

if [ -d "${GITHUB_WORKSPACE}/esp-idf" ]; then
    export IDF_PATH=${GITHUB_WORKSPACE}/esp-idf
fi

# export IDF_PATH=~/esp-idf
# TOOLCHAIN_DIR=~/esp/crosstool-NG/builds/xtensa-esp32-elf
# BUILD_DIR=${RUN_DIR}/build-test

if [ -d ${BUILD_DIR} ]; then
    echo "Cleaning up ${BUILD_DIR}"
    rm -rf ${BUILD_DIR}
fi

mkdir -p ${BUILD_DIR} ${BUILD_DIR}/config

# install GCC 8.2.0 toolchain
if [ ! -f ${TOOLCHAIN_DIR}/bin/xtensa-esp32-elf-gcc ]; then
    echo "Toolchain not found, downloading..."
    curl -k https://dl.espressif.com/dl/xtensa-esp32-elf-gcc8_2_0-esp-2019r2-linux-amd64.tar.gz \
        -o ${RUN_DIR}/xtensa-esp32-elf-gcc8_2_0-esp-2019r2-linux-amd64.tar.gz
    if [ $? -ne 0 ]; then
        echo "Download failed!"
        exit 1
    fi
    mkdir -p ${TOOLCHAIN_DIR}
    echo "Installing toolchain..."
    tar zxf ${RUN_DIR}/xtensa-esp32-elf-gcc8_2_0-esp-2019r2-linux-amd64.tar.gz -C ${TOOLCHAIN_DIR}
    if [ $? -ne 0 ]; then
        echo "Install failed!"
        exit 1
    fi
fi
echo "Adding ${TOOLCHAIN_DIR}/xtensa-esp32-elf/bin to the path"
# add toolchain to the path
export PATH=${TOOLCHAIN_DIR}/xtensa-esp32-elf/bin:${PATH}

# clone ESP-IDF
if [ ! -d ${IDF_PATH} ]; then
    echo "ESP-IDF not available, cloning..."
    git clone -b release/v4.0 --recurse-submodules https://github.com/espressif/esp-idf.git -j4 ${IDF_PATH}
    if [ $? -ne 0 ]; then
        echo "Git clone failed!"
        exit 1
    fi
fi
cd ${IDF_PATH} && python -m pip install -r requirements.txt

# generate config.env file for confgen.py and cmake
echo "Generating config.env"
cat > ${BUILD_DIR}/config.env <<CONFIG_ENV_EOF
{
    "COMPONENT_KCONFIGS": "$(find ${IDF_PATH}/components -name Kconfig -printf '%p ')",
    "COMPONENT_KCONFIGS_PROJBUILD": "${RUN_DIR}/main/Kconfig.projbuild $(find ${IDF_PATH} -name Kconfig.profjbuild -printf '%p ')",
    "COMPONENT_SDKCONFIG_RENAMES": "$(find ${IDF_PATH}/components -name sdkconfig.rename -printf '%p ')",
    "IDF_CMAKE": "y",
    "IDF_TARGET": "esp32",
    "IDF_PATH": "${IDF_PATH}"
}
CONFIG_ENV_EOF

# create default sdkconfig
export IDF_TARGET=esp32
SDKCONFIG_DEFAULTS="${RUN_DIR}/sdkconfig.defaults"
if [ "${ESP32CS_TARGET}" == "ESP32CommandStation.pcb" ]; then
    SDKCONFIG_DEFAULTS="${RUN_DIR}/sdkconfig.defaults.pcb"
fi
echo "Generating default sdkconfig"
python ${IDF_PATH}/tools/kconfig_new/confgen.py \
    --kconfig ${IDF_PATH}/Kconfig \
    --config ${RUN_DIR}/sdkconfig \
    --sdkconfig-rename ${IDF_PATH}/sdkconfig.rename \
    --defaults "${SDKCONFIG_DEFAULTS}" \
    --env-file ${BUILD_DIR}/config.env \
    --output header ${BUILD_DIR}/config/sdkconfig.h \
    --output cmake ${BUILD_DIR}/config/sdkconfig.cmake \
    --output json ${BUILD_DIR}/config/sdkconfig.json \
    --output json_menus ${BUILD_DIR}/config/kconfig_menus.json \
    --output config ${RUN_DIR}/sdkconfig
if [ $? -ne 0 ]; then
    echo "sdkconfig generation failed!"
    exit 1
fi

# build via cmake/ninja
cd ${BUILD_DIR} && cmake ${RUN_DIR} -G Ninja && ninja

# print size information
python ${IDF_PATH}/tools/idf_size.py ${BUILD_DIR}/ESP32CommandStation.map

mkdir -p "${BINARIES_DIR}"
cat > "${BINARIES_DIR}"/readme.txt << README_EOF
The binaries can be sent to the ESP32 via esptool.py similar to the following:
python esptool.py -p (PORT) -b 460800 --before default_reset --after hard_reset write_flash 
    --flash_mode dio --flash_size detect --flash_freq 40m
    0x1000 bootloader.bin
    0x8000 partition-table.bin
    0xe000 ota_data_initial.bin
    0x10000 ESP32CommandStation.bin
README_EOF

cp "${BUILD_DIR}/partition_table/partition-table.bin" \
    "${BUILD_DIR}/ota_data_initial.bin" \
    "${BUILD_DIR}/bootloader/bootloader.bin" \
    "${BUILD_DIR}/ESP32CommandStation.bin" \
    "${BINARIES_DIR}"
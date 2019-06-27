/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2017-2019 Mike Dunston

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see http://www.gnu.org/licenses
**********************************************************************/

#include "ESP32CommandStation.h"

#include <sys/stat.h>

ConfigurationManager configStore;

StaticJsonBuffer<20480> jsonConfigBuffer;

#if !defined(CONFIG_USE_SPIFFS) && !defined(CONFIG_USE_SD)
#define CONFIG_USE_SPIFFS true
#endif

#if CONFIG_USE_SPIFFS
#include <esp_spiffs.h>
// All ESP32 Command Station configuration files live under this directory on
// the configured filesystem starting with v1.3.0.
static constexpr const char *ESP32CS_CONFIG_DIR = "/spiffs/ESP32CS";

// Prior to v1.3.0 this was the configuration location, it is retained here only
// to support migration of data from previous releases.
static constexpr const char *OLD_CONFIG_DIR = "/spiffs/DCCppESP32";

#elif COFNIG_USE_SD
#include <esp_vfs_fat.h>
#include <driver/sdmmc_host.h>
#include <driver/sdspi_host.h>
#include <sdmmc_cmd.h>
sdmmc_card_t *sdcard = nullptr;

// All ESP32 Command Station configuration files live under this directory on
// the configured filesystem starting with v1.3.0.
static constexpr const char *ESP32CS_CONFIG_DIR = "/sdcard/ESP32CS";

// Prior to v1.3.0 this was the configuration location, it is retained here only
// to support migration of data from previous releases.
static constexpr const char *OLD_CONFIG_DIR = "/sdcard/DCCppESP32";
#endif

ConfigurationManager::ConfigurationManager() {
}

ConfigurationManager::~ConfigurationManager() {
#if CONFIG_USE_SPIFFS
  if(esp_spiffs_mounted(NULL)) {
    ESP_ERROR_CHECK(esp_vfs_spiffs_unregister(NULL));
  }
#elif CONFIG_USE_SD
  if(sdcard) {
    esp_vfs_fat_sdmmc_unmount();
  }
#endif
}

void ConfigurationManager::init() {
#if CONFIG_USE_SPIFFS
  esp_vfs_spiffs_conf_t conf = {
    .base_path = "/spiffs",
    .partition_label = NULL,
    .max_files = 5,
    .format_if_mount_failed = true
  };
  // Attempt to mount the partition
  esp_err_t res = esp_vfs_spiffs_register(&conf);
  // check that the partition mounted
  if(res != ESP_OK) {
    LOG(FATAL,
        "[Config] Failed to mount SPIFFS partition, err %s (%d), giving up!",
        esp_err_to_name(res), res);
  }
  size_t total = 0, used = 0;
  res = esp_spiffs_info(NULL, &total, &used);
  if(res == ESP_OK) {
    LOG(INFO, "[Config] SPIFFS usage: %d/%d bytes", used, total);
  }
#elif COFNIG_USE_SD
  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  sdspi_slot_config_t slot_config = SDSPI_SLOT_CONFIG_DEFAULT();
  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
    .format_if_mount_failed = false,
    .max_files = 5,
    .allocation_unit_size = 16 * 1024
  };
  esp_err_t res = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &sdcard);
  if (res == ESP_FAIL) {
    LOG(FATAL, "[Config] Failed to mount the SD card. Is there an SD card inserted?");
  } else if(res != ESP_OK) {
    LOG(FATAL,
        "[Config] Failed to mount the SD card. %s (%d)",
        esp_err_to_name(res), res);
  }
  sdmmc_card_print_info(stdout, sdcard);
#endif
  mkdir(ESP32CS_CONFIG_DIR, ACCESSPERMS);
}

void ConfigurationManager::clear() {
  rmdir(ESP32CS_CONFIG_DIR);
  mkdir(ESP32CS_CONFIG_DIR, ACCESSPERMS);
}

bool ConfigurationManager::exists(const char *name) {
  std::string oldConfigFilePath = StringPrintf("%s/%s", OLD_CONFIG_DIR, name);
  std::string configFilePath = StringPrintf("%s/%s", ESP32CS_CONFIG_DIR, name);
  if(access(oldConfigFilePath.c_str(), F_OK) != -1 && !access(configFilePath.c_str(), F_OK)) {
    LOG(INFO,
        "[Config] Migrating configuration file %s to %s.",
        oldConfigFilePath.c_str(), configFilePath.c_str());
    rename(oldConfigFilePath.c_str(), configFilePath.c_str());
  }
  return !access(configFilePath.c_str(), F_OK);
}

void ConfigurationManager::remove(const char *name) {
  std::string configFilePath = StringPrintf("%s/%s", ESP32CS_CONFIG_DIR, name);
  unlink(configFilePath.c_str());
}

JsonObject &ConfigurationManager::load(const char *name) {
  std::string configFilePath = StringPrintf("%s/%s", ESP32CS_CONFIG_DIR, name);
  LOG(INFO, "[Config] Loading %s", configFilePath.c_str());
  std::string configFileContent = read_file_to_string(configFilePath.c_str());
  jsonConfigBuffer.clear();
  JsonObject &root = jsonConfigBuffer.parseObject(configFileContent);
  return root;
}

JsonObject &ConfigurationManager::load(const char *name, DynamicJsonBuffer &buffer) {
  std::string configFilePath = StringPrintf("%s/%s", ESP32CS_CONFIG_DIR, name);
  LOG(INFO, "[Config] Loading %s", configFilePath.c_str());
  std::string configFileContent = read_file_to_string(configFilePath.c_str());
  JsonObject &root = buffer.parseObject(configFileContent);
  return root;
}

void ConfigurationManager::store(const char *name, const JsonObject &json) {
  std::string configFilePath = StringPrintf("%s/%s", ESP32CS_CONFIG_DIR, name);
  LOG(INFO, "[Config] Storing %s", configFilePath.c_str());
  std::string configFileContent = "";
  json.printTo(configFileContent);
  write_string_to_file(configFilePath, configFileContent);
}

JsonObject &ConfigurationManager::createRootNode(bool clearBuffer) {
  if(clearBuffer) {
    jsonConfigBuffer.clear();
  }
  return jsonConfigBuffer.createObject();
}
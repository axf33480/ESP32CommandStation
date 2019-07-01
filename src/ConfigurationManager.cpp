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

#include <dirent.h>
#include <driver/sdmmc_defs.h>
#include <driver/sdmmc_host.h>
#include <driver/sdmmc_types.h>
#include <driver/sdspi_host.h>
#include <esp_spiffs.h>
#include <esp_vfs_fat.h>
#include <sdmmc_cmd.h>
#include <sys/stat.h>
#include <sys/types.h>
ConfigurationManager *configStore;

StaticJsonBuffer<20480> jsonConfigBuffer;

//#define CONFIG_USE_SD true

#define SPIFFS_FILESYSTEM_PREFIX "/spiffs"

// Handle for the SD card (if mounted)
sdmmc_card_t *sdcard = nullptr;

#if CONFIG_USE_SD
#define FILESYSTEM_PREFIX "/sdcard"
#else
// default to SPIFFS storage
#define FILESYSTEM_PREFIX SPIFFS_FILESYSTEM_PREFIX
#endif

// All ESP32 Command Station configuration files live under this directory on
// the configured filesystem starting with v1.3.0.
static const char* const ESP32CS_CONFIG_DIR = FILESYSTEM_PREFIX "/ESP32CS";

// Prior to v1.3.0 this was the configuration location, it is retained here only
// to support migration of data from previous releases.
static const char* const OLD_CONFIG_DIR = FILESYSTEM_PREFIX "/DCCppESP32";

void recursiveWalkTree(const std::string &path, bool remove=false) {
  LOG(VERBOSE, "[Config] Reading directory: %s", path.c_str());
  DIR *dir = opendir(path.c_str());
  if(dir) {
    dirent *ent = NULL;
    while((ent = readdir(dir)) != NULL) {
      std::string fullPath = path + "/" + ent->d_name;
      if(ent->d_type == DT_REG) {
        if(remove) {
          LOG(VERBOSE, "[Config] Removing: %s", fullPath.c_str());
          unlink(fullPath.c_str());
        } else {
          struct stat statbuf = {0};
          stat(fullPath.c_str(), &statbuf);
          LOG(VERBOSE, "[Config] %s (%d bytes)", fullPath.c_str(), (int)statbuf.st_size);
        }
      } else if(ent->d_type == DT_DIR) {
        recursiveWalkTree(fullPath, remove);
      }
    }
    closedir(dir);
    if(remove) {
      LOG(VERBOSE, "[Config] Removing directory: %s", path.c_str());
      rmdir(path.c_str());
    }
  } else {
    LOG_ERROR("[Config] Failed to open directory: %s", path.c_str());
  }
}

ConfigurationManager::ConfigurationManager() {
  esp_vfs_spiffs_conf_t conf = {
    .base_path = SPIFFS_FILESYSTEM_PREFIX,
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
  if(esp_spiffs_info(NULL, &total, &used) == ESP_OK) {
    LOG(INFO, "[Config] SPIFFS usage: %.2f/%.2f KiB", (float)(used / 1024.0f), (float)(total / 1024.0f));
  }
#if CONFIG_USE_SD
  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  sdspi_slot_config_t slot_config = SDSPI_SLOT_CONFIG_DEFAULT();
  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
    .format_if_mount_failed = true,
    .max_files = 5,
    .allocation_unit_size = 16 * 1024
  };
  ESP_ERROR_CHECK(
    esp_vfs_fat_sdmmc_mount(FILESYSTEM_PREFIX,
                            &host,
                            &slot_config,
                            &mount_config,
                            &sdcard));
  FATFS *fsinfo;
  DWORD clusters;
  if(f_getfree("0:", &clusters, &fsinfo) == FR_OK) {
    LOG(INFO, "[Config] SD usage: %.2f/%.2f MB",
        (float)(((uint64_t)fsinfo->csize * (fsinfo->n_fatent - 2 - fsinfo->free_clst)) * fsinfo->ssize) / 1048576,
        (float)(((uint64_t)fsinfo->csize * (fsinfo->n_fatent - 2)) * fsinfo->ssize) / 1048576);
  } else {
    LOG(INFO, "[Config] SD capacity %.2f MB",
        (float)(((uint64_t)sdcard->csd.capacity) * sdcard->csd.sector_size) / 1048576);
  }
#endif
  mkdir(ESP32CS_CONFIG_DIR, ACCESSPERMS);
  std::string configRoot = FILESYSTEM_PREFIX;
  recursiveWalkTree(configRoot);
}

ConfigurationManager::~ConfigurationManager() {
  // Unmount the SPIFFS partition
  if(esp_spiffs_mounted(NULL)) {
    LOG(INFO, "[Config] Unmounting SPIFFS...");
    ESP_ERROR_CHECK(esp_vfs_spiffs_unregister(NULL));
  }
  // Unmount the SD card if it was mounted
  if(sdcard) {
    LOG(INFO, "[Config] Unmounting SD...");
    esp_vfs_fat_sdmmc_unmount();
  }
}

void ConfigurationManager::clear() {
  std::string configRoot = ESP32CS_CONFIG_DIR;
  recursiveWalkTree(configRoot, true);
  mkdir(configRoot.c_str(), ACCESSPERMS);
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
  LOG(VERBOSE, "[Config] Checking for %s", configFilePath.c_str());
  struct stat statbuf = {0};
  if(stat(configFilePath.c_str(), &statbuf)) {
    return false;
  }
  return S_ISREG(statbuf.st_mode);
}

void ConfigurationManager::remove(const char *name) {
  std::string configFilePath = StringPrintf("%s/%s", ESP32CS_CONFIG_DIR, name);
  LOG(VERBOSE, "[Config] Removing %s", configFilePath.c_str());
  unlink(configFilePath.c_str());
}

JsonObject &ConfigurationManager::load(const char *name) {
  std::string configFilePath = StringPrintf("%s/%s", ESP32CS_CONFIG_DIR, name);
  LOG(VERBOSE, "[Config] Loading %s", configFilePath.c_str());
  std::string configFileContent = read_file_to_string(configFilePath.c_str());
  jsonConfigBuffer.clear();
  JsonObject &root = jsonConfigBuffer.parseObject(configFileContent);
  return root;
}

JsonObject &ConfigurationManager::load(const char *name, DynamicJsonBuffer &buffer) {
  std::string configFilePath = StringPrintf("%s/%s", ESP32CS_CONFIG_DIR, name);
  LOG(VERBOSE, "[Config] Loading %s", configFilePath.c_str());
  std::string configFileContent = read_file_to_string(configFilePath.c_str());
  JsonObject &root = buffer.parseObject(configFileContent);
  return root;
}

void ConfigurationManager::store(const char *name, const JsonObject &json) {
  std::string configFilePath = StringPrintf("%s/%s", ESP32CS_CONFIG_DIR, name);
  LOG(VERBOSE, "[Config] Storing %s", configFilePath.c_str());
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

openlcb::NodeID ConfigurationManager::getNodeId() {
  return UINT64_C(LCC_NODE_ID);
}

bool ConfigurationManager::needLCCCan(gpio_num_t *rxPin, gpio_num_t *txPin) {
#if LCC_CAN_RX_PIN != NOT_A_PIN && LCC_CAN_TX_PIN != NOT_A_PIN
  *rxPin = (gpio_num_t)LCC_CAN_RX_PIN;
  *txPin = (gpio_num_t)LCC_CAN_TX_PIN;
  return true;
#else
  return false;
#endif
}

void ConfigurationManager::configureWiFi(openlcb::SimpleCanStack *stack, const WiFiConfiguration &cfg) {
  wifi_mode_t wifiMode = WIFI_MODE_STA;
#if WIFI_ENABLE_SOFT_AP
  wifiMode =  WIFI_MODE_APSTA;
#endif


#if !defined(WIFI_STATIC_IP_ADDRESS) || !defined(WIFI_STATIC_IP_GATEWAY) || !defined(WIFI_STATIC_IP_SUBNET)
  tcpip_adapter_ip_info_t *stationStaticIP = nullptr;
  ip_addr_t stationDNSServer = ip_addr_any;
#else
  tcpip_adapter_ip_info_t _staticIP = {
      htonl(WIFI_STATIC_IP_ADDRESS),
      htonl(WIFI_STATIC_IP_SUBNET),
      htonl(WIFI_STATIC_IP_GATEWAY)
  };
  tcpip_adapter_ip_info_t *stationStaticIP = &_staticIP;
#ifdef WIFI_STATIC_IP_DNS
  ip_addr_t stationDNSServer = IPADDR4_INIT(htonl(WIFI_STATIC_IP_DNS));
#else
  ip_addr_t stationDNSServer = ip_addr_any;
#endif
#endif
  wifiManager.reset(new Esp32WiFiManager(SSID_NAME, SSID_PASSWORD,
                                         stack, cfg,
                                         HOSTNAME_PREFIX, wifiMode,
                                         stationStaticIP, stationDNSServer,
                                         WIFI_SOFT_AP_CHANNEL, WIFI_SOFT_AP_MAX_CLIENTS,
                                         WIFI_AUTH_OPEN));
}

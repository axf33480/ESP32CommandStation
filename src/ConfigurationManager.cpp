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
#include <fstream>

ConfigurationManager *configStore;

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

// Global handle for WiFi Manager
unique_ptr<Esp32WiFiManager> wifiManager;

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
          struct stat statbuf;
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

  if(exists(ESP32_CS_CONFIG_JSON)) {
    LOG(INFO, "[Config] Found existing CS config file.");
    JsonObject config = load(ESP32_CS_CONFIG_JSON);
    serializeJson(jsonBuffer, csConfig_);
    if(config.containsKey(JSON_WIFI_NODE)) {
      JsonObject wifiConfig = config.getMember(JSON_WIFI_NODE);
      std::string wifiMode = wifiConfig[JSON_WIFI_MODE_NODE];
      if(!wifiMode.compare(JSON_VALUE_WIFI_MODE_SOFTAP_ONLY)) {
        wifiMode_ =  WIFI_MODE_AP;
      } else if(!wifiMode.compare(JSON_VALUE_WIFI_MODE_SOFTAP_STATION)) {
        wifiMode_ =  WIFI_MODE_APSTA;
      } else if(!wifiMode.compare(JSON_VALUE_WIFI_MODE_STATION_ONLY)) {
        wifiMode_ =  WIFI_MODE_STA;
      }
      if(wifiConfig.containsKey(JSON_WIFI_STATION_NODE)) {
        JsonObject stationConfig = wifiConfig.getMember(JSON_WIFI_STATION_NODE);
        std::string stationMode = stationConfig[JSON_WIFI_MODE_NODE];
        if(!stationMode.compare(JSON_VALUE_STATION_IP_MODE_STATIC)) {
          stationStaticIP_.reset(new tcpip_adapter_ip_info_t());
          stationStaticIP_->ip.addr = ipaddr_addr(stationConfig[JSON_WIFI_STATION_IP_NODE]);
          stationStaticIP_->gw.addr = ipaddr_addr(stationConfig[JSON_WIFI_STATION_GATEWAY_NODE]);
          stationStaticIP_->netmask.addr = ipaddr_addr(stationConfig[JSON_WIFI_STATION_NETMASK_NODE]);
        }
      }
      if(wifiConfig.containsKey(JSON_WIFI_DNS_NODE)) {
        stationDNSServer_.u_addr.ip4.addr = ipaddr_addr(wifiConfig[JSON_WIFI_DNS_NODE]);
      }
    }
  } else {
    LOG(INFO, "[Config] CS Config not found, seeding defaults");
    JsonObject config = createRootNode();
    JsonObject wifiConfig = config.createNestedObject(JSON_WIFI_NODE);
    JsonObject stationConfig = wifiConfig.createNestedObject(JSON_WIFI_STATION_NODE);
#if WIFI_ENABLE_SOFT_AP
    wifiMode_ = WIFI_MODE_APSTA;
    wifiConfig[JSON_WIFI_MODE_NODE] = JSON_VALUE_WIFI_MODE_STATION_ONLY;
#else
    wifiConfig[JSON_WIFI_MODE_NODE] = JSON_VALUE_WIFI_MODE_STATION_ONLY;
#endif
#ifdef WIFI_STATIC_IP_DNS
    stationDNSServer_.u_addr.ip4.addr = ipaddr_addr(WIFI_STATIC_IP_DNS);
    wifiConfig[JSON_WIFI_DNS_NODE] = WIFI_STATIC_IP_DNS;
#endif
#if defined(WIFI_STATIC_IP_ADDRESS) && defined(WIFI_STATIC_IP_GATEWAY) && defined(WIFI_STATIC_IP_SUBNET)
    stationConfig[JSON_WIFI_MODE_NODE] = JSON_VALUE_STATION_IP_MODE_STATIC;
    stationConfig[JSON_WIFI_STATION_IP_NODE] = WIFI_STATIC_IP_ADDRESS;
    stationConfig[JSON_WIFI_STATION_GATEWAY_NODE] = WIFI_STATIC_IP_GATEWAY;
    stationConfig[JSON_WIFI_STATION_NETMASK_NODE] = WIFI_STATIC_IP_SUBNET;
    stationStaticIP_.reset(new tcpip_adapter_ip_info_t());
    stationStaticIP_->ip.addr = ipaddr_addr(WIFI_STATIC_IP_ADDRESS);
    stationStaticIP_->gw.addr = ipaddr_addr(WIFI_STATIC_IP_GATEWAY);
    stationStaticIP_->netmask.addr = ipaddr_addr(WIFI_STATIC_IP_SUBNET);
#else
    stationConfig[JSON_WIFI_STATION_NODE] = JSON_VALUE_STATION_IP_MODE_DHCP;
#endif
    serializeJson(jsonBuffer, csConfig_);
    store(ESP32_CS_CONFIG_JSON, config);
  }
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
  LOG(INFO, "[Config] Clearing persistent config...");
  std::string configRoot = ESP32CS_CONFIG_DIR;
  recursiveWalkTree(configRoot, true);
  mkdir(configRoot.c_str(), ACCESSPERMS);
}

bool ConfigurationManager::exists(const char *name) {
  std::string oldConfigFilePath = getFilePath(name, true);
  std::string configFilePath = getFilePath(name);
  if(std::ifstream(oldConfigFilePath).good() && !std::ifstream(configFilePath).good()) {
    LOG(INFO, "[Config] Migrating configuration file %s to %s.",
        oldConfigFilePath.c_str(), configFilePath.c_str());
    rename(oldConfigFilePath.c_str(), configFilePath.c_str());
  }
  LOG(VERBOSE, "[Config] Checking for %s", configFilePath.c_str());
  return std::ifstream(configFilePath).good();
}

void ConfigurationManager::remove(const char *name) {
  std::string configFilePath = getFilePath(name);
  LOG(VERBOSE, "[Config] Removing %s", configFilePath.c_str());
  unlink(configFilePath.c_str());
}

JsonObject ConfigurationManager::load(const char *name) {
  std::string configFilePath = getFilePath(name);
  LOG(VERBOSE, "[Config] Loading %s", configFilePath.c_str());
  jsonBuffer.clear();
  std::ifstream configFile(configFilePath);
  if(configFile.good()) {
    deserializeJson(jsonBuffer, configFile);
  }
  return jsonBuffer.as<JsonObject>();
}

JsonObject ConfigurationManager::load(const char *name, DynamicJsonDocument &buffer) {
  std::string configFilePath = getFilePath(name);
  LOG(VERBOSE, "[Config] Loading %s", configFilePath.c_str());
  std::ifstream configFile(configFilePath);
  deserializeJson(buffer, configFile);
  return buffer.as<JsonObject>();
}

void ConfigurationManager::store(const char *name, const JsonObject json) {
  std::string configFilePath = getFilePath(name);
  LOG(VERBOSE, "[Config] Storing %s", configFilePath.c_str());
  std::ofstream configFile(configFilePath, std::ios::out | std::ios::trunc);
  serializeJson(jsonBuffer, configFile);
}

JsonObject ConfigurationManager::createRootNode(bool clearBuffer) {
  if(clearBuffer) {
    jsonBuffer.clear();
  }
  return jsonBuffer.as<JsonObject>();
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
  if(wifiMode_ == WIFI_MODE_AP) {
    LOG(INFO, "[Config] WiFi Mode: SoftAP");
  } else if(wifiMode_ == WIFI_MODE_APSTA) {
    LOG(INFO, "[Config] WiFi Mode: Station + SoftAP");
  } else if(wifiMode_ == WIFI_MODE_STA) {
    LOG(INFO, "[Config] WiFi Mode: Station");
  }
  if(stationStaticIP_.get()) {
    LOG(INFO, "[Config] WiFi Station IP-MODE: STATIC IP: " IPSTR ", GW: " IPSTR ", SN: " IPSTR,
        IP2STR(&stationStaticIP_->ip), IP2STR(&stationStaticIP_->gw), IP2STR(&stationStaticIP_->netmask));
  } else {
    LOG(INFO, "[Config] WiFi Station IP-MODE: DHCP");
  }

  if(stationDNSServer_.u_addr.ip4.addr != ip_addr_any.u_addr.ip4.addr) {
    LOG(INFO, "[Config] WiFi Station DNS: " IPSTR, IP2STR(&stationDNSServer_.u_addr.ip4));
  }

  wifiManager.reset(new Esp32WiFiManager(wifiSSID_.c_str(),
                                         wifiPassword_.c_str(),
                                         stack, cfg,
                                         HOSTNAME_PREFIX, wifiMode_,
                                         stationStaticIP_.get(), stationDNSServer_,
                                         WIFI_SOFT_AP_CHANNEL, WIFI_SOFT_AP_MAX_CLIENTS,
                                         WIFI_AUTH_OPEN));
}

std::string ConfigurationManager::getFilePath(const char *name, bool oldPath) {
  if(oldPath) {
    return StringPrintf("%s/%s", OLD_CONFIG_DIR, name);
  }
  return StringPrintf("%s/%s", ESP32CS_CONFIG_DIR, name);
}

std::string ConfigurationManager::getCSConfig() {
  return csConfig_;
}
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

using nlohmann::json;

unique_ptr<ConfigurationManager> configStore;

static constexpr const char * ESP32_CS_CONFIG_JSON = "esp32cs-config.json";

// Handle for the SD card (if mounted)
sdmmc_card_t *sdcard = nullptr;

// All ESP32 Command Station configuration files live under this directory on
// the configured filesystem starting with v1.3.0.
static const char* const ESP32CS_CONFIG_DIR = CS_CONFIG_FILESYSTEM "/ESP32CS";

// Prior to v1.3.0 this was the configuration location, it is retained here only
// to support migration of data from previous releases.
static const char* const OLD_CONFIG_DIR = CS_CONFIG_FILESYSTEM "/DCCppESP32";

// Global handle for WiFi Manager
unique_ptr<Esp32WiFiManager> wifiManager;

// holder of the parsed command station configuration.
json commandStationConfig;

void recursiveWalkTree(const string &path, bool remove=false)
{
  DIR *dir = opendir(path.c_str());
  if (dir)
  {
    dirent *ent = NULL;
    while ((ent = readdir(dir)) != NULL)
    {
      string fullPath = path + "/" + ent->d_name;
      if (ent->d_type == DT_REG)
      {
        struct stat statbuf;
        stat(fullPath.c_str(), &statbuf);
        if (remove)
        {
          LOG(VERBOSE, "[Config] Deleting %s (%lu bytes)", fullPath.c_str()
            , statbuf.st_size);
          ERRNOCHECK(fullPath.c_str(), unlink(fullPath.c_str()));
        }
        else
        {
          LOG(VERBOSE, "[Config] %s (%lu bytes)", fullPath.c_str()
            , statbuf.st_size);
        }
      }
      else if (ent->d_type == DT_DIR)
      {
        recursiveWalkTree(fullPath, remove);
      }
    }
    closedir(dir);
    if (remove)
    {
      rmdir(path.c_str());
    }
  }
  else
  {
    LOG_ERROR("[Config] Failed to open directory: %s", path.c_str());
  }
}

ConfigurationManager::ConfigurationManager()
{
  bool factory_reset_config{config_cs_force_factory_reset() == CONSTANT_TRUE};
  bool lcc_factory_reset{config_lcc_force_factory_reset() == CONSTANT_TRUE};
  bool persist_config{true};
  struct stat statbuf;

  esp_vfs_spiffs_conf_t conf =
  {
    .base_path = "/spiffs",
    .partition_label = NULL,
    .max_files = 5,
    .format_if_mount_failed = true
  };
  // Attempt to mount the partition
  esp_err_t res = esp_vfs_spiffs_register(&conf);
  // check that the partition mounted
  if (res != ESP_OK)
  {
    LOG(FATAL
      , "[Config] Failed to mount SPIFFS partition, err %s (%d), giving up!"
      , esp_err_to_name(res), res);
  }
  size_t total = 0, used = 0;
  if (esp_spiffs_info(NULL, &total, &used) == ESP_OK)
  {
    LOG(INFO, "[Config] SPIFFS usage: %.2f/%.2f KiB", (float)(used / 1024.0f)
      , (float)(total / 1024.0f));
  }

#if CONFIG_USE_SD
  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  sdspi_slot_config_t slot_config = SDSPI_SLOT_CONFIG_DEFAULT();
  esp_vfs_fat_sdmmc_mount_config_t mount_config =
  {
    .format_if_mount_failed = true,
    .max_files = 5,
    .allocation_unit_size = 16 * 1024
  };
  ESP_ERROR_CHECK(
    esp_vfs_fat_sdmmc_mount("/sdcard",
                            &host,
                            &slot_config,
                            &mount_config,
                            &sdcard));
  FATFS *fsinfo;
  DWORD clusters;
  if (f_getfree("0:", &clusters, &fsinfo) == FR_OK)
  {
    LOG(INFO, "[Config] SD usage: %.2f/%.2f MB",
        (float)(((uint64_t)fsinfo->csize * (fsinfo->n_fatent - 2 - fsinfo->free_clst)) * fsinfo->ssize) / 1048576L,
        (float)(((uint64_t)fsinfo->csize * (fsinfo->n_fatent - 2)) * fsinfo->ssize) / 1048576L);
  }
  else
  {
    LOG(INFO, "[Config] SD capacity %.2f MB",
        (float)(((uint64_t)sdcard->csd.capacity) * sdcard->csd.sector_size) / 1048576);
  }
#endif

  if (factory_reset_config)
  {
    LOG(WARNING, "!!!! WARNING WARNING WARNING WARNING WARNING !!!!");
    LOG(WARNING, "!!!! WARNING WARNING WARNING WARNING WARNING !!!!");
    LOG(WARNING, "!!!! WARNING WARNING WARNING WARNING WARNING !!!!");
    LOG(WARNING,
        "[Config] The factory reset flag has been set to true, all persistent "
        "data will be cleared.");
    uint8_t countdown = 10;
    while (--countdown)
    {
      LOG(WARNING, "[Config] Factory reset will be initiated in %d seconds..."
        , countdown);
      usleep(SEC_TO_USEC(1));
    }
    LOG(WARNING, "[Config] Factory reset starting!");
  }

  LOG(VERBOSE, "[Config] Persistent storage contents:");
  recursiveWalkTree(CS_CONFIG_FILESYSTEM, factory_reset_config);
  // Pre-create ESP32 CS configuration directory.
  mkdir(ESP32CS_CONFIG_DIR, ACCESSPERMS);
  // Pre-create LCC configuration directory.
  mkdir(LCC_PERSISTENT_CONFIG_DIR, ACCESSPERMS);

  if (exists(ESP32_CS_CONFIG_JSON))
  {
    LOG(INFO, "[Config] Found existing CS config file, attempting to load...");
    commandStationConfig = json::parse(load(ESP32_CS_CONFIG_JSON));
    if (validateConfiguration())
    {
      LOG(INFO
        , "[Config] Existing configuration successfully loaded and validated.");
      factory_reset_config = false;
      // Check for any missing default configuration settings.
      persist_config = seedDefaultConfigSections();
      // Check if we need to force a reset of the LCC config (node id change)
      auto lccConfig = commandStationConfig[JSON_LCC_NODE];
      if (lccConfig.contains(JSON_LCC_FORCE_RESET_NODE) &&
          lccConfig[JSON_LCC_FORCE_RESET_NODE].get<bool>())
      {
        LOG(WARNING, "[Config] LCC force factory_reset flag is ENABLED!");
        commandStationConfig[JSON_LCC_NODE][JSON_LCC_FORCE_RESET_NODE] = false;
        persist_config = true;
        lcc_factory_reset = true;
      }
    }
    else
    {
      LOG_ERROR("[Config] Existing configuration failed one (or more) "
                "validation(s) and will be regenerated!");
      LOG_ERROR("[Config] Old config: %s", commandStationConfig.dump().c_str());
    }
  }
  else
  {
    factory_reset_config = true;
  }

  if (factory_reset_config)
  {
    LOG(INFO, "[Config] Generating default configuration...");
    // TODO: break this up so it starts with an empty config and makes calls to
    // various "getDefaultConfigXXX" for each section.
    commandStationConfig = {};
    persist_config = seedDefaultConfigSections();
    // force factory reset of LCC data
    lcc_factory_reset = true;
  }

  // if we need to persist the updated config do so now.
  if (persist_config)
  {
    LOG(INFO, "[Config] Persisting configuration settings");
    store(ESP32_CS_CONFIG_JSON, commandStationConfig.dump());
  }
  LOG(VERBOSE, "[Config] %s", commandStationConfig.dump().c_str());

  // If we are not currently forcing a factory reset, verify if the LCC config
  // file is the correct size. If it is not the expected size force a factory
  // reset.
  if (!lcc_factory_reset &&
      stat(LCC_NODE_CONFIG_FILE, &statbuf) >= 0 &&
      statbuf.st_size != openlcb::CONFIG_FILE_SIZE)
  {
    LOG(WARNING
      , "[LCC] Corrupt configuration file detected, %s is too small: %lu "
        "bytes, expected: %zu bytes"
      , LCC_NODE_CONFIG_FILE, statbuf.st_size, openlcb::CONFIG_FILE_SIZE);
    lcc_factory_reset = true;
  }

  // If we need an LCC factory reset trigger is now.
  if (lcc_factory_reset)
  {
    configStore->factory_reset_lcc(false);
  }
}

ConfigurationManager::~ConfigurationManager()
{
  extern std::unique_ptr<OpenMRN> openmrn;

  // Shutdown the auto-sync handler if it is running before unmounting the FS.
  if (configAutoSync_.get())
  {
    LOG(INFO, "[Config] Disabling automatic fsync(%d) calls...", configFd_);
    SyncNotifiable n;
    configAutoSync_->shutdown(&n);
    n.wait_for_notification();
    configAutoSync_.reset(nullptr);
  }

  // shutdown the executor so that no more tasks will run
  openmrn->stack()->executor()->shutdown();

  // close the config file if it is open
  if (configFd_ >= 0)
  {
    ::close(configFd_);
  }

  // Unmount the SPIFFS partition
  if (esp_spiffs_mounted(NULL))
  {
    LOG(INFO, "[Config] Unmounting SPIFFS...");
    ESP_ERROR_CHECK(esp_vfs_spiffs_unregister(NULL));
  }

  // Unmount the SD card if it was mounted
  if (sdcard)
  {
    LOG(INFO, "[Config] Unmounting SD...");
    esp_vfs_fat_sdmmc_unmount();
  }
}

void ConfigurationManager::clear()
{
  LOG(INFO, "[Config] Clearing persistent config...");
  string configRoot = ESP32CS_CONFIG_DIR;
  recursiveWalkTree(configRoot, true);
  mkdir(configRoot.c_str(), ACCESSPERMS);
}

bool ConfigurationManager::exists(const string &name)
{
  string oldConfigFilePath = getFilePath(name, true);
  string configFilePath = getFilePath(name);
  if (!access(oldConfigFilePath.c_str(), F_OK) &&
      access(configFilePath.c_str(), F_OK))
  {
    LOG(INFO, "[Config] Migrating configuration file %s to %s."
      , oldConfigFilePath.c_str()
      , configFilePath.c_str());
    rename(oldConfigFilePath.c_str(), configFilePath.c_str());
  }
  LOG(VERBOSE, "[Config] Checking for %s", configFilePath.c_str());
  return access(configFilePath.c_str(), F_OK) == 0;
}

void ConfigurationManager::remove(const string &name)
{
  string configFilePath = getFilePath(name);
  LOG(VERBOSE, "[Config] Removing %s", configFilePath.c_str());
  unlink(configFilePath.c_str());
}

string ConfigurationManager::load(const string &name)
{
  string configFilePath = getFilePath(name);
  if (!exists(name))
  {
    LOG(VERBOSE, "[Config] Failed to load: %s", configFilePath.c_str());
    return "{}";
  }
  LOG(VERBOSE, "[Config] Loading %s", configFilePath.c_str());
  return read_file_to_string(configFilePath);
}

void ConfigurationManager::store(const char *name, const string &content)
{
  string configFilePath = getFilePath(name);
  LOG(VERBOSE, "[Config] Storing %s, %d bytes", configFilePath.c_str()
    , content.length());
  write_string_to_file(configFilePath, content);
}

void ConfigurationManager::factory_reset_lcc(bool warn)
{
  if (!access(LCC_NODE_CONFIG_FILE, F_OK) ||
      !access(LCC_NODE_CDI_FILE, F_OK))
  {
    if (warn)
    {
      LOG(WARNING, "[Config] LCC factory reset underway...");
    }

    if (!access(LCC_NODE_CONFIG_FILE, F_OK))
    {
      LOG(WARNING, "[Config] Removing %s", LCC_NODE_CONFIG_FILE);
      ERRNOCHECK(LCC_NODE_CONFIG_FILE, unlink(LCC_NODE_CONFIG_FILE));
    }
    if (!access(LCC_NODE_CDI_FILE, F_OK))
    {
      LOG(WARNING, "[Config] Removing %s", LCC_NODE_CDI_FILE);
      ERRNOCHECK(LCC_NODE_CDI_FILE, unlink(LCC_NODE_CDI_FILE));
    }
    if (warn)
    {
      LOG(WARNING
        , "[Config] The ESP32 CommandStation needs to be restarted in order "
          "to complete the factory reset.");
    }
  }
}

NodeID ConfigurationManager::getNodeId()
{
  auto lccConfig = commandStationConfig[JSON_LCC_NODE];
  return (NodeID)lccConfig[JSON_LCC_NODE_ID_NODE].get<uint64_t>();
}

bool ConfigurationManager::setNodeID(string value)
{
  LOG(INFO, "[Config] Current NodeID: %s, updated NodeID: %s"
    , uint64_to_string_hex(getNodeId()).c_str(), value.c_str());
  // remove period characters if present
  value.erase(std::remove(value.begin(), value.end(), '.'), value.end());
  // convert the string to a uint64_t value
  uint64_t new_node_id{0};
  std::stringstream stream;
  stream << std::hex << value;
  stream >> new_node_id;
  if (new_node_id != getNodeId())
  {
    LOG(INFO
      , "[Config] Persisting NodeID: %s and enabling forced factory_reset."
      , uint64_to_string_hex(new_node_id).c_str());
    commandStationConfig[JSON_LCC_NODE][JSON_LCC_NODE_ID_NODE] = new_node_id;
    commandStationConfig[JSON_LCC_NODE][JSON_LCC_FORCE_RESET_NODE] = true;
    store(ESP32_CS_CONFIG_JSON, commandStationConfig.dump());
    return true;
  }
  return false;
}

void ConfigurationManager::configureLCC(OpenMRN *openmrn
                                      , const esp32cs::Esp32ConfigDef &cfg)
{
  auto lccConfig = commandStationConfig[JSON_LCC_NODE];
  if (lccConfig.contains(JSON_LCC_CAN_NODE))
  {
    auto canConfig = lccConfig[JSON_LCC_CAN_NODE];
    gpio_num_t canRXPin =
      (gpio_num_t)canConfig[JSON_LCC_CAN_RX_NODE].get<uint8_t>();
    gpio_num_t canTXPin =
      (gpio_num_t)canConfig[JSON_LCC_CAN_TX_NODE].get<uint8_t>();
    if (canRXPin < GPIO_NUM_MAX && canTXPin < GPIO_NUM_MAX)
    {
      LOG(INFO, "[Config] Enabling LCC CAN interface (rx: %d, tx: %d)"
        , canRXPin, canTXPin);
      openmrn->add_can_port(
        new Esp32HardwareCan("esp32can", canRXPin, canTXPin, false));
    }
  }

  // Create the CDI.xml dynamically if it doesn't already exist.
  openmrn->create_config_descriptor_xml(cfg, LCC_NODE_CDI_FILE);

  // Create the default internal configuration file if it doesn't already exist.
  configFd_ =
    openmrn->stack()->create_config_file_if_needed(cfg.seg().internal_config()
                                                 , ESP32CS_CDI_VERSION
                                                 , openlcb::CONFIG_FILE_SIZE);

#if CONFIG_USE_SD
  // ESP32 FFat library uses a 512b cache in memory by default for the SD VFS
  // adding a periodic fsync call for the LCC configuration file ensures that
  // config changes are saved since the LCC config file is less than 512b.
  LOG(INFO, "[Config] Creating automatic fsync(%d) calls every %dsec."
    , configFd_, config_lcc_sd_sync_interval_sec());
  configAutoSync_.reset(new AutoSyncFileFlow(openmrn->stack()->service()
                      , configFd_
                      , SEC_TO_USEC(config_lcc_sd_sync_interval_sec())));
#endif // CONFIG_USE_SD
}

void ConfigurationManager::setWiFiStationParams(string ssid, string password
                                              , string ip, string gateway
                                              , string subnet)
{
  string wifimode = commandStationConfig[JSON_WIFI_NODE][JSON_WIFI_MODE_NODE];
  if (wifimode.compare(JSON_VALUE_WIFI_MODE_SOFTAP_ONLY))
  {
    LOG(INFO, "[Config] Current config is SoftAP only, enabling Station mode");
    commandStationConfig[JSON_WIFI_NODE][JSON_WIFI_MODE_NODE] = JSON_VALUE_WIFI_MODE_SOFTAP_STATION;
  }
  LOG(INFO, "[Config] Reconfiguring Station SSID to '%s'", ssid.c_str());
  if (!ssid.empty())
  {
    commandStationConfig[JSON_WIFI_NODE][JSON_WIFI_STATION_NODE][JSON_WIFI_SSID_NODE] = ssid;
    commandStationConfig[JSON_WIFI_NODE][JSON_WIFI_STATION_NODE][JSON_WIFI_PASSWORD_NODE] = password;
  }
  if (ip.empty())
  {
    string ipmode = commandStationConfig[JSON_WIFI_NODE][JSON_WIFI_STATION_NODE][JSON_WIFI_MODE_NODE];
    if (ipmode.compare(JSON_VALUE_STATION_IP_MODE_DHCP))
    {
      LOG(INFO, "[Config] Reconfiguring Station IP mode to DHCP");
      commandStationConfig[JSON_WIFI_NODE][JSON_WIFI_STATION_NODE][JSON_WIFI_MODE_NODE] = JSON_VALUE_STATION_IP_MODE_DHCP;
      commandStationConfig[JSON_WIFI_NODE][JSON_WIFI_STATION_NODE][JSON_WIFI_STATION_IP_NODE] = "";
      commandStationConfig[JSON_WIFI_NODE][JSON_WIFI_STATION_NODE][JSON_WIFI_STATION_GATEWAY_NODE] = "";
      commandStationConfig[JSON_WIFI_NODE][JSON_WIFI_STATION_NODE][JSON_WIFI_STATION_NETMASK_NODE] = "";
    }
  }
  else
  {
    LOG(INFO
      , "[Config] Reconfiguring Station IP mode to STATIC using:\n"
        "ip: %s\n gateway: %s\nsubnet: %s"
      , ip.c_str(), gateway.c_str(), subnet.c_str());
    commandStationConfig[JSON_WIFI_NODE][JSON_WIFI_STATION_NODE][JSON_WIFI_MODE_NODE] = JSON_VALUE_STATION_IP_MODE_STATIC;
    commandStationConfig[JSON_WIFI_NODE][JSON_WIFI_STATION_NODE][JSON_WIFI_STATION_IP_NODE] = ip;
    commandStationConfig[JSON_WIFI_NODE][JSON_WIFI_STATION_NODE][JSON_WIFI_STATION_GATEWAY_NODE] = gateway;
    commandStationConfig[JSON_WIFI_NODE][JSON_WIFI_STATION_NODE][JSON_WIFI_STATION_NETMASK_NODE] = subnet;
  }
  // persist the new config
  store(ESP32_CS_CONFIG_JSON, commandStationConfig.dump());
}

string ConfigurationManager::getFilePath(const string &name, bool oldPath)
{
  if (oldPath)
  {
    return StringPrintf("%s/%s", OLD_CONFIG_DIR, name.c_str());
  }
  return StringPrintf("%s/%s", ESP32CS_CONFIG_DIR, name.c_str());
}

bool ConfigurationManager::validateConfiguration()
{
  if (!commandStationConfig.contains(JSON_WIFI_NODE))
  {
    LOG_ERROR("[Config] WiFi configuration not found.");
    return false;
  }
  else if (!commandStationConfig.contains(JSON_LCC_NODE))
  {
    LOG_ERROR("[Config] LCC configuration not found.");
    return false;
  }

  auto wifiNode = commandStationConfig[JSON_WIFI_NODE];
  LOG(VERBOSE, "[Config] WiFi config: %s", wifiNode.dump().c_str());

  // Verify that the wifi operating mode is one of the three supported
  // modes.
  string wifiMode = wifiNode[JSON_WIFI_MODE_NODE];
  if (wifiMode.compare(JSON_VALUE_WIFI_MODE_SOFTAP_ONLY) &&
      wifiMode.compare(JSON_VALUE_WIFI_MODE_SOFTAP_STATION) &&
      wifiMode.compare(JSON_VALUE_WIFI_MODE_STATION_ONLY))
  {
    LOG_ERROR("[Config] Unknown WiFi operating mode: %s!", wifiMode.c_str());
    return false;
  }

  // If we are not operating in AP only mode we should verify we have
  // an SSID.
  if (wifiMode.compare(JSON_VALUE_WIFI_MODE_SOFTAP_ONLY) &&
    (!wifiNode[JSON_WIFI_STATION_NODE].contains(JSON_WIFI_SSID_NODE) ||
     !wifiNode[JSON_WIFI_STATION_NODE].contains(JSON_WIFI_PASSWORD_NODE)))
  {
    LOG_ERROR("[Config] SSID/Password was not specified for Station mode!");
    return false;
  }

  // If we are operating in SoftAP only we require a default SSID name
  if (!wifiMode.compare(JSON_VALUE_WIFI_MODE_SOFTAP_ONLY) &&
      !wifiNode[JSON_WIFI_SOFTAP_NODE].contains(JSON_WIFI_SSID_NODE))
  {
    LOG_ERROR("[Config] SSID was not specified for SoftAP mode!");
    return false;
  }

  // verify LCC configuration
  auto lccNode = commandStationConfig[JSON_LCC_NODE];
  LOG(VERBOSE, "[Config] LCC config: %s", lccNode.dump().c_str());

  if (!lccNode.contains(JSON_LCC_NODE_ID_NODE) ||
      lccNode[JSON_LCC_NODE_ID_NODE].get<uint64_t>() == 0)
  {
    LOG_ERROR("[Config] Missing LCC node ID!");
    return false;
  }

  if (!lccNode.contains(JSON_LCC_CAN_NODE) ||
      !lccNode[JSON_LCC_CAN_NODE].contains(JSON_LCC_CAN_RX_NODE) ||
      !lccNode[JSON_LCC_CAN_NODE].contains(JSON_LCC_CAN_TX_NODE))
  {
    LOG_ERROR("[Config] LCC CAN configuration invalid.");
    return false;
  }

  return true;
}

bool ConfigurationManager::seedDefaultConfigSections()
{
  bool config_updated{false};

  // If LCC config is missing default it.
  if (!commandStationConfig.contains(JSON_LCC_NODE))
  {
    commandStationConfig[JSON_LCC_NODE] =
    {
      { JSON_LCC_NODE_ID_NODE, UINT64_C(LCC_NODE_ID) },
      { JSON_LCC_CAN_NODE,
        {
          { JSON_LCC_CAN_RX_NODE, LCC_CAN_RX_PIN },
          { JSON_LCC_CAN_TX_NODE, LCC_CAN_TX_PIN },
        }
      },
      { JSON_LCC_FORCE_RESET_NODE, false },
    };
    config_updated = true;
  }
  // If LCC config is missing default it.
  if (!commandStationConfig.contains(JSON_WIFI_NODE))
  {
    commandStationConfig[JSON_WIFI_NODE] =
    {
#if WIFI_ENABLE_SOFT_AP_ONLY
      { JSON_WIFI_MODE_NODE, JSON_VALUE_WIFI_MODE_SOFTAP_ONLY },
      { JSON_WIFI_SOFTAP_NODE,
        {
          { JSON_WIFI_SSID_NODE, wifiSSID_ },
/*
          { JSON_WIFI_PASSWORD_NODE, wifiPassword_ },
*/
        },
      },
#elif WIFI_ENABLE_SOFT_AP
      { JSON_WIFI_MODE_NODE, JSON_VALUE_WIFI_MODE_SOFTAP_STATION },
      { JSON_WIFI_SOFTAP_NODE,
        {
          { JSON_WIFI_SSID_NODE, wifiSSID_ },
        },
      },
#else
      { JSON_WIFI_MODE_NODE, JSON_VALUE_WIFI_MODE_STATION_ONLY },
#endif
#ifdef WIFI_STATIC_IP_DNS
      { JSON_WIFI_DNS_NODE, WIFI_STATIC_IP_DNS },
#endif
#if !WIFI_ENABLE_SOFT_AP_ONLY
      { JSON_WIFI_STATION_NODE, 
        {
#if defined(WIFI_STATIC_IP_ADDRESS) && defined(WIFI_STATIC_IP_GATEWAY) && defined(WIFI_STATIC_IP_SUBNET)
          { JSON_WIFI_MODE_NODE, JSON_VALUE_STATION_IP_MODE_STATIC },
          { JSON_WIFI_STATION_IP_NODE, WIFI_STATIC_IP_ADDRESS },
          { JSON_WIFI_STATION_GATEWAY_NODE, WIFI_STATIC_IP_GATEWAY },
          { JSON_WIFI_STATION_NETMASK_NODE, WIFI_STATIC_IP_SUBNET },
#else
          { JSON_WIFI_MODE_NODE, JSON_VALUE_STATION_IP_MODE_DHCP },
#endif
          { JSON_WIFI_SSID_NODE, wifiSSID_ },
          { JSON_WIFI_PASSWORD_NODE, wifiPassword_ },
        }
      },
#endif
    };
    config_updated = true;
  }
  // If HC12 config is missing default it.
  if (!commandStationConfig.contains(JSON_HC12_NODE))
  {
    commandStationConfig[JSON_HC12_NODE] =
    {
      { JSON_HC12_ENABLED_NODE, HC12_RADIO_ENABLED },
      { JSON_HC12_UART_NODE, HC12_UART_NUM },
      { JSON_HC12_RX_NODE, HC12_RX_PIN },
      { JSON_HC12_TX_NODE, HC12_TX_PIN },
    };
    // flag to indicate we need to persist the configuration update
    config_updated = true;
  }
  // If HBridge config is missing default it.
  if (!commandStationConfig.contains(JSON_HBRIDGES_NODE))
  {
    commandStationConfig[JSON_HBRIDGES_NODE] =
    {
      { "OPS",
        {
          { JSON_HBRIDGE_PREAMBLE_BITS_NODE, OPS_PREAMBLE_BITS },
          { JSON_HBRIDGE_RMT_CHANNEL_NODE, RMT_CHANNEL_0 },
          { JSON_HBRIDGE_ENABLE_PIN_NODE, OPS_ENABLE_PIN },
          { JSON_HBRIDGE_SIGNAL_PIN_NODE, OPS_SIGNAL_PIN },
          { JSON_HBRIDGE_THERMAL_PIN_NODE, OPS_THERMAL_PIN },
          { JSON_HBRIDGE_SENSE_PIN_NODE, OPS_CURRENT_SENSE_ADC },
          { JSON_RAILCOM_NODE, RAILCOM_ENABLE_PIN != NOT_A_PIN }
        },
      },
      { "PROG",
        {
          { JSON_HBRIDGE_PREAMBLE_BITS_NODE, PROG_PREAMBLE_BITS },
          { JSON_HBRIDGE_RMT_CHANNEL_NODE, RMT_CHANNEL_1 },
          { JSON_HBRIDGE_ENABLE_PIN_NODE, PROG_ENABLE_PIN },
          { JSON_HBRIDGE_SIGNAL_PIN_NODE, PROG_SIGNAL_PIN },
          { JSON_HBRIDGE_SENSE_PIN_NODE, PROG_CURRENT_SENSE_ADC },
        },
      }
    };
    // flag to indicate we need to persist the configuration update
    config_updated = true;
  }
  // If RailCom config is missing default it.
  if (!commandStationConfig.contains(JSON_RAILCOM_NODE))
  {
    commandStationConfig[JSON_RAILCOM_NODE] =
    {
      { JSON_RAILCOM_ENABLE_PIN_NODE, RAILCOM_ENABLE_PIN },
      { JSON_RAILCOM_BRAKE_PIN_NODE, RAILCOM_BRAKE_ENABLE_PIN },
      { JSON_RAILCOM_SHORT_PIN_NODE, RAILCOM_SHORT_PIN },
      { JSON_RAILCOM_UART_NODE, RAILCOM_UART },
      { JSON_RAILCOM_RX_NODE, RAILCOM_UART_RX_PIN },
    };
    // flag to indicate we need to persist the configuration update
    config_updated = true;
  }
  return config_updated;
}

void ConfigurationManager::parseWiFiConfig()
{
  auto wifiConfig = commandStationConfig[JSON_WIFI_NODE];
  string wifiMode = wifiConfig[JSON_WIFI_MODE_NODE];
  if (!wifiMode.compare(JSON_VALUE_WIFI_MODE_SOFTAP_ONLY))
  {
    wifiMode_ =  WIFI_MODE_AP;
  }
  else if (!wifiMode.compare(JSON_VALUE_WIFI_MODE_SOFTAP_STATION))
  {
    wifiMode_ =  WIFI_MODE_APSTA;
  }
  else if (!wifiMode.compare(JSON_VALUE_WIFI_MODE_STATION_ONLY))
  {
    wifiMode_ =  WIFI_MODE_STA;
  }
  if (wifiMode_ != WIFI_MODE_AP)
  {
    auto stationConfig = wifiConfig[JSON_WIFI_STATION_NODE];
    wifiSSID_ = stationConfig[JSON_WIFI_SSID_NODE];
    wifiPassword_ = stationConfig[JSON_WIFI_PASSWORD_NODE];
    string stationMode = stationConfig[JSON_WIFI_MODE_NODE];
    if (!stationMode.compare(JSON_VALUE_STATION_IP_MODE_STATIC))
    {
      stationStaticIP_.reset(new tcpip_adapter_ip_info_t());
      string value = stationConfig[JSON_WIFI_STATION_IP_NODE];
      stationStaticIP_->ip.addr = ipaddr_addr(value.c_str());
      value = stationConfig[JSON_WIFI_STATION_GATEWAY_NODE];
      stationStaticIP_->gw.addr = ipaddr_addr(value.c_str());
      value = stationConfig[JSON_WIFI_STATION_NETMASK_NODE];
      stationStaticIP_->netmask.addr = ipaddr_addr(value.c_str());
    }
  }
  else
  {
    wifiSSID_ = wifiConfig[JSON_WIFI_SOFTAP_NODE][JSON_WIFI_SSID_NODE];  
  }
  if (wifiConfig.contains(JSON_WIFI_DNS_NODE))
  {
    string value = wifiConfig[JSON_WIFI_DNS_NODE];
    stationDNSServer_.u_addr.ip4.addr = ipaddr_addr(value.c_str());
  }
}

void ConfigurationManager::configureEnabledModules(SimpleCanStack *stack
                                                 , const esp32cs::Esp32ConfigDef &cfg)
{
  parseWiFiConfig();
  wifiManager.reset(new Esp32WiFiManager(wifiSSID_.c_str()
                                       , wifiPassword_.c_str()
                                       , stack
                                       , cfg.seg().wifi()
                                       , HOSTNAME_PREFIX
                                       , wifiMode_
                                       , stationStaticIP_.get()
                                       , stationDNSServer_
                                       , WIFI_SOFT_AP_CHANNEL));

  // Initialize the turnout manager and register it with the LCC stack to
  // process accessories packets.
  turnoutManager.reset(new TurnoutManager(stack->node()));

  auto hc12Config = commandStationConfig[JSON_HC12_NODE];
  if (hc12Config[JSON_HC12_ENABLED_NODE].get<bool>())
  {
    LOG(INFO, "[Config] Enabling HC12 Radio");
    hc12_.emplace(stack->service()
                , (uart_port_t)hc12Config[JSON_HC12_UART_NODE].get<uint8_t>()
                , (gpio_num_t)hc12Config[JSON_HC12_RX_NODE].get<uint8_t>()
                , (gpio_num_t)hc12Config[JSON_HC12_TX_NODE].get<uint8_t>()
    );
  }
  ota_.emplace(stack->service());
  infoScreen_.emplace(stack);
  statusLED_.emplace(stack->service());
  
  // Task Monitor, periodically dumps runtime state to STDOUT.
  taskMon_.emplace(stack->service());

  wifiInterface.init();
  nextionInterfaceInit();

#if ENABLE_OUTPUTS
  LOG(INFO, "[Config] Enabling GPIO Outputs");
  OutputManager::init();
#endif

#if ENABLE_SENSORS
  LOG(INFO, "[Config] Enabling GPIO Inputs");
  SensorManager::init();
  S88BusManager::init();
  RemoteSensorManager::init();
#endif

#if LOCONET_ENABLED
  LOG(INFO, "[Config] Enabling LocoNet interface");
  initializeLocoNet();
#endif
}

string ConfigurationManager::getCSConfig()
{
  return commandStationConfig.dump();
}

string ConfigurationManager::getCSFeatures()
{
  json features = 
  {
    { JSON_S88_SENSOR_BASE_NODE, S88_FIRST_SENSOR }
  , { JSON_S88_NODE, S88_ENABLED && ENABLE_SENSORS ? JSON_VALUE_TRUE
                                                   : JSON_VALUE_FALSE }
  , { JSON_OUTPUTS_NODE, ENABLE_OUTPUTS ? JSON_VALUE_TRUE : JSON_VALUE_FALSE }
  , { JSON_SENSORS_NODE, ENABLE_SENSORS ? JSON_VALUE_TRUE : JSON_VALUE_FALSE }
  , { JSON_HC12_NODE
    , commandStationConfig[JSON_HC12_NODE][JSON_HC12_ENABLED_NODE].get<bool>() }
  };
  return features.dump();
}
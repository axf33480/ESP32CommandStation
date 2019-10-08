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

static constexpr const char ESP32_CS_CONFIG_JSON[] = "esp32cs-config.json";

// Global handle for WiFi Manager
unique_ptr<Esp32WiFiManager> wifiManager;

// holder of the parsed command station configuration.
json csConfig;

#ifdef DEDICATED_MODULE_EXECUTOR
// executor used for all modules to offload them from the core LCC excutor
static Executor<1> moduleExecutor{NO_THREAD()};
static Service moduleService(&moduleExecutor);
#endif

// CDI Helper which sets the provided path if it is different than the value
// passed in.
#define CDI_COMPARE_AND_SET(PATH, fd, value, updated) \
  {                                                   \
    auto current = PATH().read(fd);                   \
    if (current != value)                             \
    {                                                 \
      PATH().write(fd, value);                        \
      updated = true;                                 \
    }                                                 \
  }

// Helper which will trigger a config reload event to be queued when
// updated is true.
#define MAYBE_TRIGGER_UPDATE(updated)                             \
  if (updated)                                                    \
  {                                                               \
    extern unique_ptr<OpenMRN> openmrn;                           \
    openmrn->stack()->executor()->add(new CallbackExecutable([]() \
    {                                                             \
      openmrn->stack()->config_service()->trigger_update();       \
    }));                                                          \
  }

// Helper which will trigger a config reload event to be queued when
// updated is true.
#define SCHEDULE_REBOOT()                                       \
  extern unique_ptr<OpenMRN> openmrn;                           \
  openmrn->stack()->executor()->add(new CallbackExecutable([]() \
  {                                                             \
    reboot();                                                   \
  }));

// Helper which converts a string to a uint64 value.
static inline uint64_t string_to_uint64(string value)
{
  // remove period characters if present
  value.erase(std::remove(value.begin(), value.end(), '.'), value.end());
  // convert the string to a uint64_t value
  return std::stoull(value, nullptr, 16);
}

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
          LOG(VERBOSE, "[Config] %s (%lu bytes) mtime: %s", fullPath.c_str()
            , statbuf.st_size, ctime(&statbuf.st_mtime));
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

ConfigurationManager::ConfigurationManager(const esp32cs::Esp32ConfigDef &cfg)
  : cfg_(cfg)
{
  bool factory_reset_config{config_cs_force_factory_reset() == CONSTANT_TRUE};
  bool lcc_factory_reset{config_lcc_force_factory_reset() == CONSTANT_TRUE};
  bool persist_config{true};
  struct stat statbuf;

  sdmmc_host_t sd_host = SDSPI_HOST_DEFAULT();
  sdspi_slot_config_t sd_slot = SDSPI_SLOT_CONFIG_DEFAULT();
  esp_vfs_fat_sdmmc_mount_config_t sd_cfg =
  {
    .format_if_mount_failed = true,
    .max_files = 5,
    .allocation_unit_size = 16 * 1024
  };
  esp_err_t err = esp_vfs_fat_sdmmc_mount(CFG_MOUNT, &sd_host, &sd_slot
                                        , &sd_cfg, &sd_);
  if (err == ESP_OK)
  {
    LOG(INFO, "[Config] SD card mounted successfully.");
    FATFS *fsinfo;
    DWORD clusters;
    if (f_getfree("0:", &clusters, &fsinfo) == FR_OK)
    {
      LOG(INFO, "[Config] SD usage: %.2f/%.2f MB",
          (float)(((uint64_t)fsinfo->csize *
                  (fsinfo->n_fatent - 2 - fsinfo->free_clst)) *
                  fsinfo->ssize) / 1048576L,
          (float)(((uint64_t)fsinfo->csize * (fsinfo->n_fatent - 2)) *
                  fsinfo->ssize) / 1048576L);
    }
    else
    {
      LOG(INFO, "[Config] SD capacity %.2f MB",
          (float)(((uint64_t)sd_->csd.capacity) *
                  sd_->csd.sector_size) / 1048576);
    }
    LOG(INFO, "[Config] SD will be used for persistent storage.");
  }
  else
  {
    // unmount the SD VFS since it failed to successfully mount. We will
    // remount SPIFFS in it's place instead.
    esp_vfs_fat_sdmmc_unmount();
    LOG(INFO, "[Config] SD Card not present or mounting failed, using SPIFFS");
    esp_vfs_spiffs_conf_t conf =
    {
      .base_path = CFG_MOUNT,
      .partition_label = NULL,
      .max_files = 5,
      .format_if_mount_failed = true
    };
    // Attempt to mount the partition
    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));
    // check that the partition mounted
    size_t total = 0, used = 0;
    if (esp_spiffs_info(NULL, &total, &used) == ESP_OK)
    {
      LOG(INFO, "[Config] SPIFFS usage: %.2f/%.2f KiB", (float)(used / 1024.0f)
        , (float)(total / 1024.0f));
    }
    else
    {
      LOG_ERROR("[Config] Unable to retrieve SPIFFS utilization statistics.");
    }
    LOG(INFO, "[Config] SPIFFS will be used for persistent storage.");
  }

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
  recursiveWalkTree(CFG_MOUNT, factory_reset_config);
  // Pre-create ESP32 CS configuration directory.
  mkdir(CS_CONFIG_DIR, ACCESSPERMS);
  // Pre-create LCC configuration directory.
  mkdir(LCC_CFG_DIR, ACCESSPERMS);

  if (exists(ESP32_CS_CONFIG_JSON))
  {
    LOG(INFO, "[Config] Found existing CS config file, attempting to load...");

    csConfig = json::parse(load(ESP32_CS_CONFIG_JSON)     // content to parse
                         , nullptr                        // callback (unused)
                         , false);                        // disable exceptions

    if (!csConfig.is_discarded() && // parse failure will set root to discarded
        validateConfiguration())
    {
      LOG(INFO
        , "[Config] Existing configuration successfully loaded and validated.");
      factory_reset_config = false;
      // Check for any missing default configuration settings.
      persist_config = seedDefaultConfigSections();
      // Check if we need to force a reset of the LCC config (node id change)
      auto lccConfig = csConfig[JSON_LCC_NODE];
      if (lccConfig.contains(JSON_LCC_FORCE_RESET_NODE) &&
          lccConfig[JSON_LCC_FORCE_RESET_NODE].get<bool>())
      {
        LOG(WARNING, "[Config] LCC force factory_reset flag is ENABLED!");
        csConfig[JSON_LCC_NODE][JSON_LCC_FORCE_RESET_NODE] = false;
        persist_config = true;
        lcc_factory_reset = true;
      }
    }
    else
    {
      LOG_ERROR("[Config] Existing configuration failed one (or more) "
                "validation(s) and will be regenerated!");
      LOG(VERBOSE, "[Config] Old config: %s", csConfig.dump().c_str());
      factory_reset_config = true;
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
    csConfig = {};
    persist_config = seedDefaultConfigSections();
    // force factory reset of LCC data
    lcc_factory_reset = true;
  }

  // if we need to persist the updated config do so now.
  if (persist_config)
  {
    persistConfig();
  }

  LOG(VERBOSE, "[Config] %s", csConfig.dump().c_str());

  // If we are not currently forcing a factory reset, verify if the LCC config
  // file is the correct size. If it is not the expected size force a factory
  // reset.
  if (!lcc_factory_reset &&
      stat(LCC_CONFIG_FILE, &statbuf) != 0 &&
      statbuf.st_size != openlcb::CONFIG_FILE_SIZE)
  {
    LOG(WARNING
      , "[LCC] Corrupt configuration file detected, %s is too small: %lu "
        "bytes, expected: %zu bytes"
      , LCC_CONFIG_FILE, statbuf.st_size, openlcb::CONFIG_FILE_SIZE);
    lcc_factory_reset = true;
  }
  else
  {
    LOG(VERBOSE, "[LCC] node config file(%s) is expected size %lu bytes"
      , LCC_CONFIG_FILE, statbuf.st_size);
  }

  // If we need an LCC factory reset trigger is now.
  if (lcc_factory_reset)
  {
    configStore->factory_reset_lcc(false);
  }

#ifdef DEDICATED_MODULE_EXECUTOR
  moduleExecutor.start_thread("modules"
                            , openmrn_arduino::OPENMRN_TASK_PRIORITY - 1
                            , 4096);
#endif
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
    LOG(INFO, "[Config] Waiting for sync to stop");
    n.wait_for_notification();
    configAutoSync_.reset(nullptr);
  }

  // shutdown the executor so that no more tasks will run
  LOG(INFO, "[Config] Shutting down executor");
  openmrn->stack()->executor()->shutdown();

  LOG(INFO, "[Config] Shutting down Httpd executor");
  Singleton<esp32cs::http::Httpd>::instance()->executor()->shutdown();

#ifdef DEDICATED_MODULE_EXECUTOR
  LOG(INFO, "[Config] Shutting down Modules executor");
  moduleExecutor.shutdown();
#endif

  // close the config file if it is open
  if (configFd_ >= 0)
  {
    LOG(INFO, "[Config] Closing config file.");
    ::close(configFd_);
  }

  // Unmount the SPIFFS partition
  if (esp_spiffs_mounted(NULL))
  {
    LOG(INFO, "[Config] Unmounting SPIFFS...");
    ESP_ERROR_CHECK(esp_vfs_spiffs_unregister(NULL));
  }

  // Unmount the SD card if it was mounted
  if (sd_)
  {
    LOG(INFO, "[Config] Unmounting SD...");
    ESP_ERROR_CHECK(esp_vfs_fat_sdmmc_unmount());
  }
}

void ConfigurationManager::clear()
{
  LOG(INFO, "[Config] Clearing persistent config...");
  recursiveWalkTree(CS_CONFIG_DIR, true);
  mkdir(CS_CONFIG_DIR, ACCESSPERMS);
}

bool ConfigurationManager::exists(const string &name)
{
  struct stat statbuf;
  string configFilePath = getFilePath(name);
  LOG(VERBOSE, "[Config] Checking for %s", configFilePath.c_str());
  // this code is not using access(path, F_OK) as that is not available for
  // SPIFFS VFS. stat(path, buf) does work though.
  return !stat(configFilePath.c_str(), &statbuf);
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
    LOG_ERROR("[Config] %s does not exist, returning blank json object"
            , configFilePath.c_str());
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

void ConfigurationManager::factory_reset()
{
  remove(ESP32_CS_CONFIG_JSON);
  SCHEDULE_REBOOT();
}

void ConfigurationManager::factory_reset_lcc(bool warn)
{
  struct stat statbuf;
  // this code is not using access(path, F_OK) as that is not available for
  // SPIFFS VFS. stat(path, buf) does work though.
  if (!stat(LCC_CONFIG_FILE, &statbuf) || !stat(LCC_CDI_XML, &statbuf))
  {
    if (warn)
    {
      LOG(WARNING, "[Config] LCC factory reset underway...");
    }

    if (!stat(LCC_CONFIG_FILE, &statbuf))
    {
      LOG(WARNING, "[Config] Removing %s", LCC_CONFIG_FILE);
      ERRNOCHECK(LCC_CONFIG_FILE, unlink(LCC_CONFIG_FILE));
    }
    if (!stat(LCC_CDI_XML, &statbuf))
    {
      LOG(WARNING, "[Config] Removing %s", LCC_CDI_XML);
      ERRNOCHECK(LCC_CDI_XML, unlink(LCC_CDI_XML));
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
  auto lccConfig = csConfig[JSON_LCC_NODE];
  return (NodeID)lccConfig[JSON_LCC_NODE_ID_NODE].get<uint64_t>();
}

bool ConfigurationManager::setNodeID(string value)
{
  LOG(VERBOSE, "[Config] Current NodeID: %s, updated NodeID: %s"
    , uint64_to_string_hex(getNodeId()).c_str(), value.c_str());
  uint64_t new_node_id = string_to_uint64(value);
  if (new_node_id != getNodeId())
  {
    LOG(INFO
      , "[Config] Persisting NodeID: %s and enabling forced factory_reset."
      , uint64_to_string_hex(new_node_id).c_str());
    csConfig[JSON_LCC_NODE][JSON_LCC_NODE_ID_NODE] = new_node_id;
    csConfig[JSON_LCC_NODE][JSON_LCC_FORCE_RESET_NODE] = true;
    persistConfig();
    SCHEDULE_REBOOT();
    return true;
  }
  return false;
}

void ConfigurationManager::configureLCC(OpenMRN *openmrn)
{
  auto lccConfig = csConfig[JSON_LCC_NODE];
  if (lccConfig.contains(JSON_LCC_CAN_NODE))
  {
    auto canConfig = lccConfig[JSON_LCC_CAN_NODE];
    bool canEnabled = canConfig[JSON_LCC_CAN_ENABLED_NODE];
    gpio_num_t canRXPin =
      (gpio_num_t)canConfig[JSON_LCC_CAN_RX_NODE].get<uint8_t>();
    gpio_num_t canTXPin =
      (gpio_num_t)canConfig[JSON_LCC_CAN_TX_NODE].get<uint8_t>();
    if (canEnabled && canRXPin < GPIO_NUM_MAX && canTXPin < GPIO_NUM_MAX)
    {
      LOG(INFO, "[Config] Enabling LCC CAN interface (rx: %d, tx: %d)"
        , canRXPin, canTXPin);
      openmrn->add_can_port(
        new Esp32HardwareCan("esp32can", canRXPin, canTXPin, false));
    }
  }

  // Create the CDI.xml dynamically if it doesn't already exist.
  openmrn->create_config_descriptor_xml(cfg_, LCC_CDI_XML);

  // Create the default internal configuration file if it doesn't already exist.
  configFd_ =
    openmrn->stack()->create_config_file_if_needed(cfg_.seg().internal_config()
                                                 , ESP32CS_CDI_VERSION
                                                 , openlcb::CONFIG_FILE_SIZE);

  if (sd_)
  {
    // ESP32 FFat library uses a 512b cache in memory by default for the SD VFS
    // adding a periodic fsync call for the LCC configuration file ensures that
    // config changes are saved since the LCC config file is less than 512b.
    LOG(INFO, "[Config] Creating automatic fsync(%d) calls every %d seconds."
      , configFd_, config_lcc_sd_sync_interval_sec());
    configAutoSync_.reset(new AutoSyncFileFlow(openmrn->stack()->service()
                        , configFd_
                        , SEC_TO_USEC(config_lcc_sd_sync_interval_sec())));
  }
  if (config_lcc_print_all_packets() == CONSTANT_TRUE)
  {
    LOG(INFO, "[Config] Configuring LCC packet printer");
    openmrn->stack()->print_all_packets();
  }
}

void ConfigurationManager::setLCCHub(bool enabled)
{
  bool upd = false;
  CDI_COMPARE_AND_SET(cfg_.seg().wifi().hub().enable, configFd_, enabled, upd);
  MAYBE_TRIGGER_UPDATE(upd);
}

bool ConfigurationManager::setLCCCan(bool enabled)
{
  if (csConfig[JSON_LCC_NODE][JSON_LCC_CAN_NODE][JSON_LCC_CAN_ENABLED_NODE] != enabled)
  {
    csConfig[JSON_LCC_NODE][JSON_LCC_CAN_NODE][JSON_LCC_CAN_ENABLED_NODE] = enabled;
    persistConfig();
    SCHEDULE_REBOOT();
    return true;
  }
  return false;
}

bool ConfigurationManager::setWiFiMode(string mode)
{
  string current_mode = csConfig[JSON_WIFI_NODE][JSON_WIFI_MODE_NODE];
  if (current_mode.compare(mode))
  {
    LOG(INFO
      , "[Config] Updating WiFi mode from %s to %s, restart will be required."
      , current_mode.c_str(), mode.c_str());
    csConfig[JSON_WIFI_NODE][JSON_WIFI_MODE_NODE] = mode;
    persistConfig();
    SCHEDULE_REBOOT();
    return true;
  }
  return false;
}

bool ConfigurationManager::setWiFiStationParams(string ssid, string password
                                              , string ip, string gateway
                                              , string subnet)
{
  bool reboot_needed = false;
  string wifimode = csConfig[JSON_WIFI_NODE][JSON_WIFI_MODE_NODE];
  if (!wifimode.compare(JSON_VALUE_WIFI_MODE_SOFTAP_ONLY))
  {
    LOG(INFO, "[Config] Current config is SoftAP only, enabling Station mode");
    csConfig[JSON_WIFI_NODE][JSON_WIFI_MODE_NODE] = JSON_VALUE_WIFI_MODE_SOFTAP_STATION;
    reboot_needed = true;
  }
  LOG(INFO, "[Config] Reconfiguring Station SSID to '%s'", ssid.c_str());
  if (!ssid.empty())
  {
    csConfig[JSON_WIFI_NODE][JSON_WIFI_STATION_NODE][JSON_WIFI_SSID_NODE] = ssid;
    csConfig[JSON_WIFI_NODE][JSON_WIFI_STATION_NODE][JSON_WIFI_PASSWORD_NODE] = password;
    reboot_needed = true;
  }
  if (ip.empty())
  {
    auto station = csConfig[JSON_WIFI_NODE][JSON_WIFI_STATION_NODE];
    if (!station.contains(JSON_WIFI_MODE_NODE) ||
        station[JSON_WIFI_MODE_NODE].get<string>().compare(JSON_VALUE_STATION_IP_MODE_DHCP))
    {
      LOG(INFO, "[Config] Reconfiguring Station IP mode to DHCP");
      csConfig[JSON_WIFI_NODE][JSON_WIFI_STATION_NODE][JSON_WIFI_MODE_NODE] = JSON_VALUE_STATION_IP_MODE_DHCP;
      csConfig[JSON_WIFI_NODE][JSON_WIFI_STATION_NODE][JSON_WIFI_STATION_IP_NODE] = "";
      csConfig[JSON_WIFI_NODE][JSON_WIFI_STATION_NODE][JSON_WIFI_STATION_GATEWAY_NODE] = "";
      csConfig[JSON_WIFI_NODE][JSON_WIFI_STATION_NODE][JSON_WIFI_STATION_NETMASK_NODE] = "";
      reboot_needed = true;
    }
  }
  else
  {
    LOG(INFO
      , "[Config] Reconfiguring Station IP mode to STATIC using:\n"
        "ip: %s\n gateway: %s\nsubnet: %s"
      , ip.c_str(), gateway.c_str(), subnet.c_str());
    csConfig[JSON_WIFI_NODE][JSON_WIFI_STATION_NODE][JSON_WIFI_MODE_NODE] = JSON_VALUE_STATION_IP_MODE_STATIC;
    csConfig[JSON_WIFI_NODE][JSON_WIFI_STATION_NODE][JSON_WIFI_STATION_IP_NODE] = ip;
    csConfig[JSON_WIFI_NODE][JSON_WIFI_STATION_NODE][JSON_WIFI_STATION_GATEWAY_NODE] = gateway;
    csConfig[JSON_WIFI_NODE][JSON_WIFI_STATION_NODE][JSON_WIFI_STATION_NETMASK_NODE] = subnet;
    reboot_needed = true;
  }
  persistConfig();
  if (reboot_needed)
  {
    SCHEDULE_REBOOT();
  }
  return reboot_needed;
}

void ConfigurationManager::setWiFiUplinkParams(SocketClientParams::SearchMode mode
                                             , string uplink_service_name
                                             , string manual_hostname
                                             , uint16_t manual_port)
{
  auto wifi = cfg_.seg().wifi();
  bool upd = false;
  CDI_COMPARE_AND_SET(wifi.uplink().search_mode, configFd_, mode, upd);
  CDI_COMPARE_AND_SET(wifi.uplink().auto_address().service_name, configFd_
                    , uplink_service_name, upd);
  CDI_COMPARE_AND_SET(wifi.uplink().manual_address().ip_address, configFd_
                    , manual_hostname, upd);
  CDI_COMPARE_AND_SET(wifi.uplink().manual_address().port, configFd_
                    , manual_port, upd);

  MAYBE_TRIGGER_UPDATE(upd);
}

void ConfigurationManager::setHBridgeEvents(uint8_t index
                                          , string evt_short_on, string evt_short_off
                                          , string evt_shutdown_on, string evt_shutdown_off
                                          , string evt_thermal_on, string evt_thermal_off)
{
  bool upd = false;
  auto hbridge = cfg_.seg().hbridge().entry(index);
  CDI_COMPARE_AND_SET(hbridge.event_short, configFd_
                    , string_to_uint64(evt_short_on), upd);
  CDI_COMPARE_AND_SET(hbridge.event_short_cleared, configFd_
                    , string_to_uint64(evt_short_off), upd);
  CDI_COMPARE_AND_SET(hbridge.event_shutdown, configFd_
                    , string_to_uint64(evt_shutdown_on), upd);
  CDI_COMPARE_AND_SET(hbridge.event_shutdown_cleared, configFd_
                    , string_to_uint64(evt_shutdown_off), upd);
  // only OPS has the thermal pin
  if (index == OPS_CDI_TRACK_OUTPUT_INDEX)
  {
    CDI_COMPARE_AND_SET(hbridge.event_thermal_shutdown, configFd_
                      , string_to_uint64(evt_thermal_on), upd);
    CDI_COMPARE_AND_SET(hbridge.event_thermal_shutdown_cleared, configFd_
                      , string_to_uint64(evt_thermal_off), upd);
  }
  MAYBE_TRIGGER_UPDATE(upd);
}

string ConfigurationManager::getFilePath(const string &name)
{
  return StringPrintf("%s/%s", CS_CONFIG_DIR, name.c_str());
}

void ConfigurationManager::persistConfig()
{
  OSMutexLock l(&configMux_);
  LOG(INFO, "[Config] Persisting configuration settings");
  const std::vector<string> non_persistent_cdi_entries{JSON_CDI_UPLINK_NODE
                                                     , JSON_CDI_HUB_NODE
                                                     , JSON_HBRIDGES_NODE};
  for (auto &entry : non_persistent_cdi_entries)
  {
    auto pos = csConfig[JSON_CDI_NODE].find(entry);
    if (pos != csConfig[JSON_CDI_NODE].end())
    {
      LOG(INFO
        , "[Config] Removing non-persistent config section: %s"
        , entry.c_str());
      csConfig[JSON_CDI_NODE].erase(pos);
    }
  }

  store(ESP32_CS_CONFIG_JSON, csConfig.dump());
}

bool ConfigurationManager::validateConfiguration()
{
  OSMutexLock l(&configMux_);
  if (!csConfig.contains(JSON_WIFI_NODE))
  {
    LOG_ERROR("[Config] WiFi configuration not found.");
    return false;
  }
  else if (!csConfig.contains(JSON_LCC_NODE))
  {
    LOG_ERROR("[Config] LCC configuration not found.");
    return false;
  }
  else if (!csConfig.contains(JSON_HBRIDGES_NODE))
  {
    LOG_ERROR("[Config] H-Bridge configuration not found.");
    return false;
  }

  auto wifiNode = csConfig[JSON_WIFI_NODE];
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
  auto lccNode = csConfig[JSON_LCC_NODE];
  LOG(VERBOSE, "[Config] LCC config: %s", lccNode.dump().c_str());

  if (!lccNode.contains(JSON_LCC_NODE_ID_NODE) ||
      lccNode[JSON_LCC_NODE_ID_NODE].get<uint64_t>() == 0)
  {
    LOG_ERROR("[Config] Missing LCC node ID!");
    return false;
  }

  if (!lccNode.contains(JSON_LCC_CAN_NODE) ||
      !lccNode[JSON_LCC_CAN_NODE].contains(JSON_LCC_CAN_RX_NODE) ||
      !lccNode[JSON_LCC_CAN_NODE].contains(JSON_LCC_CAN_TX_NODE) ||
      !lccNode[JSON_LCC_CAN_NODE].contains(JSON_LCC_CAN_ENABLED_NODE))
  {
    LOG_ERROR("[Config] LCC CAN configuration invalid.");
    return false;
  }

  // verify h-bridge configuration
  auto hbridges = csConfig[JSON_HBRIDGES_NODE];
  if (!hbridges.contains("OPS"))
  {
    LOG_ERROR("[Config] H-Bridge configuration invalid (missing OPS).");
    return false;
  }
  else if (!hbridges.contains("PROG"))
  {
    LOG_ERROR("[Config] H-Bridge configuration invalid (missing PROG).");
    return false;
  }

  return true;
}

bool ConfigurationManager::seedDefaultConfigSections()
{
  bool config_updated{false};

  // If LCC config is missing default it.
  if (!csConfig.contains(JSON_LCC_NODE))
  {
    csConfig[JSON_LCC_NODE] =
    {
      { JSON_LCC_NODE_ID_NODE, UINT64_C(LCC_NODE_ID) },
      { JSON_LCC_CAN_NODE,
        {
          { JSON_LCC_CAN_ENABLED_NODE, (bool)(LCC_CAN_RX_PIN != NOT_A_PIN &&
                                              LCC_CAN_TX_PIN != NOT_A_PIN) },
          { JSON_LCC_CAN_RX_NODE, LCC_CAN_RX_PIN },
          { JSON_LCC_CAN_TX_NODE, LCC_CAN_TX_PIN },
        }
      },
      { JSON_LCC_FORCE_RESET_NODE, false },
    };
    config_updated = true;
  }
  // If LCC config is missing default it.
  if (!csConfig.contains(JSON_WIFI_NODE))
  {
    csConfig[JSON_WIFI_NODE] =
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
  if (!csConfig.contains(JSON_HC12_NODE))
  {
    csConfig[JSON_HC12_NODE] =
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
  if (!csConfig.contains(JSON_HBRIDGES_NODE))
  {
    csConfig[JSON_HBRIDGES_NODE] =
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
  if (!csConfig.contains(JSON_RAILCOM_NODE))
  {
    csConfig[JSON_RAILCOM_NODE] =
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
  auto wifiConfig = csConfig[JSON_WIFI_NODE];
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

void ConfigurationManager::configureEnabledModules(SimpleCanStack *stack)
{
#ifdef DEDICATED_MODULE_EXECUTOR
  Service *service = &moduleService;
#else
  Service *service = stack->service();
#endif
  parseWiFiConfig();
  wifiManager.reset(new Esp32WiFiManager(wifiSSID_.c_str()
                                       , wifiPassword_.c_str()
                                       , stack
                                       , cfg_.seg().wifi()
                                       , HOSTNAME_PREFIX
                                       , wifiMode_
                                       , stationStaticIP_.get()
                                       , stationDNSServer_
                                       , WIFI_SOFT_AP_CHANNEL));

  // Initialize the turnout manager and register it with the LCC stack to
  // process accessories packets.
  turnoutManager.reset(new TurnoutManager(stack->node(), service));

  auto hc12Config = csConfig[JSON_HC12_NODE];
  if (hc12Config[JSON_HC12_ENABLED_NODE].get<bool>())
  {
    LOG(INFO, "[Config] Enabling HC12 Radio");
    hc12_.emplace(service
                , (uart_port_t)hc12Config[JSON_HC12_UART_NODE].get<uint8_t>()
                , (gpio_num_t)hc12Config[JSON_HC12_RX_NODE].get<uint8_t>()
                , (gpio_num_t)hc12Config[JSON_HC12_TX_NODE].get<uint8_t>()
    );
  }
  LOG(INFO, "[Config] Registering OTA monitor");
  ota_.emplace(service);
  LOG(INFO, "[Config] Enabling InfoScreen module");
  infoScreen_.emplace(stack, service);
  LOG(INFO, "[Config] Enabling StatusLED module");
  statusLED_.emplace(service);

  // Task Monitor, periodically dumps runtime state to STDOUT.
  LOG(INFO, "[Config] Registering FreeRTOS Task Monitor");
  taskMon_.emplace(service);

  LOG(INFO, "[Config] Registering WiFi handlers (JMRI, WebServer)");
  wifiInterface.init();

#if NEXTION_ENABLED
  LOG(INFO, "[Config] Enabling Nextion module");
  nextionInterfaceInit();
#endif

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
  OSMutexLock l(&configMux_);
  openlcb::TcpClientConfig<openlcb::TcpClientDefaultParams> uplink =
    cfg_.seg().wifi().uplink();
  openmrn_arduino::HubConfiguration hub = cfg_.seg().wifi().hub();
  esp32cs::TrackOutputConfig ops = cfg_.seg().hbridge().entry(0);
  esp32cs::TrackOutputConfig prog = cfg_.seg().hbridge().entry(1);

  // insert non-persistent CDI elements that we can modify from web
  csConfig[JSON_CDI_NODE][JSON_CDI_UPLINK_NODE] =
  {
    {JSON_CDI_UPLINK_RECONNECT_NODE,
      CDI_READ_TRIMMED(uplink.reconnect, configFd_)},
    {JSON_CDI_UPLINK_MODE_NODE,
      CDI_READ_TRIMMED(uplink.search_mode, configFd_)},
    {JSON_CDI_UPLINK_AUTO_HOST_NODE,
      uplink.auto_address().host_name().read(configFd_)},
    {JSON_CDI_UPLINK_AUTO_SERVICE_NODE,
      uplink.auto_address().service_name().read(configFd_)},
    {JSON_CDI_UPLINK_MANUAL_HOST_NODE,
      uplink.manual_address().ip_address().read(configFd_)},
    {JSON_CDI_UPLINK_MANUAL_PORT_NODE,
      CDI_READ_TRIMMED(uplink.manual_address().port, configFd_)},
  };
  csConfig[JSON_CDI_NODE][JSON_CDI_HUB_NODE] =
  {
    {JSON_CDI_HUB_ENABLE_NODE, CDI_READ_TRIMMED(hub.enable, configFd_)},
    {JSON_CDI_HUB_PORT_NODE, CDI_READ_TRIMMED(hub.port, configFd_)},
    {JSON_CDI_HUB_SERVICE_NODE, hub.service_name().read(configFd_)},
  };
  csConfig[JSON_CDI_NODE][JSON_HBRIDGES_NODE] =
  {
    {"OPS",
      {
        {JSON_DESCRIPTION_NODE, ops.description().read(configFd_)},
        {JSON_CDI_HBRIDGE_SHORT_EVENT_NODE,
          uint64_to_string_hex(ops.event_short().read(configFd_))},
        {JSON_CDI_HBRIDGE_SHORT_CLEAR_EVENT_NODE,
          uint64_to_string_hex(ops.event_short_cleared().read(configFd_))},
        {JSON_CDI_HBRIDGE_SHUTDOWN_EVENT_NODE,
          uint64_to_string_hex(ops.event_shutdown().read(configFd_))},
        {JSON_CDI_HBRIDGE_SHUTDOWN_CLEAR_EVENT_NODE,
          uint64_to_string_hex(ops.event_shutdown_cleared().read(configFd_))},
        {JSON_CDI_HBRIDGE_THERMAL_EVENT_NODE,
          uint64_to_string_hex(ops.event_thermal_shutdown().read(configFd_))},
        {JSON_CDI_HBRIDGE_THERMAL_CLEAR_EVENT_NODE,
          uint64_to_string_hex(ops.event_thermal_shutdown_cleared().read(configFd_))},
      }
    },
    {"PROG",
      {
        {JSON_DESCRIPTION_NODE, prog.description().read(configFd_)},
        {JSON_CDI_HBRIDGE_SHORT_EVENT_NODE,
          uint64_to_string_hex(prog.event_short().read(configFd_))},
        {JSON_CDI_HBRIDGE_SHORT_CLEAR_EVENT_NODE,
          uint64_to_string_hex(prog.event_short_cleared().read(configFd_))},
        {JSON_CDI_HBRIDGE_SHUTDOWN_EVENT_NODE,
          uint64_to_string_hex(prog.event_shutdown().read(configFd_))},
        {JSON_CDI_HBRIDGE_SHUTDOWN_CLEAR_EVENT_NODE,
          uint64_to_string_hex(prog.event_shutdown_cleared().read(configFd_))},
      }
    },
  };
  return csConfig.dump();
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
    , csConfig[JSON_HC12_NODE][JSON_HC12_ENABLED_NODE].get<bool>() }
  };
  return features.dump();
}

#include "os/OSSelectWakeup.hxx"
#include "utils/logging.h"

void empty_signal_handler(int)
{
}

#ifdef ESP32
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <esp_vfs.h>

/// Protects the initialization of vfs_id.
static pthread_once_t vfs_init_once = PTHREAD_ONCE_INIT;
/// This per-thread key will store the OSSelectWakeup object that has been
/// locked to any given calling thread.
static pthread_key_t select_wakeup_key;

static int esp_wakeup_open(const char * path, int flags, int mode) {
    // This virtual FS has only one fd, 0.
    return 0;
}

static void esp_end_select()
{
    OSSelectWakeup *parent =
        (OSSelectWakeup *)pthread_getspecific(select_wakeup_key);
    HASSERT(parent);
    parent->esp_end_select();
}

/// This function is called inline from the ESP32's select implementation. It is
/// passed in as a function pointer to the VFS API.
/// @param nfds see standard select API
/// @param readfds see standard select API
/// @param writefds see standard select API
/// @param exceptfds see standard select API
/// @param signal_sem if non-NULL, the select can be woken up by notifying this
/// semaphore. If NULL, the select can be woken up by notifying the LWIP
/// semaphore. By the API contract this pointer needs to be passed into
/// esp_vfs_select_triggered.
static esp_err_t esp_start_select(int nfds, fd_set *readfds, fd_set *writefds,
    fd_set *exceptfds, SemaphoreHandle_t *signal_sem)
{
    OSSelectWakeup *parent =
        (OSSelectWakeup *)pthread_getspecific(select_wakeup_key);
    LOG(VERBOSE, "esp start select %p  (thr %p parent %p)", signal_sem, os_thread_self(), parent);
    HASSERT(parent);
    parent->esp_start_select(signal_sem);
    return ESP_OK;
}

void OSSelectWakeup::esp_start_select(void *signal_sem)
{
    AtomicHolder h(this);
    espSem_ = signal_sem;
    woken_ = false;
}

void OSSelectWakeup::esp_end_select()
{
    AtomicHolder h(this);
    woken_ = true;
}

void OSSelectWakeup::esp_wakeup()
{
    if (woken_)
    {
        return;
    }
    AtomicHolder h(this);
    if (woken_)
    {
        return;
    }
    woken_ = true;
    LOG(VERBOSE, "wakeup es %p %u", espSem_, *(unsigned*)espSem_);
    if (espSem_)
    {
        esp_vfs_select_triggered((SemaphoreHandle_t *)espSem_);
    }
}

void OSSelectWakeup::esp_wakeup_from_isr()
{
    if (woken_)
    {
        return;
    }
    AtomicHolder h(this);
    if (woken_)
    {
        return;
    }
    woken_ = true;
    BaseType_t woken = pdFALSE;
    if (espSem_)
    {
        esp_vfs_select_triggered_isr((SemaphoreHandle_t *)espSem_, &woken);
    }
    if (woken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
}

static void esp_vfs_init()
{
    esp_vfs_t vfs;
    memset(&vfs, 0, sizeof(vfs));
    vfs.flags = ESP_VFS_FLAG_DEFAULT;
    vfs.start_select = &esp_start_select;
    vfs.end_select = &esp_end_select;
    vfs.open = &esp_wakeup_open;
    ESP_ERROR_CHECK(esp_vfs_register("/dev/wakeup", &vfs, nullptr));
    HASSERT(0 == pthread_key_create(&select_wakeup_key, nullptr));
}

void OSSelectWakeup::esp_allocate_vfs_fd()
{
    pthread_once(&vfs_init_once, &esp_vfs_init);
    vfsFd_ = ::open("/dev/wakeup/0", 0, 0);
    HASSERT(vfsFd_ >= 0);
    pthread_setspecific(select_wakeup_key, this);
    LOG(VERBOSE, "VFSALLOC wakeup fd %d (thr %p test %p)", vfsFd_
      , os_thread_self(), pthread_getspecific(select_wakeup_key));
}

void OSSelectWakeup::esp_deallocate_vfs_fd()
{
}

#endif // ESP32

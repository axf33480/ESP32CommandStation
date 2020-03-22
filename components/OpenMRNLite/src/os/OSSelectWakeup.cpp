/** \copyright
 * Copyright (c) 2015, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \file OSSelectWakeup.cxx
 * Helper class for portable wakeup of a thread blocked in a select call.
 *
 * @author Balazs Racz
 * @date 10 Apr 2015
 */

#include "os/OSSelectWakeup.hxx"
#include "utils/logging.h"

void empty_signal_handler(int)
{
}

#ifdef ESP32
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <esp_system.h>
#include <esp_vfs.h>

/// Protects the initialization of vfs_id.
static pthread_once_t vfs_init_once = PTHREAD_ONCE_INIT;
/// This per-thread key will store the OSSelectWakeup object that has been
/// locked to any given calling thread.
static pthread_key_t select_wakeup_key;
static int wakeup_fd;

// With IDF v4+ we do not need to trigger the lwip semaphore so exclude the
// function definitions.
#if !defined(ESP_IDF_VERSION_MAJOR)
extern "C"
{
    void *sys_thread_sem_get();
    void sys_sem_signal(void *);
    void sys_sem_signal_isr(void *);
}
#endif // IDF v4+

static int esp_wakeup_open(const char * path, int flags, int mode)
{
    // This virtual FS has only one fd, 0.
    return 0;
}

#if defined(ESP_IDF_VERSION_MAJOR)
static esp_err_t esp_end_select(void *args)
#else
static void esp_end_select()
#endif // IDF v4+
{
    OSSelectWakeup *parent =
        (OSSelectWakeup *)pthread_getspecific(select_wakeup_key);
    HASSERT(parent);
#if defined(ESP_IDF_VERSION_MAJOR)
    return parent->esp_end_select(args);
#else
    parent->esp_end_select();
#endif // IDF v4+
}

/// This function is called inline from the ESP32's select implementation. It is
/// passed in as a function pointer to the VFS API.
/// @param nfds see standard select API
/// @param readfds see standard select API
/// @param writefds see standard select API
/// @param exceptfds see standard select API
#if defined(ESP_IDF_VERSION_MAJOR)
/// @param signal_sem is the semaphore object to trigger when the select has
/// completed.
/// @param end_select_args are the arguments to pass to end_select upon wakeup.
static esp_err_t esp_start_select(int nfds, fd_set *readfds, fd_set *writefds,
    fd_set *exceptfds, esp_vfs_select_sem_t signal_sem, void **end_select_args)
#else
/// @param signal_sem if non-NULL, the select can be woken up by notifying this
/// semaphore. If NULL, the select can be woken up by notifying the LWIP
/// semaphore. By the API contract this pointer needs to be passed into
/// esp_vfs_select_triggered.
static esp_err_t esp_start_select(int nfds, fd_set *readfds, fd_set *writefds,
    fd_set *exceptfds, SemaphoreHandle_t *signal_sem)
#endif // IDF v4+
{
    OSSelectWakeup *parent =
        (OSSelectWakeup *)pthread_getspecific(select_wakeup_key);
    HASSERT(parent);
#if defined(ESP_IDF_VERSION_MAJOR)
    parent->esp_start_select(signal_sem, end_select_args);
#else
    LOG(VERBOSE, "esp start select %p  (thr %p parent %p)", signal_sem, os_thread_self(), parent);
    HASSERT(parent);
    parent->esp_start_select(signal_sem);
#endif // IDF v4+
    return ESP_OK;
}

#if defined(ESP_IDF_VERSION_MAJOR)
void OSSelectWakeup::esp_start_select(esp_vfs_select_sem_t signal_sem, void **args)
#else
void OSSelectWakeup::esp_start_select(void *signal_sem)
#endif // IDF v4+
{
    AtomicHolder h(this);
    espSem_ = signal_sem;
#if defined(ESP_IDF_VERSION_MAJOR)
    endSelectArgs_ = args;
#endif // IDF v4+
    woken_ = false;
}

#if defined(ESP_IDF_VERSION_MAJOR)
esp_err_t OSSelectWakeup::esp_end_select(void *args)
#else
void OSSelectWakeup::esp_end_select()
#endif // IDF v4+
{
    AtomicHolder h(this);
    woken_ = true;
#if defined(ESP_IDF_VERSION_MAJOR)
    return ESP_OK;
#endif // IDF v4+
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
#if defined(ESP_IDF_VERSION_MAJOR)
    if (espSem_.sem)
    {
        esp_vfs_select_triggered(espSem_);
    }
#else
    LOG(VERBOSE, "wakeup es %p %u lws %p", espSem_, *(unsigned*)espSem_, lwipSem_);
    if (espSem_)
    {
        esp_vfs_select_triggered((SemaphoreHandle_t *)espSem_);
    }
    else
    {
        // Works around a bug in the implementation of
        // esp_vfs_select_triggered, which internally calls
        // sys_sem_signal(sys_thread_sem_get()); This is buggy because
        // sys_thread_sem_get() will get the semaphore that belongs to the
        // calling thread, not the target thread to wake up.
        sys_sem_signal(lwipSem_);
    }
#endif // IDF v4+
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
#if defined(ESP_IDF_VERSION_MAJOR)
    if (espSem_.sem)
    {
        esp_vfs_select_triggered_isr(espSem_, &woken);
    }
#else
    if (espSem_)
    {
        esp_vfs_select_triggered_isr((SemaphoreHandle_t *)espSem_, &woken);
    }
    else
    {
        // Works around a bug in the implementation of
        // esp_vfs_select_triggered, which internally calls
        // sys_sem_signal(sys_thread_sem_get()); This is buggy because
        // sys_thread_sem_get() will get the semaphore that belongs to the
        // calling thread, not the target thread to wake up.
        sys_sem_signal_isr(lwipSem_);
    }
#endif // IDF v4+
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
    wakeup_fd = ::open("/dev/wakeup/0", 0, 0);
    HASSERT(wakeup_fd >= 0);
    LOG(VERBOSE, "VFSINIT wakeup fd %d", wakeup_fd);
}

void OSSelectWakeup::esp_allocate_vfs_fd()
{
#if !defined(ESP_IDF_VERSION_MAJOR)
    lwipSem_ = sys_thread_sem_get();
#endif // not IDF v4+
    pthread_once(&vfs_init_once, &esp_vfs_init);
    vfsFd_ = wakeup_fd;
    pthread_setspecific(select_wakeup_key, this);
    LOG(VERBOSE, "VFSALLOC wakeup fd %d (thr %p test %p)", vfsFd_
      , os_thread_self(), pthread_getspecific(select_wakeup_key));
}

void OSSelectWakeup::esp_deallocate_vfs_fd()
{
}

#endif // ESP32

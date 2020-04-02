/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2019-2020 Mike Dunston

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

#ifndef DUPLEXED_TRACK_IF_H_
#define DUPLEXED_TRACK_IF_H_

#include <executor/Executor.hxx>
#include <executor/StateFlow.hxx>
#include <dcc/Packet.hxx>

namespace esp32cs
{

/// StateFlow that accepts dcc::Packet structures and sends them to a local
/// device driver for producing the track signal.
///
/// The device driver must support the notifiable-based asynchronous write
/// model.
class DuplexedTrackIf : public StateFlow<Buffer<dcc::Packet>, QList<1>>
{
public:
    /** Creates a TrackInterface from an fd to the mainline and an fd for prog.
     *
     * This class currently does synchronous writes to the device. In order not
     * to block the executor, you have to create a new threadexecutor.
     *
     * @param service THE EXECUTOR OF THIS SERVICE WILL BE BLOCKED.
     * @param pool_size will determine how many packets the current flow's
     * alloc() will have.
     */
    DuplexedTrackIf(Service *service, int pool_size);

    FixedPool *pool() OVERRIDE
    {
        return &pool_;
    }

    /// You must call this function before sending any packets.
    ///
    /// @param fd is the file descriptor to the OPS track output.
    void set_fd_ops(int fd)
    {
        fd_ops_ = fd;
    }

    /// You must call this function before sending any packets.
    ///
    /// @param fd is the file descriptor to the PROG track output.
    void set_fd_prog(int fd)
    {
        fd_prog_ = fd;
    }
protected:
    Action entry() OVERRIDE;

    /// @return next action.
    Action finish()
    {
        return release_and_exit();
    }

    /// Filedes of the device to which we are writing the generated packets.
    int fd_ops_{-1};

    /// Filedes of the device to which we are writing the generated packets.
    int fd_prog_{-1};

    /// Packet pool from which to allocate packets.
    FixedPool pool_;
};

} // namespace esp32cs

#endif // DUPLEXED_TRACK_IF_H_
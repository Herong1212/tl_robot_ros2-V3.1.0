#ifndef TL_TELEOP__SINGLE_INSTANCE_H_
#define TL_TELEOP__SINGLE_INSTANCE_H_

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <string>
#include <utility>

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

namespace tl_teleop
{

// The PC service supports one active SDK consumer. Keep the lock for the whole
// process lifetime; never unlink it, which could let two owners lock two inodes.
class SingleInstance
{
public:
    explicit SingleInstance(std::string path = default_path()) : path_(std::move(path)) {}
    SingleInstance(const SingleInstance &) = delete;
    SingleInstance &operator=(const SingleInstance &) = delete;

    ~SingleInstance()
    {
        if (fd_ >= 0)
            ::close(fd_);
    }

    bool acquire(std::string &error)
    {
        if (fd_ >= 0)
            return true;
        int fd = ::open(path_.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
        if (fd < 0)
        {
            error = "Cannot open teleop instance lock " + path_ + ": " + std::strerror(errno);
            return false;
        }
        if (::flock(fd, LOCK_EX | LOCK_NB) != 0)
        {
            const int lock_error = errno;
            char owner[32]{};
            const auto count = ::pread(fd, owner, sizeof(owner) - 1, 0);
            ::close(fd);
            if (lock_error == EWOULDBLOCK || lock_error == EAGAIN)
            {
                error = "Another tl_teleop instance is running (PID " +
                    (count > 0 ? std::string(owner, count) : "unknown") +
                    "). Stop the existing instance before starting another VR SDK consumer.";
            }
            else
                error = "Cannot lock " + path_ + ": " + std::strerror(lock_error);
            return false;
        }
        const auto pid = std::to_string(::getpid());
        if (::ftruncate(fd, 0) != 0 ||
            ::pwrite(fd, pid.data(), pid.size(), 0) != static_cast<ssize_t>(pid.size()))
        {
            error = "Cannot record teleop instance PID in " + path_;
            ::close(fd);
            return false;
        }
        fd_ = fd;
        return true;
    }

private:
    static std::string default_path()
    {
        // Explicit override is for isolated process tests. Ordinary desktop and
        // background launches must use the same path regardless of XDG setup.
        const char *test_path = std::getenv("TL_TELEOP_INSTANCE_LOCK");
        if (test_path && test_path[0] == '/')
            return test_path;
        return "/tmp/tl_teleop_" + std::to_string(::getuid()) + ".lock";
    }

    std::string path_;
    int fd_{-1};
};

} // namespace tl_teleop

#endif // TL_TELEOP__SINGLE_INSTANCE_H_

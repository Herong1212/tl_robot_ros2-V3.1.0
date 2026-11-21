#include <csignal>
#include <string>

#include <gtest/gtest.h>
#include <sys/wait.h>
#include <unistd.h>

#include "tl_teleop/single_instance.h"

class SingleInstanceTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        char path[] = "/tmp/tl_teleop_instance_test_XXXXXX";
        const int fd = ::mkstemp(path);
        ASSERT_GE(fd, 0);
        ::close(fd);
        path_ = path;
    }
    void TearDown() override { ::unlink(path_.c_str()); }
    std::string path_;
};

TEST_F(SingleInstanceTest, RejectsSecondProcessAndReleasesOnOwnerExit)
{
    int started[2], finish[2];
    ASSERT_EQ(::pipe(started), 0);
    ASSERT_EQ(::pipe(finish), 0);
    const auto pid = ::fork();
    ASSERT_GE(pid, 0);
    if (pid == 0)
    {
        ::close(started[0]);
        ::close(finish[1]);
        tl_teleop::SingleInstance owner(path_);
        std::string error;
        const char acquired = owner.acquire(error) ? '1' : '0';
        const auto written = ::write(started[1], &acquired, 1);
        char stop;
        const auto read = ::read(finish[0], &stop, 1);
        ::_exit(written == 1 && read == 1 && acquired == '1' ? 0 : 1);
    }
    ::close(started[1]);
    ::close(finish[0]);
    char acquired = '0';
    EXPECT_EQ(::read(started[0], &acquired, 1), 1);
    EXPECT_EQ(acquired, '1');
    tl_teleop::SingleInstance contender(path_);
    std::string error;
    EXPECT_FALSE(contender.acquire(error));
    EXPECT_NE(error.find(std::to_string(pid)), std::string::npos);
    EXPECT_EQ(::write(finish[1], "x", 1), 1);
    int status = 0;
    EXPECT_EQ(::waitpid(pid, &status, 0), pid);
    EXPECT_TRUE(WIFEXITED(status));
    EXPECT_EQ(WEXITSTATUS(status), 0);
    EXPECT_TRUE(contender.acquire(error));
    ::close(started[0]);
    ::close(finish[1]);
}

TEST_F(SingleInstanceTest, ReleasesWithoutDeletingLockFile)
{
    std::string error;
    {
        tl_teleop::SingleInstance owner(path_);
        ASSERT_TRUE(owner.acquire(error));
        ASSERT_TRUE(owner.acquire(error));
    }
    EXPECT_EQ(::access(path_.c_str(), F_OK), 0);
    tl_teleop::SingleInstance next(path_);
    EXPECT_TRUE(next.acquire(error));
}

#pragma once

#if defined (WIN32)
#include <ctime>
#include <sys/stat.h>

// from http://stackoverflow.com/questions/592448/c-how-to-set-file-permissions-cross-platform
typedef int mode_t;

static const mode_t S_ISUID = 0x08000000;           ///< does nothing
static const mode_t S_ISGID = 0x04000000;           ///< does nothing
static const mode_t S_ISVTX = 0x02000000;           ///< does nothing
#ifndef S_IRUSR
static const mode_t S_IRUSR = mode_t(_S_IREAD);     ///< read by user
#endif
#ifndef S_IWUSR
static const mode_t S_IWUSR = mode_t(_S_IWRITE);    ///< write by user
#endif
#ifndef S_IXUSR
static const mode_t S_IXUSR = 0x00400000;           ///< does nothing
#endif
#   ifndef STRICT_UGO_PERMISSIONS
#ifndef S_IRGRP
static const mode_t S_IRGRP = mode_t(_S_IREAD);     ///< read by *USER*
#endif
#ifndef S_IWGRP
static const mode_t S_IWGRP = mode_t(_S_IWRITE);    ///< write by *USER*
#endif
#ifndef S_IXGRP
static const mode_t S_IXGRP = 0x00080000;           ///< does nothing
#endif
#ifndef S_IROTH
static const mode_t S_IROTH = mode_t(_S_IREAD);     ///< read by *USER*
#endif
#ifndef S_IWOTH
static const mode_t S_IWOTH = mode_t(_S_IWRITE);    ///< write by *USER*
#endif
#ifndef S_IXOTH
static const mode_t S_IXOTH = 0x00010000;           ///< does nothing
#endif
#   else
static const mode_t S_IRGRP = 0x00200000;           ///< does nothing
static const mode_t S_IWGRP = 0x00100000;           ///< does nothing
static const mode_t S_IXGRP = 0x00080000;           ///< does nothing
static const mode_t S_IROTH = 0x00040000;           ///< does nothing
static const mode_t S_IWOTH = 0x00020000;           ///< does nothing
static const mode_t S_IXOTH = 0x00010000;           ///< does nothing
#   endif
static const mode_t MS_MODE_MASK = 0x0000ffff;           ///< low word

#include <io.h>

//Adapted from https://github.com/swem/player-git-svn/blob/master/replace/nanosleep.c


#define _WIN32_WINNT 0x0500
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#define NOGDI
#include <windows.h>


// Replacement for nanosleep on Windows.
// NOTES:
// The rem argument is never filled. You cannot rely on it.
// The return value will always be zero. There is no way to tell if the sleep was interrupted.
// The resolution of this is 100 nanoseconds, despite the argument being seconds and nanoseconds.
static int nanosleep(const struct timespec *req, struct timespec *rem)
{
	HANDLE timer = NULL;
	LARGE_INTEGER sleepTime;

	sleepTime.QuadPart = req->tv_sec * 1000000000 + req->tv_nsec / 100;

	timer = CreateWaitableTimer(NULL, TRUE, NULL);
	if (timer == NULL)
	{
		return -1;
	}

	if (!SetWaitableTimer(timer, &sleepTime, 0, NULL, NULL, 0))
	{
		return -1;
	}

	if (WaitForSingleObject(timer, INFINITE) != WAIT_OBJECT_0)
	{
		return -1;
	}

	return 0;
}

//Adapted from https://android.googlesource.com/platform/bionic/+/ab61eb366ac48addf2bca6093a34455193f5c8df/libc/upstream-freebsd/lib/libc/gen/usleep.c

typedef unsigned int useconds_t;

static int usleep(useconds_t useconds)
{
	struct timespec time_to_sleep;
	time_to_sleep.tv_nsec = (useconds % 1000000) * 1000;
	time_to_sleep.tv_sec = useconds / 1000000;
	return (nanosleep(&time_to_sleep, NULL));
}

#endif

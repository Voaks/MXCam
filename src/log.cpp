// log.cpp
#include <windows.h>
#include <cstdarg>
#include <cstdio>

#include "log.h"

namespace mxcam {
namespace {

HANDLE  g_file = INVALID_HANDLE_VALUE;
wchar_t g_directory[1024] = L"";
SRWLOCK g_lock = SRWLOCK_INIT;

}  // namespace

void LogOpen(const wchar_t* path)
{
    g_file = CreateFileW(path, GENERIC_WRITE, FILE_SHARE_READ, nullptr, CREATE_ALWAYS,
                         FILE_ATTRIBUTE_NORMAL, nullptr);
    wcsncpy_s(g_directory, path, _TRUNCATE);
    wchar_t* slash = wcsrchr(g_directory, L'\\');
    if (slash)
        slash[1] = L'\0';
    else
        g_directory[0] = L'\0';
}

const wchar_t* LogDirectory() { return g_directory; }

void LogClose()
{
    if (g_file != INVALID_HANDLE_VALUE) {
        CloseHandle(g_file);
        g_file = INVALID_HANDLE_VALUE;
    }
}

void Log(const char* format, ...)
{
    if (g_file == INVALID_HANDLE_VALUE)
        return;

    char line[1024];
    SYSTEMTIME now;
    GetLocalTime(&now);
    int length = snprintf(line, sizeof line, "[%02u:%02u:%02u.%03u] ",
                          now.wHour, now.wMinute, now.wSecond, now.wMilliseconds);

    // Leave room for "\r\n"; vsnprintf truncates long messages safely.
    const int room = static_cast<int>(sizeof line) - length - 2;
    va_list args;
    va_start(args, format);
    const int written = vsnprintf(line + length, room, format, args);
    va_end(args);
    if (written > 0)
        length += written < room ? written : room - 1;
    line[length++] = '\r';
    line[length++] = '\n';

    // WriteFile goes straight to the OS cache, so the log survives a game crash.
    AcquireSRWLockExclusive(&g_lock);
    DWORD ignored = 0;
    WriteFile(g_file, line, static_cast<DWORD>(length), &ignored, nullptr);
    ReleaseSRWLockExclusive(&g_lock);
}

}  // namespace mxcam

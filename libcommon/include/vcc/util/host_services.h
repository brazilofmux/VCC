#pragma once
/*
Copyright 2026 by the VCC Project Contributors.
This file is part of VCC (Virtual Color Computer).

    VCC (Virtual Color Computer) is free software: you can redistribute it
    and/or modify it under the terms of the GNU General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    VCC (Virtual Color Computer) is distributed in the hope that it will be
    useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with VCC (Virtual Color Computer). If not, see
    <http://www.gnu.org/licenses/>.
*/

// host_services: the single seam between the portable core and the host OS.
//
// On Windows this is exactly <Windows.h> - zero behavior change for the
// MSBuild path. On other hosts it supplies the narrow slice of the Win32
// surface the portable core actually touches: opaque handle types,
// RECT/POINT, MessageBox routed to stderr, OutputDebugString, and inert
// clipboard stubs. The Windows-only shell (Vcc.cpp, config.cpp,
// DirectX/DirectSound, dialogs) is deliberately NOT covered here; that
// code is replaced wholesale by the SDL shell per docs/porting-macos.md.
//
// Core files should include this instead of <Windows.h>. Growing the
// non-Windows section is fine, but each addition should serve a file in
// the portable set - if shell code needs something, port the shell code
// instead.

#ifdef _WIN32

#include <Windows.h>

#else

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <dlfcn.h>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#ifdef __APPLE__
#include <mach-o/dyld.h>
#endif

// MSVC calling-convention keywords are meaningless off Windows.
#define __fastcall
#define __cdecl
#define CALLBACK
#define WINAPI
#define __declspec(x)

// MSVC-isms that appear in the portable core.
#define _inline inline
#define __int64 long long
#define _stricmp strcasecmp

typedef void*           HWND;
typedef void*           HINSTANCE;
typedef void*           HMODULE;
typedef void*           HANDLE;
typedef void*           HGLOBAL;
typedef void*           HDC;
typedef void*           HBITMAP;
typedef int32_t         LONG;
typedef uint32_t        DWORD;
typedef uint16_t        WORD;
typedef uint8_t         BYTE;
typedef int             BOOL;
typedef unsigned int    UINT;
typedef char*           LPSTR;
typedef const char*     LPCSTR;
typedef intptr_t        LPARAM;
typedef uintptr_t       WPARAM;
typedef intptr_t        LRESULT;
typedef intptr_t        INT_PTR;
typedef void*           LPVOID;
typedef void*           PVOID;
typedef uint8_t*        PBYTE;
typedef uint32_t        ULONG;

struct RECT  { LONG left; LONG top; LONG right; LONG bottom; };
struct POINT { LONG x; LONG y; };

// Same layout as the Win32 GUID (audio.h carries one in its device-
// selection interface; off Windows it is never dereferenced).
struct _GUID
{
    uint32_t Data1;
    uint16_t Data2;
    uint16_t Data3;
    uint8_t  Data4[8];
};
typedef _GUID GUID;

#ifndef TRUE
#define TRUE  1
#define FALSE 0
#endif

#define CW_USEDEFAULT   ((int)0x80000000)
#define MAX_PATH        260

#define MB_OK               0x00000000u
#define MB_YESNO            0x00000004u
#define MB_ICONERROR        0x00000010u
#define MB_ICONWARNING      0x00000030u
#define MB_SETFOREGROUND    0x00010000u
#define MB_TOPMOST          0x00040000u
#define MB_TASKMODAL        0x00002000u
#define IDOK            1
#define IDYES           6
#define IDNO            7

inline void OutputDebugStringA(const char* s)
{
    std::fputs(s ? s : "", stderr);
}
#define OutputDebugString OutputDebugStringA

inline int MessageBoxA(HWND, const char* text, const char* caption, UINT)
{
    std::fprintf(stderr, "[%s] %s\n",
                 caption ? caption : "VCC", text ? text : "");
    return IDOK;
}
#define MessageBox MessageBoxA

// Clipboard: headless hosts have none. OpenClipboard reports failure so
// callers take their error paths; the allocation trio stays real so
// SetClipboard's unchecked memcpy(GlobalLock(hMem), ...) never faults.
#define CF_TEXT         1u
#define GMEM_MOVEABLE   2u

// File handles for the printer-capture path (mc6821.cpp): HANDLE is a
// FILE* in disguise, CreateFile maps the creation disposition onto an
// fopen mode, and the printer-monitor "console" is stdout. The capture
// feature works the same as on Windows.
#define INVALID_HANDLE_VALUE        ((HANDLE)(intptr_t)-1)
#define GENERIC_READ                0x80000000u
#define GENERIC_WRITE               0x40000000u
#define FILE_SHARE_READ             1u
#define CREATE_NEW                  1u
#define CREATE_ALWAYS               2u
#define OPEN_EXISTING               3u
#define OPEN_ALWAYS                 4u
#define FILE_ATTRIBUTE_NORMAL       0x80u
#define FILE_BEGIN                  0u
#define FILE_CURRENT                1u
#define FILE_END                    2u
#define INVALID_SET_FILE_POINTER    ((DWORD)-1)
#define INVALID_FILE_SIZE           ((DWORD)-1)
#define INVALID_FILE_ATTRIBUTES     ((DWORD)-1)
#define STD_OUTPUT_HANDLE           ((DWORD)-11)

inline HANDLE CreateFileA(const char* name, DWORD access, DWORD, void*,
                          DWORD disposition, DWORD, HANDLE)
{
    const char* mode;
    switch (disposition)
    {
    case CREATE_NEW:    mode = "w+bx"; break;
    case CREATE_ALWAYS: mode = "w+b"; break;
    case OPEN_ALWAYS:   mode = (access & GENERIC_WRITE) ? "a+b" : "ab"; break;
    default:            mode = (access & GENERIC_WRITE) ? "r+b" : "rb"; break;
    }
    FILE* f = std::fopen(name, mode);
    return f ? (HANDLE)f : INVALID_HANDLE_VALUE;
}
#define CreateFile CreateFileA

inline BOOL ReadFile(HANDLE h, void* data, DWORD len,
                     unsigned long* read, void*)
{
    if (h == nullptr || h == INVALID_HANDLE_VALUE)
        return FALSE;
    const size_t n = std::fread(data, 1, len, (FILE*)h);
    if (read) *read = (unsigned long)n;
    return TRUE;
}

inline BOOL ReadFile(HANDLE h, void* data, DWORD len,
                     DWORD* read, void* overlapped)
{
    unsigned long n = 0;
    const BOOL ok = ReadFile(h, data, len, &n, overlapped);
    if (read) *read = (DWORD)n;
    return ok;
}

inline DWORD SetFilePointer(HANDLE h, int32_t distance, int32_t* high,
                            DWORD method)
{
    if (h == nullptr || h == INVALID_HANDLE_VALUE || high != nullptr)
        return INVALID_SET_FILE_POINTER;
    const int whence = (method == FILE_BEGIN) ? SEEK_SET
                     : (method == FILE_CURRENT) ? SEEK_CUR : SEEK_END;
    if (std::fseek((FILE*)h, distance, whence) != 0)
        return INVALID_SET_FILE_POINTER;
    const long pos = std::ftell((FILE*)h);
    return pos < 0 ? INVALID_SET_FILE_POINTER : (DWORD)pos;
}

inline DWORD GetFileSize(HANDLE h, DWORD* high)
{
    if (h == nullptr || h == INVALID_HANDLE_VALUE)
        return INVALID_FILE_SIZE;
    if (high) *high = 0;
    struct stat st;
    if (fstat(fileno((FILE*)h), &st) != 0)
        return INVALID_FILE_SIZE;
    return (DWORD)st.st_size;
}

inline BOOL SetEndOfFile(HANDLE h)
{
    if (h == nullptr || h == INVALID_HANDLE_VALUE)
        return FALSE;
    FILE* f = (FILE*)h;
    std::fflush(f);
    const long pos = std::ftell(f);
    return pos >= 0 && ftruncate(fileno(f), pos) == 0;
}

inline DWORD GetFileAttributesA(const char* name)
{
    struct stat st;
    if (name == nullptr || stat(name, &st) != 0)
        return INVALID_FILE_ATTRIBUTES;
    return FILE_ATTRIBUTE_NORMAL;
}
#define GetFileAttributes GetFileAttributesA

// Device-ioctl surface for the raw-floppy support in FD502/wd1793.cpp
// (the fdrawcmd.sys driver is Windows-only). Defining CTL_CODE here also
// keeps fdrawcmd.h from including <winioctl.h>. The shim always fails,
// so InitController reports no raw-disk support and those paths stay
// dormant - .dsk image files are the real path on every host.
#define METHOD_BUFFERED     0
#define METHOD_IN_DIRECT    1
#define METHOD_OUT_DIRECT   2
#define METHOD_NEITHER      3
#define FILE_DEVICE_UNKNOWN 0x22
#define FILE_ANY_ACCESS     0
#define FILE_READ_DATA      1
#define FILE_WRITE_DATA     2
#define CTL_CODE(DeviceType, Function, Method, Access) \
    (((DWORD)(DeviceType) << 16) | ((DWORD)(Access) << 14) | \
     ((DWORD)(Function) << 2) | (DWORD)(Method))

inline BOOL DeviceIoControl(HANDLE, DWORD, void*, DWORD, void*, DWORD,
                            DWORD* returned, void*)
{
    if (returned) *returned = 0;
    return FALSE;
}

// Window plumbing referenced by cartridge modules: no windows exist, so
// handles are null and host notifications (menu rebuild, CPU reset
// requests via WM_VCC_* messages) are quietly dropped. The SDL shell
// replaces SendMessage with a real host-notification hook.
inline HWND    GetActiveWindow()                   { return nullptr; }
inline HWND    GetForegroundWindow()               { return nullptr; }
inline LRESULT SendMessageA(HWND, UINT, WPARAM, LPARAM) { return 0; }
#define SendMessage SendMessageA

inline BOOL FlushFileBuffers(HANDLE h)
{
    if (h == nullptr || h == INVALID_HANDLE_VALUE)
        return FALSE;
    return std::fflush((FILE*)h) == 0;
}

// Win32 code spells the bytes-moved out-param as both DWORD* and
// unsigned long* (identical types there, distinct on LP64) - overloads
// cover both.
inline BOOL WriteFile(HANDLE h, const void* data, DWORD len,
                      unsigned long* written, void*)
{
    if (h == nullptr || h == INVALID_HANDLE_VALUE)
        return FALSE;
    const size_t n = std::fwrite(data, 1, len, (FILE*)h);
    std::fflush((FILE*)h);
    if (written) *written = (unsigned long)n;
    return n == len;
}

inline BOOL WriteFile(HANDLE h, const void* data, DWORD len,
                      DWORD* written, void* overlapped)
{
    unsigned long n = 0;
    const BOOL ok = WriteFile(h, data, len, &n, overlapped);
    if (written) *written = (DWORD)n;
    return ok;
}

inline BOOL CloseHandle(HANDLE h)
{
    if (h == nullptr || h == INVALID_HANDLE_VALUE)
        return FALSE;
    if ((FILE*)h == stdout)
        return TRUE;
    std::fclose((FILE*)h);
    return TRUE;
}

inline BOOL   AllocConsole()                { return TRUE; }
inline BOOL   FreeConsole()                 { return TRUE; }
inline HANDLE GetStdHandle(DWORD)           { return (HANDLE)stdout; }
inline BOOL   SetConsoleTitle(const char*)  { return TRUE; }

inline BOOL WriteConsoleA(HANDLE h, const void* data, DWORD len,
                          unsigned long* written, void*)
{
    FILE* f = (h == nullptr || h == INVALID_HANDLE_VALUE) ? stdout : (FILE*)h;
    const size_t n = std::fwrite(data, 1, len, f);
    std::fflush(f);
    if (written) *written = (unsigned long)n;
    return TRUE;
}
#define WriteConsole WriteConsoleA

// Wall-clock time for the Cloud9 RTC device: a real implementation, not
// a stub - the emulated clock should tell true time on every host.
struct SYSTEMTIME
{
    WORD wYear;
    WORD wMonth;
    WORD wDayOfWeek;
    WORD wDay;
    WORD wHour;
    WORD wMinute;
    WORD wSecond;
    WORD wMilliseconds;
};

// Setting the host clock from inside the emulator (Cloud9 RTC write-
// enable) is a Windows-only party trick; on POSIX it needs root and is
// the wrong thing to do anyway. Report failure, change nothing.
inline BOOL SetLocalTime(const SYSTEMTIME*) { return FALSE; }

inline void GetLocalTime(SYSTEMTIME* st)
{
    std::time_t t = std::time(nullptr);
    std::tm tm_buf {};
    localtime_r(&t, &tm_buf);
    st->wYear         = (WORD)(tm_buf.tm_year + 1900);
    st->wMonth        = (WORD)(tm_buf.tm_mon + 1);
    st->wDayOfWeek    = (WORD)tm_buf.tm_wday;
    st->wDay          = (WORD)tm_buf.tm_mday;
    st->wHour         = (WORD)tm_buf.tm_hour;
    st->wMinute       = (WORD)tm_buf.tm_min;
    st->wSecond       = (WORD)tm_buf.tm_sec;
    st->wMilliseconds = 0;
}

// Cartridge modules stay shared libraries off Windows, exactly as the
// cartridge_loader/cpak_cartridge design intends: LoadLibrary is dlopen
// with a name translation, GetProcAddress is dlsym. A Windows-flavored
// module path like "C:/vcc/__bin/Win32/Debug/harddisk.dll" resolves by
// basename to "<dir-of-executable>/harddisk.dylib" (then .so), so the
// same vcc.ini drives both hosts.
inline DWORD GetModuleFileNameA(HMODULE, char* out, DWORD nSize);

inline HMODULE LoadLibraryA(const char* name)
{
    if (name == nullptr)
        return nullptr;
    const char* base = name;
    for (const char* p = name; *p; ++p)
    {
        if (*p == '/' || *p == '\\')
            base = p + 1;
    }
    std::string stem(base);
    const size_t dot = stem.rfind('.');
    if (dot != std::string::npos)
        stem.resize(dot);

    char exe[1024];
    std::string dir;
    if (GetModuleFileNameA(nullptr, exe, sizeof(exe)) > 0)
    {
        dir = exe;
        const size_t slash = dir.rfind('/');
        dir = (slash == std::string::npos) ? "" : dir.substr(0, slash + 1);
    }

    const std::string candidates[] = {
        dir + stem + ".dylib",
        dir + stem + ".so",
        dir + "lib" + stem + ".dylib",
        dir + "lib" + stem + ".so",
        std::string(name),
    };
    for (const auto& candidate : candidates)
    {
        if (void* handle = dlopen(candidate.c_str(), RTLD_NOW | RTLD_LOCAL))
            return handle;
    }
    return nullptr;
}
#define LoadLibrary LoadLibraryA
inline HMODULE GetModuleHandleA(const char*)       { return nullptr; }
#define GetModuleHandle GetModuleHandleA
inline void*   GetProcAddress(HMODULE h, const char* sym)
{
    return h ? dlsym(h, sym) : nullptr;
}
inline BOOL    FreeLibrary(HMODULE h)              { if (h) dlclose(h); return TRUE; }
inline DWORD   GetLastError()                      { return 0; }

// Path of the running executable (module handles other than null have no
// POSIX equivalent and fall back to the executable). Returns the length
// copied, or nSize when truncated - callers that grow-and-retry (see
// fileutil.cpp ModulePath) keep working.
inline DWORD GetModuleFileNameA(HMODULE, char* out, DWORD nSize)
{
    if (out == nullptr || nSize == 0)
        return 0;
    char buf[4096];
#ifdef __APPLE__
    uint32_t cap = sizeof(buf);
    if (_NSGetExecutablePath(buf, &cap) != 0)
        return 0;
#else
    ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n <= 0)
        return 0;
    buf[n] = '\0';
#endif
    size_t len = std::strlen(buf);
    if (len >= nSize)
    {
        std::memcpy(out, buf, nSize - 1);
        out[nSize - 1] = '\0';
        return nSize;
    }
    std::memcpy(out, buf, len + 1);
    return (DWORD)len;
}
#define GetModuleFileName GetModuleFileNameA

// INI-file config storage (the private-profile API). Real
// implementations in libcommon/src/util/host_profile.cpp - same file
// format and semantics as the Win32 API, so existing vcc.ini configs
// carry over between hosts.
DWORD GetPrivateProfileStringA(const char* app, const char* key,
                               const char* def, char* out, DWORD size,
                               const char* file);
UINT  GetPrivateProfileIntA(const char* app, const char* key, int def,
                            const char* file);
BOOL  WritePrivateProfileStringA(const char* app, const char* key,
                                 const char* value, const char* file);
#define GetPrivateProfileString   GetPrivateProfileStringA
#define GetPrivateProfileInt      GetPrivateProfileIntA
#define WritePrivateProfileString WritePrivateProfileStringA

inline BOOL    OpenClipboard(HWND)                 { return FALSE; }
inline BOOL    CloseClipboard()                    { return TRUE; }
inline BOOL    EmptyClipboard()                    { return TRUE; }
inline HANDLE  GetClipboardData(UINT)              { return nullptr; }
inline HGLOBAL GlobalAlloc(UINT, size_t n)         { return std::calloc(1, n ? n : 1); }
inline void*   GlobalLock(HGLOBAL h)               { return h; }
inline BOOL    GlobalUnlock(HGLOBAL)               { return TRUE; }
inline HANDLE  SetClipboardData(UINT, HGLOBAL h)   { std::free(h); return nullptr; }

#endif // _WIN32

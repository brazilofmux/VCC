// Application Windows Messages sent to VCC main message loop

#pragma once
#ifdef _WIN32
#include <Winuser.h>
#else
// No message loop off Windows; the values just need to be distinct.
// SendMessage in host_services.h drops them (menu rebuild / CPU reset
// notifications) until the SDL shell provides a host hook.
#ifndef WM_APP
#define WM_APP 0x8000
#endif
#endif

#define WM_VCC_CPU_RESET    WM_APP+101
#define WM_VCC_UPD_MENU     WM_APP+102

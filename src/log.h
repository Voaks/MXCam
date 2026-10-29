// log.h
//   Tiny thread-safe text log (mxcam.log next to the DLL).
#pragma once

namespace mxcam {

void LogOpen(const wchar_t* path);
void LogClose();

// printf-style. Each call writes one timestamped line.
void Log(const char* format, ...);

// Folder of the log file (with a trailing backslash), for other output files.
const wchar_t* LogDirectory();

}  // namespace mxcam

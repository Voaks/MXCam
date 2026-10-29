@echo off
rem Builds the MXCam opengl32.dll proxy (Release) for both x86 and x64.
setlocal

set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" (
    echo vswhere.exe not found - is Visual Studio 2022 installed?
    exit /b 1
)
for /f "usebackq tokens=*" %%i in (`"%VSWHERE%" -latest -requires Microsoft.Component.MSBuild -find MSBuild\**\Bin\MSBuild.exe`) do set "MSBUILD=%%i"
if not defined MSBUILD (
    echo MSBuild not found.
    exit /b 1
)

"%MSBUILD%" "%~dp0MXCamProxy.vcxproj" /m /nologo /v:minimal /p:Configuration=Release /p:Platform=Win32 || exit /b 1
"%MSBUILD%" "%~dp0MXCamProxy.vcxproj" /m /nologo /v:minimal /p:Configuration=Release /p:Platform=x64 || exit /b 1

echo.
echo Built:
echo   32-bit: %~dp0bin\Win32\Release\opengl32.dll
echo   64-bit: %~dp0bin\x64\Release\opengl32.dll

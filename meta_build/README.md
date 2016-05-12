# vicalib-cmakeall

## Ubuntu 16.04 prerequisites
  ```
  export http_proxy=http://proxy
  export ALL_PROXY=socks5://proxy
  export HTTPS_PROXY=socks5://proxy
  sudo apt-get install build-essential ccache cmake git mesa-common-dev ninja-build
  git config --global http.proxy $http_proxy
  git config --global user.name "<Full Name>"
  git config --global user.email "<email.address>"
  ```
## Ubuntu 16.04 building
  ```
  export PATH=/usr/lib/ccache:$PATH
  git clone https://<hostname>/vicalib-cmakeall.git src
  mkdir build
  cd build
  cmake -GNinja ../src
  ninja
  ```

## Cross-compiling for Android on a Windows Host
  For CMake version 3.7.2
  Ensure the environment variables http_proxy and https_proxy are set
  For example:
  ```
  set http_proxy=http://proxy
  set https_proxy=https://proxy
  ```

  For CMake version 3.9.4
  Ensure the environment variables http_proxy and https_proxy are set
  For example:
  ```
  set ALL_PROXY=socks5://proxy
  set HTTPS_PROXY=socks5://proxy
  ```

  Then if you have ninja installed [available here](https://chocolatey.org/packages/ninja)
  ```
  cmake ..\vicalib-cmakeall -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_ANDROID_ARCH_ABI=armeabi-v7a -DCMAKE_ANDROID_NDK_TOOLCHAIN_VERSION=4.9 -DCMAKE_TOOLCHAIN_FILE=..\vicalib-cmakeall\CMake\Android.toolchain.cmake -DCMAKE_ANDROID_NDK:FILEPATH=<PATH TO NDK DIR>
  ninja
  ```

## Mac

  Install the dependencies using Homebrew.
  ```
  brew install cmake ninja
  ```

  And the *uninstall* (or at least unlink) things that get in the way.
  ```
  for dep in glog gflags freeglut eigen ceres-solver glew protobuf tinyxml2 rapidjson opencv; do brew unlink $dep; done
  ```

  Proxy-wise, external dependencies are picked up via http/https, but internal deps are picked up via SSH, so make sure you
  have `http_proxy`, `https_proxy` and `ALL_PROXY` set but *not* `GIT_SSH` (at least as a proxy).
  ```
  unset GIT_SSH
  export {ALL_PROXY,http{s,}_proxy}=http://proxy
  ```

  If you have XQuartz installed then you use `BUILD_GUI=True`, otherwise set it to `False`.
  ```
  cmake -GNinja -Bbuild -H. -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_GUI=False
  cmake --build build
  ```

## Windows

  Install the dependencies using https://chocolatey.org/.
  ```
  choco install ninja cmake vswhere
  ```
  Install Visual Studio 2015 and Intel MKL https://software.intel.com/en-us/mkl/choose-download

  Tested on Windows 10

  Build
  ```
  for /f "usebackq tokens=*" %i in (`vswhere -latest -property installationPath`) do ("%i/VC/Auxiliary/Build/vcvars64.bat" -vcvars_ver=14.0)
  set INCLUDE=%INCLUDE%;C:\Program Files (x86)\IntelSWTools\compilers_and_libraries\windows\mkl\include
  set LIB=%LIB%;C:\Program Files (x86)\IntelSWTools\compilers_and_libraries\windows\mkl\lib\intel64
  cmake -G Ninja -Bbuild_dir -Hmeta_bulid -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_GUI=False
  cmake --build build_dir --config RelWithDebInfo
  ```


## Windows build agent setup

  The following summerizes a workaround because [Windows OpenSSH](https://github.com/PowerShell/Win32-OpenSSH)  doesn't support Unix sockets yet.

  Internal repos are picked up via ssh and require the ssh-agent to be running for password protected private key.

  Modify start-ssh-agent from the Git for Windows distribution as follows:
  ```diff
--- C:\Program Files\Git\cmd\start-ssh-agent.cmd 2019-04-19 16:52:18.000000000 -0700
+++ C:\Program Files\Git\cmd\start-ssh-agent.cmd 2019-04-19 13:27:44.000000000 -0700
@@ -73,13 +73,13 @@
     )
 )

 :ssh-agent-done
 :failure

-@ENDLOCAL & @SETX "SSH_AUTH_SOCK=%SSH_AUTH_SOCK%" ^
-          & @SETX "SSH_AGENT_PID=%SSH_AGENT_PID%"
+@ENDLOCAL & @SETX SSH_AUTH_SOCK %SSH_AUTH_SOCK% ^
+          & @SETX SSH_AGENT_PID %SSH_AGENT_PID%

 @ECHO %cmdcmdline% | @FINDSTR /l "\"\"" >NUL
 @IF NOT ERRORLEVEL 1 @(
     @CALL cmd %*
 )
 ```
 * This will set the `SSH_AUTH_SOCK` and `SSH_AGENT_PID` at the system level to be available to users over ssh.
 * After making the change call `start-ssh-agent` to add the key.
 * Assuming [Windows OpenSSH](https://github.com/PowerShell/Win32-OpenSSH) is install as a service, restart the sshd with `net restart sshd`
 * Disconnect and reconnect the Jenkins build agent.

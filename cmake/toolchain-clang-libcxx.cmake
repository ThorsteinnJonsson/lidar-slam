# clang-18 + libc++ toolchain.
#
# The stdlib choice lives here (not in add_compile_options) so it propagates to
# sub-builds that receive CMAKE_TOOLCHAIN_FILE, notably Rerun's Arrow
# ExternalProject, which otherwise defaults to libstdc++ and fails to link
# against our libc++ objects. Using *_INIT so these seed the flags rather than
# clobbering anything a build type or preset adds.
set(CMAKE_C_COMPILER clang-18)
set(CMAKE_CXX_COMPILER clang++-18)

set(CMAKE_CXX_FLAGS_INIT "-stdlib=libc++")
set(CMAKE_EXE_LINKER_FLAGS_INIT "-stdlib=libc++ -lc++abi")
set(CMAKE_SHARED_LINKER_FLAGS_INIT "-stdlib=libc++ -lc++abi")
set(CMAKE_MODULE_LINKER_FLAGS_INIT "-stdlib=libc++ -lc++abi")

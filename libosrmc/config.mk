VERSION_MAJOR = 6
VERSION_MINOR = 0
PREFIX ?= /usr/local
TARGET := $(shell echo $$target)

ifeq ($(TARGET),)
    UNAME_S := $(shell uname -s 2>/dev/null)
    ifeq ($(OS),Windows_NT)
        TARGET := mingw
    else ifeq ($(UNAME_S),Darwin)
        TARGET := apple
    else
        TARGET := linux
    endif
else
    ifneq ($(findstring -mingw,$(TARGET)),)
        TARGET := mingw
    else ifneq ($(findstring -apple-,$(TARGET)),)
        TARGET := apple
    else ifneq ($(findstring -linux,$(TARGET)),)
        TARGET := linux
    else
        TARGET := linux
    endif
endif

ifeq ($(TARGET),mingw)
    SHARED_EXT = .dll
    IMPLIB_EXT = .dll.a
    PKG_CONFIG_PATHS = $(PREFIX)/lib/pkgconfig
    PKG_CONFIG_SEP = ;
    LDFLAGS = -shared -Wl,--out-implib,libosrmc$(IMPLIB_EXT)
else ifeq ($(TARGET),apple)
    SHARED_EXT = .dylib
    IMPLIB_EXT =
    PKG_CONFIG_PATHS = $(PREFIX)/lib/pkgconfig
    PKG_CONFIG_SEP = :
    LDFLAGS = -dynamiclib -install_name $(PREFIX)/lib/libosrmc.$(VERSION_MAJOR)$(SHARED_EXT)
else
    SHARED_EXT = .so
    IMPLIB_EXT =
    PKG_CONFIG_PATHS = $(PREFIX)/lib/pkgconfig
    PKG_CONFIG_SEP = :
    LDFLAGS = -shared -Wl,-soname,libosrmc.so.$(VERSION_MAJOR)
endif

CXX ?= g++
CXXFLAGS = -O2 -Wall -Wextra -pedantic -std=c++20 -fvisibility=hidden -fPIC -fno-rtti
CXXFLAGS += $(shell LD_LIBRARY_PATH="" PKG_CONFIG_PATH=$(PKG_CONFIG_PATHS)$(PKG_CONFIG_SEP)$$$$PKG_CONFIG_PATH pkg-config --cflags libosrm 2>/dev/null || true)
ifneq ($(EXTRA_CXXFLAGS),)
    CXXFLAGS += $(EXTRA_CXXFLAGS)
endif

ifeq ($(TARGET),apple)
    LDLIBS = -lc++
else
    LDLIBS = -lstdc++
endif
ifeq ($(TARGET),mingw)
    LDLIBS += -static-libgcc -static-libstdc++
endif
# Get OSRM libraries via pkg-config, restricting search to PREFIX/lib
OSRM_LIBS_RAW := $(shell LD_LIBRARY_PATH="" LIBRARY_PATH="$(PREFIX)/lib" PKG_CONFIG_PATH=$(PKG_CONFIG_PATHS)$(PKG_CONFIG_SEP)$$$$PKG_CONFIG_PATH pkg-config --libs --static libosrm 2>/dev/null || LIBRARY_PATH="$(PREFIX)/lib" PKG_CONFIG_PATH=$(PKG_CONFIG_PATHS)$(PKG_CONFIG_SEP)$$$$PKG_CONFIG_PATH pkg-config --libs libosrm 2>/dev/null || true)
# Convert Boost:: targets to -lboost_* flags and strip absolute paths
# This forces linker to use LIBRARY_PATH, preventing system boost from being used
OSRM_LIBS := $(shell echo "$(OSRM_LIBS_RAW)" | sed -E 's/Boost::date_time/-lboost_date_time/g; s/Boost::iostreams/-lboost_iostreams/g; s/Boost::thread/-lboost_thread/g; s/Boost::regex/-lboost_regex/g; s/Boost::chrono/-lboost_chrono/g; s/Boost::filesystem/-lboost_filesystem/g; s/Boost::[a-z_]*/-lboost_regex/g' | sed -E 's|/[^ ]*\.tbd[^ ]*||g' | sed -E 's|/[^ ]*\.a[^ ]*||g' | sed -E 's|/[^ ]*\.so[^ ]*||g' | sed -E 's|/[^ ]*\.dylib[^ ]*||g')
LDLIBS += $(OSRM_LIBS)

PREFIX = /usr/local

VERSION_MAJOR = 5
VERSION_MINOR = 4

CXXFLAGS = -O2 -Wall -Wextra -pedantic -std=c++20 -fvisibility=hidden -fPIC -fno-rtti $(shell PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:/usr/lib/pkgconfig:$$PKG_CONFIG_PATH pkg-config --cflags libosrm)
LDFLAGS  = -shared -Wl,-soname,libosrmc.so.$(VERSION_MAJOR)
LDLIBS   = -lstdc++ $(shell PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:/usr/lib/pkgconfig:$$PKG_CONFIG_PATH pkg-config --libs libosrm)

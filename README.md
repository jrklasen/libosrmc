# libosrmc

C wrapper around the C++ libosrm library. Useful for writing FFI bindings and guaranteeing ABI stability.

Note: Originally tested against OSRM 5.4 release. Updated and modernized for OSRM 6.0 release

**Notable changes from OSRM 5.4 to 6.0:**
- Requires C++20 standard (updated from C++11)
- JSON API changed: use `std::get<>()` instead of `.get<>()` for accessing JSON values
- JSON Null checking: use `std::holds_alternative<osrm::json::Null>()` instead of `.is<>()`
- StorageConfig constructor now requires `std::filesystem::path` instead of `const char*`

##### Dependencies

- **OSRM 6.0**: libosrm with pkg-config support
- **C++20 compiler**: GCC 10+ or Clang 12+
- **pkg-config**: For discovering OSRM configuration

**Supported platforms:** Linux, macOS, Windows (MinGW)

##### Quick Start

```bash
cd libosrmc
make
sudo make install
```

The build system automatically detects your platform:
- **Linux**: `libosrmc.so`
- **macOS**: `libosrmc.dylib`
- **Windows**: `libosrmc.dll` + `libosrmc.dll.a`

Installation prefix defaults to `/usr/local` (Unix) or `/mingw64` (Windows). Modify `config.mk` to change.

Please refer to [`osrmc/osrmc.h`](https://github.com/daniel-j-h/libosrmc/blob/master/libosrmc/osrmc.h) for library documentation.

##### Todo

- [ ] Remaining Services
- [ ] Callbacks for Responses
- [ ] Use from Language FFIs
- [ ] Make Python Integration Exception-Safe

##### License

Copyright © 2016 Daniel J. Hofmann

Distributed under the MIT License (MIT).

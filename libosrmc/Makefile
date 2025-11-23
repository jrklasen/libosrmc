include config.mk

LIBRARY = libosrmc$(SHARED_EXT)
OBJECTS = osrmc.o
HEADER = osrmc.h

all: $(LIBRARY)

%.o: %.cc $(HEADER)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(LIBRARY): $(OBJECTS) $(HEADER)
	LIBRARY_PATH="$(PREFIX)/lib:$${LIBRARY_PATH}" $(CXX) $(LDFLAGS) -o $@ $(OBJECTS) $(LDLIBS)

install: $(LIBRARY)
	@mkdir -p $(DESTDIR)$(PREFIX)/include/osrmc
	install -m 0644 $(HEADER) $(DESTDIR)$(PREFIX)/include/osrmc
ifeq ($(TARGET),mingw)
	@mkdir -p $(DESTDIR)$(PREFIX)/bin $(DESTDIR)$(PREFIX)/lib
	install -m 0755 $(LIBRARY) $(DESTDIR)$(PREFIX)/bin
	install -m 0755 libosrmc$(IMPLIB_EXT) $(DESTDIR)$(PREFIX)/lib
else
	@mkdir -p $(DESTDIR)$(PREFIX)/lib
	install -m 0755 $(LIBRARY) $(DESTDIR)$(PREFIX)/lib
ifeq ($(TARGET),linux)
	ln -sf $(LIBRARY) $(DESTDIR)$(PREFIX)/lib/$(LIBRARY).$(VERSION_MAJOR)
	ln -sf $(LIBRARY) $(DESTDIR)$(PREFIX)/lib/$(LIBRARY).$(VERSION_MAJOR).$(VERSION_MINOR)
endif
endif

clean:
	@$(RM) $(OBJECTS) $(LIBRARY) libosrmc$(IMPLIB_EXT)

.PHONY: all clean install

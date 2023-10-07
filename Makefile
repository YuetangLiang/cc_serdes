export ARCH ?= arm64
export CROSS_COMPILE ?= /work/ipu04_6010/toolchains/aarch64--glibc--stable-2020.08-1/bin/aarch64-linux-

TARGET_NAME = libmaf_serdes

SONAME=${TARGET_NAME}.so
LIBSO=$(SONAME)
LIBA=$(TARGET_NAME).a
GG=${CROSS_COMPILE}g++
CC=${CROSS_COMPILE}gcc
AR=${CROSS_COMPILE}ar
RANLIB=${CROSS_COMPILE}ranlib
CPPFLAGS=\
	-std=gnu++11 -fpermissive -Wno-narrowing  -Wno-overflow  -Wno-write-strings -Wno-format \
	-I. -I.. -fPIC
CFLAGS=\
	-pthread \
	-I.      \
	-I..     \
	-fPIC

OBJS=\
	$(patsubst   %.c, %.o, $(shell find . -name "*.c")) \
	$(patsubst %.cpp, %.o, $(shell find . -name "*.cpp"))

LDFLAGS = -lc -pthread

.PHONY:	all clean

all: $(LIBA) $(LIBSO)

$(LIBSO): $(OBJS)
	$(CC) $(CFLAGS) -shared -Wl,-soname,$(SONAME)  $(OBJS) $(LDFLAGS) -o $@

$(LIBA): $(OBJS)
	$(AR) cr  $@ $(OBJS)
	$(RANLIB) $@

.c.o:
	$(CC) $(CFLAGS)    -c -o $@ $<

.cpp.o:
	$(GG) $(CPPFLAGS)  -c -o $@ $<

clean:
	rm -f ${OBJS}  $(LIBSO) *~ #$(LIBA)

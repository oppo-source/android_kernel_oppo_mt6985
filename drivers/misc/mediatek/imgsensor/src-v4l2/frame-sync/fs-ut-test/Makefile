# SPDX-License-Identifier: GPL-2.0
# Copyright (C) 2020 MediaTek Inc.

# CROSS_COMPILE = aarch64-linux-gnu-
# CFLAGS = -DFS_UT -Werror -Wall -Wframe-larger-than=512 --static
ASAN = -fsanitize=kernel-address,leak,undefined
CFLAGS = -DFS_UT -Werror -Wall $(ASAN) -Wframe-larger-than=512 -Walloc-zero -Walloca -Wnonnull
LDFLAGS = --static

INCS = -I ../ \
#	   -I ../../../inc/ \

LIBS = -static-libasan -lasan -lpthread

# $(info INCS = $(INCS))

SRCS = ut_fs_test.c \
	   ../frame_sync.c \
	   ../frame_sync_algo.c \
	   ../frame_monitor.c \
	   ../frame_sync_util.c \
	   ../hw_sensor_sync_algo.c \
	   ../custom/custom_hw_sync.c \

#OBJS = ut_fs_test.o \
#	   frame_sync.o \
#	   frame_sync_algo.o \
#	   frame_monitor.o \
#	   frame_sync_util.o \

TARGET = ut_fs_test

all: $(OPTS) $(TARGET)

debug: DEBUG_FLAGS = -g
debug: ut_fs_test

ut_fs_test: $(SRCS)
#	gcc $(LDFLAGS) $(CFLAGS) $(DEBUG_FLAGS) $(INCS) $^ -o $@ $(LIBS)
	gcc $(CFLAGS) $(DEBUG_FLAGS) $(INCS) $^ -o $@ $(LIBS)

# ut_fs_test: $(OBJS)
#	gcc $(LDFLAGS) $(INCS) -o $(LIBS) $@ $^

%.o: %.c
	gcc $(CFLAGS) $(INCS) -c $<

%.o: ../%.c
	gcc $(CFLAGS) $(INCS) $(LIBS) -c $<

clean:
	rm -f *.o $(TARGET)

# 抓窗口用的小工具，源码在这里，二进制不入库（见 .gitignore）。
#
# 为什么需要这些：本机是 Wayland 会话，目标程序跑在 XWayland 上。
# 根窗口读不了（xwd -root / PIL.ImageGrab 都是 BadMatch），只能抓客户窗口；
# 而按标题找窗口会命中 mutter 的框架窗口，按窗口 ID 才可靠。
#
#   make            编译全部
#   make clean      清理

CC      ?= gcc
CFLAGS  ?= -O2 -Wall

# pngrab 用到 XShm 和 zlib；其余只要 X11
BINS = win_grab win_id_resize win_resize xclick pngrab

all: $(BINS)

win_grab: win_grab.c
	$(CC) $(CFLAGS) $< -o $@ -lX11

win_id_resize: win_id_resize.c
	$(CC) $(CFLAGS) $< -o $@ -lX11

win_resize: win_resize.c
	$(CC) $(CFLAGS) $< -o $@ -lX11

# xclick 走 XTEST。系统没有 libXtst-dev，所以直接链运行库；
# 如果装了 libxtst-dev，把下面换成 -lXtst 即可。
XTST = /usr/lib/x86_64-linux-gnu/libXtst.so.6
xclick: xclick.c
	$(CC) $(CFLAGS) $< -o $@ -lX11 $(XTST)

pngrab: pngrab.c
	$(CC) $(CFLAGS) $< -o $@ -lX11 -lXext -lz

clean:
	rm -f $(BINS)

.PHONY: all clean

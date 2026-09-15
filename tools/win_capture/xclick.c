/*
 * xclick.c —— 通过 XTEST 扩展模拟鼠标（移动 / 点击 / 滚轮）。
 *
 * 本机没有 xdotool；libXtst 的**运行时库存在**，但缺开发头文件 XTest.h
 * （装不了，没有 sudo），因此这里自行声明需要的三个 XTst 函数原型并直接
 * 链接 -lXtst。
 *
 * 注意：不能用 XSendEvent 伪造按钮事件 —— Qt 检查 send_event 标志并丢弃
 * 合成事件（实测点击后画面零变化）。XTEST 走服务器输入通路，与真实鼠标
 * 无法区分。
 *
 * 用法：
 *   ./xclick move  <x> <y>
 *   ./xclick click <button> [x y]     # 左键=1
 *   ./xclick wheel <up|down> [n]
 *   ./xclick query
 */

#include <X11/Xlib.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* 自行声明：避免依赖缺失的 X11/extensions/XTest.h */
extern int XTestQueryExtension(Display *dpy, int *event_base, int *error_base,
                               int *major, int *minor);
extern int XTestFakeButtonEvent(Display *dpy, unsigned int button,
                                int is_press, unsigned long delay);
extern int XTestFakeMotionEvent(Display *dpy, int screen, int x, int y,
                                unsigned long delay);

int main(int argc, char **argv) {
    if (argc < 2) { fprintf(stderr, "用法见源码注释\n"); return 2; }
    Display *d = XOpenDisplay(NULL);
    if (!d) { fprintf(stderr, "无法打开 display\n"); return 3; }

    int ev, err, maj, min;
    if (!XTestQueryExtension(d, &ev, &err, &maj, &min)) {
        fprintf(stderr, "服务器不支持 XTEST 扩展\n");
        XCloseDisplay(d);
        return 4;
    }

    const char *cmd = argv[1];
    if (!strcmp(cmd, "move") && argc >= 4) {
        XTestFakeMotionEvent(d, -1, atoi(argv[2]), atoi(argv[3]), CurrentTime);
        XFlush(d);
    } else if (!strcmp(cmd, "click") && argc >= 3) {
        int btn = atoi(argv[2]);
        if (argc >= 5) {
            XTestFakeMotionEvent(d, -1, atoi(argv[3]), atoi(argv[4]), CurrentTime);
            XFlush(d); usleep(180000);
        }
        XTestFakeButtonEvent(d, btn, True, CurrentTime);  XFlush(d);
        usleep(90000);
        XTestFakeButtonEvent(d, btn, False, CurrentTime); XFlush(d);
    } else if (!strcmp(cmd, "wheel") && argc >= 3) {
        int btn = (!strcmp(argv[2], "up")) ? 4 : 5;
        int n = (argc >= 4) ? atoi(argv[3]) : 1;
        for (int i = 0; i < n; i++) {
            XTestFakeButtonEvent(d, btn, True, CurrentTime);  XFlush(d);
            usleep(50000);
            XTestFakeButtonEvent(d, btn, False, CurrentTime); XFlush(d);
            usleep(60000);
        }
    } else if (!strcmp(cmd, "query")) {
        Window r, c; int rx, ry, wx, wy; unsigned int m;
        XQueryPointer(d, DefaultRootWindow(d), &r, &c, &rx, &ry, &wx, &wy, &m);
        printf("%d %d\n", rx, ry);
    } else {
        fprintf(stderr, "参数不对\n"); XCloseDisplay(d); return 2;
    }
    XFlush(d);
    usleep(60000);
    XCloseDisplay(d);
    return 0;
}

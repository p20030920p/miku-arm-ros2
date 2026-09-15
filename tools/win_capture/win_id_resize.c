/*
 * win_id_resize.c —— 按窗口 ID 调整尺寸（绕过窗口管理器的框架窗口）。
 *
 * win_resize 按标题找窗口时会命中 mutter 的框架窗口，调整它无效。
 * 这里直接指定客户端窗口 ID，用于修掉 Qt 给出的不合理尺寸
 * （实测 RViz 在无窗口管理器约束下得到 1600x1600，3D 视图被挤成窄条）。
 *
 * 用法：./win_id_resize <0xID> <宽> <高>
 */
#include <X11/Xlib.h>
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char **argv) {
    if (argc < 4) { fprintf(stderr, "用法: %s <0xID> <宽> <高>\n", argv[0]); return 2; }
    Display *d = XOpenDisplay(NULL);
    if (!d) { fprintf(stderr, "无法打开 display\n"); return 3; }
    Window w = (Window)strtoul(argv[1], NULL, 0);
    int W = atoi(argv[2]), H = atoi(argv[3]);
    XMoveResizeWindow(d, w, 0, 0, W, H);
    XFlush(d);
    XWindowAttributes a;
    if (!XGetWindowAttributes(d, w, &a)) { fprintf(stderr, "取属性失败\n"); XCloseDisplay(d); return 4; }
    printf("窗口 0x%lx -> %dx%d\n", w, a.width, a.height);
    XCloseDisplay(d);
    return 0;
}

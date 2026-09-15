/*
 * win_resize.c —— 按窗口标题调整 X 窗口尺寸。
 *
 * 本机没有窗口管理器（Wayland 会话，GNOME Shell 管的是 Wayland 客户端），
 * RViz 作为 XWayland 客户端拿到的主窗口尺寸可能不合理（实测 225x1820），
 * 3D 视图被挤成一条。这里直接调用 XResizeWindow 修正。
 *
 * 用法：./win_resize "<标题子串>" <宽> <高>
 */
#include <X11/Xlib.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static Window find(Display *d, Window w, const char *needle, int *best_area) {
    Window best = 0;
    char *name = NULL;
    if (XFetchName(d, w, &name) && name) {
        if (strstr(name, needle)) {
            XWindowAttributes a;
            if (XGetWindowAttributes(d, w, &a)) {
                int area = a.width * a.height;
                if (area > *best_area) { *best_area = area; best = w; }
            }
        }
        XFree(name);
    }
    Window root, parent, *kids = NULL;
    unsigned int n = 0;
    if (XQueryTree(d, w, &root, &parent, &kids, &n)) {
        for (unsigned int i = 0; i < n; i++) {
            int sub = 0;
            Window r = find(d, kids[i], needle, &sub);
            if (r && sub > *best_area) { *best_area = sub; best = r; }
        }
        if (kids) XFree(kids);
    }
    return best;
}

int main(int argc, char **argv) {
    if (argc < 4) { fprintf(stderr, "用法: %s <标题子串> <宽> <高>\n", argv[0]); return 2; }
    Display *d = XOpenDisplay(NULL);
    if (!d) { fprintf(stderr, "无法打开 display\n"); return 3; }
    int area = 0;
    Window w = find(d, DefaultRootWindow(d), argv[1], &area);
    if (!w) { fprintf(stderr, "未找到窗口 '%s'\n", argv[1]); XCloseDisplay(d); return 4; }
    int W = atoi(argv[2]), H = atoi(argv[3]);
    XMoveResizeWindow(d, w, 0, 0, W, H);
    XFlush(d);
    XWindowAttributes a;
    XGetWindowAttributes(d, w, &a);
    printf("OK 窗口 0x%lx -> %dx%d (请求 %dx%d)\n", w, a.width, a.height, W, H);
    XCloseDisplay(d);
    return 0;
}

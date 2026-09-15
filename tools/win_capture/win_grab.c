/*
 * win_grab.c —— 按窗口标题截取指定 X 窗口，输出 PPM。
 *
 * 为什么不用 xwd / ImageMagick：
 *   本机是 Wayland 会话 + XWayland。XWayland **禁止读回 root 窗口**
 *   （XGetImage 返回 BadMatch），所以 xwd -root、PIL.ImageGrab 全都失败。
 *   但读回**客户端自己的窗口**是允许的（实测可行），因此这里按标题找到目标
 *   窗口再读它的像素。
 *
 * 用法：
 *   ./win_grab "<窗口标题子串>" out.ppm [max_width]
 * 退出码：0 成功，非 0 表示未找到窗口或读回失败。
 */
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static Window find_by_title(Display *d, Window win, const char *needle, int depth) {
    char *name = NULL;
    if (XFetchName(d, win, &name) && name) {
        int hit = strstr(name, needle) != NULL;
        XFree(name);
        if (hit) return win;
    }
    Window root, parent, *kids = NULL;
    unsigned int n = 0;
    if (XQueryTree(d, win, &root, &parent, &kids, &n)) {
        for (unsigned int i = 0; i < n; i++) {
            Window r = find_by_title(d, kids[i], needle, depth + 1);
            if (r) { if (kids) XFree(kids); return r; }
        }
        if (kids) XFree(kids);
    }
    return 0;
}

int main(int argc, char **argv) {
    if (argc < 3) { fprintf(stderr, "用法: %s <标题子串> <out.ppm> [max_w]\n", argv[0]); return 2; }
    const char *needle = argv[1];
    Display *d = XOpenDisplay(NULL);
    if (!d) { fprintf(stderr, "无法打开 display\n"); return 3; }

    Window w = find_by_title(d, DefaultRootWindow(d), needle, 0);
    if (!w) { fprintf(stderr, "未找到标题含 '%s' 的窗口\n", needle); XCloseDisplay(d); return 4; }

    XWindowAttributes a;
    if (!XGetWindowAttributes(d, w, &a)) { fprintf(stderr, "取窗口属性失败\n"); XCloseDisplay(d); return 5; }
    int ww = a.width, wh = a.height;
    int max_w = (argc > 3) ? atoi(argv[3]) : 0;

    XImage *img = XGetImage(d, w, 0, 0, ww, wh, AllPlanes, ZPixmap);
    if (!img) { fprintf(stderr, "XGetImage 失败（窗口 %dx%d）\n", ww, wh); XCloseDisplay(d); return 6; }

    /* 目标尺寸（可选缩放：最近邻，够用且无需额外依赖） */
    int ow = (max_w > 0 && max_w < ww) ? max_w : ww;
    int oh = (max_w > 0 && max_w < ww) ? (int)((long)wh * max_w / ww) : wh;

    FILE *f = fopen(argv[2], "wb");
    if (!f) { fprintf(stderr, "无法写入 %s\n", argv[2]); XDestroyImage(img); XCloseDisplay(d); return 7; }
    fprintf(f, "P6\n%d %d\n255\n", ow, oh);

    for (int y = 0; y < oh; y++) {
        int sy = (ow != ww) ? (int)((long)y * wh / oh) : y;
        for (int x = 0; x < ow; x++) {
            int sx = (ow != ww) ? (int)((long)x * ww / ow) : x;
            unsigned long p = XGetPixel(img, sx, sy);
            unsigned char rgb[3] = {
                (unsigned char)((p & img->red_mask)   >> 16),
                (unsigned char)((p & img->green_mask) >> 8),
                (unsigned char)( p & img->blue_mask)
            };
            fwrite(rgb, 1, 3, f);
        }
    }
    fclose(f);
    printf("OK %dx%d -> %s (%dx%d)\n", ww, wh, argv[2], ow, oh);
    XDestroyImage(img);
    XCloseDisplay(d);
    return 0;
}

/* ---------------------------------------------------------------------------
 * 同一文件里的第二个工具入口：win_resize
 *   ./win_resize <标题子串> <宽> <高>
 * XWayland 下 Qt 可能给出不合理的主窗口尺寸（实测 RViz 得到 225x1820），
 * 而本机没有窗口管理器，所以用 XResizeWindow 直接改。
 * 通过 win_grab 的同名可执行文件以 -r 参数调用，避免多一个源文件。
 * ------------------------------------------------------------------------- */

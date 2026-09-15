/*
 * pngrab.c —— 抓指定窗口的一块区域，直接写成 PNG。
 *
 * 用 XShmGetImage 走共享内存：1900x1233 单帧 5 ms。普通的 XGetImage
 * 同一块区域要 126 ms（XWayland 下每行单独走协议），录演示帧率不够。
 * 共享内存不可用时自动退回 XGetImage。
 *
 * 用法：./pngrab <0xID> <out.png|-> [x0 y0 x1 y1]
 *       输出名写 - 时把 RGB24 原始字节写到 stdout，由 Python 侧用 PIL 存
 *       （自己写 PNG 容易踩 chunk/CRC 的坑，交给 Pillow 更稳）。
 *       x0 y0 x1 y1 缺省表示整个客户区。
 */
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/extensions/XShm.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <zlib.h>

static void put32(unsigned char *p, unsigned v) {
    p[0] = (unsigned char)(v >> 24); p[1] = (unsigned char)(v >> 16);
    p[2] = (unsigned char)(v >> 8);  p[3] = (unsigned char)v;
}

/* 极简 PNG 写入：8 位 RGB，无隔行 */
static int write_png(const char *path, const unsigned char *rgb, int w, int h) {
    FILE *f = fopen(path, "wb");
    if (!f) return -1;

    unsigned char hdr[13];
    put32(hdr, (unsigned)w);
    put32(hdr + 4, (unsigned)h);
    hdr[8] = 8;   /* bit depth */
    hdr[9] = 2;   /* color type: truecolor */
    hdr[10] = 0; hdr[11] = 0; hdr[12] = 0;

    /* 每行前面加一个 filter 字节 0 */
    size_t rawlen = (size_t)h * (size_t)(w * 3 + 1);
    unsigned char *raw = malloc(rawlen);
    if (!raw) { fclose(f); return -1; }
    for (int y = 0; y < h; y++) {
        raw[(size_t)y * (w * 3 + 1)] = 0;
        memcpy(raw + (size_t)y * (w * 3 + 1) + 1, rgb + (size_t)y * w * 3, (size_t)w * 3);
    }
    uLongf clen = compressBound((uLong)rawlen);
    unsigned char *comp = malloc(clen);
    if (!comp) { free(raw); fclose(f); return -1; }
    if (compress2(comp, &clen, raw, (uLong)rawlen, 6) != Z_OK) {
        free(raw); free(comp); fclose(f); return -1;
    }
    free(raw);

    unsigned char sig[8] = {137, 80, 78, 71, 13, 10, 26, 10};
    fwrite(sig, 1, 8, f);

    /* IHDR */
    unsigned char chunk[64];
    unsigned len = 13;
    put32(chunk, len);
    memcpy(chunk + 4, "IHDR", 4);
    memcpy(chunk + 8, hdr, 13);
    unsigned long crc = crc32(0L, Z_NULL, 0);
    crc = crc32(crc, chunk + 4, 4 + 13);
    put32(chunk + 21, (unsigned)crc);
    fwrite(chunk, 1, 25, f);

    /* IDAT：chunk 头在 chunk[0..11]（长度+类型+CRC），压缩数据另写 comp。
     * 注意别把 CRC 写到 chunk+0，那会盖掉长度字段的前 4 字节。 */
    put32(chunk, (unsigned)clen);
    memcpy(chunk + 4, "IDAT", 4);
    crc = crc32(0L, Z_NULL, 0);
    crc = crc32(crc, chunk + 4, 4);
    crc = crc32(crc, comp, (uInt)clen);
    put32(chunk + 8, (unsigned)crc);
    fwrite(chunk, 1, 12, f);
    fwrite(comp, 1, clen, f);
    free(comp);

    /* IEND */
    put32(chunk, 0);
    memcpy(chunk + 4, "IEND", 4);
    crc = crc32(0L, Z_NULL, 0);
    crc = crc32(crc, chunk + 4, 4);
    put32(chunk + 8, (unsigned)crc);
    fwrite(chunk, 1, 12, f);

    fclose(f);
    return 0;
}

int main(int argc, char **argv) {
    if (argc < 3) {
        fprintf(stderr, "用法: %s <0xID> <out.png> [x0 y0 x1 y1]\n", argv[0]);
        return 2;
    }
    Display *d = XOpenDisplay(NULL);
    if (!d) { fprintf(stderr, "无法打开 display\n"); return 3; }
    Window w = (Window)strtoul(argv[1], NULL, 0);

    XWindowAttributes a;
    if (!XGetWindowAttributes(d, w, &a)) {
        fprintf(stderr, "取窗口属性失败\n"); XCloseDisplay(d); return 4;
    }
    int x0 = 0, y0 = 0, x1 = a.width, y1 = a.height;
    if (argc >= 7) {
        x0 = atoi(argv[3]); y0 = atoi(argv[4]);
        x1 = atoi(argv[5]); y1 = atoi(argv[6]);
        if (x0 < 0) x0 = 0;
        if (y0 < 0) y0 = 0;
        if (x1 > a.width) x1 = a.width;
        if (y1 > a.height) y1 = a.height;
    }
    int W = x1 - x0, H = y1 - y0;
    if (W <= 0 || H <= 0) { fprintf(stderr, "裁剪区域无效\n"); XCloseDisplay(d); return 5; }

    unsigned char *rgb = malloc((size_t)W * H * 3);
    if (!rgb) { XCloseDisplay(d); return 7; }

    XImage *img = NULL;
    XShmSegmentInfo shm;
    int use_shm = 0;
    memset(&shm, 0, sizeof(shm));

    if (XShmQueryExtension(d)) {
        img = XShmCreateImage(d, DefaultVisual(d, 0), (unsigned)a.depth, ZPixmap,
                              NULL, &shm, (unsigned)W, (unsigned)H);
        if (img) {
            shm.shmid = shmget(IPC_PRIVATE, (size_t)img->bytes_per_line * H,
                               IPC_CREAT | 0600);
            if (shm.shmid >= 0) {
                shm.shmaddr = img->data = shmat(shm.shmid, NULL, 0);
                shm.readOnly = False;
                if (shm.shmaddr != (char *)-1 && XShmAttach(d, &shm)) {
                    use_shm = 1;
                } else {
                    XDestroyImage(img); img = NULL;
                }
            } else {
                XDestroyImage(img); img = NULL;
            }
        }
    }
    if (use_shm) {
        if (!XShmGetImage(d, w, img, x0, y0, AllPlanes)) {
            fprintf(stderr, "XShmGetImage 失败\n");
            free(rgb); XCloseDisplay(d); return 6;
        }
    } else {
        img = XGetImage(d, w, x0, y0, (unsigned)W, (unsigned)H, AllPlanes, ZPixmap);
        if (!img) { fprintf(stderr, "XGetImage 失败\n"); free(rgb); XCloseDisplay(d); return 6; }
    }

    int bpp = img->bits_per_pixel / 8;
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            const unsigned char *p = (const unsigned char *)img->data +
                                     (size_t)y * img->bytes_per_line + (size_t)x * bpp;
            unsigned char *o = rgb + ((size_t)y * W + x) * 3;
            o[0] = p[2]; o[1] = p[1]; o[2] = p[0];
        }
    }
    int rc;
    if (strcmp(argv[2], "-") == 0) {
        rc = (fwrite(rgb, 3, (size_t)W * H, stdout) == (size_t)W * H) ? 0 : -1;
    } else {
        rc = write_png(argv[2], rgb, W, H);
    }
    free(rgb);
    XCloseDisplay(d);
    if (rc != 0) { fprintf(stderr, "写 PNG 失败\n"); return 8; }
    printf("%dx%d %s -> %s\n", W, H, use_shm ? "shm" : "xget", argv[2]);
    return 0;
}

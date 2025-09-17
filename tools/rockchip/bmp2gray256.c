/*
 * (C) Copyright 2020 Rockchip Electronics Co., Ltd
 *
 * SPDX-License-Identifier:     GPL-2.0+
 * Author: Wenping Zhang <wenping.zhang@rock-chips.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <rk_eink.h>

struct bmp_header {
    /* Bitmap File Header */
    char signature[2];         // 位图文件的类型，必须为BMP (2个字节)
    uint32_t file_size;        // 位图文件的大小，以字节为单位 (4个字节)
    uint32_t reserved;         // 位图文件保留字，必须为0 (4个字节)
    uint32_t data_offset;      // 位图数据的起始位置，以相对于位图 (4个字节)

    /* Bitmap InfoHeader */
    uint32_t size;             // 位图信息头所占用字节数 (4个字节)　
    uint32_t width;            // 位图的宽度，以像素为单位 (4个字节)
    uint32_t height;           // 位图的高度，以像素为单位 (4个字节)
    uint16_t planes;           // 目标设备的级别，必须为1 (2个字节)
    uint16_t bit_count;        // 每个像素所需的位数，必须是1(双色)、4(16色)、8(256色)、24(真彩色)或32(增强真彩色)之一 (2个字节)　　
    uint32_t compression;      // 位图压缩类型，必须是 0(不压缩)、 1(BI_RLE8压缩类型)或2(BI_RLE4压缩类型)之一) (4个字节)
    uint32_t image_size;       // 位图的大小，以字节为单位 (4个字节)　
    uint32_t x_pixels_per_m;   // 位图水平分辨率，每米像素数 (4个字节)
    uint32_t y_pixels_per_m;   // 位图垂直分辨率，每米像素数 (4个字节)
    uint32_t colors_used;      // 位图实际使用的颜色表中的颜色数 (4个字节)
    uint32_t colors_important; // 位图显示过程中重要的颜色数(4个字节)

    /* ColorTable */           // 颜色表，个数由BitCount来确定，当biBitCount=1, 4, 8时，分别有2, 16, 256个表项，当biBitCount=24时，没有颜色表项

    /* Pixels */               // 像素数据，存储顺序是行内是从左到右，扫描行之间是从下到上。位图的一个像素值所占的字节数如下：
    // 当BitCount=1时，8个像素占1个字节;
    // 当BitCount=4时，2个像素占1个字节;
    // 当BitCount=8时，1个像素占1个字节;
    // 当BitCount=24时，1个像素占3个字节，分别是B、G、R;
} __attribute__((packed));

struct bmp_image {
    struct bmp_header hdr;
    uint8_t color_table[0];
};

struct pixel_u16 {
    uint16_t blue: 4;
    uint16_t green: 4;
    uint16_t red: 4;
    uint16_t alpha: 4;
} __attribute__((packed));

struct pixel_u24 {
    uint8_t blue;
    uint8_t green;
    uint8_t red;
} __attribute__((packed));

struct pixel_u32 {
    uint8_t blue;
    uint8_t green;
    uint8_t red;
    uint8_t alpha;
} __attribute__((packed));

/*
 * Every part of logo.img must be aligned in RK_BLK_SIZE,
 * use macro aligned_in_blk to calculate the the real size
 */
#define RK_BLK_SIZE          512
#define ALIGN(x, y)          (((x) + (y) - 1) & ~((y) - 1))

uint32_t screen_w;
uint32_t screen_h;

static inline int size_of_image(type)
{
    if (0xFFFF == type) {
        return screen_w * screen_h;
    }

    return (screen_w * screen_h) >> 1;
}

static int get_logo_resolution(char *in_path, uint32_t *logo_width, uint32_t *logo_height)
{
    struct bmp_header bmp_hdr;
    FILE *file;
    int size;
    int ret = 0;

    if (!in_path) {
        fprintf(stderr, "Invalid bmp file path.\n");
    }

    file = fopen(in_path, "rb");
    if (!file) {
        fprintf(stderr, "File %s open failed.\n", in_path);
        return -1;
    }
    size = sizeof(struct bmp_header);
    if (size != fread(&bmp_hdr, 1, size, file)) {
        fprintf(stderr, "Incomplete read of file %s.\n", in_path);
        ret = -1;
        goto out;
    }
    if (!(bmp_hdr.signature[0] == 'B' && bmp_hdr.signature[1] == 'M')) {
        printf("cat not find bmp file\n");
        ret = -1;
        goto out;
    }
    *logo_width = bmp_hdr.width;
    *logo_height = bmp_hdr.height;
    fprintf(stderr, "logo resolution is %d x %d.\n", *logo_width, *logo_height);
out:
    fclose(file);
    return ret;
}

/*
 * The bmp pixel is scan from left-bottom to right-top
 */
static int convert_bmp_idx_to_gray_idx(int idx, uint32_t w, uint32_t h, int type)
{
    int row = h - (idx / w) - 1;

    if (0xFFFF == type) {
        return (row * w + idx % w);
    }

    return (row * w + idx % w) / 2;
}

static int convert_one_image(char *in_path, void *out_buf, int type)
{
    struct bmp_image *bmp;
    struct bmp_header *bmp_hdr;
    FILE *file;
    void *bmp_buf;
    int size;
    int ret = -1;
    uint8_t *gr16_data = (uint8_t *)out_buf;

    if (!out_buf || !in_path) {
        fprintf(stderr, "in_path or out_buf is NULL.\n");
        return -1;
    }

    file = fopen(in_path, "rb");
    if (!file) {
        fprintf(stderr, "File %s open failed.\n", in_path);
        return -1;
    }

    fseek(file, 0, SEEK_END);
    size = ftell(file);
    fseek(file, 0, SEEK_SET);

    bmp_buf = calloc(1, size);
    if (!bmp_buf) {
        fprintf(stderr, "Allocate memory of %d bytes failed.\n", size);
        fclose(file);
        return -1;
    }
    if (size != fread(bmp_buf, 1, size, file)) {
        fprintf(stderr, "Incomplete read of file %s.\n", in_path);
        goto out;
    }

    bmp = (struct bmp_image *)bmp_buf;
    bmp_hdr = &bmp->hdr;
    if (!(bmp_hdr->signature[0] == 'B' && bmp_hdr->signature[1] == 'M')) {
        printf("cat not find bmp file\n");
        goto out;
    }

    if (size != le32_to_cpu(bmp_hdr->file_size)) {
        fprintf(stderr, "Invalid BMP file size %d.\n", le32_to_cpu(bmp_hdr->file_size));
        goto out;
    }
    printf("bmp_hdr->width=%d, bmp_hdr->height=%d\n", bmp_hdr->width, bmp_hdr->height);
    printf("screen_w=%d, screen_h=%d\n", screen_w, screen_h);
    if (le32_to_cpu(bmp_hdr->width) != screen_w || le32_to_cpu(bmp_hdr->height) != screen_h) {
        fprintf(stderr, "The image size must same with others.\n");
        goto out;
    }

    printf("bmp depth is %d\n", bmp_hdr->bit_count);
    //convert rgb to gray data, and write to output buffer
    // the used algorithm please refer to below url:
    // https://www.cnblogs.com/zhangjiansheng/p/6925722.html
    // we use below algorithm:
    // Gray = (R*19595 + G*38469 + B*7472) >> 16
    switch (bmp_hdr->bit_count) {
        case 16: {
            struct pixel_u16 *color_u16;
            int i;

            color_u16 = (struct pixel_u16 *)bmp->color_table;
            if (0xFFFF == type) {
                for (i = 0; i < screen_w * screen_h; i++) {
                    struct pixel_u16 *pix = &color_u16[i];
                    int j = convert_bmp_idx_to_gray_idx(i, screen_w, screen_h, type);
                    uint32_t gray_px = (pix->red * 19595 + pix->green * 38469 + pix->blue * 7472) >> 12;
                    gr16_data[j] = gray_px;
                }
            } else {
                for (i = 0; i < screen_w * screen_h / 2; i++) {
                    struct pixel_u16 *pix1 = &color_u16[2 * i];
                    struct pixel_u16 *pix2 = &color_u16[2 * i + 1];
                    int j = convert_bmp_idx_to_gray_idx(2 * i, screen_w, screen_h, type);
                    /*
                    * the rgb value of pixel_u16 is 4 bits,
                    * so the counted grayscale value is 4bit
                    */
                    uint32_t gray_px1 = (pix1->red * 19595 + pix1->green * 38469 + pix1->blue * 7472) >> 16;
                    uint32_t gray_px2 = (pix2->red * 19595 + pix2->green * 38469 + pix2->blue * 7472) >> 16;
                    gr16_data[j] = gray_px1 | (gray_px2 << 4);
                }
            }
        }
        break;
        case 24: {
            struct pixel_u24 *color_u24;
            int i;

            color_u24 = (struct pixel_u24 *)bmp->color_table;
            if (0xFFFF == type) {
                for (i = 0; i < screen_w * screen_h; i++) {
                    struct pixel_u24 *pix = &color_u24[i];
                    int j = convert_bmp_idx_to_gray_idx(i, screen_w, screen_h, type);
                    uint32_t gray_px = ((pix->red * 19595 + pix->green * 38469 + pix->blue * 7472) >> 16);
                    gr16_data[j] = gray_px;
                }
            } else {
                for (i = 0; i < screen_w * screen_h / 2; i++) {
                    struct pixel_u24 *pix1 = &color_u24[2 * i];
                    struct pixel_u24 *pix2 = &color_u24[2 * i + 1];
                    int j = convert_bmp_idx_to_gray_idx(2 * i, screen_w, screen_h, type);
                    /*
                    * The rgb value of pixel_u24 is 8 bits,
                    * so the counted grayscale
                    * value need to divide into 16
                    */
                    uint32_t gray_px1 = ((pix1->red * 19595 + pix1->green * 38469 + pix1->blue * 7472) >> 16) >> 4;
                    uint32_t gray_px2 = ((pix2->red * 19595 + pix2->green * 38469 + pix2->blue * 7472) >> 16) >> 4;
                    gr16_data[j] = gray_px1 | (gray_px2 << 4);
                }
            }
        }
        break;
        case 32: {
            struct pixel_u32 *color_u32;
            int i;

            color_u32 = (struct pixel_u32 *)bmp->color_table;
            if (0xFFFF == type) {
                for (i = 0; i < screen_w * screen_h; i++) {
                    struct pixel_u32 *pix = &color_u32[i];
                    int j = convert_bmp_idx_to_gray_idx(i, screen_w, screen_h, type);
                    uint32_t gray_px = ((pix->red * 19595 + pix->green * 38469 + pix->blue * 7472) >> 16);
                    gr16_data[j] = gray_px;
                }
            } else {
                for (i = 0; i < screen_w * screen_h / 2; i++) {
                    struct pixel_u32 *pix1 = &color_u32[2 * i];
                    struct pixel_u32 *pix2 = &color_u32[2 * i + 1];
                    int j = convert_bmp_idx_to_gray_idx(2 * i, screen_w, screen_h, type);
                    /*
                    * The rgb value of pixel_u32 is 8 bits,
                    * so the counted grayscale
                    * value need to divide into 16
                    */
                    uint32_t gray_px1 = ((pix1->red * 19595 + pix1->green * 38469 + pix1->blue * 7472) >> 16) >> 4;
                    uint32_t gray_px2 = ((pix2->red * 19595 + pix2->green * 38469 + pix2->blue * 7472) >> 16) >> 4;
                    gr16_data[j] = gray_px1 | (gray_px2 << 4);
                }
            }
        }
        break;
        default:
            ret = -1;
            printf("Invalid bit count[%d],only support 16/24/32 bpp bmp\n", bmp_hdr->bit_count);
            break;
    }

    fprintf(stderr, "Convert image success\n");
    ret = 0;
out:
    free(bmp_buf);
    fclose(file);
    return ret;
}

static void *init_grayscale_logo_buf(int size)
{
    void *out_buf;

    fprintf(stderr, "size of out put buf is %d\n", size);
    out_buf = calloc(1, size);
    return out_buf;
}

static void deinit_grayscale_logo_buf(void *buf)
{
    if (buf) {
        free(buf);
        buf = NULL;
    }
}

static int image_mirror(unsigned char *in_buf, unsigned char *out_buf, unsigned short w, unsigned short h)
{
    int i;

    if (!in_buf || !out_buf) {
        printf("mirror in buffer or out buffer is NULL\n");
        return -1;
    }

    for (i = 0; i < h; i++) {
        unsigned short column_len = w / 2;
        unsigned char *column_in = in_buf + i * column_len;
        unsigned char *column_out = out_buf + (h - i - 1) * column_len;

        memcpy(column_out, column_in, column_len);
    }

    return 0;
}


static void save(unsigned char *buf, unsigned int size)
{
    char *path = "./logo.bin";
    int ret = -1;

    FILE *file = fopen(path, "wb+");
    if (!file) {
        fprintf(stderr, "File %s open failed.\n", path);
        return;
    }

    ret = fwrite(buf, size, 1, file);
    if (ret != 1) {
        fprintf(stderr, "write image to file %s failed\n", path);
    }

    fclose(file);
}

int main(int argc, char *argv[])
{
    char in_path[256] = {0};
    void *out_buf, *mirror_buf;
    int ret = -1, type = 0xFFFF, img_size = 0, mirror = 0;

    fprintf(stderr, "The command line has %d arguments :\n", argc - 1);
    for (int count = 1; count < argc; ++count) {
        fprintf(stderr, "%d: %s\n", count, argv[count]);
    }
    if(argc-1 == 0) {
        printf("must input bmp filepath!!\n");
        return -1;
    }
    if (2 <= (argc - 1)) {
        int len = strlen(argv[1]);
        if (len > 256) {
            printf("transfer logo path %s is too long\n", argv[0]);
            return -1;
        }
        memcpy(in_path, argv[1], len);
        type = strtol(argv[2], NULL, 16);
        if (3 == (argc - 1)) {
            mirror = atoi(argv[3]);
        }
    } else {
        fprintf(stderr, "transfer logo args error!\n");
        return -1;
    }

    fprintf(stderr, "transfer logo image: %s type: 0x%x mirror: %d\n", in_path, type, mirror);

    ret = get_logo_resolution(in_path, &screen_w, &screen_h);
    if (ret < 0) {
        fprintf(stderr, "Get height and width from logo image failed.\n");
        return -1;
    }

    if (screen_w == 0 || screen_h == 0) {
        fprintf(stderr, "The screen weight and screen height must be set.\n");
        return -1;
    }

    img_size = size_of_image(type);
    out_buf = init_grayscale_logo_buf(img_size);
    if (!out_buf) {
        fprintf(stderr, "Can't malloc buffer for grayscale image.\n");
        return -1;
    }

    fprintf(stderr, "screen size=%d.\n", screen_w * screen_h);
    ret = convert_one_image(in_path, out_buf, type);
    if (ret < 0) {
        printf("Convert image failed, type is %d\n", type);
        goto exit;
    }

    if (mirror) {
        mirror_buf = init_grayscale_logo_buf(img_size);
        if (!mirror_buf) {
            fprintf(stderr, "Can't malloc mirror buffer for grayscale image.\n");
            return -1;
        }
        image_mirror((unsigned char *)out_buf, (unsigned char *)mirror_buf, screen_w, screen_h);
        save(mirror_buf, img_size);
    } else {
        save(out_buf, img_size);
    }

exit:
    deinit_grayscale_logo_buf(out_buf);

    return ret;
}


#ifndef _IMAGE_LIB_H_
#define _IMAGE_LIB_H_

#define IMAGE_LIB_USING_NEON

#ifdef IMAGE_LIB_USING_NEON
#include <arm_neon.h>
#endif

enum IMAGE_TYPE{
    IMAGE_TYPE_8U  = 0x11,
    IMAGE_TYPE_8S  = 0x21,
    IMAGE_TYPE_16U = 0x32,
    IMAGE_TYPE_16S = 0x42,
    IMAGE_TYPE_32U = 0x54,
    IMAGE_TYPE_32S = 0x64,
    IMAGE_TYPE_32F = 0x74,
};

struct matrix_s{
    int type;
    int channel;
    int cols;
    int rows;
    unsigned char *data;
};

struct point2f{
    float x;
    float y;
};

struct point3f{
    float x;
    float y;
    float z;
};

struct point2i{
    int x;
    int y;
};

struct size2i{
    int x;
    int y;
};

void  matrix_create(struct matrix_s *mat,int cols,int rows,int channel,int type);
void  matrix_init(struct matrix_s *img,int cols,int rows,int channel,int type,unsigned char *buffer);
int   make_deriv_kernel(struct matrix_s *kx,struct matrix_s *ky,int dx,int dy,int size);
int   matrix_calc_pixel_deriv(struct matrix_s *img,int x,int y,struct matrix_s *kx,struct matrix_s *ky);
int   matrix_copy_aera(struct matrix_s *src,struct matrix_s *dst,struct point2i *start,struct size2i *size);
int   matrix_convert_type(struct matrix_s *src,struct matrix_s *dst);
int   matrix_filter_x(struct matrix_s *src,struct matrix_s *dst,struct matrix_s *kernel,float scale);
int   matrix_filter_y(struct matrix_s *src,struct matrix_s *dst,struct matrix_s *kernel,float scale);
float matrix_get_pixel_val(struct matrix_s *img,float x,float y);
/*  scale image to 1/2  */
int   matrix_binning(struct matrix_s *src,struct matrix_s *dst);
int   matrix_sobel(struct matrix_s *src,struct matrix_s *dst,int dx,int dy);
int   matrix_copy_channel(struct matrix_s *src,struct matrix_s *dst,int src_ch,int dst_ch);
void  print_memory_int(void *addr,int len);
void  matrix_destroy(struct matrix_s *mat);

#ifdef IMAGE_LIB_USING_NEON
int   matrix_binning_8x8_neon_u8(struct matrix_s *src,struct matrix_s *dst,struct point2i *pos);
/*  scale image to 1/2 by using neon */
int   matrix_binning_neon_u8(struct matrix_s *src,struct matrix_s *dst);
int   matrix_fast_corner_neon_u8(struct matrix_s *img,struct point2i *pos,unsigned char threshold);
int   matrix_block_sad_8x8_neon_u8(struct matrix_s *img,struct point2i *pos);
int   matrix_sobel_neon(struct matrix_s *src,struct matrix_s *dst,int dx,int dy);
int   matrix_calc_pixel_deriv_neon(struct matrix_s *img,int x,int y, int32x4_t vkernel[3]);

#endif

#endif

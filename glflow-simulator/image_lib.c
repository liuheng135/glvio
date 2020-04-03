#include "image_lib.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
	
void matrix_create(struct matrix_s *mat,int cols,int rows,int channel,int type)
{
    int mem_size;

    mat->cols = cols;
    mat->rows = rows;
    mat->type = type;
    mat->channel = channel;

    mem_size = cols * rows * channel * (type & 0x0f);
    mat->data = (unsigned char *)malloc(mem_size);
    if(mat->data != NULL){
        memset(mat->data,0x00,mem_size);
    }
}

void matrix_destroy(struct matrix_s *mat)
{
    mat->cols = 0;
    mat->rows = 0;
    mat->type = 0;
    mat->channel = 0;
    if(mat->data != NULL){
        free(mat->data);
        mat->data = NULL;
    }
}

void matrix_init(struct matrix_s *img,int cols,int rows,int channel,int type,unsigned char *buffer)
{
    img->cols = cols;
    img->rows = rows;
    img->type = type;
    img->channel = channel;
    img->data = buffer;
}

int make_deriv_kernel(struct matrix_s *kx,struct matrix_s *ky,int dx,int dy,int size)
{
    int i;
    int *kx_data = (int *)kx->data;
    int *ky_data = (int *)ky->data;

    short order_table[3][3] = {
        {1,2,1},
        {-1,0,1},
        {1,-2,1}
    };

    if(kx->type != IMAGE_TYPE_32S || kx->type != IMAGE_TYPE_32S){
        return -1;
    }

    if(size != 3){
        return -2;
    }

    if(kx->data == NULL || ky->data == NULL){
        return -3;
    }

    if(dx < 0 || dx > 2){
        return -4;
    }

    if(dy < 0 || dy > 2){
        return -5;
    }

    for(i = 0;i < 3;i++){
        kx_data[i] = order_table[dx][i];
        ky_data[i] = order_table[dy][i];
    }
    kx->cols = 3;
    kx->rows = 1;
    ky->cols = 3;
    ky->rows = 1;

    return 0;
}

#ifdef IMAGE_LIB_USING_NEON
int matrix_binning_8x8_neon_u8(struct matrix_s *src,struct matrix_s *dst,struct point2i *pos)
{
    int i;
    unsigned char buffer[8];
    unsigned int  *bp1 = (unsigned int *)buffer;
    unsigned int  *bp2 = (unsigned int *)(buffer+4);
    unsigned int  *dp1;
    unsigned int  *dp2;
    unsigned char *src_ptr = src->data + pos->y * src->cols + pos->x;
    unsigned char *dst_ptr = dst->data + pos->y / 2 * dst->cols + pos->x / 2;

    uint8x8_t c1;
    uint8x8_t c2;
    uint8x8_t ca1;
    uint8x8_t ca2;

    if(pos->x < 0 || pos->x > (src->cols - 8)){
        return -1;
    }

    if(pos->y < 0 || pos->y > (src->rows - 8)){
        return -1;
    }

    for(i = 0;i < 2; i++){
        c1 = vld1_u8(src_ptr);
        c2 = vld1_u8(src_ptr+src->cols);
        ca1 = vhadd_u8(c1,c2);
        ca1 = vshr_n_u8(ca1,1);

        src_ptr += src->cols * 2;
        c1 = vld1_u8(src_ptr);
        c2 = vld1_u8(src_ptr+src->cols);
        ca2 = vhadd_u8(c1,c2);
        ca2 = vshr_n_u8(ca2,1);

        ca1 = vpadd_u8(ca1,ca2); 
        vst1_u8(buffer,ca1);
        dp1 = (unsigned int *)dst_ptr;
        dp2 = (unsigned int *)(dst_ptr + dst->cols);
        *dp1 = *bp1;
        *dp2 = *bp2;
        src_ptr += src->cols * 2;
        dst_ptr += dst->cols * 2;
    }
    return 0;
}

int matrix_binning_neon_u8(struct matrix_s *src,struct matrix_s *dst)
{
    struct point2i pos;
    register unsigned int tmp;
    int i,j,si,sj;
    int quotient_x = src->cols / 8;
    int quotient_y = src->rows / 8;
    int remainder_x = src->cols % 8;
    int remainder_y = src->rows % 8;

    if(src->type != dst->type || dst->type != IMAGE_TYPE_8U){
        return -1;
    }

    if(src->channel != 1 || src->channel != dst->channel){
        return -2;
    }


    if(dst->cols * 2 != src->cols || dst->rows * 2 != src->rows){
        return -3;
    }

    if(dst->data == NULL || src->data == NULL){
        return -4;
    }

    for(j = 0; j < src->rows;j+=8){
        for(i = 0; i < src->cols;i+=8){
            pos.x = i;
            pos.y = j;
            matrix_binning_8x8_neon_u8(src,dst,&pos);
        }
    }

    if(remainder_x > 0){
        printf("rx = %d\r\n",remainder_x);
        for(j = 0; j < quotient_y * 4;j++){
            for(i = quotient_x * 4; i < quotient_x * 4 + remainder_x / 2;i++){
                si = i*2;
                sj = j*2;
                tmp = src->data[sj * src->cols + si] + src->data[sj * src->cols + si + 1] \
                    + src->data[(sj + 1) * src->cols + si] + src->data[(sj + 1) * src->cols + si + 1];
                dst->data[j * dst->cols + i] = (unsigned char)(tmp >> 2);
            }
        }
    }

    if(remainder_y > 0){
        printf("ry = %d\r\n",remainder_y);
        for(j = quotient_y * 4; j < quotient_y * 4 + remainder_y / 2;j++){
            for(i = 0; i < dst->cols;i++){
                si = i*2;
                sj = j*2;
                tmp = src->data[sj * src->cols + si] + src->data[sj * src->cols + si + 1] \
                    + src->data[(sj + 1) * src->cols + si] + src->data[(sj + 1) * src->cols + si + 1];
                dst->data[j * dst->cols + i] = (unsigned char)(tmp >> 2);
            }
        }
    }
    return 0;
}

int matrix_fast_corner_neon_u8(struct matrix_s *img,struct point2i *pos,unsigned char threshold)
{
    int i;
    int count;
    unsigned char buffer[16];
    unsigned char center;
 
    uint8x16_t va;
    uint8x16_t vb;
    uint8x16_t vc;

    int offset_table[16][2] = \
        {{-1,-3},{0,-3},{1,-3},
      {-2,-2},            {2,-2},
    {-3,-1},                {3,-1},
    {-3,0},                  {3,0},
    {-3,1},                  {3,1},
      {-2,2},             {2,2},
        {-1,3},{0,3},{1,3}};

    for(i = 0;i < 16;i++){
        buffer[i] = *(img->data + pos->x + offset_table[i][0]+(pos->y+offset_table[i][1]) * img->cols);
    }
    center = *(img->data + pos->x + pos->y * img->cols);

    va = vld1q_u8(buffer);
    vb = vld1q_dup_u8(&center);
    vc = vabdq_u8(va,vb);
    vst1q_u8(buffer,vc);

    count = 0;
    for(i = 0;i < 16;i++){
        if(buffer[i] > threshold){
            count++;
        }
    }

    return count;
}

int matrix_block_sad_8x8_neon_u8(struct matrix_s *img,struct point2i *pos)
{
    int i;
    unsigned int   buffer[4];
    unsigned char *ptr;
    unsigned int   zero;
 
    uint8x8_t   va;
    uint8x8_t   vb;
    uint16x8_t  vc;
    uint32x4_t  vsum;
    uint8x8x2_t va2;

    if((pos->x + 8 >= img->cols) || (pos->y + 8 >= img->rows)){
        return 0;
    }

    ptr = img->data + pos->x + pos->y * img->cols;
    zero = 0;
    vsum = vld1q_dup_u32(&zero);
    for(i = 0;i < 8;i++){
        va = vld1_u8(ptr);
        ptr += img->cols;
        vb = vld1_u8(ptr);
        vc = vabdl_u8(va,vb);
        vsum = vpadalq_u16(vsum,vc);
    }

    ptr = img->data + pos->x + pos->y * img->cols;
    for(i = 0; i < 4; i++){
        va = vld1_u8(ptr);
        ptr += img->cols;
        vb = vld1_u8(ptr);
        va2 = vtrn_u8(va,vb);
        vc = vabdl_u8(va2.val[0],va2.val[1]);
        vsum = vpadalq_u16(vsum,vc); 

        ptr = ptr - img->cols + 1;
        va = vld1_u8(ptr);
        ptr += img->cols;
        vb = vld1_u8(ptr);
        va2 = vtrn_u8(va,vb);
        vc = vabdl_u8(va2.val[0],va2.val[1]);
        vsum = vpadalq_u16(vsum,vc);

        ptr -= 1;
        ptr += img->cols;
    }

    vst1q_u32(buffer,vsum);

    return buffer[0] + buffer[1] + buffer[2] + buffer[3];
}

int  matrix_calc_pixel_deriv_neon(struct matrix_s *img,int x,int y, int32x4_t vkernel[3])
{
    int buffer[4];
    int32x4_t vdat;
    int32x4_t vsum;
   
    unsigned char   *img_data_8u    = img->data;
    int             *img_data_32s   = (int *)img->data;

    buffer[3] = 0;

    vsum = vld1q_dup_s32(&buffer[3]);

    if(img->type == IMAGE_TYPE_8U){
        img_data_8u += (y-1) * img->cols + x - 1;

        buffer[0] = (int)*(img_data_8u);
        buffer[1] = (int)*(img_data_8u+1);
        buffer[2] = (int)*(img_data_8u+2);
        vdat = vld1q_s32(buffer);
        vsum = vmlaq_s32(vsum,vdat,vkernel[0]);

        img_data_8u += img->cols;
        buffer[0] = (int)*(img_data_8u);
        buffer[1] = (int)*(img_data_8u+1);
        buffer[2] = (int)*(img_data_8u+2);
        vdat = vld1q_s32(buffer);
        vsum = vmlaq_s32(vsum,vdat,vkernel[1]);

        img_data_8u += img->cols;
        buffer[0] = (int)*(img_data_8u);
        buffer[1] = (int)*(img_data_8u+1);
        buffer[2] = (int)*(img_data_8u+2);

        vdat = vld1q_s32(buffer);
        vsum = vmlaq_s32(vsum,vdat,vkernel[2]);
        vst1q_s32(buffer,vsum);
        return buffer[0] + buffer[1] + buffer[2];
    }

    if(img->type == IMAGE_TYPE_32S){
        img_data_32s += (y - 1) * img->cols + x - 1;

        vdat = vld1q_s32(img_data_32s);
        vsum = vmlaq_s32(vsum,vdat,vkernel[0]);

        img_data_32s+= img->cols;
        vdat = vld1q_s32(img_data_32s);
        vsum = vmlaq_s32(vsum,vdat,vkernel[1]);

        img_data_32s+= img->cols;
        vdat = vld1q_s32(img_data_32s);
        vsum = vmlaq_s32(vsum,vdat,vkernel[2]);

        vst1q_s32(buffer,vsum);
        return buffer[0] + buffer[1] + buffer[2];
    }

    return 0;
}

int matrix_sobel_neon(struct matrix_s *src,struct matrix_s *dst,int dx,int dy)
{
    int i,j;
	int *ptra,*ptrb;
    int border_x;
	int border_y;
    int buffer[4];
    struct matrix_s kx,ky;
    int32x4_t vkernel[3];

    if(dst->type != IMAGE_TYPE_32S){
        return -1;
    }

    if(src->cols != dst->cols || src->rows != dst->rows){
        return -2;
    }

    if(src->data == NULL || dst->data == NULL){
        return -3;
    }

    matrix_create(&kx,3,1,1,IMAGE_TYPE_32S);
    matrix_create(&ky,3,1,1,IMAGE_TYPE_32S);
    if(kx.data == NULL || ky.data == NULL){
        return -4;
    }
    make_deriv_kernel(&kx,&ky,dx,dy,3);

    ptra = (int *)kx.data;
    ptrb = (int *)ky.data;

    buffer[3] = 0;
    for(i = 0;i < 3;i++){
        buffer[0] = *ptra * *(ptrb + i);
        buffer[1] = *(ptra + 1) * *(ptrb + i);
        buffer[2] = *(ptra + 2) * *(ptrb + i);
        vkernel[i] = vld1q_s32(buffer);
    }
    
    border_x = (kx.cols - 1) / 2;
    border_y = (ky.cols - 1) / 2;
    ptra = (int *)(dst->data + border_y * dst->cols + border_x);

    for(j = border_y ; j < src->rows - border_y - 1; j++){
        for(i = border_x; i < src->cols - border_x; i++){
            *ptra++ =  matrix_calc_pixel_deriv_neon(src,i,j,vkernel);
        }
        ptra+=2;
    }

    ptra+=2;
    for(i = border_x; i < src->cols - border_x - 1; i++){
        *ptra++ =  matrix_calc_pixel_deriv_neon(src,i,j,vkernel);
    }
    *ptra++ =  matrix_calc_pixel_deriv(src,i,j,&kx,&ky);

    matrix_destroy(&kx);
    matrix_destroy(&ky);
    return 0;
}

#endif

int matrix_binning(struct matrix_s *src,struct matrix_s *dst)
{
    int i,j;
    int si,sj;
    unsigned int tmp;

    unsigned char  *src_data_8uc1;
    int            *src_data_32sc1;
    float          *src_data_32fc1;

    unsigned char  *dst_data_8uc1;
    int            *dst_data_32sc1;
    float          *dst_data_32fc1;

    if(src->type != dst->type){
        return -1;
    }

    if(src->channel != 1 || src->channel != dst->channel){
        return -2;
    }


    if(dst->cols * 2 != src->cols || dst->rows * 2 != src->rows){
        return -3;
    }

    if(dst->data == NULL || src->data == NULL){
        return -4;
    }

    if(dst->type == IMAGE_TYPE_8U){
        src_data_8uc1 = src->data;
        dst_data_8uc1 = dst->data;
        for(j = 0; j < dst->rows; j++){
            for(i = 0; i < dst->cols; i++){
                si = i*2;
                sj = j*2;
                tmp = src_data_8uc1[sj * src->cols + si] + src_data_8uc1[sj * src->cols + si + 1] \
                    + src_data_8uc1[(sj + 1) * src->cols + si] + src_data_8uc1[(sj + 1) * src->cols + si + 1];
                dst_data_8uc1[j * dst->cols + i] = (unsigned char)(tmp >> 2);
            }
        }
        return 0;
    }

    if(dst->type == IMAGE_TYPE_32S){
        src_data_32sc1 = (int *)src->data;
        dst_data_32sc1 = (int *)dst->data;
        for(j = 0; j < dst->rows; j++){
            for(i = 0; i < dst->cols; i++){
                si = i*2;
                sj = j*2;
                tmp = src_data_32sc1[sj * src->cols + si] + src_data_32sc1[sj * src->cols + si + 1] \
                    + src_data_32sc1[(sj + 1) * src->cols + si] + src_data_32sc1[(sj + 1) * src->cols + si + 1];
                dst_data_32sc1[j * dst->cols + i] = tmp / 4;
            }
        }
        return 0;
    }

    if(dst->type == IMAGE_TYPE_32F){
        src_data_32fc1 = (float *)src->data;
        dst_data_32fc1 = (float *)dst->data;
        for(j = 0; j < dst->rows; j++){
            for(i = 0; i < dst->cols; i++){
                si = i*2;
                sj = j*2;
                tmp = src_data_32fc1[sj * src->cols + si] + src_data_32fc1[sj * src->cols + si + 1] \
                    + src_data_32fc1[(sj + 1) * src->cols + si] + src_data_32fc1[(sj + 1) * src->cols + si + 1];
                dst_data_32fc1[j * dst->cols + i] = tmp * 0.25f;
            }
        }
        return 0;
    }
    return -5;
}

int   matrix_convert_type(struct matrix_s *src,struct matrix_s *dst)
{
    int i,j;

    unsigned char  *src_data_8uc1;
    int            *src_data_32sc1;
    //unsigned int   *src_data_32uc1;
    float          *src_data_32fc1;

    unsigned char  *dst_data_8uc1;
    int            *dst_data_32sc1;
    unsigned int   *dst_data_32uc1;
    float          *dst_data_32fc1;

    if(src->cols != dst->cols || src->rows != dst->rows){
        return -1;
    }

    if(src->channel != dst->channel){
        return -2;
    }

    if(src->type == dst->type){
        return 0;
    }

    if(src->channel != 1){
        /* only support 1 channel for now  */
        return -3;
    }

    
    /* uint8_t to int32_t */
    if((src->type == IMAGE_TYPE_8U) && (dst->type = IMAGE_TYPE_32S)){
        src_data_8uc1 = (unsigned char*)src->data;
        dst_data_32sc1 = (int*)dst->data;

        for(j = 0; j <  src->rows; j++){
            for(i = 0; i < src->cols; i++){
                *dst_data_32sc1++ = (int)*src_data_8uc1++;
            }
        }
        return 0;
    }

    /* uint8_t to uint32_t */
    if((src->type == IMAGE_TYPE_8U) && (dst->type = IMAGE_TYPE_32U)){
        src_data_8uc1 = (unsigned char*)src->data;
        dst_data_32uc1 = (unsigned int*)dst->data;

        for(j = 0; j <  src->rows; j++){
            for(i = 0; i < src->cols; i++){
                *dst_data_32uc1++ = (unsigned int)*src_data_8uc1++;
            }
        }
        return 0;
    }

    /* uint8_t to float */
    if((src->type == IMAGE_TYPE_8U) && (dst->type = IMAGE_TYPE_32F)){
        src_data_8uc1 = (unsigned char*)src->data;
        dst_data_32fc1 = (float*)dst->data;

        for(j = 0; j <  src->rows; j++){
            for(i = 0; i < src->cols; i++){
                *dst_data_32fc1++ = (float)*src_data_8uc1++;
            }
        }
        return 0;
    }

    /* int32_t to uint8_t */
    if((src->type == IMAGE_TYPE_32S) && (dst->type = IMAGE_TYPE_8U)){
        src_data_32sc1 = (int *)src->data;
        dst_data_8uc1 = (unsigned char*)dst->data;

        for(j = 0; j <  src->rows; j++){
            for(i = 0; i < src->cols; i++){
                *dst_data_8uc1++ = (unsigned char)(*src_data_32sc1++ & 0xff);
            }
        }
        return 0;
    }

    /* int32_t to float */
    if((src->type == IMAGE_TYPE_32S) && (dst->type = IMAGE_TYPE_32F)){
        src_data_32sc1 = (int *)src->data;
        dst_data_32fc1 = (float *)dst->data;

        for(j = 0; j <  src->rows; j++){
            for(i = 0; i < src->cols; i++){
                *dst_data_32fc1++ = (float)*src_data_32sc1++;
            }
        }
        return 0;
    }

    /* int32_t to float */
    if((src->type == IMAGE_TYPE_32S) && (dst->type = IMAGE_TYPE_32F)){
        src_data_32sc1 = (int *)src->data;
        dst_data_32fc1 = (float *)dst->data;

        for(j = 0; j <  src->rows; j++){
            for(i = 0; i < src->cols; i++){
                *dst_data_32fc1++ = (float)*src_data_32sc1++;
            }
        }
        return 0;
    }

    /* float to  int32_t */
    if((src->type == IMAGE_TYPE_32F) && (dst->type = IMAGE_TYPE_32S)){
        src_data_32fc1 = (float *)src->data;
        dst_data_32sc1 = (int *)dst->data;

        for(j = 0; j <  src->rows; j++){
            for(i = 0; i < src->cols; i++){
                *dst_data_32sc1++ = (int)*src_data_32fc1++;
            }
        }
        return 0;
    }

    /* float to  uint8_t */
    if((src->type == IMAGE_TYPE_32F) && (dst->type = IMAGE_TYPE_8U)){
        src_data_32fc1 = (float *)src->data;
        dst_data_8uc1 = (unsigned char*)dst->data;

        for(j = 0; j <  src->rows; j++){
            for(i = 0; i < src->cols; i++){
                *dst_data_8uc1++ = (unsigned char)*src_data_32fc1++;
            }
        }
        return 0;
    }

    return -5;
}

int matrix_copy_aera(struct matrix_s *src,struct matrix_s *dst,struct point2i *start,struct size2i *size)
{
    unsigned char  *src_data_8uc1;
    int            *src_data_32sc1;
    float          *src_data_32fc1;

    unsigned char  *dst_data_8uc1;
    int            *dst_data_32sc1;
    float          *dst_data_32fc1;
    

    int sx,sy;


    if(src->type != dst->type){
        return -1;
    }

    if((size->x) != dst->cols){
        return -2;
    }

    if((size->y) != dst->rows){
        return -2;
    }

    if(src->data == NULL || dst->data == NULL){
        return -3;
    }

    if(src->type == IMAGE_TYPE_8U){
        src_data_8uc1 = src->data;
        dst_data_8uc1 = dst->data;

        src_data_8uc1 += start->y * src->cols + start->x;
        for(sy = 0;sy <  size->y;sy++){
            for(sx = 0;sx < size->x;sx++){
                *dst_data_8uc1++ = *src_data_8uc1++;
            }
            src_data_8uc1+=(src->cols - size->x);
        }
    }

    if(src->type == IMAGE_TYPE_32S){
        src_data_32sc1 = (int *)src->data;
        dst_data_32sc1 = (int *)dst->data;

        src_data_32sc1 += start->y * src->cols + start->x;
        for(sy = start->y;sy < start->y + size->y;sy++){
            for(sx = start->x;sx < start->x + size->x;sx++){
                *dst_data_32sc1++ = *src_data_32sc1++;
            }
            src_data_32sc1+=(src->cols - size->x);
        }
    }

    if(src->type == IMAGE_TYPE_32F){
        src_data_32fc1 = (float *)src->data;
        dst_data_32fc1 = (float *)dst->data;

        src_data_32fc1 += start->y * src->cols + start->x;
        for(sy = start->y;sy < start->y + size->y;sy++){
            for(sx = start->x;sx < start->x + size->x;sx++){
                *dst_data_32fc1++ = *src_data_32fc1++;
            }
            src_data_32fc1+=(src->cols - size->x);
        }
    }
    return 0;
}

int  matrix_calc_pixel_deriv(struct matrix_s *img,int x,int y,struct matrix_s *kx,struct matrix_s *ky)
{
	int i;
    int kx_data[3];
    int ky_data[3];
    int px_buffer[3];

    unsigned char   *img_data_8u    = img->data;
    int             *img_data_32s   = (int *)img->data;

    int *kx_ptr_32s = (int *)kx->data;
    int *ky_ptr_32s = (int *)ky->data;
    for(i = 0;i < 3;i++){
        kx_data[i] = kx_ptr_32s[i];
        ky_data[i] = ky_ptr_32s[i];
    }

    if(img->type == IMAGE_TYPE_8U){
        img_data_8u += y * img->cols + x;

        px_buffer[0] = kx_data[0] * (int)*(img_data_8u - img->cols - 1) + kx_data[1] * (int)*(img_data_8u - img->cols)  + kx_data[2] * (int)*(img_data_8u - img->cols + 1);
        px_buffer[1] = kx_data[0] * (int)*(img_data_8u - 1)             + kx_data[1] * (int)*(img_data_8u)              + kx_data[2] * (int)*(img_data_8u + 1);
        px_buffer[2] = kx_data[0] * (int)*(img_data_8u + img->cols - 1) + kx_data[1] * (int)*(img_data_8u + img->cols)  + kx_data[2] * (int)*(img_data_8u + img->cols + 1);

        return px_buffer[0] * ky_data[0] + px_buffer[1] * ky_data[1] + px_buffer[2] * ky_data[2];
    }

    if(img->type == IMAGE_TYPE_32S){
        img_data_32s += y * img->cols + x;

        px_buffer[0] = kx_data[0] * *(img_data_32s - img->cols - 1) + kx_data[1] * *(img_data_32s - img->cols) + kx_data[2] * *(img_data_32s - img->cols + 1);
        px_buffer[1] = kx_data[0] * *(img_data_32s - 1)             + kx_data[1] * *(img_data_32s)             + kx_data[2] * *(img_data_32s + 1);
        px_buffer[2] = kx_data[0] * *(img_data_32s + img->cols - 1) + kx_data[1] * *(img_data_32s + img->cols) + kx_data[2] * *(img_data_32s + img->cols + 1);

        return px_buffer[0] * ky_data[0] + px_buffer[1] * ky_data[1] + px_buffer[2] * ky_data[2];
    }

    return 0;
}

int matrix_sobel(struct matrix_s *src,struct matrix_s *dst,int dx,int dy)
{
    struct matrix_s kx,ky;
	int border_x;
	int border_y;
    int i,j;

    int *dst_data_32s   = (int *)dst->data;

    if(dst->type != IMAGE_TYPE_32S){
        return -1;
    }

    if(src->cols != dst->cols || src->rows != dst->rows){
        return -2;
    }

    if(src->data == NULL || dst->data == NULL){
        return -3;
    }

    matrix_create(&kx,3,1,1,IMAGE_TYPE_32S);
    matrix_create(&ky,3,1,1,IMAGE_TYPE_32S);
    if(kx.data == NULL || ky.data == NULL){
        return -4;
    }
    make_deriv_kernel(&kx,&ky,dx,dy,3);

    border_x = (kx.cols - 1) / 2;
    border_y = (ky.cols - 1) / 2;

    dst_data_32s+= border_y * dst->cols + border_x;

    for(j = border_y ; j < src->rows - border_y; j++){
        for(i = border_x; i < src->cols - border_x; i++){
            *dst_data_32s++ =  matrix_calc_pixel_deriv(src,i,j,&kx,&ky);
        }
        dst_data_32s+=2;
    }

    matrix_destroy(&kx);
    matrix_destroy(&ky);
    return 0;
}

int  matrix_filter_x(struct matrix_s *src,struct matrix_s *dst,struct matrix_s *kernel,float scale)
{
	int i;
    int border_x = (kernel->cols - 1) / 2;

    int             *dst_data_32s   = (int *)dst->data;
    float           *dst_data_32fc1 = (float *)dst->data;
    

    unsigned char   *src_data_8u    = src->data;
    int             *src_data_32s   = (int *)src->data;
    float           *src_data_32fc1 = (float *)src->data;

    int *kx_ptr_32s;
    int x,y;
    int kx_data[3];
    float kxf_data[3];

    if(dst->type != IMAGE_TYPE_32S && dst->type != IMAGE_TYPE_32F){
        return -1;
    }

    if(src->cols != dst->cols || src->rows != dst->rows){
        return -2;
    }

    if(src->data == NULL || dst->data == NULL){
        return -3;
    }

    if(kernel->data == NULL){
        return -4;
    }

    kx_ptr_32s = (int *)kernel->data;
    for(i = 0;i < 3;i++){
        kx_data[i] = kx_ptr_32s[i];
        kxf_data[i] = kx_data[i] * scale;
    }

    printf("kxf = %f %f %f\r\n",kxf_data[0],kxf_data[1],kxf_data[2]);
    if(dst->type == IMAGE_TYPE_32F){
        if(src->type == IMAGE_TYPE_8U){
            dst_data_32fc1 += 1;
            src_data_8u    += 1;
            for(y = 0; y < src->rows; y++){
                for(x = border_x; x < src->cols - border_x; x++){
                    *dst_data_32fc1 = kxf_data[0] *  (float)*(src_data_8u - 1) + kx_data[1] * (float)*src_data_8u + kx_data[2] * (float)*(src_data_8u + 1);
                    dst_data_32fc1++;
                    src_data_8u++;
                }
                dst_data_32fc1+=2;
                src_data_8u+=2;
            }
        }

        if(src->type == IMAGE_TYPE_32S){
            dst_data_32fc1  += 1;
            src_data_32s    += 1;
            for(y = 0; y < src->rows; y++){
                for(x = border_x; x < src->cols - border_x; x++){
                    *dst_data_32fc1 = kxf_data[0] *  (float)*(src_data_32s - 1) + kx_data[1] * (float)*src_data_32s + kx_data[2] * (float)*(src_data_32s + 1);
                    dst_data_32fc1++;
                    src_data_32s++;
                }
                dst_data_32fc1+=2;
                src_data_32s+=2;
            }
        }

        if(src->type == IMAGE_TYPE_32F){
            dst_data_32fc1  += 1;
            src_data_32fc1  += 1;
            for(y = 0; y < src->rows; y++){
                for(x = border_x; x < src->cols - border_x; x++){
                    *dst_data_32fc1 = kxf_data[0] *  *(src_data_32fc1 - 1) + kx_data[1] * *src_data_32fc1 + kx_data[2] * *(src_data_32fc1 + 1);
                    dst_data_32fc1++;
                    src_data_32fc1++;
                }
                dst_data_32fc1+=2;
                src_data_32fc1+=2;
            }
        }
    }else{
        if(src->type == IMAGE_TYPE_8U){
            dst_data_32s += 1;
            src_data_8u  += 1;

            for(y = 0; y < src->rows; y++){
                for(x = border_x; x < src->cols - border_x; x++){
                    *dst_data_32s =  *(src_data_8u - 1) * kx_data[0] + *src_data_8u * kx_data[1] + *(src_data_8u + 1) * kx_data[2];
                    dst_data_32s++;
                    src_data_8u++;
                }
                dst_data_32s+=2;
                src_data_8u+=2;
            }
        }

        if(src->type == IMAGE_TYPE_32S){
            dst_data_32s += 1;
            src_data_32s  += 1;
            for(y = 0; y < src->rows; y++){
                for(x = border_x; x < src->cols - border_x; x++){
                    *dst_data_32s =  *(src_data_32s - 1) * kx_data[0] + *src_data_32s * kx_data[1] + *(src_data_32s + 1) * kx_data[2];
                    dst_data_32s++;
                    src_data_32s++;
                }
                dst_data_32s+=2;
                src_data_32s+=2;
            }
        }
    }
    return 0;
}

int matrix_filter_y(struct matrix_s *src,struct matrix_s *dst,struct matrix_s *kernel,float scale)
{
	int i;
    int border_y = (kernel->cols - 1) / 2;

    int             *dst_data_32s = (int *)dst->data;
    float           *dst_data_32fc1 = (float *)dst->data;

    unsigned char   *src_data_8u = src->data;
    int             *src_data_32s = (int *)src->data;
    float           *src_data_32fc1 = (float *)src->data;

    int *ky_ptr_32s;
    int x,y;
    int ky_data[3];
    float kyf_data[3];

    if(dst->type != IMAGE_TYPE_32S && dst->type != IMAGE_TYPE_32F){
        return -1;
    }

    if(src->cols != dst->cols || src->rows != dst->rows){
        return -2;
    }

    if(src->data == NULL || dst->data == NULL){
        return -3;
    }

    if(kernel->data == NULL ){
        return -4;
    }

    ky_ptr_32s = (int *)kernel->data;
    for(i = 0;i < 3;i++){
        ky_data[i]  = ky_ptr_32s[i];
        kyf_data[i] = ky_data[i] * scale;
    }

    if(dst->type == IMAGE_TYPE_32F){
        if(src->type == IMAGE_TYPE_8U){
            dst_data_32fc1 += src->cols;
            src_data_8u    += src->cols;

            for(y = border_y; y < src->rows - border_y; y++){
                for(x = 0; x < src->cols; x++){
                    *dst_data_32fc1 =  kyf_data[0] * (float)*(src_data_8u - src->cols) + kyf_data[1] * (float)*src_data_8u +kyf_data[2] * (float)*(src_data_8u + src->cols);
                    dst_data_32fc1++;
                    src_data_8u++;
                }
            }
        }

        if(src->type == IMAGE_TYPE_32S){
            dst_data_32fc1 += src->cols;
            src_data_32s   += src->cols;

            for(y = border_y; y < src->rows - border_y; y++){
                for(x = 0; x < src->cols; x++){
                    *dst_data_32fc1 =  kyf_data[0] * (float)*(src_data_32s - src->cols) + kyf_data[1] * (float)*src_data_32s +kyf_data[2] * (float)*(src_data_32s + src->cols);
                    dst_data_32fc1++;
                    src_data_32s++;
                }
            }
        }

        if(src->type == IMAGE_TYPE_32F){
            dst_data_32fc1 += src->cols;
            src_data_32fc1 += src->cols;
            for(y = border_y; y < src->rows - border_y; y++){
                for(x = 0; x < src->cols; x++){
                    *dst_data_32fc1 =  kyf_data[0] * *(src_data_32fc1 - src->cols) + kyf_data[1] * *src_data_32fc1 +kyf_data[2] * *(src_data_32fc1 + src->cols);
                    dst_data_32fc1++;
                    src_data_32fc1++;
                }
            }
        }
    }else{
        if(src->type == IMAGE_TYPE_8U){
            dst_data_32s += src->cols;
            src_data_8u  += src->cols;

            for(y = border_y; y < src->rows - border_y; y++){
                for(x = 0; x < src->cols; x++){
                    *dst_data_32s =  *(src_data_8u - src->cols) * ky_data[0] + *src_data_8u * ky_data[1] + *(src_data_8u + src->cols) * ky_data[2];
                    dst_data_32s++;
                    src_data_8u++;
                }
            }
        }

        if(src->type == IMAGE_TYPE_32S){
            dst_data_32s += src->cols;
            src_data_32s += src->cols;

            for(y = border_y; y < src->rows - border_y; y++){
                for(x = 0; x < src->cols; x++){
                    *dst_data_32s =  *(src_data_32s - src->cols) * ky_data[0] + *src_data_32s * ky_data[1] + *(src_data_32s + src->cols) * ky_data[2];
                    dst_data_32s++;
                    src_data_32s++;
                }
            }
        }
    }
    return 0;
}

float matrix_get_pixel_val(struct matrix_s *img,float x,float y)
{
    int ix = (int)x;
    int iy = (int)y;

	float v00;
	float v01;
	float v10;
	float v11;
	
    float a = x - (float)ix;
    float b = y - (float)iy;

    float w00 = (1.f - a)*(1.f - b); 
    float w01 = a*(1.f - b);
    float w10 = (1.f - a)*b; 
    float w11 = a*b;

    unsigned char   *img_data_8u = img->data;
    int             *img_data_32s = (int *)img->data;
    float           *img_data_32f = (float *)img->data;

    if(img->type == IMAGE_TYPE_8U){
        img_data_8u = img_data_8u + iy * img->cols + ix;

        v00 = *img_data_8u * w00;
        v01 = *(img_data_8u+1) * w01;
        v10 = *(img_data_8u + img->cols) * w10;
        v11 = *(img_data_8u + img->cols + 1) * w11;

        return  v00 + v01 + v10 + v11;
    }

    if(img->type == IMAGE_TYPE_32S){
        img_data_32s = img_data_32s + iy * img->cols + ix;

        v00 = *img_data_32s * w00;
        v01 = *(img_data_32s+1) * w01;
        v10 = *(img_data_32s + img->cols) * w10;
        v11 = *(img_data_32s + img->cols + 1) * w11;

        return  v00 + v01 + v10 + v11; 
    }

    if(img->type == IMAGE_TYPE_32F){
        img_data_32f = img_data_32f + iy * img->cols + ix;

        v00 = *img_data_32f * w00;
        v01 = *(img_data_32f+1) * w01;
        v10 = *(img_data_32f + img->cols) * w10;
        v11 = *(img_data_32f + img->cols + 1) * w11;

        return  v00 + v01 + v10 + v11; 
    }
    return 0.f;
}

int matrix_copy_channel(struct matrix_s *src,struct matrix_s *dst,int src_ch,int dst_ch)
{
    int i,j;

    unsigned char  *src_data_8uc1  = src->data;
    int            *src_data_32sc1 = (int *)src->data;
    float          *src_data_32fc1 = (float *)src->data;

    unsigned char  *dst_data_8uc1  = dst->data;
    int            *dst_data_32sc1 = (int *)dst->data;
    float          *dst_data_32fc1 = (float *)dst->data;

    if(src->type != dst->type){
        return -1;
    }

    if(src->cols != dst->cols || src->rows != dst->rows){
        return -2;
    }

    if(src->data == NULL || dst->data == NULL){
        return -3;
    }

    if(src->type == IMAGE_TYPE_8U){
        src_data_8uc1 += src_ch;
        dst_data_8uc1 += dst_ch;

        for(j = 0;j < src->rows;j++){
            for(i = 0;i < src->cols;i++){
                *dst_data_8uc1 = *src_data_8uc1;
                src_data_8uc1 += src->channel;
                dst_data_8uc1 += dst->channel;
            } 
        }
    }

    if(src->type == IMAGE_TYPE_32S){
        src_data_32sc1 += src_ch;
        dst_data_32sc1 += dst_ch;

        //printf("dst+ch = 0x%x\r\n",dst_data_32sc1);

        for(j = 0;j < src->rows;j++){
            for(i = 0;i < src->cols;i++){
                *dst_data_32sc1 = *src_data_32sc1;
                src_data_32sc1 += src->channel;
                dst_data_32sc1 += dst->channel;
            }
        }
    }

    if(src->type == IMAGE_TYPE_32F){
        src_data_32fc1 += src_ch;
        dst_data_32fc1 += dst_ch;

        for(j = 0;j < src->rows;j++){
            for(i = 0;i < src->cols;i++){
                *dst_data_32fc1 = *src_data_32fc1;
                src_data_32fc1 += src->channel;
                dst_data_32fc1 += dst->channel;
            } 
        }
    }
    return 0;
}
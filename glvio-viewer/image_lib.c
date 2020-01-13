#include "image_lib.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>


void *pvPortMalloc( size_t xWantedSize );
	
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
        return -1;
    }

    if(kx->data == NULL || ky->data == NULL){
        return -1;
    }

    if(dx < 0 || dx > 2){
        return -1;
    }

    if(dy < 0 || dy > 2){
        return -1;
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

void print_memory_int(void *addr,int len)
{
	int i;
    int *ptr = (int *)addr;

    printf("0x%x:",(unsigned int)addr);

    for(i = 0; i < len;i++){
        printf("%x ",ptr[i]);
    }
    printf("\r\n");
}
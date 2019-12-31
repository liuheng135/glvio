#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <sys/sem.h>
#include <sys/shm.h>
#include <semaphore.h>
#include <sys/ipc.h>
#include "camera.h"
#include <stdbool.h>

//��װ���ioctl()
static void xioctl(int fh, int request, void *arg)
{
    int r;
    do {
        r = ioctl(fh, request, arg);
    } while ( (r == -1) && ((errno == EINTR) || (errno == EAGAIN)) );
    if (r == -1) {
        printf("%s(): ioctl('request = %d') failed: %s\n", __func__, request, strerror(errno));
        exit(EXIT_FAILURE);
    }
}

struct my_buffer {
    void   *start;
    size_t length;
};

static struct my_buffer *g_my_buffers; 
static struct v4l2_buffer g_frame;  //���������е�һ֡
static int    g_camera_fd = 0;      //����ͷ���豸�ļ����

int camera_init(const char *dev_name, int width, int height)
{
    g_camera_fd = open(dev_name, O_RDWR | O_NONBLOCK, 0);
    //g_camera_fd = open(dev_name, O_RDWR);
    if (g_camera_fd < 0) {
        printf("%s(): open('%s') failed: %s\n", __func__, dev_name, strerror(errno));
        return -1;
    }
    
    //fcntl(g_camera_fd, F_SETFD, FD_CLOEXEC);
    
    struct v4l2_input inp;
    memset(&inp, 0, sizeof(inp));
    inp.index = 0;
    if (-1 == ioctl(g_camera_fd, VIDIOC_S_INPUT, &inp)) {
        printf("%s(): ioctl('VIDIOC_S_INPUT') failed: %s\n", __func__, strerror(errno));
        return -1;
    }
    
    struct v4l2_streamparm parms;
    memset(&parms, 0, sizeof(parms));
    parms.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parms.parm.capture.timeperframe.numerator = 1;
    parms.parm.capture.timeperframe.denominator = FPS;
    //parms.parm.capture.capturemode = V4L2_MODE_VIDEO; //V4L2_MODE_IMAGE
    if (-1 == ioctl(g_camera_fd, VIDIOC_S_PARM, &parms)) {
        printf("%s(): ioctl('VIDIOC_S_PARM') failed: %s\n", __func__, strerror(errno));
        return -1;
    }
    
    struct v4l2_format format;  //֡�ĸ�ʽ�������ȣ��߶ȵ�
    memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;  //�趨��Ƶ�����ʽ
    format.fmt.pix.width  = width;
    format.fmt.pix.height = height;
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420;
    format.fmt.pix.field = V4L2_FIELD_NONE;
    xioctl(g_camera_fd, VIDIOC_S_FMT, &format);  //���õ�ǰ������Ƶ�����ʽ, ͬʱ��������
    if (format.fmt.pix.pixelformat != V4L2_PIX_FMT_YUV420) {
        printf("%s(): Libv4l didn't accept YUV420 format. Can't proceed\n", __func__);
        return -1;
    }
    if (    (format.fmt.pix.width  != (unsigned int)width) 
         || (format.fmt.pix.height != (unsigned int)height) ) {  //��������в����Ƿ����óɹ�
        printf("%s(): Set resolution failed: Driver is sending image at %dx%d\n", __func__, format.fmt.pix.width, format.fmt.pix.height);
        return -1;
    }
    
    struct v4l2_requestbuffers req;  //����������֡��������������������ĸ���
    memset(&req, 0, sizeof(req));
    req.count = BUFFER_CNT;  //����4֡ͼ���ڴ�
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    xioctl(g_camera_fd, VIDIOC_REQBUFS, &req);  //�����ڴ�
    g_my_buffers = (struct my_buffer *)calloc(req.count, sizeof(*g_my_buffers));  //calloc()���뵽���ڴ������
    
    struct v4l2_buffer frame;  //���������е�һ֡
    unsigned int i = 0;
    for (i = 0; i < req.count; i++) {
        memset(&frame, 0, sizeof(frame));
        frame.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        frame.memory = V4L2_MEMORY_MMAP;
        frame.index  = i;
        xioctl(g_camera_fd, VIDIOC_QUERYBUF, &frame);  //�� VIDIOC_REQBUFS �з�������ݻ���ת���������ַ

        g_my_buffers[i].length = frame.length;
        g_my_buffers[i].start = mmap(NULL, frame.length, PROT_READ | PROT_WRITE, MAP_SHARED, g_camera_fd, frame.m.offset);
        if (MAP_FAILED == g_my_buffers[i].start) {
            printf("%s(): mmap() failed: %s\n", __func__, strerror(errno));
            return -1;
        }
    }
    
    for (i = 0; i < req.count; i++) {
        memset(&frame, 0, sizeof(frame));
        frame.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        frame.memory = V4L2_MEMORY_MMAP;
        frame.index  = i;
        xioctl(g_camera_fd, VIDIOC_QBUF, &frame);  //���뻺�����
    }
    
    enum v4l2_buf_type V4l2_buf_type;
    V4l2_buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;  //���������ͣ�������Զ�� V4L2_BUF_TYPE_VIDEO_CAPTURE
    xioctl(g_camera_fd, VIDIOC_STREAMON, &V4l2_buf_type);  //��ʼ��Ƶ��ʾ���� 

#ifdef FLOW_SHM_PIC
	camera_flow_init(&format);
#endif

    return 0;
}


uint8_t *camera_get_image(void)
{
    int ret = 0;
    static bool s_first_run = true;
    
    if(s_first_run == false) {
        //���������������β����������ѭ���ɼ�
        xioctl(g_camera_fd, VIDIOC_QBUF, &g_frame);  
    }
    
    //����camera_fd�Ƿ������ݹ���
    do {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(g_camera_fd, &fds);
        struct timeval tv;
        tv.tv_sec = 2;
        tv.tv_usec = 0;
        ret = select(g_camera_fd + 1, &fds, NULL, NULL, &tv);
    } while ((ret == -1) && (errno == EINTR));
    if (ret == -1) {
        printf("%s(): select() failed: %s", __func__, strerror(errno));
        return NULL;
    }
    
    memset(&g_frame, 0, sizeof(g_frame));
    g_frame.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    g_frame.memory = V4L2_MEMORY_MMAP;
    xioctl(g_camera_fd, VIDIOC_DQBUF, &g_frame);  //��������ȡ���Ѳɼ����ݵ�֡���壬ȡ��ԭʼ�ɼ�����
    //printf("%s(): g_frame.index = %d\n", __func__, g_frame.index);

    s_first_run = false;
    return (uint8_t *)g_my_buffers[g_frame.index].start;
}

void camera_deinit(void)
{
    enum v4l2_buf_type V4l2_buf_type;
    V4l2_buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(g_camera_fd, VIDIOC_STREAMOFF, &V4l2_buf_type);  //������Ƶ��ʾ���� 
    
    int i = 0;
    for (i = 0; i < BUFFER_CNT; ++i) {
        munmap(g_my_buffers[i].start, g_my_buffers[i].length);  //ȡ������start��ָ��ӳ���ڴ���ʼ��ַ
    }
    
    close(g_camera_fd);
}


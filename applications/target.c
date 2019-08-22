#include <rtthread.h>
#include "follow.h"

#define     DBG_SECTION_NAME    "target"
#define     DBG_LEVEL           DBG_LOG
#include <rtdbg.h>

#define DEFAULT_TARGET_DEVICE       "uart6"

// Thread
#define THREAD_STACK_SIZE      1024
#define THREAD_PRIORITY        ((RT_THREAD_PRIORITY_MAX / 3) + 5)
#define THREAD_TICK            10

rt_thread_t tid_target = RT_NULL;
rt_device_t dev_target = RT_NULL;
rt_sem_t rx_sem = RT_NULL;

extern rt_mq_t pos_mq;

extern float stof(const char *s);

static int parse_frame_data(char *s, int count, float *data)
{
    int cnt = 0;
    int idx = 0;
    char *str = s;

    while (cnt < count)
    {
        data[cnt++] = stof(str);

        while(s[++idx] != ' ');
        str = &s[idx];
    }
    data[cnt++] = stof(str);

    return cnt;
}

static char target_getchar(void)
{
    char tmp;
    
    while (rt_device_read(dev_target, -1, &tmp, 1) != 1)
        rt_sem_take(rx_sem, RT_WAITING_FOREVER);

    return tmp;
}

static rt_err_t target_rx_ind(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(rx_sem);

    return RT_EOK;
}

int target_set_device(const char *device_name)
{
    rt_device_t dev = RT_NULL;

    dev = rt_device_find(device_name);
    if (dev == RT_NULL)
    {
        LOG_E("Can not find device: %s\n", device_name);
        return RT_ERROR;
    }

    /* check whether it's a same device */
    if (dev == dev_target) return RT_ERROR;
    /* open this device and set the new device in finsh shell */
    if (rt_device_open(dev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX) == RT_EOK)
    {
        if (dev_target != RT_NULL)
        {
            /* close old finsh device */
            rt_device_close(dev_target);
            rt_device_set_rx_indicate(dev_target, RT_NULL);
        }

        dev_target = dev;
        rt_device_set_rx_indicate(dev_target, target_rx_ind);
    }

    return RT_EOK;
}

#define FRAME_HEAD_BYTE1    '0'
#define FRAME_HEAD_BYTE2    ' '
#define FRAME_TAIL_BYTE1    (uint8_t)(0x0D)
#define FRAME_TAIL_BYTE2    (uint8_t)(0x0A)

#define STATION_HEAD_BYTE1  0
#define STATION_HEAD_BYTE2  1
#define STATION_DATA        2
// #define STATION_TAIL_BYTE1  -1
#define STATION_TAIL_BYTE2  -2

// const TAIL_BYTES[] = {0x0D, 0x0D, 0x0A, 0x0D, 0x0A};

static void target_thread_entry(void *param)
{
    char ch;
    char buffer[60];
    uint16_t idx;
    int station = STATION_HEAD_BYTE1;
    
    while(1)
    {
        ch = target_getchar();
        // parse frame
        switch (station)
        {
        case STATION_HEAD_BYTE1:
            idx = 0;
            if (ch == FRAME_HEAD_BYTE1)
                station = STATION_HEAD_BYTE2;
            break;
        case STATION_HEAD_BYTE2:
            if (ch == FRAME_HEAD_BYTE2)
                station = STATION_DATA;
            else
                station = STATION_HEAD_BYTE1;
            break;
        case STATION_DATA:
            buffer[idx++] = ch;

            if (ch == FRAME_TAIL_BYTE1)
                station = STATION_TAIL_BYTE2;
            break;
        case STATION_TAIL_BYTE2:
            station = STATION_HEAD_BYTE1;
            if (ch == FRAME_TAIL_BYTE2)
            {
                // buffer[idx] = '\0';
                // rt_kprintf(buffer);
                float data[4];
                parse_frame_data(buffer, 4, data);
                rt_kprintf("target-pos: %d %d %d %d\n", (int)(1000*data[0]), (int)(1000*data[1]), (int)(1000*data[2]), (int)(1000*data[3]));
                if (pos_mq != RT_NULL)
                {   
                    struct pos_data pos = {
                        .x = (int)(1000*data[0]),
                        .y = (int)(1000*data[1]),
                        .height = (int)(1000*data[2]),
                        .width = (int)(1000*data[3])
                    };
                    rt_mq_send(pos_mq, &pos, sizeof(struct pos_data));
                }
            }

            break;
        default:
            break;
        }
        // parse data

    }
}

int target_init(void *param)
{
    if (target_set_device((char *)param) != RT_EOK)
    {
        LOG_E("Failed to find device");
        return RT_ERROR;
    }

    rx_sem = rt_sem_create("targetRx", 0, RT_IPC_FLAG_FIFO);
    if (rx_sem == RT_NULL)
    {
        LOG_E("Failed to create sem\n");
        return RT_ERROR;
    }

    tid_target = rt_thread_create("target", target_thread_entry, RT_NULL, THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TICK);
    if (tid_target == RT_NULL)
    {
        LOG_E("Failed to create thread\n");
        return RT_ERROR;
    }

    rt_thread_startup(tid_target);

    return RT_EOK;
}

static void target(int argc, char *argv[])
{
    if (argc < 3)
    {
        return;
    }
    if (!rt_strcmp("init", argv[1]))
    {
        rt_kprintf(argv[2]);
        target_init(argv[2]);
    }
    if (!rt_strcmp("set_device", argv[1]))
    {
        rt_kprintf(argv[2]);
        target_set_device(argv[2]);
    }
}
MSH_CMD_EXPORT(target, target test);

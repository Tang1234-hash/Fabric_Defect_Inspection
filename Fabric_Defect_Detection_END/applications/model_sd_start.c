#include <rtthread.h>
#include <rtdevice.h>
#include <string.h>
#include <dfs_fs.h>
#include "nnom.h"
#include "weights_32.h"
#include <model_sd_start.h>
#include <onenet.h>
#include <cJSON.h>

#define DBG_TAG "defect_inspect"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

// ===================== 配置 =====================
#define UART_DEVICE_NAME         "uart3"
#define BAUD_RATE               BAUD_RATE_9600
#define IMAGE_SIZE              1024
#define BYTES_PER_LINE          16
#define VAL_MAX                 127
#define DEFECT_CLASS_CNT        7

//  最大支持10张图（可改大），运行时由main控制实际数量
#define DEFAULT_DETECTION_ROUND_CNT    10

#define FRAME_HEAD              0xA6//接受到图片的数据头，说明要传图片了
#define FRAME_TAIL_SEQ_LEN      2
#define FRAME_TAIL_SEQ          {0x5D, 0x5B}//连续接收到两个该数据尾，说明图片传完了

const char* defect_names[DEFECT_CLASS_CNT] = {
    "crack",
    "dirt",
    "glue",
    "line",
    "oil",
    "no defect",
    "wear"
};

static rt_device_t uart_dev = RT_NULL;
static uint8_t uart_rx_buf[64];
static uint8_t recv_buf[1032];
static uint32_t recv_cnt = 0;
static uint8_t frame_start = 0;
static uint8_t image_buffer[IMAGE_SIZE];
static nnom_model_t* defect_model = NULL;

extern int8_t nnom_input_data[1024];
extern int8_t nnom_output_data[DEFECT_CLASS_CNT];

//  运行时动态次数（main会修改它）
uint8_t detection_round_cnt = DEFAULT_DETECTION_ROUND_CNT;
rt_mutex_t detection_round_mutex = RT_NULL;

static uint8_t detect_round = 0;
static uint16_t defect_counter[DEFECT_CLASS_CNT];
static uint8_t last_detect[DEFAULT_DETECTION_ROUND_CNT];
static rt_uint8_t frame_ready = 0;

// 互斥锁初始化
int detection_mutex_init(void)
{
    LOG_I("detection_mutex_init start");

    detection_round_mutex = rt_mutex_create("det_mutex", RT_IPC_FLAG_FIFO);
    if (detection_round_mutex == RT_NULL) {
        LOG_E("mutex create failed");
        return -1;
    }
    return 0;
}
INIT_APP_EXPORT(detection_mutex_init);//初始化互斥量

static uint8_t clamp_val(uint8_t val)
{
    return (val > VAL_MAX) ? VAL_MAX : val;
}

static int defect_model_init(void)
{
    if (defect_model != NULL) {
        LOG_W("model already init");
        return 0;
    }
    defect_model = nnom_model_create();
    if (defect_model == NULL) {
        LOG_E("model create fail");
        return -1;
    }
    LOG_I("model init ok");
    return 0;
}

static int parse_output_result(int8_t* output, rt_size_t size)//输出图片识别结果
{
    int8_t max_val = -128;
    rt_uint8_t max_idx = 0;

    for (int i = 0; i < DEFECT_CLASS_CNT; i++) {
        if (output[i] > max_val) {
            max_val = output[i];
            max_idx = i;
        }
    }

    LOG_I("result: %s (%d)", defect_names[max_idx], max_idx);

    rt_mutex_take(detection_round_mutex, RT_WAITING_FOREVER);
    uint8_t total = detection_round_cnt;
    rt_mutex_release(detection_round_mutex);

    if (detect_round < total) {
        last_detect[detect_round] = max_idx;
    }

    defect_counter[max_idx]++;
    return 0;
}

static int prepare_input_data(int8_t* input, rt_size_t size)
{
    rt_memcpy(input, image_buffer, size);
    return 0;
}

static int defect_inspect_main(void)
{
    prepare_input_data(nnom_input_data, IMAGE_SIZE);
    model_run(defect_model);
    parse_output_result(nnom_output_data, DEFECT_CLASS_CNT);
    return 0;
}

static rt_err_t uart_rx_callback(rt_device_t dev, rt_size_t size)//接收图片存在数组里
{
    const uint8_t tail_seq[] = FRAME_TAIL_SEQ;
    rt_size_t len = rt_device_read(dev, 0, uart_rx_buf, sizeof(uart_rx_buf));

    for (rt_size_t i = 0; i < len; i++) {
        uint8_t d = uart_rx_buf[i];
        if (!frame_start) {
            if (d == FRAME_HEAD) {
                frame_start = 1;
                recv_cnt = 0;
                memset(recv_buf, 0, sizeof(recv_buf));
            }
            continue;
        }

        if (recv_cnt < sizeof(recv_buf))
            recv_buf[recv_cnt++] = d;

        if (recv_cnt >= IMAGE_SIZE + FRAME_TAIL_SEQ_LEN) {
            uint32_t pos = recv_cnt - FRAME_TAIL_SEQ_LEN;
            uint8_t match = 1;
            for (uint8_t j = 0; j < FRAME_TAIL_SEQ_LEN; j++) {
                if (recv_buf[pos + j] != tail_seq[j]) {
                    match = 0; break;
                }
            }

            if (match) {
                for (uint32_t j = 0; j < IMAGE_SIZE; j++)
                    image_buffer[j] = clamp_val(recv_buf[j]);
                frame_ready = 1;
                frame_start = 0;
                recv_cnt = 0;
            } else {
                memmove(recv_buf, recv_buf + 1, sizeof(recv_buf) - 1);
                recv_cnt--;
            }
        }
    }
    return RT_EOK;
}

/* ==================== 上传OneNET ==================== */
static int onenet_upload_defect_report(uint8_t total, const uint8_t *detect_results)
{
    // 1. 用 cJSON 构造 {"total":n, "results":["crack","dirt",...]}
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "total", total);
    cJSON *arr = cJSON_AddArrayToObject(root, "results");
    for (int i = 0; i < total; i++) {
        cJSON_AddItemToArray(arr, cJSON_CreateString(defect_names[detect_results[i]]));
    }
    char *value_str = cJSON_PrintUnformatted(root);

    // 2. 构造 OneJson 属性上报 JSON：{"id":"xxx","params":{"defect_report":{"value":...}}}
    char id_str[12];
    snprintf(id_str, sizeof(id_str), "%u", rand());
    cJSON *prop_root = cJSON_CreateObject();
    cJSON_AddStringToObject(prop_root, "id", id_str);
    cJSON *params = cJSON_CreateObject();
    cJSON_AddItemToObject(prop_root, "params", params);
    cJSON *entry = cJSON_CreateObject();
    cJSON_AddStringToObject(entry, "value", value_str);  // value 直接放 JSON 字符串
    cJSON_AddItemToObject(params, "defect_report", entry);

    char *json_str = cJSON_PrintUnformatted(prop_root);
    char topic[128];
    snprintf(topic, sizeof(topic), "$sys/%s/%s/thing/property/post",
             ONENET_INFO_PROID, ONENET_INFO_DEVID);

    int ret = onenet_mqtt_publish(topic, (uint8_t *)json_str, strlen(json_str));
    LOG_I("defect_report %s", ret == 0 ? "posted" : "failed");

    cJSON_Delete(root);
    cJSON_Delete(prop_root);
    cJSON_free(value_str);
    cJSON_free(json_str);
    return ret;
}

/* 上传缺陷计数（各类缺陷数量）*/
static int onenet_upload_defect_count(const uint16_t *defect_counter)
{
    // 1. 构造 {"crack":3, "dirt":0, ...}
    cJSON *root = cJSON_CreateObject();
    for (int i = 0; i < DEFECT_CLASS_CNT; i++) {
        cJSON_AddNumberToObject(root, defect_names[i], defect_counter[i]);
    }
    char *value_str = cJSON_PrintUnformatted(root);

    // 2. 构造 OneJson 属性上报
    char id_str[12];
    snprintf(id_str, sizeof(id_str), "%u", rand());
    cJSON *prop_root = cJSON_CreateObject();
    cJSON_AddStringToObject(prop_root, "id", id_str);
    cJSON *params = cJSON_CreateObject();
    cJSON_AddItemToObject(prop_root, "params", params);
    cJSON *entry = cJSON_CreateObject();
    cJSON_AddStringToObject(entry, "value", value_str);
    cJSON_AddItemToObject(params, "defect_count", entry);

    char *json_str = cJSON_PrintUnformatted(prop_root);
    char topic[128];
    snprintf(topic, sizeof(topic), "$sys/%s/%s/thing/property/post",
             ONENET_INFO_PROID, ONENET_INFO_DEVID);

    int ret = onenet_mqtt_publish(topic, (uint8_t *)json_str, strlen(json_str));
    LOG_I("defect_count %s", ret == 0 ? "posted" : "failed");

    cJSON_Delete(root);
    cJSON_Delete(prop_root);
    cJSON_free(value_str);
    cJSON_free(json_str);
    return ret;
}

static void generate_defect_stat_csv(void)//相关函数见rtthread文档
{
    int fd;
    char filename[64];
    sprintf(filename, "/defect_stat.csv");

    fd = open(filename, O_WRONLY | O_CREAT | O_TRUNC);//获取sd卡文件
    if (fd < 0) return;

    rt_mutex_take(detection_round_mutex, RT_WAITING_FOREVER);
    uint8_t total = detection_round_cnt;
    rt_mutex_release(detection_round_mutex);

    char header[128] = "type,";
    const char* names[] = {"first","second","third","forth","fifth","sixth","seventh","eighth","ninth","tenth"};//限制十张图片
    for (int i = 0; i < total; i++) {
        strcat(header, names[i]);
        if (i != total-1) strcat(header, ",");// , 代表excel右移
    }
    strcat(header, "\n");// \n 代表excel下移
    write(fd, header, strlen(header));

    for (int i = 0; i < DEFECT_CLASS_CNT; i++) {
        char line[256];
        sprintf(line, "%s", defect_names[i]);
        for (int j = 0; j < total; j++) {
            strcat(line, (last_detect[j] == i) ? ",√" : ",");
        }
        strcat(line, "\n");
        write(fd, line, strlen(line));
    }

    close(fd);
    LOG_I("CSV save ok, total %d images", total);

    onenet_upload_defect_report(total, last_detect);
    rt_thread_mdelay(300);   // 可适当调整，200~500 ms
    onenet_upload_defect_count(defect_counter);
    rt_thread_mdelay(200);

    rt_pin_write(58, PIN_HIGH);//蜂鸣器
    rt_thread_mdelay(80);
    rt_pin_write(58, PIN_LOW);
    rt_thread_mdelay(80);
    memset(defect_counter, 0, sizeof(defect_counter));
    memset(last_detect, 0, sizeof(last_detect));
    detect_round = 0;
}

static void uart_recv_thread(void* parameter)
{
    LOG_I("uart_recv_thread started");

    defect_model_init();

    uart_dev = rt_device_find(UART_DEVICE_NAME);
    struct serial_configure cfg = RT_SERIAL_CONFIG_DEFAULT;
    cfg.baud_rate = BAUD_RATE;
    rt_device_control(uart_dev, RT_DEVICE_CTRL_CONFIG, &cfg);
    rt_device_set_rx_indicate(uart_dev, uart_rx_callback);
    rt_device_open(uart_dev, RT_DEVICE_FLAG_INT_RX);

    memset(defect_counter, 0, sizeof(defect_counter));
    memset(last_detect, 0, sizeof(last_detect));
    detect_round = 0;

    while (1) {
        if (frame_ready) {
            frame_ready = 0;
            defect_inspect_main();
            detect_round++;

            rt_mutex_take(detection_round_mutex, RT_WAITING_FOREVER);
            uint8_t total = detection_round_cnt;
            rt_mutex_release(detection_round_mutex);//接收main.c的图片数量

            if (detect_round >= total) {
                generate_defect_stat_csv();
            }
        }
        rt_thread_mdelay(10);
    }
}

int uart_image_recv_app_init(void)
{
    LOG_I("uart_image_recv_app_init start");

    rt_thread_t tid = rt_thread_create("uart_img", uart_recv_thread, RT_NULL, 8192, 10, 10);
    if (tid) rt_thread_startup(tid);
    return 0;
}
INIT_APP_EXPORT(uart_image_recv_app_init);

int sd_init(void)
{
    rt_device_t blk_dev;
    rt_thread_mdelay(300);
    blk_dev = rt_device_find("sd0");
    if (!blk_dev) return -1;
    rt_device_init(blk_dev);

    for (int i = 0; i < 3; i++) {
        if (dfs_mount("sd0", "/", "elm", 0, 0) == 0) {
            rt_kprintf("[SD] mount ok\n");
            return 0;
        }
        rt_thread_mdelay(200);
    }
    rt_kprintf("[SD] mount fail\n");
    return -1;
}

#ifndef APPLICATIONS_MODEL_SD_START_H_
#define APPLICATIONS_MODEL_SD_START_H_

extern uint8_t image_count;//全局变量

int sd_init(void);
int uart_image_recv_app_init(void);

#endif /* APPLICATIONS_MODEL_SD_START_H_ */

#ifndef _ASIC_INNO_
#define _ASIC_INNO_

#define ASIC_CHAIN_NUM                  4
#define ASIC_CHIP_NUM                   72

#define ASIC_CHIP_A_BUCKET              (ASIC_CHAIN_NUM * ASIC_CHIP_NUM)
#define ASIC_INNO_FAN_PWM0_DEVICE_NAME  ("/dev/pwmgen0.0")
//#define ASIC_INNO_FAN_PWM1_DEVICE_NAME  ("/dev/pwmgen0.1")
#define ASIC_INNO_FAN_PWM_STEP          (10)
#define ASIC_INNO_FAN_PWM_DUTY_MAX      (100)
/* 此宏表示PWM载波频率:7KHz,经验值 */
#define ASIC_INNO_FAN_PWM_FREQ_TARGET   (7000)
/* 此宏表示分频比 分频比=50M/PWM载波频率 */
#define ASIC_INNO_FAN_PWM_FREQ          (50000000 / ASIC_INNO_FAN_PWM_FREQ_TARGET)
/* 芯片上电高于100°(<445),告警 */
#define ASIC_INNO_FAN_TEMP_VAL_THRESHOLD (445)
/* chip高于80°后,90% */
#define ASIC_INNO_FAN_TEMP_UP_THRESHOLD (80.0f)
/* chip板低于45°后,60% */
#define ASIC_INNO_FAN_TEMP_DOWN_THRESHOLD (45.0f)
/* 算力板高于90°后,风扇最强 */
#define ASIC_INNO_FAN_TEMP_MAX_THRESHOLD (100.0f)

/* 去掉的最高分和最低分比例 */
#define ASIC_INNO_FAN_TEMP_MARGIN_RATE  (5.0f / 72)
/* 数值越小,温度打印得越频繁 */
#define ASIC_INNO_FAN_CTLR_FREQ_DIV     (1000)

#define WEAK_CHIP_THRESHOLD	5
#define BROKEN_CHIP_THRESHOLD 5
#define WEAK_CHIP_SYS_CLK	(600 * 1000)
#define BROKEN_CHIP_SYS_CLK	(400 * 1000)

#include "asic_inno_cmd.h"

bool check_chip(struct A1_chain *a1, int i);
int chain_detect(struct A1_chain *a1, int idxpll);
bool abort_work(struct A1_chain *a1);

int get_current_ms(void);
bool is_chip_disabled(struct A1_chain *a1, uint8_t chip_id);
void disable_chip(struct A1_chain *a1, uint8_t chip_id);

bool get_nonce(struct A1_chain *a1, uint8_t *nonce, uint8_t *chip_id, uint8_t *job_id);
bool set_work(struct A1_chain *a1, uint8_t chip_id, struct work *work, uint8_t queue_states);
void check_disabled_chips(struct A1_chain *a1, int pllnum);

#endif


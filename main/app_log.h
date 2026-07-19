#ifndef APP_LOG_H
#define APP_LOG_H

/*
 * ============================================================
 *  Silabs app_log → ESP-IDF esp_log 兼容垫片
 *
 *  目的:让 cmd_parser.c 与 EFR32BG22 戒指项目共享同一份源文件,
 *  引擎修bug时两个项目直接同步文件,不产生分叉。
 *
 *  映射说明:
 *  - app_log_info 映射为 ESP_LOGD(Debug级):cmd_parser 内部的
 *    逐字节打印属于调试信息,默认不刷屏;需要时
 *    esp_log_level_set("CMD", ESP_LOG_DEBUG) 打开。
 *  - APP_LOG_NL 置空:Silabs端它是"\n",而 ESP_LOGx 自带换行。
 * ============================================================
 */

#include "esp_log.h"

/*
 * 屏蔽 -Wformat:cmd_parser.c 中 %lu/%lX 按 Silabs ARM 工具链书写
 * (该平台 uint32_t = unsigned long),而 ESP-IDF v5 xtensa 上
 * uint32_t = unsigned int 会触发格式告警。两者同为32位,运行时
 * 行为一致;为保证源文件跨项目零改动共享,在垫片层屏蔽此告警。
 * 注意:仅 #include 本头文件的翻译单元受影响(即 cmd_parser.c 等
 * 与戒指项目共享的文件),本项目自有代码不包含此头,不受影响。
 */
#pragma GCC diagnostic ignored "-Wformat"

#define APP_LOG_NL ""

#define app_log_info(fmt, ...) ESP_LOGD("CMD", fmt, ##__VA_ARGS__)
#define app_log_warning(fmt, ...) ESP_LOGW("CMD", fmt, ##__VA_ARGS__)
#define app_log_error(fmt, ...) ESP_LOGE("CMD", fmt, ##__VA_ARGS__)

#endif // APP_LOG_H
/**
 * @file  app_build.h — 构建标签
 *
 * AGENTS.md §5 规则 4：**每次有效改动升一个构建标签**，冷启动后先在屏上确认标签再判行为。
 * 两块板都要有；motion 用 `R` 序列（当前 R63），本仓用 `E` 序列以区分。
 */

#ifndef APP_BUILD_H
#define APP_BUILD_H

/* 版本标签，单点定义在 app_build.c。全大写（OLED 字库限制，见 platform/Inc/app_oled.h）。 */
extern const char build_tag[];

#endif /* APP_BUILD_H */

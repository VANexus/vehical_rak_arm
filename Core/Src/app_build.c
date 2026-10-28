/**
 * @file  app_build.c — 构建标签（单点定义）
 *
 * 升标签时只改这一个字符串，并在提交说明里写清"升到哪一档、为什么"。
 * 全大写 —— OLED 字库只到 'Z'（见 platform/Inc/app_oled.h 的约束说明）。
 */
#include "app_build.h"

const char build_tag[] = "E3";

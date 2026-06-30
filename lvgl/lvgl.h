#ifndef LVGL_LVGL_H
#define LVGL_LVGL_H

#include <stdint.h>

#ifndef LV_ATTRIBUTE_MEM_ALIGN
#define LV_ATTRIBUTE_MEM_ALIGN
#endif

#ifndef LV_ATTRIBUTE_LARGE_CONST
#define LV_ATTRIBUTE_LARGE_CONST
#endif

#ifndef LV_ATTRIBUTE_IMG
#define LV_ATTRIBUTE_IMG
#endif

#ifndef LV_ATTRIBUTE_IMG_FORWARD
#define LV_ATTRIBUTE_IMG_FORWARD
#endif

#ifndef LV_ATTRIBUTE_IMG_REVERSE
#define LV_ATTRIBUTE_IMG_REVERSE
#endif

#ifndef LV_ATTRIBUTE_IMG_STRAIGHT
#define LV_ATTRIBUTE_IMG_STRAIGHT
#endif

#ifndef LV_ATTRIBUTE_IMG_TURN_LEFT
#define LV_ATTRIBUTE_IMG_TURN_LEFT
#endif

#ifndef LV_ATTRIBUTE_IMG_TURN_RIGHT
#define LV_ATTRIBUTE_IMG_TURN_RIGHT
#endif

#ifndef LV_ATTRIBUTE_IMG_PARKING
#define LV_ATTRIBUTE_IMG_PARKING
#endif

#ifndef LV_ATTRIBUTE_IMG_STOP
#define LV_ATTRIBUTE_IMG_STOP
#endif

#ifndef LV_ATTRIBUTE_IMG_ERROR
#define LV_ATTRIBUTE_IMG_ERROR
#endif

typedef enum
{
	LV_IMG_CF_UNKNOWN = 0,
	LV_IMG_CF_RAW,
	LV_IMG_CF_RAW_ALPHA,
	LV_IMG_CF_RAW_CHROMA_KEYED,
	LV_IMG_CF_TRUE_COLOR,
	LV_IMG_CF_TRUE_COLOR_ALPHA,
	LV_IMG_CF_TRUE_COLOR_CHROMA_KEYED,
	LV_IMG_CF_INDEXED_1BIT,
	LV_IMG_CF_INDEXED_2BIT,
	LV_IMG_CF_INDEXED_4BIT,
	LV_IMG_CF_INDEXED_8BIT,
	LV_IMG_CF_ALPHA_1BIT,
	LV_IMG_CF_ALPHA_2BIT,
	LV_IMG_CF_ALPHA_4BIT,
	LV_IMG_CF_ALPHA_8BIT
} lv_img_cf_t;

typedef struct
{
	uint32_t cf : 5;
	uint32_t always_zero : 3;
	uint32_t reserved : 2;
	uint32_t w : 11;
	uint32_t h : 11;
} lv_img_header_t;

typedef struct
{
	lv_img_header_t header;
	uint32_t data_size;
	const uint8_t *data;
} lv_img_dsc_t;

#define LV_IMAGE_HEADER_MAGIC 0x19U

typedef enum
{
	LV_COLOR_FORMAT_UNKNOWN = 0,
	LV_COLOR_FORMAT_I1 = 1,
	LV_COLOR_FORMAT_A1 = 2
} lv_color_format_t;

typedef struct
{
	uint32_t magic;
	uint32_t cf;
	uint32_t flags;
	uint32_t w;
	uint32_t h;
	uint32_t stride;
	uint32_t reserved_2;
} lv_image_header_t;

typedef struct
{
	lv_image_header_t header;
	uint32_t data_size;
	const uint8_t *data;
	const void *reserved;
} lv_image_dsc_t;

#endif

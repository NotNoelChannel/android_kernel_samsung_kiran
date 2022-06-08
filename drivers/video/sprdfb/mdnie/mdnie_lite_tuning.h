/*
 * Copyright (C) 2014 Samsung Electronics Co.Ltd
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#ifndef _MDNIE_LITE_TUNING_H_
#define _MDNIE_LITE_TUNING_H_

#define SIG_MDNIE_UI_MODE 0
#define SIG_MDNIE_GALLERY 1
#define SIG_MDNIE_VIDEO_MODE 2
#define SIG_MDNIE_VT 3
#define SIG_MDNIE_CAMERA_MODE 4
#define SIG_MDNIE_BROWSER 5
#define SIG_MDNIE_NEGATIVE 6
#define SIG_MDNIE_EMAIL 7
#define SIG_MDNIE_EBOOK 8
#define SIG_MDNIE_OUTDOOR 9
#define SIG_MDNIE_GRAY 10

#define SIG_MDNIE_DYNAMIC 0
#define SIG_MDNIE_STANDARD 1
#define SIG_MDNIE_MOVIE 2

#ifdef BROWSER_COLOR_TONE_SET
#define SIG_MDNIE_BROWSER_TONE1	40
#define SIG_MDNIE_BROWSER_TONE2	41
#define SIG_MDNIE_BROWSER_TONE3	42
#endif

enum lcd_mdnie_mode {
	MDNIE_UI_MODE,
	MDNIE_GALLERY,
	MDNIE_VIDEO_MODE,
	MDNIE_VT_MODE,
	MDNIE_CAMERA_MODE,
	MDNIE_BROWSER_MODE,
	MDNIE_NEGATIVE_MODE,
	MDNIE_EMAIL_MODE,
	MDNIE_EBOOK_MODE,
	MDNIE_GRAY_MODE,
	MAX_MDNIE_MODE,
};

enum lcd_mdnie_negative {
	MDNIE_NEGATIVE_OFF = 0,
	MDNIE_NEGATIVE_ON,
};

enum background_mode {
	DYNAMIC_MODE = 0,
	STANDARD_MODE,
	NATURAL_MODE,
	MOVIE_MODE,
	AUTO_MODE,
	MAX_BACKGROUND_MODE,
};

enum outdoor_mode {
	OUTDOOR_OFF_MODE = 0,
	OUTDOOR_ON_MODE,
	MAX_OUTDOOR_MODE,
};

enum accessibility {
	ACCESSIBILITY_OFF,
	NEGATIVE,
	COLOR_BLIND,
	ACCESSIBILITY_MAX,
};

struct mdnie_lite_tun_type {
	bool mdnie_enable;
	enum background_mode background;
	enum outdoor_mode outdoor;
	enum lcd_mdnie_mode scenario;
	enum lcd_mdnie_negative negative;
	enum accessibility blind;
};

unsigned int mdss_dsi_show_cabc(void);
void mdss_dsi_store_cabc(unsigned int cabc);
void init_mdnie_class(void);
void is_negative_on(void);
void mdnie_lite_tuning_init(struct sprdfb_device *dev);
#ifdef CONFIG_FB_VSYNC_SUPPORT
extern int32_t sprdfb_dispc_wait_for_vsync(struct sprdfb_device *dev);
#endif
#endif /*_MDNIE_LITE_TUNING_H_*/

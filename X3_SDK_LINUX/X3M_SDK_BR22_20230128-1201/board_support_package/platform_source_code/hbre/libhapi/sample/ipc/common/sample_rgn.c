/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @file:
 * @brief:
 * @author:
 * @email:
 * @date:
 */

#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <getopt.h>

#include "hb_rgn.h"
#include "sample.h"

void* sample_rgn_thread(void *rgnparams)
{
    int ret = 0;
    RGN_CHN_S chn;
    RGN_ATTR_S pstRegion;
    RGN_BITMAP_S bitmapAttr;
    RGN_DRAW_WORD_S drawWord;
    RGN_CHN_ATTR_S chn_attr;
    int w, h, x, y, rgnChnTime, rgnChnPlace;
    time_t tt,tt_last=0;
    rgnParam *rgnparam = (rgnParam*)rgnparams;

    rgnChnTime = rgnparam->rgnChn;
    rgnChnPlace = rgnparam->rgnChn + 6;
    x = rgnparam->x;
    y = rgnparam->y;
    w = rgnparam->w;
    h = rgnparam->h;

    chn.s32PipelineId = 0;
    if (rgnChnTime == 0)
	    chn.enChnId = CHN_DS0;
    else if (rgnChnTime == 1)
	    chn.enChnId = CHN_DS1;
    else if (rgnChnTime == 2)
	    chn.enChnId = CHN_US;
    else if (rgnChnTime == 3)
	    chn.enChnId = CHN_DS2;
    else if (rgnChnTime == 4)
	    chn.enChnId = CHN_DS3;
    else if (rgnChnTime == 5)
	    chn.enChnId = CHN_DS4;
    else 
        return NULL;

    pstRegion.enType = OVERLAY_RGN;
    pstRegion.stOverlayAttr.stSize.u32Width = w;
    pstRegion.stOverlayAttr.stSize.u32Height = h;
    pstRegion.stOverlayAttr.enPixelFmt = PIXEL_FORMAT_VGA_4;
    pstRegion.stOverlayAttr.enBgColor  = 16;

    chn_attr.bShow = true;
    chn_attr.bInvertEn = false;
    // chn_attr.enType = OVERLAY_RGN;
    chn_attr.unChnAttr.stOverlayChn.stPoint.u32X = x;
    chn_attr.unChnAttr.stOverlayChn.stPoint.u32Y = y;
    // chn_attr.unChnAttr.stOverlayChn.u32Layer = 3;

    bitmapAttr.enPixelFormat = PIXEL_FORMAT_VGA_4;
    bitmapAttr.stSize = pstRegion.stOverlayAttr.stSize;
    bitmapAttr.pAddr = malloc(w * h / 2);
    memset(bitmapAttr.pAddr, 0xff, w * h / 2);

    ret = HB_RGN_Create(rgnChnTime, &pstRegion);
    if (ret) {
        printf("HB_RGN_Create failed\n");
        return NULL;
    }
    ret = HB_RGN_Create(rgnChnPlace, &pstRegion);
    if (ret) {
        printf("HB_RGN_Create failed\n");
        return NULL;
    }

    ret = HB_RGN_AttachToChn(rgnChnTime, &chn, &chn_attr);
    if (ret) {
        printf("HB_RGN_AttachToChn failed\n");
        return NULL;
    }
    chn_attr.unChnAttr.stOverlayChn.stPoint.u32Y += 100;
    ret = HB_RGN_AttachToChn(rgnChnPlace, &chn, &chn_attr);
    if (ret) {
        printf("HB_RGN_AttachToChn failed\n");
        return NULL;
    }

    // drawWord.bInvertEn = 1;
    drawWord.enFontSize = FONT_SIZE_MEDIUM;
    drawWord.enFontColor = FONT_COLOR_WHITE;
    drawWord.stPoint.u32X = 0;
    drawWord.stPoint.u32Y = 0;
    drawWord.bFlushEn = false;
    // unsigned char str[10] = {0xce, 0xd2, 0xce, 0xd2,
    //		 0xce, 0xd2, 0xce, 0xd2, 0xce, 0xd2};

    char str[32] = {0xb5, 0xd8, 0xb5, 0xe3, 0xca, 0xbe, 0xc0, 0xfd};
    drawWord.pu8Str = (uint8_t*)str;
    drawWord.pAddr = bitmapAttr.pAddr;
    drawWord.stSize = bitmapAttr.stSize;

    ret = HB_RGN_DrawWord(rgnChnPlace, &drawWord);
    if (ret) {
        printf("HB_RGN_DrawWord failed\n");
        return NULL;
    }

    ret = HB_RGN_SetBitMap(rgnChnPlace, &bitmapAttr);
    if (ret) {
        printf("HB_RGN_SetBitMap failed\n");
        return NULL;
    }

    while(g_exit == 0) {
        tt = time(0);
        if (tt>tt_last) {
            char str[32];
            strftime(str, sizeof(str), "%Y-%m-%d %H:%M:%S", localtime(&tt));
            drawWord.pu8Str = (uint8_t*)str;
            // drawWord.u32LumThresh = 120;
            drawWord.pAddr = bitmapAttr.pAddr;
            drawWord.stSize = bitmapAttr.stSize;
            // printf("================ osd: %s\n", str);
            ret = HB_RGN_DrawWord(rgnChnTime, &drawWord);
            if (ret) {
                printf("HB_RGN_DrawWord failed\n");
                return NULL;
            }
            ret = HB_RGN_SetBitMap(rgnChnTime, &bitmapAttr);
            if (ret) {
                printf("HB_RGN_SetBitMap failed\n");
                return NULL;
            }
            tt_last = tt;
        }
        usleep(100000);
    }

    ret = HB_RGN_DetachFromChn(rgnChnPlace, &chn);
    if (ret) {
        printf("HB_RGN_DetachFromChn failed\n");
        return NULL;
    }
    ret = HB_RGN_DetachFromChn(rgnChnTime, &chn);
    if (ret) {
        printf("HB_RGN_DetachFromChn failed\n");
        return NULL;
    }

	ret = HB_RGN_Destroy(rgnChnTime);
    if (ret) {
        printf("HB_RGN_Destroy failed\n");
        return NULL;
    }
    ret = HB_RGN_Destroy(rgnChnPlace);
    if (ret) {
        printf("HB_RGN_Destroy failed\n");
        return NULL;
    }

    free(bitmapAttr.pAddr);
    printf("========== quit osd thread %d\n", chn.enChnId);
    return NULL;
}

// void *rgn_init(int handles)
// {
// 	RGN_HANDLE handle = handles;
// 	RGN_CHN_S chn;
//     int need_overlay = 1;
//     int need_attr = 0;
//     int need_invert = 1;
//     int need_line = 1;
//     int need_canvas = 0;
//     int need_cover = 1;
//     int need_word = 1;
//     int need_batch = 0;

//     chn.s32PipelineId = 0;
//     if (handles == 0)
// 	    chn.enChnId = CHN_DS0;
//     else if (handles == 1)
// 	    chn.enChnId = CHN_DS1;
//     else if (handles == 2)
// 	    chn.enChnId = CHN_US;

// 	if (need_overlay) {
// 		RGN_ATTR_S pstRegion;
// 		RGN_CANVAS_S pstCanvasInfo;
// 		RGN_BITMAP_S bitmapAttr;
// 		RGN_DRAW_WORD_S drawWord;
// 		RGN_DRAW_LINE_S drawLine;
// 		int flag;
// 		RGN_CHN_ATTR_S chn_attr;

// 		pstRegion.enType = OVERLAY_RGN;
// 		pstRegion.stOverlayAttr.stSize.u32Width = 700;
// 		pstRegion.stOverlayAttr.stSize.u32Height = 100;
// 		pstRegion.stOverlayAttr.enPixelFmt = PIXEL_FORMAT_VGA_4;

// 		chn_attr.bShow = true;
// 		chn_attr.enType = OVERLAY_RGN;
// 		chn_attr.unChnAttr.stOverlayChn.stPoint.u32X = 0;
// 		chn_attr.unChnAttr.stOverlayChn.stPoint.u32Y = handle * 100;
// 		chn_attr.unChnAttr.stOverlayChn.u32Layer = 3;

// 		bitmapAttr.enPixelFormat = PIXEL_FORMAT_VGA_4;
// 		bitmapAttr.stSize = pstRegion.stOverlayAttr.stSize;
// 		bitmapAttr.pAddr = malloc(700 * 50);
// 		memset(bitmapAttr.pAddr, 0xff, 700 * 50);

// 		HB_RGN_Create(handle, &pstRegion);

// 		if (need_attr) {
// 			HB_RGN_GetAttr(handle, &pstRegion);
// 			printf("region(%d) get region attribute width(%d) height(%d)\n",
// 				handle,
// 				pstRegion.stOverlayAttr.stSize.u32Width,
// 				pstRegion.stOverlayAttr.stSize.u32Height);

// 			pstRegion.stOverlayAttr.stSize.u32Width = 650;
// 			HB_RGN_SetAttr(handle, &pstRegion);
// 			HB_RGN_GetAttr(handle, &pstRegion);
// 			printf("region(%d) modify done, get region attribute width(%d)"
// 					" height(%d)\n",
// 				handle,
// 				pstRegion.stOverlayAttr.stSize.u32Width,
// 				pstRegion.stOverlayAttr.stSize.u32Height);
// 		}

// 		HB_RGN_AttachToChn(handle, &chn, &chn_attr);

// 		if (need_attr) {
// 			HB_RGN_GetDisplayAttr(handle, &chn, &chn_attr);
// 			printf("region(%d) get chn attribute x(%d) y(%d)\n",
// 				handle,
// 				chn_attr.unChnAttr.stOverlayChn.stPoint.u32X,
// 				chn_attr.unChnAttr.stOverlayChn.stPoint.u32Y);

// 			chn_attr.unChnAttr.stOverlayChn.stPoint.u32X = 20;
// 			HB_RGN_SetDisplayAttr(handle, &chn, &chn_attr);
// 			HB_RGN_GetDisplayAttr(handle, &chn, &chn_attr);
// 			printf("region(%d) modify attribute done, get chn attribute "
// 					"x(%d) y(%d)\n",
// 				handle,
// 				chn_attr.unChnAttr.stOverlayChn.stPoint.u32X,
// 				chn_attr.unChnAttr.stOverlayChn.stPoint.u32Y);
// 		}

// 		if (need_word) {
// 			drawWord.bInvertEn = need_invert;
// 			drawWord.enFontSize = FONT_SIZE_MEDIUM;
// 			drawWord.enFontColor = FONT_COLOR_WHITE;
// 			drawWord.stPoint.u32X = 0;
// 			drawWord.stPoint.u32Y = 0;
// 			// unsigned char str[10] = {0xce, 0xd2, 0xce, 0xd2,
// 			//		 0xce, 0xd2, 0xce, 0xd2, 0xce, 0xd2};
// 			time_t tt = time(0);
// 			char str[32];
// 			strftime(str, sizeof(str), "%Y-%m-%d %H:%M:%S", localtime(&tt));
// 			drawWord.pu8Str = str;
// 			drawWord.u32LumThresh = 120;
// 			drawWord.pAddr = bitmapAttr.pAddr;
// 			drawWord.stSize = bitmapAttr.stSize;

// 			if (need_canvas) {
// 				HB_RGN_GetCanvasInfo(handle, &pstCanvasInfo);
// 				drawWord.pAddr = pstCanvasInfo.pAddr;
// 			}
// 			HB_RGN_DrawWord(handle, &chn, &drawWord);
// 		}

// 		if (need_line) {
// 			drawLine.stStartPoint.u32X = 400;
// 			drawLine.stStartPoint.u32Y = 0;
// 			drawLine.stEndPoint.u32X = 500;
// 			drawLine.stEndPoint.u32Y = 100;
// 			drawLine.pAddr = bitmapAttr.pAddr;
// 			drawLine.stSize = bitmapAttr.stSize;
// 			drawLine.u32Color = FONT_COLOR_WHITE;
// 			drawLine.u32Thick = 4;

// 			RGN_DRAW_LINE_S drawLine2 = drawLine;
// 			drawLine2.stEndPoint.u32X = 600;
// 			drawLine2.stEndPoint.u32Y = 100;
// 			RGN_DRAW_LINE_S drawLineArray[2] = {drawLine, drawLine2};

// 			if (need_canvas) {
// 				HB_RGN_GetCanvasInfo(handle, &pstCanvasInfo);
// 				drawLine.pAddr = pstCanvasInfo.pAddr;
// 				drawLine2.pAddr = pstCanvasInfo.pAddr;
// 			}
// 			HB_RGN_DrawLineArray(handle, drawLineArray, 2);
// 		}

// 		if (need_canvas) {
// 			if (!need_batch) {
// 				HB_RGN_UpdateCanvas(handle);
// 			}
// 		} else {
// 			HB_RGN_SetBitMap(handle, &bitmapAttr);
// 		}

// 		free(bitmapAttr.pAddr);
// 	}

// 	if (need_cover) {
// 		RGN_HANDLE handle_cover = handle + 5;
// 		RGN_CHN_ATTR_S chn_attr_cover;
// 		RGN_ATTR_S pstRegion_cover;

// 		pstRegion_cover.enType = COVER_RGN;
// 		pstRegion_cover.stOverlayAttr.enPixelFmt = PIXEL_FORMAT_VGA_4;

// 		chn_attr_cover.bShow = true;
// 		chn_attr_cover.enType = COVER_RGN;
// 		chn_attr_cover.unChnAttr.stCoverChn.stRect.u32X = 700;
// 		chn_attr_cover.unChnAttr.stCoverChn.stRect.u32Y = handle * 100;
// 		chn_attr_cover.unChnAttr.stCoverChn.stRect.u32Width = 100;
// 		chn_attr_cover.unChnAttr.stCoverChn.stRect.u32Height = 100;
// 		chn_attr_cover.unChnAttr.stCoverChn.u32Color = FONT_COLOR_WHITE;
// 		chn_attr_cover.unChnAttr.stCoverChn.u32Layer = 4;
// 		HB_RGN_Create(handle_cover, &pstRegion_cover);
// 		HB_RGN_AttachToChn(handle_cover, &chn, &chn_attr_cover);
// 	}

// }
/* SPDX-License-Identifier: GPL-2.0 */

#ifndef _DT_BINDINGS_IIO_ADC_INGENIC_ADC_H
#define _DT_BINDINGS_IIO_ADC_INGENIC_ADC_H

/* ADC channel idx. */
#define INGENIC_ADC_AUX		0
#define INGENIC_ADC_BATTERY	1
#define INGENIC_ADC_AUX2	2
#define INGENIC_ADC_TOUCH_XP	3
#define INGENIC_ADC_TOUCH_YP	4
#define INGENIC_ADC_TOUCH_XN	5
#define INGENIC_ADC_TOUCH_YN	6
#define INGENIC_ADC_TOUCH_XD	7
#define INGENIC_ADC_TOUCH_YD	8
#define INGENIC_ADC_AUX0	9

/* Special combined ADC XYZ touch channels for JZ4740,
   must only use one of the supported combinations:
   Xd -> Yd
   Xs -> Ys
   Xd -> Yd -> Zs
   Xs -> Ys -> Zs
   Xd -> Yd -> Z1d -> Z2d */
#define INGENIC_ADC_TOUCH_XYZ_XD	16
#define INGENIC_ADC_TOUCH_XYZ_YD	17
#define INGENIC_ADC_TOUCH_XYZ_XS	18
#define INGENIC_ADC_TOUCH_XYZ_YS	19
#define INGENIC_ADC_TOUCH_XYZ_Z1	20
#define INGENIC_ADC_TOUCH_XYZ_Z2	21
#define INGENIC_ADC_TOUCH_XYZ_ZS	22

#endif

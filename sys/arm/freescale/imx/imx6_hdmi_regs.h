/*-
 * Copyright (c) 2015 Oleksandr Tymoshenko <gonzo@freebsd.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#ifndef	__IMX6__HDMI_REGS_H__
#define	__IMX6__HDMI_REGS_H__
#define	HDMI_DESIGN_ID		0x0000
#define	HDMI_REVISION_ID	0x0001
#define	HDMI_PRODUCT_ID0	0x0002
#define	HDMI_PRODUCT_ID1	0x0003

/* Interrupt Registers */
#define	HDMI_IH_FC_STAT0                        0x0100
#define	HDMI_IH_FC_STAT1                        0x0101
#define	HDMI_IH_FC_STAT2                        0x0102
#define	HDMI_IH_AS_STAT0                        0x0103
#define	HDMI_IH_PHY_STAT0                       0x0104
#define		HDMI_IH_PHY_STAT0_HPD	(1 << 0)
#define	HDMI_IH_I2CM_STAT0                      0x0105
#define	HDMI_IH_CEC_STAT0                       0x0106
#define	HDMI_IH_VP_STAT0                        0x0107
#define	HDMI_IH_I2CMPHY_STAT0                   0x0108
#define	HDMI_IH_AHBDMAAUD_STAT0                 0x0109

#define	HDMI_IH_MUTE_FC_STAT0                   0x0180
#define	HDMI_IH_MUTE_FC_STAT1                   0x0181
#define	HDMI_IH_MUTE_FC_STAT2                   0x0182
#define		HDMI_IH_MUTE_FC_STAT2_OVERFLOW_MASK	(0x3)
#define	HDMI_IH_MUTE_AS_STAT0                   0x0183
#define	HDMI_IH_MUTE_PHY_STAT0                  0x0184
#define	HDMI_IH_MUTE_I2CM_STAT0                 0x0185
#define	HDMI_IH_MUTE_CEC_STAT0                  0x0186
#define	HDMI_IH_MUTE_VP_STAT0                   0x0187
#define	HDMI_IH_MUTE_I2CMPHY_STAT0              0x0188
#define	HDMI_IH_MUTE_AHBDMAAUD_STAT0            0x0189
#define	HDMI_IH_MUTE                            0x01FF
#define		HDMI_IH_MUTE_MUTE_WAKEUP_INTERRUPT	(1<<1)
#define		HDMI_IH_MUTE_MUTE_ALL_INTERRUPT		(1<<0)

/* Video Sample Registers */
#define HDMI_TX_INVID0                          0x0200
#define		HDMI_TX_INVID0_INTERNAL_DE_GENERATOR_MASK	0x80
#define		HDMI_TX_INVID0_INTERNAL_DE_GENERATOR_ENABLE	0x80
#define		HDMI_TX_INVID0_INTERNAL_DE_GENERATOR_DISABLE	0x00
#define		HDMI_TX_INVID0_VIDEO_MAPPING_MASK		0x1F
#define		HDMI_TX_INVID0_VIDEO_MAPPING_OFFSET		0
#define HDMI_TX_INSTUFFING                      0x0201
#define		HDMI_TX_INSTUFFING_BDBDATA_STUFFING_MASK	0x4
#define		HDMI_TX_INSTUFFING_BDBDATA_STUFFING_ENABLE	0x4
#define		HDMI_TX_INSTUFFING_BDBDATA_STUFFING_DISABLE	0x0
#define		HDMI_TX_INSTUFFING_RCRDATA_STUFFING_MASK	0x2
#define		HDMI_TX_INSTUFFING_RCRDATA_STUFFING_ENABLE	0x2
#define		HDMI_TX_INSTUFFING_RCRDATA_STUFFING_DISABLE	0x0
#define		HDMI_TX_INSTUFFING_GYDATA_STUFFING_MASK		0x1
#define		HDMI_TX_INSTUFFING_GYDATA_STUFFING_ENABLE	0x1
#define		HDMI_TX_INSTUFFING_GYDATA_STUFFING_DISABLE	0x0
#define HDMI_TX_GYDATA0                         0x0202
#define HDMI_TX_GYDATA1                         0x0203
#define HDMI_TX_RCRDATA0                        0x0204
#define HDMI_TX_RCRDATA1                        0x0205
#define HDMI_TX_BCBDATA0                        0x0206
#define HDMI_TX_BCBDATA1                        0x0207

/* Video Packetizer Registers */
#define	HDMI_VP_STATUS                          0x0800
#define	HDMI_VP_PR_CD                           0x0801
#define		HDMI_VP_PR_CD_COLOR_DEPTH_MASK		0xF0
#define		HDMI_VP_PR_CD_COLOR_DEPTH_OFFSET	4
#define		HDMI_VP_PR_CD_DESIRED_PR_FACTOR_MASK	0x0F
#define		HDMI_VP_PR_CD_DESIRED_PR_FACTOR_OFFSET	0

#define	HDMI_VP_STUFF                           0x0802
#define		HDMI_VP_STUFF_IDEFAULT_PHASE_MASK		0x20
#define		HDMI_VP_STUFF_IDEFAULT_PHASE_OFFSET		5
#define		HDMI_VP_STUFF_IFIX_PP_TO_LAST_MASK		0x10
#define		HDMI_VP_STUFF_IFIX_PP_TO_LAST_OFFSET		4
#define		HDMI_VP_STUFF_ICX_GOTO_P0_ST_MASK		0x8
#define		HDMI_VP_STUFF_ICX_GOTO_P0_ST_OFFSET		3
#define		HDMI_VP_STUFF_YCC422_STUFFING_MASK		0x4
#define		HDMI_VP_STUFF_YCC422_STUFFING_STUFFING_MODE	0x4
#define		HDMI_VP_STUFF_YCC422_STUFFING_DIRECT_MODE	0x0
#define		HDMI_VP_STUFF_PP_STUFFING_MASK			0x2
#define		HDMI_VP_STUFF_PP_STUFFING_STUFFING_MODE		0x2
#define		HDMI_VP_STUFF_PP_STUFFING_DIRECT_MODE		0x0
#define		HDMI_VP_STUFF_PR_STUFFING_MASK			0x1
#define		HDMI_VP_STUFF_PR_STUFFING_STUFFING_MODE		0x1
#define		HDMI_VP_STUFF_PR_STUFFING_DIRECT_MODE		0x0
#define	HDMI_VP_REMAP                           0x0803
#define		HDMI_VP_REMAP_MASK			0x3
#define		HDMI_VP_REMAP_YCC422_24BIT		0x2
#define		HDMI_VP_REMAP_YCC422_20BIT		0x1
#define		HDMI_VP_REMAP_YCC422_16BIT		0x0
#define	HDMI_VP_CONF                            0x0804
#define		HDMI_VP_CONF_BYPASS_EN_MASK	0x40
#define		HDMI_VP_CONF_BYPASS_EN_ENABLE	0x40
#define		HDMI_VP_CONF_BYPASS_EN_DISABLE	0x00
#define		HDMI_VP_CONF_PP_EN_ENMASK	0x20
#define		HDMI_VP_CONF_PP_EN_ENABLE	0x20
#define		HDMI_VP_CONF_PP_EN_DISABLE	0x00
#define		HDMI_VP_CONF_PR_EN_MASK		0x10
#define		HDMI_VP_CONF_PR_EN_ENABLE	0x10
#define		HDMI_VP_CONF_PR_EN_DISABLE	0x00
#define		HDMI_VP_CONF_YCC422_EN_MASK	0x8
#define		HDMI_VP_CONF_YCC422_EN_ENABLE	0x8
#define		HDMI_VP_CONF_YCC422_EN_DISABLE	0x0
#define		HDMI_VP_CONF_BYPASS_SELECT_MASK	0x4
#define		HDMI_VP_CONF_BYPASS_SELECT_VID_PACKETIZER	0x4
#define		HDMI_VP_CONF_BYPASS_SELECT_PIX_REPEATER	0x0
#define		HDMI_VP_CONF_OUTPUT_SELECTOR_MASK	0x3
#define		HDMI_VP_CONF_OUTPUT_SELECTOR_BYPASS	0x3
#define		HDMI_VP_CONF_OUTPUT_SELECTOR_YCC422	0x1
#define		HDMI_VP_CONF_OUTPUT_SELECTOR_PP	0x0
#define	HDMI_VP_STAT                            0x0805
#define	HDMI_VP_INT                             0x0806
#define	HDMI_VP_MASK                            0x0807
#define	HDMI_VP_POL                             0x0808

/* Frame Composer Registers */
#define	HDMI_FC_INVIDCONF                       0x1000
#define		HDMI_FC_INVIDCONF_VSYNC_IN_POLARITY_ACTIVE_HIGH	0x40
#define		HDMI_FC_INVIDCONF_VSYNC_IN_POLARITY_ACTIVE_LOW	0x00
#define		HDMI_FC_INVIDCONF_HSYNC_IN_POLARITY_ACTIVE_HIGH	0x20
#define		HDMI_FC_INVIDCONF_HSYNC_IN_POLARITY_ACTIVE_LOW	0x00
#define		HDMI_FC_INVIDCONF_DE_IN_POLARITY_ACTIVE_HIGH	0x10
#define		HDMI_FC_INVIDCONF_DE_IN_POLARITY_ACTIVE_LOW	0x00
#define		HDMI_FC_INVIDCONF_DVI_MODEZ_HDMI_MODE		0x8
#define		HDMI_FC_INVIDCONF_DVI_MODEZ_DVI_MODE		0x0
#define		HDMI_FC_INVIDCONF_R_V_BLANK_IN_OSC_ACTIVE_HIGH	0x2
#define		HDMI_FC_INVIDCONF_R_V_BLANK_IN_OSC_ACTIVE_LOW	0x0
#define		HDMI_FC_INVIDCONF_IN_I_P_INTERLACED		0x1
#define		HDMI_FC_INVIDCONF_IN_I_P_PROGRESSIVE		0x0
#define	HDMI_FC_INHACTV0                        0x1001
#define	HDMI_FC_INHACTV1                        0x1002
#define	HDMI_FC_INHBLANK0                       0x1003
#define	HDMI_FC_INHBLANK1                       0x1004
#define	HDMI_FC_INVACTV0                        0x1005
#define	HDMI_FC_INVACTV1                        0x1006
#define	HDMI_FC_INVBLANK                        0x1007
#define	HDMI_FC_HSYNCINDELAY0                   0x1008
#define	HDMI_FC_HSYNCINDELAY1                   0x1009
#define	HDMI_FC_HSYNCINWIDTH0                   0x100A
#define	HDMI_FC_HSYNCINWIDTH1                   0x100B
#define	HDMI_FC_VSYNCINDELAY                    0x100C
#define	HDMI_FC_VSYNCINWIDTH                    0x100D
#define	HDMI_FC_INFREQ0                         0x100E
#define	HDMI_FC_INFREQ1                         0x100F
#define	HDMI_FC_INFREQ2                         0x1010
#define	HDMI_FC_CTRLDUR                         0x1011
#define	HDMI_FC_EXCTRLDUR                       0x1012
#define	HDMI_FC_EXCTRLSPAC                      0x1013
#define	HDMI_FC_CH0PREAM                        0x1014
#define	HDMI_FC_CH1PREAM                        0x1015
#define	HDMI_FC_CH2PREAM                        0x1016
#define	HDMI_FC_AVICONF3                        0x1017
#define	HDMI_FC_GCP                             0x1018
#define	HDMI_FC_AVICONF0                        0x1019
#define	HDMI_FC_AVICONF1                        0x101A
#define	HDMI_FC_AVICONF2                        0x101B
#define	HDMI_FC_AVIVID                          0x101C
#define	HDMI_FC_AVIETB0                         0x101D
#define	HDMI_FC_AVIETB1                         0x101E
#define	HDMI_FC_AVISBB0                         0x101F
#define	HDMI_FC_AVISBB1                         0x1020
#define	HDMI_FC_AVIELB0                         0x1021
#define	HDMI_FC_AVIELB1                         0x1022
#define	HDMI_FC_AVISRB0                         0x1023
#define	HDMI_FC_AVISRB1                         0x1024
#define	HDMI_FC_AUDICONF0                       0x1025
#define	HDMI_FC_AUDICONF1                       0x1026
#define	HDMI_FC_AUDICONF2                       0x1027
#define	HDMI_FC_AUDICONF3                       0x1028
#define	HDMI_FC_VSDIEEEID0                      0x1029
#define	HDMI_FC_VSDSIZE                         0x102A
#define	HDMI_FC_VSDIEEEID1                      0x1030
#define	HDMI_FC_VSDIEEEID2                      0x1031
#define	HDMI_FC_VSDPAYLOAD0                     0x1032
#define	HDMI_FC_VSDPAYLOAD1                     0x1033
#define	HDMI_FC_VSDPAYLOAD2                     0x1034
#define	HDMI_FC_VSDPAYLOAD3                     0x1035
#define	HDMI_FC_VSDPAYLOAD4                     0x1036
#define	HDMI_FC_VSDPAYLOAD5                     0x1037
#define	HDMI_FC_VSDPAYLOAD6                     0x1038
#define	HDMI_FC_VSDPAYLOAD7                     0x1039
#define	HDMI_FC_VSDPAYLOAD8                     0x103A
#define	HDMI_FC_VSDPAYLOAD9                     0x103B
#define	HDMI_FC_VSDPAYLOAD10                    0x103C
#define	HDMI_FC_VSDPAYLOAD11                    0x103D
#define	HDMI_FC_VSDPAYLOAD12                    0x103E
#define	HDMI_FC_VSDPAYLOAD13                    0x103F
#define	HDMI_FC_VSDPAYLOAD14                    0x1040
#define	HDMI_FC_VSDPAYLOAD15                    0x1041
#define	HDMI_FC_VSDPAYLOAD16                    0x1042
#define	HDMI_FC_VSDPAYLOAD17                    0x1043
#define	HDMI_FC_VSDPAYLOAD18                    0x1044
#define	HDMI_FC_VSDPAYLOAD19                    0x1045
#define	HDMI_FC_VSDPAYLOAD20                    0x1046
#define	HDMI_FC_VSDPAYLOAD21                    0x1047
#define	HDMI_FC_VSDPAYLOAD22                    0x1048
#define	HDMI_FC_VSDPAYLOAD23                    0x1049
#define	HDMI_FC_SPDVENDORNAME0                  0x104A
#define	HDMI_FC_SPDVENDORNAME1                  0x104B
#define	HDMI_FC_SPDVENDORNAME2                  0x104C
#define	HDMI_FC_SPDVENDORNAME3                  0x104D
#define	HDMI_FC_SPDVENDORNAME4                  0x104E
#define	HDMI_FC_SPDVENDORNAME5                  0x104F
#define	HDMI_FC_SPDVENDORNAME6                  0x1050
#define	HDMI_FC_SPDVENDORNAME7                  0x1051
#define	HDMI_FC_SDPPRODUCTNAME0                 0x1052
#define	HDMI_FC_SDPPRODUCTNAME1                 0x1053
#define	HDMI_FC_SDPPRODUCTNAME2                 0x1054
#define	HDMI_FC_SDPPRODUCTNAME3                 0x1055
#define	HDMI_FC_SDPPRODUCTNAME4                 0x1056
#define	HDMI_FC_SDPPRODUCTNAME5                 0x1057
#define	HDMI_FC_SDPPRODUCTNAME6                 0x1058
#define	HDMI_FC_SDPPRODUCTNAME7                 0x1059
#define	HDMI_FC_SDPPRODUCTNAME8                 0x105A
#define	HDMI_FC_SDPPRODUCTNAME9                 0x105B
#define	HDMI_FC_SDPPRODUCTNAME10                0x105C
#define	HDMI_FC_SDPPRODUCTNAME11                0x105D
#define	HDMI_FC_SDPPRODUCTNAME12                0x105E
#define	HDMI_FC_SDPPRODUCTNAME13                0x105F
#define	HDMI_FC_SDPPRODUCTNAME14                0x1060
#define	HDMI_FC_SPDPRODUCTNAME15                0x1061
#define	HDMI_FC_SPDDEVICEINF                    0x1062
#define	HDMI_FC_AUDSCONF                        0x1063
#define	HDMI_FC_AUDSSTAT                        0x1064
#define	HDMI_FC_DATACH0FILL                     0x1070
#define	HDMI_FC_DATACH1FILL                     0x1071
#define	HDMI_FC_DATACH2FILL                     0x1072
#define	HDMI_FC_CTRLQHIGH                       0x1073
#define	HDMI_FC_CTRLQLOW                        0x1074
#define	HDMI_FC_ACP0                            0x1075
#define	HDMI_FC_ACP28                           0x1076
#define	HDMI_FC_ACP27                           0x1077
#define	HDMI_FC_ACP26                           0x1078
#define	HDMI_FC_ACP25                           0x1079
#define	HDMI_FC_ACP24                           0x107A
#define	HDMI_FC_ACP23                           0x107B
#define	HDMI_FC_ACP22                           0x107C
#define	HDMI_FC_ACP21                           0x107D
#define	HDMI_FC_ACP20                           0x107E
#define	HDMI_FC_ACP19                           0x107F
#define	HDMI_FC_ACP18                           0x1080
#define	HDMI_FC_ACP17                           0x1081
#define	HDMI_FC_ACP16                           0x1082
#define	HDMI_FC_ACP15                           0x1083
#define	HDMI_FC_ACP14                           0x1084
#define	HDMI_FC_ACP13                           0x1085
#define	HDMI_FC_ACP12                           0x1086
#define	HDMI_FC_ACP11                           0x1087
#define	HDMI_FC_ACP10                           0x1088
#define	HDMI_FC_ACP9                            0x1089
#define	HDMI_FC_ACP8                            0x108A
#define	HDMI_FC_ACP7                            0x108B
#define	HDMI_FC_ACP6                            0x108C
#define	HDMI_FC_ACP5                            0x108D
#define	HDMI_FC_ACP4                            0x108E
#define	HDMI_FC_ACP3                            0x108F
#define	HDMI_FC_ACP2                            0x1090
#define	HDMI_FC_ACP1                            0x1091
#define	HDMI_FC_ISCR1_0                         0x1092
#define	HDMI_FC_ISCR1_16                        0x1093
#define	HDMI_FC_ISCR1_15                        0x1094
#define	HDMI_FC_ISCR1_14                        0x1095
#define	HDMI_FC_ISCR1_13                        0x1096
#define	HDMI_FC_ISCR1_12                        0x1097
#define	HDMI_FC_ISCR1_11                        0x1098
#define	HDMI_FC_ISCR1_10                        0x1099
#define	HDMI_FC_ISCR1_9                         0x109A
#define	HDMI_FC_ISCR1_8                         0x109B
#define	HDMI_FC_ISCR1_7                         0x109C
#define	HDMI_FC_ISCR1_6                         0x109D
#define	HDMI_FC_ISCR1_5                         0x109E
#define	HDMI_FC_ISCR1_4                         0x109F
#define	HDMI_FC_ISCR1_3                         0x10A0
#define	HDMI_FC_ISCR1_2                         0x10A1
#define	HDMI_FC_ISCR1_1                         0x10A2
#define	HDMI_FC_ISCR2_15                        0x10A3
#define	HDMI_FC_ISCR2_14                        0x10A4
#define	HDMI_FC_ISCR2_13                        0x10A5
#define	HDMI_FC_ISCR2_12                        0x10A6
#define	HDMI_FC_ISCR2_11                        0x10A7
#define	HDMI_FC_ISCR2_10                        0x10A8
#define	HDMI_FC_ISCR2_9                         0x10A9
#define	HDMI_FC_ISCR2_8                         0x10AA
#define	HDMI_FC_ISCR2_7                         0x10AB
#define	HDMI_FC_ISCR2_6                         0x10AC
#define	HDMI_FC_ISCR2_5                         0x10AD
#define	HDMI_FC_ISCR2_4                         0x10AE
#define	HDMI_FC_ISCR2_3                         0x10AF
#define	HDMI_FC_ISCR2_2                         0x10B0
#define	HDMI_FC_ISCR2_1                         0x10B1
#define	HDMI_FC_ISCR2_0                         0x10B2
#define	HDMI_FC_DATAUTO0                        0x10B3
#define	HDMI_FC_DATAUTO1                        0x10B4
#define	HDMI_FC_DATAUTO2                        0x10B5
#define	HDMI_FC_DATMAN                          0x10B6
#define	HDMI_FC_DATAUTO3                        0x10B7
#define	HDMI_FC_RDRB0                           0x10B8
#define	HDMI_FC_RDRB1                           0x10B9
#define	HDMI_FC_RDRB2                           0x10BA
#define	HDMI_FC_RDRB3                           0x10BB
#define	HDMI_FC_RDRB4                           0x10BC
#define	HDMI_FC_RDRB5                           0x10BD
#define	HDMI_FC_RDRB6                           0x10BE
#define	HDMI_FC_RDRB7                           0x10BF
#define	HDMI_FC_STAT0                           0x10D0
#define	HDMI_FC_INT0                            0x10D1
#define	HDMI_FC_MASK0                           0x10D2
#define	HDMI_FC_POL0                            0x10D3
#define	HDMI_FC_STAT1                           0x10D4
#define	HDMI_FC_INT1                            0x10D5
#define	HDMI_FC_MASK1                           0x10D6
#define	HDMI_FC_POL1                            0x10D7
#define	HDMI_FC_STAT2                           0x10D8
#define	HDMI_FC_INT2                            0x10D9
#define	HDMI_FC_MASK2                           0x10DA
#define	HDMI_FC_POL2                            0x10DB
#define	HDMI_FC_PRCONF                          0x10E0

#define	HDMI_FC_GMD_STAT                        0x1100
#define	HDMI_FC_GMD_EN                          0x1101
#define	HDMI_FC_GMD_UP                          0x1102
#define	HDMI_FC_GMD_CONF                        0x1103
#define	HDMI_FC_GMD_HB                          0x1104
#define	HDMI_FC_GMD_PB0                         0x1105
#define	HDMI_FC_GMD_PB1                         0x1106
#define	HDMI_FC_GMD_PB2                         0x1107
#define	HDMI_FC_GMD_PB3                         0x1108
#define	HDMI_FC_GMD_PB4                         0x1109
#define	HDMI_FC_GMD_PB5                         0x110A
#define	HDMI_FC_GMD_PB6                         0x110B
#define	HDMI_FC_GMD_PB7                         0x110C
#define	HDMI_FC_GMD_PB8                         0x110D
#define	HDMI_FC_GMD_PB9                         0x110E
#define	HDMI_FC_GMD_PB10                        0x110F
#define	HDMI_FC_GMD_PB11                        0x1110
#define	HDMI_FC_GMD_PB12                        0x1111
#define	HDMI_FC_GMD_PB13                        0x1112
#define	HDMI_FC_GMD_PB14                        0x1113
#define	HDMI_FC_GMD_PB15                        0x1114
#define	HDMI_FC_GMD_PB16                        0x1115
#define	HDMI_FC_GMD_PB17                        0x1116
#define	HDMI_FC_GMD_PB18                        0x1117
#define	HDMI_FC_GMD_PB19                        0x1118
#define	HDMI_FC_GMD_PB20                        0x1119
#define	HDMI_FC_GMD_PB21                        0x111A
#define	HDMI_FC_GMD_PB22                        0x111B
#define	HDMI_FC_GMD_PB23                        0x111C
#define	HDMI_FC_GMD_PB24                        0x111D
#define	HDMI_FC_GMD_PB25                        0x111E
#define	HDMI_FC_GMD_PB26                        0x111F
#define	HDMI_FC_GMD_PB27                        0x1120

#define	HDMI_FC_DBGFORCE                        0x1200
#define	HDMI_FC_DBGAUD0CH0                      0x1201
#define	HDMI_FC_DBGAUD1CH0                      0x1202
#define	HDMI_FC_DBGAUD2CH0                      0x1203
#define	HDMI_FC_DBGAUD0CH1                      0x1204
#define	HDMI_FC_DBGAUD1CH1                      0x1205
#define	HDMI_FC_DBGAUD2CH1                      0x1206
#define	HDMI_FC_DBGAUD0CH2                      0x1207
#define	HDMI_FC_DBGAUD1CH2                      0x1208
#define	HDMI_FC_DBGAUD2CH2                      0x1209
#define	HDMI_FC_DBGAUD0CH3                      0x120A
#define	HDMI_FC_DBGAUD1CH3                      0x120B
#define	HDMI_FC_DBGAUD2CH3                      0x120C
#define	HDMI_FC_DBGAUD0CH4                      0x120D
#define	HDMI_FC_DBGAUD1CH4                      0x120E
#define	HDMI_FC_DBGAUD2CH4                      0x120F
#define	HDMI_FC_DBGAUD0CH5                      0x1210
#define	HDMI_FC_DBGAUD1CH5                      0x1211
#define	HDMI_FC_DBGAUD2CH5                      0x1212
#define	HDMI_FC_DBGAUD0CH6                      0x1213
#define	HDMI_FC_DBGAUD1CH6                      0x1214
#define	HDMI_FC_DBGAUD2CH6                      0x1215
#define	HDMI_FC_DBGAUD0CH7                      0x1216
#define	HDMI_FC_DBGAUD1CH7                      0x1217
#define	HDMI_FC_DBGAUD2CH7                      0x1218
#define	HDMI_FC_DBGTMDS0                        0x1219
#define	HDMI_FC_DBGTMDS1                        0x121A
#define	HDMI_FC_DBGTMDS2                        0x121B

#define	HDMI_PHY_CONF0		0x00003000
#define		HDMI_PHY_CONF0_PDZ_MASK			0x80
#define		HDMI_PHY_CONF0_PDZ_OFFSET		7
#define		HDMI_PHY_CONF0_ENTMDS_MASK		0x40
#define		HDMI_PHY_CONF0_ENTMDS_OFFSET		6
#define		HDMI_PHY_CONF0_SPARECTRL		0x20
#define		HDMI_PHY_CONF0_GEN2_PDDQ_MASK		0x10
#define		HDMI_PHY_CONF0_GEN2_PDDQ_OFFSET		4
#define		HDMI_PHY_CONF0_GEN2_TXPWRON_MASK	0x8
#define		HDMI_PHY_CONF0_GEN2_TXPWRON_OFFSET	3
#define		HDMI_PHY_CONF0_GEN2_ENHPDRXSENSE_MASK	0x4
#define		HDMI_PHY_CONF0_GEN2_ENHPDRXSENSE_OFFSET	2
#define		HDMI_PHY_CONF0_SELDATAENPOL_MASK	0x2
#define		HDMI_PHY_CONF0_SELDATAENPOL_OFFSET	1
#define		HDMI_PHY_CONF0_SELDIPIF_MASK		0x1
#define		HDMI_PHY_CONF0_SELDIPIF_OFFSET		0
#define	HDMI_PHY_TST0                           0x3001
#define		HDMI_PHY_TST0_TSTCLR_MASK		0x20
#define		HDMI_PHY_TST0_TSTCLR_OFFSET		5
#define		HDMI_PHY_TST0_TSTEN_MASK		0x10
#define		HDMI_PHY_TST0_TSTEN_OFFSET		4
#define		HDMI_PHY_TST0_TSTCLK_MASK		0x1
#define		HDMI_PHY_TST0_TSTCLK_OFFSET		0
#define	HDMI_PHY_TST1                           0x3002
#define	HDMI_PHY_TST2                           0x3003
#define	HDMI_PHY_STAT0                          0x3004
#define		HDMI_PHY_TX_PHY_LOCK			0x01
#define	HDMI_PHY_INT0                           0x3005
#define	HDMI_PHY_MASK0                          0x3006
#define	HDMI_PHY_POL0                           0x3007
#define		HDMI_PHY_HPD		(1 << 1)

/* HDMI Master PHY Registers */
#define	HDMI_PHY_I2CM_SLAVE_ADDR                0x3020
#define		HDMI_PHY_I2CM_SLAVE_ADDR_PHY_GEN2	0x69
#define		HDMI_PHY_I2CM_SLAVE_ADDR_HEAC_PHY	0x49
#define	HDMI_PHY_I2CM_ADDRESS_ADDR              0x3021
#define	HDMI_PHY_I2CM_DATAO_1_ADDR              0x3022
#define	HDMI_PHY_I2CM_DATAO_0_ADDR              0x3023
#define	HDMI_PHY_I2CM_DATAI_1_ADDR              0x3024
#define	HDMI_PHY_I2CM_DATAI_0_ADDR              0x3025
#define	HDMI_PHY_I2CM_OPERATION_ADDR            0x3026
#define	HDMI_PHY_I2CM_INT_ADDR                  0x3027
#define	HDMI_PHY_I2CM_CTLINT_ADDR               0x3028
#define	HDMI_PHY_I2CM_DIV_ADDR                  0x3029
#define	HDMI_PHY_I2CM_SOFTRSTZ_ADDR             0x302a
#define	HDMI_PHY_I2CM_SS_SCL_HCNT_1_ADDR        0x302b
#define	HDMI_PHY_I2CM_SS_SCL_HCNT_0_ADDR        0x302c
#define	HDMI_PHY_I2CM_SS_SCL_LCNT_1_ADDR        0x302d
#define	HDMI_PHY_I2CM_SS_SCL_LCNT_0_ADDR        0x302e
#define	HDMI_PHY_I2CM_FS_SCL_HCNT_1_ADDR        0x302f
#define	HDMI_PHY_I2CM_FS_SCL_HCNT_0_ADDR        0x3030
#define	HDMI_PHY_I2CM_FS_SCL_LCNT_1_ADDR        0x3031
#define	HDMI_PHY_I2CM_FS_SCL_LCNT_0_ADDR        0x3032

/* Audio Sampler Registers */
#define	HDMI_AUD_CONF0                          0x3100
#define	HDMI_AUD_CONF1                          0x3101
#define	HDMI_AUD_INT                            0x3102
#define	HDMI_AUD_CONF2                          0x3103
#define	HDMI_AUD_N1                             0x3200
#define	HDMI_AUD_N2                             0x3201
#define	HDMI_AUD_N3                             0x3202
#define	HDMI_AUD_CTS1                           0x3203
#define	HDMI_AUD_CTS2                           0x3204
#define	HDMI_AUD_CTS3                           0x3205
#define	HDMI_AUD_INPUTCLKFS                     0x3206
#define	HDMI_AUD_SPDIFINT			0x3302
#define	HDMI_AUD_CONF0_HBR                      0x3400
#define	HDMI_AUD_HBR_STATUS                     0x3401
#define	HDMI_AUD_HBR_INT                        0x3402
#define	HDMI_AUD_HBR_POL                        0x3403
#define	HDMI_AUD_HBR_MASK                       0x3404

/*
 * Generic Parallel Audio Interface Registers
 * Not used as GPAUD interface is not enabled in hw
 */
#define	HDMI_GP_CONF0                           0x3500
#define	HDMI_GP_CONF1                           0x3501
#define	HDMI_GP_CONF2                           0x3502
#define	HDMI_GP_STAT                            0x3503
#define	HDMI_GP_INT                             0x3504
#define	HDMI_GP_MASK                            0x3505
#define	HDMI_GP_POL                             0x3506

/* Main Controller Registers */
#define HDMI_MC_SFRDIV                          0x4000
#define HDMI_MC_CLKDIS                          0x4001
#define		HDMI_MC_CLKDIS_HDCPCLK_DISABLE		(1 << 6)
#define		HDMI_MC_CLKDIS_CECCLK_DISABLE		(1 << 5)
#define		HDMI_MC_CLKDIS_CSCCLK_DISABLE		(1 << 4)
#define		HDMI_MC_CLKDIS_AUDCLK_DISABLE		(1 << 3)
#define		HDMI_MC_CLKDIS_PREPCLK_DISABLE		(1 << 2)
#define		HDMI_MC_CLKDIS_TMDSCLK_DISABLE		(1 << 1)
#define		HDMI_MC_CLKDIS_PIXELCLK_DISABLE		(1 << 0)

#define HDMI_MC_SWRSTZ                          0x4002
#define		HDMI_MC_SWRSTZ_TMDSSWRST_REQ		0x02
#define HDMI_MC_OPCTRL                          0x4003
#define HDMI_MC_FLOWCTRL                        0x4004
#define		HDMI_MC_FLOWCTRL_FEED_THROUGH_OFF_MASK		0x1
#define		HDMI_MC_FLOWCTRL_FEED_THROUGH_OFF_CSC_IN_PATH	0x1
#define		HDMI_MC_FLOWCTRL_FEED_THROUGH_OFF_CSC_BYPASS	0x0
#define HDMI_MC_PHYRSTZ                         0x4005
#define		HDMI_MC_PHYRSTZ_ASSERT			0x0
#define		HDMI_MC_PHYRSTZ_DEASSERT		0x1
#define HDMI_MC_LOCKONCLOCK                     0x4006
#define HDMI_MC_HEACPHY_RST                     0x4007
#define		HDMI_MC_HEACPHY_RST_ASSERT		0x1
#define		HDMI_MC_HEACPHY_RST_DEASSERT		0x0

/* HDCP Encryption Engine Registers */
#define	HDMI_A_HDCPCFG0                         0x5000
#define	HDMI_A_HDCPCFG1                         0x5001
#define	HDMI_A_HDCPOBS0                         0x5002
#define	HDMI_A_HDCPOBS1                         0x5003
#define	HDMI_A_HDCPOBS2                         0x5004
#define	HDMI_A_HDCPOBS3                         0x5005
#define	HDMI_A_APIINTCLR                        0x5006
#define	HDMI_A_APIINTSTAT                       0x5007
#define	HDMI_A_APIINTMSK                        0x5008
#define	HDMI_A_VIDPOLCFG                        0x5009
#define	HDMI_A_OESSWCFG                         0x500A
#define	HDMI_A_TIMER1SETUP0                     0x500B
#define	HDMI_A_TIMER1SETUP1                     0x500C
#define	HDMI_A_TIMER2SETUP0                     0x500D
#define	HDMI_A_TIMER2SETUP1                     0x500E
#define	HDMI_A_100MSCFG                         0x500F
#define	HDMI_A_2SCFG0                           0x5010
#define	HDMI_A_2SCFG1                           0x5011
#define	HDMI_A_5SCFG0                           0x5012
#define	HDMI_A_5SCFG1                           0x5013
#define	HDMI_A_SRMVERLSB                        0x5014
#define	HDMI_A_SRMVERMSB                        0x5015
#define	HDMI_A_SRMCTRL                          0x5016
#define	HDMI_A_SFRSETUP                         0x5017
#define	HDMI_A_I2CHSETUP                        0x5018
#define	HDMI_A_INTSETUP                         0x5019
#define	HDMI_A_PRESETUP                         0x501A
#define	HDMI_A_SRM_BASE                         0x5020

/* CEC Engine Registers */
#define	HDMI_CEC_CTRL                           0x7D00
#define	HDMI_CEC_STAT                           0x7D01
#define	HDMI_CEC_MASK                           0x7D02
#define	HDMI_CEC_POLARITY                       0x7D03
#define	HDMI_CEC_INT                            0x7D04
#define	HDMI_CEC_ADDR_L                         0x7D05
#define	HDMI_CEC_ADDR_H                         0x7D06
#define	HDMI_CEC_TX_CNT                         0x7D07
#define	HDMI_CEC_RX_CNT                         0x7D08
#define	HDMI_CEC_TX_DATA0                       0x7D10
#define	HDMI_CEC_TX_DATA1                       0x7D11
#define	HDMI_CEC_TX_DATA2                       0x7D12
#define	HDMI_CEC_TX_DATA3                       0x7D13
#define	HDMI_CEC_TX_DATA4                       0x7D14
#define	HDMI_CEC_TX_DATA5                       0x7D15
#define	HDMI_CEC_TX_DATA6                       0x7D16
#define	HDMI_CEC_TX_DATA7                       0x7D17
#define	HDMI_CEC_TX_DATA8                       0x7D18
#define	HDMI_CEC_TX_DATA9                       0x7D19
#define	HDMI_CEC_TX_DATA10                      0x7D1a
#define	HDMI_CEC_TX_DATA11                      0x7D1b
#define	HDMI_CEC_TX_DATA12                      0x7D1c
#define	HDMI_CEC_TX_DATA13                      0x7D1d
#define	HDMI_CEC_TX_DATA14                      0x7D1e
#define	HDMI_CEC_TX_DATA15                      0x7D1f
#define	HDMI_CEC_RX_DATA0                       0x7D20
#define	HDMI_CEC_RX_DATA1                       0x7D21
#define	HDMI_CEC_RX_DATA2                       0x7D22
#define	HDMI_CEC_RX_DATA3                       0x7D23
#define	HDMI_CEC_RX_DATA4                       0x7D24
#define	HDMI_CEC_RX_DATA5                       0x7D25
#define	HDMI_CEC_RX_DATA6                       0x7D26
#define	HDMI_CEC_RX_DATA7                       0x7D27
#define	HDMI_CEC_RX_DATA8                       0x7D28
#define	HDMI_CEC_RX_DATA9                       0x7D29
#define	HDMI_CEC_RX_DATA10                      0x7D2a
#define	HDMI_CEC_RX_DATA11                      0x7D2b
#define	HDMI_CEC_RX_DATA12                      0x7D2c
#define	HDMI_CEC_RX_DATA13                      0x7D2d
#define	HDMI_CEC_RX_DATA14                      0x7D2e
#define	HDMI_CEC_RX_DATA15                      0x7D2f
#define	HDMI_CEC_LOCK                           0x7D30
#define	HDMI_CEC_WKUPCTRL                       0x7D31

/* I2C Master Registers (E-DDC) */
#define	HDMI_I2CM_SLAVE                         0x7E00
#define	HDMI_I2CMESS                            0x7E01
#define	HDMI_I2CM_DATAO                         0x7E02
#define	HDMI_I2CM_DATAI                         0x7E03
#define	HDMI_I2CM_OPERATION                     0x7E04
#define		HDMI_PHY_I2CM_OPERATION_ADDR_WRITE	0x10
#define		HDMI_PHY_I2CM_OPERATION_ADDR_READ	0x1
#define	HDMI_I2CM_INT                           0x7E05
#define	HDMI_I2CM_CTLINT                        0x7E06
#define	HDMI_I2CM_DIV                           0x7E07
#define	HDMI_I2CM_SEGADDR                       0x7E08
#define	HDMI_I2CM_SOFTRSTZ                      0x7E09
#define	HDMI_I2CM_SEGPTR                        0x7E0A
#define	HDMI_I2CM_SS_SCL_HCNT_1_ADDR            0x7E0B
#define	HDMI_I2CM_SS_SCL_HCNT_0_ADDR            0x7E0C
#define	HDMI_I2CM_SS_SCL_LCNT_1_ADDR            0x7E0D
#define	HDMI_I2CM_SS_SCL_LCNT_0_ADDR            0x7E0E
#define	HDMI_I2CM_FS_SCL_HCNT_1_ADDR            0x7E0F
#define	HDMI_I2CM_FS_SCL_HCNT_0_ADDR            0x7E10
#define	HDMI_I2CM_FS_SCL_LCNT_1_ADDR            0x7E11
#define	HDMI_I2CM_FS_SCL_LCNT_0_ADDR            0x7E12

#endif	/* __IMX6__HDMI_REGS_H__ */


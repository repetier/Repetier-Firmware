/*
 Ardiono pin mapping is very logical: ArduinoPin = port * 16 + portpin
 With port = 0 for port A until 4 for port E, portpin is 0..15
Symbolic names PA0 - PE15 are defined already in variant.h!

*/

#include "PinNames.h"
#include "PortNames.h"
#include "stm32yyxx_ll_gpio.h"

#ifndef _FASTIO_H
#define _FASTIO_H

// Port A
#define GPIO_0 (GPIO_TypeDef*)PA_0
#define GPIO_1 (GPIO_TypeDef*)PA_1
#define GPIO_2 (GPIO_TypeDef*)PA_2
#define GPIO_3 (GPIO_TypeDef*)PA_3
#define GPIO_4 (GPIO_TypeDef*)PA_4
#define GPIO_5 (GPIO_TypeDef*)PA_5
#define GPIO_6 (GPIO_TypeDef*)PA_6
#define GPIO_7 (GPIO_TypeDef*)PA_7
#define GPIO_8 (GPIO_TypeDef*)PA_8
#define GPIO_9 (GPIO_TypeDef*)PA_9
#define GPIO_10 (GPIO_TypeDef*)PA_10
#define GPIO_11 (GPIO_TypeDef*)PA_11
#define GPIO_12 (GPIO_TypeDef*)PA_12
#define GPIO_13 (GPIO_TypeDef*)PA_13
#define GPIO_14 (GPIO_TypeDef*)PA_14
#define GPIO_15 (GPIO_TypeDef*)PA_15

#define GPIO_0_MASK LL_GPIO_PIN_0
#define GPIO_1_MASK LL_GPIO_PIN_1
#define GPIO_2_MASK LL_GPIO_PIN_2
#define GPIO_3_MASK LL_GPIO_PIN_3
#define GPIO_4_MASK LL_GPIO_PIN_4
#define GPIO_5_MASK LL_GPIO_PIN_5
#define GPIO_6_MASK LL_GPIO_PIN_6
#define GPIO_7_MASK LL_GPIO_PIN_7
#define GPIO_8_MASK LL_GPIO_PIN_8
#define GPIO_9_MASK LL_GPIO_PIN_9
#define GPIO_10_MASK LL_GPIO_PIN_10
#define GPIO_11_MASK LL_GPIO_PIN_11
#define GPIO_12_MASK LL_GPIO_PIN_12
#define GPIO_13_MASK LL_GPIO_PIN_13
#define GPIO_14_MASK LL_GPIO_PIN_14
#define GPIO_15_MASK LL_GPIO_PIN_15

// Port B
#define GPIO_16 (GPIO_TypeDef*)PB_0
#define GPIO_17 (GPIO_TypeDef*)PB_1
#define GPIO_18 (GPIO_TypeDef*)PB_2
#define GPIO_19 (GPIO_TypeDef*)PB_3
#define GPIO_20 (GPIO_TypeDef*)PB_4
#define GPIO_21 (GPIO_TypeDef*)PB_5
#define GPIO_22 (GPIO_TypeDef*)PB_6
#define GPIO_23 (GPIO_TypeDef*)PB_7
#define GPIO_24 (GPIO_TypeDef*)PB_8
#define GPIO_25 (GPIO_TypeDef*)PB_9
#define GPIO_26 (GPIO_TypeDef*)PB_10
#define GPIO_27 (GPIO_TypeDef*)PB_11
#define GPIO_28 (GPIO_TypeDef*)PB_12
#define GPIO_29 (GPIO_TypeDef*)PB_13
#define GPIO_30 (GPIO_TypeDef*)PB_14
#define GPIO_31 (GPIO_TypeDef*)PB_15

#define GPIO_16_MASK LL_GPIO_PIN_0
#define GPIO_17_MASK LL_GPIO_PIN_1
#define GPIO_18_MASK LL_GPIO_PIN_2
#define GPIO_19_MASK LL_GPIO_PIN_3
#define GPIO_20_MASK LL_GPIO_PIN_4
#define GPIO_21_MASK LL_GPIO_PIN_5
#define GPIO_22_MASK LL_GPIO_PIN_6
#define GPIO_23_MASK LL_GPIO_PIN_7
#define GPIO_24_MASK LL_GPIO_PIN_8
#define GPIO_25_MASK LL_GPIO_PIN_9
#define GPIO_26_MASK LL_GPIO_PIN_10
#define GPIO_27_MASK LL_GPIO_PIN_11
#define GPIO_28_MASK LL_GPIO_PIN_12
#define GPIO_29_MASK LL_GPIO_PIN_13
#define GPIO_30_MASK LL_GPIO_PIN_14
#define GPIO_31_MASK LL_GPIO_PIN_15

// Port C
#if defined GPIOC_BASE
#define GPIO_32 (GPIO_TypeDef*)PC_0
#define GPIO_33 (GPIO_TypeDef*)PC_1
#define GPIO_34 (GPIO_TypeDef*)PC_2
#define GPIO_35 (GPIO_TypeDef*)PC_3
#define GPIO_36 (GPIO_TypeDef*)PC_4
#define GPIO_37 (GPIO_TypeDef*)PC_5
#define GPIO_38 (GPIO_TypeDef*)PC_6
#define GPIO_39 (GPIO_TypeDef*)PC_7
#define GPIO_40 (GPIO_TypeDef*)PC_8
#define GPIO_41 (GPIO_TypeDef*)PC_9
#define GPIO_42 (GPIO_TypeDef*)PC_10
#define GPIO_43 (GPIO_TypeDef*)PC_11
#define GPIO_44 (GPIO_TypeDef*)PC_12
#define GPIO_45 (GPIO_TypeDef*)PC_13
#define GPIO_46 (GPIO_TypeDef*)PC_14
#define GPIO_47 (GPIO_TypeDef*)PC_15

#define GPIO_32_MASK LL_GPIO_PIN_0
#define GPIO_33_MASK LL_GPIO_PIN_1
#define GPIO_34_MASK LL_GPIO_PIN_2
#define GPIO_35_MASK LL_GPIO_PIN_3
#define GPIO_36_MASK LL_GPIO_PIN_4
#define GPIO_37_MASK LL_GPIO_PIN_5
#define GPIO_38_MASK LL_GPIO_PIN_6
#define GPIO_39_MASK LL_GPIO_PIN_7
#define GPIO_40_MASK LL_GPIO_PIN_8
#define GPIO_41_MASK LL_GPIO_PIN_9
#define GPIO_42_MASK LL_GPIO_PIN_10
#define GPIO_43_MASK LL_GPIO_PIN_11
#define GPIO_44_MASK LL_GPIO_PIN_12
#define GPIO_45_MASK LL_GPIO_PIN_13
#define GPIO_46_MASK LL_GPIO_PIN_14
#define GPIO_47_MASK LL_GPIO_PIN_15
#endif

// Port D
#if defined GPIOD_BASE
#define GPIO_48 (GPIO_TypeDef*)PD_0
#define GPIO_49 (GPIO_TypeDef*)PD_1
#define GPIO_50 (GPIO_TypeDef*)PD_2
#define GPIO_51 (GPIO_TypeDef*)PD_3
#define GPIO_52 (GPIO_TypeDef*)PD_4
#define GPIO_53 (GPIO_TypeDef*)PD_5
#define GPIO_54 (GPIO_TypeDef*)PD_6
#define GPIO_55 (GPIO_TypeDef*)PD_7
#define GPIO_56 (GPIO_TypeDef*)PD_8
#define GPIO_57 (GPIO_TypeDef*)PD_9
#define GPIO_58 (GPIO_TypeDef*)PD_10
#define GPIO_59 (GPIO_TypeDef*)PD_11
#define GPIO_60 (GPIO_TypeDef*)PD_12
#define GPIO_61 (GPIO_TypeDef*)PD_13
#define GPIO_62 (GPIO_TypeDef*)PD_14
#define GPIO_63 (GPIO_TypeDef*)PD_15

#define GPIO_48_MASK LL_GPIO_PIN_0
#define GPIO_49_MASK LL_GPIO_PIN_1
#define GPIO_50_MASK LL_GPIO_PIN_2
#define GPIO_51_MASK LL_GPIO_PIN_3
#define GPIO_52_MASK LL_GPIO_PIN_4
#define GPIO_53_MASK LL_GPIO_PIN_5
#define GPIO_54_MASK LL_GPIO_PIN_6
#define GPIO_55_MASK LL_GPIO_PIN_7
#define GPIO_56_MASK LL_GPIO_PIN_8
#define GPIO_57_MASK LL_GPIO_PIN_9
#define GPIO_58_MASK LL_GPIO_PIN_10
#define GPIO_59_MASK LL_GPIO_PIN_11
#define GPIO_60_MASK LL_GPIO_PIN_12
#define GPIO_61_MASK LL_GPIO_PIN_13
#define GPIO_62_MASK LL_GPIO_PIN_14
#define GPIO_63_MASK LL_GPIO_PIN_15
#endif

// Port E
#if defined GPIOC_BASE
#define GPIO_64 (GPIO_TypeDef*)PE_0
#define GPIO_65 (GPIO_TypeDef*)PE_1
#define GPIO_66 (GPIO_TypeDef*)PE_2
#define GPIO_67 (GPIO_TypeDef*)PE_3
#define GPIO_68 (GPIO_TypeDef*)PE_4
#define GPIO_69 (GPIO_TypeDef*)PE_5
#define GPIO_70 (GPIO_TypeDef*)PE_6
#define GPIO_71 (GPIO_TypeDef*)PE_7
#define GPIO_72 (GPIO_TypeDef*)PE_8
#define GPIO_73 (GPIO_TypeDef*)PE_9
#define GPIO_74 (GPIO_TypeDef*)PE_10
#define GPIO_75 (GPIO_TypeDef*)PE_11
#define GPIO_76 (GPIO_TypeDef*)PE_12
#define GPIO_77 (GPIO_TypeDef*)PE_13
#define GPIO_78 (GPIO_TypeDef*)PE_14
#define GPIO_79 (GPIO_TypeDef*)PE_15

#define GPIO_64_MASK LL_GPIO_PIN_0
#define GPIO_65_MASK LL_GPIO_PIN_1
#define GPIO_66_MASK LL_GPIO_PIN_2
#define GPIO_67_MASK LL_GPIO_PIN_3
#define GPIO_68_MASK LL_GPIO_PIN_4
#define GPIO_69_MASK LL_GPIO_PIN_5
#define GPIO_70_MASK LL_GPIO_PIN_6
#define GPIO_71_MASK LL_GPIO_PIN_7
#define GPIO_72_MASK LL_GPIO_PIN_8
#define GPIO_73_MASK LL_GPIO_PIN_9
#define GPIO_74_MASK LL_GPIO_PIN_10
#define GPIO_75_MASK LL_GPIO_PIN_11
#define GPIO_76_MASK LL_GPIO_PIN_12
#define GPIO_77_MASK LL_GPIO_PIN_13
#define GPIO_78_MASK LL_GPIO_PIN_14
#define GPIO_79_MASK LL_GPIO_PIN_15
#endif

// Port F
#if defined GPIOF_BASE
#define GPIO_80 (GPIO_TypeDef*)PF_0
#define GPIO_81 (GPIO_TypeDef*)PF_1
#define GPIO_82 (GPIO_TypeDef*)PF_2
#define GPIO_83 (GPIO_TypeDef*)PF_3
#define GPIO_84 (GPIO_TypeDef*)PF_4
#define GPIO_85 (GPIO_TypeDef*)PF_5
#define GPIO_86 (GPIO_TypeDef*)PF_6
#define GPIO_87 (GPIO_TypeDef*)PF_7
#define GPIO_88 (GPIO_TypeDef*)PF_8
#define GPIO_89 (GPIO_TypeDef*)PF_9
#define GPIO_90 (GPIO_TypeDef*)PF_10
#define GPIO_91 (GPIO_TypeDef*)PF_11
#define GPIO_92 (GPIO_TypeDef*)PF_12
#define GPIO_93 (GPIO_TypeDef*)PF_13
#define GPIO_94 (GPIO_TypeDef*)PF_14
#define GPIO_95 (GPIO_TypeDef*)PF_15

#define GPIO_80_MASK LL_GPIO_PIN_0
#define GPIO_81_MASK LL_GPIO_PIN_1
#define GPIO_82_MASK LL_GPIO_PIN_2
#define GPIO_83_MASK LL_GPIO_PIN_3
#define GPIO_84_MASK LL_GPIO_PIN_4
#define GPIO_85_MASK LL_GPIO_PIN_5
#define GPIO_86_MASK LL_GPIO_PIN_6
#define GPIO_87_MASK LL_GPIO_PIN_7
#define GPIO_88_MASK LL_GPIO_PIN_8
#define GPIO_89_MASK LL_GPIO_PIN_9
#define GPIO_90_MASK LL_GPIO_PIN_10
#define GPIO_91_MASK LL_GPIO_PIN_11
#define GPIO_92_MASK LL_GPIO_PIN_12
#define GPIO_93_MASK LL_GPIO_PIN_13
#define GPIO_94_MASK LL_GPIO_PIN_14
#define GPIO_95_MASK LL_GPIO_PIN_15
#endif

// Port G
#if defined GPIOG_BASE
#define GPIO_96 (GPIO_TypeDef*)PG_0
#define GPIO_97 (GPIO_TypeDef*)PG_1
#define GPIO_98 (GPIO_TypeDef*)PG_2
#define GPIO_99 (GPIO_TypeDef*)PG_3
#define GPIO_100 (GPIO_TypeDef*)PG_4
#define GPIO_101 (GPIO_TypeDef*)PG_5
#define GPIO_102 (GPIO_TypeDef*)PG_6
#define GPIO_103 (GPIO_TypeDef*)PG_7
#define GPIO_104 (GPIO_TypeDef*)PG_8
#define GPIO_105 (GPIO_TypeDef*)PG_9
#define GPIO_106 (GPIO_TypeDef*)PG_10
#define GPIO_107 (GPIO_TypeDef*)PG_11
#define GPIO_108 (GPIO_TypeDef*)PG_12
#define GPIO_109 (GPIO_TypeDef*)PG_13
#define GPIO_110 (GPIO_TypeDef*)PG_14
#define GPIO_111 (GPIO_TypeDef*)PG_15

#define GPIO_96_MASK LL_GPIO_PIN_0
#define GPIO_97_MASK LL_GPIO_PIN_1
#define GPIO_98_MASK LL_GPIO_PIN_2
#define GPIO_99_MASK LL_GPIO_PIN_3
#define GPIO_100_MASK LL_GPIO_PIN_4
#define GPIO_101_MASK LL_GPIO_PIN_5
#define GPIO_102_MASK LL_GPIO_PIN_6
#define GPIO_103_MASK LL_GPIO_PIN_7
#define GPIO_104_MASK LL_GPIO_PIN_8
#define GPIO_105_MASK LL_GPIO_PIN_9
#define GPIO_106_MASK LL_GPIO_PIN_10
#define GPIO_107_MASK LL_GPIO_PIN_11
#define GPIO_108_MASK LL_GPIO_PIN_12
#define GPIO_109_MASK LL_GPIO_PIN_13
#define GPIO_110_MASK LL_GPIO_PIN_14
#define GPIO_111_MASK LL_GPIO_PIN_15
#endif

// Port H
#if defined GPIOC_BASE
#define GPIO_112 (GPIO_TypeDef*)PH_0
#define GPIO_113 (GPIO_TypeDef*)PH_1
#define GPIO_114 (GPIO_TypeDef*)PH_2
#define GPIO_115 (GPIO_TypeDef*)PH_3
#define GPIO_116 (GPIO_TypeDef*)PH_4
#define GPIO_117 (GPIO_TypeDef*)PH_5
#define GPIO_118 (GPIO_TypeDef*)PH_6
#define GPIO_119 (GPIO_TypeDef*)PH_7
#define GPIO_120 (GPIO_TypeDef*)PH_8
#define GPIO_121 (GPIO_TypeDef*)PH_9
#define GPIO_122 (GPIO_TypeDef*)PH_10
#define GPIO_123 (GPIO_TypeDef*)PH_11
#define GPIO_124 (GPIO_TypeDef*)PH_12
#define GPIO_125 (GPIO_TypeDef*)PH_13
#define GPIO_126 (GPIO_TypeDef*)PH_14
#define GPIO_127 (GPIO_TypeDef*)PH_15

#define GPIO_112_MASK LL_GPIO_PIN_0
#define GPIO_113_MASK LL_GPIO_PIN_1
#define GPIO_114_MASK LL_GPIO_PIN_2
#define GPIO_115_MASK LL_GPIO_PIN_3
#define GPIO_116_MASK LL_GPIO_PIN_4
#define GPIO_117_MASK LL_GPIO_PIN_5
#define GPIO_118_MASK LL_GPIO_PIN_6
#define GPIO_119_MASK LL_GPIO_PIN_7
#define GPIO_120_MASK LL_GPIO_PIN_8
#define GPIO_121_MASK LL_GPIO_PIN_9
#define GPIO_122_MASK LL_GPIO_PIN_10
#define GPIO_123_MASK LL_GPIO_PIN_11
#define GPIO_124_MASK LL_GPIO_PIN_12
#define GPIO_125_MASK LL_GPIO_PIN_13
#define GPIO_126_MASK LL_GPIO_PIN_14
#define GPIO_127_MASK LL_GPIO_PIN_15
#endif

// Port I
#if defined GPIOC_BASE
#define GPIO_128 (GPIO_TypeDef*)PI_0
#define GPIO_129 (GPIO_TypeDef*)PI_1
#define GPIO_130 (GPIO_TypeDef*)PI_2
#define GPIO_131 (GPIO_TypeDef*)PI_3
#define GPIO_132 (GPIO_TypeDef*)PI_4
#define GPIO_133 (GPIO_TypeDef*)PI_5
#define GPIO_134 (GPIO_TypeDef*)PI_6
#define GPIO_135 (GPIO_TypeDef*)PI_7
#define GPIO_136 (GPIO_TypeDef*)PI_8
#define GPIO_137 (GPIO_TypeDef*)PI_9
#define GPIO_138 (GPIO_TypeDef*)PI_10
#define GPIO_139 (GPIO_TypeDef*)PI_11
#define GPIO_140 (GPIO_TypeDef*)PI_12
#define GPIO_141 (GPIO_TypeDef*)PI_13
#define GPIO_142 (GPIO_TypeDef*)PI_14
#define GPIO_143 (GPIO_TypeDef*)PI_15

#define GPIO_128_MASK LL_GPIO_PIN_0
#define GPIO_129_MASK LL_GPIO_PIN_1
#define GPIO_130_MASK LL_GPIO_PIN_2
#define GPIO_131_MASK LL_GPIO_PIN_3
#define GPIO_132_MASK LL_GPIO_PIN_4
#define GPIO_133_MASK LL_GPIO_PIN_5
#define GPIO_134_MASK LL_GPIO_PIN_6
#define GPIO_135_MASK LL_GPIO_PIN_7
#define GPIO_136_MASK LL_GPIO_PIN_8
#define GPIO_137_MASK LL_GPIO_PIN_9
#define GPIO_138_MASK LL_GPIO_PIN_10
#define GPIO_139_MASK LL_GPIO_PIN_11
#define GPIO_140_MASK LL_GPIO_PIN_12
#define GPIO_141_MASK LL_GPIO_PIN_13
#define GPIO_142_MASK LL_GPIO_PIN_14
#define GPIO_143_MASK LL_GPIO_PIN_15
#endif

// Port J
#if defined GPIOJ_BASE
#define GPIO_144 (GPIO_TypeDef*)PJ_0
#define GPIO_145 (GPIO_TypeDef*)PJ_1
#define GPIO_146 (GPIO_TypeDef*)PJ_2
#define GPIO_147 (GPIO_TypeDef*)PJ_3
#define GPIO_148 (GPIO_TypeDef*)PJ_4
#define GPIO_149 (GPIO_TypeDef*)PJ_5
#define GPIO_150 (GPIO_TypeDef*)PJ_6
#define GPIO_151 (GPIO_TypeDef*)PJ_7
#define GPIO_152 (GPIO_TypeDef*)PJ_8
#define GPIO_153 (GPIO_TypeDef*)PJ_9
#define GPIO_154 (GPIO_TypeDef*)PJ_10
#define GPIO_155 (GPIO_TypeDef*)PJ_11
#define GPIO_156 (GPIO_TypeDef*)PJ_12
#define GPIO_157 (GPIO_TypeDef*)PJ_13
#define GPIO_158 (GPIO_TypeDef*)PJ_14
#define GPIO_159 (GPIO_TypeDef*)PJ_15

#define GPIO_144_MASK LL_GPIO_PIN_0
#define GPIO_145_MASK LL_GPIO_PIN_1
#define GPIO_146_MASK LL_GPIO_PIN_2
#define GPIO_147_MASK LL_GPIO_PIN_3
#define GPIO_148_MASK LL_GPIO_PIN_4
#define GPIO_149_MASK LL_GPIO_PIN_5
#define GPIO_150_MASK LL_GPIO_PIN_6
#define GPIO_151_MASK LL_GPIO_PIN_7
#define GPIO_152_MASK LL_GPIO_PIN_8
#define GPIO_153_MASK LL_GPIO_PIN_9
#define GPIO_154_MASK LL_GPIO_PIN_10
#define GPIO_155_MASK LL_GPIO_PIN_11
#define GPIO_156_MASK LL_GPIO_PIN_12
#define GPIO_157_MASK LL_GPIO_PIN_13
#define GPIO_158_MASK LL_GPIO_PIN_14
#define GPIO_159_MASK LL_GPIO_PIN_15
#endif

// Port K
#if defined GPIOK_BASE
#define GPIO_160 (GPIO_TypeDef*)PK_0
#define GPIO_161 (GPIO_TypeDef*)PK_1
#define GPIO_162 (GPIO_TypeDef*)PK_2
#define GPIO_163 (GPIO_TypeDef*)PK_3
#define GPIO_164 (GPIO_TypeDef*)PK_4
#define GPIO_165 (GPIO_TypeDef*)PK_5
#define GPIO_166 (GPIO_TypeDef*)PK_6
#define GPIO_167 (GPIO_TypeDef*)PK_7
#define GPIO_168 (GPIO_TypeDef*)PK_8
#define GPIO_169 (GPIO_TypeDef*)PK_9
#define GPIO_170 (GPIO_TypeDef*)PK_10
#define GPIO_171 (GPIO_TypeDef*)PK_11
#define GPIO_172 (GPIO_TypeDef*)PK_12
#define GPIO_173 (GPIO_TypeDef*)PK_13
#define GPIO_174 (GPIO_TypeDef*)PK_14
#define GPIO_175 (GPIO_TypeDef*)PK_15

#define GPIO_160_MASK LL_GPIO_PIN_0
#define GPIO_161_MASK LL_GPIO_PIN_1
#define GPIO_162_MASK LL_GPIO_PIN_2
#define GPIO_163_MASK LL_GPIO_PIN_3
#define GPIO_164_MASK LL_GPIO_PIN_4
#define GPIO_165_MASK LL_GPIO_PIN_5
#define GPIO_166_MASK LL_GPIO_PIN_6
#define GPIO_167_MASK LL_GPIO_PIN_7
#define GPIO_168_MASK LL_GPIO_PIN_8
#define GPIO_169_MASK LL_GPIO_PIN_9
#define GPIO_170_MASK LL_GPIO_PIN_10
#define GPIO_171_MASK LL_GPIO_PIN_11
#define GPIO_172_MASK LL_GPIO_PIN_12
#define GPIO_173_MASK LL_GPIO_PIN_13
#define GPIO_174_MASK LL_GPIO_PIN_14
#define GPIO_175_MASK LL_GPIO_PIN_15
#endif

#endif /* _FASTIO_H */

/* Copyright (c) 2010 - 2020, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @defgroup FSM Finite State Machine API
 * @ingroup MESH_CORE
 *
 * FSM assistant. It is optional functionality to simplify FSM description.
 *
 * @{
 */

#define VA_NARGS(...) VA_NARGS_EVAL(__VA_ARGS__)
#define VA_NARGS_EVAL(...) VA_NARGS_IMPL(__VA_ARGS__, \
        /* 255, 254, */ 253, 252, 251, 250, 249, 248, 247, 246, 245, 244, 243, 242, 241, 240, \
        239, 238, 237, 236, 235, 234, 233, 232, 231, 230, 229, 228, 227, 226, 225, 224, \
        223, 222, 221, 220, 219, 218, 217, 216, 215, 214, 213, 212, 211, 210, 209, 208, \
        207, 206, 205, 204, 203, 202, 201, 200, 199, 198, 197, 196, 195, 194, 193, 192, \
        191, 190, 189, 188, 187, 186, 185, 184, 183, 182, 181, 180, 179, 178, 177, 176, \
        175, 174, 173, 172, 171, 170, 169, 168, 167, 166, 165, 164, 163, 162, 161, 160, \
        159, 158, 157, 156, 155, 154, 153, 152, 151, 150, 149, 148, 147, 146, 145, 144, \
        143, 142, 141, 140, 139, 138, 137, 136, 135, 134, 133, 132, 131, 130, 129, 128, \
        127, 126, 125, 124, 123, 122, 121, 120, 119, 118, 117, 116, 115, 114, 113, 112, \
        111, 110, 109, 108, 107, 106, 105, 104, 103, 102, 101, 100,  99,  98,  97,  96, \
         95,  94,  93,  92,  91,  90,  89,  88,  87,  86,  85,  84,  83,  82,  81,  80, \
         79,  78,  77,  76,  75,  74,  73,  72,  71,  70,  69,  68,  67,  66,  65,  64, \
         63,  62,  61,  60,  59,  58,  57,  56,  55,  54,  53,  52,  51,  50,  49,  48, \
         47,  46,  45,  44,  43,  42,  41,  40,  39,  38,  37,  36,  35,  34,  33,  32, \
         31,  30,  29,  28,  27,  26,  25,  24,  23,  22,  21,  20,  19,  18,  17,  16, \
         15,  14,  13,  12,  11,  10,   9,   8,   7,   6,   5,   4,   3,   2,   1,   0)

#define VA_NARGS_IMPL(_________1,   _2,   _3,   _4,   _5,   _6,   _7, \
                        _8,   _9,  _10,  _11,  _12,  _13,  _14,  _15, \
                      __16,  _17,  _18,  _19,  _20,  _21,  _22,  _23, \
                       _24,  _25,  _26,  _27,  _28,  _29,  _30,  _31, \
                      __32,  _33,  _34,  _35,  _36,  _37,  _38,  _39, \
                       _40,  _41,  _42,  _43,  _44,  _45,  _46,  _47, \
                      __48,  _49,  _50,  _51,  _52,  _53,  _54,  _55, \
                       _56,  _57,  _58,  _59,  _60,  _61,  _62,  _63, \
                      __64,  _65,  _66,  _67,  _68,  _69,  _70,  _71, \
                       _72,  _73,  _74,  _75,  _76,  _77,  _78,  _79, \
                      __80,  _81,  _82,  _83,  _84,  _85,  _86,  _87, \
                       _88,  _89,  _90,  _91,  _92,  _93,  _94,  _95, \
                      __96,  _97,  _98,  _99, _100, _101, _102, _103, \
                      _104, _105, _106, _107, _108, _109, _110, _111, \
                      _112, _113, _114, _115, _116, _117, _118, _119, \
                      _120, _121, _122, _123, _124, _125, _126, _127, \
                      _128, _129, _130, _131, _132, _133, _134, _135, \
                      _136, _137, _138, _139, _140, _141, _142, _143, \
                      _144, _145, _146, _147, _148, _149, _150, _151, \
                      _152, _153, _154, _155, _156, _157, _158, _159, \
                      _160, _161, _162, _163, _164, _165, _166, _167, \
                      _168, _169, _170, _171, _172, _173, _174, _175, \
                      _176, _177, _178, _179, _180, _181, _182, _183, \
                      _184, _185, _186, _187, _188, _189, _190, _191, \
                      _192, _193, _194, _195, _196, _197, _198, _199, \
                      _200, _201, _202, _203, _204, _205, _206, _207, \
                      _208, _209, _210, _211, _212, _213, _214, _215, \
                      _216, _217, _218, _219, _220, _221, _222, _223, \
                      _224, _225, _226, _227, _228, _229, _230, _231, \
                      _232, _233, _234, _235, _236, _237, _238, _239, \
                      _240, _241, _242, _243, _244, _245, _246, _247, \
                      _248, _249, _250, _251, _252, _253, /* _254, _255, */\
                      N, ...) N

#define MACRO_FOR_EACH_0(ACT, ARG, ...)
#define MACRO_FOR_EACH_1(ACT, ARG, ...)   ACT(ARG)
#define MACRO_FOR_EACH_2(ACT, ARG, ...)   ACT(ARG)  MACRO_FOR_EACH_1(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_3(ACT, ARG, ...)   ACT(ARG)  MACRO_FOR_EACH_2(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_4(ACT, ARG, ...)   ACT(ARG)  MACRO_FOR_EACH_3(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_5(ACT, ARG, ...)   ACT(ARG)  MACRO_FOR_EACH_4(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_6(ACT, ARG, ...)   ACT(ARG)  MACRO_FOR_EACH_5(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_7(ACT, ARG, ...)   ACT(ARG)  MACRO_FOR_EACH_6(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_8(ACT, ARG, ...)   ACT(ARG)  MACRO_FOR_EACH_7(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_9(ACT, ARG, ...)   ACT(ARG)  MACRO_FOR_EACH_8(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_10(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_9(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_11(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_10(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_12(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_11(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_13(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_12(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_14(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_13(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_15(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_14(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_16(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_15(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_17(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_16(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_18(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_17(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_19(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_18(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_20(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_19(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_21(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_20(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_22(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_21(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_23(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_22(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_24(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_23(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_25(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_24(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_26(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_25(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_27(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_26(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_28(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_27(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_29(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_28(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_30(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_29(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_31(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_30(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_32(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_31(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_33(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_32(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_34(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_33(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_35(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_34(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_36(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_35(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_37(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_36(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_38(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_37(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_39(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_38(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_40(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_39(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_41(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_40(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_42(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_41(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_43(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_42(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_44(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_43(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_45(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_44(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_46(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_45(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_47(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_46(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_48(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_47(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_49(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_48(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_50(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_49(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_51(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_50(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_52(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_51(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_53(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_52(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_54(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_53(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_55(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_54(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_56(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_55(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_57(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_56(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_58(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_57(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_59(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_58(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_60(ACT, ARG, ...)  ACT(ARG)  MACRO_FOR_EACH_59(ACT, __VA_ARGS__)

#define MACRO_FOR_EACH__(N, ACT, ...)      MACRO_FOR_EACH_##N(ACT, __VA_ARGS__)
#define MACRO_FOR_EACH_(N, ACT, ...)       MACRO_FOR_EACH__(N, ACT, __VA_ARGS__)

/** Performs @a ACT(), which takes an argument from @a __VA_ARGS__, for each argument.
 *  Supports up to @a N arguments, where @a N is @a N in maximum defined @a MACRO_FOR_EACH_N.
 */
#define MACRO_FOR_EACH(ACT, ...)           MACRO_FOR_EACH_(VA_NARGS(__VA_ARGS__), ACT, __VA_ARGS__)

#define MACRO_FOR_PAIR_0(ACT, ...)
#define MACRO_FOR_PAIR_1(ACT, ...)
#define MACRO_FOR_PAIR_2(ACT, ARG1, ARG2, ...)    ACT(ARG1, ARG2)
#define MACRO_FOR_PAIR_4(ACT, ARG1, ARG2, ...)    ACT(ARG1, ARG2)   MACRO_FOR_PAIR_2(ACT,  __VA_ARGS__)
#define MACRO_FOR_PAIR_6(ACT, ARG1, ARG2, ...)    ACT(ARG1, ARG2)   MACRO_FOR_PAIR_4(ACT,  __VA_ARGS__)
#define MACRO_FOR_PAIR_8(ACT, ARG1, ARG2, ...)    ACT(ARG1, ARG2)   MACRO_FOR_PAIR_6(ACT,  __VA_ARGS__)
#define MACRO_FOR_PAIR_10(ACT, ARG1, ARG2, ...)   ACT(ARG1, ARG2)   MACRO_FOR_PAIR_8(ACT,  __VA_ARGS__)
#define MACRO_FOR_PAIR_12(ACT, ARG1, ARG2, ...)   ACT(ARG1, ARG2)   MACRO_FOR_PAIR_10(ACT,  __VA_ARGS__)
#define MACRO_FOR_PAIR_14(ACT, ARG1, ARG2, ...)   ACT(ARG1, ARG2)   MACRO_FOR_PAIR_12(ACT,  __VA_ARGS__)
#define MACRO_FOR_PAIR_16(ACT, ARG1, ARG2, ...)   ACT(ARG1, ARG2)   MACRO_FOR_PAIR_14(ACT,  __VA_ARGS__)
#define MACRO_FOR_PAIR_18(ACT, ARG1, ARG2, ...)   ACT(ARG1, ARG2)   MACRO_FOR_PAIR_16(ACT,  __VA_ARGS__)
#define MACRO_FOR_PAIR_20(ACT, ARG1, ARG2, ...)   ACT(ARG1, ARG2)   MACRO_FOR_PAIR_18(ACT,  __VA_ARGS__)
#define MACRO_FOR_PAIR_22(ACT, ARG1, ARG2, ...)   ACT(ARG1, ARG2)   MACRO_FOR_PAIR_20(ACT,  __VA_ARGS__)
#define MACRO_FOR_PAIR_24(ACT, ARG1, ARG2, ...)   ACT(ARG1, ARG2)   MACRO_FOR_PAIR_22(ACT,  __VA_ARGS__)
#define MACRO_FOR_PAIR_26(ACT, ARG1, ARG2, ...)   ACT(ARG1, ARG2)   MACRO_FOR_PAIR_24(ACT,  __VA_ARGS__)
#define MACRO_FOR_PAIR_28(ACT, ARG1, ARG2, ...)   ACT(ARG1, ARG2)   MACRO_FOR_PAIR_26(ACT,  __VA_ARGS__)
#define MACRO_FOR_PAIR_30(ACT, ARG1, ARG2, ...)   ACT(ARG1, ARG2)   MACRO_FOR_PAIR_28(ACT,  __VA_ARGS__)
#define MACRO_FOR_PAIR_32(ACT, ARG1, ARG2, ...)   ACT(ARG1, ARG2)   MACRO_FOR_PAIR_30(ACT,  __VA_ARGS__)
#define MACRO_FOR_PAIR_34(ACT, ARG1, ARG2, ...)   ACT(ARG1, ARG2)   MACRO_FOR_PAIR_32(ACT,  __VA_ARGS__)
#define MACRO_FOR_PAIR_36(ACT, ARG1, ARG2, ...)   ACT(ARG1, ARG2)   MACRO_FOR_PAIR_34(ACT,  __VA_ARGS__)
#define MACRO_FOR_PAIR_38(ACT, ARG1, ARG2, ...)   ACT(ARG1, ARG2)   MACRO_FOR_PAIR_36(ACT,  __VA_ARGS__)
#define MACRO_FOR_PAIR_40(ACT, ARG1, ARG2, ...)   ACT(ARG1, ARG2)   MACRO_FOR_PAIR_38(ACT,  __VA_ARGS__)
#define MACRO_FOR_PAIR_42(ACT, ARG1, ARG2, ...)   ACT(ARG1, ARG2)   MACRO_FOR_PAIR_40(ACT,  __VA_ARGS__)
#define MACRO_FOR_PAIR_44(ACT, ARG1, ARG2, ...)   ACT(ARG1, ARG2)   MACRO_FOR_PAIR_42(ACT,  __VA_ARGS__)
#define MACRO_FOR_PAIR_46(ACT, ARG1, ARG2, ...)   ACT(ARG1, ARG2)   MACRO_FOR_PAIR_44(ACT,  __VA_ARGS__)
#define MACRO_FOR_PAIR_48(ACT, ARG1, ARG2, ...)   ACT(ARG1, ARG2)   MACRO_FOR_PAIR_46(ACT,  __VA_ARGS__)
#define MACRO_FOR_PAIR_50(ACT, ARG1, ARG2, ...)   ACT(ARG1, ARG2)   MACRO_FOR_PAIR_48(ACT,  __VA_ARGS__)
#define MACRO_FOR_PAIR_52(ACT, ARG1, ARG2, ...)   ACT(ARG1, ARG2)   MACRO_FOR_PAIR_50(ACT,  __VA_ARGS__)
#define MACRO_FOR_PAIR_54(ACT, ARG1, ARG2, ...)   ACT(ARG1, ARG2)   MACRO_FOR_PAIR_52(ACT,  __VA_ARGS__)
#define MACRO_FOR_PAIR_56(ACT, ARG1, ARG2, ...)   ACT(ARG1, ARG2)   MACRO_FOR_PAIR_54(ACT,  __VA_ARGS__)
#define MACRO_FOR_PAIR_58(ACT, ARG1, ARG2, ...)   ACT(ARG1, ARG2)   MACRO_FOR_PAIR_56(ACT,  __VA_ARGS__)
#define MACRO_FOR_PAIR_60(ACT, ARG1, ARG2, ...)   ACT(ARG1, ARG2)   MACRO_FOR_PAIR_58(ACT,  __VA_ARGS__)
#define MACRO_FOR_PAIR_62(ACT, ARG1, ARG2, ...)   ACT(ARG1, ARG2)   MACRO_FOR_PAIR_60(ACT,  __VA_ARGS__)
#define MACRO_FOR_PAIR_64(ACT, ARG1, ARG2, ...)   ACT(ARG1, ARG2)   MACRO_FOR_PAIR_62(ACT,  __VA_ARGS__)

#define MACRO_FOR_PAIR__(N, ACT, ...)      MACRO_FOR_PAIR_##N(ACT, __VA_ARGS__)
#define MACRO_FOR_PAIR_(N, ACT, ...)       MACRO_FOR_PAIR__(N, ACT, __VA_ARGS__)

/** Performs @a ACT(), which takes a pair of arguments from @a __VA_ARGS__, for each pair.
 *  Supports up to @a N/2 pairs, where @a N is @a N in maximum defined @a MACRO_FOR_PAIR_N.
 */
#define MACRO_FOR_PAIR(ACT, ...)           MACRO_FOR_PAIR_(VA_NARGS(__VA_ARGS__), ACT, __VA_ARGS__)

#define _DECLARE_ENUM(NAME) NAME,
#define _DECLARE_ENUM_PAIR(NAME, HANDLER) NAME,
#define _DECLARE_STRING(NAME) #NAME,
#define _DECLARE_STRING_PAIR(NAME, HANDLER) #NAME,
#define _DECLARE_GUARD_PROTOTYPE(NAME, HANDLER) static bool HANDLER(void * p_data);
#define _DECLARE_ACTION_PROTOTYPE(NAME, HANDLER) static void HANDLER(void * p_data);
#define _DECLARE_HANDLER(NAME, HANDLER) HANDLER,

#define DECLARE_ENUM(...) MACRO_FOR_EACH(_DECLARE_ENUM, __VA_ARGS__)
#define DECLARE_ENUM_PAIR(...) MACRO_FOR_PAIR(_DECLARE_ENUM_PAIR, __VA_ARGS__)
#define DECLARE_STRING(...) MACRO_FOR_EACH(_DECLARE_STRING, __VA_ARGS__)
#define DECLARE_STRING_PAIR(...) MACRO_FOR_PAIR(_DECLARE_STRING_PAIR, __VA_ARGS__)
#define DECLARE_GUARD_PROTOTYPE(...) MACRO_FOR_PAIR(_DECLARE_GUARD_PROTOTYPE, __VA_ARGS__)
#define DECLARE_ACTION_PROTOTYPE(...) MACRO_FOR_PAIR(_DECLARE_ACTION_PROTOTYPE, __VA_ARGS__)
#define DECLARE_HANDLER(...) MACRO_FOR_PAIR(_DECLARE_HANDLER, __VA_ARGS__)

/** @} */

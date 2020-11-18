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

#ifndef CONFIG_SCENARIOS_H_
#define CONFIG_SCENARIOS_H_

/* USER_NOTE: Add more steps here if you want to customize the nodes further. */
/* Node setup steps */
#define CFG_DECLARE(A, B)  A,
typedef enum
{
    #include "config_steps.h"
} config_steps_t;
#undef CFG_DECLARE

#define CFG_DECLARE(A, B)  A##MODEL_ID = B,
typedef enum
{
    #include "config_steps.h"
} config_steps_to_model_ids_t;
#undef CFG_DECLARE

#define CONFIG_SCENARIO_COMMON                   \
    NODE_SETUP_CONFIG_COMPOSITION_GET,           \
    NODE_SETUP_CONFIG_NETWORK_TRANSMIT,          \
    NODE_SETUP_CONFIG_APPKEY_ADD,                \
    NODE_SETUP_CONFIG_APPKEY_BIND_HEALTH,        \
    NODE_SETUP_CONFIG_PUBLICATION_HEALTH

#define CONFIG_ONOFF_CLIENT                      \
    NODE_SETUP_GET_NEXT_ELEMENT,                 \
    NODE_SETUP_CONFIG_APPKEY_BIND_ONOFF_CLIENT,  \
    NODE_SETUP_CONFIG_PUBLICATION_ONOFF_CLIENT,  \
    NODE_SETUP_CONFIG_SUBSCRIPTION_ONOFF_CLIENT

#define CONFIG_LEVEL_CLIENT                      \
    NODE_SETUP_GET_NEXT_ELEMENT,                 \
    NODE_SETUP_CONFIG_APPKEY_BIND_LEVEL_CLIENT,  \
    NODE_SETUP_CONFIG_PUBLICATION_LEVEL_CLIENT,  \
    NODE_SETUP_CONFIG_SUBSCRIPTION_LEVEL_CLIENT

#define CONFIG_LL_CLIENT                      \
    NODE_SETUP_GET_NEXT_ELEMENT,              \
    NODE_SETUP_CONFIG_APPKEY_BIND_LL_CLIENT,  \
    NODE_SETUP_CONFIG_PUBLICATION_LL_CLIENT,  \
    NODE_SETUP_CONFIG_SUBSCRIPTION_LL_CLIENT

#define CONFIG_CTL_CLIENT                     \
    NODE_SETUP_GET_NEXT_ELEMENT,              \
    NODE_SETUP_CONFIG_APPKEY_BIND_CTL_CLIENT, \
    NODE_SETUP_CONFIG_PUBLICATION_CTL_CLIENT, \
    NODE_SETUP_CONFIG_SUBSCRIPTION_CTL_CLIENT

#define CONFIG_ONOFF_SERVER                      \
    NODE_SETUP_CONFIG_APPKEY_BIND_ONOFF_SERVER,  \
    NODE_SETUP_CONFIG_PUBLICATION_ONOFF_SERVER,  \
    NODE_SETUP_CONFIG_SUBSCRIPTION_ONOFF_SERVER

#define CONFIG_LEVEL_SERVER                      \
    NODE_SETUP_CONFIG_APPKEY_BIND_LEVEL_SERVER,  \
    NODE_SETUP_CONFIG_PUBLICATION_LEVEL_SERVER,  \
    NODE_SETUP_CONFIG_SUBSCRIPTION_LEVEL_SERVER

#define CONFIG_POWER_ONOFF_SERVER                       \
    NODE_SETUP_CONFIG_APPKEY_BIND_PONOFF_SERVER,        \
    NODE_SETUP_CONFIG_APPKEY_BIND_PONOFF_SETUP_SERVER,  \
    NODE_SETUP_CONFIG_PUBLICATION_PONOFF_SERVER,        \
    NODE_SETUP_CONFIG_PUBLICATION_PONOFF_SETUP_SERVER,  \
    NODE_SETUP_CONFIG_SUBSCRIPTION_PONOFF_SERVER,       \
    NODE_SETUP_CONFIG_SUBSCRIPTION_PONOFF_SETUP_SERVER

#define CONFIG_DEFAULT_TRANSITION_TIME_SERVER           \
    NODE_SETUP_CONFIG_APPKEY_BIND_DTT_SERVER,           \
    NODE_SETUP_CONFIG_PUBLICATION_DTT_SERVER,           \
    NODE_SETUP_CONFIG_SUBSCRIPTION_DTT_SERVER

#define CONFIG_SCENE_SERVER                            \
    NODE_SETUP_CONFIG_APPKEY_BIND_SCENE_SERVER,        \
    NODE_SETUP_CONFIG_APPKEY_BIND_SCENE_SETUP_SERVER,  \
    NODE_SETUP_CONFIG_PUBLICATION_SCENE_SERVER,        \
    NODE_SETUP_CONFIG_PUBLICATION_SCENE_SETUP_SERVER,  \
    NODE_SETUP_CONFIG_SUBSCRIPTION_SCENE_SERVER,       \
    NODE_SETUP_CONFIG_SUBSCRIPTION_SCENE_SETUP_SERVER

#define CONFIG_LIGHT_LIGHTNESS_SERVER                   \
    NODE_SETUP_CONFIG_APPKEY_BIND_LL_SERVER,            \
    NODE_SETUP_CONFIG_APPKEY_BIND_LL_SETUP_SERVER,      \
    NODE_SETUP_CONFIG_PUBLICATION_LL_SERVER,            \
    NODE_SETUP_CONFIG_PUBLICATION_LL_SETUP_SERVER,      \
    NODE_SETUP_CONFIG_SUBSCRIPTION_LL_SERVER,           \
    NODE_SETUP_CONFIG_SUBSCRIPTION_LL_SETUP_SERVER,     \
    CONFIG_ONOFF_SERVER,                                \
    CONFIG_LEVEL_SERVER,                                \
    CONFIG_POWER_ONOFF_SERVER,                          \
    CONFIG_DEFAULT_TRANSITION_TIME_SERVER

#define CONFIG_LIGHT_LIGHTNESS_SERVER_NO_ONOFF_SERVER   \
    NODE_SETUP_CONFIG_APPKEY_BIND_LL_SERVER,            \
    NODE_SETUP_CONFIG_APPKEY_BIND_LL_SETUP_SERVER,      \
    NODE_SETUP_CONFIG_PUBLICATION_LL_SERVER,            \
    NODE_SETUP_CONFIG_PUBLICATION_LL_SETUP_SERVER,      \
    NODE_SETUP_CONFIG_SUBSCRIPTION_LL_SERVER,           \
    NODE_SETUP_CONFIG_SUBSCRIPTION_LL_SETUP_SERVER,     \
    CONFIG_LEVEL_SERVER,                                \
    CONFIG_POWER_ONOFF_SERVER,                          \
    CONFIG_DEFAULT_TRANSITION_TIME_SERVER

#define CONFIG_LIGHT_CONTROL_SERVER                 \
    NODE_SETUP_GET_NEXT_ELEMENT,                    \
    NODE_SETUP_CONFIG_APPKEY_BIND_LC_SERVER,        \
    NODE_SETUP_CONFIG_APPKEY_BIND_LC_SETUP_SERVER,  \
    NODE_SETUP_CONFIG_PUBLICATION_LC_SERVER,        \
    NODE_SETUP_CONFIG_PUBLICATION_LC_SETUP_SERVER,  \
    NODE_SETUP_CONFIG_SUBSCRIPTION_LC_SERVER,       \
    NODE_SETUP_CONFIG_SUBSCRIPTION_LC_SETUP_SERVER, \
    NODE_SETUP_CONFIG_SUBSCRIPTION_LC_SERVER_ON_SENSOR_STATUS, \
    CONFIG_ONOFF_SERVER

#define CONFIG_CTL_TEMPERATURE_SERVER                      \
    NODE_SETUP_GET_NEXT_ELEMENT,                           \
    NODE_SETUP_CONFIG_APPKEY_BIND_CTL_TEMPERATURE_SERVER,  \
    NODE_SETUP_CONFIG_PUBLICATION_CTL_TEMPERATURE_SERVER,  \
    NODE_SETUP_CONFIG_SUBSCRIPTION_CTL_TEMPERATURE_SERVER, \
    CONFIG_LEVEL_SERVER

#define CONFIG_CTL_SERVER                            \
    NODE_SETUP_CONFIG_APPKEY_BIND_CTL_SERVER,        \
    NODE_SETUP_CONFIG_APPKEY_BIND_CTL_SETUP_SERVER,  \
    NODE_SETUP_CONFIG_PUBLICATION_CTL_SERVER,        \
    NODE_SETUP_CONFIG_PUBLICATION_CTL_SETUP_SERVER,  \
    NODE_SETUP_CONFIG_SUBSCRIPTION_CTL_SERVER,       \
    NODE_SETUP_CONFIG_SUBSCRIPTION_CTL_SETUP_SERVER, \
    CONFIG_CTL_TEMPERATURE_SERVER

#define CONFIG_SENSOR_SERVER                            \
    NODE_SETUP_CONFIG_APPKEY_BIND_SENSOR_SERVER,        \
    NODE_SETUP_CONFIG_APPKEY_BIND_SENSOR_SETUP_SERVER,  \
    NODE_SETUP_CONFIG_PUBLICATION_SENSOR_SERVER,        \
    NODE_SETUP_CONFIG_PUBLICATION_SENSOR_SETUP_SERVER,  \
    NODE_SETUP_CONFIG_SUBSCRIPTION_SENSOR_SERVER,       \
    NODE_SETUP_CONFIG_SUBSCRIPTION_SENSOR_SETUP_SERVER

#define CONFIG_SENSOR_CLIENT                     \
    NODE_SETUP_GET_NEXT_ELEMENT,                 \
    NODE_SETUP_CONFIG_APPKEY_BIND_SENSOR_CLIENT, \
    NODE_SETUP_CONFIG_PUBLICATION_SENSOR_CLIENT, \
    NODE_SETUP_CONFIG_SUBSCRIPTION_SENSOR_CLIENT

#define CONFIG_SCENE_CLIENT                     \
    NODE_SETUP_GET_NEXT_ELEMENT,                \
    NODE_SETUP_CONFIG_APPKEY_BIND_SCENE_CLIENT, \
    NODE_SETUP_CONFIG_PUBLICATION_SCENE_CLIENT, \
    NODE_SETUP_CONFIG_SUBSCRIPTION_SCENE_CLIENT

/* USER_NOTE:
You can define one or more such configuration steps for a given node in your network. The choice
of the steps can be done in @ref setup_select_steps() function.
*/
#define CONFIG_SCENARIO_LIGHT_SWITCH_CLIENT_EXAMPLE    \
    CONFIG_SCENARIO_COMMON,                            \
    CONFIG_ONOFF_CLIENT,                               \
    CONFIG_ONOFF_CLIENT,                               \
    NODE_SETUP_DONE

#define CONFIG_SCENARIO_LPN_EXAMPLE                    \
    CONFIG_SCENARIO_COMMON,                            \
    CONFIG_ONOFF_CLIENT,                               \
    NODE_SETUP_DONE

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
#define CONFIG_SCENARIO_LIGHT_SWITCH_SERVER_EXAMPLE    \
    CONFIG_SCENARIO_COMMON,                            \
    CONFIG_ONOFF_SERVER,                               \
    CONFIG_SCENE_SERVER,                               \
    NODE_SETUP_DONE
#else
#define CONFIG_SCENARIO_LIGHT_SWITCH_SERVER_EXAMPLE    \
    CONFIG_SCENARIO_COMMON,                            \
    CONFIG_ONOFF_SERVER,                               \
    NODE_SETUP_DONE
#endif

/* Sequence of steps for the level clients */
#define CONFIG_SCENARIO_DIMMING_CLIENT_EXAMPLE     \
    CONFIG_SCENARIO_COMMON,                        \
    CONFIG_LEVEL_CLIENT,                           \
    CONFIG_LEVEL_CLIENT,                           \
    NODE_SETUP_DONE

/* Sequence of steps for the level servers */
#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
#define CONFIG_SCENARIO_DIMMING_SERVER_EXAMPLE     \
    CONFIG_SCENARIO_COMMON,                        \
    CONFIG_LEVEL_SERVER,                           \
    CONFIG_SCENE_SERVER,                           \
    NODE_SETUP_DONE
#else
#define CONFIG_SCENARIO_DIMMING_SERVER_EXAMPLE     \
    CONFIG_SCENARIO_COMMON,                        \
    CONFIG_LEVEL_SERVER,                           \
    NODE_SETUP_DONE
#endif

#define CONFIG_SCENARIO_LIGHTNESS_CLIENT_EXAMPLE   \
    CONFIG_SCENARIO_COMMON,                        \
    CONFIG_LL_CLIENT,                              \
    CONFIG_LL_CLIENT,                              \
    NODE_SETUP_DONE

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
#define CONFIG_SCENARIO_LIGHTNESS_SERVER_EXAMPLE   \
    CONFIG_SCENARIO_COMMON,                        \
    CONFIG_LIGHT_LIGHTNESS_SERVER,                 \
    CONFIG_SCENE_SERVER,                           \
    NODE_SETUP_DONE
#else
#define CONFIG_SCENARIO_LIGHTNESS_SERVER_EXAMPLE   \
    CONFIG_SCENARIO_COMMON,                        \
    CONFIG_LIGHT_LIGHTNESS_SERVER,                 \
    NODE_SETUP_DONE
#endif

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
#define CONFIG_SCENARIO_LIGHT_LC_SERVER_EXAMPLE    \
    CONFIG_SCENARIO_COMMON,                        \
    CONFIG_LIGHT_LIGHTNESS_SERVER_NO_ONOFF_SERVER, \
    CONFIG_SCENE_SERVER,                           \
    CONFIG_LIGHT_CONTROL_SERVER,                   \
    NODE_SETUP_DONE
#else
#define CONFIG_SCENARIO_LIGHT_LC_SERVER_EXAMPLE    \
    CONFIG_SCENARIO_COMMON,                        \
    CONFIG_LIGHT_LIGHTNESS_SERVER_NO_ONOFF_SERVER, \
    CONFIG_LIGHT_CONTROL_SERVER,                   \
    NODE_SETUP_DONE
#endif

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
#define CONFIG_SCENARIO_LIGHT_CTL_SERVER_EXAMPLE  \
    CONFIG_SCENARIO_COMMON,                       \
    CONFIG_LIGHT_LIGHTNESS_SERVER,                \
    CONFIG_SCENE_SERVER,                          \
    CONFIG_CTL_SERVER,                            \
    NODE_SETUP_DONE
#else
#define CONFIG_SCENARIO_LIGHT_CTL_SERVER_EXAMPLE  \
    CONFIG_SCENARIO_COMMON,                       \
    CONFIG_LIGHT_LIGHTNESS_SERVER,                \
    CONFIG_CTL_SERVER,                            \
    NODE_SETUP_DONE
#endif

#define CONFIG_SCENARIO_LIGHT_CTL_LC_SERVER_EXAMPLE   \
    CONFIG_SCENARIO_COMMON,                           \
    CONFIG_LIGHT_LIGHTNESS_SERVER,                    \
    CONFIG_CTL_SERVER,                                \
    CONFIG_LIGHT_CONTROL_SERVER,                      \
    NODE_SETUP_DONE

#define CONFIG_SCENARIO_LIGHT_CTL_CLIENT_EXAMPLE  \
    CONFIG_SCENARIO_COMMON,                       \
    CONFIG_CTL_CLIENT,                            \
    CONFIG_CTL_CLIENT,                            \
    NODE_SETUP_DONE

#define CONFIG_SCENARIO_SENSOR_SERVER_EXAMPLE \
    CONFIG_SCENARIO_COMMON,                   \
    CONFIG_SENSOR_SERVER,                     \
    NODE_SETUP_DONE

#define CONFIG_SCENARIO_SENSOR_CLIENT_EXAMPLE  \
    CONFIG_SCENARIO_COMMON,                    \
    CONFIG_SENSOR_CLIENT,                      \
    CONFIG_SENSOR_CLIENT,                      \
    NODE_SETUP_DONE

#define CONFIG_SCENARIO_SCENE_CLIENT_EXAMPLE  \
    CONFIG_SCENARIO_COMMON,                   \
    CONFIG_SCENE_CLIENT,                      \
    CONFIG_SCENE_CLIENT,                      \
    NODE_SETUP_DONE

#endif /* CONFIG_SCENARIOS_H_ */

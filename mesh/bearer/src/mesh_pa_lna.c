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
#include "mesh_pa_lna.h"

#include <stdbool.h>
#include <stddef.h>

#include "mesh_pa_lna_internal.h"
#include "toolchain.h"
#include "nrf_mesh_assert.h"
#include "radio_config.h"
#include "bearer_handler.h"
#include "nrf_mesh_config_bearer.h"
#include "nordic_common.h"

NRF_MESH_STATIC_ASSERT(MESH_PA_SETUP_TIME_US <= RADIO_RAMPUP_TIME);
NRF_MESH_STATIC_ASSERT(MESH_LNA_SETUP_TIME_US <= RADIO_RAMPUP_TIME);

static struct
{
    mesh_pa_lna_gpiote_params_t params;
    volatile bool enabled;
} m_gpiote;

static void gpiote_setup(const ble_pa_lna_cfg_t * p_cfg, uint32_t gpiote_ch_id)
{
    NRF_GPIOTE->CONFIG[gpiote_ch_id] =
        (GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos) |
        ((p_cfg->active_high ? GPIOTE_CONFIG_OUTINIT_Low : GPIOTE_CONFIG_OUTINIT_High) << GPIOTE_CONFIG_OUTINIT_Pos) |
        (GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos) |
        (p_cfg->gpio_pin << GPIOTE_CONFIG_PSEL_Pos);
}

static void ppi_setup(const ble_pa_lna_cfg_t * p_cfg, __IO uint32_t * p_event)
{
    *p_event = 0;
    NRF_PPI->CH[m_gpiote.params.ppi_ch_id_set].EEP = (uint32_t) p_event;
    NRF_PPI->CH[m_gpiote.params.ppi_ch_id_set].TEP = (uint32_t) &NRF_GPIOTE->TASKS_OUT[m_gpiote.params.gpiote_ch_id];
    NRF_PPI->CHENSET = (1UL << m_gpiote.params.ppi_ch_id_set);
}

static void pin_clear(const ble_pa_lna_cfg_t * p_cfg)
{
    if (p_cfg->enable)
    {
        if (p_cfg->active_high)
        {
            NRF_GPIO->OUTCLR = (1UL << p_cfg->gpio_pin);
        }
        else
        {
            NRF_GPIO->OUTSET = (1UL << p_cfg->gpio_pin);
        }
    }
}

static void pin_init(const ble_pa_lna_cfg_t * p_cfg)
{
    if (p_cfg->enable)
    {
        NRF_GPIO->PIN_CNF[p_cfg->gpio_pin] =
            ((uint32_t) GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
            ((uint32_t) GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
            ((uint32_t) GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
            ((uint32_t) GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
            ((uint32_t) GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
        NRF_GPIO->DIRSET = (GPIO_DIRSET_PIN0_Output << (GPIO_DIRSET_PIN0_Pos + p_cfg->gpio_pin));
        pin_clear(p_cfg);
    }
}

uint32_t mesh_pa_lna_gpiote_enable(const mesh_pa_lna_gpiote_params_t * p_params)
{
    if (p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }
    if ((p_params->ppi_ch_id_set >= TS_TIMER_PPI_CH_START &&
         p_params->ppi_ch_id_set <= TS_TIMER_PPI_CH_STOP) ||
        (p_params->ppi_ch_id_clr >= TS_TIMER_PPI_CH_START &&
         p_params->ppi_ch_id_clr <= TS_TIMER_PPI_CH_STOP) ||
        (p_params->ppi_ch_id_set >= ARRAY_SIZE(NRF_PPI->CH)) ||
        (p_params->ppi_ch_id_clr >= ARRAY_SIZE(NRF_PPI->CH)) ||
        (p_params->ppi_ch_id_set == p_params->ppi_ch_id_clr) ||
        (p_params->gpiote_ch_id >= ARRAY_SIZE(NRF_GPIOTE->CONFIG)) ||
        (p_params->lna_cfg.gpio_pin >= ARRAY_SIZE(NRF_GPIO->PIN_CNF)) ||
        (p_params->pa_cfg.gpio_pin >= ARRAY_SIZE(NRF_GPIO->PIN_CNF)))
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    m_gpiote.enabled = false;
    m_gpiote.params = *p_params;
    m_gpiote.enabled = true;
    pin_init(&p_params->lna_cfg);
    pin_init(&p_params->pa_cfg);
    return NRF_SUCCESS;
}

void mesh_pa_lna_gpiote_disable(void)
{
    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);
    if (m_gpiote.enabled)
    {
        m_gpiote.enabled = false;
        mesh_pa_lna_cleanup();
    }
    _ENABLE_IRQS(was_masked);
}

const mesh_pa_lna_gpiote_params_t * mesh_pa_lna_gpiote_params_get(void)
{
    return ((m_gpiote.enabled) ? &m_gpiote.params : NULL);
}

void mesh_pa_setup_start(uint32_t rampup_start_capture_time, uint32_t timer_index)
{
    if (m_gpiote.enabled && m_gpiote.params.pa_cfg.enable)
    {
        gpiote_setup(&m_gpiote.params.pa_cfg, m_gpiote.params.ppi_ch_id_set);
        ppi_setup(&m_gpiote.params.pa_cfg, &BEARER_ACTION_TIMER->EVENTS_COMPARE[timer_index]);
        BEARER_ACTION_TIMER->CC[timer_index] = rampup_start_capture_time + RADIO_RAMPUP_TIME - MESH_PA_SETUP_TIME_US;
    }
}

void mesh_lna_setup_start(uint32_t rampup_start_capture_time, uint32_t timer_index)
{
    if (m_gpiote.enabled && m_gpiote.params.lna_cfg.enable)
    {
        gpiote_setup(&m_gpiote.params.lna_cfg, m_gpiote.params.ppi_ch_id_set);
        ppi_setup(&m_gpiote.params.lna_cfg, &BEARER_ACTION_TIMER->EVENTS_COMPARE[timer_index]);
        BEARER_ACTION_TIMER->CC[timer_index] = rampup_start_capture_time + RADIO_RAMPUP_TIME - MESH_LNA_SETUP_TIME_US;
    }
}

void mesh_pa_lna_disable_start(void)
{
    if (m_gpiote.enabled)
    {
        NRF_PPI->CHENCLR = (1UL << m_gpiote.params.ppi_ch_id_set);
    }
}

void mesh_pa_lna_setup_stop(void)
{
    if (m_gpiote.enabled)
    {
        NRF_RADIO->EVENTS_DISABLED = 0;
        NRF_PPI->CH[m_gpiote.params.ppi_ch_id_clr].EEP = (uint32_t) &NRF_RADIO->EVENTS_DISABLED;
        NRF_PPI->CH[m_gpiote.params.ppi_ch_id_clr].TEP = (uint32_t) &NRF_GPIOTE->TASKS_OUT[m_gpiote.params.gpiote_ch_id];
        NRF_PPI->CHENSET = (1UL << m_gpiote.params.ppi_ch_id_clr);
    }
}

void mesh_pa_lna_cleanup(void)
{
    if (m_gpiote.enabled)
    {
        NRF_PPI->CHENCLR = (1UL << m_gpiote.params.ppi_ch_id_set) | (1UL << m_gpiote.params.ppi_ch_id_clr);
        NRF_GPIOTE->CONFIG[m_gpiote.params.gpiote_ch_id] = 0;
        pin_clear(&m_gpiote.params.pa_cfg);
        pin_clear(&m_gpiote.params.lna_cfg);
    }
}

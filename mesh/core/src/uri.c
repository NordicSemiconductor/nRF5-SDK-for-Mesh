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

#include "uri.h"

#include <string.h>

#include "nrf_mesh_assert.h"

/*****************************************************************************
* Local defines
*****************************************************************************/
#define CODE_POINT_COUNT    (sizeof(m_code_point_strings) / sizeof(m_code_point_strings[0]))
/*****************************************************************************
* Static globals
*****************************************************************************/
static const char* m_code_point_strings[] =
{
    NULL,
    NULL,
    "aaa:",
    "aaas:",
    "about:",
    "acap:",
    "acct:",
    "cap:",
    "cid:",
    "coap:",
    "coaps:",
    "crid:",
    "data:",
    "dav:",
    "dict:",
    "dns:",
    "file:",
    "ftp:",
    "geo:",
    "go:",
    "gopher:",
    "h323:",
    "http:",
    "https:",
    "iax:",
    "icap:",
    "im:",
    "imap:",
    "info:",
    "ipp:",
    "ipps:",
    "iris:",
    "iris.beep:",
    "iris.xpc:",
    "iris.xpcs:",
    "iris.lwz:",
    "jabber:",
    "ldap:",
    "mailto:",
    "mid:",
    "msrp:",
    "msrps:",
    "mtqp:",
    "mupdate:",
    "news:",
    "nfs:",
    "ni:",
    "nih:",
    "nntp:",
    "opaquelocktoken:",
    "pop:",
    "pres:",
    "reload:",
    "rtsp:",
    "rtsps:",
    "rtspu:",
    "service:",
    "session:",
    "shttp:",
    "sieve:",
    "sip:",
    "sips:",
    "sms:",
    "snmp:",
    "soap.beep:",
    "soap.beeps:",
    "stun:",
    "stuns:",
    "tag:",
    "tel:",
    "telnet:",
    "tftp:",
    "thismessage:",
    "tn3270:",
    "tip:",
    "turn:",
    "turns:",
    "tv:",
    "urn:",
    "vemmi:",
    "ws:",
    "wss:",
    "xcon:",
    "xcon-userid:",
    "xmlrpc.beep:",
    "xmlrpc.beeps:",
    "xmpp:",
    "z39.50r:",
    "z39.50s:",
    "acr:",
    "adiumxtra:",
    "afp:",
    "afs:",
    "aim:",
    "apt:",
    "attachment:",
    "aw:",
    "barion:",
    "beshare:",
    "bitcoin:",
    "bolo:",
    "callto:",
    "chrome:",
    "chrome-extension:",
    "com-eventbrite-attendee:",
    "content:",
    "cvs:",
    "dlna-playsingle:",
    "dlna-playcontainer:",
    "dtn:",
    "dvb:",
    "ed2k:",
    "facetime:",
    "feed:",
    "feedready:",
    "finger:",
    "fish:",
    "gg:",
    "git:",
    "gizmoproject:",
    "gtalk:",
    "ham:",
    "hcp:",
    "icon:",
    "ipn:",
    "irc:",
    "irc6:",
    "ircs:",
    "itms:",
    "jar:",
    "jms:",
    "keyparc:",
    "lastfm:",
    "ldaps:",
    "magnet:",
    "maps:",
    "market:",
    "message:",
    "mms:",
    "ms-help:",
    "ms-settings-power:",
    "msnim:",
    "mumble:",
    "mvn:",
    "notes:",
    "oid:",
    "palm:",
    "paparazzi:",
    "pkcs11:",
    "platform:",
    "proxy:",
    "psyc:",
    "query:",
    "res:",
    "resource:",
    "rmi:",
    "rsync:",
    "rtmfp:",
    "rtmp:",
    "secondlife:",
    "sftp:",
    "sgn:",
    "skype:",
    "smb:",
    "smtp:",
    "soldat:",
    "spotify:",
    "ssh:",
    "steam:",
    "submit:",
    "svn:",
    "teamspeak:",
    "teliaeid:",
    "things:",
    "udp:",
    "unreal:",
    "ut2004:",
    "ventrilo:",
    "view-source:",
    "webcal:",
    "wtai:",
    "wyciwyg:",
    "xfire:",
    "xri:",
    "ymsgr:",
    "example:",
    "ms-settings-cloudstorage:",
};

/*****************************************************************************
* Static functions
*****************************************************************************/

/*****************************************************************************
* Interface functions
*****************************************************************************/
uint8_t uri_data_generate(uint8_t* p_dest, const char* p_src)
{
    NRF_MESH_ASSERT(p_dest != NULL);
    NRF_MESH_ASSERT(p_src != NULL);
    uint8_t code_point_len = 0;
    *p_dest = uri_scheme_name_code_point(p_src, &code_point_len);
    p_dest++;
    p_src += code_point_len;
    const uint8_t remaining_len = strlen(p_src);
    NRF_MESH_ASSERT(remaining_len <= URI_DATA_MAXLEN);
    memcpy(p_dest, p_src, remaining_len);
    return 1 + remaining_len;
}

uint8_t uri_data_parse(char* p_dest, const uint8_t* p_src, uint8_t data_len)
{
    NRF_MESH_ASSERT(p_dest != NULL);
    NRF_MESH_ASSERT(p_src != NULL);
    NRF_MESH_ASSERT(data_len != 0 && data_len < URI_DATA_MAXLEN);
    const char* p_code_point_str = uri_scheme_name_string(p_src[0]);
    uint8_t code_point_len = 0;
    if (p_code_point_str != NULL)
    {
        code_point_len = strlen(p_code_point_str);
        memcpy(p_dest, p_code_point_str, code_point_len);
        p_dest += code_point_len;
    }
    memcpy(p_dest, &p_src[1], data_len - 1);
    p_dest[data_len + code_point_len] = '\0';
    return code_point_len + data_len - 1;
}

uint8_t uri_scheme_name_code_point(const char* const p_name, uint8_t* p_len)
{
    NRF_MESH_ASSERT(p_name != NULL);
    NRF_MESH_ASSERT(p_len != NULL);
    if (p_name != NULL)
    {
        /* Linear search through a sorted list, could be optimized: */
        for (uint32_t i = 2; i < CODE_POINT_COUNT; i++)
        {
            *p_len = strlen(m_code_point_strings[i]);
            if (memcmp(p_name, m_code_point_strings[i], *p_len) == 0)
            {
                return i;
            }
        }
    }
    /* Nothing was found, according to CSSv6, this corresponds to code point 1. */
    *p_len = 0;
    return 1;
}

const char* uri_scheme_name_string(uint8_t code_point)
{
    if (code_point < CODE_POINT_COUNT)
    {
        return m_code_point_strings[code_point];
    }
    else
    {
        return NULL;
    }
}



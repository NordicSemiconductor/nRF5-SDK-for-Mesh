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

#ifndef URI_H__
#define URI_H__

#include <stdint.h>

/**
 * @defgroup URI Uniform Resource Identifier
 * @ingroup MESH_CORE
 * Small utility for generating and parsing URIs, as defined in Bluetooth CSSv6
 * Ch. 1.18. Note that the source file does not have to be included in the
 * build if you only mean to assemble URIs.
 * @{
 */

/** Maxmimum URI data length */
#define URI_DATA_MAXLEN  (64)
/** Longest code point string */
#define URI_CODE_POINT_MAXLEN (25)
/** Maxmimum URI string length */
#define URI_SCHEME_MAXLEN  (URI_SCHEME_MAXLEN + URI_CODE_POINT_MAXLEN)

/**
 * @defgroup URI_SCHEMES URI scheme code points
 *
 * Defines the code points for the different URI schemes. When building URIs,
 * use these defines instead of the string they represent.
 *
 * Example:
 * ("http://www.example.com") should be written as (URI_SCHEME_HTTP "//www.example.com")
 * @{
 */

#define URI_SCHEME_NONE                      "\x01" /**< URI Scheme code point for "none:". */
#define URI_SCHEME_AAA                       "\x02" /**< URI Scheme code point for "aaa:". */
#define URI_SCHEME_AAAS                      "\x03" /**< URI Scheme code point for "aaas:". */
#define URI_SCHEME_ABOUT                     "\x04" /**< URI Scheme code point for "about:". */
#define URI_SCHEME_ACAP                      "\x05" /**< URI Scheme code point for "acap:". */
#define URI_SCHEME_ACCT                      "\x06" /**< URI Scheme code point for "acct:". */
#define URI_SCHEME_CAP                       "\x07" /**< URI Scheme code point for "cap:". */
#define URI_SCHEME_CID                       "\x08" /**< URI Scheme code point for "cid:". */
#define URI_SCHEME_COAP                      "\x09" /**< URI Scheme code point for "coap:". */
#define URI_SCHEME_COAPS                     "\x0a" /**< URI Scheme code point for "coaps:". */
#define URI_SCHEME_CRID                      "\x0b" /**< URI Scheme code point for "crid:". */
#define URI_SCHEME_DATA                      "\x0c" /**< URI Scheme code point for "data:". */
#define URI_SCHEME_DAV                       "\x0d" /**< URI Scheme code point for "dav:". */
#define URI_SCHEME_DICT                      "\x0e" /**< URI Scheme code point for "dict:". */
#define URI_SCHEME_DNS                       "\x0f" /**< URI Scheme code point for "dns:". */
#define URI_SCHEME_FILE                      "\x10" /**< URI Scheme code point for "file:". */
#define URI_SCHEME_FTP                       "\x11" /**< URI Scheme code point for "ftp:". */
#define URI_SCHEME_GEO                       "\x12" /**< URI Scheme code point for "geo:". */
#define URI_SCHEME_GO                        "\x13" /**< URI Scheme code point for "go:". */
#define URI_SCHEME_GOPHER                    "\x14" /**< URI Scheme code point for "gopher:". */
#define URI_SCHEME_H323                      "\x15" /**< URI Scheme code point for "h323:". */
#define URI_SCHEME_HTTP                      "\x16" /**< URI Scheme code point for "http:". */
#define URI_SCHEME_HTTPS                     "\x17" /**< URI Scheme code point for "https:". */
#define URI_SCHEME_IAX                       "\x18" /**< URI Scheme code point for "iax:". */
#define URI_SCHEME_ICAP                      "\x19" /**< URI Scheme code point for "icap:". */
#define URI_SCHEME_IM                        "\x1a" /**< URI Scheme code point for "im:". */
#define URI_SCHEME_IMAP                      "\x1b" /**< URI Scheme code point for "imap:". */
#define URI_SCHEME_INFO                      "\x1c" /**< URI Scheme code point for "info:". */
#define URI_SCHEME_IPP                       "\x1d" /**< URI Scheme code point for "ipp:". */
#define URI_SCHEME_IPPS                      "\x1e" /**< URI Scheme code point for "ipps:". */
#define URI_SCHEME_IRIS                      "\x1f" /**< URI Scheme code point for "iris:". */
#define URI_SCHEME_IRIS_BEEP                 "\x20" /**< URI Scheme code point for "iris.beep:". */
#define URI_SCHEME_IRIS_XPC                  "\x21" /**< URI Scheme code point for "iris.xpc:". */
#define URI_SCHEME_IRIS_XPCS                 "\x22" /**< URI Scheme code point for "iris.xpcs:". */
#define URI_SCHEME_IRIS_LWZ                  "\x23" /**< URI Scheme code point for "iris.lwz:". */
#define URI_SCHEME_JABBER                    "\x24" /**< URI Scheme code point for "jabber:". */
#define URI_SCHEME_LDAP                      "\x25" /**< URI Scheme code point for "ldap:". */
#define URI_SCHEME_MAILTO                    "\x26" /**< URI Scheme code point for "mailto:". */
#define URI_SCHEME_MID                       "\x27" /**< URI Scheme code point for "mid:". */
#define URI_SCHEME_MSRP                      "\x28" /**< URI Scheme code point for "msrp:". */
#define URI_SCHEME_MSRPS                     "\x29" /**< URI Scheme code point for "msrps:". */
#define URI_SCHEME_MTQP                      "\x2a" /**< URI Scheme code point for "mtqp:". */
#define URI_SCHEME_MUPDATE                   "\x2b" /**< URI Scheme code point for "mupdate:". */
#define URI_SCHEME_NEWS                      "\x2c" /**< URI Scheme code point for "news:". */
#define URI_SCHEME_NFS                       "\x2d" /**< URI Scheme code point for "nfs:". */
#define URI_SCHEME_NI                        "\x2e" /**< URI Scheme code point for "ni:". */
#define URI_SCHEME_NIH                       "\x2f" /**< URI Scheme code point for "nih:". */
#define URI_SCHEME_NNTP                      "\x30" /**< URI Scheme code point for "nntp:". */
#define URI_SCHEME_OPAQUELOCKTOKEN           "\x31" /**< URI Scheme code point for "opaquelocktoken:". */
#define URI_SCHEME_POP                       "\x32" /**< URI Scheme code point for "pop:". */
#define URI_SCHEME_PRES                      "\x33" /**< URI Scheme code point for "pres:". */
#define URI_SCHEME_RELOAD                    "\x34" /**< URI Scheme code point for "reload:". */
#define URI_SCHEME_RTSP                      "\x35" /**< URI Scheme code point for "rtsp:". */
#define URI_SCHEME_RTSPS                     "\x36" /**< URI Scheme code point for "rtsps:". */
#define URI_SCHEME_RTSPU                     "\x37" /**< URI Scheme code point for "rtspu:". */
#define URI_SCHEME_SERVICE                   "\x38" /**< URI Scheme code point for "service:". */
#define URI_SCHEME_SESSION                   "\x39" /**< URI Scheme code point for "session:". */
#define URI_SCHEME_SHTTP                     "\x3a" /**< URI Scheme code point for "shttp:". */
#define URI_SCHEME_SIEVE                     "\x3b" /**< URI Scheme code point for "sieve:". */
#define URI_SCHEME_SIP                       "\x3c" /**< URI Scheme code point for "sip:". */
#define URI_SCHEME_SIPS                      "\x3d" /**< URI Scheme code point for "sips:". */
#define URI_SCHEME_SMS                       "\x3e" /**< URI Scheme code point for "sms:". */
#define URI_SCHEME_SNMP                      "\x3f" /**< URI Scheme code point for "snmp:". */
#define URI_SCHEME_SOAP_BEEP                 "\x40" /**< URI Scheme code point for "soap.beep:". */
#define URI_SCHEME_SOAP_BEEPS                "\x41" /**< URI Scheme code point for "soap.beeps:". */
#define URI_SCHEME_STUN                      "\x42" /**< URI Scheme code point for "stun:". */
#define URI_SCHEME_STUNS                     "\x43" /**< URI Scheme code point for "stuns:". */
#define URI_SCHEME_TAG                       "\x44" /**< URI Scheme code point for "tag:". */
#define URI_SCHEME_TEL                       "\x45" /**< URI Scheme code point for "tel:". */
#define URI_SCHEME_TELNET                    "\x46" /**< URI Scheme code point for "telnet:". */
#define URI_SCHEME_TFTP                      "\x47" /**< URI Scheme code point for "tftp:". */
#define URI_SCHEME_THISMESSAGE               "\x48" /**< URI Scheme code point for "thismessage:". */
#define URI_SCHEME_TN3270                    "\x49" /**< URI Scheme code point for "tn3270:". */
#define URI_SCHEME_TIP                       "\x4a" /**< URI Scheme code point for "tip:". */
#define URI_SCHEME_TURN                      "\x4b" /**< URI Scheme code point for "turn:". */
#define URI_SCHEME_TURNS                     "\x4c" /**< URI Scheme code point for "turns:". */
#define URI_SCHEME_TV                        "\x4d" /**< URI Scheme code point for "tv:". */
#define URI_SCHEME_URN                       "\x4e" /**< URI Scheme code point for "urn:". */
#define URI_SCHEME_VEMMI                     "\x4f" /**< URI Scheme code point for "vemmi:". */
#define URI_SCHEME_WS                        "\x50" /**< URI Scheme code point for "ws:". */
#define URI_SCHEME_WSS                       "\x51" /**< URI Scheme code point for "wss:". */
#define URI_SCHEME_XCON                      "\x52" /**< URI Scheme code point for "xcon:". */
#define URI_SCHEME_XCON_USERID               "\x53" /**< URI Scheme code point for "xcon_userid:". */
#define URI_SCHEME_XMLRPC_BEEP               "\x54" /**< URI Scheme code point for "xmlrpc.beep:". */
#define URI_SCHEME_XMLRPC_BEEPS              "\x55" /**< URI Scheme code point for "xmlrpc.beeps:". */
#define URI_SCHEME_XMPP                      "\x56" /**< URI Scheme code point for "xmpp:". */
#define URI_SCHEME_Z39_50R                   "\x57" /**< URI Scheme code point for "z39.50r:". */
#define URI_SCHEME_Z39_50S                   "\x58" /**< URI Scheme code point for "z39.50s:". */
#define URI_SCHEME_ACR                       "\x59" /**< URI Scheme code point for "acr:". */
#define URI_SCHEME_ADIUMXTRA                 "\x5a" /**< URI Scheme code point for "adiumxtra:". */
#define URI_SCHEME_AFP                       "\x5b" /**< URI Scheme code point for "afp:". */
#define URI_SCHEME_AFS                       "\x5c" /**< URI Scheme code point for "afs:". */
#define URI_SCHEME_AIM                       "\x5d" /**< URI Scheme code point for "aim:". */
#define URI_SCHEME_APT                       "\x5e" /**< URI Scheme code point for "apt:". */
#define URI_SCHEME_ATTACHMENT                "\x5f" /**< URI Scheme code point for "attachment:". */
#define URI_SCHEME_AW                        "\x60" /**< URI Scheme code point for "aw:". */
#define URI_SCHEME_BARION                    "\x61" /**< URI Scheme code point for "barion:". */
#define URI_SCHEME_BESHARE                   "\x62" /**< URI Scheme code point for "beshare:". */
#define URI_SCHEME_BITCOIN                   "\x63" /**< URI Scheme code point for "bitcoin:". */
#define URI_SCHEME_BOLO                      "\x64" /**< URI Scheme code point for "bolo:". */
#define URI_SCHEME_CALLTO                    "\x65" /**< URI Scheme code point for "callto:". */
#define URI_SCHEME_CHROME                    "\x66" /**< URI Scheme code point for "chrome:". */
#define URI_SCHEME_CHROME_EXTENSION          "\x67" /**< URI Scheme code point for "chrome_extension:". */
#define URI_SCHEME_COM_EVENTBRITE_ATTENDEE   "\x68" /**< URI Scheme code point for "com_eventbrite_attendee:". */
#define URI_SCHEME_CONTENT                   "\x69" /**< URI Scheme code point for "content:". */
#define URI_SCHEME_CVS                       "\x6a" /**< URI Scheme code point for "cvs:". */
#define URI_SCHEME_DLNA_PLAYSINGLE           "\x6b" /**< URI Scheme code point for "dlna_playsingle:". */
#define URI_SCHEME_DLNA_PLAYCONTAINER        "\x6c" /**< URI Scheme code point for "dlna_playcontainer:". */
#define URI_SCHEME_DTN                       "\x6d" /**< URI Scheme code point for "dtn:". */
#define URI_SCHEME_DVB                       "\x6e" /**< URI Scheme code point for "dvb:". */
#define URI_SCHEME_ED2K                      "\x6f" /**< URI Scheme code point for "ed2k:". */
#define URI_SCHEME_FACETIME                  "\x70" /**< URI Scheme code point for "facetime:". */
#define URI_SCHEME_FEED                      "\x71" /**< URI Scheme code point for "feed:". */
#define URI_SCHEME_FEEDREADY                 "\x72" /**< URI Scheme code point for "feedready:". */
#define URI_SCHEME_FINGER                    "\x73" /**< URI Scheme code point for "finger:". */
#define URI_SCHEME_FISH                      "\x74" /**< URI Scheme code point for "fish:". */
#define URI_SCHEME_GG                        "\x75" /**< URI Scheme code point for "gg:". */
#define URI_SCHEME_GIT                       "\x76" /**< URI Scheme code point for "git:". */
#define URI_SCHEME_GIZMOPROJECT              "\x77" /**< URI Scheme code point for "gizmoproject:". */
#define URI_SCHEME_GTALK                     "\x78" /**< URI Scheme code point for "gtalk:". */
#define URI_SCHEME_HAM                       "\x79" /**< URI Scheme code point for "ham:". */
#define URI_SCHEME_HCP                       "\x7a" /**< URI Scheme code point for "hcp:". */
#define URI_SCHEME_ICON                      "\x7b" /**< URI Scheme code point for "icon:". */
#define URI_SCHEME_IPN                       "\x7c" /**< URI Scheme code point for "ipn:". */
#define URI_SCHEME_IRC                       "\x7d" /**< URI Scheme code point for "irc:". */
#define URI_SCHEME_IRC6                      "\x7e" /**< URI Scheme code point for "irc6:". */
#define URI_SCHEME_IRCS                      "\x7f" /**< URI Scheme code point for "ircs:". */
#define URI_SCHEME_ITMS                      "\x80" /**< URI Scheme code point for "itms:". */
#define URI_SCHEME_JAR                       "\x81" /**< URI Scheme code point for "jar:". */
#define URI_SCHEME_JMS                       "\x82" /**< URI Scheme code point for "jms:". */
#define URI_SCHEME_KEYPARC                   "\x83" /**< URI Scheme code point for "keyparc:". */
#define URI_SCHEME_LASTFM                    "\x84" /**< URI Scheme code point for "lastfm:". */
#define URI_SCHEME_LDAPS                     "\x85" /**< URI Scheme code point for "ldaps:". */
#define URI_SCHEME_MAGNET                    "\x86" /**< URI Scheme code point for "magnet:". */
#define URI_SCHEME_MAPS                      "\x87" /**< URI Scheme code point for "maps:". */
#define URI_SCHEME_MARKET                    "\x88" /**< URI Scheme code point for "market:". */
#define URI_SCHEME_MESSAGE                   "\x89" /**< URI Scheme code point for "message:". */
#define URI_SCHEME_MMS                       "\x8a" /**< URI Scheme code point for "mms:". */
#define URI_SCHEME_MS_HELP                   "\x8b" /**< URI Scheme code point for "ms_help:". */
#define URI_SCHEME_MS_SETTINGS_POWER         "\x8c" /**< URI Scheme code point for "ms_settings_power:". */
#define URI_SCHEME_MSNIM                     "\x8d" /**< URI Scheme code point for "msnim:". */
#define URI_SCHEME_MUMBLE                    "\x8e" /**< URI Scheme code point for "mumble:". */
#define URI_SCHEME_MVN                       "\x8f" /**< URI Scheme code point for "mvn:". */
#define URI_SCHEME_NOTES                     "\x90" /**< URI Scheme code point for "notes:". */
#define URI_SCHEME_OID                       "\x91" /**< URI Scheme code point for "oid:". */
#define URI_SCHEME_PALM                      "\x92" /**< URI Scheme code point for "palm:". */
#define URI_SCHEME_PAPARAZZI                 "\x93" /**< URI Scheme code point for "paparazzi:". */
#define URI_SCHEME_PKCS11                    "\x94" /**< URI Scheme code point for "pkcs11:". */
#define URI_SCHEME_PLATFORM                  "\x95" /**< URI Scheme code point for "platform:". */
#define URI_SCHEME_PROXY                     "\x96" /**< URI Scheme code point for "proxy:". */
#define URI_SCHEME_PSYC                      "\x97" /**< URI Scheme code point for "psyc:". */
#define URI_SCHEME_QUERY                     "\x98" /**< URI Scheme code point for "query:". */
#define URI_SCHEME_RES                       "\x99" /**< URI Scheme code point for "res:". */
#define URI_SCHEME_RESOURCE                  "\x9a" /**< URI Scheme code point for "resource:". */
#define URI_SCHEME_RMI                       "\x9b" /**< URI Scheme code point for "rmi:". */
#define URI_SCHEME_RSYNC                     "\x9c" /**< URI Scheme code point for "rsync:". */
#define URI_SCHEME_RTMFP                     "\x9d" /**< URI Scheme code point for "rtmfp:". */
#define URI_SCHEME_RTMP                      "\x9e" /**< URI Scheme code point for "rtmp:". */
#define URI_SCHEME_SECONDLIFE                "\x9f" /**< URI Scheme code point for "secondlife:". */
#define URI_SCHEME_SFTP                      "\xa0" /**< URI Scheme code point for "sftp:". */
#define URI_SCHEME_SGN                       "\xa1" /**< URI Scheme code point for "sgn:". */
#define URI_SCHEME_SKYPE                     "\xa2" /**< URI Scheme code point for "skype:". */
#define URI_SCHEME_SMB                       "\xa3" /**< URI Scheme code point for "smb:". */
#define URI_SCHEME_SMTP                      "\xa4" /**< URI Scheme code point for "smtp:". */
#define URI_SCHEME_SOLDAT                    "\xa5" /**< URI Scheme code point for "soldat:". */
#define URI_SCHEME_SPOTIFY                   "\xa6" /**< URI Scheme code point for "spotify:". */
#define URI_SCHEME_SSH                       "\xa7" /**< URI Scheme code point for "ssh:". */
#define URI_SCHEME_STEAM                     "\xa8" /**< URI Scheme code point for "steam:". */
#define URI_SCHEME_SUBMIT                    "\xa9" /**< URI Scheme code point for "submit:". */
#define URI_SCHEME_SVN                       "\xaa" /**< URI Scheme code point for "svn:". */
#define URI_SCHEME_TEAMSPEAK                 "\xab" /**< URI Scheme code point for "teamspeak:". */
#define URI_SCHEME_TELIAEID                  "\xac" /**< URI Scheme code point for "teliaeid:". */
#define URI_SCHEME_THINGS                    "\xad" /**< URI Scheme code point for "things:". */
#define URI_SCHEME_UDP                       "\xae" /**< URI Scheme code point for "udp:". */
#define URI_SCHEME_UNREAL                    "\xaf" /**< URI Scheme code point for "unreal:". */
#define URI_SCHEME_UT2004                    "\xb0" /**< URI Scheme code point for "ut2004:". */
#define URI_SCHEME_VENTRILO                  "\xb1" /**< URI Scheme code point for "ventrilo:". */
#define URI_SCHEME_VIEW_SOURCE               "\xb2" /**< URI Scheme code point for "view_source:". */
#define URI_SCHEME_WEBCAL                    "\xb3" /**< URI Scheme code point for "webcal:". */
#define URI_SCHEME_WTAI                      "\xb4" /**< URI Scheme code point for "wtai:". */
#define URI_SCHEME_WYCIWYG                   "\xb5" /**< URI Scheme code point for "wyciwyg:". */
#define URI_SCHEME_XFIRE                     "\xb6" /**< URI Scheme code point for "xfire:". */
#define URI_SCHEME_XRI                       "\xb7" /**< URI Scheme code point for "xri:". */
#define URI_SCHEME_YMSGR                     "\xb8" /**< URI Scheme code point for "ymsgr:". */
#define URI_SCHEME_EXAMPLE                   "\xb9" /**< URI Scheme code point for "example:". */
#define URI_SCHEME_MS_SETTINGS_CLOUDSTORAGE  "\xba" /**< URI Scheme code point for "ms_settings_cloudstorage:". */

/** @} */

/**
 * Generate URI data from the given URI string.
 *
 * @param[in,out] p_dest Destination buffer to write the URI data to. Must be
 * at least as large as the source string.
 * @param[in] p_src Source string to generate from. Must be a NULL-terminated
 * string, that result in a URI data representation no longer than @ref
 * URI_SCHEME_MAXLEN.
 *
 * @return Resulting length of the @p p_dest array.
 */
uint8_t uri_data_generate(uint8_t* p_dest, const char* p_src);

/**
 * Parse the given URI data into a string.
 *
 * @param[in,out] p_dest Destination buffer to write the resulting string to.
 * Must be the length of the source array + @ref URI_CODE_POINT_MAXLEN.
 * @param[in] p_src Source URI data to parse.
 * @param[in] data_len Length of the given URI data in @p p_src.
 *
 * @return The resulting string length.
 */
uint8_t uri_data_parse(char* p_dest, const uint8_t* p_src, uint8_t data_len);

/**
 * Get the URI scheme code point of the start of the given string, according to
 * https://www.bluetooth.com/specifications/assigned-numbers/uri-scheme-name-string-mapping.
 *
 * @param[in] p_string String to get a code point from.
 * @param[out] p_len Will be set to the length of the found code point string,
 * to allow the user to move their string pointer to the proceeding character.
 *
 * @return Code point for the start of the given string, or 1 if no code point
 * corresponding to the given string was found.
 */
uint8_t uri_scheme_name_code_point(const char* const p_string, uint8_t* p_len);

/**
 * Get the string representation of the given URI code point, according to
 * https://www.bluetooth.com/specifications/assigned-numbers/uri-scheme-name-string-mapping.
 *
 * @param[in] code_point Code point to get the URI string representation for.
 *
 * @return The string representation fo the given URI code point, or NULL if
 * the given code point doesn't match any known code points.
 */
const char* uri_scheme_name_string(uint8_t code_point);




/** @} */

#endif /* URI_H__ */


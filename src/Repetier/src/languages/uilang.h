/*
    This file is part of Repetier-Firmware.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Repetier-Firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.

*/

#if !defined(UI_DISPLAY_CHARSET) || UI_DISPLAY_CHARSET > 3
#define UI_DISPLAY_CHARSET 1
#endif

//Symbolic character values for specific symbols.
//May be overridden for different controllers, character sets, etc.
#define cUP "\001"
#define cDEG "\002"
#define cSEL "\003"
#define cUNSEL "\004"
#define cTEMP "\005"
#define cFOLD "\006"
#define bFOLD 6
#define cARROW "\176"

#if UI_DISPLAY_CHARSET == 0 // ASCII fall back
#define CHAR_RIGHT '-'
#define CHAR_SELECTOR '>'
#define CHAR_SELECTED '*'
#define STR_auml "ae"
#define STR_Auml "Ae"
#define STR_uuml "ue"
#define STR_Uuml "Ue"
#define STR_ouml "oe"
#define STR_Ouml "Oe"
#define STR_szlig "ss"
#endif

#if UI_DISPLAY_CHARSET == 1 // HD44870 charset with knji chars
#define CHAR_RIGHT 0x7e
#define CHAR_SELECTOR '>'
#define CHAR_SELECTED '*'
#define STR_auml "\xe1"
#define STR_Auml "Ae"
#define STR_uuml "\365"
#define STR_Uuml "Ue"
#define STR_ouml "\357"
#define STR_Ouml "Oe"
#define STR_szlig "\342"
#endif

#if UI_DISPLAY_CHARSET == 2 // Western charset
#define CHAR_RIGHT 0xbc
#define CHAR_SELECTOR 0xf6
#define CHAR_SELECTED '*'
#define STR_auml "\204"
#define STR_Auml "\216"
#define STR_uuml "\201"
#define STR_Uuml "\212"
#define STR_ouml "\204"
#define STR_Ouml "\211"
#define STR_szlig "160"
#endif


#if UI_DISPLAY_CHARSET==3 // U8glib
#define CHAR_RIGHT 187 //>>
#define CHAR_SELECTOR 255 //'>'
#define CHAR_SELECTED 254 //'*'
#define STR_auml "\344"
#define STR_Auml "\304"
#define STR_uuml "\374"
#define STR_Uuml "\334"
#define STR_ouml "\366"
#define STR_Ouml "\326"
#define STR_szlig "\337"
#endif

#define LANGUAGE_EN_ID 0
#define LANGUAGE_DE_ID 1
#define LANGUAGE_NL_ID 2
#define LANGUAGE_PT_ID 3
#define LANGAUGE_IT_ID 4
#define LANGUAGE_ES_ID 5
#define LANGUAGE_SE_ID 6
#define LANGUAGE_FR_ID 7
#define LANGUAGE_CZ_ID 8
#define LANGUAGE_PL_ID 9
#define LANGUAGE_TR_ID 10
#define LANGUAGE_FI_ID 11

#define NUM_LANGUAGES_KNOWN 12
#define NUM_TRANSLATED_WORDS 311

// Universal definitions

#define UI_TEXT_SEL              cSEL
#define UI_TEXT_NOSEL            cUNSEL

#include "ids.h"
#include "en.h"
#include "de.h"
#include "nl.h"
#include "es.h"
#include "pt.h"
#include "fr.h"
#include "it.h"
#include "se.h"
#include "cz.h"
#include "pl.h"
#include "tr.h"
#include "fi.h"

#define TRANS(x) UI_STRING(F ## x,x)
#ifdef CUSTOM_TRANSLATIONS
#include "CustomTranslations.h"
#else
#define NUM_EXTRA_TRANSLATIONS 0
#define CUSTOM_TRANS_EN
#define CUSTOM_TRANS_DE
#define CUSTOM_TRANS_NL
#define CUSTOM_TRANS_ES
#define CUSTOM_TRANS_PT
#define CUSTOM_TRANS_FR
#define CUSTOM_TRANS_IT
#define CUSTOM_TRANS_SE
#define CUSTOM_TRANS_CZ
#define CUSTOM_TRANS_PL
#define CUSTOM_TRANS_TR
#define CUSTOM_TRANS_FI
#endif

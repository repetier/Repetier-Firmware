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

    This firmware is a nearly complete rewrite of the sprinter firmware
    by kliment (https://github.com/kliment/Sprinter)
    which based on Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
*/

#include "Repetier.h"

#if UI_DISPLAY_TYPE != NO_DISPLAY

#if LANGUAGE_EN_ACTIVE+LANGUAGE_DE_ACTIVE+LANGUAGE_ES_ACTIVE+LANGUAGE_PT_ACTIVE+LANGUAGE_FR_ACTIVE+LANGUAGE_NL_ACTIVE+LANGUAGE_IT_ACTIVE+LANGUAGE_SE_ACTIVE+LANGUAGE_CZ_ACTIVE+LANGUAGE_PL_ACTIVE < 1
#error No language for diplay selected
#endif // LANGUAGE_EN_ACTIVE

#define TRANS(x) UI_STRING(F ## x,x)

// Translations of ui

#if LANGUAGE_EN_ACTIVE
TRANS(UI_TEXT_ON_EN);
TRANS(UI_TEXT_OFF_EN);
TRANS(UI_TEXT_NA_EN);
TRANS(UI_TEXT_YES_EN);
TRANS(UI_TEXT_NO_EN);
TRANS(UI_TEXT_PRINT_POS_EN);
TRANS(UI_TEXT_PRINTING_EN);
TRANS(UI_TEXT_IDLE_EN);
TRANS(UI_TEXT_NOSDCARD_EN);
TRANS(UI_TEXT_ERROR_EN);
TRANS(UI_TEXT_BACK_EN);
TRANS(UI_TEXT_QUICK_SETTINGS_EN);
TRANS(UI_TEXT_ERRORMSG_EN);

PGM_P translations_en[NUM_TRANSLATED_WORDS] PROGMEM = {
 FUI_TEXT_ON_EN,
 FUI_TEXT_OFF_EN,
 FUI_TEXT_NA_EN,
 FUI_TEXT_YES_EN,
 FUI_TEXT_NO_EN,
 FUI_TEXT_PRINT_POS_EN,
 FUI_TEXT_PRINTING_EN,
 FUI_TEXT_IDLE_EN,
 FUI_TEXT_NOSDCARD_EN,
 FUI_TEXT_ERROR_EN,
 FUI_TEXT_BACK_EN,
 FUI_TEXT_QUICK_SETTINGS_EN,
 FUI_TEXT_ERRORMSG_EN
};
#define LANG_EN_TABLE translations_en
#else
#define LANG_EN_TABLE NULL
#endif // LANGUAGE_EN_ACTIVE

#if LANGUAGE_DE_ACTIVE
TRANS(UI_TEXT_ON_DE);
TRANS(UI_TEXT_OFF_DE);
TRANS(UI_TEXT_NA_DE);
TRANS(UI_TEXT_YES_DE);
TRANS(UI_TEXT_NO_DE);
TRANS(UI_TEXT_PRINT_POS_DE);
TRANS(UI_TEXT_PRINTING_DE);
TRANS(UI_TEXT_IDLE_DE);
TRANS(UI_TEXT_NOSDCARD_DE);
TRANS(UI_TEXT_ERROR_DE);
TRANS(UI_TEXT_BACK_DE);
TRANS(UI_TEXT_QUICK_SETTINGS_DE);
TRANS(UI_TEXT_ERRORMSG_DE);

PGM_P translations_de[NUM_TRANSLATED_WORDS] PROGMEM = {
 FUI_TEXT_ON_DE,
 FUI_TEXT_OFF_DE,
 FUI_TEXT_NA_DE,
 FUI_TEXT_YES_DE,
 FUI_TEXT_NO_DE,
 FUI_TEXT_PRINT_POS_DE,
 FUI_TEXT_PRINTING_DE,
 FUI_TEXT_IDLE_DE,
 FUI_TEXT_NOSDCARD_DE,
 FUI_TEXT_ERROR_DE,
 FUI_TEXT_BACK_DE,
 FUI_TEXT_QUICK_SETTINGS_DE,
 FUI_TEXT_ERRORMSG_DE
};
#define LANG_DE_TABLE translations_de
#else
#define LANG_DE_TABLE NULL
#endif // LANGUAGE_DE_ACTIVE

#if LANGUAGE_NL_ACTIVE
TRANS(UI_TEXT_ON_NL);
TRANS(UI_TEXT_OFF_NL);
TRANS(UI_TEXT_NA_NL);
TRANS(UI_TEXT_YES_NL);
TRANS(UI_TEXT_NO_NL);
TRANS(UI_TEXT_PRINT_POS_NL);
TRANS(UI_TEXT_PRINTING_NL);
TRANS(UI_TEXT_IDLE_NL);
TRANS(UI_TEXT_NOSDCARD_NL);
TRANS(UI_TEXT_ERROR_NL);
TRANS(UI_TEXT_BACK_NL);
TRANS(UI_TEXT_QUICK_SETTINGS_NL);
TRANS(UI_TEXT_ERRORMSG_NL);

PGM_P translations_nl[NUM_TRANSLATED_WORDS] PROGMEM = {
 FUI_TEXT_ON_NL,
 FUI_TEXT_OFF_NL,
 FUI_TEXT_NA_NL,
 FUI_TEXT_YES_NL,
 FUI_TEXT_NO_NL,
 FUI_TEXT_PRINT_POS_NL,
 FUI_TEXT_PRINTING_NL,
 FUI_TEXT_IDLE_NL,
 FUI_TEXT_NOSDCARD_NL,
 FUI_TEXT_ERROR_NL,
 FUI_TEXT_BACK_NL,
 FUI_TEXT_QUICK_SETTINGS_NL,
 FUI_TEXT_ERRORMSG_NL
};
#define LANG_NL_TABLE translations_nl
#else
#define LANG_NL_TABLE NULL
#endif // LANGUAGE_DE_ACTIVE


#define LANG_PT_TABLE NULL
#define LANG_IT_TABLE NULL
#define LANG_ES_TABLE NULL
#define LANG_SE_TABLE NULL
#define LANG_FR_TABLE NULL
#define LANG_CZ_TABLE NULL
#define LANG_PL_TABLE NULL

// References to the possible languages

PGM_P const * translations[NUM_LANGUAGES_KNOWN] PROGMEM = {
    LANG_EN_TABLE,
    LANG_DE_TABLE,
    LANG_NL_TABLE,
    LANG_PT_TABLE,
    LANG_IT_TABLE,
    LANG_ES_TABLE,
    LANG_SE_TABLE,
    LANG_FR_TABLE,
    LANG_CZ_TABLE,
    LANG_PL_TABLE
};

// Array in flash to select only valid languages

const uint8_t availableLanguages[] PROGMEM = {
#if LANGUAGE_EN_ACTIVE
    0,
#endif // LANGUAGE_EN_ACTIVE
#if LANGUAGE_DE_ACTIVE
    1,
#endif // LANGUAGE_DE_ACTIVE
#if LANGUAGE_NL_ACTIVE
    2,
#endif // LANGUAGE_NL_ACTIVE
#if LANGUAGE_PT_ACTIVE
    3,
#endif // LANGUAGE_PT_ACTIVE
#if LANGUAGE_IT_ACTIVE
    4,
#endif // LANGUAGE_IT_ACTIVE
#if LANGUAGE_ES_ACTIVE
    5,
#endif // LANGUAGE_ES_ACTIVE
#if LANGUAGE_SE_ACTIVE
    6,
#endif // LANGUAGE_SE_ACTIVE
#if LANGUAGE_FR_ACTIVE
    7,
#endif // LANGUAGE_FR_ACTIVE
#if LANGUAGE_CZ_ACTIVE
    8,
#endif // LANGUAGE_CZ_ACTIVE
#if LANGUAGE_PL_ACTIVE
    9,
#endif // LANGUAGE_PL_ACTIVE
    255
};

void Com::selectLanguage(fast8_t lang) {
    unsigned int pos = (unsigned int)&availableLanguages;
    uint8_t best = 255,cur;
    while((cur = HAL::readFlashByte((PGM_P)pos)) != 255) {
        if(best == 255 || cur == lang)
            best = cur;
        pos++;
    }
    Com::selectedLanguage = best;
}

const char* Com::translatedF(int textId) {
    PGM_P *adr = (PGM_P*)pgm_read_word(&translations[selectedLanguage]);
    return (const char *)pgm_read_word(&adr[textId]);
}

#endif

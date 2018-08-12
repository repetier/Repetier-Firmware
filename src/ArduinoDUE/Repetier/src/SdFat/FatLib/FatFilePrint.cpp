/**
 * Copyright (c) 20011-2017 Bill Greiman
 * This file is part of the SdFat library for SD memory cards.
 *
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */
#include "../../../Repetier.h"
#include <math.h>
#include "FatFile.h"
#include "FmtNumber.h"
#if SDSUPPORT
//------------------------------------------------------------------------------
// print uint8_t with width 2
static void print2u(uint8_t v) {
  char c0 = '?';
  char c1 = '?';
  if (v < 100) {
    c1 = v/10;
    c0 = v - 10*c1 + '0';
    c1 += '0';
  }
  Com::print(c1);
  Com::print(c0);
}
//------------------------------------------------------------------------------
static void printU32(uint32_t v) {
  char buf[11];
  char* ptr = buf + sizeof(buf);
  *--ptr = 0;
  Com::print(fmtDec(v, ptr));
}
//------------------------------------------------------------------------------
static void printHex(uint8_t w, uint16_t h) {
  char buf[5];
  char* ptr = buf + sizeof(buf);
  *--ptr = 0;
  for (uint8_t i = 0; i < w; i++) {
    char c = h & 0XF;
    *--ptr = c < 10 ? c + '0' : c + 'A' - 10;
    h >>= 4;
  }
  Com::print(ptr);
}
//------------------------------------------------------------------------------
void FatFile::dmpFile(uint32_t pos, size_t n) {
  char text[17];
  text[16] = 0;
  if (n >= 0XFFF0) {
    n = 0XFFF0;
  }
  if (!seekSet(pos)) {
    return;
  }
  for (size_t i = 0; i <= n; i++) {
    if ((i & 15) == 0) {
      if (i) {
        Com::print(' ');
        Com::print(text);
        if (i == n) {
          break;
        }
      }
      Com::println();
      if (i >= n) {
        break;
      }
      printHex(4, i);
      Com::print(' ');
    }
    int16_t h = read();
    if (h < 0) {
      break;
    }
    Com::print(' ');
    printHex(2, h);
    text[i&15] = ' ' <= h && h < 0X7F ? h : '.';
  }
  Com::println();
}
//------------------------------------------------------------------------------

/*
void FatFile::ls(uint8_t flags, uint8_t indent) {
  FatFile file;
  rewind();
  while (file.openNext(this, O_READ)) {
    // indent for dir level
    if (!file.isHidden() || (flags & LS_A)) {
      for (uint8_t i = 0; i < indent; i++) {
        Com::print(' ');
      }
      if (flags & LS_DATE) {
        file.printModifyDateTime();
        Com::print(' ');
      }
      if (flags & LS_SIZE) {
        file.printFileSize();
        Com::print(' ');
      }
      file.printName();
      if (file.isDir()) {
        Com::print('/');
      }
      Com::println();
      if ((flags & LS_R) && file.isDir()) {
        file.ls(flags, indent + 2);
      }
    }
    file.close();
  }
}
*/
extern int8_t RFstricmp(const char* s1, const char* s2);
extern int8_t RFstrnicmp(const char* s1, const char* s2, size_t n);

void FatFile::lsRecursive(uint8_t level, bool isJson)
{
  FatFile file;
#if JSON_OUTPUT
  bool firstFile = true;
#endif
  rewind();

  while (file.openNext(this, O_READ)) {
    file.getName(tempLongFilename, LONG_FILENAME_LENGTH);
    HAL::pingWatchdog();
    if(file.isHidden()) {
      file.close();
      continue;
    }
    // if (! (file.isFile() || file.isDir())) continue;
    if (strcmp(tempLongFilename, "..") == 0) {
      file.close();
      continue;
    }
    if (tempLongFilename[0] == '.') {
      file.close();
      continue; // MAC CRAP
    }
    if (file.isDir()) {
      if (level >= SD_MAX_FOLDER_DEPTH) {
        file.close();
        continue; // can't go deeper
      }
      if (level && !isJson) {
        Com::print(fullName);
        Com::printF(Com::tSlash);
      }
#if JSON_OUTPUT
      if (isJson) {
        if (!firstFile) Com::print(',');
	      Com::print('"');Com::print('*');
        SDCard::printEscapeChars(tempLongFilename);
	      Com::print('"');
        firstFile = false;
      } else {
        Com::print(tempLongFilename);
        Com::printFLN(Com::tSlash); // End with / to mark it as directory entry, so we can see empty directories.
      }
#else
      Com::print(tempLongFilename);
      Com::printFLN(Com::tSlash); // End with / to mark it as directory entry, so we can see empty directories.
#endif
      char *tmp;
      // Add directory name
      if(level) strcat(fullName, "/");
      strcat(fullName, tempLongFilename);
      if(!isJson) {
        file.lsRecursive(level + 1, false);
      }
      // remove added directory name
      if ((tmp = strrchr(fullName, '/')) != NULL)
        *tmp = 0;
      else
        *fullName = 0;
    } else { // is filename
      if(level && !isJson) {
        Com::print(fullName);
        Com::printF(Com::tSlash);
      }
#if JSON_OUTPUT
      if (isJson) {
        if (!firstFile) Com::printF(Com::tComma);
		    Com::print('"');
        SDCard::printEscapeChars(tempLongFilename);
		    Com::print('"');
        firstFile = false;
      } else
#endif
      {
        Com::print(tempLongFilename);
#if SD_EXTENDED_DIR
        Com::printF(Com::tSpace, (long) file.fileSize());
#endif
        Com::println();
      }
    }
    file.close();
  }
}

//------------------------------------------------------------------------------
/** List directory contents.
 *
 * \param[in] pr Print stream for list.
 *
 * \param[in] flags The inclusive OR of
 *
 * LS_DATE - %Print file modification date
 *
 * LS_SIZE - %Print file size.
 *
 * LS_R - Recursive list of subdirectories.
 *
 * \param[in] indent Amount of space before file name. Used for recursive
 * list to indicate subdirectory level.
 */
void FatFile::ls(uint8_t flags, uint8_t indent) {
  *fullName = 0;
  lsRecursive(0, false);
}

#if JSON_OUTPUT
void FatFile::lsJSON() {
  *fullName = 0;
  lsRecursive(0, true);
}
#endif


//------------------------------------------------------------------------------
bool FatFile::printCreateDateTime() {
  dir_t dir;
  if (!dirEntry(&dir)) {
    DBG_FAIL_MACRO;
    goto fail;
  }
  printFatDate(dir.creationDate);
  Com::print(' ');
  printFatTime(dir.creationTime);
  return true;

fail:
  return false;
}
//------------------------------------------------------------------------------
void FatFile::printFatDate(uint16_t fatDate) {
  printU32(FAT_YEAR(fatDate));
  Com::print('-');
  print2u(FAT_MONTH(fatDate));
  Com::print('-');
  print2u(FAT_DAY(fatDate));
}
//------------------------------------------------------------------------------
void FatFile::printFatTime(uint16_t fatTime) {
  print2u(FAT_HOUR(fatTime));
  Com::print(':');
  print2u(FAT_MINUTE(fatTime));
  Com::print(':');
  print2u(FAT_SECOND(fatTime));
}
//------------------------------------------------------------------------------
/** Template for FatFile::printField() */
template <typename Type>
static int printFieldT(FatFile* file, char sign, Type value, char term) {
  char buf[3*sizeof(Type) + 3];
  char* str = &buf[sizeof(buf)];

  if (term) {
    *--str = term;
    if (term == '\n') {
      *--str = '\r';
    }
  }
#ifdef OLD_FMT
  do {
    Type m = value;
    value /= 10;
    *--str = '0' + m - 10*value;
  } while (value);
#else  // OLD_FMT
  str = fmtDec(value, str);
#endif  // OLD_FMT
  if (sign) {
    *--str = sign;
  }
  return file->write(str, &buf[sizeof(buf)] - str);
}
//------------------------------------------------------------------------------

int FatFile::printField(float value, char term, uint8_t prec) {
  char buf[24];
  char* str = &buf[sizeof(buf)];
  if (term) {
    *--str = term;
    if (term == '\n') {
      *--str = '\r';
    }
  }
  str = fmtFloat(value, str, prec);
  return write(str, buf + sizeof(buf) - str);
}
//------------------------------------------------------------------------------
int FatFile::printField(uint16_t value, char term) {
  return printFieldT(this, 0, value, term);
}
//------------------------------------------------------------------------------
int FatFile::printField(int16_t value, char term) {
  char sign = 0;
  if (value < 0) {
    sign = '-';
    value = -value;
  }
  return printFieldT(this, sign, (uint16_t)value, term);
}
//------------------------------------------------------------------------------
int FatFile::printField(uint32_t value, char term) {
  return printFieldT(this, 0, value, term);
}
//------------------------------------------------------------------------------
int FatFile::printField(int32_t value, char term) {
  char sign = 0;
  if (value < 0) {
    sign = '-';
    value = -value;
  }
  return printFieldT(this, sign, (uint32_t)value, term);
}
//------------------------------------------------------------------------------
bool FatFile::printModifyDateTime() {
  dir_t dir;
  if (!dirEntry(&dir)) {
    DBG_FAIL_MACRO;
    goto fail;
  }
  printFatDate(dir.lastWriteDate);
  Com::print(' ');
  printFatTime(dir.lastWriteTime);
  return true;

fail:
  return false;
}
//------------------------------------------------------------------------------
void FatFile::printFileSize() {
  char buf[11];
  char *ptr = buf + sizeof(buf);
  *--ptr = 0;
  ptr = fmtDec(fileSize(), ptr);
  while (ptr > buf) {
    *--ptr = ' ';
  }
  Com::print(buf);
}
#endif
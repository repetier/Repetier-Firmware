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
    along with Foobar.  If not, see <http://www.gnu.org/licenses/>.

    This firmware is a nearly complete rewrite of the sprinter firmware
    by kliment (https://github.com/kliment/Sprinter)
    which based on Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
*/

#include "Reptier.h"
#include "ui.h"

#if SDSUPPORT

#ifndef SD_ALLOW_LONG_NAMES
#define SD_ALLOW_LONG_NAMES false
#endif

SDCard sd;

SDCard::SDCard() {
  sdmode = false;
  sdactive = false;
  savetosd = false;
    //power to SD reader
#if SDPOWER > -1
  SET_OUTPUT(SDPOWER); 
  WRITE(SDPOWER,HIGH);
#endif
#if defined(SDCARDDETECT) && SDCARDDETECT>-1
  SET_INPUT(SDCARDDETECT);
  WRITE(SDCARDDETECT,HIGH);
#endif
}
void SDCard::automount() {
#if defined(SDCARDDETECT) && SDCARDDETECT>-1
  if(READ(SDCARDDETECT) != SDCARDDETECTINVERTED) {
    if(sdactive) { // Card removed
      sdactive = false;
      savetosd = false;
      sdmode = false;
      UI_STATUS(UI_TEXT_SD_REMOVED);
      OUT_P_LN(UI_TEXT_SD_REMOVED);
    }
  } else {
    if(!sdactive) {
      UI_STATUS(UI_TEXT_SD_INSERTED);
      OUT_P_LN(UI_TEXT_SD_INSERTED);
      initsd();
    }
  }
#endif
}
void SDCard::initsd() {
  sdactive = false;
  #if SDSS >- 1
    /*if(dir[0].isOpen())
        dir[0].close();*/
    if(!fat.begin(SDSS,SPI_FULL_SPEED)) {
        OUT_P_LN("SD init fail");
        return;
    }
    fat.setStdOut(&out);
    sdactive = true;
  #endif
}
  
void SDCard::write_command(GCode *code) {
     unsigned int sum1=0,sum2=0; // for fletcher-16 checksum
      byte buf[100];
      byte p=2;
      file.writeError = false;
      int params = 128 | (code->params & ~1);
      *(int*)buf = params;
      if(GCODE_IS_V2(code)) { // Read G,M as 16 bit value
        *(int*)&buf[p] = code->params2;p+=2;
        if(GCODE_HAS_STRING(code))
          buf[p++] = strlen(code->text);
        if(code->params & 2) {*(int*)&buf[p] = code->M;p+=2;}
        if(code->params & 4) {*(int*)&buf[p]= code->G;p+=2;}
      } else {
        if(code->params & 2) {buf[p++] = (byte)code->M;}
        if(code->params & 4) {buf[p++] = (byte)code->G;}
      }
      if(code->params & 8) {*(float*)&buf[p] = code->X;p+=4;}
      if(code->params & 16) {*(float*)&buf[p] = code->Y;p+=4;}
      if(code->params & 32) {*(float*)&buf[p] = code->Z;p+=4;}
      if(code->params & 64) {*(float*)&buf[p] = code->E;p+=4;}
      if(code->params & 256) {*(float*)&buf[p] = code->F;p+=4;}
      if(code->params & 512) {buf[p++] = code->T;}
      if(code->params & 1024) {*(long int*)&buf[p] = code->S;p+=4;}
      if(code->params & 2048) {*(long int*)&buf[p] = code->P;p+=4;}
      if(code->params2 & 1) {*(float*)&buf[p] = code->I;p+=4;}
      if(code->params2 & 2) {*(float*)&buf[p] = code->J;p+=4;}
      if(GCODE_HAS_STRING(code)) { // read 16 byte into string
        char *sp = code->text;
        if(GCODE_IS_V2(code)) {
          byte i = strlen(code->text);
          for(;i;i--) buf[p++] = *sp++;
        } else {
          for(byte i=0;i<16;++i) buf[p++] = *sp++;
        }
      }
      byte *ptr = buf;
      byte len = p;
      while (len) {
        byte tlen = len > 21 ? 21 : len;
        len -= tlen;
        do {
          sum1 += *ptr++;
          if(sum1>=255) sum1-=255;
          sum2 += sum1;
          if(sum2>=255) sum2-=255;
        } while (--tlen);
      }
      buf[p++] = sum1;
      buf[p++] = sum2;
      file.write(buf,p);
      if (file.writeError){
          OUT_P_LN("error writing to file");
      }
}
char *SDCard::createFilename(char *buffer,const dir_t &p)
{
  char *pos = buffer,*src = (char*)p.name;
  for (byte i = 0; i < 11; i++,src++) 
  {   
    if (*src == ' ') continue;
    if (i == 8) 
      *pos++ = '.';
    *pos++ = *src;
  }
  *pos = 0;
  return pos;
}
bool SDCard::showFilename(const uint8_t *name) {
  if (*name == DIR_NAME_DELETED || *name == '.') return false;
#if !SD_ALLOW_LONG_NAMES
  byte i=11;
  while(i--) {
     if(*name=='~') return false;
     name++;
  }
#endif
  return true; 
}
void  SDCard::lsRecursive(SdBaseFile *parent,byte level)
{
  dir_t *p;
  uint8_t cnt=0;
  char *oldpathend = pathend;
  char filename[13];
  parent->rewind();
  while ((p= parent->readDirCache()))
  {
    if (p->name[0] == DIR_NAME_FREE) break;
    if (!showFilename(p->name)) continue;
    if (!DIR_IS_FILE_OR_SUBDIR(p)) continue;
    if( DIR_IS_SUBDIR(p))
    {
      if(level>=SD_MAX_FOLDER_DEPTH) continue; // can't go deeper
      if(level<SD_MAX_FOLDER_DEPTH) {
        createFilename(filename,*p);
        if(level) {
          out.print(fullName);
          out.print('/');
        }
        out.print(filename);
        out.println('/'); //End with / to mark it as directory entry, so we can see empty directories.
      }
      char *tmp = oldpathend;
      if(level) *tmp++ = '/';
      char *dirname = tmp;
      pathend = createFilename(tmp,*p);
      SdBaseFile next;
      uint16_t index = parent->curPosition()/32 - 1;
      if(next.open(parent,dirname, O_READ))
        lsRecursive(&next,level+1);
      parent->seekSet(32 * (index + 1));
      *oldpathend = 0;
    } else {
      createFilename(filename,*p);
      if(level) {
        out.print(fullName);
        out.print('/');
      }
      out.print(filename);
#ifdef SD_EXTENDED_DIR
      out.write(' ');
      out.print(p->fileSize);
#endif
      out.println();
    }
  }
}
void SDCard::ls() {
  OUT_P_LN("Begin file list");
  *fullName = 0;
  pathend = fullName;
  fat.chdir();
  lsRecursive(fat.vwd(),0);
  OUT_P_LN("End file list");
}

void SDCard::selectFile(char *filename) {
  if(!sdactive) return;
  sdmode = false;
  file.close();
  fat.chdir();
  if (file.open(fat.vwd(),filename, O_READ)) {
    OUT_P("File opened:");
    out.print(filename);
    OUT_P_L_LN(" Size:",file.fileSize());
    sdpos = 0;
    filesize = file.fileSize();
    OUT_P_LN("File selected");
  } else {
    OUT_P_LN("file.open failed");
  }
}
void SDCard::printStatus() {
  if(sdactive){
    OUT_P_L("SD printing byte ",sdpos);
    OUT_P_L_LN("/",filesize);
  } else {
    OUT_P_LN("Not SD printing");
  }
}
void SDCard::startWrite(char *filename) {
  if(!sdactive) return;
  file.close();
  sdmode = false;
  fat.chdir();
  if(!file.open(filename, O_CREAT | O_APPEND | O_WRITE | O_TRUNC))  {
    OUT_P("open failed, File: ");
    out.print(filename);
    out.print('.');
  } else {
    UI_STATUS(UI_TEXT_UPLOADING);
    savetosd = true;
    OUT_P("Writing to file: ");
    out.println(filename);
  }
}
void SDCard::finishWrite() {
  if(!savetosd) return; // already closed or never opened
  file.sync();
  file.close();
  savetosd = false;
  OUT_P_LN("Done saving file.");
  UI_CLEAR_STATUS;
}
void SDCard::deleteFile(char *filename) {
  if(!sdactive) return;
  sdmode = false;
  file.close();
  if(fat.remove(filename)) {
    OUT_P_LN("File deleted");
  } else {
    if(fat.rmdir(filename))
      OUT_P_LN("File deleted");
    else
      OUT_P_LN("Deletion failed");
  }
}
void SDCard::makeDirectory(char *filename) {
  if(!sdactive) return;
  sdmode = false;
  file.close();
  if(fat.mkdir(filename)) {
    OUT_P_LN("Directory created");    
  } else {
    OUT_P_LN("Creation failed");
  }
}
#endif


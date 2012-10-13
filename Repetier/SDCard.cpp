#include "Reptier.h"
#include "ui.h"

#ifdef SDSUPPORT

  SDCard sd;

  Sd2Card card; // ~14 Byte
  SdVolume volume;
  SdFile file;
  uint32_t filesize = 0;
  uint32_t sdpos = 0;
  bool sdmode = false;
  bool sdactive = false;
  bool savetosd = false;
  int16_t n;

SDCard::SDCard() {
  sdmode = false;
  sdactive = false;
  savetosd = false;
    //power to SD reader
#if SDPOWER > -1
  pinMode(SDPOWER,OUTPUT); 
  digitalWrite(SDPOWER,HIGH);
#endif

}
void SDCard::initsd() {
  sdactive = false;
  #if SDSS >- 1
    if(dir[0].isOpen())
        dir[0].close();
    if (!card.init(SPI_FULL_SPEED,SDSS)){
        //if (!card.init(SPI_HALF_SPEED,SDSS))
        OUT_P_LN("SD init fail");
    }
    else if (!volume.init(&card)) {
          OUT_P_LN("volume.init failed");
    } else if (!dir[0].openRoot(&volume)) 
       OUT_P_LN("openRoot failed");
    else 
       sdactive = true;
  #endif
}
  
void SDCard::write_command(GCode *code) {
     unsigned int sum1=0,sum2=0; // for fletcher-16 checksum
      byte buf[52];
      byte p=2;
      file.writeError = false;
      int params = 128 | (code->params & ~1);
      *(int*)buf = params;      
      if(code->params & 2) {buf[p++] = code->M;}
      if(code->params & 4) {buf[p++] = code->G;}
      if(code->params & 8) {*(float*)&buf[p] = code->X;p+=4;}
      if(code->params & 16) {*(float*)&buf[p] = code->Y;p+=4;}
      if(code->params & 32) {*(float*)&buf[p] = code->Z;p+=4;}
      if(code->params & 64) {*(float*)&buf[p] = code->E;p+=4;}
      if(code->params & 256) {*(float*)&buf[p] = code->F;p+=4;}
      if(code->params & 512) {buf[p++] = code->T;}
      if(code->params & 1024) {*(long int*)&buf[p] = code->S;p+=4;}
      if(code->params & 2048) {*(long int*)&buf[p] = code->P;p+=4;}
      if(GCODE_HAS_STRING(code)) { // read 16 byte into string
       char *sp = code->text;
       for(byte i=0;i<16;++i) buf[p++] = *sp++;
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
void  SDCard::lsRecursive(byte level)
{
  dir_t p;
  uint8_t cnt=0;
  char *oldpathend = pathend;
  char filename[13];
  while (dir[level].readDir(p) > 0)
  {
    if (p.name[0] == DIR_NAME_FREE) break;
    if (p.name[0] == DIR_NAME_DELETED || p.name[0] == '.') continue;
    if (!DIR_IS_FILE_OR_SUBDIR(&p)) continue;
    if( DIR_IS_SUBDIR(&p))
    {
      if(level>SD_MAX_FOLDER_DEPTH) continue; // can't go deeper
      if(level<SD_MAX_FOLDER_DEPTH) {
        createFilename(filename,p);
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
      pathend = createFilename(pathend,p);
      dir[level+1].close();
      if(!dir[level+1].open(dir[level],dirname, O_READ))
        continue; // Skip dir if error
      lsRecursive(level+1);
    } else {
      createFilename(filename,p);
      if(level) {
        out.print(fullName);
        out.print('/');
      }
      out.print(filename);
#ifdef SD_EXTENDED_DIR
      out.write(' ');
      out.print(p.fileSize);
#endif
      out.println();
    }
  }
}
void SDCard::ls() {
  OUT_P_LN("Begin file list");
  *fullName = 0;
  pathend = fullName;
  dir[0].rewind();
  lsRecursive(0);
  OUT_P_LN("End file list");
}

void SDCard::selectFile(char *filename) {
  if(!sdactive) return;
  sdmode = false;
  file.close();
  SdFile *dirfile = getDirectory(filename);
  if(dirfile==NULL) {
    OUT_ERROR_P_LN("Invalid directory");
    return;
  }
  if (file.open(dirfile, shortname, O_READ)) {
    OUT_P("File opened:");
    out.print(filename);
    OUT_P(" Size:");
    out.println(file.fileSize());
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
  SdFile *dirfile = getDirectory(filename);
  if(dirfile==NULL) {
    OUT_ERROR_P_LN("Invalid directory");
    return;
  }
  if(!file.open(dirfile,shortname, O_CREAT | O_APPEND | O_WRITE | O_TRUNC))  {
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
  SdFile *dirfile = getDirectory(filename);
  if(dirfile==NULL) {
    OUT_ERROR_P_LN("Invalid directory");
    return;
  }
  if(SdFile::remove(dirfile, shortname)) {
    OUT_P_LN("File deleted");
  } else {
    SdFile rdir;
    rdir.open(dirfile,shortname,O_READ);
    if(rdir.rmDir())
      OUT_P_LN("File deleted");
    else
      OUT_P_LN("Deletion failed");
  }
}
void SDCard::makeDirectory(char *filename) {
  if(!sdactive) return;
  OUT_P("Creat directory ");out.println(filename);
  sdmode = false;
  file.close();
  SdFile *dirfile = getDirectory(filename);
  if(dirfile==NULL) {
    OUT_ERROR_P_LN("Invalid directory");
    return;
  }
  SdFile newdir;
  if(newdir.makeDir(dirfile, shortname)) {
    OUT_P_LN("Directory created");
  } else {
    OUT_P_LN("Creation failed");
  }
}
/**
Get the directory SdFile containing name. Also sets shortname to the beginning of the
filename in the target directory.
*/
SdFile *SDCard::getDirectory(char* name)
{
  if(!sdactive) return NULL;
  byte level = 0; 
  char *searchStart = name; 
  char *dirend;
  dir[0].rewind();
  do {
    OUT_P("Dir:");out.println(searchStart);
    dirend = strchr(searchStart,'/');
    if(dirend==NULL) {
      shortname = searchStart;
      OUT_P_I_LN("Level:",level);
      OUT_P("Shortname: ");out.println(shortname);
      return &dir[level];
    }
    char subdirname[13];
    level++;
    if(level>SD_MAX_FOLDER_DEPTH) return NULL; // Not allowed nesting level
    dir[level].close(); // might be open from earlier access
    byte len=dirend-searchStart;
    strncpy(subdirname,searchStart,len);
    subdirname[len]=0;
    OUT_P("Sub:");out.println(subdirname);
    if(!dir[level].open(dir[level-1],subdirname,O_READ)) return NULL; // Unexpected error
    searchStart = dirend+1;
  } while(1);
}
#endif


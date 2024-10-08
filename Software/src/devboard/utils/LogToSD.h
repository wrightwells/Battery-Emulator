#ifndef LOG_TO_SD_H
#define LOG_TO_SD_H

//#include "../../include.h"

#include <FS.h>
#include "../../lib/SD/src/SD.h"

void setupLogToSD(const String &BatteryName);

void listDir(fs::FS &fs, const char *dirname, uint8_t levels);
void createDir(fs::FS &fs, const char *path);
void removeDir(fs::FS &fs, const char *path);
void readFile(fs::FS &fs, const char *path);
void writeFile(fs::FS &fs, const char *path, const char *message);
void appendFile(fs::FS &fs, const char *path, const char *message);
void renameFile(fs::FS &fs, const char *path1, const char *path2);
void deleteFile(fs::FS &fs, const char *path);

void addToBuffer(const String &logData);
void flushBufferToSD();

#endif // LOG_TO_SD_H

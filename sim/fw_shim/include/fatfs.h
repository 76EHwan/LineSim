/* Stand-in for the CubeMX FATFS/App/fatfs.h: the file-system objects the firmware shares. */
#ifndef SIM_FATFS_H
#define SIM_FATFS_H

#include "ff.h"

#ifdef __cplusplus
extern "C" {
#endif

extern uint8_t retSD;
extern char SDPath[4];
extern FATFS SDFatFS;
extern FIL SDFile;
extern FATFS SDFatFS_NC;
extern FIL SDFile_NC;
extern FILINFO fno;

void MX_FATFS_Init(void);

#ifdef __cplusplus
}
#endif
#endif /* SIM_FATFS_H */

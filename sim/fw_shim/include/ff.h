/*
 * FatFs API (same names, codes and flags as ChaN's FatFs R0.12+) backed by a
 * folder on the host PC that plays the role of the SD card.
 */
#ifndef SIM_FF_H
#define SIM_FF_H

#include "sim_compat.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef unsigned int   UINT;
typedef unsigned char  BYTE;
typedef uint16_t       WORD;
typedef uint32_t       DWORD;
typedef uint64_t       QWORD;
typedef char           TCHAR;
typedef DWORD          FSIZE_t;
typedef DWORD          LBA_t;

typedef enum {
	FR_OK = 0, FR_DISK_ERR, FR_INT_ERR, FR_NOT_READY, FR_NO_FILE, FR_NO_PATH, FR_INVALID_NAME,
	FR_DENIED, FR_EXIST, FR_INVALID_OBJECT, FR_WRITE_PROTECTED, FR_INVALID_DRIVE, FR_NOT_ENABLED,
	FR_NO_FILESYSTEM, FR_MKFS_ABORTED, FR_TIMEOUT, FR_LOCKED, FR_NOT_ENOUGH_CORE,
	FR_TOO_MANY_OPEN_FILES, FR_INVALID_PARAMETER
} FRESULT;

typedef struct {
	BYTE  fs_type;
	WORD  csize;
	DWORD n_fatent;
	DWORD free_clst;
	int   sim_mounted;
} FATFS;

typedef struct {
	void   *sim_fp;   /* host FILE* */
	BYTE    flag;
	FSIZE_t fptr;
	FSIZE_t obj_size;
} FIL;

typedef struct { void *sim_handle; } DIR;

typedef struct {
	FSIZE_t fsize;
	WORD    fdate;
	WORD    ftime;
	BYTE    fattrib;
	TCHAR   altname[13];
	TCHAR   fname[256];
} FILINFO;

#define FA_READ          0x01
#define FA_WRITE         0x02
#define FA_OPEN_EXISTING 0x00
#define FA_CREATE_NEW    0x04
#define FA_CREATE_ALWAYS 0x08
#define FA_OPEN_ALWAYS   0x10
#define FA_OPEN_APPEND   0x30

#define AM_RDO 0x01
#define AM_HID 0x02
#define AM_SYS 0x04
#define AM_DIR 0x10
#define AM_ARC 0x20

FRESULT f_mount(FATFS *fs, const TCHAR *path, BYTE opt);
FRESULT f_open(FIL *fp, const TCHAR *path, BYTE mode);
FRESULT f_close(FIL *fp);
FRESULT f_read(FIL *fp, void *buff, UINT btr, UINT *br);
FRESULT f_write(FIL *fp, const void *buff, UINT btw, UINT *bw);
FRESULT f_lseek(FIL *fp, FSIZE_t ofs);
FRESULT f_sync(FIL *fp);
FRESULT f_truncate(FIL *fp);
FRESULT f_stat(const TCHAR *path, FILINFO *fno);
FRESULT f_unlink(const TCHAR *path);
FRESULT f_mkdir(const TCHAR *path);
FRESULT f_rename(const TCHAR *path_old, const TCHAR *path_new);
FRESULT f_getfree(const TCHAR *path, DWORD *nclst, FATFS **fatfs);
FRESULT f_opendir(DIR *dp, const TCHAR *path);
FRESULT f_closedir(DIR *dp);
FRESULT f_readdir(DIR *dp, FILINFO *fno);
int     f_puts(const TCHAR *str, FIL *cp);
int     f_printf(FIL *fp, const TCHAR *str, ...);
TCHAR  *f_gets(TCHAR *buff, int len, FIL *fp);

#define f_eof(fp)  ((int)((fp)->fptr == (fp)->obj_size))
#define f_error(fp) (0)
#define f_tell(fp) ((fp)->fptr)
#define f_size(fp) ((fp)->obj_size)
#define f_rewind(fp) f_lseek((fp), 0)

#ifdef __cplusplus
}
#endif
#endif /* SIM_FF_H */

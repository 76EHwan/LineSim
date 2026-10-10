/*
 * FatFs on top of a host folder (sim_sd_root()). Paths like "0:/Drive_Data/x.txt",
 * "/Drive_Data/x.txt" and "x.txt" all resolve inside that folder.
 */
#include "ff.h"
#include "fatfs.h"
#include "sim_mcu.h"

#include <sys/stat.h>
#include <io.h>
#include <direct.h>
#include <errno.h>

uint8_t retSD;
char SDPath[4] = "0:/";
FATFS SDFatFS;
FIL SDFile;
FATFS SDFatFS_NC;
FIL SDFile_NC;
FILINFO fno;

void MX_FATFS_Init(void) {}

static int g_mounted = 0;

static void host_path(const TCHAR *path, char *out, size_t n) {
	const char *p = path ? path : "";
	if (p[0] && p[1] == ':')    /* drive number prefix "0:" */
		p += 2;
	for (; *p == '/' || *p == '\\'; p++)
		;
	snprintf(out, n, "%s/%s", sim_sd_root(), p);
	for (char *c = out; *c; c++)
		if (*c == '\\')
			*c = '/';
}

static int host_is_dir(const char *hp) {
	struct _stat64 st;
	return _stat64(hp, &st) == 0 && (st.st_mode & _S_IFDIR);
}

static int parent_exists(const char *hp) {
	char buf[512];
	snprintf(buf, sizeof buf, "%s", hp);
	char *slash = strrchr(buf, '/');
	if (!slash)
		return 1;
	*slash = '\0';
	return host_is_dir(buf);
}

FRESULT f_mount(FATFS *fs, const TCHAR *path, BYTE opt) {
	(void) path; (void) opt;
	if (fs) {
		fs->sim_mounted = 1;
		fs->csize = 64;
		fs->n_fatent = 1u << 20;
		fs->free_clst = 1u << 19;
		g_mounted = 1;
	} else {
		g_mounted = 0;
	}
	return FR_OK;
}

FRESULT f_open(FIL *fp, const TCHAR *path, BYTE mode) {
	memset(fp, 0, sizeof *fp);
	if (!g_mounted)
		return FR_NOT_ENABLED;
	char hp[512];
	host_path(path, hp, sizeof hp);
	if (!parent_exists(hp))
		return FR_NO_PATH;
	if (host_is_dir(hp))
		return FR_DENIED;

	struct _stat64 st;
	int exists = _stat64(hp, &st) == 0;
	const char *fm;
	if (mode & FA_CREATE_NEW) {
		if (exists)
			return FR_EXIST;
		fm = "w+b";
	} else if (mode & FA_CREATE_ALWAYS) {
		fm = "w+b";
	} else if ((mode & FA_OPEN_ALWAYS) && !exists) {
		fm = "w+b";
	} else {
		if (!exists)
			return FR_NO_FILE;
		fm = (mode & FA_WRITE) ? "r+b" : "rb";
	}
	FILE *f = fopen(hp, fm);
	if (!f)
		return FR_DENIED;
	fp->sim_fp = f;
	fp->flag = mode;
	fseek(f, 0, SEEK_END);
	fp->obj_size = (FSIZE_t) ftell(f);
	if ((mode & FA_OPEN_APPEND) == FA_OPEN_APPEND) {
		fp->fptr = fp->obj_size;
	} else {
		fseek(f, 0, SEEK_SET);
		fp->fptr = 0;
	}
	return FR_OK;
}

FRESULT f_close(FIL *fp) {
	if (!fp->sim_fp)
		return FR_INVALID_OBJECT;
	fclose((FILE*) fp->sim_fp);
	fp->sim_fp = NULL;
	return FR_OK;
}

FRESULT f_read(FIL *fp, void *buff, UINT btr, UINT *br) {
	*br = 0;
	if (!fp->sim_fp)
		return FR_INVALID_OBJECT;
	if (!(fp->flag & FA_READ))
		return FR_DENIED;
	size_t n = fread(buff, 1, btr, (FILE*) fp->sim_fp);
	*br = (UINT) n;
	fp->fptr += (FSIZE_t) n;
	return FR_OK;
}

FRESULT f_write(FIL *fp, const void *buff, UINT btw, UINT *bw) {
	*bw = 0;
	if (!fp->sim_fp)
		return FR_INVALID_OBJECT;
	if (!(fp->flag & FA_WRITE))
		return FR_DENIED;
	size_t n = fwrite(buff, 1, btw, (FILE*) fp->sim_fp);
	*bw = (UINT) n;
	fp->fptr += (FSIZE_t) n;
	if (fp->fptr > fp->obj_size)
		fp->obj_size = fp->fptr;
	return n == btw ? FR_OK : FR_DISK_ERR;
}

FRESULT f_lseek(FIL *fp, FSIZE_t ofs) {
	if (!fp->sim_fp)
		return FR_INVALID_OBJECT;
	fseek((FILE*) fp->sim_fp, (long) ofs, SEEK_SET);
	fp->fptr = ofs;
	return FR_OK;
}

FRESULT f_sync(FIL *fp) {
	if (!fp->sim_fp)
		return FR_INVALID_OBJECT;
	fflush((FILE*) fp->sim_fp);
	return FR_OK;
}

FRESULT f_truncate(FIL *fp) {
	if (!fp->sim_fp)
		return FR_INVALID_OBJECT;
	fflush((FILE*) fp->sim_fp);
	_chsize_s(_fileno((FILE*) fp->sim_fp), (long long) fp->fptr);
	fp->obj_size = fp->fptr;
	return FR_OK;
}

FRESULT f_stat(const TCHAR *path, FILINFO *fno) {
	if (!g_mounted)
		return FR_NOT_ENABLED;
	char hp[512];
	host_path(path, hp, sizeof hp);
	struct _stat64 st;
	if (_stat64(hp, &st) != 0)
		return parent_exists(hp) ? FR_NO_FILE : FR_NO_PATH;
	if (fno) {
		memset(fno, 0, sizeof *fno);
		fno->fsize = (FSIZE_t) st.st_size;
		fno->fattrib = (st.st_mode & _S_IFDIR) ? AM_DIR : AM_ARC;
		const char *name = strrchr(hp, '/');
		snprintf(fno->fname, sizeof fno->fname, "%s", name ? name + 1 : hp);
	}
	return FR_OK;
}

FRESULT f_unlink(const TCHAR *path) {
	if (!g_mounted)
		return FR_NOT_ENABLED;
	char hp[512];
	host_path(path, hp, sizeof hp);
	if (!parent_exists(hp))
		return FR_NO_PATH;
	if (host_is_dir(hp))
		return _rmdir(hp) == 0 ? FR_OK : FR_DENIED;
	if (remove(hp) != 0)
		return errno == ENOENT ? FR_NO_FILE : FR_DENIED;
	return FR_OK;
}

FRESULT f_mkdir(const TCHAR *path) {
	if (!g_mounted)
		return FR_NOT_ENABLED;
	char hp[512];
	host_path(path, hp, sizeof hp);
	struct _stat64 st;
	if (_stat64(hp, &st) == 0)
		return FR_EXIST;
	if (!parent_exists(hp))
		return FR_NO_PATH;
	return _mkdir(hp) == 0 ? FR_OK : FR_DENIED;
}

FRESULT f_rename(const TCHAR *path_old, const TCHAR *path_new) {
	if (!g_mounted)
		return FR_NOT_ENABLED;
	char a[512], b[512];
	host_path(path_old, a, sizeof a);
	host_path(path_new, b, sizeof b);
	return rename(a, b) == 0 ? FR_OK : FR_NO_FILE;
}

FRESULT f_getfree(const TCHAR *path, DWORD *nclst, FATFS **fatfs) {
	(void) path;
	if (!g_mounted)
		return FR_NOT_ENABLED;
	*nclst = SDFatFS_NC.free_clst ? SDFatFS_NC.free_clst : (1u << 19);
	if (fatfs)
		*fatfs = &SDFatFS_NC;
	return FR_OK;
}

FRESULT f_opendir(DIR *dp, const TCHAR *path) { (void) path; dp->sim_handle = NULL; return FR_NO_PATH; }
FRESULT f_closedir(DIR *dp) { (void) dp; return FR_OK; }
FRESULT f_readdir(DIR *dp, FILINFO *fno) { (void) dp; fno->fname[0] = 0; return FR_OK; }

int f_puts(const TCHAR *str, FIL *cp) {
	UINT bw;
	UINT n = (UINT) strlen(str);
	return f_write(cp, str, n, &bw) == FR_OK ? (int) bw : -1;
}

int f_printf(FIL *fp, const TCHAR *str, ...) {
	char buf[1024];
	va_list ap;
	va_start(ap, str);
	int n = vsnprintf(buf, sizeof buf, str, ap);
	va_end(ap);
	if (n < 0)
		return -1;
	return f_puts(buf, fp);
}

TCHAR* f_gets(TCHAR *buff, int len, FIL *fp) {
	if (!fp->sim_fp)
		return NULL;
	char *r = fgets(buff, len, (FILE*) fp->sim_fp);
	if (r)
		fp->fptr = (FSIZE_t) ftell((FILE*) fp->sim_fp);
	return r;
}

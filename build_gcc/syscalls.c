#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/stat.h>
#include <unistd.h>

extern char _end;

static char *heap_end;

static char *current_stack_pointer(void)
{
    char *sp;
    __asm volatile ("mrs %0, msp" : "=r" (sp));
    return sp;
}

void *_sbrk(ptrdiff_t incr)
{
    char *prev_heap_end;

    if (heap_end == NULL) {
        heap_end = &_end;
    }

    prev_heap_end = heap_end;
    if ((heap_end + incr) > current_stack_pointer()) {
        errno = ENOMEM;
        return (void *)-1;
    }

    heap_end += incr;
    return prev_heap_end;
}

int _write(int file, char *ptr, int len)
{
    (void)file;
    (void)ptr;
    return len;
}

int _read(int file, char *ptr, int len)
{
    (void)file;
    (void)ptr;
    (void)len;
    return 0;
}

int _close(int file)
{
    (void)file;
    errno = EBADF;
    return -1;
}

int _fstat(int file, struct stat *st)
{
    (void)file;
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int file)
{
    (void)file;
    return 1;
}

int _lseek(int file, int ptr, int dir)
{
    (void)file;
    (void)ptr;
    (void)dir;
    return 0;
}

int _kill(int pid, int sig)
{
    (void)pid;
    (void)sig;
    errno = EINVAL;
    return -1;
}

int _getpid(void)
{
    return 1;
}

void _exit(int status)
{
    (void)status;
    __asm volatile ("cpsid i");
    while (1) {
    }
}

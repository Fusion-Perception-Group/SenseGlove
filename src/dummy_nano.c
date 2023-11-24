/**
 * @file nano_dummy.c
 * @brief Get rid of stupid warnings from newlib nano
 * @version 0.1
 * @date 2023-10-29
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifdef __cplusplus
extern "C"
{
#endif
#define __weak __attribute__((weak))
    __weak void _close()
    {
    }
    __weak void _lseek()
    {
    }
    __weak void _read()
    {
    }
    __weak void _write()
    {
    }
    __weak void _fstat_r()
    {
    }
    __weak void _isatty_r()
    {
    }
    __weak void _getpid_r()
    {
    }
    __weak void _kill_r()
    {
    }
#ifdef __cplusplus
}
#endif
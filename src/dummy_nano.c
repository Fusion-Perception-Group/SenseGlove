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
    void _close()
    {
    }
    void _lseek()
    {
    }
    void _read()
    {
    }
    void _write()
    {
    }
    void _fstat_r()
    {
    }
    void _isatty_r()
    {
    }
    void _getpid_r()
    {
    }
    void _kill_r()
    {
    }
#ifdef __cplusplus
}
#endif
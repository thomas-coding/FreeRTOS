/* For GCC compiler revise _write() function for printf functionality */

extern void uart_putc(char c);
int _write(int file, char *ptr, int len)
{
    int i;
    file = file;
    for (i = 0; i < len; i++)
    {
        uart_putc(*ptr++);
    }
    return len;
}

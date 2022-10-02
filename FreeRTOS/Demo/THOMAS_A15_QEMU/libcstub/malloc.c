/* Defining malloc/free should overwrite the
standard versions provided by the compiler. */

/* I don't know if use newlib malloc, what need to do, so now used freertos malloc */
#include <stddef.h>

void malloc (size_t size)
{
    /* Call the FreeRTOS version of malloc. */
    return pvPortMalloc( size );
}
void free (void* ptr)
{
    /* Call the FreeRTOS version of free. */
    vPortFree( ptr );
}

#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include "log.h"

int log_puts(char *fmt, ...)
{
    int i, d, ret, len, j;
    char c, *s;
    uint8_t *hbuf;
    double f;
    char strfmt[10];
    va_list ap;

    va_start(ap, fmt);

    if(fmt == NULL){
        return 0;
    }

    i = 0;
    while(*fmt){
        if(*fmt == '%'){
            strfmt[0] = '%';
            j=1;
            while( ( fmt[j]>='0' && fmt[j]<='9' ) ||
                  ( fmt[j]== '-' ) || ( fmt[j]== '+' ) || ( fmt[j]== '.' ) ){
                strfmt[j] = fmt[j];
                j++;
            }
            strfmt[j] = fmt[j];
            fmt += j;
            j++;
            strfmt[j] = '\0';

            switch(*fmt){
            case '%':
                ret = printf(strfmt);
                i+=ret;
                break;
            case 'd':
                d = va_arg(ap, int);
                ret = printf(strfmt, d);
                i+=ret;
                break;
            case 'u':
                d = va_arg(ap, int);
                ret = printf(strfmt, (uint32_t)d);
                i+=ret;
                break;
            case 'x':
            case 'X':
                d = va_arg(ap, int);
                ret = printf(strfmt, d);
                i+=ret;
                break;
            case 'h':
            case 'H':
                hbuf = va_arg(ap, uint8_t *);
                len = va_arg(ap, int);
                for(d=0; d<len; d++){
                    if(*fmt == 'h'){
                        ret = printf("%02X", hbuf[d]);
                    }else{
                        ret = printf("%02X ", hbuf[d]);
                    }
                    i+=ret;
                }
                break;
            case 's':
                s = va_arg(ap, char *);
                ret = printf(strfmt, s);
                i+=ret;
                break;
            case 'c':
                c = (char)va_arg(ap, int);
                ret = printf(strfmt, c);
                i+=ret;
                break;
            case 'f':
                f = va_arg(ap, double);
                ret = printf(strfmt, f);
                i+=ret;
                break;
            }
            fmt++;
        }else{
            fputc(*fmt++, stdout);
            i++;
        }
    }
    printf("\n");
    i++;

    va_end(ap);

    return i;
}

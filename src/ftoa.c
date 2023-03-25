#include <stdio.h>
#include <stdlib.h>

float ftoa_1dec(float f, char * a)
{
    int d;
    int i;
    // f = 9.467
    d = (int)(f * 100.0f);// f in integer centi units : 946
    d = (d+5) / 10;       // f rounded in integer deci units : 951 / 10 = 95
    i = d / 10;           // integer units : 95 / 10 = 9
    d = d - (i*10);       // 1 digit fraction : 95 - 90 = 5
    sprintf( a, "%d.%1d", i, d); // 9.5
    return ( atof( a ) );
}

float ftoa_2dec(float f, char * a)
{
    int d;
    int i;
    // f = 9.467
    d = (int)(f * 1000.0f);// f in integer milli units : 9467
    d = (d+5) / 10;        // f rounded in integer centi units : 9472 / 10 = 947
    i = d / 100;           // integer units : 947 / 100 = 9
    d = d - (i*100);       // 2 digit fraction : 947 - 900 = 47
    sprintf( a, "%d.%02d", i, d); // 9.47
    return ( atof( a ) );
}


#ifndef MISC_H
#define MISC_H

#include "fcntl.h"
#include "sys/ioctl.h"
#include "termios.h"

int kbhit_and_getch()
{
    static bool initialized = false;
    static struct termios old_attribute;
    static struct termios new_attribute;
    static int old_f;
    static int new_f;

    if (!initialized)
    {
        initialized = true;

        tcgetattr(0, &old_attribute);
        tcgetattr(0, &new_attribute);
        new_attribute.c_lflag &= ~ICANON;
        new_attribute.c_lflag &= ~ECHO;

        old_f = fcntl(0, F_GETFL, 0);
        new_f = fcntl(0, F_GETFL, 0);
        new_f |= O_NONBLOCK;
    }

    tcsetattr(0, TCSANOW, &new_attribute);
    fcntl(0, F_SETFL, new_f);

    int ch = getchar();

    tcsetattr(0, TCSANOW, &old_attribute);
    fcntl(0, F_SETFL, old_f);

    if (ch != EOF)
    {
        return ch;
    }
    else
    {
        return -1;
    }
}

#endif
#include "Message.h"

#include <stdio.h>
#include <stdlib.h>

static int               msg_level;
static inv_msg_printer_t msg_printer;

void inv_msg_printer_default(int level, const char * str, va_list ap)
{
	(void)level, (void)str, (void)ap;
}

void inv_msg_setup(int level, inv_msg_printer_t printer)
{
	msg_level   = level;
	if (level < INV_MSG_LEVEL_OFF)
		msg_level = INV_MSG_LEVEL_OFF;
	else if (level > INV_MSG_LEVEL_MAX)
		msg_level = INV_MSG_LEVEL_MAX;
	msg_printer = printer;
}

void inv_msg(int level, const char * str, ...)
{
	if(level && level <= msg_level && msg_printer) {
		va_list ap;
		va_start(ap, str);
		msg_printer(level, str, ap);
		va_end(ap);
	}
}

int inv_msg_get_level(void)
{
	return msg_level;
}

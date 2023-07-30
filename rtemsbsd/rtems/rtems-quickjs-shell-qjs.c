#include <machine/rtems-bsd-user-space.h>
#include <machine/rtems-bsd-program.h>

#include <rtems/quickjscmds-config.h>

#include <machine/rtems-bsd-commands.h>

//#include <machine/quickjscmds-config.h>
//#include "rtems-quickjs-commands.h"

// hello

rtems_shell_cmd_t rtems_shell_QJS_Command = {
  "qjs",                        /* name */
  "qjs [args]",                 /* usage */
  "net",                        /* topic */
  rtems_quickjs_command_qjs,    /* command */
  NULL,                         /* alias */
  NULL                          /* next */
};

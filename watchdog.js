Components.utils.import('resource://gre/modules/ctypes.jsm');

try { libc.declare; } catch (_) { var libc = ctypes.open('libc.so.6'); }

var libc_alarm = libc.declare('alarm', ctypes.default_abi, ctypes.unsigned_int,
                              ctypes.unsigned_int);

function watchdog_keepalive(timeout)
{
  if (!timeout) timeout = 10;
  libc_alarm(timeout);
}

function watchdog_disable()
{
  libc_alarm(0);
}

// we'll use plenty ctypes
Components.utils.import('resource://gre/modules/ctypes.jsm');
// DEBUG turns off exceptions on serial settings
// DEBUG=true allows test cases with regular files
// DEBUG=false/undefined enforces access to serial ports only
const DEBUG=true;

/* ctypes.jsm types:
   bool short unsigned_short int unsigned_int
   long unsigned_long long_long unsigned_long_long
   float double char signed_char unsigned_char
   size_t ssize_t intptr_t uintptr_t jschar void_t voidptr_t
   Int64 UInt64 */

// as used in C headers
const speed_t = ctypes.unsigned_int;
const tcflag_t = ctypes.unsigned_int;
const cc_t = ctypes.unsigned_char;

// serial consts
const struct_termios = new ctypes.StructType(
  'struct_termios',
  [{c_iflag: tcflag_t},
   {c_oflag: tcflag_t},
   {c_cflag: tcflag_t},
   {c_lflag: tcflag_t},
   {c_line: cc_t},
   {c_cc: cc_t.array(32)},
   {c_ispeed: speed_t},
   {c_ospeed: speed_t}]);

const O_RDWR = 2;
const O_NOCTTY = 256;

const speedmap = {
  0 : 0,
  50 : 1,
  75 : 2,
  110 : 3,
  134 : 4,
  150 : 5,
  200 : 6,
  300 : 7,
  600 : 8,
  1200 : 9,
  1800 : 10,
  2400 : 11,
  4800 : 12,
  9600 : 13,
  19200 : 14,
  38400 : 15,
  57600 : 4097,
  115200 : 4098,
  230400 : 4099,
  460800 : 4100,
  500000 : 4101,
  576000 : 4102,
  921600 : 4103,
  1000000 : 4104,
  1152000 : 4105,
  2000000 : 4107,
  2500000 : 4108,
  3000000 : 4109,
  3500000 : 4110,
  4000000 : 4111 };

const TCSAFLUSH = 2;

const IXON = 1024;
const IXOFF = 4096;
const CRTSCTS = 2147483648;
const PARODD = 512;
const PARENB = 256;
const CMSPAR = 1073741824;
const CLOCAL = 2048;
const ICANON = 2;
const IGNCR = 128;
const OPOST = 1;
const ONLCR = 4;
const IGNBRK = 1;
const IGNPAR = 4;
const INPCK = 16;
const CREAD = 128;
const CS8 = 48;
const CS7 = 32;
const CS6 = 16;
const CS5 = 0;
const CSIZE = 48;
const CSTOPB = 64;
const VTIME = 5;
const VMIN = 6;

const TCIFLUSH = 0;
const TCOFLUSH = 1;
const TCIOFLUSH = 2;

const F_GETFL = 3;
const O_NONBLOCK = 2048;
const EAGAIN = 11;

try { libc.declare; } catch (_) { var libc = ctypes.open('libc.so.6'); }

var __errno_location = libc.declare('__errno_location',
                                    ctypes.default_abi, ctypes.int.ptr);
var strerror_r = libc.declare('__strerror_r',
                              ctypes.default_abi, ctypes.char.ptr,
                              ctypes.int, ctypes.char.ptr, ctypes.size_t);
function errno() { return __errno_location().contents; }

function strerror(err)
{
  var buffer = ctypes.char.array(100)();
  var msg = strerror_r(err, buffer, 100);
  return msg.readString();
}

var tcgetattr = libc.declare('tcgetattr', ctypes.default_abi, ctypes.int,
                              ctypes.int, struct_termios.ptr);
var tcsetattr = libc.declare('tcsetattr', ctypes.default_abi, ctypes.int,
                              ctypes.int, ctypes.int, struct_termios.ptr);
var cfsetospeed = libc.declare('cfsetospeed', ctypes.default_abi, ctypes.int,
                                struct_termios.ptr, speed_t);
var cfsetispeed = libc.declare('cfsetispeed', ctypes.default_abi, ctypes.int,
                                struct_termios.ptr, speed_t);
var tcflush = libc.declare('tcflush', ctypes.default_abi, ctypes.int,
                            ctypes.int, ctypes.int);
var open = libc.declare('open', ctypes.default_abi, ctypes.int,
                         ctypes.char.ptr, ctypes.int);
var close = libc.declare('close', ctypes.default_abi, ctypes.int,
                          ctypes.int);
var read = libc.declare('read', ctypes.default_abi, ctypes.ssize_t,
                         ctypes.int, ctypes.voidptr_t, ctypes.size_t);
var write = libc.declare('write', ctypes.default_abi, ctypes.ssize_t,
                          ctypes.int, ctypes.voidptr_t, ctypes.size_t);

// test null byte handling
function serial_write(fd, data) {
  if (typeof(data) == 'string')
    var buffer = string_to_c_array(data);
  else if (data instanceof Uint8Array)
    var buffer = typed_array_to_c_array(data);
  else if (data instanceof Array)
    var buffer = ctypes.unsigned_char.array()(data);
  else
    // works for e.g. ascii strings
    var buffer = ctypes.unsigned_char.array(data.length)(data);

  var rv = write(fd, buffer.address(), buffer.length);
  if (rv < 0)
  {
    var errsv = errno();
    if (errsv == EAGAIN) return 0;
    raise_error('write', SerialException, errsv);
  }

  // TODO test disconnected device (e.g. usb serial)
  // read return value 0 means nothing yet, not end of file
  //if (rv == 0) throw new SerialException('write: device disconnected');

  return rv;
}

function typed_array_to_c_array(data)
{
  var buffer = ctypes.unsigned_char.array()(data.length);
  for (let i = 0; i < data.length; i++) buffer[i] = data[i];
  return buffer;
}

function string_to_c_array(data)
{
  var buffer = ctypes.unsigned_char.array()(data.length);
  for (let i = 0; i < data.length; i++) buffer[i] = data.charCodeAt(i);
  return buffer;
}

function serial_read_ascii_string(fd, len)
{ return serial_read(fd, len, 'ascii_string'); }

function serial_read_string(fd, len)
{ return serial_read(fd, len, 'string'); }

function serial_read(fd, len, return_type)
{
  var buffer = ctypes.unsigned_char.array()(len);
  len = read(fd, buffer.address(), buffer.length);
  if (len < 0)
  {
    var errsv = errno();
    if (errsv != EAGAIN) raise_error('read', SerialException, errsv);
    buffer[0] = len = 0;
  }

  // read return value 0 means nothing yet, not end of file
  //if (len == 0) throw new SerialException('read: device disconnected');

  if (return_type == 'string')
    return String.fromCharCode.apply(null, buffer).slice(0, len);

  if (return_type == 'ascii_string') return buffer.readString();

  return (new Uint8Array(buffer)).subarray(0, len);
}

// TODO add poll or select to the mix

function raise_error(message, klass, errsv)
{
  if (klass === undefined) klass = Error;
  if (errsv === undefined) errsv = errno();
  var errmsg = strerror(errsv);
  throw new klass(message + ': [Errno ' + errsv + '] ' + errmsg);
}

function SerialException(msg)
{
  if (!(this instanceof SerialException)) return new SerialException(msg);
  this.name = 'SerialException';
  this.message = msg;
}
SerialException.prototype = Error.prototype;

function serial_set_baudrate(fd, baud)
{
  var speed = speedmap[baud];
  if (speed === undefined)
    throw new SerialException('unknown baudrate ' + baud);
  var t = struct_termios();
  var rv = 0;
  if (DEBUG || rv == 0) rv = tcgetattr(fd, t.address());
  //debug('== ' + (rv == 0) + ' === ' + (rv === 0));
  if (DEBUG || rv == 0) rv = cfsetospeed(t.address(), speed);
  if (DEBUG || rv == 0) rv = cfsetispeed(t.address(), speed);
  if (DEBUG || rv == 0) rv = tcsetattr(fd, TCSAFLUSH, t.address());
  if (!DEBUG && rv < 0) raise_error('serial_set_baudrate ' + baud, SerialException);
}

function serial_set_flowcontrol(fd, flow)
{
  flow = flow.toLowerCase();
  var fl = 0;
  switch (flow) {
    case 'none': fl = 0; break;
    case 'xon': fl = 1; break;
    case 'xoff': fl = 2; break;
    case 'xon/xoff': fl = 3; break;
    case 'rts/cts': fl = 4; break;
    case 'dtr/dsr': fl = 8; break; // FIXME dtr/dsr unsopposrted by linux kernel
    default: throw new SerialException('unknown flow control ' + flow);
  }
  var t = struct_termios();
  var rv = 0;
  if (DEBUG || rv == 0) rv = tcgetattr(fd, t.address());
  t.c_iflag &= ~(IXON | IXOFF);
  t.c_iflag |= ((fl & 1) ? IXON : 0) | ((fl & 2) ? IXOFF : 0);
  t.c_cflag &= ~(CRTSCTS);
  t.c_cflag |= (fl == 4) ? CRTSCTS : 0;
  if (DEBUG || rv == 0) rv = tcsetattr(fd, TCSAFLUSH, t.address());
  if (!DEBUG && rv < 0) raise_error('serial_set_flowcontrol ' + flow, SerialException);
}

function serial_set_parity(fd, par)
{
  // Odd, Even, None, Mark, Space; any case, only first letter matters
  par = par.toUpperCase()[0];
  var t = struct_termios();
  var rv = 0;
  if (DEBUG || rv == 0) rv = tcgetattr(fd, t.address());
  t.c_cflag &= ~tcflag_t(PARODD | PARENB | CMSPAR);
  if (par == 'O') t.c_cflag |= PARENB | PARODD;
  if (par == 'E') t.c_cflag |= PARENB;
  if (par == 'N') t.c_cflag |= 0;
  if (par == 'M') t.c_cflag |= CMSPAR;
  if (par == 'S') t.c_cflag |= PARODD | CMSPAR;
  if (DEBUG || rv == 0) rv = tcsetattr(fd, TCSAFLUSH, t.address());
  if (!DEBUG && rv < 0) raise_error('serial_set_parity ' + par, SerialException);
}

function serial_set_stopbits(fd, bits)
{
  var t = struct_termios();
  var rv = 0;
  if (DEBUG || rv == 0) rv = tcgetattr(fd, t.address());
  t.c_cflag &= ~tcflag_t(CSTOPB);
  t.c_cflag |= tcflag_t(bits == 2 ? CSTOPB : 0);
  if (DEBUG || rv == 0) rv = tcsetattr(fd, TCSAFLUSH, t.address());
  if (!DEBUG && rv < 0) raise_error('serial_set_stopbits ' + bits, SerialException);
}

function serial_set_bytesize(fd, bytesize)
{
  var t = struct_termios();
  var rv = 0;
  if (DEBUG || rv == 0) rv = tcgetattr(fd, t.address());
  t.c_cflag &= ~tcflag_t(CSIZE);
  t.c_cflag |= {8: CS8, 7: CS7, 6: CS6, 5: CS5}[bytesize];
  if (DEBUG || rv == 0) rv = tcsetattr(fd, TCSAFLUSH, t.address());
  if (!DEBUG && rv < 0) raise_error('serial_set_bytsize ' + bytsize, SerialException);
}

function serial_set_raw(fd)
{
  var t = struct_termios();
  var rv = 0;
  if (DEBUG || rv == 0) rv = tcgetattr(fd, t.address());
  t.c_iflag = (IGNBRK | IGNPAR | INPCK);
  t.c_oflag = 0;
  t.c_cflag = (CS8 | CREAD | CLOCAL);
  t.c_lflag = 0;
  t.c_cc[VMIN] = 1;
  t.c_cc[VTIME] = 0;
  if (DEBUG || rv == 0) rv = tcsetattr(fd, TCSAFLUSH, t.address());
  if (!DEBUG && rv < 0) raise_error('serial_set_raw', SerialException);
}

function serial_set_canonical(fd, c)
{
  var t = struct_termios();
  var rv = 0;
  if (DEBUG || rv == 0) rv = tcgetattr(fd, t.address());
  t.c_cflag &= ~tcflag_t(ICANON);
  t.c_cflag |= tcflag_t(c ? ICANON : 0);
  if (DEBUG || rv == 0) rv = tcsetattr(fd, TCSAFLUSH, t.address());
  if (!DEBUG && rv < 0) raise_error('serial_set_canonical ' + c, SerialException);
}

function serial_set_crnl(fd, c)
{
  var t = struct_termios();
  var rv = 0;
  if (DEBUG || rv == 0) rv = tcgetattr(fd, t.address());
  t.c_iflag &= ~tcflag_t(IGNCR);
  t.c_iflag |= c ? IGNCR : 0;
  t.c_oflag &= ~tcflag_t(OPOST | ONLCR);
  t.c_oflag |= c ? (OPOST | ONLCR) : 0;
  if (DEBUG || rv == 0) rv = tcsetattr(fd, TCSAFLUSH, t.address());
  if (!DEBUG && rv < 0) raise_error('serial_set_crnl ' + c, SerialException);
}

function serial_set_modem(fd, m)
{
  var t = struct_termios();
  var rv = 0;
  if (DEBUG || rv == 0) rv = tcgetattr(fd, t.address());
  t.c_cflag &= ~tcflag_t(CLOCAL);
  t.c_cflag |= tcflag_t(m ? 0 : CLOCAL);
  if (DEBUG || rv == 0) rv = tcsetattr(fd, TCSAFLUSH, t.address());
  if (!DEBUG && rv < 0) raise_error('serial_set_modem ' + m, SerialException);
}

function serial_set_blocking(fd, blocking)
{
  var rv = 0;
  var tmp = rv = fcntl(fd, F_GETFL);
  if (blocking) tmp &= ~O_NONBLOCK;
  else tmp |= ~O_NONBLOCK;
  if (DEBUG || rv == 0) rv = fcntl(fd, F_SETFL, ctypes.long(tmp));
  if (!DEBUG && rv < 0) raise_error('serial_set_blocking ' + blocking, SerialException);
}

function serial_flush(fd, mode)
{
  if (!mode) mode = 'io';
  var rv = tcflush(fd, {i: TCIFLUSH, o: TCOFLUSH, io: TCIOFLUSH}[mode]);
  if (!DEBUG && rv < 0) raise_error('serial_flush ' + mode, SerialException);
}

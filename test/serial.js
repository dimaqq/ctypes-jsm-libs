// test cases
try { debug.foo; } catch (_) { debug = function(m) { dump(m + '\n'); }; }

// test write casting
function test_serial_write_string() {
  var fd = open('/tmp/testfilestring', O_RDWR);
  serial_write(fd, 'a\x00b\xffc');
  close(fd);
}

function test_serial_write_array() {
  var fd = open('/tmp/testfilearray', O_RDWR);
  serial_write(fd, [70, 0, 255]);
  close(fd);
}

function test_serial_write_typed_array() {
  var buffer = new Uint8Array(3);
  buffer[0] = 43;
  buffer[1] = 0;
  buffer[2] = 255;
  var fd = open('/tmp/testfiletypedarray', O_RDWR);
  serial_write(fd, buffer);
  close(fd);
}

function test_serial_read_ascii_string() {
  var fd = open('/tmp/testfileread', O_RDWR);
  var rv = serial_read_ascii_string(fd, 100);
  debug('read data ' + rv + ' len ' + rv.length);
  close(fd);
}

function test_serial_read_string() {
  var fd = open('/tmp/testfileread', O_RDWR);
  var rv = serial_read_string(fd, 100);
  debug('read data ' + rv);
  debug('read data len ' + rv.length);
  close(fd);
}

function test_serial_read() {
  var fd = open('/tmp/testfileread', O_RDWR);
  var rv = serial_read(fd, 100);
  debug('read data ' + rv + ' len ' + rv.length);
  close(fd);
}

try {
test_serial_write_typed_array();
test_serial_write_string();
test_serial_write_array();
test_serial_read_ascii_string();
test_serial_read_string();
test_serial_read();
} catch (e) { debug(e); }

function test_serial_baudrate() {
  var fd = open('/dev/ttyS0', O_RDWR | O_NOCTTY | O_NONBLOCK);
  debug('fd ' + fd);
  serial_set_baudrate(fd, 115200);
  close(fd);
}

#include <termios.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>


static void fail(const char *msg) {
  fprintf(stderr, "Operation failed: %s (%m)", msg);
  fflush(stderr);
  _exit(0);
}

void setupSerial(int fd) {
    // nuke any existing buffered data
  if (tcflush(fd, TCIOFLUSH)) { fail("tcflush"); }

  if (fcntl(fd, F_SETFL, FNDELAY)) { fail("FNDELAY"); }

  termios tio;
  memset(&tio, 0, sizeof(termios));
  if (tcgetattr(fd, &tio)) { fail("tcgetattr"); }
  cfmakeraw(&tio);
  if (tcsetattr(fd, TCSANOW, &tio)) { fail("flow control"); }

}
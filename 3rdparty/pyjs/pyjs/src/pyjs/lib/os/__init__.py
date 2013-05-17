# Pyjs replacement of Python's os module
# Copyright (C) 2012 João Ventura <joaojonesventura@gmail.com>

import path
from os.path import (walk, curdir, pardir, sep, pathsep, defpath, extsep, altsep, devnull)

name = 'pyjs'

# Constants from posix
SEEK_SET = 0
SEEK_CUR = 1
SEEK_END = 2
EX_CANTCREAT = 73
EX_CONFIG = 78
EX_DATAERR = 65
EX_IOERR = 74
EX_NOHOST = 68
EX_NOINPUT = 66
EX_NOPERM = 77
EX_NOUSER = 67
EX_OK = 0
EX_OSERR = 71
EX_OSFILE = 72
EX_PROTOCOL = 76
EX_SOFTWARE = 70
EX_TEMPFAIL = 75
EX_UNAVAILABLE = 69
EX_USAGE = 64
F_OK = 0
NGROUPS_MAX = 65536
O_APPEND = 1024
O_ASYNC = 8192
O_CREAT = 64
O_DIRECT = 16384
O_DIRECTORY = 65536
O_DSYNC = 4096
O_EXCL = 128
O_LARGEFILE = 32768
O_NDELAY = 2048
O_NOATIME = 262144
O_NOCTTY = 256
O_NOFOLLOW = 131072
O_NONBLOCK = 2048
O_RDONLY = 0
O_RDWR = 2
O_RSYNC = 1052672
O_SYNC = 1052672
O_TRUNC = 512
O_WRONLY = 1
R_OK = 4
TMP_MAX = 238328
WCONTINUED = 8
WNOHANG = 1
WUNTRACED = 2
W_OK = 2
X_OK = 1

environ = {}

def urandom(n):
    # """urandom(n) -> str
    # Return a string of n random bytes suitable for cryptographic use.
    # """
    #try:
    #    _urandomfd = open("/dev/urandom", O_RDONLY)
    #except (OSError, IOError):
    #    raise NotImplementedError("/dev/urandom (or equivalent) not found")
    #try:
    #    bs = b""
    #    while n - len(bs) >= 1:
    #        bs += read(_urandomfd, n - len(bs))
    #finally:
    #    close(_urandomfd)
    raise NotImplementedError("/dev/urandom (or equivalent) not found")
    return bs

def unlink(*args):
    raise NotImplementedError("PyJS does not support filesystem access")

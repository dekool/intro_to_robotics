import errno

def get_error_name(e):
    if e==errno.EPERM:
        return 'Operation not permitted'
    if e==errno.ENOENT:
        return 'No such file or directory'
    if e==errno.ESRCH:
        return 'No such process'
    if e==errno.EINTR:
        return 'Interrupted system call'
    if e==errno.EIO:
        return 'I/O error'
    if e==errno.ENXIO:
        return 'No such device or address'
    if e==errno.E2BIG:
        return 'Arg list too long'
    if e==errno.ENOEXEC:
        return 'Exec format error'
    if e==errno.EBADF:
        return 'Bad file number'
    if e==errno.ECHILD:
        return 'No child processes'
    if e==errno.EAGAIN:
        return 'Try again'
    if e==errno.ENOMEM:
        return 'Out of memory'
    if e==errno.EACCES:
        return 'Permission denied'
    if e==errno.EFAULT:
        return 'Bad address'
    if e==errno.ENOTBLK:
        return 'Block device required'
    if e==errno.EBUSY:
        return 'Device or resource busy'
    if e==errno.EEXIST:
        return 'File exists'
    if e==errno.EXDEV:
        return 'Cross-device link'
    if e==errno.ENODEV:
        return 'No such device'
    if e==errno.ENOTDIR:
        return 'Not a directory'
    if e==errno.EISDIR:
        return 'Is a directory'
    if e==errno.EINVAL:
        return 'Invalid argument'
    if e==errno.ENFILE:
        return 'File table overflow'
    if e==errno.EMFILE:
        return 'Too many open files'
    if e==errno.ENOTTY:
        return 'Not a typewriter'
    if e==errno.ETXTBSY:
        return 'Text file busy'
    if e==errno.EFBIG:
        return 'File too large'
    if e==errno.ENOSPC:
        return 'No space left on device'
    if e==errno.ESPIPE:
        return 'Illegal seek'
    if e==errno.EROFS:
        return 'Read-only file system'
    if e==errno.EMLINK:
        return 'Too many links'
    if e==errno.EPIPE:
        return 'Broken pipe'
    if e==errno.EDOM:
        return 'Math argument out of domain of func'
    if e==errno.ERANGE:
        return 'Math result not representable'
    if e==errno.EDEADLK:
        return 'Resource deadlock would occur'
    if e==errno.ENAMETOOLONG:
        return 'File name too long'
    if e==errno.ENOLCK:
        return 'No record locks available'
    if e==errno.ENOSYS:
        return 'Function not implemented'
    if e==errno.ENOTEMPTY:
        return 'Directory not empty'
    if e==errno.ELOOP:
        return 'Too many symbolic links encountered'
    if e==errno.EWOULDBLOCK:
        return 'Operation would block'
    if e==errno.ENOMSG:
        return 'No message of desired type'
    if e==errno.EIDRM:
        return 'Identifier removed'
    if e==errno.ECHRNG:
        return 'Channel number out of range'
    if e==errno.EL2NSYNC:
        return 'Level 2 not synchronized'
    if e==errno.EL3HLT:
        return 'Level 3 halted'
    if e==errno.EL3RST:
        return 'Level 3 reset'
    if e==errno.ELNRNG:
        return 'Link number out of range'
    if e==errno.EUNATCH:
        return 'Protocol driver not attached'
    if e==errno.ENOCSI:
        return 'No CSI structure available'
    if e==errno.EL2HLT:
        return 'Level 2 halted'
    if e==errno.EBADE:
        return 'Invalid exchange'
    if e==errno.EBADR:
        return 'Invalid request descriptor'
    if e==errno.EXFULL:
        return 'Exchange full'
    if e==errno.ENOANO:
        return 'No anode'
    if e==errno.EBADRQC:
        return 'Invalid request code'
    if e==errno.EBADSLT:
        return 'Invalid slot'
    if e==errno.EDEADLOCK:
        return 'File locking deadlock error'
    if e==errno.EBFONT:
        return 'Bad font file format'
    if e==errno.ENOSTR:
        return 'Device not a stream'
    if e==errno.ENODATA:
        return 'No data available'
    if e==errno.ETIME:
        return 'Timer expired'
    if e==errno.ENOSR:
        return 'Out of streams resources'
    if e==errno.ENONET:
        return 'Machine is not on the network'
    if e==errno.ENOPKG:
        return 'Package not installed'
    if e==errno.EREMOTE:
        return 'Object is remote'
    if e==errno.ENOLINK:
        return 'Link has been severed'
    if e==errno.EADV:
        return 'Advertise error'
    if e==errno.ESRMNT:
        return 'Srmount error'
    if e==errno.ECOMM:
        return 'Communication error on send'
    if e==errno.EPROTO:
        return 'Protocol error'
    if e==errno.EMULTIHOP:
        return 'Multihop attempted'
    if e==errno.EDOTDOT:
        return 'RFS specific error'
    if e==errno.EBADMSG:
        return 'Not a data message'
    if e==errno.EOVERFLOW:
        return 'Value too large for defined data type'
    if e==errno.ENOTUNIQ:
        return 'Name not unique on network'
    if e==errno.EBADFD:
        return 'File descriptor in bad state'
    if e==errno.EREMCHG:
        return 'Remote address changed'
    if e==errno.ELIBACC:
        return 'Can not access a needed shared library'
    if e==errno.ELIBBAD:
        return 'Accessing a corrupted shared library'
    if e==errno.ELIBSCN:
        return '.lib section in a.out corrupted'
    if e==errno.ELIBMAX:
        return 'Attempting to link in too many shared libraries'
    if e==errno.ELIBEXEC:
        return 'Cannot exec a shared library directly'
    if e==errno.EILSEQ:
        return 'Illegal byte sequence'
    if e==errno.ERESTART:
        return 'Interrupted system call should be restarted'
    if e==errno.ESTRPIPE:
        return 'Streams pipe error'
    if e==errno.EUSERS:
        return 'Too many users'
    if e==errno.ENOTSOCK:
        return 'Socket operation on non-socket'
    if e==errno.EDESTADDRREQ:
        return 'Destination address required'
    if e==errno.EMSGSIZE:
        return 'Message too long'
    if e==errno.EPROTOTYPE:
        return 'Protocol wrong type for socket'
    if e==errno.ENOPROTOOPT:
        return 'Protocol not available'
    if e==errno.EPROTONOSUPPORT:
        return 'Protocol not supported'
    if e==errno.ESOCKTNOSUPPORT:
        return 'Socket type not supported'
    if e==errno.EOPNOTSUPP:
        return 'Operation not supported on transport endpoint'
    if e==errno.EPFNOSUPPORT:
        return 'Protocol family not supported'
    if e==errno.EAFNOSUPPORT:
        return 'Address family not supported by protocol'
    if e==errno.EADDRINUSE:
        return 'Address already in use'
    if e==errno.EADDRNOTAVAIL:
        return 'Cannot assign requested address'
    if e==errno.ENETDOWN:
        return 'Network is down'
    if e==errno.ENETUNREACH:
        return 'Network is unreachable'
    if e==errno.ENETRESET:
        return 'Network dropped connection because of reset'
    if e==errno.ECONNABORTED:
        return 'Software caused connection abort'
    if e==errno.ECONNRESET:
        return 'Connection reset by peer'
    if e==errno.ENOBUFS:
        return 'No buffer space available'
    if e==errno.EISCONN:
        return 'Transport endpoint is already connected'
    if e==errno.ENOTCONN:
        return 'Transport endpoint is not connected'
    if e==errno.ESHUTDOWN:
        return 'Cannot send after transport endpoint shutdown'
    if e==errno.ETOOMANYREFS:
        return 'Too many references: cannot splice'
    if e==errno.ETIMEDOUT:
        return 'Connection timed out'
    if e==errno.ECONNREFUSED:
        return 'Connection refused'
    if e==errno.EHOSTDOWN:
        return 'Host is down'
    if e==errno.EHOSTUNREACH:
        return 'No route to host'
    if e==errno.EALREADY:
        return 'Operation already in progress'
    if e==errno.EINPROGRESS:
        return 'Operation now in progress'
    if e==errno.ESTALE:
        return 'Stale NFS file handle'
    if e==errno.EUCLEAN:
        return 'Structure needs cleaning'
    if e==errno.ENOTNAM:
        return 'Not a XENIX named type file'
    if e==errno.ENAVAIL:
        return 'No XENIX semaphores available'
    if e==errno.EISNAM:
        return 'Is a named type file'
    if e==errno.EREMOTEIO:
        return 'Remote I/O error'
    if e==errno.EDQUOT:
        return 'Quota exceeded'
    return "Unknown"
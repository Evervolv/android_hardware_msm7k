/*
 * Sun RPC is a product of Sun Microsystems, Inc. and is provided for
 * unrestricted use provided that this legend is included on all tape
 * media and as a part of the software program in whole or part.  Users
 * may copy or modify Sun RPC without charge, but are not authorized
 * to license or distribute it to anyone else except as part of a product or
 * program developed by the user.
 *
 * SUN RPC IS PROVIDED AS IS WITH NO WARRANTIES OF ANY KIND INCLUDING THE
 * WARRANTIES OF DESIGN, MERCHANTIBILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE, OR ARISING FROM A COURSE OF DEALING, USAGE OR TRADE PRACTICE.
 *
 * Sun RPC is provided with no support and without any obligation on the
 * part of Sun Microsystems, Inc. to assist in its use, correction,
 * modification or enhancement.
 *
 * SUN MICROSYSTEMS, INC. SHALL HAVE NO LIABILITY WITH RESPECT TO THE
 * INFRINGEMENT OF COPYRIGHTS, TRADE SECRETS OR ANY PATENTS BY SUN RPC
 * OR ANY PART THEREOF.
 *
 * In no event will Sun Microsystems, Inc. be liable for any lost revenue
 * or profits or other special, indirect and consequential damages, even if
 * Sun has been advised of the possibility of such damages.
 *
 * Sun Microsystems, Inc.
 * 2550 Garcia Avenue
 * Mountain View, California  94043
 */

#include <rpc/rpc.h>
#include <rpc/rpc_router_ioctl.h>
#include <debug.h>

#include <sys/types.h>   
#include <sys/stat.h>     
#include <fcntl.h>        
#include <unistd.h>      
#include <stdio.h>
#include <errno.h>

#define DUMP_DATA 0

int r_open(const char *router)
{
  int handle = open(router, O_RDWR, 0);  

  if(handle < 0)
      E("error opening %s: %s\n", router, strerror(errno));
  return handle;
}

void r_close(int handle)
{
    if(close(handle) < 0) E("error: %s\n", strerror(errno));
}

int r_read(int handle, char *buf, uint32 size)
{
	int rc = read((int) handle, (void *)buf, size);
	if (rc < 0)
		E("error reading RPC packet: %d (%s)\n", errno, strerror(errno));
#if DUMP_DATA
	else {
		int len = rc / 4;
		uint32_t *data = (uint32_t *)buf;
		fprintf(stdout, "RPC in  %02d:", rc);
		while (len--)
			fprintf(stdout, " %08x", *data++);
		fprintf(stdout, "\n");
	}
#endif
	return rc;
}

int r_write (int handle, const char *buf, uint32 size)
{
	int rc = write(handle, (void *)buf, size);
	if (rc < 0)
		E("error writing RPC packet: %d (%s)\n", errno, strerror(errno));
#if DUMP_DATA
	else {
		int len = rc / 4;
		uint32_t *data = (uint32_t *)buf;
		fprintf(stdout, "RPC out %02d:", rc);
		while (len--)
			fprintf(stdout, " %08x", *data++);
		fprintf(stdout, "\n");
	}
#endif
	return rc;
}

int r_control(int handle, const uint32 cmd, void *arg)
{
  return ioctl(handle, cmd, arg);
}




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

#ifndef DEBUG_H
#define DEBUG_H

#include <stdio.h>

#define LOG_TAG "RPC"
#include <utils/Log.h>

#if 1
#define PRINT(x...) do {                                    \
        fprintf(stdout, "%s(%d) ", __FUNCTION__, __LINE__); \
        fprintf(stdout, ##x);                               \
    } while(0)
#else
#define PRINT(x...) do {                                    \
        fprintf(stdout, "%s(%d) ", __FUNCTION__, __LINE__); \
        fprintf(stdout, ##x);                               \
        LOGI(x);                               \
    } while(0)
#endif

#ifdef DEBUG
#define D PRINT
#else
#define D(x...) do { } while(0)
#endif

#ifdef VERBOSE
#define V PRINT
#else
#define V(x...) do { } while(0)
#endif

#define E(x...) do {                                        \
        fprintf(stderr, "%s(%d) ", __FUNCTION__, __LINE__); \
        fprintf(stderr, ##x);                               \
        LOGE(x);                                            \
    } while(0)

#define FAILIF(cond, msg...) do {                                              \
        if (__builtin_expect (cond, 0)) {                                      \
            fprintf(stderr, "%s:%s:(%d): ", __FILE__, __FUNCTION__, __LINE__); \
            fprintf(stderr, ##msg);                                            \
        }                                                                      \
    } while(0)

#endif/*DEBUG_H*/

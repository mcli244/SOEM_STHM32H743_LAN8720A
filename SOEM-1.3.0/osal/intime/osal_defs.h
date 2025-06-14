/*
 * Licensed under the GNU General Public License version 2 with exceptions. See
 * LICENSE file in the project root for full license information
 */

#ifndef _osal_defs_
#define _osal_defs_

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef PACKED
#define PACKED_BEGIN
#define PACKED  __attribute__((__packed__))
#define PACKED_END
#endif

#define OSAL_THREAD_HANDLE task_t *
#define OSAL_THREAD_FUNC void
#define OSAL_THREAD_FUNC_RT void

#define __packed    __attribute__((__packed__))


#ifdef __cplusplus
}
#endif

#endif

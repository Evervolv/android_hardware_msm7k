/*
 * Copyright (C) 2008 The Android Open Source Project
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define LOG_TAG "Overlay"

#include <hardware/hardware.h>
#include <hardware/overlay.h>

#include <fcntl.h>
#include <errno.h>

#include <cutils/log.h>
#include <cutils/atomic.h>

#include <linux/fb.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <linux/msm_mdp.h>
#include <linux/msm_rotator.h>

#include "gralloc_priv.h"

#define USE_MSM_ROTATOR
/*****************************************************************************/

struct overlay_control_context_t {
	struct overlay_control_device_t device;
};

struct overlay_data_context_t {
	struct overlay_data_device_t device;

	/* our private state goes below here */
	int mFD;
	int rotator;
	int pmem;
	void * pmem_addr;
	uint32_t pmem_offset;
	struct msmfb_overlay_data od;
	struct msmfb_overlay_data od_rot;
	struct msm_rotator_data_info rot;
};

static int overlay_device_open(const struct hw_module_t* module, const char* name,
							   struct hw_device_t** device);

static struct hw_module_methods_t overlay_module_methods = {
	open: overlay_device_open
};

struct overlay_module_t HAL_MODULE_INFO_SYM = {
	common: {
		tag: HARDWARE_MODULE_TAG,
		version_major: 1,
		version_minor: 0,
		id: OVERLAY_HARDWARE_MODULE_ID,
		name: "QCT MSM OVERLAY module",
		author: "QuIC, Inc.",
		methods: &overlay_module_methods,
	}
};

struct overlay_hw_info {
	int w;
	int h;
	int bpp;
	int ystride;
};

struct handle_t : public native_handle {
	/* add the data fields we need here, for instance: */
	int ov_fd;
	int rot_fd;
	int size;
	struct mdp_overlay ov;
	struct overlay_hw_info ovHwInfo;
	struct msm_rotator_img_info rotInfo;
};

static int handle_get_ovId(const overlay_handle_t overlay) {
	return static_cast<const struct handle_t *>(overlay)->ov.id;
}

static int handle_get_rotId(const overlay_handle_t overlay) {
	return static_cast<const struct handle_t *>(overlay)->rotInfo.session_id;
}


static int handle_get_size(const overlay_handle_t overlay) {
	return static_cast<const struct handle_t *>(overlay)->size;
}

/** convert OVERLAY_FORMAT to MDP format */
static int get_format(int format) {
	switch (format) {
		case OVERLAY_FORMAT_RGBA_8888:     return MDP_RGBA_8888;
		case OVERLAY_FORMAT_BGRA_8888:     return MDP_BGRA_8888;
		case OVERLAY_FORMAT_RGB_565:       return MDP_RGB_565;
		case HAL_PIXEL_FORMAT_YCbCr_422_SP:  return MDP_Y_CBCR_H2V1;
		case HAL_PIXEL_FORMAT_YCrCb_420_SP:  return MDP_Y_CRCB_H2V2;
		case HAL_PIXEL_FORMAT_YCbCr_420_SP:  return MDP_Y_CBCR_H2V2;
	}
	return -1;
}

static int get_size(int format, int w, int h) {
	int size;

	size = w * h;
	switch (format) {
		case OVERLAY_FORMAT_RGBA_8888:     size *= 4; break;
		case OVERLAY_FORMAT_BGRA_8888:     size *= 4; break;
		case OVERLAY_FORMAT_RGB_565:       size *= 2; break;
		case HAL_PIXEL_FORMAT_YCbCr_422_SP:  size *= 2; break;
		case HAL_PIXEL_FORMAT_YCrCb_420_SP:  size = (size*3)/2; break;
		case HAL_PIXEL_FORMAT_YCbCr_420_SP:  size = (size*3)/2; break;
		default: return 0;
	}
	return size;
}

static void show_ov(struct mdp_overlay *ov) {
  LOGE("ov.id: %d", ov->id);
  LOGE("ov.src.w: %d", ov->src.width);
  LOGE("ov.src.h: %d", ov->src.height);
  LOGE("ov.src.f: %d", ov->src.format);
  LOGE("ov.srcR.x: %d", ov->src_rect.x);
  LOGE("ov.srcR.y: %d", ov->src_rect.y);
  LOGE("ov.srcR.w: %d", ov->src_rect.w);
  LOGE("ov.srcR.h: %d", ov->src_rect.h);

  LOGE("ov.dstR.x: %d", ov->dst_rect.x);
  LOGE("ov.dstR.y: %d", ov->dst_rect.y);
  LOGE("ov.dstR.w: %d", ov->dst_rect.w);
  LOGE("ov.dstR.h: %d", ov->dst_rect.h);

  LOGE("ov.z: %d", ov->z_order);
  LOGE("ov.alpha: %d", ov->alpha);
  LOGE("ov.tp: 0x%x", ov->transp_mask);
  LOGE("ov.flag: %d", ov->flags);
  LOGE("ov.is_fg: %d", ov->is_fg);
}

/*****************************************************************************/

/*
 * This is the overlay_t object, it is returned to the user and represents
 * an overlay.
 * This handles will be passed across processes and possibly given to other
 * HAL modules (for instance video decode modules).
 */
class overlay_object : public overlay_t {
	handle_t mHandle;

	static overlay_handle_t getHandleRef(struct overlay_t* overlay) {
		/* returns a reference to the handle, caller doesn't take ownership */
		return &(static_cast<overlay_object *>(overlay)->mHandle);
	}

public:
	overlay_object(int fb, int rotator, int w, int h, int format, int size,
				   int dW, int dH, int dBpp, int dYstride) {
		this->overlay_t::getHandleRef = getHandleRef;
		mHandle.version = sizeof(native_handle);
		mHandle.numFds = 2;
		mHandle.numInts = (sizeof(mHandle) - 5) / 4;

		mHandle.ov_fd = fb;
		mHandle.rot_fd = rotator;

		memset((void *)&mHandle.ov, 0, sizeof(mHandle.ov));
		mHandle.ov.id = MSMFB_NEW_REQUEST;
		mHandle.ov.src.width  = w;
		mHandle.ov.src.height = h;
		mHandle.ov.src.format = format;
		mHandle.ov.src_rect.x = 0;
		mHandle.ov.src_rect.y = 0;
		mHandle.ov.src_rect.w = w;
		mHandle.ov.src_rect.h = h;
		mHandle.ov.dst_rect.x = 0;
		mHandle.ov.dst_rect.y = 0;

		mHandle.ov.dst_rect.w = w;
		if (mHandle.ov.dst_rect.w > dW)
			mHandle.ov.dst_rect.w = dW;

		mHandle.ov.dst_rect.h = h;
		if (mHandle.ov.dst_rect.h > dH)
			mHandle.ov.dst_rect.h = dH;

		mHandle.ov.z_order = 0;
		mHandle.ov.alpha = 0xb2;
		mHandle.ov.transp_mask = 0x0;
		mHandle.ov.flags = 0;
		mHandle.ov.is_fg = 0;

		mHandle.ovHwInfo.w = dW;
		mHandle.ovHwInfo.h = dH;
		mHandle.ovHwInfo.bpp = dBpp;
		mHandle.ovHwInfo.ystride = dYstride;

		mHandle.size = size;
	}

	struct mdp_overlay * getHwOv(void) {
		return &mHandle.ov;
	}
	int getHwOvId(void) {return mHandle.ov.id;}
	struct overlay_hw_info * getOvHwInfo(void) {
		return &mHandle.ovHwInfo;
	}
	int getOvFd(void) {return mHandle.ov_fd;}
	int getRotFd(void) {return mHandle.rot_fd;}
        int getRotSessionId(void) {return mHandle.rotInfo.session_id;}
	struct msm_rotator_img_info * getRot(void) {
		return &mHandle.rotInfo;
	}
    void rotator_dest_swap (void) {
        int tmp = mHandle.rotInfo.dst.width;
        mHandle.rotInfo.dst.width = mHandle.rotInfo.dst.height;
        mHandle.rotInfo.dst.height = tmp;
    }
};

// ****************************************************************************
// Control module
// ****************************************************************************

	static int overlay_get(struct overlay_control_device_t *dev, int name) {
		int result = -1;
		switch (name) {
			case OVERLAY_MINIFICATION_LIMIT:
				result = 8;
				break;
			case OVERLAY_MAGNIFICATION_LIMIT:
				result = 8;
				break;
			case OVERLAY_SCALING_FRAC_BITS:
				result = 32;
				break;
			case OVERLAY_ROTATION_STEP_DEG:
				result = 90; // 90 rotation steps (for instance)
				break;
			case OVERLAY_HORIZONTAL_ALIGNMENT:
				result = 1;	// 1-pixel alignment
				break;
			case OVERLAY_VERTICAL_ALIGNMENT:
				result = 1;	// 1-pixel alignment
				break;
			case OVERLAY_WIDTH_ALIGNMENT:
				result = 1;	// 1-pixel alignment
				break;
			case OVERLAY_HEIGHT_ALIGNMENT:
				result = 1;	// 1-pixel alignment
				break;
		}
		return result;
	}

	static int overlay_start_rotator(int fd, struct msm_rotator_img_info *ri, int w,
									 int h, int format) {
		int result;

		ri->src.format = format;
		ri->src.width = w;
		ri->src.height = h;
		ri->dst.format = format;
		ri->dst.width = w;
		ri->dst.height = h;
		ri->dst_x = 0;
		ri->dst_y = 0;
		ri->src_rect.x = 0;
		ri->src_rect.y = 0;
		ri->src_rect.w = w;
		ri->src_rect.h = h;
		ri->rotations = 0;
		ri->enable = 0;
		ri->session_id = 0;

		/* let's start the engine and get the session id assigned */
		result = ioctl(fd, MSM_ROTATOR_IOCTL_START, ri);
		if (result < 0) {
			LOGE("%s: MSM_ROTATOR_IOCTL_START failed! = %d\n", __FUNCTION__, result);
			return result;
		}

		return 0;
	}


	static overlay_t* overlay_createOverlay(struct overlay_control_device_t *dev,
											uint32_t w, uint32_t h, int32_t format) {
		overlay_object            *overlay = NULL;
		overlay_control_context_t *ctx = (overlay_control_context_t *)dev;
		struct fb_fix_screeninfo finfo;
		struct fb_var_screeninfo vinfo;
		int hw_format;
		int fb, rotator;
		struct mdp_overlay * hwOv;

		/* Create overlay object. Talk to the h/w here and adjust to what it can
		 * do. the overlay_t returned can  be a C++ object, subclassing overlay_t
		 * if needed.
		 *
		 * we probably want to keep a list of the overlay_t created so they can
		 * all be cleaned up in overlay_close().
		 */

		hw_format = get_format(format);
		if (hw_format == -1) {
			LOGE("%s: unsupported format! = %d\n", __FUNCTION__, format);
			return NULL;
		}

		fb = open("/dev/graphics/fb0", O_RDWR, 0);
		if (fb < 0) {
			LOGE("%s: error opening overlay device!", __FUNCTION__);
			return NULL;
		}


		if (ioctl(fb, FBIOGET_FSCREENINFO, &finfo) == -1) {
			LOGE("%s: FBIOGET_FSCREENINFO failed!", __FUNCTION__);
			goto overlay_create_fail;
		}

		if (ioctl(fb, FBIOGET_VSCREENINFO, &vinfo) == -1) {
			LOGE("%s: FBIOGET_VSCREENINFO failed!", __FUNCTION__);
			goto overlay_create_fail;
		}
#ifdef USE_MSM_ROTATOR
		rotator = open("/dev/msm_rotator", O_RDWR, 0);
		if (rotator < 0) {
			LOGE("%s: error opening rotator device!", __FUNCTION__);
			goto overlay_create_fail;
		}
#endif
		/* number of buffer is not being used as overlay buffers are coming from client */
		overlay = new overlay_object(fb, rotator, w, h, hw_format,
				get_size(format, w, h), vinfo.xres, vinfo.yres, vinfo.bits_per_pixel, finfo.line_length);

		if (overlay == NULL) {
			LOGE("%s: can't create overlay object!", __FUNCTION__);
			return NULL;
		}

		hwOv = overlay->getHwOv();

		if (ioctl(fb, MSMFB_OVERLAY_SET, hwOv)) {
			LOGE("%s: MSMFB_OVERLAY_SET error!", __FUNCTION__);
			goto overlay_create_fail3;
		}

#ifdef USE_MSM_ROTATOR
		if (overlay_start_rotator(rotator, overlay->getRot(), w, h, hw_format)) {
			LOGE("%s: error starting rotator device!", __FUNCTION__);
			goto overlay_create_fail3;
		}
#endif

		return overlay;

		overlay_create_fail3:
		delete overlay;
#ifdef USE_MSM_ROTATOR
		close(rotator);
#endif
		overlay_create_fail:
		close(fb);
		return NULL;
	}

	static void overlay_destroyOverlay(struct overlay_control_device_t *dev,
									   overlay_t* overlay)
	{
		overlay_control_context_t *ctx = (overlay_control_context_t *)dev;
		overlay_object *obj = static_cast<overlay_object *>(overlay);
		int ovId;


		/* free resources associated with this overlay_t */
		if (obj) {
			ovId = obj->getHwOvId();
#ifdef USE_MSM_ROTATOR
			int session_id = obj->getRotSessionId();
			ioctl(obj->getRotFd(), MSM_ROTATOR_IOCTL_FINISH,
				&session_id);
#endif
			ioctl(obj->getOvFd(), MSMFB_OVERLAY_UNSET, &ovId);
			close(obj->getOvFd());
#ifdef USE_MSM_ROTATOR
			close(obj->getRotFd());
#endif
		}

		delete overlay;
	}

	static int overlay_setPosition(struct overlay_control_device_t *dev,
								   overlay_t* overlay,
								   int x, int y, uint32_t w, uint32_t h) {

		/* set this overlay's position (talk to the h/w) */
		overlay_control_context_t *ctx = (overlay_control_context_t *)dev;
		overlay_object * obj = static_cast<overlay_object *>(overlay);
		struct mdp_overlay ov;

		struct overlay_hw_info * ovHw;
		ovHw = obj->getOvHwInfo();

               if ((x < 0)
                   || (y < 0)
                   || ((x + w) > ovHw->w)
                   || ((y + h) > ovHw->h)) {
                   return -1;
               }
		ov.id = obj->getHwOvId();
		if (ioctl(obj->getOvFd(), MSMFB_OVERLAY_GET, &ov)) {
			LOGE("%s: MSMFB_OVERLAY_GET error!", __FUNCTION__);
			return -errno;
		}

		ov.dst_rect.x = x;
		ov.dst_rect.y = y;
		ov.dst_rect.w = w;
		ov.dst_rect.h = h;

		if (ioctl(obj->getOvFd(), MSMFB_OVERLAY_SET, &ov)) {
			LOGE("%s: MSMFB_OVERLAY_SET error!", __FUNCTION__);
			return -errno;
		}

		return 0;
	}

	static int overlay_commit(struct overlay_control_device_t *dev,
								   overlay_t* overlay)
	{
		return 0;
	}

	static int overlay_getPosition(struct overlay_control_device_t *dev,
								   overlay_t* overlay,
								   int* x, int* y, uint32_t* w, uint32_t* h) {

		/* get this overlay's position */
		overlay_object * obj = static_cast<overlay_object *>(overlay);
		struct mdp_overlay ov;

		ov.id = obj->getHwOvId();
		if (ioctl(obj->getOvFd(), MSMFB_OVERLAY_GET, &ov)) {
			LOGE("%s: MSMFB_OVERLAY_GET error!", __FUNCTION__);
			return -errno;
		}

		*x = ov.dst_rect.x;
		*y = ov.dst_rect.y;
		*w = ov.dst_rect.w;
		*h = ov.dst_rect.h;

		return 0;
	}

	static int overlay_setRot(int rotator, struct msm_rotator_img_info *ri, int flag) {
		int result;

		ri->rotations = flag;

		if (flag == MDP_ROT_NOP)
			ri->enable = 0;
		else
			ri->enable = 1;

		result = ioctl(rotator, MSM_ROTATOR_IOCTL_START, ri);
		if (result < 0)
			LOGE("%s: MSM_ROTATOR_IOCTL_START failed! = %d\n", __FUNCTION__, result);

		return result;
	}

	static void overlay_src_swap(struct mdp_overlay *ov) {
		int tmp;

		tmp = ov->src.width;
		ov->src.width = ov->src.height;
		ov->src.height = tmp;

		tmp = ov->src_rect.w;
		ov->src_rect.w = ov->src_rect.h;
		ov->src_rect.h = tmp;
	}

	static int overlay_setParameter(struct overlay_control_device_t *dev,
									overlay_t* overlay, int param, int value) {

		overlay_control_context_t *ctx = (overlay_control_context_t *)dev;
		overlay_object *obj = static_cast<overlay_object *>(overlay);
		int result = 0;
		int flag;
		int tmp;
		struct mdp_overlay ov;

		ov.id = obj->getHwOvId();
		if (ioctl(obj->getOvFd(), MSMFB_OVERLAY_GET, &ov)) {
			LOGE("%s: MSMFB_OVERLAY_GET error!", __FUNCTION__);
			return -errno;
		}

		/* set this overlay's parameter (talk to the h/w) */
		switch (param) {
			case OVERLAY_ROTATION_DEG:
				/* only 90 rotations are supported, the call fails
				 * for other values */
				LOGE("========= ROTATION request - not supported yet!");
				break;
			case OVERLAY_DITHER:
				break;
			case OVERLAY_TRANSFORM:
#ifdef USE_MSM_ROTATOR
				switch ( value ) {
					case 0:
						if (ov.user_data[0] & MDP_ROT_90) {
							tmp = ov.src_rect.y;
							ov.src_rect.y = ov.src.width - (ov.src_rect.x + ov.src_rect.w);
							ov.src_rect.x = tmp;
							overlay_src_swap(&ov);
							obj->rotator_dest_swap();
						}
						flag = MDP_ROT_NOP;
						ov.user_data[0] = flag;
						break;

					case OVERLAY_TRANSFORM_ROT_90:
						if (ov.user_data[0] == 0) {
							tmp = ov.src_rect.x;
							ov.src_rect.x = ov.src.height - (ov.src_rect.y + ov.src_rect.h);
							ov.src_rect.y = tmp;
							overlay_src_swap(&ov);
							obj->rotator_dest_swap();
						}
						flag = MDP_ROT_90;
						ov.user_data[0] = flag;
					   	break;
					default: return -EINVAL;
				}

				result = overlay_setRot(obj->getRotFd(), obj->getRot(), flag);

				if (ioctl(obj->getOvFd(), MSMFB_OVERLAY_SET, &ov))
					LOGE("%s: MSMFB_OVERLAY_SET error!", __FUNCTION__);
#endif
				result = -EINVAL;
				break;
			default:
				result = -EINVAL;
				break;
		}
		return result;
	}

	static int overlay_control_close(struct hw_device_t *dev)
	{
		struct overlay_control_context_t* ctx = (struct overlay_control_context_t*)dev;
		if (ctx) {
			/* free all resources associated with this device here
			 * in particular the overlay_handle_t, outstanding overlay_t, etc...
			 */
			free(ctx);
		}
		return 0;
	}

// ****************************************************************************
// Data module
// ****************************************************************************

	int overlay_initialize(struct overlay_data_device_t *dev,
						   overlay_handle_t handle)
	{
		/*
		 * overlay_handle_t should contain all the information to "inflate" this
		 * overlay. Typically it'll have a file descriptor, informations about
		 * how many buffers are there, etc...
		 * It is also the place to mmap all buffers associated with this overlay
		 * (see getBufferAddress).
		 *
		 * NOTE: this function doesn't take ownership of overlay_handle_t
		 *
		 */

		struct overlay_data_context_t* ctx = (struct overlay_data_context_t*)dev;
		int i;

		ctx->od.data.memory_id = -1;
		ctx->od.id = handle_get_ovId(handle);
		ctx->od_rot.id = handle_get_ovId(handle);

		ctx->mFD = open("/dev/graphics/fb0", O_RDWR, 0);
		if (ctx->mFD < 0) {
			LOGE("%s: error opening frame buffer!", __FUNCTION__);
			return -errno;
		}
#ifndef USE_MSM_ROTATOR
                return 0;
#else
		ctx->pmem = open("/dev/pmem_adsp", O_RDWR | O_SYNC);
		if (ctx->pmem < 0) {
			LOGE("Could not open pmem device!\n");
			goto ov_init_fail;
		}

		ctx->od_rot.data.memory_id = ctx->pmem;
		ctx->pmem_offset = handle_get_size(handle);
		ctx->pmem_addr = (void *) mmap(NULL, ctx->pmem_offset * 2, PROT_READ | PROT_WRITE,
									   MAP_SHARED, ctx->pmem, 0);
		if (ctx->pmem_addr == MAP_FAILED) {
			LOGE("%s(): pmem mmap() failed size=(%d)x2", __FUNCTION__, ctx->pmem_offset);
			goto ov_init_fail2;
		}

		ctx->rotator = open("/dev/msm_rotator", O_RDWR, 0);
		if (ctx->rotator < 0) {
			LOGE("%s: error opening rotator device!", __FUNCTION__);
			goto ov_init_fail3;
		}

		ctx->rot.session_id = handle_get_rotId(handle);
		ctx->rot.dst.memory_id = ctx->pmem;
		ctx->rot.dst.offset = 0;
		return 0;

		ov_init_fail3:
		munmap(ctx->pmem_addr, ctx->pmem_offset * 2);
		ov_init_fail2:
		close(ctx->pmem);
		ov_init_fail:
		close(ctx->mFD);
		return -1;
#endif
	}

	int overlay_dequeueBuffer(struct overlay_data_device_t *dev,
							  overlay_buffer_t* buf)
	{
		/* blocks until a buffer is available and return an opaque structure
		 * representing this buffer.
		 */

		/* no interal overlay buffer to dequeue */
		LOGE("%s: no buffer to dequeue ...\n", __FUNCTION__);

		return 0;
	}

	int overlay_queueBuffer(struct overlay_data_device_t *dev,
							overlay_buffer_t buffer)
	{
		/* Mark this buffer for posting and recycle or free overlay_buffer_t. */
		struct overlay_data_context_t *ctx = (struct overlay_data_context_t*)dev;
		struct msmfb_overlay_data *odPtr;
		int result=-1;

		if (ctx->od.data.memory_id == -1) {
			LOGE("%s: src memory_id is not set!", __FUNCTION__);
			return result;
		}

		ctx->rot.src.memory_id = ctx->od.data.memory_id;
		ctx->rot.src.offset = (uint32_t) buffer;
		ctx->rot.dst.offset = (ctx->rot.dst.offset) ? 0 : ctx->pmem_offset;

#ifdef USE_MSM_ROTATOR
		result = ioctl(ctx->rotator, MSM_ROTATOR_IOCTL_ROTATE, &ctx->rot);
#endif
		if (!result) {
			ctx->od_rot.data.offset = (uint32_t) ctx->rot.dst.offset;
			odPtr = &ctx->od_rot;
		} else {
			ctx->od.data.offset = (uint32_t) buffer;
			odPtr = &ctx->od;
		}


		if (ioctl(ctx->mFD, MSMFB_OVERLAY_PLAY, odPtr)) {
			LOGE("%s: MSMFB_OVERLAY_PLAY error!", __FUNCTION__);
			return -errno;
		}

		return result;
	}

	int overlay_setFd(struct overlay_data_device_t *dev, int fd)
	{
		struct overlay_data_context_t* ctx = (struct overlay_data_context_t*)dev;

		ctx->od.data.memory_id = fd;
		return 0;
	}

	static int overlay_setCrop(struct overlay_data_device_t *dev, uint32_t x,
                           uint32_t y, uint32_t w, uint32_t h)
	{
		int tmp;
		struct mdp_overlay ov;
		struct overlay_data_context_t* ctx = (struct overlay_data_context_t*)dev;

		ov.id = ctx->od.id;
		if (ioctl(ctx->mFD, MSMFB_OVERLAY_GET, &ov)) {
			LOGE("%s: MSMFB_OVERLAY_GET error!", __FUNCTION__);
			return -errno;
		}

		if (ov.user_data[0] == MDP_ROT_90) {
			tmp = x;
			x = ov.src.width - (y + h);
			y = tmp;

			tmp = w;
			w = h;
			h = tmp;
		}

	    if ((ov.src_rect.x == x) &&
		(ov.src_rect.y == y) &&
		(ov.src_rect.w == w) &&
		(ov.src_rect.h == h))
	            return 0;

	    ov.src_rect.x = x;
	    ov.src_rect.y = y;
	    ov.src_rect.w = w;
	    ov.src_rect.h = h;

            if (ioctl(ctx->mFD, MSMFB_OVERLAY_SET, &ov)) {
                    LOGE("%s: MSMFB_OVERLAY_SET error!", __FUNCTION__);
		    return -errno;
            }

            return 0;
	}

	void *overlay_getBufferAddress(struct overlay_data_device_t *dev,
								   overlay_buffer_t buffer)
	{
		/* overlay buffers are coming from client */
		return( NULL );
	}

	int overlay_getBufferCount(struct overlay_data_device_t *dev)
	{
		return( 0 );
	}


	static int overlay_data_close(struct hw_device_t *dev)
	{
		struct overlay_data_context_t* ctx = (struct overlay_data_context_t*)dev;
		if (ctx) {
			/* free all resources associated with this device here
			 * in particular all pending overlay_buffer_t if needed.
			 *
			 * NOTE: overlay_handle_t passed in initialize() is NOT freed and
			 * its file descriptors are not closed (this is the responsibility
			 * of the caller).
			 */

			munmap(ctx->pmem_addr, ctx->pmem_offset * 2);
#ifdef USE_MSM_ROTATOR
			close(ctx->rotator);
			close(ctx->pmem);
#endif
			close(ctx->mFD);
			free(ctx);
		}
		return 0;
	}

/*****************************************************************************/

	static int overlay_device_open(const struct hw_module_t* module, const char* name,
								   struct hw_device_t** device)
	{
		int status = -EINVAL;

		if (!strcmp(name, OVERLAY_HARDWARE_CONTROL)) {
			struct overlay_control_context_t *dev;
			dev = (overlay_control_context_t*)malloc(sizeof(*dev));

			if (!dev)
				return status;

			/* initialize our state here */
			memset(dev, 0, sizeof(*dev));

			/* initialize the procs */
			dev->device.common.tag = HARDWARE_DEVICE_TAG;
			dev->device.common.version = 0;
			dev->device.common.module = const_cast<hw_module_t*>(module);
			dev->device.common.close = overlay_control_close;

			dev->device.get = overlay_get;
			dev->device.createOverlay = overlay_createOverlay;
			dev->device.destroyOverlay = overlay_destroyOverlay;
			dev->device.setPosition = overlay_setPosition;
			dev->device.getPosition = overlay_getPosition;
			dev->device.setParameter = overlay_setParameter;
			dev->device.commit = overlay_commit;

			*device = &dev->device.common;
			status = 0;
		} else if (!strcmp(name, OVERLAY_HARDWARE_DATA)) {
			struct overlay_data_context_t *dev;
			dev = (overlay_data_context_t*)malloc(sizeof(*dev));

			if (!dev)
				return status;

			/* initialize our state here */
			memset(dev, 0, sizeof(*dev));

			/* initialize the procs */
			dev->device.common.tag = HARDWARE_DEVICE_TAG;
			dev->device.common.version = 0;
			dev->device.common.module = const_cast<hw_module_t*>(module);
			dev->device.common.close = overlay_data_close;

			dev->device.initialize = overlay_initialize;
	                dev->device.setCrop = overlay_setCrop;
			dev->device.dequeueBuffer = overlay_dequeueBuffer;
			dev->device.queueBuffer = overlay_queueBuffer;
			dev->device.setFd = overlay_setFd;
			dev->device.getBufferAddress = overlay_getBufferAddress;
			dev->device.getBufferCount = overlay_getBufferCount;

			*device = &dev->device.common;
			status = 0;
		}
		return status;
	}

/*
** Copyright 2008, Google Inc.
**
** Licensed under the Apache License, Version 2.0 (the "License"); 
** you may not use this file except in compliance with the License. 
** You may obtain a copy of the License at 
**
**     http://www.apache.org/licenses/LICENSE-2.0 
**
** Unless required by applicable law or agreed to in writing, software 
** distributed under the License is distributed on an "AS IS" BASIS, 
** WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. 
** See the License for the specific language governing permissions and 
** limitations under the License.
*/

#include <math.h>

// #define LOG_NDEBUG 0
#define LOG_TAG "AudioHardwareMSM72XX"
#include <utils/Log.h>
#include <utils/String8.h>

#include <stdio.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dlfcn.h>
#include <fcntl.h>

// hardware specific functions

#include "BluetoothHeadsetTable.h"
#include "AudioHardware.h"

#define LOG_SND_RPC 0  // Set to 1 to log sound RPC's

namespace android {

/* When this macro is non-zero, we initialize playback and recording
 * only once--when we construct the AudioHardwareMSM72xx class--and 
 * deinitialize them in this class' destructor.  When the macro is 
 * set to zero, we initialize playback and recording before we start
 * the respective operation, and deinitialize them after we stop that
 * operation.
 */
#define INIT_AUDIO_ONCE (0)
static int get_audpp_filter(void);
static int msm72xx_enable_audpp(uint16_t enable_mask);

static uint16_t adrc_flag;
static uint16_t eq_flag;
static uint16_t rx_iir_flag;
static bool audpp_filter_inited = false;

// ----------------------------------------------------------------------------

AudioHardware::AudioHardware() :
    mInit(false), mStandby(false), mOutputStandby(true),
    mMicMute(true), mBluetoothNrec(true), mBluetoothId(0), mOutput(0),
    mInput(0), mSndEndpoints(NULL),
    SND_DEVICE_CURRENT(-1),
    SND_DEVICE_HANDSET(-1),
    SND_DEVICE_SPEAKER(-1),            
    SND_DEVICE_BT(-1),
    SND_DEVICE_BT_EC_OFF(-1),
    SND_DEVICE_HEADSET(-1),
    SND_DEVICE_HEADSET_AND_SPEAKER(-1)
{
    if (get_audpp_filter() == 0)
        audpp_filter_inited = true;

#if INIT_AUDIO_ONCE
    if (msm72xx_init_record())
        LOGE("msm72xx_init_record failed");
    else 
#endif
    {
        int fd = open("/dev/msm_snd", O_RDWR);
        if (fd >= 0) {        
            int rc = ioctl(fd, SND_GET_ENDPOINTS, NULL);
            rc /= sizeof(snd_endpoint);
            if (rc >= 0) {
                mSndEndpoints = new snd_endpoint[rc];
                mNumSndEndpoints = rc;
                LOGI("++++ NUMBER OF ENDPOINTS: %d", rc);
                if (ioctl(fd, SND_GET_ENDPOINTS, mSndEndpoints) < 0) {
                    LOGE("Could not retrieve MSM SND endpoints.");
                }
                else {
                    mInit = true;
                    LOGV("constructed");                    
                    struct snd_endpoint *ept = mSndEndpoints;
                    for (int cnt = 0; cnt < mNumSndEndpoints; cnt++, ept++) {
                        LOGI("[%s] [%d]", ept->name, ept->id);
#define CHECK_FOR(desc) \
do { \
    if (!strcmp(ept->name, #desc)) { \
        LOGI("++++ [%s] [%d] HIT", ept->name, ept->id); \
        SND_DEVICE_##desc = ept->id; \
    } \
} while(0)
                        CHECK_FOR(CURRENT);
                        CHECK_FOR(HANDSET);
                        CHECK_FOR(SPEAKER);
                        CHECK_FOR(BT);
                        CHECK_FOR(BT_EC_OFF);
                        CHECK_FOR(HEADSET);
                        CHECK_FOR(HEADSET_AND_SPEAKER);
#undef CHECK_FOR
                        mBluetoothId = 0;
                        for (unsigned hs = 0; 
                                 hs < sizeof(KNOWN_HEADSETS) /
                                      sizeof(KNOWN_HEADSETS[0]);
                                 hs++) {
                            if (KNOWN_HEADSETS[hs].id < 0 &&
                                    !strcmp(ept->name,
                                            KNOWN_HEADSETS[hs].probe)) {
                                LOGI("++BT [%s] [%d] HIT", ept->name, ept->id);
                                KNOWN_HEADSETS[hs].id = ept->id;
                            }
                        }
                    }
                }
            }
            else LOGE("Could not retrieve number of MSM SND endpoints.");
            close(fd);
        }
        else LOGE("Could not open MSM SND driver.");
    }
}

AudioHardware::~AudioHardware()
{
    delete mInput;
    delete mOutput;
    delete [] mSndEndpoints;

#if INIT_AUDIO_ONCE
    msm72xx_deinit_record();
#endif
    mInit = false;
}

status_t AudioHardware::initCheck()
{
    return mInit ? NO_ERROR : NO_INIT;
}

AudioStreamOut* AudioHardware::openOutputStream(
        int format, int channelCount, uint32_t sampleRate)
{
    Mutex::Autolock lock(mLock);

    // only one output stream allowed
    if (mOutput) return 0;

    // create new output stream
    AudioStreamOutMSM72xx* out = new AudioStreamOutMSM72xx();
    if (out->set(this, format, channelCount, sampleRate) == NO_ERROR) {
        mOutput = out;
        standby_nosync();
    } else {
        delete out;
    }
    return mOutput;
}

void AudioHardware::closeOutputStream(AudioStreamOutMSM72xx* out) {
    Mutex::Autolock lock(mLock);
    if (mOutput != out) {
        LOGW("Attempt to close invalid output stream");
    }
    else {
        mOutput = 0;
    }
}

AudioStreamIn* AudioHardware::openInputStream(
        int format, int channelCount, uint32_t sampleRate)
{
    Mutex::Autolock lock(mLock);

    // input stream already open?
    if (mInput) return 0;

    AudioStreamInMSM72xx* in = new AudioStreamInMSM72xx();
    if (in->set(this, format, channelCount, sampleRate) != NO_ERROR) {
        delete in;
        return 0;
    }

    mInput = in;
    return mInput;
}

void AudioHardware::closeInputStream(AudioStreamInMSM72xx* in) {
    Mutex::Autolock lock(mLock);
    if (mInput != in) {
        LOGW("Attempt to close invalid input stream");
    }
    else {
        mInput = 0;
    }
}

// always call this with mutex held
status_t AudioHardware::standby_nosync()
{
    if (mStandby == false) {
        LOGD("Going to standby");
        mStandby = true;
        return mOutput->standby();
    }
    return NO_ERROR;
}

// always call with mutex held
status_t AudioHardware::checkStandby()
{
    // don't go into standby if audio is active
    if (!mOutputStandby) return NO_ERROR;
    return standby_nosync();
}

status_t AudioHardware::standby()
{
    Mutex::Autolock lock(mLock);
    mOutputStandby = true;
    return checkStandby();
}

status_t AudioHardware::setMicMute(bool state)
{
    Mutex::Autolock lock(mLock);
    return setMicMute_nosync(state);
}

// always call with mutex held
status_t AudioHardware::setMicMute_nosync(bool state)
{
    if (mMicMute != state) {
        mMicMute = state;
        return doAudioRouteOrMute(SND_DEVICE_CURRENT);
    }
    return NO_ERROR;
}

status_t AudioHardware::getMicMute(bool* state)
{
    *state = mMicMute;
    return NO_ERROR;
}

status_t AudioHardware::setParameter(const char *key, const char *value)
{
    LOGV("%s key = %s value = %s\n", __FUNCTION__, key, value);

    if (key == NULL || value == NULL) {
        LOGE("%s called with null argument, ignoring (key = %s, value = %s",
             __FUNCTION__, key, value);
        return BAD_VALUE;
    }

    const char BT_NREC_KEY[] = "bt_headset_nrec";
    const char BT_NAME_KEY[] = "bt_headset_name";
    const char BT_NREC_VALUE_ON[] = "on";

    if (!strncmp(key, BT_NREC_KEY, sizeof(BT_NREC_KEY))) {
        if (!strncmp(value, BT_NREC_VALUE_ON, sizeof(BT_NREC_VALUE_ON))) {
            mBluetoothNrec = true;
        } else {
            mBluetoothNrec = false;
            LOGI("Turning noise reduction and echo cancellation off for BT "
                 "headset");
        }
        doRouting();
    } else if (!strncmp(key, BT_NAME_KEY, sizeof(BT_NAME_KEY))) {
        mBluetoothId = 0;
        for (uint32_t i = 0; i < sizeof(KNOWN_HEADSETS) / sizeof(KNOWN_HEADSETS[0]);
                i++) {
            if (!strncasecmp(value, KNOWN_HEADSETS[i].name,
                             sizeof(KNOWN_HEADSETS[i].name)) &&
                KNOWN_HEADSETS[i].id >= 0) {
                mBluetoothId = KNOWN_HEADSETS[i].id;
                LOGI("Using custom acoustic parameters for %s", value);
                break;
            }
        }
        if (mBluetoothId == 0) {
            LOGI("Using default acoustic parameters "
                 "(%s not in acoustic database)", value);
            doRouting();
        }
    }

    return NO_ERROR;
}

int check_and_set_audpp_parameters(char *buf, int size)
{
    char *p, *ps;
    static const char *const seps = ",";
    int table_num;
    int i, j;
    uint16_t adrc_filter[8];
    eq_filter_type eq[12];
    rx_iir_filter iir_cfg;
    eqalizer eqalizer;
    int fd;
    void *audioeq;
    void *(*eq_cal)(int32_t, int32_t, int32_t, uint16_t, int32_t, int32_t *, int32_t *, uint16_t *);
    uint16_t numerator[6];
    uint16_t denominator[4];
    uint16_t shift[2];

    fd = open("/dev/msm_pcm_ctl", O_RDWR);
    if (fd < 0) {
        LOGE("Cannot open audio device");
        return -EPERM;
    }
    if (buf[0] == 'A' && buf[1] == '1') {
        /* IIR filter */
	if (!(p = strtok(buf, ",")))
            goto token_err;
	
	/* Table header */	
        table_num = strtol(p + 1, &ps, 10);
        if (!(p = strtok(NULL, seps)))
            goto token_err;
    	/* Table description */
    	if (!(p = strtok(NULL, seps)))
            goto token_err;

	for (i = 0; i < 48; i++) {
	    j = (i >= 40)? i : ((i % 2)? (i - 1) : (i + 1));
            iir_cfg.iir_params[j] = (uint16_t)strtol(p, &ps, 16);
	    if (!(p = strtok(NULL, seps)))
                goto token_err;
        }
        rx_iir_flag = (uint16_t)strtol(p, &ps, 16);
	if (!(p = strtok(NULL, seps)))
            goto token_err;
	iir_cfg.num_bands = (uint16_t)strtol(p, &ps, 16);
	
        if (ioctl(fd, AUDIO_SET_RX_IIR, &iir_cfg) < 0) {
            LOGE("set rx iir filter error.");
            return -EIO;
        }
    } else if (buf[0] == 'B' && buf[1] == '1') {
        /* This is the ADRC record we are looking for.  Tokenize it */
        if (!(p = strtok(buf, ",")))
            goto token_err;

        /* Table header */
        table_num = strtol(p + 1, &ps, 10);
        if (!(p = strtok(NULL, seps)))
            goto token_err;

        /* Table description */
        if (!(p = strtok(NULL, seps)))
            goto token_err;
        adrc_flag = (uint16_t)strtol(p, &ps, 16);

        if (!(p = strtok(NULL, seps)))
            goto token_err;
        adrc_filter[0] = (uint16_t)strtol(p, &ps, 16);

        if (!(p = strtok(NULL, seps)))
            goto token_err;
        adrc_filter[1] = (uint16_t)strtol(p, &ps, 16);

        if (!(p = strtok(NULL, seps)))
            goto token_err;
        adrc_filter[2] = (uint16_t)strtol(p, &ps, 16);

        if (!(p = strtok(NULL, seps)))
            goto token_err;
        adrc_filter[3] = (uint16_t)strtol(p, &ps, 16);

        if (!(p = strtok(NULL, seps)))
            goto token_err;
        adrc_filter[4] = (uint16_t)strtol(p, &ps, 16);

        if (!(p = strtok(NULL, seps)))
            goto token_err;
        adrc_filter[5] = (uint16_t)strtol(p, &ps, 16);

        if (!(p = strtok(NULL, seps)))
            goto token_err;
        adrc_filter[6] = (uint16_t)strtol(p, &ps, 16);

        if (!(p = strtok(NULL, seps)))
            goto token_err;
        adrc_filter[7] = (uint16_t)strtol(p, &ps, 16);

        if (!(p = strtok(NULL, seps)))
            goto token_err;

        LOGI("ADRC Filter ADRC FLAG = %02x.", adrc_flag);
        LOGI("ADRC Filter COMP THRESHOLD = %02x.", adrc_filter[0]);
        LOGI("ADRC Filter COMP SLOPE = %02x.", adrc_filter[1]);
        LOGI("ADRC Filter COMP RMS TIME = %02x.", adrc_filter[2]);
        LOGI("ADRC Filter COMP ATTACK[0] = %02x.", adrc_filter[3]);
        LOGI("ADRC Filter COMP ATTACK[1] = %02x.", adrc_filter[4]);
        LOGI("ADRC Filter COMP RELEASE[0] = %02x.", adrc_filter[5]);
        LOGI("ADRC Filter COMP RELEASE[1] = %02x.", adrc_filter[6]);
        LOGI("ADRC Filter COMP DELAY = %02x.", adrc_filter[7]);

        if (ioctl(fd, AUDIO_SET_ADRC, &adrc_filter) < 0) {
            LOGE("set adrc filter error.");
            return -EIO;
        }
    } else if (buf[0] == 'C' && buf[1] == '1') {
        /* This is the EQ record we are looking for.  Tokenize it */
        if (!(p = strtok(buf, ",")))
            goto token_err;

        /* Table header */
        table_num = strtol(p + 1, &ps, 10);
        if (!(p = strtok(NULL, seps)))
            goto token_err;
        /* Table description */
        if (!(p = strtok(NULL, seps)))
            goto token_err;

        eq_flag = (uint16_t)strtol(p, &ps, 16);
        if (!(p = strtok(NULL, seps)))
            goto token_err;
        LOGI("EQ flag = %02x.", eq_flag);

        audioeq = ::dlopen("/system/lib/libaudioeq.so", RTLD_NOW);
        if (audioeq == NULL) {
            LOGE("audioeq library open failure");
            return -1;
        }
        eq_cal = (void *(*) (int32_t, int32_t, int32_t, uint16_t, int32_t, int32_t *, int32_t *, uint16_t *))::dlsym(audioeq, "audioeq_calccoefs");
        memset(&eqalizer, 0, sizeof(eqalizer));
        /* Temp add the bands here */
        eqalizer.bands = 8;
        for (i = 0; i < eqalizer.bands; i++) {

            eq[i].gain = (uint16_t)strtol(p, &ps, 16);

            if (!(p = strtok(NULL, seps)))
                goto token_err;
            eq[i].freq = (uint16_t)strtol(p, &ps, 16);

            if (!(p = strtok(NULL, seps)))
                goto token_err;
            eq[i].type = (uint16_t)strtol(p, &ps, 16);

            if (!(p = strtok(NULL, seps)))
                goto token_err;
            eq[i].qf = (uint16_t)strtol(p, &ps, 16);

            if (!(p = strtok(NULL, seps)))
                goto token_err;
            //LOGI("gain[%d] = %d", i, eq[i].gain);
            //LOGI("freq[%d] = %d", i, eq[i].freq);
            //LOGI("type[%d] = %d", i, eq[i].type);
            //LOGI("  qf[%d] = %d", i, eq[i].qf);
            eq_cal(eq[i].gain, eq[i].freq, 48000, eq[i].type, eq[i].qf, (int32_t*)numerator, (int32_t *)denominator, shift);
            for (j = 0; j < 6; j++) {
                eqalizer.params[ ( i * 6) + j] = numerator[j];
            }
            for (j = 0; j < 4; j++) {
                eqalizer.params[(eqalizer.bands * 6) + (i * 4) + j] = denominator[j];
            }
            eqalizer.params[(eqalizer.bands * 10) + i] = shift[0];
        }
        ::dlclose(audioeq);

        if (ioctl(fd, AUDIO_SET_EQ, &eqalizer) < 0) {
            LOGE("set Equalizer error.");
            return -EIO;
        }
    }
    close(fd);
    return 0;

token_err:
    LOGE("malformatted pcm control buffer");
    return -EINVAL;
}

static int get_audpp_filter(void)
{
    struct stat st;
    char *read_buf;
    char *next_str, *current_str;
    int csvfd;

    LOGI("get_audpp_filter");
    static const char *const path = 
        "/system/etc/AudioFilter.csv";
    csvfd = open(path, O_RDONLY);
    if (csvfd < 0) {
        /* failed to open normal acoustic file ... */
        LOGE("failed to open AUDIO_NORMAL_FILTER %s: %s (%d).",
             path, strerror(errno), errno);
        return -1;
    } else LOGI("open %s success.", path);

    if (fstat(csvfd, &st) < 0) {
        LOGE("failed to stat %s: %s (%d).",
             path, strerror(errno), errno);
        close(csvfd);
        return -1;
    }

    read_buf = (char *) mmap(0, st.st_size,
                    PROT_READ | PROT_WRITE,
                    MAP_PRIVATE,
                    csvfd, 0);
    
    if (read_buf == MAP_FAILED) {
        LOGE("failed to mmap parameters file: %s (%d)",
             strerror(errno), errno);
        close(csvfd);
        return -1;
    }

    current_str = read_buf;

    while (1) {
        int len;
        next_str = strchr(current_str, '\n');
        if (!next_str)
           break;
        len = next_str - current_str;
        *next_str++ = '\0';
        if (check_and_set_audpp_parameters(current_str, len)) {
            LOGI("failed to set audpp parameters, exiting.");
            munmap(read_buf, st.st_size);
	    close(csvfd);
	    return -1;
        }
        current_str = next_str;
    }

    munmap(read_buf, st.st_size);
    close(csvfd);    
    return 0;
}

static int msm72xx_enable_audpp(uint16_t enable_mask)
{
    int fd;

    if (!audpp_filter_inited) return -EINVAL;
    
    fd = open("/dev/msm_pcm_ctl", O_RDWR);
    if (fd < 0) {
        LOGE("Cannot open audio device");
        return -EPERM;
    }

    if (adrc_flag == 0 && (enable_mask & ADRC_ENABLE))
       	enable_mask &= ~ADRC_ENABLE;
    if (eq_flag == 0 && (enable_mask & EQ_ENABLE))
	enable_mask &= ~EQ_ENABLE;
    if (rx_iir_flag == 0 && (enable_mask & IIR_ENABLE))
        enable_mask &= ~IIR_ENABLE;   
	
    LOGD("msm72xx_enable_audpp: 0x%04x", enable_mask);
    if (ioctl(fd, AUDIO_ENABLE_AUDPP, &enable_mask) < 0) {
        LOGE("enable audpp error");
        close(fd);
        return -EPERM;
    }
    
    close(fd);
    return 0;
}

static status_t set_volume_rpc(uint32_t device,
                               uint32_t method,
                               uint32_t volume)
{
    int fd;
#if LOG_SND_RPC
    LOGD("rpc_snd_set_volume(%d, %d, %d)\n", device, method, volume);
#endif

    if (device == -1UL) return NO_ERROR;

    fd = open("/dev/msm_snd", O_RDWR);
    if (fd < 0) {
        LOGE("Can not open snd device");
        return -EPERM;
    }
    /* rpc_snd_set_volume(
     *     device,            # Any hardware device enum, including
     *                        # SND_DEVICE_CURRENT
     *     method,            # must be SND_METHOD_VOICE to do anything useful
     *     volume,            # integer volume level, in range [0,5].
     *                        # note that 0 is audible (not quite muted)
     *  )
     * rpc_snd_set_volume only works for in-call sound volume.
     */
    struct snd_volume_config args;
    args.device = device;
    args.method = method;
    args.volume = volume;

    if (ioctl(fd, SND_SET_VOLUME, &args) < 0) {
        LOGE("snd_set_volume error.");
        close(fd);
        return -EIO;
    }

    close(fd);
    return NO_ERROR;
}

status_t AudioHardware::setVoiceVolume(float v)
{
    if (v < 0.0) {
        LOGW("setVoiceVolume(%f) under 0.0, assuming 0.0\n", v);
        v = 0.0;
    } else if (v > 1.0) {
        LOGW("setVoiceVolume(%f) over 1.0, assuming 1.0\n", v);
        v = 1.0;
    }

    int vol = lrint(v * 5.0);
    LOGD("setVoiceVolume(%f)\n", v);
    LOGI("Setting in-call volume to %d (available range is 0 to 5)\n", vol);

    set_volume_rpc(SND_DEVICE_CURRENT, SND_METHOD_VOICE, vol);
    return NO_ERROR;
}

status_t AudioHardware::setMasterVolume(float v)
{
    Mutex::Autolock lock(mLock);
    int vol = ceil(v * 5.0);
    LOGI("Set master volume to %d.\n", vol);

    set_volume_rpc(SND_DEVICE_HANDSET, SND_METHOD_VOICE, vol);
    set_volume_rpc(SND_DEVICE_SPEAKER, SND_METHOD_VOICE, vol);
    set_volume_rpc(SND_DEVICE_BT,      SND_METHOD_VOICE, vol);
    set_volume_rpc(SND_DEVICE_HEADSET, SND_METHOD_VOICE, vol);
    // We return an error code here to let the audioflinger do in-software
    // volume on top of the maximum volume that we set through the SND API.
    // return error - software mixer will handle it
    return -1;
}

static status_t do_route_audio_rpc(uint32_t device,
                                   bool ear_mute, bool mic_mute)
{
    if (device == -1UL)
        return NO_ERROR;

    int fd;
#if LOG_SND_RPC
    LOGD("rpc_snd_set_device(%d, %d, %d)\n", device, ear_mute, mic_mute);
#endif

    fd = open("/dev/msm_snd", O_RDWR);
    if (fd < 0) {
        LOGE("Can not open snd device");
        return -EPERM;
    }
    // RPC call to switch audio path
    /* rpc_snd_set_device(
     *     device,            # Hardware device enum to use
     *     ear_mute,          # Set mute for outgoing voice audio
     *                        # this should only be unmuted when in-call
     *     mic_mute,          # Set mute for incoming voice audio
     *                        # this should only be unmuted when in-call or
     *                        # recording.
     *  )
     */
    struct snd_device_config args;
    args.device = device;
    args.ear_mute = ear_mute ? SND_MUTE_MUTED : SND_MUTE_UNMUTED;
    args.mic_mute = mic_mute ? SND_MUTE_MUTED : SND_MUTE_UNMUTED;

    if (ioctl(fd, SND_SET_DEVICE, &args) < 0) {
        LOGE("snd_set_device error.");
        close(fd);
        return -EIO;
    }

    close(fd);
    return NO_ERROR;
}

// always call with mutex held
status_t AudioHardware::doAudioRouteOrMute(uint32_t device)
{
    if (device == (uint32_t)SND_DEVICE_BT) {
        if (mBluetoothId) {
            device = mBluetoothId;
        } else if (!mBluetoothNrec) {
            device = SND_DEVICE_BT_EC_OFF;
        }
    }
    return do_route_audio_rpc(device,
            mMode != AudioSystem::MODE_IN_CALL, mMicMute);
}

static int count_bits(uint32_t vector)
{
    int bits;
    for (bits = 0; vector; bits++) {
        vector &= vector - 1;
    }
    return bits;
}

status_t AudioHardware::doRouting()
{
    Mutex::Autolock lock(mLock);
    uint32_t routes = mRoutes[mMode];
    if (count_bits(routes) > 1) {
        if (routes !=
                (AudioSystem::ROUTE_HEADSET | AudioSystem::ROUTE_SPEAKER)) {
            LOGW("Hardware does not support requested route combination (%#X),"
                 " picking closest possible route...", routes);
        }
    }

    status_t ret = NO_ERROR;
    if (routes & AudioSystem::ROUTE_BLUETOOTH) {
        LOGI("Routing audio to Bluetooth PCM\n");
        ret = doAudioRouteOrMute(SND_DEVICE_BT);
        msm72xx_enable_audpp(ADRC_DISABLE | EQ_DISABLE | IIR_DISABLE);
    } else if ((routes & AudioSystem::ROUTE_HEADSET) &&
               (routes & AudioSystem::ROUTE_SPEAKER)) {
        LOGI("Routing audio to Wired Headset and Speaker\n");
        ret = doAudioRouteOrMute(SND_DEVICE_HEADSET_AND_SPEAKER);
        msm72xx_enable_audpp(ADRC_ENABLE | EQ_ENABLE | IIR_ENABLE);
    } else if (routes & AudioSystem::ROUTE_HEADSET) {
        LOGI("Routing audio to Wired Headset\n");
        ret = doAudioRouteOrMute(SND_DEVICE_HEADSET);
        msm72xx_enable_audpp(ADRC_DISABLE | EQ_DISABLE | IIR_DISABLE);
    } else if (routes & AudioSystem::ROUTE_SPEAKER) {
        LOGI("Routing audio to Speakerphone\n");
        ret = doAudioRouteOrMute(SND_DEVICE_SPEAKER);
        msm72xx_enable_audpp(ADRC_ENABLE | EQ_ENABLE | IIR_ENABLE);
    } else {
        LOGI("Routing audio to Handset\n");
        ret = doAudioRouteOrMute(SND_DEVICE_HANDSET);
        msm72xx_enable_audpp(ADRC_DISABLE | EQ_DISABLE | IIR_DISABLE);
    }

    // check for standby
    checkStandby();

    return ret;
}

status_t AudioHardware::checkMicMute()
{
    Mutex::Autolock lock(mLock);
    if (mMode != AudioSystem::MODE_IN_CALL) {
        setMicMute_nosync(true);
    }
    
    return NO_ERROR;
}

status_t AudioHardware::dumpInternals(int fd, const Vector<String16>& args)
{
    const size_t SIZE = 256;
    char buffer[SIZE];
    String8 result;
    result.append("AudioHardware::dumpInternals\n");
    snprintf(buffer, SIZE, "\tmInit: %s\n", mInit? "true": "false");
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmStandby: %s\n", mStandby? "true": "false");
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmOutputStandby: %s\n", mOutputStandby? "true": "false");
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmMicMute: %s\n", mMicMute? "true": "false");
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmBluetoothNrec: %s\n", mBluetoothNrec? "true": "false");
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmBluetoothId: %d\n", mBluetoothId);
    result.append(buffer);
    ::write(fd, result.string(), result.size());
    return NO_ERROR;
}

status_t AudioHardware::dump(int fd, const Vector<String16>& args)
{
    dumpInternals(fd, args);
    if (mInput) {
        mInput->dump(fd, args);
    }
    if (mOutput) {
        mOutput->dump(fd, args);
    }
    return NO_ERROR;
}

// ----------------------------------------------------------------------------

AudioHardware::AudioStreamOutMSM72xx::AudioStreamOutMSM72xx() :
    mHardware(0), mFd(-1), mStartCount(0), mRetryCount(0)
{
}

status_t AudioHardware::AudioStreamOutMSM72xx::set(
        AudioHardware* hw, int format, int channels, uint32_t rate)
{
    // fix up defaults
    if (format == 0) format = AudioSystem::PCM_16_BIT;
    if (channels == 0) channels = channelCount();
    if (rate == 0) rate = sampleRate();

    // check values
    if ((format != AudioSystem::PCM_16_BIT) ||
            (channels != channelCount()) ||
            (rate != sampleRate()))
        return BAD_VALUE;

    mHardware = hw;

    return NO_ERROR;
}

AudioHardware::AudioStreamOutMSM72xx::~AudioStreamOutMSM72xx()
{
    if (!mHardware->mStandby) {
        if (mFd > 0) close(mFd);
    }
    mHardware->closeOutputStream(this);
}

ssize_t AudioHardware::AudioStreamOutMSM72xx::write(const void* buffer, size_t bytes)
{
    // LOGD("AudioStreamOutMSM72xx::write(%p, %u)", buffer, bytes);
    status_t status = NO_INIT;
    size_t count = bytes;
    const uint8_t* p = static_cast<const uint8_t*>(buffer);

    mHardware->mOutputStandby = false;
    if (mHardware->mStandby) {

        // open driver
        LOGV("open driver");
        status = ::open("/dev/msm_pcm_out", O_RDWR);
        if (status < 0) {
            LOGE("Cannot open audio device");
            goto Error;
        }
        mFd = status;

        // configuration
        LOGV("get config");
        struct msm_audio_config config;
        status = ioctl(mFd, AUDIO_GET_CONFIG, &config);
        if (status < 0) {
            LOGE("Cannot read config");
            goto Error;
        }

        LOGV("set config");
        config.channel_count = channelCount();
        config.sample_rate = sampleRate();
        config.buffer_size = bufferSize();
        config.buffer_count = 2;
        config.codec_type = CODEC_TYPE_PCM;
        status = ioctl(mFd, AUDIO_SET_CONFIG, &config);
        if (status < 0) {
            LOGE("Cannot set config");
            goto Error;
        }

        LOGV("buffer_size: %u", config.buffer_size);
        LOGV("buffer_count: %u", config.buffer_count);
        LOGV("channel_count: %u", config.channel_count);
        LOGV("sample_rate: %u", config.sample_rate);

        // fill 2 buffers before AUDIO_START
        mStartCount = 2;
        mHardware->mStandby = false;
    }

    while (count) {
        ssize_t written = ::write(mFd, p, count);
        if (written >= 0) {
            count -= written;
            p += written;
        } else {
            if (errno != EAGAIN) return written;
            mRetryCount++;
            LOGW("EAGAIN - retry");
        }
    }

    // start audio after we fill 2 buffers
    if (mStartCount) {
        if (--mStartCount == 0) {
            ioctl(mFd, AUDIO_START, 0);
        }
    }
    return bytes;

Error:
    if (mFd > 0) {
        ::close(mFd);
        mFd = -1;
    }
    // Simulate audio output timing in case of error
    usleep(bytes * 1000000 / sizeof(int16_t) / channelCount() / sampleRate());

    return status;
}

status_t AudioHardware::AudioStreamOutMSM72xx::standby()
{
    status_t status = NO_ERROR;
    if (mFd > 0) {
        ::close(mFd);
        mFd = -1;
    }
    mHardware->mStandby = true;
    return status;
}

status_t AudioHardware::AudioStreamOutMSM72xx::dump(int fd, const Vector<String16>& args)
{
    const size_t SIZE = 256;
    char buffer[SIZE];
    String8 result;
    result.append("AudioStreamOutMSM72xx::dump\n");
    snprintf(buffer, SIZE, "\tsample rate: %d\n", sampleRate());
    result.append(buffer);
    snprintf(buffer, SIZE, "\tbuffer size: %d\n", bufferSize());
    result.append(buffer);
    snprintf(buffer, SIZE, "\tchannel count: %d\n", channelCount());
    result.append(buffer);
    snprintf(buffer, SIZE, "\tformat: %d\n", format());
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmHardware: %p\n", mHardware);
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmFd: %d\n", mFd);
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmStartCount: %d\n", mStartCount);
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmRetryCount: %d\n", mRetryCount);
    result.append(buffer);
    ::write(fd, result.string(), result.size());
    return NO_ERROR;
}

// ----------------------------------------------------------------------------

AudioHardware::AudioStreamInMSM72xx::AudioStreamInMSM72xx() :
    mHardware(0), mFd(-1), mRecordEnabled(false), mRetryCount(0)
{
}

status_t AudioHardware::AudioStreamInMSM72xx::set(
        AudioHardware* hw, int format, int channelCount, uint32_t sampleRate)
{
    LOGV("AudioStreamInMSM72xx::set(%d, %d, %u)", format, channelCount, sampleRate);
    if (mFd >= 0) {
        LOGE("Audio record already open");
        return -EPERM;
    }

    // open audio input device
    status_t status = ::open("/dev/msm_pcm_in", O_RDWR);
    if (status < 0) {
        LOGE("Cannot open audio device");
        goto Error;
    }
    mFd = status;

    // configuration
    LOGV("get config");
    struct msm_audio_config config;
    status = ioctl(mFd, AUDIO_GET_CONFIG, &config);
    if (status < 0) {
        LOGE("Cannot read config");
        goto Error;
    }

    LOGV("set config");
    config.channel_count = channelCount;
    config.sample_rate = sampleRate;
    config.buffer_size = bufferSize();
    config.buffer_count = 2;
    config.codec_type = CODEC_TYPE_PCM;
    status = ioctl(mFd, AUDIO_SET_CONFIG, &config);
    if (status < 0) {
        LOGE("Cannot set config");
        goto Error;
    }

    LOGV("confirm config");
    status = ioctl(mFd, AUDIO_GET_CONFIG, &config);
    if (status < 0) {
        LOGE("Cannot read config");
        goto Error;
    }
    LOGV("buffer_size: %u", config.buffer_size);
    LOGV("buffer_count: %u", config.buffer_count);
    LOGV("channel_count: %u", config.channel_count);
    LOGV("sample_rate: %u", config.sample_rate);

    mHardware = hw;
    mHardware->setMicMute_nosync(false);
    mRecordEnabled = false;
    return NO_ERROR;

Error:
    if (mFd > 0) {
        ::close(mFd);
        mFd = -1;
    }
    return status;
}

AudioHardware::AudioStreamInMSM72xx::~AudioStreamInMSM72xx()
{
    LOGV("AudioHardware::closeRecord");
    if (mFd < 0) return;
    mHardware->checkMicMute();
    ::close(mFd);
    mRecordEnabled = false;
    mHardware->closeInputStream(this);
}

ssize_t AudioHardware::AudioStreamInMSM72xx::read( void* buffer, ssize_t bytes)
{
    LOGV("AudioStreamInMSM72xx::read(%p, %u)", buffer, bytes);
    if (mFd < 0) return NO_INIT;

    size_t count = bytes;
    uint8_t* p = static_cast<uint8_t*>(buffer);

    if (!mRecordEnabled) {
        if (ioctl(mFd, AUDIO_START, 0)) {
            LOGE("Error starting record");
            return -1;
        }
        mRecordEnabled = 1;
    }

    while (count) {
        ssize_t bytesRead = ::read(mFd, buffer, count);
        if (bytesRead >= 0) {
            count -= bytesRead;
            p += bytesRead;
        } else {
            if (errno != EAGAIN) return bytesRead;
            mRetryCount++;
            LOGW("EAGAIN - retrying");
        }
    }
    return bytes;
}

status_t AudioHardware::AudioStreamInMSM72xx::dump(int fd, const Vector<String16>& args)
{
    const size_t SIZE = 256;
    char buffer[SIZE];
    String8 result;
    result.append("AudioStreamInMSM72xx::dump\n");
    snprintf(buffer, SIZE, "\tsample rate: %d\n", sampleRate());
    result.append(buffer);
    snprintf(buffer, SIZE, "\tbuffer size: %d\n", bufferSize());
    result.append(buffer);
    snprintf(buffer, SIZE, "\tchannel count: %d\n", channelCount());
    result.append(buffer);
    snprintf(buffer, SIZE, "\tformat: %d\n", format());
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmHardware: %p\n", mHardware);
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmFd count: %d\n", mFd);
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmRecordEnabled: %s\n", mRecordEnabled? "true": "false");
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmRetryCount: %d\n", mRetryCount);
    result.append(buffer);
    ::write(fd, result.string(), result.size());
    return NO_ERROR;
}

// ----------------------------------------------------------------------------

extern "C" AudioHardwareInterface* createAudioHardware(void) {
    return new AudioHardware();
}

}; // namespace android

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

//#define LOG_NDEBUG 0
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

#include "AudioHardware.h"
#include <media/AudioRecord.h>

#define LOG_SND_RPC 0  // Set to 1 to log sound RPC's

namespace android {
static int audpre_index, tx_iir_index;
static void * acoustic;
const uint32_t AudioHardware::inputSamplingRates[] = {
        8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000
};
// ----------------------------------------------------------------------------

AudioHardware::AudioHardware() :
    mInit(false), mMicMute(true), mBluetoothNrec(true), mBluetoothId(0),
    mOutput(0), mInput(0), mSndEndpoints(NULL),
    SND_DEVICE_CURRENT(-1),
    SND_DEVICE_HANDSET(-1),
    SND_DEVICE_SPEAKER(-1),
    SND_DEVICE_BT(-1),
    SND_DEVICE_BT_EC_OFF(-1),
    SND_DEVICE_HEADSET(-1),
    SND_DEVICE_HEADSET_AND_SPEAKER(-1)
{

    int (*snd_get_num)();
    int (*snd_get_endpoint)(int, msm_snd_endpoint *);
    int (*set_acoustic_parameters)();

    struct msm_snd_endpoint *ept;

    acoustic = ::dlopen("/system/lib/libhtc_acoustic.so", RTLD_NOW);
    if (acoustic == NULL ) {
        LOGE("Could not open libhtc_acoustic.so");
        return;
    }

    set_acoustic_parameters = (int (*)(void))::dlsym(acoustic, "set_acoustic_parameters");
    if ((*set_acoustic_parameters) == 0 ) {
        LOGE("Could not open set_acoustic_parameters()");
        return;
    }

    int rc = set_acoustic_parameters();
    if (rc < 0) {
        LOGE("Could not set acoustic parameters to share memory: %d", rc);
//        return;
    }

    snd_get_num = (int (*)(void))::dlsym(acoustic, "snd_get_num_endpoints");
    if ((*snd_get_num) == 0 ) {
        LOGE("Could not open snd_get_num()");
//        return;
    }

    mNumSndEndpoints = snd_get_num();
    LOGD("mNumSndEndpoints = %d", mNumSndEndpoints);
    mSndEndpoints = new msm_snd_endpoint[mNumSndEndpoints];
    mInit = true;
    LOGV("constructed %d SND endpoints)", mNumSndEndpoints);
    ept = mSndEndpoints;
    snd_get_endpoint = (int (*)(int, msm_snd_endpoint *))::dlsym(acoustic, "snd_get_endpoint");
    if ((*snd_get_endpoint) == 0 ) {
        LOGE("Could not open snd_get_endpoint()");
        return;
    }

    for (int cnt = 0; cnt < mNumSndEndpoints; cnt++, ept++) {
        ept->id = cnt;
        snd_get_endpoint(cnt, ept);
#define CHECK_FOR(desc) \
        if (!strcmp(ept->name, #desc)) { \
            SND_DEVICE_##desc = ept->id; \
            LOGD("BT MATCH " #desc); \
        } else
        CHECK_FOR(CURRENT)
        CHECK_FOR(HANDSET)
        CHECK_FOR(SPEAKER)
        CHECK_FOR(BT)
        CHECK_FOR(BT_EC_OFF)
        CHECK_FOR(HEADSET)
        CHECK_FOR(HEADSET_AND_SPEAKER) {}
#undef CHECK_FOR
    }
}

AudioHardware::~AudioHardware()
{
    delete mInput;
    delete mOutput;
    delete [] mSndEndpoints;
    ::dlclose(acoustic);
    mInit = false;
}

status_t AudioHardware::initCheck()
{
    return mInit ? NO_ERROR : NO_INIT;
}

AudioStreamOut* AudioHardware::openOutputStream(
        int format, int channelCount, uint32_t sampleRate, status_t *status)
{
    Mutex::Autolock lock(mLock);

    // only one output stream allowed
    if (mOutput) {
        if (status) {
            *status = INVALID_OPERATION;
        }
        return 0;
    }

    // create new output stream
    AudioStreamOutMSM72xx* out = new AudioStreamOutMSM72xx();
    status_t lStatus = out->set(this, format, channelCount, sampleRate);
    if (status) {
        *status = lStatus;
    }
    if (lStatus == NO_ERROR) {
        mOutput = out;
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
        int inputSource, int format, int channelCount, uint32_t sampleRate,
        status_t *status, AudioSystem::audio_in_acoustics acoustic_flags)
{
    // check for valid input source
    if ((inputSource < AudioRecord::DEFAULT_INPUT) ||
        (inputSource >= AudioRecord::NUM_INPUT_SOURCES)) {
        return 0;
    }

    mLock.lock();
    // input stream already open?
    if (mInput) {
        if (status) {
            *status = INVALID_OPERATION;
        }
        mLock.unlock();
        return 0;
    }

    AudioStreamInMSM72xx* in = new AudioStreamInMSM72xx();
    status_t lStatus = in->set(this, format, channelCount, sampleRate, acoustic_flags);
    if (status) {
        *status = lStatus;
    }
    if (lStatus != NO_ERROR) {
        mLock.unlock();
        delete in;
        return 0;
    }

    mInput = in;
    mLock.unlock();

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

bool AudioHardware::checkOutputStandby()
{
    if (mOutput)
        if (!mOutput->checkStandby())
            return false;

    return true;
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
        for (int i = 0; i < mNumSndEndpoints; i++) {
            if (!strcasecmp(value, mSndEndpoints[i].name)) {
                mBluetoothId = mSndEndpoints[i].id;
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
static unsigned calculate_audpre_table_index(unsigned index)
{
    switch (index) {
        case 48000:    return SAMP_RATE_INDX_48000;
        case 44100:    return SAMP_RATE_INDX_44100;
        case 32000:    return SAMP_RATE_INDX_32000;
        case 24000:    return SAMP_RATE_INDX_24000;
        case 22050:    return SAMP_RATE_INDX_22050;
        case 16000:    return SAMP_RATE_INDX_16000;
        case 12000:    return SAMP_RATE_INDX_12000;
        case 11025:    return SAMP_RATE_INDX_11025;
        case 8000:    return SAMP_RATE_INDX_8000;
        default:     return -1;
    }
}
size_t AudioHardware::getInputBufferSize(uint32_t sampleRate, int format, int channelCount)
{
    if (checkInputSampleRate(sampleRate) != NO_ERROR) {
        LOGW("getInputBufferSize bad sampling rate: %d", sampleRate);
        return 0;
    }
    if (format != AudioSystem::PCM_16_BIT) {
        LOGW("getInputBufferSize bad format: %d", format);
        return 0;
    }
    if (channelCount < 1 || channelCount > 2) {
        LOGW("getInputBufferSize bad channel count: %d", channelCount);
        return 0;
    }

    return 2048*channelCount;
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
     struct msm_snd_volume_config args;
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

    Mutex::Autolock lock(mLock);
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
    struct msm_snd_device_config args;
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
    int (*msm72xx_enable_audpp)(int);
    msm72xx_enable_audpp = (int (*)(int))::dlsym(acoustic, "msm72xx_enable_audpp");
    status_t ret = NO_ERROR;
    if (routes & AudioSystem::ROUTE_BLUETOOTH_SCO) {
        LOGI("Routing audio to Bluetooth PCM\n");
        ret = doAudioRouteOrMute(SND_DEVICE_BT);
        msm72xx_enable_audpp(ADRC_DISABLE | EQ_DISABLE | RX_IIR_DISABLE);
    } else if ((routes & AudioSystem::ROUTE_HEADSET) &&
               (routes & AudioSystem::ROUTE_SPEAKER)) {
        LOGI("Routing audio to Wired Headset and Speaker\n");
        ret = doAudioRouteOrMute(SND_DEVICE_HEADSET_AND_SPEAKER);
        msm72xx_enable_audpp(ADRC_ENABLE | EQ_ENABLE | RX_IIR_ENABLE);
    } else if (routes & AudioSystem::ROUTE_HEADSET) {
        LOGI("Routing audio to Wired Headset\n");
        ret = doAudioRouteOrMute(SND_DEVICE_HEADSET);
        msm72xx_enable_audpp(ADRC_DISABLE | EQ_DISABLE | RX_IIR_DISABLE);
    } else if (routes & AudioSystem::ROUTE_SPEAKER) {
        LOGI("Routing audio to Speakerphone\n");
        ret = doAudioRouteOrMute(SND_DEVICE_SPEAKER);
        msm72xx_enable_audpp(ADRC_ENABLE | EQ_ENABLE | RX_IIR_ENABLE);
    } else {
        LOGI("Routing audio to Handset\n");
        ret = doAudioRouteOrMute(SND_DEVICE_HANDSET);
        msm72xx_enable_audpp(ADRC_DISABLE | EQ_DISABLE | RX_IIR_DISABLE);
    }

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

status_t AudioHardware::checkInputSampleRate(uint32_t sampleRate)
{
    for (uint32_t i = 0; i < sizeof(inputSamplingRates)/sizeof(uint32_t); i++) {
        if (sampleRate == inputSamplingRates[i]) {
            return NO_ERROR;
        } 
    }
    return BAD_VALUE;
}

// ----------------------------------------------------------------------------

AudioHardware::AudioStreamOutMSM72xx::AudioStreamOutMSM72xx() :
    mHardware(0), mFd(-1), mStartCount(0), mRetryCount(0), mStandby(true)
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
    if (mFd > 0) close(mFd);
    mHardware->closeOutputStream(this);
}

ssize_t AudioHardware::AudioStreamOutMSM72xx::write(const void* buffer, size_t bytes)
{
    // LOGD("AudioStreamOutMSM72xx::write(%p, %u)", buffer, bytes);
    status_t status = NO_INIT;
    size_t count = bytes;
    const uint8_t* p = static_cast<const uint8_t*>(buffer);

    if (mStandby) {

        // open driver
        LOGV("open driver");
        status = ::open("/dev/msm_pcm_out", O_RDWR);
        if (status < 0) {
            LOGE("Cannot open /dev/msm_pcm_out errno: %d", errno);
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
        config.buffer_count = AUDIO_HW_NUM_OUT_BUF;
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
        mStartCount = AUDIO_HW_NUM_OUT_BUF;
        mStandby = false;
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
    usleep(bytes * 1000000 / frameSize() / sampleRate());

    return status;
}

status_t AudioHardware::AudioStreamOutMSM72xx::standby()
{
    status_t status = NO_ERROR;
    if (!mStandby && mFd > 0) {
        ::close(mFd);
        mFd = -1;
    }
    mStandby = true;
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
    snprintf(buffer, SIZE, "\tmStandby: %s\n", mStandby? "true": "false");
    result.append(buffer);
    ::write(fd, result.string(), result.size());
    return NO_ERROR;
}

bool AudioHardware::AudioStreamOutMSM72xx::checkStandby()
{
    return mStandby;
}

// ----------------------------------------------------------------------------

AudioHardware::AudioStreamInMSM72xx::AudioStreamInMSM72xx() :
    mHardware(0), mFd(-1), mState(AUDIO_INPUT_CLOSED), mRetryCount(0),
    mFormat(AUDIO_HW_IN_FORMAT), mChannelCount(AUDIO_HW_IN_CHANNELS), 
    mSampleRate(AUDIO_HW_IN_SAMPLERATE), mBufferSize(AUDIO_HW_IN_BUFFERSIZE),
    mAcoustics((AudioSystem::audio_in_acoustics)0)
{
}

status_t AudioHardware::AudioStreamInMSM72xx::set(
        AudioHardware* hw, int format, int channelCount, uint32_t sampleRate,
        AudioSystem::audio_in_acoustics acoustic_flags)
{
    LOGV("AudioStreamInMSM72xx::set(%d, %d, %u)", format, channelCount, sampleRate);
    if (mFd >= 0) {
        LOGE("Audio record already open");
        return -EPERM;
    }

    // open audio input device
    status_t status = ::open("/dev/msm_pcm_in", O_RDWR);
    if (status < 0) {
        LOGE("Cannot open /dev/msm_pcm_in errno: %d", errno);
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

    mFormat = format;
    mChannelCount = config.channel_count;
    mSampleRate = config.sample_rate;
    mBufferSize = config.buffer_size;

    mHardware = hw;
    mHardware->setMicMute_nosync(false);
    mState = AUDIO_INPUT_OPENED;
    audpre_index = calculate_audpre_table_index(sampleRate);
    tx_iir_index = (audpre_index * 2) + (hw->checkOutputStandby() ? 0 : 1);
    LOGD("audpre_index = %d, tx_iir_index = %d\n", audpre_index, tx_iir_index);

    /**
     * If audio-preprocessing failed, we should not block record.
     */
    int (*msm72xx_set_audpre_params)(int, int);
    msm72xx_set_audpre_params = (int (*)(int, int))::dlsym(acoustic, "msm72xx_set_audpre_params");
    status = msm72xx_set_audpre_params(audpre_index, tx_iir_index);
    if (status < 0)
        LOGE("Cannot set audpre parameters");

    int (*msm72xx_enable_audpre)(int, int, int);
    msm72xx_enable_audpre = (int (*)(int, int, int))::dlsym(acoustic, "msm72xx_enable_audpre");
    mAcoustics = acoustic_flags;
    status = msm72xx_enable_audpre((int)acoustic_flags, audpre_index, tx_iir_index);
    if (status < 0)
        LOGE("Cannot enable audpre");

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
    LOGV("AudioStreamInMSM72xx destructor");
    if (mHardware) {
        standby();
        mHardware->closeInputStream(this);
    }
}

ssize_t AudioHardware::AudioStreamInMSM72xx::read( void* buffer, ssize_t bytes)
{
    LOGV("AudioStreamInMSM72xx::read(%p, %ld)", buffer, bytes);
    if (!mHardware) return -1;

    size_t count = bytes;
    uint8_t* p = static_cast<uint8_t*>(buffer);

    if (mState < AUDIO_INPUT_OPENED) {
        Mutex::Autolock lock(mHardware->mLock);
        if (set(mHardware, mFormat, mChannelCount, mSampleRate, mAcoustics) != NO_ERROR) {
            return -1;
        }
    }
    
    if (mState < AUDIO_INPUT_STARTED) {
        if (ioctl(mFd, AUDIO_START, 0)) {
            LOGE("Error starting record");
            return -1;
        }
        mState = AUDIO_INPUT_STARTED;
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

status_t AudioHardware::AudioStreamInMSM72xx::standby()
{
    if (!mHardware) return -1;
    if (mState > AUDIO_INPUT_CLOSED) {
        if (mFd > 0) {
            ::close(mFd);
            mFd = -1;
        }
        mHardware->checkMicMute();
        mState = AUDIO_INPUT_CLOSED;
    }
    return NO_ERROR;
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
    snprintf(buffer, SIZE, "\tmState: %d\n", mState);
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

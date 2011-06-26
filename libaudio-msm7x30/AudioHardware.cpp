/*
** Copyright 2008, The Android Open-Source Project
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

#define LOG_NDEBUG 0
#define LOG_NDDEBUG 0

#define LOG_TAG "AudioHardwareMSM7X30"
#include <utils/Log.h>
#include <utils/String8.h>

#include <stdio.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dlfcn.h>
#include <fcntl.h>
#include <cutils/properties.h>

// hardware specific functions
#include "control.h"
#include "AudioHardware.h"
#include <media/AudioSystem.h>
#include <media/AudioRecord.h>
#include <media/mediarecorder.h>

extern "C" {
#include <linux/spi_aic3254.h>
#include <linux/tpa2051d3.h>
}

#define LOG_SND_RPC 0  // Set to 1 to log sound RPC's

#define DUALMIC_KEY "dualmic_enabled"
#define TTY_MODE_KEY "tty_mode"

#ifdef WITH_QCOM_SPEECH
#define AMRNB_DEVICE_IN "/dev/msm_amrnb_in"
#define EVRC_DEVICE_IN "/dev/msm_evrc_in"
#define QCELP_DEVICE_IN "/dev/msm_qcelp_in"
#endif
#define AAC_DEVICE_IN "/dev/msm_aac_in"

#ifdef WITH_QCOM_SPEECH
#define AMRNB_FRAME_SIZE 32
#define EVRC_FRAME_SIZE 23
#define QCELP_FRAME_SIZE 35
#endif

namespace android {

Mutex   mDeviceSwitchLock;
static void * acoustic;
const uint32_t AudioHardware::inputSamplingRates[] = {
    8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000
};

static const uint32_t INVALID_DEVICE = 65535;
static const uint32_t SND_DEVICE_HANDSET = 0;
static const uint32_t SND_DEVICE_SPEAKER = 1;
static const uint32_t SND_DEVICE_HEADSET = 2;
static const uint32_t SND_DEVICE_BT = 3;
static const uint32_t SND_DEVICE_CARKIT = 4;
static const uint32_t SND_DEVICE_TTY_FULL = 5;
static const uint32_t SND_DEVICE_TTY_VCO = 6;
static const uint32_t SND_DEVICE_TTY_HCO = 7;
static const uint32_t SND_DEVICE_NO_MIC_HEADSET = 8;
static const uint32_t SND_DEVICE_FM_HEADSET = 9;
static const uint32_t SND_DEVICE_HEADSET_AND_SPEAKER = 10;
static const uint32_t SND_DEVICE_FM_SPEAKER = 11;
static const uint32_t SND_DEVICE_FM_HANDSET = 12;
static const uint32_t SND_DEVICE_HEADPHONE_AND_SPEAKER = 16;
static const uint32_t SND_DEVICE_IN_S_SADC_OUT_HANDSET = 17;
static const uint32_t SND_DEVICE_IN_S_SADC_OUT_SPEAKER_PHONE = 18;
static const uint32_t SND_DEVICE_HANDSET_BACK_MIC = 20;
static const uint32_t SND_DEVICE_SPEAKER_BACK_MIC = 21;
static const uint32_t SND_DEVICE_NO_MIC_HEADSET_BACK_MIC = 28;
static const uint32_t SND_DEVICE_HEADSET_AND_SPEAKER_BACK_MIC = 30;
static const uint32_t SND_DEVICE_I2S_SPEAKER = 32;
static const uint32_t SND_DEVICE_BT_EC_OFF = 45;
static const uint32_t SND_DEVICE_HAC = 252;
static const uint32_t SND_DEVICE_USB_HEADSET = 253;

static uint32_t DEVICE_HANDSET_RX = 0;           //handset_rx
static uint32_t DEVICE_HANDSET_TX = 1;           //handset_tx
static uint32_t DEVICE_HEADSET_RX = 2;           //headset_stereo_rx
static uint32_t DEVICE_HEADSET_MONO_RX = 3;      //headset_mono_rx
static uint32_t DEVICE_HEADSET_TX = 4;           //headset_mono_tx
static uint32_t DEVICE_FMRADIO_HANDSET_RX = 5;   //fmradio_handset_rx
static uint32_t DEVICE_SPEAKER_RX = 6;           //speaker_mono_rx
static uint32_t DEVICE_FMRADIO_SPEAKER_RX = 7;   //fmradio_speaker_rx
static uint32_t DEVICE_FMRADIO_HEADSET_RX = 8;   //fmradio_headset_rx
static uint32_t DEVICE_TTY_HEADSET_MONO_TX = 9;  //tty_headset_mono_tx
static uint32_t DEVICE_TTY_HEADSET_MONO_RX = 10; //tty_headset_mono_rx
static uint32_t DEVICE_SPEAKER_TX = 11;          //speaker_mono_tx
static uint32_t DEVICE_SPEAKER_HEADSET_RX = 12;  //headset_speaker_stereo_rx
static uint32_t DEVICE_USB_HEADSET_RX = 13;      //usb_headset_stereo_rx
static uint32_t DEVICE_HAC_RX = 14;              //hac_mono_rx
static uint32_t DEVICE_ALT_RX = 15;              //alt_mono_rx
static uint32_t DEVICE_VR_HANDSET = 16;          //handset_vr_tx
static uint32_t DEVICE_BT_SCO_RX = 17;           //bt_sco_rx
static uint32_t DEVICE_BT_SCO_TX = 18;           //bt_sco_tx
static uint32_t DEVICE_COUNT = DEVICE_BT_SCO_TX +1;

static bool support_aic3254 = true;
static bool support_tpa2051 = true;
static int alt_enable = 0;
static int hac_enable = 0;
static uint32_t cur_aic_tx = 0;
static uint32_t cur_aic_rx = 0;
static const char * cur_aic_effect = NULL;

int dev_cnt = 0;
const char ** name = NULL;
int mixer_cnt = 0;
static uint32_t cur_tx = INVALID_DEVICE;
static uint32_t cur_rx = INVALID_DEVICE;

typedef struct routing_table
{
    unsigned short dec_id;
    uint32_t dev_id;
    uint32_t dev_id_tx;
    int stream_type;
    bool active;
    struct routing_table *next;
} Routing_table;
Routing_table* head;
Mutex       mRoutingTableLock;

typedef struct device_table
{
    int dev_id;
    int class_id;
    int capability;
}Device_table;
Device_table* device_list;

static void amr_transcode(unsigned char *src, unsigned char *dst);

enum STREAM_TYPES {
PCM_PLAY=1,
PCM_REC,
LPA_DECODE,
VOICE_CALL,
FM_RADIO
};

#define DEV_ID(X) device_list[X].dev_id
void addToTable(int decoder_id,int device_id,int device_id_tx,int stream_type,bool active) {
    LOGD("addToTable (dec_id %d, dev_rx %d, dev_tx %d, type %d, active %d)", decoder_id, device_id, device_id_tx, stream_type, active);
    Routing_table* temp_ptr;
    Mutex::Autolock lock(mRoutingTableLock);
    temp_ptr = (Routing_table* ) malloc(sizeof(Routing_table));
    temp_ptr->next = NULL;
    temp_ptr->dec_id = decoder_id;
    temp_ptr->dev_id = device_id;
    temp_ptr->dev_id_tx = device_id_tx;
    temp_ptr->stream_type = stream_type;
    temp_ptr->active = active;
    //add new Node to head 
    temp_ptr->next =head->next;
    head->next = temp_ptr;
}
bool isStreamOn(int Stream_type) {
    Routing_table* temp_ptr;
    Mutex::Autolock lock(mRoutingTableLock);
    temp_ptr = head->next;
    while(temp_ptr!=NULL) {
        if(temp_ptr->stream_type == Stream_type)
                return true;
        temp_ptr=temp_ptr->next;
    }
    return false;
}
bool isStreamOnAndActive(int Stream_type) {
    Routing_table* temp_ptr;
    Mutex::Autolock lock(mRoutingTableLock);
    temp_ptr = head->next;
    while(temp_ptr!=NULL) {
        if(temp_ptr->stream_type == Stream_type) {
            if(temp_ptr->active == true) {
                return true;
            }
            else {
                return false;
            }
        }
        temp_ptr=temp_ptr->next;
    }
    return false;
}
bool isStreamOnAndInactive(int Stream_type) {
    Routing_table* temp_ptr;
    Mutex::Autolock lock(mRoutingTableLock);
    temp_ptr = head->next;
    while(temp_ptr!=NULL) {
        if(temp_ptr->stream_type == Stream_type) {
            if(temp_ptr->active == false) {
                return true;
            }
            else {
                return false;
            }
        }
        temp_ptr=temp_ptr->next;
    }
    return false;
}
Routing_table*  getNodeByStreamType(int Stream_type) {
    Routing_table* temp_ptr;
    Mutex::Autolock lock(mRoutingTableLock);
    temp_ptr = head->next;
    while(temp_ptr!=NULL) {
        if(temp_ptr->stream_type == Stream_type) {
            return temp_ptr;
        }
        temp_ptr=temp_ptr->next;
    }
    return NULL;
}
void modifyActiveStateOfStream(int Stream_type, bool Active) {
    Routing_table* temp_ptr;
    Mutex::Autolock lock(mRoutingTableLock);
    temp_ptr = head->next;
    while(temp_ptr!=NULL) {
        if(temp_ptr->stream_type == Stream_type) {
            temp_ptr->active = Active;
            return;
        }
        temp_ptr=temp_ptr->next;
    }
}
void modifyActiveDeviceOfStream(int Stream_type,int Device_id,int Device_id_tx) {
    Routing_table* temp_ptr;
    temp_ptr = head->next;
    while(temp_ptr!=NULL) {
        if(temp_ptr->stream_type == Stream_type) {
            temp_ptr->dev_id = Device_id;
            temp_ptr->dev_id_tx = Device_id_tx;
            return;
        }
        temp_ptr=temp_ptr->next;
    }
}
void printTable()
{
    Routing_table * temp_ptr;
    Mutex::Autolock lock(mRoutingTableLock);
    temp_ptr = head->next;
    while(temp_ptr!=NULL) {
        printf("%d %d %d %d %d\n",temp_ptr->dec_id,temp_ptr->dev_id,temp_ptr->dev_id_tx,temp_ptr->stream_type,temp_ptr->active);
        temp_ptr = temp_ptr->next;
    }
}
void deleteFromTable(int Stream_type) {
    LOGD("deleteFromTable, type %d", Stream_type);
    Routing_table *temp_ptr,*temp1;
    Mutex::Autolock lock(mRoutingTableLock);
    temp_ptr = head;
    while(temp_ptr->next!=NULL) {
        if(temp_ptr->next->stream_type == Stream_type) {
            temp1 = temp_ptr->next;
            temp_ptr->next = temp_ptr->next->next;
            free(temp1);
            return;
        }
        temp_ptr=temp_ptr->next;
    }

}

bool isDeviceListEmpty() {
    if(head->next == NULL)
        return true;
    else
        return false;
}

int enableDevice(int device,short enable) {
    LOGD("value of device and enable is %d %d",device,enable);
    if( msm_en_device(DEV_ID(device), enable)) {
        LOGE("msm_en_device(%d, %d) failed errno = %d",DEV_ID(device), enable, errno);
        return -1;
    }
    return 0;
}

void updateACDB(uint32_t new_rx_device, uint32_t new_tx_device, uint32_t new_rx_acdb, uint32_t new_tx_acdb)
{
    LOGD("updateACDB: (%d, %d, %d, %d) ", new_tx_device, new_rx_device, new_tx_acdb, new_rx_acdb);

    int rc = -1;
    int (*update_acdb_id)(uint32_t, uint32_t, uint32_t, uint32_t);

    update_acdb_id = (int (*)(uint32_t, uint32_t, uint32_t, uint32_t))::dlsym(acoustic, "update_acdb_id");
    if ((*update_acdb_id) == 0 ) {
        LOGE("Could not open update_acdb_id()");
    }

    rc = update_acdb_id(new_tx_device, new_rx_device, new_tx_acdb, new_rx_acdb);
    if (rc < 0) {
        LOGE("Could not set update_acdb_id: %d", rc);
    }
}

static status_t updateDeviceInfo(uint32_t rx_device, uint32_t tx_device,
                                 uint32_t rx_acdb_id, uint32_t tx_acdb_id ) {
    LOGD("updateDeviceInfo: E rx_device %d and tx_device %d", rx_device, tx_device);
    bool isRxDeviceEnabled = false,isTxDeviceEnabled = false;
    Routing_table *temp_ptr,*temp_head;
    temp_head = head;

    Mutex::Autolock lock(mDeviceSwitchLock);

    if(!getNodeByStreamType(VOICE_CALL) && !getNodeByStreamType(PCM_PLAY) &&
       !getNodeByStreamType(LPA_DECODE) && !getNodeByStreamType(FM_RADIO)) {
        LOGD("No active voicecall/playback, disabling cur_rx %d", cur_rx);
        if(cur_rx != INVALID_DEVICE && enableDevice(cur_rx, 0)) {
            LOGE("Disabling device failed for cur_rx %d", cur_rx);
        }
        cur_rx = rx_device;
    }

    if(!getNodeByStreamType(VOICE_CALL) && !getNodeByStreamType(PCM_REC)) {
        LOGD("No active voicecall/recording, disabling cur_tx %d", cur_tx);
        if(cur_tx != INVALID_DEVICE && enableDevice(cur_tx, 0)) {
            LOGE("Disabling device failed for cur_tx %d", cur_tx);
        }
        cur_tx = tx_device;
    }

    Mutex::Autolock lock_1(mRoutingTableLock);

    while(temp_head->next != NULL) {
        temp_ptr = temp_head->next;
        switch(temp_ptr->stream_type) {
            case PCM_PLAY:
            case LPA_DECODE:
            case FM_RADIO:
                LOGD("The node type is %d and cur device %d new device %d ", temp_ptr->stream_type, temp_ptr->dev_id, rx_device);
                if(rx_device == INVALID_DEVICE)
                    return -1;
                if(rx_device == temp_ptr->dev_id)
                    break;
                LOGV("rx_device = %d,temp_ptr->dev_id = %d",rx_device,temp_ptr->dev_id);
                if(isRxDeviceEnabled == false) {
                    enableDevice(temp_ptr->dev_id,0);
                    enableDevice(rx_device,1);
                    isRxDeviceEnabled = true;
                }
                if(msm_route_stream(PCM_PLAY,temp_ptr->dec_id,DEV_ID(temp_ptr->dev_id),0)) {
                    LOGE("msm_route_stream(PCM_PLAY,%d,%d,0) failed",temp_ptr->dec_id,DEV_ID(temp_ptr->dev_id));
                }
                if(msm_route_stream(PCM_PLAY,temp_ptr->dec_id,DEV_ID(rx_device),1)) {
                    LOGE("msm_route_stream(PCM_PLAY,%d,%d,1) failed",temp_ptr->dec_id,DEV_ID(rx_device));
                }
                modifyActiveDeviceOfStream(temp_ptr->stream_type,rx_device,INVALID_DEVICE);
                cur_tx = tx_device ;
                cur_rx = rx_device ;
                break;
            case PCM_REC:
                LOGD("case PCM_REC");
                if(tx_device == INVALID_DEVICE)
                    return -1;
                if(tx_device == temp_ptr->dev_id)
                    break;

                if(isTxDeviceEnabled == false) {
                    enableDevice(temp_ptr->dev_id,0);
                    enableDevice(tx_device,1);
                   isTxDeviceEnabled = true;
                }
                if(msm_route_stream(PCM_REC,temp_ptr->dec_id,DEV_ID(temp_ptr->dev_id),0)) {
                    LOGE("msm_route_stream(PCM_PLAY,%d,%d,0) failed",temp_ptr->dec_id,DEV_ID(temp_ptr->dev_id));
                }
                if(msm_route_stream(PCM_REC,temp_ptr->dec_id,DEV_ID(tx_device),1)) {
                    LOGE("msm_route_stream(PCM_REC,%d,%d,1) failed",temp_ptr->dec_id,DEV_ID(tx_device));
                }
                modifyActiveDeviceOfStream(PCM_REC,tx_device,INVALID_DEVICE);
                cur_tx = tx_device ;
                cur_rx = rx_device ;
                break;
            case VOICE_CALL:
                LOGD("case VOICE_CALL");
                if(rx_device == INVALID_DEVICE || tx_device == INVALID_DEVICE)
                    return -1;
                if(rx_device == temp_ptr->dev_id && tx_device == temp_ptr->dev_id_tx)
                    break;

                updateACDB(rx_device, tx_device, rx_acdb_id, tx_acdb_id);
                msm_route_voice(DEV_ID(rx_device),DEV_ID(tx_device),1);

                // Temporary work around for Speaker mode. The driver is not
                // supporting Speaker Rx and Handset Tx combo
                if(isRxDeviceEnabled == false) {
                    if (rx_device != temp_ptr->dev_id)
                    {
                        enableDevice(temp_ptr->dev_id,0);
                    }
                    isRxDeviceEnabled = true;
                }
                if(isTxDeviceEnabled == false) {
                    if (tx_device != temp_ptr->dev_id_tx)
                    {
                        enableDevice(temp_ptr->dev_id_tx,0);
                    }
                    isTxDeviceEnabled = true;
                }

                if (rx_device != temp_ptr->dev_id)
                {
                    enableDevice(rx_device,1);
                }

                if (tx_device != temp_ptr->dev_id_tx)
                {
                    enableDevice(tx_device,1);
                }

                cur_rx = rx_device;
                cur_tx = tx_device;
                modifyActiveDeviceOfStream(VOICE_CALL,cur_rx,cur_tx);
                break;
            default:
                break;
        }
        temp_head = temp_head->next;
    }

    LOGD("updateDeviceInfo: X cur_rx %d cur_tx %d", cur_rx, cur_tx);
    return NO_ERROR;
}

void freeMemory() {
    Routing_table *temp;
    while(head != NULL) {
        temp = head->next;
        free(head);
        head = temp;
    }
free(device_list);
}

//
// ----------------------------------------------------------------------------

AudioHardware::AudioHardware() :
    mInit(false), mMicMute(true), mBluetoothNrec(true), mBluetoothId(0),
    mHACSetting(false), mBluetoothIdTx(0), mBluetoothIdRx(0),
    mOutput(0), mCurSndDevice(INVALID_DEVICE), mVoiceVolume(VOICE_VOLUME_MAX),
    mTtyMode(TTY_OFF), mDualMicEnabled(false)
{
    int (*snd_get_num)();
    int (*snd_get_bt_endpoint)(msm_bt_endpoint *);
    int (*set_acoustic_parameters)();
    int (*set_tpa2051_parameters)();
    int (*set_aic3254_parameters)();

    struct msm_bt_endpoint *ept;

    int control;
    int i = 0,index = 0;

    acoustic =:: dlopen("/system/lib/libhtc_acoustic.so", RTLD_NOW);
    if (acoustic == NULL ) {
        LOGD("Could not open libhtc_acoustic.so");
        /* this is not really an error on non-htc devices... */
        mNumBTEndpoints = 0;
        mInit = true;
        return;
    }

    head = (Routing_table* ) malloc(sizeof(Routing_table));
    head->next = NULL;

    LOGD("msm_mixer_open: Opening the device");
    control = msm_mixer_open("/dev/snd/controlC0", 0);
    if(control< 0)
        LOGE("ERROR opening the device");

    mixer_cnt = msm_mixer_count();
    LOGD("msm_mixer_count:mixer_cnt = %d",mixer_cnt);

    dev_cnt = msm_get_device_count();
    LOGV("got device_count %d",dev_cnt);
    if (dev_cnt <= 0) {
        LOGE("NO devices registered\n");
        return;
    }

    name = msm_get_device_list();
    device_list = (Device_table* )malloc(sizeof(Device_table)*DEVICE_COUNT);
    if (device_list == NULL) {
        LOGE("malloc failed for device list");
        return;
    }
    for (i = 0;i<dev_cnt;i++)
        device_list[i].dev_id = INVALID_DEVICE;

    for (i = 0; i < dev_cnt;i++) {
        LOGV("******* name[%d] = [%s] *********", i, (char* )name[i]);
        if (strcmp((char* )name[i],"handset_rx") == 0)
            index = DEVICE_HANDSET_RX;
        else if (strcmp((char* )name[i],"handset_tx") == 0)
            index = DEVICE_HANDSET_TX;
        else if (strcmp((char* )name[i],"headset_stereo_rx") == 0)
            index = DEVICE_HEADSET_RX;
        else if (strcmp((char* )name[i],"headset_mono_rx") == 0)
            index = DEVICE_HEADSET_MONO_RX;
        else if (strcmp((char* )name[i],"headset_mono_tx") == 0)
            index = DEVICE_HEADSET_TX;
        else if (strcmp((char* )name[i],"fmradio_handset_rx") == 0)
            index = DEVICE_FMRADIO_HANDSET_RX;
        else if (strcmp((char* )name[i],"speaker_mono_rx") == 0)
            index = DEVICE_SPEAKER_RX;
        else if (strcmp((char* )name[i],"fmradio_speaker_rx") == 0)
            index = DEVICE_FMRADIO_SPEAKER_RX;
        else if (strcmp((char* )name[i],"fmradio_headset_rx") == 0)
            index = DEVICE_FMRADIO_HEADSET_RX;
        else if (strcmp((char* )name[i],"tty_headset_mono_tx") == 0)
            index = DEVICE_TTY_HEADSET_MONO_TX;
        else if (strcmp((char* )name[i],"tty_headset_mono_rx") == 0)
            index = DEVICE_TTY_HEADSET_MONO_RX;
        else if (strcmp((char* )name[i],"speaker_mono_tx") == 0)
            index = DEVICE_SPEAKER_TX;
        else if (strcmp((char*)name[i],"headset_speaker_stereo_rx") == 0)
            index = DEVICE_SPEAKER_HEADSET_RX;
        else if (strcmp((char*)name[i],"usb_headset_stereo_rx") == 0)
            index = DEVICE_USB_HEADSET_RX;
        else if (strcmp((char*)name[i],"hac_mono_rx") == 0)
            index = DEVICE_HAC_RX;
        else if (strcmp((char*)name[i],"alt_mono_rx") == 0)
            index = DEVICE_ALT_RX;
        else if (strcmp((char*)name[i],"handset_vr_tx") == 0)
            index = DEVICE_VR_HANDSET;
        else if (strcmp((char* )name[i],"bt_sco_rx") == 0)
            index = DEVICE_BT_SCO_RX;
        else if (strcmp((char* )name[i],"bt_sco_tx") == 0)
            index = DEVICE_BT_SCO_TX;
        else 
            continue;
        LOGV("index = %d", index);

        device_list[index].dev_id = msm_get_device((char* )name[i]);
        if (device_list[index].dev_id >= 0) {
            LOGV("Found device: %s:index = %d,dev_id: %d",( char* )name[i], index,device_list[index].dev_id);
        }
        device_list[index].class_id = msm_get_device_class(device_list[index].dev_id);
        device_list[index].capability = msm_get_device_capability(device_list[index].dev_id);
        LOGV("class ID = %d,capablity = %d for device %d",device_list[index].class_id,device_list[index].capability,device_list[index].dev_id);
    }

    set_acoustic_parameters = (int (*)(void))::dlsym(acoustic, "set_acoustic_parameters");
    if ((*set_acoustic_parameters) == 0 ) {
        LOGE("Could not open set_acoustic_parameters()");
        return;
    }

    int rc = set_acoustic_parameters();
    if (rc < 0) {
        LOGD("Could not set acoustic parameters to share memory: %d", rc);
    }

    snd_get_num = (int (*)(void))::dlsym(acoustic, "snd_get_num");
    if ((*snd_get_num) == 0 ) {
        LOGD("Could not open snd_get_num()");
    }

    mNumBTEndpoints = snd_get_num();
    LOGV("mNumBTEndpoints = %d", mNumBTEndpoints);
    mBTEndpoints = new msm_bt_endpoint[mNumBTEndpoints];
    mInit = true;
    LOGV("constructed %d SND endpoints)", mNumBTEndpoints);
    ept = mBTEndpoints;
    snd_get_bt_endpoint = (int (*)(msm_bt_endpoint *))::dlsym(acoustic, "snd_get_bt_endpoint");
    if ((*snd_get_bt_endpoint) == 0 ) {
        LOGE("Could not open snd_get_bt_endpoint()");
        return;
    }
    snd_get_bt_endpoint(mBTEndpoints);

    for (int i = 0; i < mNumBTEndpoints; i++) {
        LOGV("BT name %s (tx,rx)=(%d,%d)", mBTEndpoints[i].name, mBTEndpoints[i].tx, mBTEndpoints[i].rx);
    }

    char value[PROPERTY_VALUE_MAX];
    // Check the system property for enable or not the ALT function
    property_get("htc.audio.alt.enable", value, "0");
    alt_enable = atoi(value);
    LOGV("Enable ALT function: %d", alt_enable);

    // Check the system property for enable or not the HAC function
    property_get("htc.audio.hac.enable", value, "0");
    hac_enable = atoi(value);
    LOGV("Enable HAC function: %d", hac_enable);

    set_tpa2051_parameters = (int (*)(void))::dlsym(acoustic, "set_tpa2051_parameters");
    if ((*set_tpa2051_parameters) == 0) {
        LOGD("set_tpa2051_parameters() not present");
        support_tpa2051 = false;
    }

    if (support_tpa2051) {
        rc = set_tpa2051_parameters();
        if (rc < 0) {
            LOGD("Speaker amplifies tpa2051 is not supported");
            support_tpa2051 = false;
        }
    }

    set_aic3254_parameters = (int (*)(void))::dlsym(acoustic, "set_aic3254_parameters");
    if ((*set_aic3254_parameters) == 0 ) {
        LOGD("set_aic3254_parameters() not present");
        support_aic3254 = false;
    }

    if (support_aic3254) {
        rc = set_aic3254_parameters();
        if (rc < 0) {
            LOGD("AIC3254 DSP is not supported");
            support_aic3254 = false;
        }
    }

    mInit = true;
}

AudioHardware::~AudioHardware()
{
    for (size_t index = 0; index < mInputs.size(); index++) {
        closeInputStream((AudioStreamIn*)mInputs[index]);
    }
    mInputs.clear();
    closeOutputStream((AudioStreamOut*)mOutput);
    if (acoustic) {
        ::dlclose(acoustic);
        acoustic = 0;
    }
    msm_mixer_close();
    freeMemory();

    mInit = false;
}

status_t AudioHardware::initCheck()
{
    return mInit ? NO_ERROR : NO_INIT;
}

AudioStreamOut* AudioHardware::openOutputStream(
        uint32_t devices, int *format, uint32_t *channels, uint32_t *sampleRate, status_t *status)
{
    { // scope for the lock
        Mutex::Autolock lock(mLock);

        LOGD("AudioHardware::openOutputStream");

        // only one output stream allowed
        if (mOutput) {
            if (status) {
                *status = INVALID_OPERATION;
            }
            return 0;
        }

        // create new output stream
        AudioStreamOutMSM72xx* out = new AudioStreamOutMSM72xx();
        status_t lStatus = out->set(this, devices, format, channels, sampleRate);
        if (status) {
            *status = lStatus;
        }
        if (lStatus == NO_ERROR) {
            mOutput = out;
        } else {
            delete out;
        }
    }
    return mOutput;
}

#ifdef WITH_QCOM_LPA
AudioStreamOut* AudioHardware::openOutputSession(
        uint32_t devices, int *format, status_t *status, int sessionId)
{
    AudioSessionOutMSM7xxx* out;
    { // scope for the lock
        Mutex::Autolock lock(mLock);

        // create new output stream
        out = new AudioSessionOutMSM7xxx();
        status_t lStatus = out->set(this, devices, format, sessionId);
        if (status) {
            *status = lStatus;
        }
        if (lStatus != NO_ERROR) {
            delete out;
            out = NULL;
        }
    }
    return out;
}
#endif

void AudioHardware::closeOutputStream(AudioStreamOut* out) {
    Mutex::Autolock lock(mLock);
    if (mOutput == 0 || mOutput != out) {
        LOGW("Attempt to close invalid output stream");
    }
    else {
        delete mOutput;
        mOutput = 0;
    }
}

AudioStreamIn* AudioHardware::openInputStream(
        uint32_t devices, int *format, uint32_t *channels, uint32_t *sampleRate, status_t *status,
        AudioSystem::audio_in_acoustics acoustic_flags)
{
    // check for valid input source
    if (!AudioSystem::isInputDevice((AudioSystem::audio_devices)devices)) {
        return 0;
    }

    mLock.lock();

    AudioStreamInMSM72xx* in = new AudioStreamInMSM72xx();
    status_t lStatus = in->set(this, devices, format, channels, sampleRate, acoustic_flags);
    if (status) {
        *status = lStatus;
    }
    if (lStatus != NO_ERROR) {
        mLock.unlock();
        delete in;
        return 0;
    }

    mInputs.add(in);
    mLock.unlock();

    return in;
}

void AudioHardware::closeInputStream(AudioStreamIn* in) {
    Mutex::Autolock lock(mLock);

    ssize_t index = mInputs.indexOf((AudioStreamInMSM72xx *)in);
    if (index < 0) {
        LOGW("Attempt to close invalid input stream");
    } else {
        mLock.unlock();
        delete mInputs[index];
        mLock.lock();
        mInputs.removeAt(index);
    }
}

status_t AudioHardware::setMode(int mode)
{
    int prevMode = mMode;
    status_t status = AudioHardwareBase::setMode(mode);
    if (status == NO_ERROR) {
        // make sure that doAudioRouteOrMute() is called by doRouting()
        // even if the new device selected is the same as current one.
        if (((prevMode != AudioSystem::MODE_IN_CALL) && (mMode == AudioSystem::MODE_IN_CALL)) ||
             ((prevMode == AudioSystem::MODE_IN_CALL) && (mMode != AudioSystem::MODE_IN_CALL))) {
             clearCurDevice();
        }
    }
    return status;
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
        LOGD("setMicMute_nosync calling voice mute with the mMicMute %d", mMicMute);
        msm_set_voice_tx_mute(mMicMute);
    }
    return NO_ERROR;
}

status_t AudioHardware::getMicMute(bool* state)
{
    *state = mMicMute;
    return NO_ERROR;
}

status_t AudioHardware::setParameters(const String8& keyValuePairs)
{
    AudioParameter param = AudioParameter(keyValuePairs);
    String8 value;
    String8 key;
    const char BT_NREC_KEY[] = "bt_headset_nrec";
    const char BT_NAME_KEY[] = "bt_headset_name";
    const char BT_NREC_VALUE_ON[] = "on";
    const char HAC_KEY[] = "HACSetting";
    const char HAC_VALUE_ON[] = "ON";

    LOGV("setParameters() %s", keyValuePairs.string());

    if (keyValuePairs.length() == 0) return BAD_VALUE;

    if (hac_enable) {
        key = String8(HAC_KEY);
        if (param.get(key, value) == NO_ERROR) {
            if (value == HAC_VALUE_ON) {
                mHACSetting = true;
                LOGD("Enable HAC");
            } else {
                mHACSetting = false;
                LOGD("Disable HAC");
            }
        }
    }

    key = String8(BT_NREC_KEY);
    if (param.get(key, value) == NO_ERROR) {
        if (value == BT_NREC_VALUE_ON) {
            mBluetoothNrec = true;
        } else {
            mBluetoothNrec = false;
            LOGI("Turning noise reduction and echo cancellation off for BT "
                 "headset");
        }
    }

    key = String8(BT_NAME_KEY);
    if (param.get(key, value) == NO_ERROR) {
       doRouting(NULL);
    }

    key = String8(DUALMIC_KEY);
    if (param.get(key, value) == NO_ERROR) {
        if (value == "true") {
            mDualMicEnabled = true;
            LOGI("DualMike feature Enabled");
        } else {
            mDualMicEnabled = false;
            LOGI("DualMike feature Disabled");
        }
        doRouting(NULL);
    }

    key = String8(TTY_MODE_KEY);
    if (param.get(key, value) == NO_ERROR) {
        if (value == "full") {
            mTtyMode = TTY_FULL;
        } else if (value == "hco") {
            mTtyMode = TTY_HCO;
        } else if (value == "vco") {
            mTtyMode = TTY_VCO;
        } else {
            mTtyMode = TTY_OFF;
        }
        if(mMode != AudioSystem::MODE_IN_CALL) {
           return NO_ERROR;
        }
        LOGI("Changed TTY Mode=%s", value.string());
        doRouting(NULL);
    }

    return NO_ERROR;
}

String8 AudioHardware::getParameters(const String8& keys)
{
    AudioParameter param = AudioParameter(keys);
    String8 value;

    String8 key = String8(DUALMIC_KEY);
    if (param.get(key, value) == NO_ERROR) {
        value = String8(mDualMicEnabled ? "true" : "false");
        param.add(key, value);
    }
#ifdef WITH_QCOM_SPEECH
    key = String8("tunneled-input-formats");
    if ( param.get(key,value) == NO_ERROR ) {
        param.addInt(String8("AMR"), true );
        param.addInt(String8("QCELP"), true );
        param.addInt(String8("EVRC"), true );
    }
#endif
    LOGV("AudioHardware::getParameters() %s", param.toString().string());
    return param.toString();
}

size_t AudioHardware::getInputBufferSize(uint32_t sampleRate, int format, int channelCount)
{
    if ((format != AudioSystem::PCM_16_BIT) &&
#ifdef WITH_QCOM_SPEECH
        (format != AudioSystem::AMR_NB)      &&
        (format != AudioSystem::EVRC)      &&
        (format != AudioSystem::QCELP)  &&
#endif 
        (format != AudioSystem::AAC)){
        LOGW("getInputBufferSize bad format: %d", format);
        return 0;
    }
    if (channelCount < 1 || channelCount > 2) {
        LOGW("getInputBufferSize bad channel count: %d", channelCount);
        return 0;
    }

#ifdef WITH_QCOM_SPEECH
    if (format == AudioSystem::AMR_NB)
       return 320*channelCount;
    else if (format == AudioSystem::EVRC)
       return 230*channelCount;
    else if (format == AudioSystem::QCELP)
       return 350*channelCount;
    else if (format == AudioSystem::AAC)
#else
    if (format == AudioSystem::AAC)
#endif
       return 2048;
    else
       return 2048*channelCount;
}

static status_t set_volume_rpc(uint32_t device,
                               uint32_t method,
                               uint32_t volume)
{
    LOGV("set_volume_rpc(%d, %d, %d)\n", device, method, volume);

    if (device == -1UL) return NO_ERROR;
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
    int vol = lrint(v * 100.0);
    if (mCurSndDevice == SND_DEVICE_HAC) {
        LOGI("HAC enable: Setting in-call volume to maximum.\n");
        if (msm_set_voice_rx_vol(VOICE_VOLUME_MAX)) {
            LOGE("msm_set_voice_rx_vol(%d) failed errno = %d", VOICE_VOLUME_MAX, errno);
            return -1;
        }
    } else {
        LOGD("setVoiceVolume(%f)\n", v);
        LOGI("Setting in-call volume to %d (available range is 0 to 100)\n", vol);

        if (msm_set_voice_rx_vol(vol)) {
            LOGE("msm_set_voice_rx_vol(%d) failed errno = %d", vol, errno);
            return -1;
        }
    }

    mVoiceVolume = vol;
    LOGV("msm_set_voice_rx_vol(%d) succeeded", vol);

    if (mMode == AudioSystem::MODE_IN_CALL &&
        mCurSndDevice != SND_DEVICE_BT &&
        mCurSndDevice != SND_DEVICE_CARKIT &&
        mCurSndDevice != SND_DEVICE_BT_EC_OFF)
    {
        uint32_t new_tx_acdb = getACDB(MOD_TX, mCurSndDevice);
        uint32_t new_rx_acdb = getACDB(MOD_RX, mCurSndDevice);

        int (*update_voice_acdb)(uint32_t, uint32_t);

        update_voice_acdb = (int (*)(uint32_t, uint32_t))::dlsym(acoustic, "update_voice_acdb");
        if ((*update_voice_acdb) == 0 ) {
            LOGE("Could not open update_voice_acdb()");
        }

        int rc = update_voice_acdb(new_tx_acdb, new_rx_acdb);
        if (rc < 0)
            LOGE("Could not set update_voice_acdb: %d", rc);
        else
            LOGI("update_voice_acdb(%d, %d) succeeded", new_tx_acdb, new_rx_acdb);
    }

    return NO_ERROR;
}

/*
status_t AudioHardware::setFmVolume(float v)
{
    int vol = AudioSystem::logToLinear( v );
    if ( vol > 100 ) {
        vol = 100;
    }
    else if ( vol < 0 ) {
        vol = 0;
    LOGV("setFmVolume(%f)\n", v);
    Routing_table* temp = NULL;
    temp = getNodeByStreamType(FM_RADIO);
    if(temp == NULL){
        LOGV("No Active FM stream is running");
        return NO_ERROR;
    }
    if(msm_set_volume(temp->dec_id, vol)) {
        LOGE("msm_set_volume(%f) failed for FM errno = %d", vol, errno);
        return -1;
    }
    LOGV("msm_set_volume(%f) for FM succeeded", vol);
    return NO_ERROR;
}
*/

static status_t set_volume_fm(uint32_t volume)
{
    int returnval = 0;
    float ratio = 2.5;

    char s1[100] = "hcitool cmd 0x3f 0xa 0x5 0xc0 0x41 0xf 0 0x20 0 0 0";
    char s2[100] = "hcitool cmd 0x3f 0xa 0x5 0xe4 0x41 0xf 0 0x00 0 0 0";
    char s3[100] = "hcitool cmd 0x3f 0xa 0x5 0xe0 0x41 0xf 0 ";

    char stemp[10] = "";
    char *pstarget = s3;

    volume = (unsigned int)(volume * ratio);

    sprintf(stemp, "0x%x ", volume);
    pstarget = strcat(s3, stemp);
    pstarget = strcat(s3, "0 0 0");

    system(s1);
    system(s2);
    system(s3);

    return returnval;
}

status_t AudioHardware::setFmVolume(float v)
{
    int vol = AudioSystem::logToLinear(v);
    if (vol > 100) {
        vol = 100;
    } else if (vol < 0) {
        vol = 0;
    }
    LOGV("setFmVolume %d", vol);
    set_volume_fm(vol);
    return NO_ERROR;
}

status_t AudioHardware::setMasterVolume(float v)
{
    Mutex::Autolock lock(mLock);
    int vol = ceil(v * 7.0);
    LOGI("Set master volume to %d.\n", vol);

    set_volume_rpc(SND_DEVICE_HANDSET, SND_METHOD_VOICE, vol);
    set_volume_rpc(SND_DEVICE_SPEAKER, SND_METHOD_VOICE, vol);
    set_volume_rpc(SND_DEVICE_BT,      SND_METHOD_VOICE, vol);
    set_volume_rpc(SND_DEVICE_HEADSET, SND_METHOD_VOICE, vol);
    //TBD - does HDMI require this handling

    // We return an error code here to let the audioflinger do in-software
    // volume on top of the maximum volume that we set through the SND API.
    // return error - software mixer will handle it
    return -1;
}

status_t do_tpa2051_control(int mode)
{
    int fd, rc;
    int tpa_mode = 0;

    if (mode) {
        if (cur_rx == DEVICE_HEADSET_RX)
            tpa_mode = TPA2051_MODE_VOICECALL_HEADSET;
        if (cur_rx == DEVICE_SPEAKER_RX)
            tpa_mode = TPA2051_MODE_VOICECALL_SPKR;
    } else {
        if (cur_rx == DEVICE_FMRADIO_HEADSET_RX)
            tpa_mode = TPA2051_MODE_FM_HEADSET;
        if (cur_rx == DEVICE_FMRADIO_SPEAKER_RX)
            tpa_mode = TPA2051_MODE_FM_SPKR;
        if (cur_rx == DEVICE_SPEAKER_HEADSET_RX)
            tpa_mode = TPA2051_MODE_RING;
        if (cur_rx == DEVICE_HEADSET_RX)
            tpa_mode = TPA2051_MODE_PLAYBACK_HEADSET;
        if (cur_rx == DEVICE_SPEAKER_RX)
            tpa_mode = TPA2051_MODE_PLAYBACK_SPKR;
    }

    fd = open("/dev/tpa2051d3", O_RDWR);
    if (fd < 0) {
        LOGE("can't open /dev/tpa2051d3 %d", fd);
        return -1;
    }

    if (tpa_mode) {
        rc = ioctl(fd, TPA2051_SET_MODE, &tpa_mode);
        if (rc < 0)
            LOGE("ioctl TPA2051_SET_MODE failed: %s", strerror(errno));
        else
            LOGD("Update TPA2051_SET_MODE to mode %d success", tpa_mode);
    }

    close(fd);
    return 0;
}

static status_t do_route_audio_rpc(uint32_t device,
                                   bool ear_mute, bool mic_mute,
                                   uint32_t rx_acdb_id, uint32_t tx_acdb_id)
{
    uint32_t new_rx_device = INVALID_DEVICE, new_tx_device = INVALID_DEVICE;

    Routing_table* temp = NULL;
    LOGV("do_route_audio_rpc(%d, %d, %d)", device, ear_mute, mic_mute);

    if (device == SND_DEVICE_HANDSET) {
        new_rx_device = DEVICE_HANDSET_RX;
        new_tx_device = DEVICE_HANDSET_TX;
        LOGV("In HANDSET");
    }
    else if (device == SND_DEVICE_SPEAKER) {
        new_rx_device = DEVICE_SPEAKER_RX;
        new_tx_device = DEVICE_SPEAKER_TX;
        LOGV("In SPEAKER");
    }
    else if (device == SND_DEVICE_HEADSET) {
        new_rx_device = DEVICE_HEADSET_RX;
        new_tx_device = DEVICE_HEADSET_TX;
        LOGV("In HEADSET");
    }
    else if (device == SND_DEVICE_NO_MIC_HEADSET) {
        new_rx_device = DEVICE_HEADSET_RX;
        new_tx_device = DEVICE_HANDSET_TX;
        LOGV("In NO MIC HEADSET");
    }
    else if (device == SND_DEVICE_FM_HANDSET) {
        new_rx_device = DEVICE_FMRADIO_HANDSET_RX;
        new_tx_device = DEVICE_HANDSET_TX;
        LOGV("In FM HANDSET");
    }
    else if (device == SND_DEVICE_FM_SPEAKER) {
        new_rx_device = DEVICE_FMRADIO_SPEAKER_RX;
        new_tx_device = DEVICE_HANDSET_TX;
        LOGV("In FM SPEAKER");
    }
    else if(device == SND_DEVICE_FM_HEADSET) {
        new_rx_device = DEVICE_FMRADIO_HEADSET_RX;
        new_tx_device = DEVICE_HANDSET_TX;
        LOGV("In FM HEADSET");
    }
/*
    else if(device == SND_DEVICE_IN_S_SADC_OUT_HANDSET) {
        new_rx_device = DEVICE_HANDSET_RX;
        new_tx_device = DEVICE_DUALMIC_HANDSET_TX;
        LOGV("In DUALMIC_HANDSET");
    }
    else if(device == SND_DEVICE_IN_S_SADC_OUT_SPEAKER_PHONE) {
        new_rx_device = DEVICE_SPEAKER_RX;
        new_tx_device = DEVICE_DUALMIC_SPEAKER_TX;
        LOGV("In DUALMIC_SPEAKER");
    }
*/
    else if (device == SND_DEVICE_TTY_FULL) {
        new_rx_device = DEVICE_TTY_HEADSET_MONO_RX;
        new_tx_device = DEVICE_TTY_HEADSET_MONO_TX;
        LOGV("In TTY_FULL");
    }
    else if (device == SND_DEVICE_TTY_VCO) {
        new_rx_device = DEVICE_TTY_HEADSET_MONO_RX;
        new_tx_device = DEVICE_HANDSET_TX;
        LOGV("In TTY_VCO");
    }
    else if (device == SND_DEVICE_TTY_HCO) {
        new_rx_device = DEVICE_HANDSET_RX;
        new_tx_device = DEVICE_TTY_HEADSET_MONO_TX;
        LOGV("In TTY_HCO");
    }
    else if ((device == SND_DEVICE_BT) || (device == SND_DEVICE_BT_EC_OFF)) {
        new_rx_device = DEVICE_BT_SCO_RX;
        new_tx_device = DEVICE_BT_SCO_TX;
        LOGV("In BT_HCO");
    }
    else if (device == SND_DEVICE_HEADSET_AND_SPEAKER) {
        new_rx_device = DEVICE_SPEAKER_HEADSET_RX;
        new_tx_device = DEVICE_HEADSET_TX;
        LOGV("In DEVICE_SPEAKER_HEADSET_RX and DEVICE_HEADSET_TX");
    }
    else if (device == SND_DEVICE_HEADPHONE_AND_SPEAKER) {
        new_rx_device = DEVICE_SPEAKER_HEADSET_RX;
        new_tx_device = DEVICE_HANDSET_TX;
        LOGV("In DEVICE_SPEAKER_HEADSET_RX and DEVICE_HANDSET_TX");
    }
    else if (device == SND_DEVICE_USB_HEADSET) {
        new_rx_device = DEVICE_USB_HEADSET_RX;
        new_tx_device = DEVICE_SPEAKER_TX;
        LOGV("In USB_HEADSET");
    }
    else if (device == SND_DEVICE_HAC) {
        new_rx_device = DEVICE_HAC_RX;
        new_tx_device = DEVICE_HANDSET_TX;
        LOGV("In HAC");
    }
/* No framework support
    else if (device == SND_DEVICE_HDMI) {
        new_rx_device = DEVICE_HDMI_STERO_RX;
        new_tx_device = cur_tx;
        LOGV("In DEVICE_HDMI_STERO_RX and cur_tx");
    }
*/
    if(new_rx_device != INVALID_DEVICE)
        LOGD("new_rx = %d", DEV_ID(new_rx_device));
    if(new_tx_device != INVALID_DEVICE)
        LOGD("new_tx = %d", DEV_ID(new_tx_device));

    if (ear_mute == false && !isStreamOn(VOICE_CALL)) {
        LOGV("Going to enable RX/TX device for voice stream");
        // Routing Voice
        if ( (new_rx_device != INVALID_DEVICE) && (new_tx_device != INVALID_DEVICE)) {
            LOGD("Starting voice on Rx %d and Tx %d device", DEV_ID(new_rx_device), DEV_ID(new_tx_device));
            updateACDB(new_rx_device, new_tx_device, rx_acdb_id, tx_acdb_id);
            msm_route_voice(DEV_ID(new_rx_device),DEV_ID(new_tx_device), 1);
        } else {
            return -1;
        }

        if(cur_rx == INVALID_DEVICE || new_rx_device == INVALID_DEVICE)
            return -1;

        if(cur_tx == INVALID_DEVICE || new_tx_device == INVALID_DEVICE)
            return -1;

        //Enable RX device
        if(new_rx_device != cur_rx) {
            enableDevice(cur_rx,0);
        }
        enableDevice(new_rx_device,1);

        //Enable TX device
        if(new_tx_device != cur_tx) {
            enableDevice(cur_tx,0);
        }
        enableDevice(new_tx_device,1);

        // start Voice call
        LOGD("Starting voice call and UnMuting the call");
        msm_start_voice();
        msm_set_voice_tx_mute(0);
        cur_rx = new_rx_device;
        cur_tx = new_tx_device;
        addToTable(0,cur_rx,cur_tx,VOICE_CALL,true);
        updateDeviceInfo(new_rx_device, new_tx_device, rx_acdb_id, tx_acdb_id);
    } else if (ear_mute == true && isStreamOnAndActive(VOICE_CALL)) {
        LOGV("Going to disable RX/TX device during end of voice call");
        temp = getNodeByStreamType(VOICE_CALL);
        if(temp == NULL)
            return 0;

        // Ending voice call
        LOGD("Ending Voice call");
        msm_end_voice();
        deleteFromTable(VOICE_CALL);
        updateDeviceInfo(new_rx_device, new_tx_device, 0, 0);
        if(new_rx_device != INVALID_DEVICE && new_tx_device != INVALID_DEVICE) {
            cur_rx = new_rx_device;
            cur_tx = new_tx_device;
        }
    }
    else {
        updateDeviceInfo(new_rx_device, new_tx_device, rx_acdb_id, tx_acdb_id);
    }

    if (support_tpa2051)
        do_tpa2051_control(ear_mute ^1);

    return NO_ERROR;
}

status_t AudioHardware::doAudioRouteOrMute(uint32_t device)
{
    uint32_t rx_acdb_id = 0;
    uint32_t tx_acdb_id = 0;

    if (device == SND_DEVICE_BT) {
        if (!mBluetoothNrec) {
            device = SND_DEVICE_BT_EC_OFF;
        }
    }

    if (support_aic3254) {
        aic3254_config(device, "Original");
        do_aic3254_control(mMode, mRecordState,
                           checkOutputStandby(), device);
    }

    if (device == SND_DEVICE_BT) {
        if (mBluetoothIdTx != 0) {
            rx_acdb_id = mBluetoothIdRx;
            tx_acdb_id = mBluetoothIdTx;
        } else {
            /* use default BT entry defined in AudioBTID.csv */
            rx_acdb_id = mBTEndpoints[0].rx;
            tx_acdb_id = mBTEndpoints[0].tx;
            LOGD("Update ACDB ID to default BT setting");
        }
    }  else if (device == SND_DEVICE_CARKIT
                || device == SND_DEVICE_BT_EC_OFF) {
        if (mBluetoothIdTx != 0) {
            rx_acdb_id = mBluetoothIdRx;
            tx_acdb_id = mBluetoothIdTx;
        } else {
            /* use default carkit entry defined in AudioBTID.csv */
            rx_acdb_id = mBTEndpoints[1].rx;
            tx_acdb_id = mBTEndpoints[1].tx;
            LOGD("Update ACDB ID to default carkit setting");
        }
    } else if (mMode == AudioSystem::MODE_IN_CALL
               && hac_enable && mHACSetting &&
               device == SND_DEVICE_HANDSET) {
        LOGD("Update acdb id to hac profile.");
        rx_acdb_id = ACDB_ID_HAC_HANDSET_SPKR;
        tx_acdb_id = ACDB_ID_HAC_HANDSET_MIC;
    } else {
        if (mMode == AudioSystem::MODE_IN_CALL) {
            rx_acdb_id = getACDB(MOD_RX, device);
            tx_acdb_id = getACDB(MOD_TX, device);
        } else {
            if (!checkOutputStandby()) {
                rx_acdb_id = getACDB(MOD_PLAY, device);
            }
            if (mRecordState) {
                tx_acdb_id = getACDB(MOD_REC, device);
            }
        }
    }

    LOGV("doAudioRouteOrMute: rx acdb %d, tx acdb %d", rx_acdb_id, tx_acdb_id);
    LOGV("doAudioRouteOrMute() device %d, mMode %d, mMicMute %d", device, mMode, mMicMute);

    return do_route_audio_rpc(device, mMode != AudioSystem::MODE_IN_CALL,
                              mMicMute, rx_acdb_id, tx_acdb_id);
}

status_t AudioHardware::get_mMode(void)
{
    return mMode;
}

status_t AudioHardware::set_mRecordState(bool onoff)
{
    mRecordState = onoff;
    return 0;
}

status_t AudioHardware::get_mRecordState(void)
{
    return mRecordState;
}

status_t AudioHardware::get_batt_temp(int *batt_temp)
{
    int fd, len;
    const char *fn =
            "/sys/devices/platform/rs30100001:00000000/power_supply/battery/batt_temp";

    char get_batt_temp[6] = { 0 };

    if ((fd = open(fn, O_RDONLY)) < 0) {
        LOGE("%s: cannot open %s: %s\n", __FUNCTION__, fn, strerror(errno));
        return UNKNOWN_ERROR;
    }

    if ((len = read(fd, get_batt_temp, sizeof(get_batt_temp))) <= 1) {
        LOGE("read battery temp fail: %s\n", strerror(errno));
        close(fd);
        return BAD_VALUE;
    }

    *batt_temp = strtol(get_batt_temp, NULL, 10);
    close(fd);
    return NO_ERROR;
}

status_t AudioHardware::get_snd_dev(void)
{
    Mutex::Autolock lock(mLock);
    return mCurSndDevice;
}

uint32_t AudioHardware::getACDB(int mode, uint32_t device)
{
    uint32_t acdb_id = 0;
    int batt_temp = 0;

    if (mMode == AudioSystem::MODE_IN_CALL &&
        device <= SND_DEVICE_NO_MIC_HEADSET) {
        if (mode == MOD_RX) {
            switch (device) {
                case SND_DEVICE_HANDSET:
                    acdb_id = mVoiceVolume / 20 + 201;
                    break;
                case SND_DEVICE_HEADSET:
                case SND_DEVICE_NO_MIC_HEADSET:
                    acdb_id = mVoiceVolume / 20 + 401;
                    break;
                case SND_DEVICE_SPEAKER:
                    acdb_id = mVoiceVolume / 20 + 601;
                    break;
                default:
                    break;
            }
        } else if (mode == MOD_TX) {
            switch (device) {
                case SND_DEVICE_HANDSET:
                    acdb_id = mVoiceVolume / 20 + 101;
                    break;
                case SND_DEVICE_HEADSET:
                case SND_DEVICE_NO_MIC_HEADSET:
                    acdb_id = mVoiceVolume / 20 + 301;
                    break;
                case SND_DEVICE_SPEAKER:
                    acdb_id = mVoiceVolume / 20 + 501;
                    break;
                default:
                    break;
            }
        }
    } else {
        if (mode == MOD_PLAY) {
            switch (device) {
                case SND_DEVICE_HEADSET:
                case SND_DEVICE_NO_MIC_HEADSET:
                case SND_DEVICE_NO_MIC_HEADSET_BACK_MIC:
                case SND_DEVICE_FM_HEADSET:
                    acdb_id = ACDB_ID_HEADSET_PLAYBACK;
                    break;
                case SND_DEVICE_SPEAKER:
                case SND_DEVICE_FM_SPEAKER:
                case SND_DEVICE_SPEAKER_BACK_MIC:
                    acdb_id = ACDB_ID_SPKR_PLAYBACK;
                    if(alt_enable) {
                        LOGD("Enable ALT for speaker\n");
                        if (get_batt_temp(&batt_temp) == NO_ERROR) {
                            if (batt_temp < 50)
                                acdb_id = ACDB_ID_ALT_SPKR_PLAYBACK;
                            LOGD("ALT batt temp = %d\n", batt_temp);
                        }
                    }
                    break;
                case SND_DEVICE_HEADSET_AND_SPEAKER:
                case SND_DEVICE_HEADSET_AND_SPEAKER_BACK_MIC:
                    acdb_id = ACDB_ID_HEADSET_RINGTONE_PLAYBACK;
                    break;
                default:
                    break;
            }
        } else if (mode == MOD_REC) {
            switch (device) {
                case SND_DEVICE_HEADSET:
                case SND_DEVICE_FM_HEADSET:
                case SND_DEVICE_FM_SPEAKER:
                case SND_DEVICE_HEADSET_AND_SPEAKER:
                    acdb_id = ACDB_ID_EXT_MIC_REC;
                    break;
                case SND_DEVICE_HANDSET:
                case SND_DEVICE_NO_MIC_HEADSET:
                case SND_DEVICE_SPEAKER:
                    acdb_id = ACDB_ID_INT_MIC_REC;
                    break;
                case SND_DEVICE_SPEAKER_BACK_MIC:
                case SND_DEVICE_NO_MIC_HEADSET_BACK_MIC:
                case SND_DEVICE_HANDSET_BACK_MIC:
                case SND_DEVICE_HEADSET_AND_SPEAKER_BACK_MIC:
                    acdb_id = ACDB_ID_CAMCORDER;
                    break;
                default:
                    break;
            }
        }
    }

    LOGV("getACDB, return ID %d\n", acdb_id);
    return acdb_id;
}

status_t AudioHardware::do_aic3254_control(int mode, bool record, bool standby, uint32_t device)
{
    LOGD("do_aic3254_control mode: %d record: %d standby: %d device: %d", mode, record, standby, device);

    uint32_t new_aic_txmode = UPLINK_OFF;
    uint32_t new_aic_rxmode = DOWNLINK_OFF;

    if (mode == AudioSystem::MODE_IN_CALL) {
        if (device == SND_DEVICE_HEADSET) {
            new_aic_rxmode = CALL_DOWNLINK_EMIC_HEADSET;
            new_aic_txmode = CALL_UPLINK_EMIC_HEADSET;
        } else if (device == SND_DEVICE_SPEAKER ||
                   device == SND_DEVICE_SPEAKER_BACK_MIC) {
            new_aic_rxmode = CALL_DOWNLINK_IMIC_SPEAKER;
            new_aic_txmode = CALL_UPLINK_IMIC_SPEAKER;
        } else if (device == SND_DEVICE_HEADSET_AND_SPEAKER ||
                   device == SND_DEVICE_HEADSET_AND_SPEAKER_BACK_MIC) {
            new_aic_rxmode = RING_HEADSET_SPEAKER;
        } else if (device == SND_DEVICE_NO_MIC_HEADSET ||
                   device == SND_DEVICE_NO_MIC_HEADSET_BACK_MIC) {
            new_aic_rxmode = CALL_DOWNLINK_IMIC_HEADSET;
            new_aic_txmode = CALL_UPLINK_IMIC_HEADSET;
        } else if (device == SND_DEVICE_HANDSET ||
                   device == SND_DEVICE_HANDSET_BACK_MIC) {
            new_aic_rxmode = CALL_DOWNLINK_IMIC_RECEIVER;
            new_aic_txmode = CALL_UPLINK_IMIC_RECEIVER;
        }
    } else {
        if (standby) {
            if (device == SND_DEVICE_FM_HEADSET) {
                new_aic_rxmode = FM_OUT_HEADSET;
                new_aic_txmode = FM_IN_HEADSET;
            } else if (device == SND_DEVICE_FM_SPEAKER) {
                new_aic_rxmode = FM_OUT_SPEAKER;
                new_aic_txmode = FM_IN_SPEAKER;
            }
        } else {
            if (device == SND_DEVICE_HEADSET_AND_SPEAKER ||
                device == SND_DEVICE_HEADSET_AND_SPEAKER_BACK_MIC) {
                new_aic_rxmode = RING_HEADSET_SPEAKER;
            } else if (device == SND_DEVICE_SPEAKER ||
                       device == SND_DEVICE_SPEAKER_BACK_MIC) {
                new_aic_rxmode = PLAYBACK_SPEAKER;
            } else if (device == SND_DEVICE_HANDSET ||
                       device == SND_DEVICE_HANDSET_BACK_MIC) {
                new_aic_rxmode = PLAYBACK_RECEIVER;
            } else if (device == SND_DEVICE_HEADSET ||
                       device == SND_DEVICE_NO_MIC_HEADSET ||
                       device == SND_DEVICE_NO_MIC_HEADSET_BACK_MIC) {
                new_aic_rxmode = PLAYBACK_HEADSET;
            }
        }

        if (record) {
            if (device == SND_DEVICE_HEADSET) {
                new_aic_txmode = VOICERECORD_EMIC;
            } else if (device == SND_DEVICE_HANDSET_BACK_MIC ||
                device == SND_DEVICE_SPEAKER_BACK_MIC ||
                device == SND_DEVICE_NO_MIC_HEADSET_BACK_MIC ||
                device == SND_DEVICE_HEADSET_AND_SPEAKER_BACK_MIC) {
                new_aic_txmode = VIDEORECORD_IMIC;
            } else if (device == SND_DEVICE_HANDSET ||
                device == SND_DEVICE_SPEAKER ||
                device == SND_DEVICE_NO_MIC_HEADSET ||
                device == SND_DEVICE_HEADSET_AND_SPEAKER) {
                new_aic_txmode = VOICERECORD_IMIC;
            }
        }
    }

    LOGD("aic3254_ioctl: new_aic_rxmode %d cur_aic_rx %d", new_aic_rxmode, cur_aic_rx);
    if (new_aic_rxmode != cur_aic_rx)
        if (aic3254_ioctl(AIC3254_CONFIG_RX, new_aic_rxmode) >= 0)
            cur_aic_rx = new_aic_rxmode;

    LOGD("aic3254_ioctl: new_aic_txmode %d cur_aic_tx %d", new_aic_txmode, cur_aic_tx);
    if (new_aic_txmode != cur_aic_tx)
        if (aic3254_ioctl(AIC3254_CONFIG_TX, new_aic_txmode) >= 0)
            cur_aic_tx = new_aic_txmode;

    if (cur_aic_tx == UPLINK_OFF && cur_aic_rx == DOWNLINK_OFF)
        aic3254_powerdown();

    return NO_ERROR;
}

void AudioHardware::aic3254_config(uint32_t Routes, const char* aic_effect)
{
    int (*set_sound_effect)(const char* effect);

    if (mMode == AudioSystem::MODE_IN_CALL)
        aic_effect = "DualMic_Phone";

    LOGD("aic3254_config effect: %s ", aic_effect);

    if (cur_aic_effect != aic_effect) {
        set_sound_effect = (int (*)(const char*))::dlsym(acoustic, "set_sound_effect");
        if ((*set_sound_effect) == 0 ) {
            LOGE("Could not open set_sound_effect()");
            return;
        }

        int rc = set_sound_effect(aic_effect);
        if (rc < 0)
            LOGE("Could not set sound effect %s: %d", aic_effect, rc);
        else
            cur_aic_effect = aic_effect;
    }
}

int AudioHardware::aic3254_ioctl(int cmd, const int argc)
{
    int rc = -1;
    int (*set_aic3254_ioctl)(int, const int*);

    LOGD("aic3254_ioctl()");

    set_aic3254_ioctl = (int (*)(int, const int*))::dlsym(acoustic, "set_aic3254_ioctl");
    if ((*set_aic3254_ioctl) == 0) {
        LOGE("Could not open set_aic3254_ioctl()");
        return rc;
    }

    LOGD("aic3254_ioctl: try ioctl 0x%x with arg %d", cmd, argc);
   
    rc = set_aic3254_ioctl(cmd, &argc);
    if (rc < 0) {
        LOGE("aic3254_ioctl failed");
    }

    return rc;
}

void AudioHardware::aic3254_powerdown()
{
    LOGD("aic3254_powerdown");
    int rc = aic3254_ioctl(AIC3254_POWERDOWN, 0);
    if (rc < 0)
        LOGE("aic3254_powerdown failed");
}

int AudioHardware::aic3254_set_volume(int volume)
{
    LOGD("aic3254_set_volume = %d", volume);

    if (aic3254_ioctl(AIC3254_CONFIG_VOLUME_L, volume) < 0)
        LOGE("aic3254_set_volume: could not set aic3254 LEFT volume %d\n", volume);
    int rc = aic3254_ioctl(AIC3254_CONFIG_VOLUME_R, volume);
    if (rc < 0)
        LOGE("aic3254_set_volume: could not set aic3254 RIGHT volume %d\n", volume);
    return rc;
}

status_t AudioHardware::doRouting(AudioStreamInMSM72xx *input)
{
    Mutex::Autolock lock(mLock);
    uint32_t outputDevices = mOutput->devices();
    status_t ret = NO_ERROR;
    uint32_t sndDevice = INVALID_DEVICE;

    if (input != NULL) {
        uint32_t inputDevice = input->devices();
        LOGI("do input routing device %x\n", inputDevice);
        // ignore routing device information when we start a recording in voice
        // call
        // Recording will happen through currently active tx device
        if((inputDevice == AudioSystem::DEVICE_IN_VOICE_CALL)
           || (inputDevice == AudioSystem::DEVICE_IN_FM_RX)
           || (inputDevice == AudioSystem::DEVICE_IN_FM_RX_A2DP)
          )
            return NO_ERROR;
        if (inputDevice != 0) {
            if (inputDevice & AudioSystem::DEVICE_IN_BLUETOOTH_SCO_HEADSET) {
                LOGI("Routing audio to Bluetooth PCM\n");
                sndDevice = SND_DEVICE_BT;
            } else if (inputDevice & AudioSystem::DEVICE_IN_WIRED_HEADSET) {
                if ((outputDevices & AudioSystem::DEVICE_OUT_WIRED_HEADSET) &&
                    (outputDevices & AudioSystem::DEVICE_OUT_SPEAKER)) {
                    LOGI("Routing audio to Wired Headset and Speaker\n");
                    sndDevice = SND_DEVICE_HEADSET_AND_SPEAKER;
                } else {
                    LOGI("Routing audio to Wired Headset\n");
                    sndDevice = SND_DEVICE_HEADSET;
                }
            } else {
                if (outputDevices & AudioSystem::DEVICE_OUT_SPEAKER) {
                    LOGI("Routing audio to Speakerphone\n");
                    sndDevice = SND_DEVICE_SPEAKER;
                } else if (outputDevices == AudioSystem::DEVICE_OUT_WIRED_HEADPHONE) {
                    LOGI("Routing audio to Speakerphone\n");
                    sndDevice = SND_DEVICE_NO_MIC_HEADSET;
                } else {
                    LOGI("Routing audio to Handset\n");
                    sndDevice = SND_DEVICE_HANDSET;
                }
            }
        }
        // if inputDevice == 0, restore output routing
    }

    if (sndDevice == INVALID_DEVICE) {
        if (outputDevices & (outputDevices - 1)) {
            if ((outputDevices & AudioSystem::DEVICE_OUT_SPEAKER) == 0) {
                LOGW("Hardware does not support requested route combination (%#X),"
                     " picking closest possible route...", outputDevices);
            }
        }
        if ((mTtyMode != TTY_OFF) && (mMode == AudioSystem::MODE_IN_CALL) &&
                (outputDevices & AudioSystem::DEVICE_OUT_WIRED_HEADSET)) {
            if (mTtyMode == TTY_FULL) {
                LOGI("Routing audio to TTY FULL Mode\n");
                sndDevice = SND_DEVICE_TTY_FULL;
            } else if (mTtyMode == TTY_VCO) {
                LOGI("Routing audio to TTY VCO Mode\n");
                sndDevice = SND_DEVICE_TTY_VCO;
            } else if (mTtyMode == TTY_HCO) {
                LOGI("Routing audio to TTY HCO Mode\n");
                sndDevice = SND_DEVICE_TTY_HCO;
            }
        } else if (outputDevices &
                   (AudioSystem::DEVICE_OUT_BLUETOOTH_SCO | AudioSystem::DEVICE_OUT_BLUETOOTH_SCO_HEADSET)) {
            LOGI("Routing audio to Bluetooth PCM\n");
            sndDevice = SND_DEVICE_BT;
        } else if (outputDevices & AudioSystem::DEVICE_OUT_BLUETOOTH_SCO_CARKIT) {
            LOGI("Routing audio to Bluetooth PCM\n");
            sndDevice = SND_DEVICE_CARKIT;

/* No framework support
        } else if (outputDevices & AudioSystem::DEVICE_OUT_AUX_HDMI) {
            LOGI("Routing audio to HDMI\n");
            sndDevice = SND_DEVICE_HDMI;
*/
        } else if ((outputDevices & AudioSystem::DEVICE_OUT_WIRED_HEADSET) &&
                   (outputDevices & AudioSystem::DEVICE_OUT_SPEAKER)) {
            LOGI("Routing audio to Wired Headset and Speaker\n");
            sndDevice = SND_DEVICE_HEADSET_AND_SPEAKER;
        } else if (outputDevices & AudioSystem::DEVICE_OUT_WIRED_HEADPHONE) {
            if (outputDevices & AudioSystem::DEVICE_OUT_SPEAKER) {
                LOGI("Routing audio to No microphone Wired Headset and Speaker (%d,%x)\n", mMode, outputDevices);
                sndDevice = SND_DEVICE_HEADPHONE_AND_SPEAKER;
            } else {
                if (outputDevices & AudioSystem::DEVICE_OUT_FM) {
                    LOGI("Routing FM audio to No microphone Wired Headset (%d,%x)\n", mMode, outputDevices);
                    sndDevice = SND_DEVICE_FM_HEADSET;
                } else {
                    LOGI("Routing audio to No microphone Wired Headset (%d,%x)\n", mMode, outputDevices);
                    sndDevice = SND_DEVICE_NO_MIC_HEADSET;
                }
            }
        } else if (outputDevices & AudioSystem::DEVICE_OUT_WIRED_HEADSET) {
            if (outputDevices & AudioSystem::DEVICE_OUT_FM) {
                LOGI("Routing FM audio to Wired Headset\n");
                sndDevice = SND_DEVICE_FM_HEADSET;
            } else {
                LOGI("Routing audio to Wired Headset\n");
                sndDevice = SND_DEVICE_HEADSET;
            }
        } else if (outputDevices & AudioSystem::DEVICE_OUT_SPEAKER) {
            if (outputDevices & AudioSystem::DEVICE_OUT_FM) {
                LOGI("Routing FM audio to Speakerphone\n");
                sndDevice = SND_DEVICE_FM_SPEAKER;
            } else {
                LOGI("Routing audio to Speakerphone\n");
                sndDevice = SND_DEVICE_SPEAKER;
            }
        } else if (outputDevices & AudioSystem::DEVICE_OUT_EARPIECE) {
            LOGI("Routing audio to Handset\n");
            sndDevice = SND_DEVICE_HANDSET;
        }
    }

    if (mDualMicEnabled && mMode == AudioSystem::MODE_IN_CALL) {
        if (sndDevice == SND_DEVICE_HANDSET) {
            LOGI("Routing audio to handset with DualMike enabled\n");
            sndDevice = SND_DEVICE_IN_S_SADC_OUT_HANDSET;
        } else if (sndDevice == SND_DEVICE_SPEAKER) {
            LOGI("Routing audio to speakerphone with DualMike enabled\n");
            sndDevice = SND_DEVICE_IN_S_SADC_OUT_SPEAKER_PHONE;
        }
    }

    if (outputDevices & AudioSystem::DEVICE_OUT_FM)
        enableFM(sndDevice);
    else
        disableFM();

    if ((sndDevice != INVALID_DEVICE && sndDevice != mCurSndDevice)) {
        ret = doAudioRouteOrMute(sndDevice);
        mCurSndDevice = sndDevice;
        if (mMode == AudioSystem::MODE_IN_CALL) {
            if (mHACSetting && hac_enable && mCurSndDevice == SND_DEVICE_HAC) {
                LOGD("HAC enable: Setting in-call volume to maximum.\n");
                if (msm_set_voice_rx_vol(VOICE_VOLUME_MAX))
                    LOGE("msm_set_voice_rx_vol(%d) failed errno = %d", VOICE_VOLUME_MAX, errno);
            } else {
                if (msm_set_voice_rx_vol(mVoiceVolume))
                    LOGE("msm_set_voice_rx_vol(%d) failed errno = %d", mVoiceVolume, errno);
            }
        }
    }

    return ret;
}

status_t AudioHardware::enableFM(int sndDevice)
{
    LOGD("enableFM");

    addToTable(0, cur_rx, INVALID_DEVICE, FM_RADIO, true);
    if(sndDevice == mCurSndDevice || mCurSndDevice == INVALID_DEVICE) {
        enableDevice(cur_rx, 1);
        msm_route_stream(PCM_PLAY, 0, DEV_ID(cur_rx), 1);
    }

    return NO_ERROR;
}

status_t AudioHardware::disableFM()
{
    LOGD("disableFM");

    Routing_table* temp = NULL;
    temp = getNodeByStreamType(FM_RADIO);

    if(temp == NULL)
        return NO_ERROR;

    LOGD("Deroute FM stream");
    if(msm_route_stream(PCM_PLAY, temp->dec_id, DEV_ID(cur_rx), 0)) {
        LOGE("msm_route_stream failed");
        deleteFromTable(FM_RADIO);
        return 0;
    }
    deleteFromTable(FM_RADIO);
    updateDeviceInfo(cur_rx, cur_tx, 0, 0);

    return NO_ERROR;
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
    for (size_t index = 0; index < mInputs.size(); index++) {
        mInputs[index]->dump(fd, args);
    }

    if (mOutput) {
        mOutput->dump(fd, args);
    }
    return NO_ERROR;
}

uint32_t AudioHardware::getInputSampleRate(uint32_t sampleRate)
{
    uint32_t i;
    uint32_t prevDelta;
    uint32_t delta;

    for (i = 0, prevDelta = 0xFFFFFFFF; i < sizeof(inputSamplingRates)/sizeof(uint32_t); i++, prevDelta = delta) {
        delta = abs(sampleRate - inputSamplingRates[i]);
        if (delta > prevDelta) break;
    }
    // i is always > 0 here
    return inputSamplingRates[i-1];
}


// ----------------------------------------------------------------------------

#ifdef WITH_QCOM_LPA
AudioHardware::AudioSessionOutMSM7xxx::AudioSessionOutMSM7xxx() :
    mHardware(0), mStartCount(0), mRetryCount(0), mStandby(true), mDevices(0), mSessionId(-1)
{
}

status_t AudioHardware::AudioSessionOutMSM7xxx::set(
        AudioHardware* hw, uint32_t devices, int *pFormat, int32_t sessionId)
{
    int lFormat = pFormat ? *pFormat : 0;

    mHardware = hw;
    mDevices = devices;

    if(sessionId >= 0) {
        LOGD("AudioSessionOutMSM7xxx::set() Adding LPA_DECODE Node to Table");

        Mutex::Autolock lock(mDeviceSwitchLock);

        addToTable(sessionId,cur_rx,INVALID_DEVICE,LPA_DECODE,true);

        if(enableDevice(cur_rx, 1)) {
            LOGE("enableDevice failed for device cur_rx %d", cur_rx);
            return 0;
        }

        LOGV("msm_route_stream(PCM_PLAY,%d,%d,0)",sessionId,DEV_ID(cur_rx));
        if(msm_route_stream(PCM_PLAY,sessionId,DEV_ID(cur_rx),1)) {
            LOGE("msm_route_stream(PCM_PLAY,%d,%d,1) failed",sessionId,DEV_ID(cur_rx));
        }
        mSessionId = sessionId;
    }
    return NO_ERROR;
}

AudioHardware::AudioSessionOutMSM7xxx::~AudioSessionOutMSM7xxx()
{
}

status_t AudioHardware::AudioSessionOutMSM7xxx::standby()
{
    Routing_table* temp = NULL;
    LOGD("AudioSessionOutMSM7xxx::standby()");
    status_t status = NO_ERROR;

    temp = getNodeByStreamType(LPA_DECODE);

    if(temp == NULL)
        return NO_ERROR;

    LOGD("Deroute lpa playback stream");
    if(msm_route_stream(PCM_PLAY, temp->dec_id,DEV_ID(temp->dev_id), 0)) {
        LOGE("could not set stream routing\n");
        deleteFromTable(LPA_DECODE);
        return -1;
    }
    deleteFromTable(LPA_DECODE);
    updateDeviceInfo(cur_rx, cur_tx, 0, 0);
    mStandby = true;
    return status;
}

bool AudioHardware::AudioSessionOutMSM7xxx::checkStandby()
{
    return mStandby;
}


status_t AudioHardware::AudioSessionOutMSM7xxx::setParameters(const String8& keyValuePairs)
{
    AudioParameter param = AudioParameter(keyValuePairs);
    String8 key = String8(AudioParameter::keyRouting);
    status_t status = NO_ERROR;
    int device;
    LOGV("AudioSessionOutMSM7xxx::setParameters() %s", keyValuePairs.string());

    if (param.getInt(key, device) == NO_ERROR) {
        mDevices = device;
        LOGV("set output routing %x", mDevices);
        status = mHardware->doRouting(NULL);
        param.remove(key);
    }

    if (param.size()) {
        status = BAD_VALUE;
    }
    return status;
}

String8 AudioHardware::AudioSessionOutMSM7xxx::getParameters(const String8& keys)
{
    AudioParameter param = AudioParameter(keys);
    String8 value;
    String8 key = String8(AudioParameter::keyRouting);

    if (param.get(key, value) == NO_ERROR) {
        LOGV("get routing %x", mDevices);
        param.addInt(key, (int)mDevices);
    }

    LOGV("AudioSessionOutMSM7xxx::getParameters() %s", param.toString().string());
    return param.toString();
}

status_t AudioHardware::AudioSessionOutMSM7xxx::getRenderPosition(uint32_t *dspFrames)
{
    //TODO: enable when supported by driver
    return INVALID_OPERATION;
}

status_t AudioHardware::AudioSessionOutMSM7xxx::setVolume(float left, float right)
{
    float v = (left + right) / 2;
    if (v < 0.0) {
        LOGW("AudioSessionOutMSM7xxx::setVolume(%f) under 0.0, assuming 0.0\n", v);
        v = 0.0;
    } else if (v > 1.0) {
        LOGW("AudioSessionOutMSM7xxx::setVolume(%f) over 1.0, assuming 1.0\n", v);
        v = 1.0;
    }

    // Ensure to convert the log volume back to linear for LPA
    float vol = v * 100;
    LOGV("AudioSessionOutMSM7xxx::setVolume(%f)\n", v);
    LOGV("Setting session volume to %f (available range is 0 to 100)\n", vol);

    if(msm_set_volume(mSessionId, vol)) {
        LOGE("msm_set_volume(%d %f) failed errno = %d", mSessionId, vol, errno);
        return -1;
    }
    LOGV("msm_set_volume(%f) succeeded", vol);
    return NO_ERROR;
}
#endif

// ----------------------------------------------------------------------------

AudioHardware::AudioStreamOutMSM72xx::AudioStreamOutMSM72xx() :
    mHardware(0), mFd(-1), mStartCount(0), mRetryCount(0), mStandby(true), mDevices(0)
{
}

status_t AudioHardware::AudioStreamOutMSM72xx::set(
        AudioHardware* hw, uint32_t devices, int *pFormat, uint32_t *pChannels, uint32_t *pRate)
{
    int lFormat = pFormat ? *pFormat : 0;
    uint32_t lChannels = pChannels ? *pChannels : 0;
    uint32_t lRate = pRate ? *pRate : 0;

    mHardware = hw;

    // fix up defaults
    if (lFormat == 0) lFormat = format();
    if (lChannels == 0) lChannels = channels();
    if (lRate == 0) lRate = sampleRate();

    // check values
    if ((lFormat != format()) ||
        (lChannels != channels()) ||
        (lRate != sampleRate())) {
        if (pFormat) *pFormat = format();
        if (pChannels) *pChannels = channels();
        if (pRate) *pRate = sampleRate();
        return BAD_VALUE;
    }

    if (pFormat) *pFormat = lFormat;
    if (pChannels) *pChannels = lChannels;
    if (pRate) *pRate = lRate;

    mDevices = devices;

    return NO_ERROR;
}

AudioHardware::AudioStreamOutMSM72xx::~AudioStreamOutMSM72xx()
{
    if (mFd >= 0) close(mFd);
}

ssize_t AudioHardware::AudioStreamOutMSM72xx::write(const void* buffer, size_t bytes)
{
    // LOGD("AudioStreamOutMSM72xx::write(%p, %u)", buffer, bytes);
    status_t status = NO_INIT;
    size_t count = bytes;
    const uint8_t* p = static_cast<const uint8_t*>(buffer);
    unsigned short dec_id = INVALID_DEVICE;

    if (mStandby) {

        // open driver
        LOGV("open driver");
        status = ::open("/dev/msm_pcm_out", O_WRONLY/*O_RDWR*/);
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
        config.channel_count = AudioSystem::popCount(channels());
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

        if (support_tpa2051)
            do_tpa2051_control(0);

        if (support_aic3254) {
            mHardware->do_aic3254_control(mHardware->get_mMode(),
                                          mHardware->get_mRecordState(),
                                          mHardware->checkOutputStandby(),
                                          mHardware->get_snd_dev());
        }
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
            if(ioctl(mFd, AUDIO_GET_SESSION_ID, &dec_id)) {
                LOGE("AUDIO_GET_SESSION_ID failed*********");
                return 0;
            }
            LOGV("dec_id = %d\n",dec_id);
            if(cur_rx == INVALID_DEVICE)
                return 0;

            Mutex::Autolock lock(mDeviceSwitchLock);

            LOGV("cur_rx for pcm playback = %d",cur_rx);
            if(enableDevice(cur_rx, 1)) {
                LOGE("enableDevice failed for device cur_rx %d", cur_rx);
                return 0;
            }

            uint32_t rx_acdb_id = mHardware->getACDB(MOD_PLAY, mHardware->get_snd_dev());
            updateACDB(cur_rx, cur_tx, rx_acdb_id, 0);

            if(msm_route_stream(PCM_PLAY, dec_id, DEV_ID(cur_rx), 1)) {
                LOGE("msm_route_stream failed");
                return 0;
            }
            addToTable(dec_id,cur_rx,INVALID_DEVICE,PCM_PLAY,true);
            ioctl(mFd, AUDIO_START, 0);
        }
    }
    return bytes;

Error:
    if (mFd >= 0) {
        ::close(mFd);
        mFd = -1;
    }
    // Simulate audio output timing in case of error
    usleep(bytes * 1000000 / frameSize() / sampleRate());

    return status;
}

status_t AudioHardware::AudioStreamOutMSM72xx::standby()
{
    Routing_table* temp = NULL;
    LOGD("AudioStreamOutMSM72xx::standby()");
    status_t status = NO_ERROR;

    temp = getNodeByStreamType(PCM_PLAY);

    if(temp == NULL)
        return NO_ERROR;

    LOGD("Deroute pcm out stream");
    if(msm_route_stream(PCM_PLAY, temp->dec_id, DEV_ID(temp->dev_id), 0)) {
        LOGE("could not set stream routing\n");
        deleteFromTable(PCM_PLAY);
        return -1;
    }
    deleteFromTable(PCM_PLAY);
    updateDeviceInfo(cur_rx, cur_tx, 0, 0);

    if (!mStandby && mFd >= 0) {
        ::close(mFd);
        mFd = -1;
    }

    mStandby = true;

    if (support_aic3254) {
        mHardware->do_aic3254_control(mHardware->get_mMode(),
                                      mHardware->get_mRecordState(),
                                      mHardware->checkOutputStandby(),
                                      mHardware->get_snd_dev());
    }

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
    snprintf(buffer, SIZE, "\tchannels: %d\n", channels());
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


status_t AudioHardware::AudioStreamOutMSM72xx::setParameters(const String8& keyValuePairs)
{
    AudioParameter param = AudioParameter(keyValuePairs);
    String8 key = String8(AudioParameter::keyRouting);
    status_t status = NO_ERROR;
    int device;
    LOGV("AudioStreamOutMSM72xx::setParameters() %s", keyValuePairs.string());

    if (param.getInt(key, device) == NO_ERROR) {
        mDevices = device;
        LOGV("set output routing %x", mDevices);
        status = mHardware->doRouting(NULL);
        param.remove(key);
    }

    if (param.size()) {
        status = BAD_VALUE;
    }
    return status;
}

String8 AudioHardware::AudioStreamOutMSM72xx::getParameters(const String8& keys)
{
    AudioParameter param = AudioParameter(keys);
    String8 value;
    String8 key = String8(AudioParameter::keyRouting);

    if (param.get(key, value) == NO_ERROR) {
        LOGV("get routing %x", mDevices);
        param.addInt(key, (int)mDevices);
    }

    LOGV("AudioStreamOutMSM72xx::getParameters() %s", param.toString().string());
    return param.toString();
}

status_t AudioHardware::AudioStreamOutMSM72xx::getRenderPosition(uint32_t *dspFrames)
{
    //TODO: enable when supported by driver
    return INVALID_OPERATION;
}


// ----------------------------------------------------------------------------

AudioHardware::AudioStreamInMSM72xx::AudioStreamInMSM72xx() :
    mHardware(0), mFd(-1), mState(AUDIO_INPUT_CLOSED), mRetryCount(0),
    mFormat(AUDIO_HW_IN_FORMAT), mChannels(AUDIO_HW_IN_CHANNELS),
    mSampleRate(AUDIO_HW_IN_SAMPLERATE), mBufferSize(AUDIO_HW_IN_BUFFERSIZE),
    mAcoustics((AudioSystem::audio_in_acoustics)0), mDevices(0)
{
}

status_t AudioHardware::AudioStreamInMSM72xx::set(
        AudioHardware* hw, uint32_t devices, int *pFormat, uint32_t *pChannels, uint32_t *pRate,
        AudioSystem::audio_in_acoustics acoustic_flags)
{
    if ((pFormat == 0) ||
        ((*pFormat != AUDIO_HW_IN_FORMAT) &&
#ifdef WITH_QCOM_SPEECH
         (*pFormat != AudioSystem::AMR_NB) &&
         (*pFormat != AudioSystem::EVRC) &&
         (*pFormat != AudioSystem::QCELP) &&
#endif
         (*pFormat != AudioSystem::AAC)))
    {
        *pFormat = AUDIO_HW_IN_FORMAT;
        return BAD_VALUE;
    }

    if((*pFormat == AudioSystem::AAC) && (*pChannels & (AudioSystem::CHANNEL_IN_VOICE_DNLINK |  AudioSystem::CHANNEL_IN_VOICE_UPLINK))) {
        LOGE("voice call recording in AAC format does not support");
        return BAD_VALUE;
    }

    if (pRate == 0) {
        return BAD_VALUE;
    }
    uint32_t rate = hw->getInputSampleRate(*pRate);
    if (rate != *pRate) {
        *pRate = rate;
        return BAD_VALUE;
    }

    if (pChannels == 0 || (*pChannels & (AudioSystem::CHANNEL_IN_MONO | AudioSystem::CHANNEL_IN_STEREO)) == 0) {
        *pChannels = AUDIO_HW_IN_CHANNELS;
        return BAD_VALUE;
    }

    mHardware = hw;

    LOGV("AudioStreamInMSM72xx::set(%d, %d, %u)", *pFormat, *pChannels, *pRate);
    if (mFd >= 0) {
        LOGE("Audio record already open");
        return -EPERM;
    }
    status_t status =0;
    struct msm_voicerec_mode voc_rec_cfg;
    if(devices == AudioSystem::DEVICE_IN_FM_RX_A2DP) {
        status = ::open("/dev/msm_a2dp_in", O_RDONLY);
        if (status < 0) {
            LOGE("Cannot open /dev/msm_a2dp_in errno: %d", errno);
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
        config.channel_count = AudioSystem::popCount((*pChannels) & (AudioSystem::CHANNEL_IN_STEREO | AudioSystem::CHANNEL_IN_MONO));
        config.sample_rate = *pRate;
        config.buffer_size = bufferSize();
        config.buffer_count = 2;
        config.codec_type = CODEC_TYPE_PCM;
        status = ioctl(mFd, AUDIO_SET_CONFIG, &config);
        if (status < 0) {
            LOGE("Cannot set config");
            if (ioctl(mFd, AUDIO_GET_CONFIG, &config) == 0) {
                if (config.channel_count == 1) {
                    *pChannels = AudioSystem::CHANNEL_IN_MONO;
                } else {
                    *pChannels = AudioSystem::CHANNEL_IN_STEREO;
                }
                *pRate = config.sample_rate;
            }
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

        mDevices = devices;
        mFormat = AUDIO_HW_IN_FORMAT;
        mChannels = *pChannels;
        mSampleRate = config.sample_rate;
        mBufferSize = config.buffer_size;
    }
    else if(*pFormat == AUDIO_HW_IN_FORMAT)
    {
        // open audio input device
        status = ::open("/dev/msm_pcm_in", O_RDONLY);
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
        config.channel_count = AudioSystem::popCount((*pChannels) & (AudioSystem::CHANNEL_IN_STEREO | AudioSystem::CHANNEL_IN_MONO));
        config.sample_rate = *pRate;
        config.buffer_size = bufferSize();
        config.buffer_count = 2;
        config.codec_type = CODEC_TYPE_PCM;
        status = ioctl(mFd, AUDIO_SET_CONFIG, &config);
        if (status < 0) {
            LOGE("Cannot set config");
            if (ioctl(mFd, AUDIO_GET_CONFIG, &config) == 0) {
                if (config.channel_count == 1) {
                    *pChannels = AudioSystem::CHANNEL_IN_MONO;
                } else {
                    *pChannels = AudioSystem::CHANNEL_IN_STEREO;
                }
                *pRate = config.sample_rate;
            }
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

        mDevices = devices;
        mFormat = AUDIO_HW_IN_FORMAT;
        mChannels = *pChannels;
        if (mDevices == AudioSystem::DEVICE_IN_VOICE_CALL)
         {
            if ((mChannels & AudioSystem::CHANNEL_IN_VOICE_DNLINK) &&
                (mChannels & AudioSystem::CHANNEL_IN_VOICE_UPLINK)) {
                 LOGI("Recording Source: Voice Call Both Uplink and Downlink");
                 voc_rec_cfg.rec_mode = VOC_REC_BOTH;
            } else if (mChannels & AudioSystem::CHANNEL_IN_VOICE_DNLINK) {
                 LOGI("Recording Source: Voice Call DownLink");
                 voc_rec_cfg.rec_mode = VOC_REC_DOWNLINK;
            } else if (mChannels & AudioSystem::CHANNEL_IN_VOICE_UPLINK) {
                 LOGI("Recording Source: Voice Call UpLink");
                 voc_rec_cfg.rec_mode = VOC_REC_UPLINK;
            }
            if (ioctl(mFd, AUDIO_SET_INCALL, &voc_rec_cfg))
            {
                LOGE("Error: AUDIO_SET_INCALL failed\n");
                goto  Error;
            }
        }
        mSampleRate = config.sample_rate;
        mBufferSize = config.buffer_size;
    }
#ifdef WITH_QCOM_SPEECH
    else if (*pFormat == AudioSystem::EVRC)
    {
        LOGI("Recording format: EVRC");
        // open evrc input device
        status = ::open(EVRC_DEVICE_IN, O_RDONLY);
        if (status < 0) {
            LOGE("Cannot open evrc device for read");
            goto Error;
        }
        mFd = status;
        mDevices = devices;
        mChannels = *pChannels;

        if (mDevices == AudioSystem::DEVICE_IN_VOICE_CALL)
        {
            if ((mChannels & AudioSystem::CHANNEL_IN_VOICE_DNLINK) &&
                (mChannels & AudioSystem::CHANNEL_IN_VOICE_UPLINK)) {
                LOGI("Recording Source: Voice Call Both Uplink and Downlink");
                voc_rec_cfg.rec_mode = VOC_REC_BOTH;
            } else if (mChannels & AudioSystem::CHANNEL_IN_VOICE_DNLINK) {
                LOGI("Recording Source: Voice Call DownLink");
                voc_rec_cfg.rec_mode = VOC_REC_DOWNLINK;
            } else if (mChannels & AudioSystem::CHANNEL_IN_VOICE_UPLINK) {
                LOGI("Recording Source: Voice Call UpLink");
                voc_rec_cfg.rec_mode = VOC_REC_UPLINK;
            }

            if (ioctl(mFd, AUDIO_SET_INCALL, &voc_rec_cfg))
            {
                LOGE("Error: AUDIO_SET_INCALL failed\n");
                goto  Error;
            }
        }

        // Config param
        struct msm_audio_stream_config config;
        if(ioctl(mFd, AUDIO_GET_STREAM_CONFIG, &config))
        {
            LOGE(" Error getting buf config param AUDIO_GET_STREAM_CONFIG \n");
            goto  Error;
        }

        LOGV("The Config buffer size is %d", config.buffer_size);
        LOGV("The Config buffer count is %d", config.buffer_count);

        mSampleRate = 8000;
        mFormat = *pFormat;
        mBufferSize = 230;
        struct msm_audio_evrc_enc_config evrc_enc_cfg;

        if (ioctl(mFd, AUDIO_GET_EVRC_ENC_CONFIG, &evrc_enc_cfg))
        {
            LOGE("Error: AUDIO_GET_EVRC_ENC_CONFIG failed\n");
            goto  Error;
        }

        LOGV("The Config cdma_rate is %d", evrc_enc_cfg.cdma_rate);
        LOGV("The Config min_bit_rate is %d", evrc_enc_cfg.min_bit_rate);
        LOGV("The Config max_bit_rate is %d", evrc_enc_cfg.max_bit_rate);

        evrc_enc_cfg.min_bit_rate = 4;
        evrc_enc_cfg.max_bit_rate = 4;

        if (ioctl(mFd, AUDIO_SET_EVRC_ENC_CONFIG, &evrc_enc_cfg))
        {
            LOGE("Error: AUDIO_SET_EVRC_ENC_CONFIG failed\n");
            goto  Error;
        }
    }
    else if (*pFormat == AudioSystem::QCELP)
    {
        LOGI("Recording format: QCELP");
        // open qcelp input device
        status = ::open(QCELP_DEVICE_IN, O_RDONLY);
        if (status < 0) {
            LOGE("Cannot open qcelp device for read");
            goto Error;
        }
        mFd = status;
        mDevices = devices;
        mChannels = *pChannels;

        if (mDevices == AudioSystem::DEVICE_IN_VOICE_CALL)
        {
            if ((mChannels & AudioSystem::CHANNEL_IN_VOICE_DNLINK) &&
                (mChannels & AudioSystem::CHANNEL_IN_VOICE_UPLINK)) {
                LOGI("Recording Source: Voice Call Both Uplink and Downlink");
                voc_rec_cfg.rec_mode = VOC_REC_BOTH;
            } else if (mChannels & AudioSystem::CHANNEL_IN_VOICE_DNLINK) {
                LOGI("Recording Source: Voice Call DownLink");
                voc_rec_cfg.rec_mode = VOC_REC_DOWNLINK;
            } else if (mChannels & AudioSystem::CHANNEL_IN_VOICE_UPLINK) {
                LOGI("Recording Source: Voice Call UpLink");
                voc_rec_cfg.rec_mode = VOC_REC_UPLINK;
            }

            if (ioctl(mFd, AUDIO_SET_INCALL, &voc_rec_cfg))
            {
                LOGE("Error: AUDIO_SET_INCALL failed\n");
                goto  Error;
            }
        }

        // Config param
        struct msm_audio_stream_config config;
        if(ioctl(mFd, AUDIO_GET_STREAM_CONFIG, &config))
        {
            LOGE(" Error getting buf config param AUDIO_GET_STREAM_CONFIG \n");
            goto  Error;
        }

        LOGV("The Config buffer size is %d", config.buffer_size);
        LOGV("The Config buffer count is %d", config.buffer_count);

        mSampleRate =8000;
        mFormat = *pFormat;
        mBufferSize = 350;

        struct msm_audio_qcelp_enc_config qcelp_enc_cfg;

        if (ioctl(mFd, AUDIO_GET_QCELP_ENC_CONFIG, &qcelp_enc_cfg))
        {
            LOGE("Error: AUDIO_GET_QCELP_ENC_CONFIG failed\n");
            goto  Error;
        }

        LOGV("The Config cdma_rate is %d", qcelp_enc_cfg.cdma_rate);
        LOGV("The Config min_bit_rate is %d", qcelp_enc_cfg.min_bit_rate);
        LOGV("The Config max_bit_rate is %d", qcelp_enc_cfg.max_bit_rate);

        qcelp_enc_cfg.min_bit_rate = 4;
        qcelp_enc_cfg.max_bit_rate = 4;

        if (ioctl(mFd, AUDIO_SET_QCELP_ENC_CONFIG, &qcelp_enc_cfg))
        {
          LOGE("Error: AUDIO_SET_QCELP_ENC_CONFIG failed\n");
          goto  Error;
        }
    }
    else if (*pFormat == AudioSystem::AMR_NB)
    {
          LOGI("Recording format: AMR_NB");
          // open amr_nb input device
          status = ::open(AMRNB_DEVICE_IN, O_RDONLY);
          if (status < 0) {
              LOGE("Cannot open amr_nb device for read");
              goto Error;
          }
          mFd = status;
          mDevices = devices;
          mChannels = *pChannels;

          if (mDevices == AudioSystem::DEVICE_IN_VOICE_CALL)
          {
              if ((mChannels & AudioSystem::CHANNEL_IN_VOICE_DNLINK) &&
                     (mChannels & AudioSystem::CHANNEL_IN_VOICE_UPLINK)) {
                  LOGI("Recording Source: Voice Call Both Uplink and Downlink");
                  voc_rec_cfg.rec_mode = VOC_REC_BOTH;
              } else if (mChannels & AudioSystem::CHANNEL_IN_VOICE_DNLINK) {
                  LOGI("Recording Source: Voice Call DownLink");
                  voc_rec_cfg.rec_mode = VOC_REC_DOWNLINK;
              } else if (mChannels & AudioSystem::CHANNEL_IN_VOICE_UPLINK) {
                  LOGI("Recording Source: Voice Call UpLink");
                  voc_rec_cfg.rec_mode = VOC_REC_UPLINK;
              }

              if (ioctl(mFd, AUDIO_SET_INCALL, &voc_rec_cfg))
              {
                 LOGE("Error: AUDIO_SET_INCALL failed\n");
                 goto  Error;
              }
          }

          // Config param 
          struct msm_audio_stream_config config;
          if(ioctl(mFd, AUDIO_GET_STREAM_CONFIG, &config))
          {
            LOGE(" Error getting buf config param AUDIO_GET_STREAM_CONFIG \n");
            goto  Error;
          }

          LOGV("The Config buffer size is %d", config.buffer_size);
          LOGV("The Config buffer count is %d", config.buffer_count);

          mSampleRate =8000;
          mFormat = *pFormat;
          mBufferSize = 320;
          struct msm_audio_amrnb_enc_config_v2 amr_nb_cfg;

          if (ioctl(mFd, AUDIO_GET_AMRNB_ENC_CONFIG_V2, &amr_nb_cfg))
          {
            LOGE("Error: AUDIO_GET_AMRNB_ENC_CONFIG_V2 failed\n");
            goto  Error;
          }

          LOGV("The Config band_mode is %d", amr_nb_cfg.band_mode);
          LOGV("The Config dtx_enable is %d", amr_nb_cfg.dtx_enable);
          LOGV("The Config frame_format is %d", amr_nb_cfg.frame_format);

          amr_nb_cfg.band_mode = 7; // Bit Rate 12.2 kbps MR122
          amr_nb_cfg.dtx_enable= 0;
          amr_nb_cfg.frame_format = 0; // IF1

          if (ioctl(mFd, AUDIO_SET_AMRNB_ENC_CONFIG_V2, &amr_nb_cfg))
          {
            LOGE("Error: AUDIO_SET_AMRNB_ENC_CONFIG_V2 failed\n");
            goto  Error;
          }
    }
#endif
    else if (*pFormat == AudioSystem::AAC)
    {
        LOGI("Recording format: AAC");
        // open aac input device
        status = ::open(AAC_DEVICE_IN, O_RDWR);
        if (status < 0) {
            LOGE("Cannot open aac device for read");
            goto Error;
        }
        mFd = status;

        struct msm_audio_stream_config config;
        if(ioctl(mFd, AUDIO_GET_STREAM_CONFIG, &config))
        {
          LOGE(" Error getting buf config param AUDIO_GET_STREAM_CONFIG \n");
          goto  Error;
        }

        LOGE("The Config buffer size is %d", config.buffer_size);
        LOGE("The Config buffer count is %d", config.buffer_count);

        struct msm_audio_aac_enc_config aac_enc_cfg;
        if (ioctl(mFd, AUDIO_GET_AAC_ENC_CONFIG, &aac_enc_cfg))
        {
          LOGE("Error: AUDIO_GET_AAC_ENC_CONFIG failed\n");
          goto  Error;
        }

        LOGV("The Config channels is %d", aac_enc_cfg.channels);
        LOGV("The Config sample_rate is %d", aac_enc_cfg.sample_rate);
        LOGV("The Config bit_rate is %d", aac_enc_cfg.bit_rate);
        LOGV("The Config stream_format is %d", aac_enc_cfg.stream_format);

        mDevices = devices;
        mChannels = *pChannels;
        aac_enc_cfg.sample_rate = mSampleRate = *pRate;
        mFormat = *pFormat;
        mBufferSize = 2048;
        if (*pChannels & (AudioSystem::CHANNEL_IN_MONO))
            aac_enc_cfg.channels =  1;
        else if (*pChannels & (AudioSystem::CHANNEL_IN_STEREO))
            aac_enc_cfg.channels =  2;
        aac_enc_cfg.bit_rate = 128000;

        LOGV("Setting the Config channels is %d", aac_enc_cfg.channels);
        LOGV("Setting the Config sample_rate is %d", aac_enc_cfg.sample_rate);
        LOGV("Setting the Config bit_rate is %d", aac_enc_cfg.bit_rate);
        LOGV("Setting the Config stream_format is %d", aac_enc_cfg.stream_format);

        if (ioctl(mFd, AUDIO_SET_AAC_ENC_CONFIG, &aac_enc_cfg))
        {
          LOGE("Error: AUDIO_SET_AAC_ENC_CONFIG failed\n");
          goto  Error;
        }
    }
    //mHardware->setMicMute_nosync(false);
    mState = AUDIO_INPUT_OPENED;

    return NO_ERROR;

Error:
    if (mFd >= 0) {
        ::close(mFd);
        mFd = -1;
    }
    return status;
}

AudioHardware::AudioStreamInMSM72xx::~AudioStreamInMSM72xx()
{
    LOGV("AudioStreamInMSM72xx destructor");
    standby();
}

ssize_t AudioHardware::AudioStreamInMSM72xx::read( void* buffer, ssize_t bytes)
{
    unsigned short dec_id = INVALID_DEVICE;
    // LOGV("AudioStreamInMSM72xx::read(%p, %ld)", buffer, bytes);
    if (!mHardware) return -1;

    size_t count = bytes;
    size_t  aac_framesize= bytes;
    uint8_t* p = static_cast<uint8_t*>(buffer);
    uint32_t* recogPtr = (uint32_t *)p;
    uint16_t* frameCountPtr;
    uint16_t* frameSizePtr;

    if (mState < AUDIO_INPUT_OPENED) {
        AudioHardware *hw = mHardware;
        hw->mLock.lock();
        status_t status = set(hw, mDevices, &mFormat, &mChannels, &mSampleRate, mAcoustics);
        if (status != NO_ERROR) {
            hw->mLock.unlock();
            return -1;
        }
        if((mDevices == AudioSystem::DEVICE_IN_FM_RX) || (mDevices == AudioSystem::DEVICE_IN_FM_RX_A2DP) ){
/*
            if(ioctl(mFd, AUDIO_GET_SESSION_ID, &dec_id)) {
                LOGE("AUDIO_GET_SESSION_ID failed*********");
                hw->mLock.unlock();
                return -1;
            }

            if(enableDevice(DEVICE_FMRADIO_STEREO_TX, 1)) {
                LOGE("enableDevice failed for device %d",DEVICE_FMRADIO_STEREO_TX);
                hw->mLock.unlock();
                return -1;
             }

            if(msm_route_stream(PCM_REC, dec_id, DEV_ID(DEVICE_FMRADIO_STEREO_TX), 1)) {
                LOGE("msm_route_stream failed");
                hw->mLock.unlock();
                return -1;
            }
            mFirstread = false;
            if (mDevices == AudioSystem::DEVICE_IN_FM_RX_A2DP) {
                addToTable(dec_id,cur_tx,INVALID_DEVICE,FM_A2DP,true);
                mFmRec = FM_A2DP_REC;
            }
            else {
                addToTable(dec_id,cur_tx,INVALID_DEVICE,FM_REC,true);
                mFmRec = FM_FILE_REC;
            }
            hw->mLock.unlock();
*/
        } else {
            hw->mLock.unlock();
            if(ioctl(mFd, AUDIO_GET_SESSION_ID, &dec_id)) {
                LOGE("AUDIO_GET_SESSION_ID failed*********");
                return -1;
            }
            LOGV("dec_id = %d,cur_tx= %d", dec_id, cur_tx);
            if(cur_tx == INVALID_DEVICE)
                cur_tx = DEVICE_HANDSET_TX;

            Mutex::Autolock lock(mDeviceSwitchLock);

            if(enableDevice(cur_tx, 1)) {
                LOGE("enableDevice failed, device %d", cur_tx);
                return -1;
            }

            uint32_t tx_acdb_id = mHardware->getACDB(MOD_REC, mHardware->get_snd_dev());
            updateACDB(cur_rx, cur_tx, 0, tx_acdb_id);

            if(msm_route_stream(PCM_REC, dec_id, DEV_ID(cur_tx), 1)) {
                LOGE("msm_route_stream failed");
                return -1;
            }
            addToTable(dec_id,cur_tx, INVALID_DEVICE,PCM_REC, true);
            mFirstread = false;
        }
    }

    if (mState < AUDIO_INPUT_STARTED) {
        // force routing to input device
        mHardware->set_mRecordState(1);
        mHardware->clearCurDevice();
        mHardware->doRouting(this);
        if (support_aic3254) {
            int snd_dev = mHardware->get_snd_dev();
            mHardware->aic3254_config(snd_dev, "Original");
            mHardware->do_aic3254_control(mHardware->get_mMode(),
                                          1,
                                          mHardware->checkOutputStandby(),
                                          snd_dev);
        }
        if (ioctl(mFd, AUDIO_START, 0)) {
            LOGE("Error starting record");
            standby();
            return -1;
        }
        mState = AUDIO_INPUT_STARTED;
    }

    bytes = 0;
    if(mFormat == AUDIO_HW_IN_FORMAT)
    {
        while (count) {
            ssize_t bytesRead = ::read(mFd, buffer, count);
            if (bytesRead >= 0) {
                count -= bytesRead;
                p += bytesRead;
                bytes += bytesRead;
                if(!mFirstread)
                {
                   mFirstread = true;
                   break;
                }
            } else {
                if (errno != EAGAIN) return bytesRead;
                mRetryCount++;
                LOGW("EAGAIN - retrying");
            }
        }
    }
#ifdef WITH_QCOM_SPEECH
    else if ((mFormat == AudioSystem::EVRC) || (mFormat == AudioSystem::QCELP) || (mFormat == AudioSystem::AMR_NB))
    {
        uint8_t readBuf[36];
        uint8_t *dataPtr;
        while (count) {
            dataPtr = readBuf;
            ssize_t bytesRead = ::read(mFd, readBuf, 36);
            if (bytesRead >= 0) {
                if (mFormat == AudioSystem::AMR_NB){
                   amr_transcode(dataPtr,p);
                   p += AMRNB_FRAME_SIZE;
                   count -= AMRNB_FRAME_SIZE;
                   bytes += AMRNB_FRAME_SIZE;
                   if(!mFirstread)
                   {
                      mFirstread = true;
                      break;
                   }
                }
                else {
                    dataPtr++;
                    if (mFormat == AudioSystem::EVRC){
                       memcpy(p, dataPtr, EVRC_FRAME_SIZE);
                       p += EVRC_FRAME_SIZE;
                       count -= EVRC_FRAME_SIZE;
                       bytes += EVRC_FRAME_SIZE;
                       if(!mFirstread)
                       {
                          mFirstread = true;
                          break;
                       }
                    }
                    else if (mFormat == AudioSystem::QCELP){
                       memcpy(p, dataPtr, QCELP_FRAME_SIZE);
                       p += QCELP_FRAME_SIZE;
                       count -= QCELP_FRAME_SIZE;
                       bytes += QCELP_FRAME_SIZE;
                       if(!mFirstread)
                       {
                          mFirstread = true;
                          break;
                       }
                    }
                }

            } else {
                if (errno != EAGAIN) return bytesRead;
                mRetryCount++;
                LOGW("EAGAIN - retrying");
            }
        }
    }
#endif
    else if (mFormat == AudioSystem::AAC)
    {
        *((uint32_t*)recogPtr) = 0x51434F4D ;// ('Q','C','O', 'M') Number to identify format as AAC by higher layers
        recogPtr++;
        frameCountPtr = (uint16_t*)recogPtr;
        *frameCountPtr = 0;
        p += 3*sizeof(uint16_t);
        count -= 3*sizeof(uint16_t);

        while (count > 0) {
            frameSizePtr = (uint16_t *)p;
            p += sizeof(uint16_t);
            if(!(count > 2)) break;
            count -= sizeof(uint16_t);

            ssize_t bytesRead = ::read(mFd, p, count);
            if (bytesRead > 0) {
                LOGV("Number of Bytes read = %d", bytesRead);
                count -= bytesRead;
                p += bytesRead;
                bytes += bytesRead;
                LOGV("Total Number of Bytes read = %d", bytes);

                *frameSizePtr =  bytesRead;
                (*frameCountPtr)++;
                if(!mFirstread)
                {
                   mFirstread = true;
                   break;
                }
                /*Typical frame size for AAC is around 250 bytes. So we have
                 * taken the minimum buffer size as twice of this size i.e.
                 * 512 to avoid short reads from driver */
                if(count < 512)
                {
                   LOGI("buffer passed to driver %d, is less than the min 512 bytes", count);
                   break;
                }
            }
            else if(bytesRead == 0)
            {
             LOGI("Bytes Read = %d ,Buffer no longer sufficient", bytesRead);
             break;
            } else {
                if (errno != EAGAIN) return bytesRead;
                mRetryCount++;
                LOGW("EAGAIN - retrying");
            }
        }
    }

    if (mFormat == AudioSystem::AAC)
        return aac_framesize;

        return bytes;
}

status_t AudioHardware::AudioStreamInMSM72xx::standby()
{
    bool isDriverClosed = false;
    LOGD("AudioStreamInMSM72xx::standby()");
    Routing_table* temp = NULL;

    if (mHardware) {
        mHardware->set_mRecordState(0);
        if (support_aic3254) {
            int snd_dev = mHardware->get_snd_dev();
            mHardware->aic3254_config(snd_dev, "Original");
            mHardware->do_aic3254_control(mHardware->get_mMode(),
                                          0,
                                          mHardware->checkOutputStandby(),
                                          snd_dev);
        }
    }

    if (!mHardware) return -1;
    if (mState > AUDIO_INPUT_CLOSED) {
        if (mFd >= 0) {
            ::close(mFd);
            mFd = -1;
            LOGV("driver closed");
            isDriverClosed = true;
        }
        //mHardware->checkMicMute();
        mState = AUDIO_INPUT_CLOSED;
    }

/*
    if (mFmRec == FM_A2DP_REC) {
        //A2DP Recording
        temp = getNodeByStreamType(FM_A2DP);
        if(temp == NULL)
            return NO_ERROR;
        if(msm_route_stream(PCM_PLAY, temp->dec_id, DEV_ID(DEVICE_FMRADIO_STEREO_TX), 0)) {
           LOGE("msm_route_stream failed");
           return 0;
        }
        deleteFromTable(FM_A2DP);
        if(enableDevice(DEVICE_FMRADIO_STEREO_TX, 0)) {
            LOGE("Disabling device failed for device %d", DEVICE_FMRADIO_STEREO_TX);
        }
    }
    if (mFmRec == FM_FILE_REC) {
        //FM Recording
        temp = getNodeByStreamType(FM_REC);
        if(temp == NULL)
            return NO_ERROR;
        if(msm_route_stream(PCM_PLAY, temp->dec_id, DEV_ID(DEVICE_FMRADIO_STEREO_TX), 0)) {
           LOGE("msm_route_stream failed");
           return 0;
        }
        deleteFromTable(FM_REC);
    }
*/
    temp = getNodeByStreamType(PCM_REC);
    if(temp == NULL)
        return NO_ERROR;

    if(isDriverClosed){
        LOGD("Deroute pcm in stream");
        if(msm_route_stream(PCM_REC, temp->dec_id,DEV_ID(temp->dev_id), 0)) {
            LOGE("could not set stream routing\n");
            deleteFromTable(PCM_REC);
            return -1;
        }
        LOGV("Disable device");
        deleteFromTable(PCM_REC);
        updateDeviceInfo(cur_rx, cur_tx, 0, 0);
    }//mRecordingSession condition.
    // restore output routing if necessary
    mHardware->clearCurDevice();
    mHardware->doRouting(this);

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
    snprintf(buffer, SIZE, "\tchannels: %d\n", channels());
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

status_t AudioHardware::AudioStreamInMSM72xx::setParameters(const String8& keyValuePairs)
{
    AudioParameter param = AudioParameter(keyValuePairs);
    status_t status = NO_ERROR;
    int device;
    String8 key = String8(AudioParameter::keyInputSource);
    int source;
    LOGV("AudioStreamInMSM72xx::setParameters() %s", keyValuePairs.string());

    key = String8(AudioParameter::keyRouting);
    if (param.getInt(key, device) == NO_ERROR) {
        LOGV("set input routing %x", device);
        if (device & (device - 1)) {
            status = BAD_VALUE;
        } else {
            mDevices = device;
            status = mHardware->doRouting(this);
        }
        param.remove(key);
    }

    if (param.size()) {
        status = BAD_VALUE;
    }
    return status;
}

String8 AudioHardware::AudioStreamInMSM72xx::getParameters(const String8& keys)
{
    AudioParameter param = AudioParameter(keys);
    String8 value;
    String8 key = String8(AudioParameter::keyRouting);

    if (param.get(key, value) == NO_ERROR) {
        LOGV("get routing %x", mDevices);
        param.addInt(key, (int)mDevices);
    }

    LOGV("AudioStreamInMSM72xx::getParameters() %s", param.toString().string());
    return param.toString();
}

// getActiveInput_l() must be called with mLock held
AudioHardware::AudioStreamInMSM72xx *AudioHardware::getActiveInput_l()
{
    for (size_t i = 0; i < mInputs.size(); i++) {
        // return first input found not being in standby mode
        // as only one input can be in this state
        if (mInputs[i]->state() > AudioStreamInMSM72xx::AUDIO_INPUT_CLOSED) {
            return mInputs[i];
        }
    }

    return NULL;
}
// ----------------------------------------------------------------------------

extern "C" AudioHardwareInterface* createAudioHardware(void) {
    return new AudioHardware();
}
#ifdef WITH_QCOM_SPEECH
/*===========================================================================

FUNCTION amrsup_frame_len

DESCRIPTION
  This function will determine number of bytes of AMR vocoder frame length
based on the frame type and frame rate.

DEPENDENCIES
  None.

RETURN VALUE
  number of bytes of AMR frame

SIDE EFFECTS
  None.

===========================================================================*/
int amrsup_frame_len_bits(
  amrsup_frame_type frame_type,
  amrsup_mode_type amr_mode
)
{
  int frame_len=0;


  switch (frame_type)
  {
    case AMRSUP_SPEECH_GOOD :
    case AMRSUP_SPEECH_DEGRADED :
    case AMRSUP_ONSET :
    case AMRSUP_SPEECH_BAD :
      if (amr_mode >= AMRSUP_MODE_MAX)
      {
        frame_len = 0;
      }
      else
      {
        frame_len = amrsup_122_framing.len_a
                    + amrsup_122_framing.len_b
                    + amrsup_122_framing.len_c;
      }
      break;

    case AMRSUP_SID_FIRST :
    case AMRSUP_SID_UPDATE :
    case AMRSUP_SID_BAD :
      frame_len = AMR_CLASS_A_BITS_SID;
      break;

    case AMRSUP_NO_DATA :
    case AMRSUP_SPEECH_LOST :
    default :
      frame_len = 0;
  }

  return frame_len;
}

/*===========================================================================

FUNCTION amrsup_frame_len

DESCRIPTION
  This function will determine number of bytes of AMR vocoder frame length
based on the frame type and frame rate.

DEPENDENCIES
  None.

RETURN VALUE
  number of bytes of AMR frame

SIDE EFFECTS
  None.

===========================================================================*/
int amrsup_frame_len(
  amrsup_frame_type frame_type,
  amrsup_mode_type amr_mode
)
{
  int frame_len = amrsup_frame_len_bits(frame_type, amr_mode);

  frame_len = (frame_len + 7) / 8;
  return frame_len;
}

/*===========================================================================

FUNCTION amrsup_tx_order

DESCRIPTION
  Use a bit ordering table to order bits from their original sequence.

DEPENDENCIES
  None.

RETURN VALUE
  None.

SIDE EFFECTS
  None.

===========================================================================*/
void amrsup_tx_order(
  unsigned char *dst_frame,
  int         *dst_bit_index,
  unsigned char *src,
  int         num_bits,
  const unsigned short *order
)
{
  unsigned long dst_mask = 0x00000080 >> ((*dst_bit_index) & 0x7);
  unsigned char *dst = &dst_frame[((unsigned int) *dst_bit_index) >> 3];
  unsigned long src_bit, src_mask;

  /* Prepare to process all bits
  */
  *dst_bit_index += num_bits;
  num_bits++;

  while(--num_bits) {
    /* Get the location of the bit in the input buffer */
    src_bit  = (unsigned long ) *order++;
    src_mask = 0x00000080 >> (src_bit & 0x7);

    /* Set the value of the output bit equal to the input bit */
    if (src[src_bit >> 3] & src_mask) {
      *dst |= (unsigned char ) dst_mask;
    }

    /* Set the destination bit mask and increment pointer if necessary */
    dst_mask >>= 1;
    if (dst_mask == 0) {
      dst_mask = 0x00000080;
      dst++;
    }
  }
} /* amrsup_tx_order */

/*===========================================================================

FUNCTION amrsup_if1_framing

DESCRIPTION
  Performs the transmit side framing function.  Generates AMR IF1 ordered data
  from the vocoder packet and frame type.

DEPENDENCIES
  None.

RETURN VALUE
  number of bytes of encoded frame.
  if1_frame : IF1-encoded frame.
  if1_frame_info : holds frame information of IF1-encoded frame.

SIDE EFFECTS
  None.

===========================================================================*/
static int amrsup_if1_framing(
  unsigned char              *vocoder_packet,
  amrsup_frame_type          frame_type,
  amrsup_mode_type           amr_mode,
  unsigned char              *if1_frame,
  amrsup_if1_frame_info_type *if1_frame_info
)
{
  amrsup_frame_order_type *ordering_table;
  int frame_len = 0;
  int i;

  if(amr_mode >= AMRSUP_MODE_MAX)
  {
    LOGE("Invalid AMR_Mode : %d",amr_mode);
    return 0;
  }

  /* Initialize IF1 frame data and info */
  if1_frame_info->fqi = true;

  if1_frame_info->amr_type = AMRSUP_CODEC_AMR_NB;

  memset(if1_frame, 0,
           amrsup_frame_len(AMRSUP_SPEECH_GOOD, AMRSUP_MODE_1220));


  switch (frame_type)
  {
    case AMRSUP_SID_BAD:
      if1_frame_info->fqi = false;
      /* fall thru */

    case AMRSUP_SID_FIRST:
    case AMRSUP_SID_UPDATE:
      /* Set frame type index */
      if1_frame_info->frame_type_index
      = AMRSUP_FRAME_TYPE_INDEX_AMR_SID;


      /* ===== Encoding SID frame ===== */
      /* copy the sid frame to class_a data */
      for (i=0; i<5; i++)
      {
        if1_frame[i] = vocoder_packet[i];
      }

      /* Set the SID type : SID_FIRST: Bit 35 = 0, SID_UPDATE : Bit 35 = 1 */
      if (frame_type == AMRSUP_SID_FIRST)
      {
        if1_frame[4] &= ~0x10;
      }

      if (frame_type == AMRSUP_SID_UPDATE)
      {
        if1_frame[4] |= 0x10;
      }
      else
      {
      /* Set the mode (Bit 36 - 38 = amr_mode with bits swapped)
      */
      if1_frame[4] |= (((unsigned char)amr_mode << 3) & 0x08)
        | (((unsigned char)amr_mode << 1) & 0x04) | (((unsigned char)amr_mode >> 1) & 0x02);

      frame_len = AMR_CLASS_A_BITS_SID;
      }

      break;


    case AMRSUP_SPEECH_BAD:
      if1_frame_info->fqi = false;
      /* fall thru */

    case AMRSUP_SPEECH_GOOD:
      /* Set frame type index */

        if1_frame_info->frame_type_index
        = (amrsup_frame_type_index_type)(amr_mode);

      /* ===== Encoding Speech frame ===== */
      /* Clear num bits in frame */
      frame_len = 0;

      /* Select ordering table */
      ordering_table =
      (amrsup_frame_order_type*)&amrsup_122_framing;

      amrsup_tx_order(
        if1_frame,
        &frame_len,
        vocoder_packet,
        ordering_table->len_a,
        ordering_table->class_a
      );

      amrsup_tx_order(
        if1_frame,
        &frame_len,
        vocoder_packet,
        ordering_table->len_b,
        ordering_table->class_b
      );

      amrsup_tx_order(
        if1_frame,
        &frame_len,
        vocoder_packet,
        ordering_table->len_c,
        ordering_table->class_c
      );


      /* frame_len already updated with correct number of bits */
      break;

    default:
      LOGE("Unsupported frame type %d", frame_type);
      /* fall thru */

    case AMRSUP_NO_DATA:
      /* Set frame type index */
      if1_frame_info->frame_type_index = AMRSUP_FRAME_TYPE_INDEX_NO_DATA;

      frame_len = 0;

      break;
  }  /* end switch */


  /* convert bit length to byte length */
  frame_len = (frame_len + 7) / 8;

  return frame_len;
}

static void amr_transcode(unsigned char *src, unsigned char *dst)
{
   amrsup_frame_type frame_type_in = (amrsup_frame_type) *(src++);
   amrsup_mode_type frame_rate_in = (amrsup_mode_type) *(src++);
   amrsup_if1_frame_info_type frame_info_out;
   unsigned char frameheader;

   amrsup_if1_framing(src, frame_type_in, frame_rate_in, dst+1, &frame_info_out);
   frameheader = (frame_info_out.frame_type_index << 3) + (frame_info_out.fqi << 2);
   *dst = frameheader;

   return;
}
#endif

}; // namespace android

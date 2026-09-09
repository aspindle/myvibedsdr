//Copyright (c) 2011-2020 <>< Charles Lohr - Under the MIT/x11 or NewBSD License you choose.
// NO WARRANTY! NO GUARANTEE OF SUPPORT! USE AT YOUR OWN RISK

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "os_generic.h"
#include <GLES3/gl3.h>
#include <asset_manager.h>
#include <asset_manager_jni.h>
#include <android_native_app_glue.h>
#include <android/sensor.h>
#include "CNFGAndroid.h"
#include <time.h>
#include "android_usb_devices.h"
#include "libusb.h"
#include <stdint.h>
#include <rtl-sdr.h>


#include <android/log.h>

#define LOG_TAG "RTLSDR"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)


#define CNFG_IMPLEMENTATION
#define CNFG3D

#include "CNFG.h"
#include <dlfcn.h>

#define FFT_BINS 512
#define SAMPLE_COUNT 512
#define WATERFALL_HEIGHT 300

#define SPECTRUM_Y 350
#define WATERFALL_Y 400
#define SAMPLES_Y 1200

#define SAMPLE_RATE 1000.0f

float loFrequency = 0.0f;

//float samples[SAMPLE_COUNT];
float samplesI[SAMPLE_COUNT];
float samplesQ[SAMPLE_COUNT];

float fftReal[SAMPLE_COUNT];
float fftImag[SAMPLE_COUNT];


float fft[FFT_BINS];
unsigned char waterfall[WATERFALL_HEIGHT][FFT_BINS];
float timeSeconds = 0.0f;
float noiseLevel = 0.2f;
float debugBin50;
float debugBin150;




float mountainangle;
float mountainoffsetx;
float mountainoffsety;

ASensorManager * sm;
const ASensor * as;
bool no_sensor_for_gyro = false;
ASensorEventQueue* aeq;
ALooper * l;


char usbStatus[2048] = "Not checked";
int usbResult = -1;



void SetupIMU()
{
	sm = ASensorManager_getInstance();
	as = ASensorManager_getDefaultSensor( sm, ASENSOR_TYPE_GYROSCOPE );
	no_sensor_for_gyro = as == NULL;
	l = ALooper_prepare( ALOOPER_PREPARE_ALLOW_NON_CALLBACKS );
	aeq = ASensorManager_createEventQueue( sm, (ALooper*)&l, 0, 0, 0 ); //XXX??!?! This looks wrong.
	if(!no_sensor_for_gyro) {
		ASensorEventQueue_enableSensor( aeq, as);
		printf( "setEvent Rate: %d\n", ASensorEventQueue_setEventRate( aeq, as, 10000 ) );
	}

}

float accx, accy, accz;
int accs;

void AccCheck()
{
	if(no_sensor_for_gyro) {
		return;
	}

	ASensorEvent evt;
	do
	{
		ssize_t s = ASensorEventQueue_getEvents( aeq, &evt, 1 );
		if( s <= 0 ) break;
		accx = evt.vector.v[0];
		accy = evt.vector.v[1];
		accz = evt.vector.v[2];
		mountainangle /*degrees*/ -= accz;// * 3.1415 / 360.0;// / 100.0;
		mountainoffsety += accy;
		mountainoffsetx += accx;
		accs++;
	} while( 1 );
}

unsigned frames = 0;
unsigned long iframeno = 0;

void AndroidDisplayKeyboard(int pShow);

int lastbuttonx = 0;
int lastbuttony = 0;
int lastmotionx = 0;
int lastmotiony = 0;
int lastbid = 0;
int lastmask = 0;
int lastkey, lastkeydown;

static int keyboard_up;

//step g iq samples
int rtlIQResult = -999;
rtlsdr_dev_t *rtlDev = NULL;
char debug1[128] = "n1";
char debug2[128] = "n2";
char debug3[128] = "n3";
char debug4[128] = "n4";
char debug5[128] = "n5";
int rtlDeviceOpened = 0;

//Step f librtlsdr communicating with a register(s)
libusb_context *rtlUSBContext = NULL;
libusb_device_handle *rtlUSBHandle = NULL;
int rtlRegisterResult = -999;
int rtlRegisterValue = -1;
int rtlStep = 0;
int rtlWriteResult = -999;
int rtlRegisterTest = 0;
int rtl2832Initialized = 0;


//step e libusb=0 success
int rtlLibUSBResult = -999;



//step d rtl-sdr dongle?
int CheckRTLSDRPermission(void);
int RequestRTLSDRPermission(void);
int CheckForRTLSDR(char *status);
int OpenRTLSDR(void);
int TestLibUSB(int fd);



jobject rtlUsbConnection = NULL;
int rtlUsbFD = -1;

//step c fft
void FFT(void);

//step b dsp
void GenerateSignal(float t);
void DFT(void);
void DrawSamples(void);

// step a
void UpdateFakeFFT(float t);

// necessary
void UpdateWaterfall(void);
void DrawSpectrum(void);
void DrawFrequencyAxis(void);
void DrawWaterfall(void);
void WaterfallColor(
    unsigned char value,
    unsigned char *r,
    unsigned char *g,
    unsigned char *b
);

void HandleKey( int keycode, int bDown )
{
	lastkey = keycode;
	lastkeydown = bDown;
	if( keycode == 10 && !bDown ) { keyboard_up = 0; AndroidDisplayKeyboard( keyboard_up );  }

	if( keycode == 4 ) { AndroidSendToBack( 1 ); } //Handle Physical Back Button.
}

void HandleButton( int x, int y, int button, int bDown )
{
	lastbid = button;
	lastbuttonx = x;
	lastbuttony = y;

	if( bDown ) { keyboard_up = !keyboard_up; AndroidDisplayKeyboard( keyboard_up ); }
}

void HandleMotion( int x, int y, int mask )
{
	lastmask = mask;
	lastmotionx = x;
	lastmotiony = y;
}

#define HMX 162
#define HMY 162
short screenx, screeny;
float Heightmap[HMX*HMY];

extern struct android_app * gapp;

void DrawHeightmap()
{
	int x, y;
	//float fdt = ((iframeno++)%(360*10))/10.0;

	mountainangle += .2;
	if( mountainangle < 0 ) mountainangle += 360;
	if( mountainangle > 360 ) mountainangle -= 360;

	mountainoffsety = mountainoffsety - ((mountainoffsety-100) * .1);

	float eye[3] = { (float)sin(mountainangle*(3.14159/180.0))*30*sin(mountainoffsety/100.), (float)cos(mountainangle*(3.14159/180.0))*30*sin(mountainoffsety/100.), 30*cos(mountainoffsety/100.) };
	float at[3] = { 0,0, 0 };
	float up[3] = { 0,0, 1 };

	tdSetViewport( -1, -1, 1, 1, screenx, screeny );

	tdMode( tdPROJECTION );
	tdIdentity( gSMatrix );
	tdPerspective( 30, ((float)screenx)/((float)screeny), .1, 200., gSMatrix );

	tdMode( tdMODELVIEW );
	tdIdentity( gSMatrix );
	tdTranslate( gSMatrix, 0, 0, -40 );
	tdLookAt( gSMatrix, eye, at, up );

	float scale = 60./HMX;

	for( x = 0; x < HMX-1; x++ )
	for( y = 0; y < HMY-1; y++ )
	{
		float tx = x-HMX/2;
		float ty = y-HMY/2;
		float pta[3];
		float ptb[3];
		float ptc[3];
		float ptd[3];

		float normal[3];
		float lightdir[3] = { .6, -.6, 1 };
		float tmp1[3];
		float tmp2[3];

		RDPoint pto[6];

		pta[0] = (tx+0)*scale; pta[1] = (ty+0)*scale; pta[2] = Heightmap[(x+0)+(y+0)*HMX]*scale;
		ptb[0] = (tx+1)*scale; ptb[1] = (ty+0)*scale; ptb[2] = Heightmap[(x+1)+(y+0)*HMX]*scale;
		ptc[0] = (tx+0)*scale; ptc[1] = (ty+1)*scale; ptc[2] = Heightmap[(x+0)+(y+1)*HMX]*scale;
		ptd[0] = (tx+1)*scale; ptd[1] = (ty+1)*scale; ptd[2] = Heightmap[(x+1)+(y+1)*HMX]*scale;

		tdPSub( pta, ptb, tmp2 );
		tdPSub( ptc, ptb, tmp1 );
		tdCross( tmp1, tmp2, normal );
		tdNormalizeSelf( normal );

		tdFinalPoint( pta, pta );
		tdFinalPoint( ptb, ptb );
		tdFinalPoint( ptc, ptc );
		tdFinalPoint( ptd, ptd );

		if( pta[2] >= 1.0 ) continue;
		if( ptb[2] >= 1.0 ) continue;
		if( ptc[2] >= 1.0 ) continue;
		if( ptd[2] >= 1.0 ) continue;

		if( pta[2] < 0 ) continue;
		if( ptb[2] < 0 ) continue;
		if( ptc[2] < 0 ) continue;
		if( ptd[2] < 0 ) continue;

		pto[0].x = pta[0]; pto[0].y = pta[1];
		pto[1].x = ptb[0]; pto[1].y = ptb[1];
		pto[2].x = ptd[0]; pto[2].y = ptd[1];

		pto[3].x = ptc[0]; pto[3].y = ptc[1];
		pto[4].x = ptd[0]; pto[4].y = ptd[1];
		pto[5].x = pta[0]; pto[5].y = pta[1];

//		CNFGColor(((x+y)&1)?0xFFFFFF:0x000000);

		float bright = tdDot( normal, lightdir );
		if( bright < 0 ) bright = 0;
		CNFGColor( 0xff | ( ( (int)( bright * 90 ) ) << 24 ) );

//		CNFGTackPoly( &pto[0], 3 );		CNFGTackPoly( &pto[3], 3 );
		CNFGTackSegment( pta[0], pta[1], ptb[0], ptb[1] );
		CNFGTackSegment( pta[0], pta[1], ptc[0], ptc[1] );
		CNFGTackSegment( ptb[0], ptb[1], ptc[0], ptc[1] );
	
	}
}

void TestRTLSDRLibrary(void)
{
    void *handle = dlopen("librtlsdr.so", RTLD_NOW);

    CNFGPenX = 10; 
    CNFGPenY = 100;

    if (!handle) {
        CNFGDrawText("LIBRTLSDR LOAD FAILED", 5);
        CNFGPenX = 10; 
        CNFGPenY = 130;
        CNFGDrawText(dlerror(), 10);
        return;
    }

    CNFGDrawText("LIBRTLSDR LOADED", 5);

    void *func = dlsym(handle, "rtlsdr_open_sys_dev");
    CNFGPenX = 10; 
    CNFGPenY = 130;

    if (!func) {
        CNFGDrawText("OPEN_SYS_DEV NOT FOUND", 5);
        CNFGPenX = 10; 
        CNFGPenY = 160;
        CNFGDrawText(dlerror(), 10);
        return;
    }

    CNFGDrawText("OPEN_SYS_DEV FOUND", 5);
}

int OpenRTLSDRWithLibRTLSDR(int usb_fd)
{
    snprintf(
            debug1,
            sizeof(debug1),
            "somthing"
        );

    if(!rtlsdr_open_sys_dev)
    {
        CNFGPenX = 20;
        CNFGPenY = 800;
        CNFGDrawText("OPEN_SYS_DEV NULL", 5);
        
        return -1;
    }

    rtlDev = NULL;

    int r = rtlsdr_open_sys_dev(
        &rtlDev,
        (intptr_t)usb_fd
    );

    char text[64];

    CNFGPenX = 20;
    CNFGPenY = 800;

    snprintf(
        text,
        sizeof(text),
        "OPEN_SYS_DEV RESULT = %d",
        r
    );

    CNFGDrawText(text, 5);


    CNFGPenX = 20;
    CNFGPenY = 830;

    if(r == 0)
    {
        CNFGDrawText(
            "OPEN RESULT = SUCCESS",
            5
        );
        
    }
    else
    {
        CNFGDrawText(
            "OPEN RESULT = FAILED",
            5
        );
    }


    CNFGPenX = 20;
    CNFGPenY = 860;

    if(rtlDev != NULL)
    {
        CNFGDrawText(
            "RTLDEV = NON-NULL",
            5
        );
    }
    else
    {
        CNFGDrawText(
            "RTLDEV = NULL",
            5
        );
    }
    /*
    snprintf(
            debug1,
            sizeof(debug1),
            "r: %d",
            r
        );
    snprintf(
            debug2,
            sizeof(debug2),
            "rtldev: %d",
            rtlDev
        );*/

    return r;
}



int RequestRTLSDRPermission()
{
    if(!gapp || !gapp->activity || !gapp->activity->vm)
        return -1;

    JavaVM *vm = gapp->activity->vm;
    JNIEnv *env = NULL;

    JavaVMAttachArgs args;
    args.version = JNI_VERSION_1_6;
    args.name = (char *)"RTLSDRPermission";
    args.group = NULL;

    jint result = (*vm)->AttachCurrentThread(
        vm,
        &env,
        &args
    );

    if(result != JNI_OK || !env)
        return -2;

    jclass ClassContext =
        (*env)->FindClass(
            env,
            "android/content/Context"
        );

    if(!ClassContext)
    {
        (*vm)->DetachCurrentThread(vm);
        return -3;
    }

    jfieldID lid_USB_SERVICE =
        (*env)->GetStaticFieldID(
            env,
            ClassContext,
            "USB_SERVICE",
            "Ljava/lang/String;"
        );

    if(!lid_USB_SERVICE)
    {
        (*vm)->DetachCurrentThread(vm);
        return -4;
    }

    jobject USB_SERVICE =
        (*env)->GetStaticObjectField(
            env,
            ClassContext,
            lid_USB_SERVICE
        );

    jmethodID MethodgetSystemService =
        (*env)->GetMethodID(
            env,
            ClassContext,
            "getSystemService",
            "(Ljava/lang/String;)Ljava/lang/Object;"
        );

    if(!MethodgetSystemService)
    {
        (*vm)->DetachCurrentThread(vm);
        return -5;
    }

    jobject manager =
        (*env)->CallObjectMethod(
            env,
            gapp->activity->clazz,
            MethodgetSystemService,
            USB_SERVICE
        );

    if(!manager)
    {
        (*vm)->DetachCurrentThread(vm);
        return -6;
    }

    /*
     * Get the list of USB devices.
     */

    jclass ClassUsbManager =
        (*env)->FindClass(
            env,
            "android/hardware/usb/UsbManager"
        );

    jmethodID MethodgetDeviceList =
        (*env)->GetMethodID(
            env,
            ClassUsbManager,
            "getDeviceList",
            "()Ljava/util/HashMap;"
        );

    jobject deviceList =
        (*env)->CallObjectMethod(
            env,
            manager,
            MethodgetDeviceList
        );

    if(!deviceList)
    {
        (*vm)->DetachCurrentThread(vm);
        return -7;
    }

    /*
     * Get HashMap.values().
     */

    jclass ClassHashMap =
        (*env)->FindClass(
            env,
            "java/util/HashMap"
        );

    jmethodID Methodvalues =
        (*env)->GetMethodID(
            env,
            ClassHashMap,
            "values",
            "()Ljava/util/Collection;"
        );

    jobject collection =
        (*env)->CallObjectMethod(
            env,
            deviceList,
            Methodvalues
        );

    /*
     * Get iterator.
     */

    jclass ClassCollection =
        (*env)->FindClass(
            env,
            "java/util/Collection"
        );

    jmethodID Methoditerator =
        (*env)->GetMethodID(
            env,
            ClassCollection,
            "iterator",
            "()Ljava/util/Iterator;"
        );

    jobject iterator =
        (*env)->CallObjectMethod(
            env,
            collection,
            Methoditerator
        );

    jclass ClassIterator =
        (*env)->FindClass(
            env,
            "java/util/Iterator"
        );

    jmethodID MethodhasNext =
        (*env)->GetMethodID(
            env,
            ClassIterator,
            "hasNext",
            "()Z"
        );

    jmethodID Methodnext =
        (*env)->GetMethodID(
            env,
            ClassIterator,
            "next",
            "()Ljava/lang/Object;"
        );

    /*
     * UsbDevice methods.
     */

    jclass ClassUsbDevice =
        (*env)->FindClass(
            env,
            "android/hardware/usb/UsbDevice"
        );

    jmethodID MethodgetVendorId =
        (*env)->GetMethodID(
            env,
            ClassUsbDevice,
            "getVendorId",
            "()I"
        );

    jmethodID MethodgetProductId =
        (*env)->GetMethodID(
            env,
            ClassUsbDevice,
            "getProductId",
            "()I"
        );

    /*
     * UsbManager.requestPermission()
     */

    jmethodID MethodrequestPermission =
        (*env)->GetMethodID(
            env,
            ClassUsbManager,
            "requestPermission",
            "(Landroid/hardware/usb/UsbDevice;Landroid/app/PendingIntent;)V"
        );

    if(!MethodrequestPermission)
    {
        (*vm)->DetachCurrentThread(vm);
        return -8;
    }

    /*
     * Find RTL-SDR.
     */

    while(
        (*env)->CallBooleanMethod(
            env,
            iterator,
            MethodhasNext
        )
    )
    {
        jobject device =
            (*env)->CallObjectMethod(
                env,
                iterator,
                Methodnext
            );

        jint vid =
            (*env)->CallIntMethod(
                env,
                device,
                MethodgetVendorId
            );

        jint pid =
            (*env)->CallIntMethod(
                env,
                device,
                MethodgetProductId
            );

        if(vid == 0x0BDA && pid == 0x2838)
        {
            /*
             * Create PendingIntent.
             */

            jclass ClassPendingIntent =
                (*env)->FindClass(
                    env,
                    "android/app/PendingIntent"
                );

            jclass ClassIntent =
                (*env)->FindClass(
                    env,
                    "android/content/Intent"
                );

            jmethodID IntentConstructor =
                (*env)->GetMethodID(
                    env,
                    ClassIntent,
                    "<init>",
                    "(Ljava/lang/String;)V"
                );

            if(!IntentConstructor)
            {
                (*vm)->DetachCurrentThread(vm);
                return -9;
            }

            jstring action =
                (*env)->NewStringUTF(
                    env,
                    "com.rawdrawandroid.RTLSDR_PERMISSION"
                );

            jobject intent =
                (*env)->NewObject(
                    env,
                    ClassIntent,
                    IntentConstructor,
                    action
                );

            /*
             * PendingIntent.getBroadcast(...)
             */

            jmethodID MethodgetBroadcast =
                (*env)->GetStaticMethodID(
                    env,
                    ClassPendingIntent,
                    "getBroadcast",
                    "(Landroid/content/Context;ILandroid/content/Intent;I)Landroid/app/PendingIntent;"
                );

            if(!MethodgetBroadcast)
            {
                (*vm)->DetachCurrentThread(vm);
                return -10;
            }

            jobject pendingIntent =
                (*env)->CallStaticObjectMethod(
                    env,
                    ClassPendingIntent,
                    MethodgetBroadcast,
                    gapp->activity->clazz,
                    0,
                    intent,
                    0
                );

            if(!pendingIntent)
            {
                (*vm)->DetachCurrentThread(vm);
                return -11;
            }

            /*
             * Finally ask Android for permission.
             */

            (*env)->CallVoidMethod(
                env,
                manager,
                MethodrequestPermission,
                device,
                pendingIntent
            );

            (*vm)->DetachCurrentThread(vm);

            return 1;
        }
    }

    (*vm)->DetachCurrentThread(vm);

    return -12;
}


int CheckForRTLSDR(char *status)
{
    struct android_app *app = gapp;

    if(!app)
    {
        strcpy(status, "gapp NULL");
        return 0;
    }

    if(!app->activity)
    {
        strcpy(status, "activity NULL");
        return 0;
    }

    if(!app->activity->vm)
    {
        strcpy(status, "VM NULL");
        return 0;
    }

    JavaVM *vm = app->activity->vm;

    JNIEnv *env = NULL;

    JavaVMAttachArgs args;
    args.version = JNI_VERSION_1_6;
    args.name = (char *)"RTLSDRCheck";
    args.group = NULL;

    jint result = (*vm)->AttachCurrentThread(
        vm,
        &env,
        &args
    );

    if(result != JNI_OK)
    {
        sprintf(status, "Attach failed: %d", result);
        return 0;
    }

    if(!env)
    {
        strcpy(status, "env NULL");
        (*vm)->DetachCurrentThread(vm);
        return 0;
    }

    strcpy(status, "JNI attached");

    /*
     * Find Android Context class.
     */
    jclass ContextClass =
        (*env)->FindClass(
            env,
            "android/content/Context"
        );

    if(!ContextClass)
    {
        strcpy(status, "Context class FAILED");
        (*vm)->DetachCurrentThread(vm);
        return 0;
    }

    strcpy(status, "Context class OK");

    /*
     * Get Context.USB_SERVICE
     */
    jfieldID usbServiceField =
        (*env)->GetStaticFieldID(
            env,
            ContextClass,
            "USB_SERVICE",
            "Ljava/lang/String;"
        );

    if(!usbServiceField)
    {
        strcpy(status, "USB_SERVICE field FAILED");
        (*env)->DeleteLocalRef(env, ContextClass);
        (*vm)->DetachCurrentThread(vm);
        return 0;
    }

    strcpy(status, "USB_SERVICE OK");

    /*
     * Get the string "usb".
     */
    jobject usbService =
        (*env)->GetStaticObjectField(
            env,
            ContextClass,
            usbServiceField
        );

    if(!usbService)
    {
        strcpy(status, "USB_SERVICE object FAILED");
        (*env)->DeleteLocalRef(env, ContextClass);
        (*vm)->DetachCurrentThread(vm);
        return 0;
    }

    strcpy(status, "USB_SERVICE object OK");

    /*
     * Get Context.getSystemService()
     */
    jmethodID getSystemService =
        (*env)->GetMethodID(
            env,
            ContextClass,
            "getSystemService",
            "(Ljava/lang/String;)Ljava/lang/Object;"
        );

    if(!getSystemService)
    {
        strcpy(status, "getSystemService FAILED");
        (*env)->DeleteLocalRef(env, usbService);
        (*env)->DeleteLocalRef(env, ContextClass);
        (*vm)->DetachCurrentThread(vm);
        return 0;
    }

    /*
     * NativeActivity itself is a Context.
     */
    jobject activity =
        app->activity->clazz;

    jobject usbManager =
        (*env)->CallObjectMethod(
            env,
            activity,
            getSystemService,
            usbService
        );

    if(!usbManager)
    {
        strcpy(status, "UsbManager NULL");
        (*env)->DeleteLocalRef(env, usbService);
        (*env)->DeleteLocalRef(env, ContextClass);
        (*vm)->DetachCurrentThread(vm);
        return 0;
    }

    strcpy(status, "UsbManager OK");

    /*
     * Find UsbManager.
     */
    jclass UsbManagerClass =
        (*env)->FindClass(
            env,
            "android/hardware/usb/UsbManager"
        );

    if(!UsbManagerClass)
    {
        strcpy(status, "UsbManager class FAILED");

        (*env)->DeleteLocalRef(env, usbManager);
        (*env)->DeleteLocalRef(env, usbService);
        (*env)->DeleteLocalRef(env, ContextClass);
        (*vm)->DetachCurrentThread(vm);

        return 0;
    }

    strcpy(status, "UsbManager class OK");

    /*
     * Get getDeviceList().
     */
    jmethodID getDeviceList =
        (*env)->GetMethodID(
            env,
            UsbManagerClass,
            "getDeviceList",
            "()Ljava/util/HashMap;"
        );

    if(!getDeviceList)
    {
        strcpy(status, "getDeviceList FAILED");

        (*env)->DeleteLocalRef(env, UsbManagerClass);
        (*env)->DeleteLocalRef(env, usbManager);
        (*env)->DeleteLocalRef(env, usbService);
        (*env)->DeleteLocalRef(env, ContextClass);
        (*vm)->DetachCurrentThread(vm);

        return 0;
    }

    /*
     * Ask Android for all USB devices.
     */
    jobject deviceList =
        (*env)->CallObjectMethod(
            env,
            usbManager,
            getDeviceList
        );

    if(!deviceList)
    {
        strcpy(status, "deviceList NULL");

        (*env)->DeleteLocalRef(env, UsbManagerClass);
        (*env)->DeleteLocalRef(env, usbManager);
        (*env)->DeleteLocalRef(env, usbService);
        (*env)->DeleteLocalRef(env, ContextClass);
        (*vm)->DetachCurrentThread(vm);

        return 0;
    }

    strcpy(status, "USB device list OK");

    /*
     * HashMap.values()
     */
    jclass HashMapClass =
        (*env)->FindClass(
            env,
            "java/util/HashMap"
        );

    jmethodID valuesMethod =
        (*env)->GetMethodID(
            env,
            HashMapClass,
            "values",
            "()Ljava/util/Collection;"
        );

    jobject collection =
        (*env)->CallObjectMethod(
            env,
            deviceList,
            valuesMethod
        );

    if(!collection)
    {
        strcpy(status, "USB collection NULL");

        (*env)->DeleteLocalRef(env, HashMapClass);
        (*env)->DeleteLocalRef(env, deviceList);
        (*env)->DeleteLocalRef(env, UsbManagerClass);
        (*env)->DeleteLocalRef(env, usbManager);
        (*env)->DeleteLocalRef(env, usbService);
        (*env)->DeleteLocalRef(env, ContextClass);
        (*vm)->DetachCurrentThread(vm);

        return 0;
    }

    /*
     * Collection.iterator()
     */
    jclass CollectionClass =
        (*env)->FindClass(
            env,
            "java/util/Collection"
        );

    jmethodID iteratorMethod =
        (*env)->GetMethodID(
            env,
            CollectionClass,
            "iterator",
            "()Ljava/util/Iterator;"
        );

    jobject iterator =
        (*env)->CallObjectMethod(
            env,
            collection,
            iteratorMethod
        );

    /*
     * Iterator methods.
     */
    jclass IteratorClass =
        (*env)->FindClass(
            env,
            "java/util/Iterator"
        );

    jmethodID hasNextMethod =
        (*env)->GetMethodID(
            env,
            IteratorClass,
            "hasNext",
            "()Z"
        );

    jmethodID nextMethod =
        (*env)->GetMethodID(
            env,
            IteratorClass,
            "next",
            "()Ljava/lang/Object;"
        );

    /*
     * UsbDevice methods.
     */
    jclass UsbDeviceClass =
        (*env)->FindClass(
            env,
            "android/hardware/usb/UsbDevice"
        );

    jmethodID getVendorId =
        (*env)->GetMethodID(
            env,
            UsbDeviceClass,
            "getVendorId",
            "()I"
        );

    jmethodID getProductId =
        (*env)->GetMethodID(
            env,
            UsbDeviceClass,
            "getProductId",
            "()I"
        );

    jmethodID getDeviceName =
        (*env)->GetMethodID(
            env,
            UsbDeviceClass,
            "getDeviceName",
            "()Ljava/lang/String;"
        );

    /*
     * Look through every USB device.
     */
    while((*env)->CallBooleanMethod(
        env,
        iterator,
        hasNextMethod))
    {
        jobject device =
            (*env)->CallObjectMethod(
                env,
                iterator,
                nextMethod
            );

        jint vid =
            (*env)->CallIntMethod(
                env,
                device,
                getVendorId
            );

        jint pid =
            (*env)->CallIntMethod(
                env,
                device,
                getProductId
            );

        jstring name =
            (jstring)(*env)->CallObjectMethod(
                env,
                device,
                getDeviceName
            );

        const char *nameString =
            (*env)->GetStringUTFChars(
                env,
                name,
                NULL
            );

        if(vid == 0x0BDA && pid == 0x2838)
        {
            sprintf(
                status,
                "RTL-SDR FOUND: %s",
                nameString
            );

            (*env)->ReleaseStringUTFChars(
                env,
                name,
                nameString
            );

            (*env)->DeleteLocalRef(env, name);
            (*env)->DeleteLocalRef(env, device);

            (*env)->DeleteLocalRef(env, UsbDeviceClass);
            (*env)->DeleteLocalRef(env, IteratorClass);
            (*env)->DeleteLocalRef(env, CollectionClass);
            (*env)->DeleteLocalRef(env, collection);
            (*env)->DeleteLocalRef(env, HashMapClass);
            (*env)->DeleteLocalRef(env, deviceList);
            (*env)->DeleteLocalRef(env, UsbManagerClass);
            (*env)->DeleteLocalRef(env, usbManager);
            (*env)->DeleteLocalRef(env, usbService);
            (*env)->DeleteLocalRef(env, ContextClass);

            (*vm)->DetachCurrentThread(vm);

            return 1;
        }

        (*env)->ReleaseStringUTFChars(
            env,
            name,
            nameString
        );

        (*env)->DeleteLocalRef(env, name);
        (*env)->DeleteLocalRef(env, device);
    }

    strcpy(status, "RTL-SDR not found");

    /*
     * Cleanup.
     */
    (*env)->DeleteLocalRef(env, UsbDeviceClass);
    (*env)->DeleteLocalRef(env, IteratorClass);
    (*env)->DeleteLocalRef(env, CollectionClass);
    (*env)->DeleteLocalRef(env, collection);
    (*env)->DeleteLocalRef(env, HashMapClass);
    (*env)->DeleteLocalRef(env, deviceList);
    (*env)->DeleteLocalRef(env, UsbManagerClass);
    (*env)->DeleteLocalRef(env, usbManager);
    (*env)->DeleteLocalRef(env, usbService);
    (*env)->DeleteLocalRef(env, ContextClass);

    (*vm)->DetachCurrentThread(vm);

    return 0;
}

void CheckRTLSDR()
{
    static double lastCheck = 0;

    double now = OGGetAbsoluteTime();

    if(now - lastCheck < 1.0)
        return;

    lastCheck = now;

    /*
     * If librtlsdr is already open, don't initialize it again.
     */
    if(rtlDeviceOpened)
    {
        return;
    }

    /*
     * STEP 1
     * Is the RTL-SDR physically connected?
     */
    int found = CheckForRTLSDR(usbStatus);

    CNFGPenX = 20;
    CNFGPenY = 600;

    if(!found)
    {
        CNFGDrawText(
            "STEP 1: RTL-SDR NOT FOUND",
            5
        );

        return;
    }

    CNFGDrawText(
        "STEP 1: RTL-SDR FOUND",
        5
    );


    /*
     * STEP 2
     * Has Android granted permission?
     */
    int permission = CheckRTLSDRPermission();

    CNFGPenX = 20;
    CNFGPenY = 630;

    if(permission != 1)
    {
        CNFGDrawText(
            "STEP 2: NO PERMISSION",
            5
        );

        int requestResult =
            RequestRTLSDRPermission();

        CNFGPenX = 20;
        CNFGPenY = 660;

        if(requestResult > 0)
        {
            CNFGDrawText(
                "STEP 3: PERMISSION REQUEST SENT",
                5
            );
        }
        else
        {
            CNFGDrawText(
                "STEP 3: PERMISSION REQUEST FAILED",
                5
            );
        }

        return;
    }


    /*
     * STEP 4
     */
    CNFGPenX = 20;
    CNFGPenY = 660;

    CNFGDrawText(
        "STEP 4: USB PERMISSION GRANTED",
        5
    );


    /*
     * STEP 5
     * Get Android USB file descriptor.
     */
    CNFGPenX = 20;
    CNFGPenY = 690;

    int fd = OpenRTLSDR();

    if(fd < 0)
    {
        CNFGDrawText(
            "STEP 5: USB OPEN FAILED",
            5
        );

        return;
    }

    rtlUsbFD = fd;

    CNFGDrawText(
        "STEP 5: USB FD OPENED",
        5
    );


    /*
     * STEP 6
     * Give the Android FD to librtlsdr.
     */
    CNFGPenX = 20;
    CNFGPenY = 720;

    CNFGDrawText(
        "STEP 6: OPENING LIBRTLSDR",
        5
    );
    
    

    int rtlResult =
        OpenRTLSDRWithLibRTLSDR(rtlUsbFD);


    /*
     * Show result.
     */
    CNFGPenX = 20;
    CNFGPenY = 750;

    char resultText[64];

    snprintf(
        resultText,
        sizeof(resultText),
        "OPEN_SYS_DEV RESULT: %d",
        rtlResult
    );

    CNFGDrawText(
        resultText,
        5
    );


    /*
     * STEP 7
     */
    CNFGPenX = 20;
    CNFGPenY = 780;

    if(rtlResult != 0 || rtlDev == NULL)
    {
        CNFGDrawText(
            "STEP 7: RTL DEVICE FAILED",
            5
        );
        snprintf(
            debug3,
            sizeof(debug3),
            "device NOT open"
        );

        return;
    }

    CNFGDrawText(
        "STEP 7: RTL DEVICE OPEN",
        5
    );
    snprintf(
            debug3,
            sizeof(debug3),
            "device open"
        );

    /*
     * Mark the device as successfully opened.
     *
     * CheckRTLSDR() will now stop performing initialization.
     */
    rtlDeviceOpened = 1;

    return;
}

int ConfigureRTLSDR()
{
    int r;

    r = rtlsdr_set_sample_rate(rtlDev, 2048000);
    if(r != 0)
        return r;

    r = rtlsdr_set_center_freq(rtlDev, 100000000);
    if(r != 0)
        return r;

    r = rtlsdr_set_tuner_gain_mode(rtlDev, 0);
    if(r != 0)
        return r;

    r = rtlsdr_reset_buffer(rtlDev);
    if(r != 0)
        return r;

    return 0;
}

void ReadRTLSDRTest()
{

    
    
    unsigned char buffer[1024];
    uint32_t n_read = 0;

    if(!rtlDeviceOpened || rtlDev == NULL)
        return;

    int result = rtlsdr_read_sync(
        rtlDev,
        buffer,
        sizeof(buffer),
        &n_read
    );

    if(result == 0 && n_read >= 1024)
	{
	    for(int i = 0; i < 512; i++)
	    {
		int index = i * 2;

		samplesI[i] =
		    ((float)buffer[index] - 127.5f) / 127.5f;

		samplesQ[i] =
		    ((float)buffer[index + 1] - 127.5f) / 127.5f;
	    }
	}


    CNFGPenX = 20;
    CNFGPenY = 810;

    char text[128];

    snprintf(
        debug5,
        sizeof(debug5),
        "IQ READ: result=%d bytes=%u",
        result,
        n_read
    );

    CNFGDrawText(text, 5);

    /*
     * Show the first few raw IQ bytes.
     */
    CNFGPenX = 20;
    CNFGPenY = 840;

    snprintf(
        debug4,
        sizeof(debug4),
        "IQ: %u %u %u %u %u %u",
        buffer[0],
        buffer[1],
        buffer[2],
        buffer[3],
        buffer[4],
        buffer[5]
    );

    CNFGDrawText(text, 5);
    
    
    
    
        CNFGPenX = 20;
	CNFGPenY = 840;

	snprintf(
	    debug3,
	    sizeof(debug3),
	    "I: %.3f %.3f %.3f    Q: %.3f %.3f %.3f",
	    samplesI[0],
	    samplesI[1],
	    samplesI[2],
	    samplesQ[0],
	    samplesQ[1],
	    samplesQ[2]
	);

	CNFGDrawText(text, 5);
}

int HandleDestroy()
{
	printf( "Destroying\n" );
	exit(10);
}

volatile int suspended;

void HandleSuspend()
{
	suspended = 1;
}

void HandleResume()
{
	suspended = 0;
}

uint32_t randomtexturedata[256*256];

int main()
{
	srand(time(NULL));
	int x, y;
	double ThisTime;
	double LastFPSTime = OGGetAbsoluteTime();
	int linesegs = 0;

	CNFGBGColor = 0x000040ff;
	CNFGSetupFullscreen( "Software Defined Radio", 0 );
	
	
	//CNFGSetup( "Test Bench", 0, 0 );

	float t = 0;

	/*
	for( x = 0; x < HMX; x++ )
	for( y = 0; y < HMY; y++ )
	{
		Heightmap[x+y*HMX] = tdPerlin2D( x, y )*8.;
	}*/


	const char * assettext = "Not Found";
	AAsset * file = AAssetManager_open( gapp->activity->assetManager, "asset.txt", AASSET_MODE_BUFFER );
	if( file )
	{
		size_t fileLength = AAsset_getLength(file);
		char * temp = malloc( fileLength + 1);
		memcpy( temp, AAsset_getBuffer( file ), fileLength );
		temp[fileLength] = 0;
		assettext = temp;
	}
	SetupIMU();

	while(1)
	{
		int i, pos;
		iframeno++;

		CNFGHandleInput();
				
		TestRTLSDRLibrary();
		/*
		if(rtlUsbFD < 0)
		{
		    int fd = OpenRTLSDR();

		    if(fd >= 0)
		    {
			rtlUsbFD = fd;
		    }
		}
		
		
		if(rtlUsbFD >= 0 && rtlDev == NULL)
		{
		    OpenRTLSDRWithLibRTLSDR(rtlUsbFD);
		}
		
		*/
		CheckRTLSDR();
		ConfigureRTLSDR();
		if(rtlDeviceOpened)
		{
		    ReadRTLSDRTest();
		    FFT();
		}
		
		
		AccCheck();

		if( suspended ) { usleep(50000); continue; }


		timeSeconds += 0.03f;
		loFrequency += 0.1f;

		if(loFrequency > 100)
		    loFrequency = 0;
    
		GenerateSignal(timeSeconds);

        	
        	//UpdateFakeFFT(timeSeconds);
        	UpdateWaterfall();


		CNFGClearFrame();
		
		
		

		
		CNFGColor( 0xFFFFFFFF );
		CNFGGetDimensions( &screenx, &screeny );

		// Mesh in background
		//CNFGSetLineWidth( 9 );
		//DrawHeightmap();
		//CNFGPenX = 0; CNFGPenY = 400;
		//CNFGColor( 0xffffffff );
		//CNFGDrawText( assettext, 15 );
		CNFGFlushRender();

		//CNFGPenX = 0; CNFGPenY = 480;
		//char st[50];
		//sprintf( st, "%dx%d %d %d %d %d %d %d\n%d %d\n%5.2f %5.2f %5.2f %d", screenx, screeny, lastbuttonx, lastbuttony, lastmotionx, lastmotiony, lastkey, lastkeydown, lastbid, lastmask, accx, accy, accz, accs );
		//CNFGDrawText( st, 10 );
		//CNFGSetLineWidth( 2 );

		// Square behind text
		//CNFGColor( 0x303030ff );
		//CNFGTackRectangle( 600, 0, 950, 350);

		//CNFGPenX = 10; CNFGPenY = 10;
		
		/*printing usb data*/
		char text[128];

		sprintf(
		    text,
		    "FFT 50: %.2f  FFT 150: %.2f",
		    debugBin50,
		    debugBin150
		);

		CNFGPenX = 10;
		CNFGPenY = 10;
		CNFGDrawText(text, 3);





		CNFGPenX = 200;
		CNFGPenY = 500;
		CNFGDrawText(debug1, 5);

		CNFGPenX = 200;
		CNFGPenY = 530;
		CNFGDrawText(debug2, 5);

		CNFGPenX = 200;
		CNFGPenY = 560;
		CNFGDrawText(debug3, 5);

		CNFGPenX = 200;
		CNFGPenY = 590;
		CNFGDrawText(debug4, 5);

		CNFGPenX = 200;
		CNFGPenY = 620;
		CNFGDrawText(debug5, 5);


		char usbText[256];

		sprintf(
		    usbText,
		    "USB: %s",
		    usbStatus
		);

		CNFGPenX = 10;
		CNFGPenY = 30;
		CNFGDrawText(usbText, 3);
		
		/*end printing usb data*/
		
		//int device_count = rtlsdr_get_device_count();

		//char mytext[128];
		//snprintf(mytext, sizeof(mytext), "RTL-SDR devices: %d", device_count);

		//CNFGDrawText(mytext, 20);
		
		DrawSpectrum();
        	DrawWaterfall();
        	DrawSamples();
        	DrawFrequencyAxis();
        	
        	
		//On Android, CNFGSwapBuffers must be called, and CNFGUpdateScreenWithBitmap does not have an implied framebuffer swap.
		CNFGSwapBuffers();


		/*
		ThisTime = OGGetAbsoluteTime();
		if( ThisTime > LastFPSTime + 1 )
		{
			printf( "FPS: %d\n", frames );
			frames = 0;
			linesegs = 0;
			LastFPSTime+=1;
		}*/

	}

	return(0);
}





void UpdateFakeFFT(float t)
{
    int i;

    for(i=0;i<FFT_BINS;i++)
    {
        fft[i] = 0;

        float peak1 = 150 + 40*sin(t);
        float peak2 = 330 + 25*cos(t*0.7f);

        fft[i] += 80.0f * expf(-(i-peak1)*(i-peak1)/200.0f);
        fft[i] += 60.0f * expf(-(i-peak2)*(i-peak2)/100.0f);

        fft[i] += rand()%5;
    }
}

void UpdateWaterfall()
{
    memmove(
        waterfall[1],
        waterfall[0],
        (WATERFALL_HEIGHT-1)*FFT_BINS
    );

    //for(int i=0;i<FFT_BINS/2;i++)
    //fix for negative frequency mirror
    for(int i=0;i<FFT_BINS;i++)
    {
        float v = fft[i];

        if(v < 0) v = 0;
        if(v > 255) v = 255;

        waterfall[0][i] = (unsigned char)v;
    }
}

void DrawSpectrum()
{
    CNFGColor(0xFFFFFFFF);

    int center = SPECTRUM_Y;

    //for(int i=0;i<FFT_BINS/2-1;i++)
    for(int i=0;i<FFT_BINS-1;i++)
    //removing negative frequency mirror
    {
        int x1 = i * 2;
        int x2 = (i+1) * 2;

        int y1 = center - fft[i];
        int y2 = center - fft[i+1];

        CNFGTackSegment(
            x1, y1,
            x2, y2
        );
    }
}

void DrawWaterfall()
{
    int center = WATERFALL_Y;
    for(int y=0;y<WATERFALL_HEIGHT;y++)
    {
    	//for(int x=0;x<FFT_BINS/2;x++)
    	//fix for negative frequency mirror
        for(int x=0;x<FFT_BINS;x++)
        {
            unsigned char r,g,b;

	    WaterfallColor(
	        waterfall[y][x],
	        &r,
	        &g,
	        &b
	    );

	    CNFGColor(
	        (0xff<<24) |
	        (r<<16) |
	        (g<<8) |
	        b
	    );

            CNFGTackRectangle(
                x*2,
                center+y,
                x*2+2,
                center+1+y
            );
        }
    }
}

void WaterfallColor(
    unsigned char value,
    unsigned char *r,
    unsigned char *g,
    unsigned char *b
)
{
    if(value < 64)
    {
        *r = 0;
        *g = 0;
        *b = value * 4;
    }
    else if(value < 128)
    {
        *r = 0;
        *g = (value-64)*4;
        *b = 255;
    }
    else if(value < 192)
    {
        *r = (value-128)*4;
        *g = 255;
        *b = 255-(value-128)*4;
    }
    else
    {
        *r = 255;
        *g = 255-(value-192)*4;
        *b = 0;
    }
}


void GenerateSignal(float t)
{

    float signalFrequency = 50.0f;
    
    for(int i=0;i<SAMPLE_COUNT;i++)
    {
    
    	float sampleTime = 
    	    (float) i / SAMPLE_RATE;
    	    
        // incoming signal frequency
        float signalAngle =
            2.0f*M_PI*signalFrequency*sampleTime;


        // local oscillator frequency
        float loAngle =
    		-2.0f*M_PI*loFrequency*sampleTime;


        // original IQ signal
        float signalI = cosf(signalAngle);
        float signalQ = sinf(signalAngle);


        // local oscillator
        float loI = cosf(loAngle);
        float loQ = sinf(loAngle);


        // complex multiplication:
        //
        // (signalI+j signalQ)*(loI+j loQ)

        samplesI[i] =
            signalI*loI -
            signalQ*loQ;


        samplesQ[i] =
            signalI*loQ +
            signalQ*loI;
    }
}
/* this is regular generate signal
void GenerateSignal(float t)
{
    for(int i=0;i<SAMPLE_COUNT;i++)
    {
    	float noise =
            ((float)rand()/RAND_MAX - 0.5f);
    
        float amp = 0.5f + 0.5f*sinf(t);

        samples[i] =
	    amp * sinf(2.0f*M_PI*50*i/SAMPLE_COUNT)
	    +
	    0.5f*sinf(2.0f*M_PI*150*i/SAMPLE_COUNT)
	    +
	    noise * noiseLevel;
    }
}*/

/*
void DFT(void)
{
    for(int k=0;k<FFT_BINS;k++)
    {
        float real = 0;
        float imag = 0;

        for(int n=0;n<SAMPLE_COUNT;n++)
        {
            float angle =
                2*M_PI*k*n/SAMPLE_COUNT;

            real += samples[n]*cosf(angle);
            imag -= samples[n]*sinf(angle);
        }

        float magnitude = sqrtf(
	    real*real +
	    imag*imag
	);

	fft[k] = 20.0f * log10f(magnitude + 1);
    }
    
    debugBin50 = fft[50];
    debugBin150 = fft[150];
}
*/
void FFT(void)
{
    int i, j, k;

    // Copy IQ samples into FFT working arrays
    for(i = 0; i < SAMPLE_COUNT; i++)
    {
        fftReal[i] = samplesI[i];
        fftImag[i] = samplesQ[i];
    }


    // Bit reversal
    j = 0;

    for(i = 0; i < SAMPLE_COUNT; i++)
    {
        if(i < j)
        {
            float temp;

            temp = fftReal[i];
            fftReal[i] = fftReal[j];
            fftReal[j] = temp;

            temp = fftImag[i];
            fftImag[i] = fftImag[j];
            fftImag[j] = temp;
        }

        int m = SAMPLE_COUNT >> 1;

        while(m >= 1 && j >= m)
        {
            j -= m;
            m >>= 1;
        }

        j += m;
    }


    // FFT stages
    for(int size = 2; size <= SAMPLE_COUNT; size <<= 1)
    {
        float angle = -2.0f * M_PI / size;

        float wStepReal = cosf(angle);
        float wStepImag = sinf(angle);


        for(int start = 0; start < SAMPLE_COUNT; start += size)
        {
            float currentReal = 1.0f;
            float currentImag = 0.0f;


            for(k = 0; k < size/2; k++)
            {
                int even = start + k;
                int odd  = start + k + size/2;


                // Multiply odd sample by twiddle factor
                float oddReal =
                    fftReal[odd] * currentReal -
                    fftImag[odd] * currentImag;

                float oddImag =
                    fftReal[odd] * currentImag +
                    fftImag[odd] * currentReal;


                float evenReal = fftReal[even];
                float evenImag = fftImag[even];


                // Butterfly
                fftReal[even] =
                    evenReal + oddReal;

                fftImag[even] =
                    evenImag + oddImag;


                fftReal[odd] =
                    evenReal - oddReal;

                fftImag[odd] =
                    evenImag - oddImag;


                // Rotate twiddle factor
                float temp = currentReal;

                currentReal =
                    temp * wStepReal -
                    currentImag * wStepImag;

                currentImag =
                    temp * wStepImag +
                    currentImag * wStepReal;
            }
        }
    }


    // Calculate magnitude for display
    for(i = 0; i < FFT_BINS; i++)
    {
        fft[i] = sqrtf(
            fftReal[i] * fftReal[i] +
            fftImag[i] * fftImag[i]
        );
    }
}
/*
void DrawSamples()
{
    CNFGColor(0xffffffff);

    int center = SAMPLES_Y;
    
    for(int i=0;i<SAMPLE_COUNT-1;i++)
    {
        CNFGTackSegment(
            i*2,
            center + samples[i]*100,
            (i+1)*2,
            center + samples[i+1]*100
        );
    }
}*/
void DrawSamples()
{
    int center = SAMPLES_Y;

    // I channel
    CNFGColor(0xff0000ff);

    for(int i=0;i<SAMPLE_COUNT-1;i++)
    {
        CNFGTackSegment(
            i*2,
            center + samplesI[i]*100,
            (i+1)*2,
            center + samplesI[i+1]*100
        );
    }


    // Q channel
    CNFGColor(0x00ff00ff);

    for(int i=0;i<SAMPLE_COUNT-1;i++)
    {
        CNFGTackSegment(
            i*2,
            center + samplesQ[i]*100,
            (i+1)*2,
            center + samplesQ[i+1]*100
        );
    }
}

void DrawFrequencyAxis()
{
    CNFGColor(0xffffffff);

    char text[32];

    for(int i=0;i<=FFT_BINS/2;i+=64)
    {
        float freq = i * SAMPLE_RATE / FFT_BINS;

        sprintf(
            text,
            "%.0f Hz",
            freq
        );

        CNFGPenX = i * 2;
        CNFGPenY = 370;

        CNFGDrawText(text, 2);
        
        int x = i*2;

        CNFGTackSegment(
            x,
            360,
            x,
            370
        );
    }
}

int OpenRTLSDR(void)
{
    if(rtlUsbFD >= 0)
	{
	    CNFGPenX = 10;
	    CNFGPenY = 600;
	    CNFGDrawText("FD ALREADY OPEN", 5);
	    return rtlUsbFD;
	}
    

    JNIEnv *env = NULL;

    if((*gapp->activity->vm)->AttachCurrentThread(
        gapp->activity->vm, &env, NULL) != JNI_OK)
    {
        CNFGPenX = 30;
        CNFGPenY = 600;
        CNFGDrawText("JNI ATTACH FAILED", 5);
        return -1;
    }

    jobject activity = gapp->activity->clazz;

    jclass contextClass = (*env)->GetObjectClass(env, activity);

    jfieldID usbServiceField =
        (*env)->GetStaticFieldID(
            env,
            contextClass,
            "USB_SERVICE",
            "Ljava/lang/String;");

    jstring usbService =
        (*env)->GetStaticObjectField(env, contextClass, usbServiceField);

    jmethodID getSystemService =
        (*env)->GetMethodID(
            env,
            contextClass,
            "getSystemService",
            "(Ljava/lang/String;)Ljava/lang/Object;");

    jobject usbManager =
        (*env)->CallObjectMethod(
            env,
            activity,
            getSystemService,
            usbService);

    if(!usbManager)
    {
        CNFGPenX = 30;
        CNFGPenY = 600;
        CNFGDrawText("USB MANAGER FAILED", 5);
        return -2;
    }

    CNFGPenX = 30;
    CNFGPenY = 600;
    CNFGDrawText("USB MANAGER OK", 5);

    jclass usbManagerClass = (*env)->GetObjectClass(env, usbManager);

    jmethodID getDeviceList =
        (*env)->GetMethodID(
            env,
            usbManagerClass,
            "getDeviceList",
            "()Ljava/util/HashMap;");

    jobject deviceList =
        (*env)->CallObjectMethod(
            env,
            usbManager,
            getDeviceList);

    if(!deviceList)
    {
        CNFGPenX = 30;
        CNFGPenY = 630;
        CNFGDrawText("USB DEVICE LIST FAILED", 5);
        return -3;
    }

    CNFGPenX = 30;
    CNFGPenY = 630;
    CNFGDrawText("USB DEVICE LIST OK", 5);

    jclass mapClass = (*env)->GetObjectClass(env, deviceList);

    jmethodID valuesMethod =
        (*env)->GetMethodID(
            env,
            mapClass,
            "values",
            "()Ljava/util/Collection;");

    jobject values =
        (*env)->CallObjectMethod(
            env,
            deviceList,
            valuesMethod);

    jclass collectionClass = (*env)->GetObjectClass(env, values);

    jmethodID iteratorMethod =
        (*env)->GetMethodID(
            env,
            collectionClass,
            "iterator",
            "()Ljava/util/Iterator;");

    jobject iterator =
        (*env)->CallObjectMethod(
            env,
            values,
            iteratorMethod);

    jclass iteratorClass = (*env)->GetObjectClass(env, iterator);

    jmethodID hasNextMethod =
        (*env)->GetMethodID(
            env,
            iteratorClass,
            "hasNext",
            "()Z");

    jmethodID nextMethod =
        (*env)->GetMethodID(
            env,
            iteratorClass,
            "next",
            "()Ljava/lang/Object;");

    jclass usbDeviceClass = (*env)->FindClass(
        env,
        "android/hardware/usb/UsbDevice");

    jmethodID getVendorId =
        (*env)->GetMethodID(
            env,
            usbDeviceClass,
            "getVendorId",
            "()I");

    jmethodID getProductId =
        (*env)->GetMethodID(
            env,
            usbDeviceClass,
            "getProductId",
            "()I");

    jclass usbManagerCls =
        (*env)->GetObjectClass(env, usbManager);

	

    jmethodID openDevice =
        (*env)->GetMethodID(
            env,
            usbManagerCls,
            "openDevice",
            "(Landroid/hardware/usb/UsbDevice;)"
            "Landroid/hardware/usb/UsbDeviceConnection;");

    while((*env)->CallBooleanMethod(env, iterator, hasNextMethod))
    {
        jobject device =
            (*env)->CallObjectMethod(env, iterator, nextMethod);

        jint vid =
            (*env)->CallIntMethod(env, device, getVendorId);

        jint pid =
            (*env)->CallIntMethod(env, device, getProductId);
            
        

        if(vid == 0x0BDA && pid == 0x2838)
        {
            CNFGPenX = 30;
            CNFGPenY = 660;
            CNFGDrawText("RTL-SDR FOUND", 5);



            jmethodID hasPermission =
    (*env)->GetMethodID(
        env,
        usbManagerCls,
        "hasPermission",
        "(Landroid/hardware/usb/UsbDevice;)Z");

		jboolean permission =
		    (*env)->CallBooleanMethod(
			env,
			usbManager,
			hasPermission,
			device);

		CNFGPenX = 30;
		CNFGPenY =740;

		if(permission)
		{
		    CNFGDrawText("USB PERMISSION YES", 5);
		}
		else
		{
		    CNFGDrawText("USB PERMISSION NO", 5);
		}
            /*
             * This is the important point:
             * openDevice() should only succeed if Android
             * has granted this application USB permission.
             */
            CNFGPenX = 10;
	    CNFGPenY = 600;
	    CNFGDrawText("CALLING OPEN DEVICE", 5);
	
            jobject connection =
                (*env)->CallObjectMethod(
                    env,
                    usbManager,
                    openDevice,
                    device);

            if(!connection)
            {
                CNFGPenX = 30;
                CNFGPenY = 690;
                CNFGDrawText("USB OPEN FAILED", 5);
                return -6;
            }

            CNFGPenX = 30;
            CNFGPenY = 690;
            CNFGDrawText("USB OPENED", 5);

            rtlUsbConnection =
                (*env)->NewGlobalRef(env, connection);

            if(!rtlUsbConnection)
            {
                CNFGPenX = 30;
                CNFGPenY = 720;
                CNFGDrawText("GLOBAL REF FAILED", 5);
                return -7;
            }

            jclass connectionClass =
                (*env)->GetObjectClass(env, connection);

            jmethodID getFileDescriptor =
                (*env)->GetMethodID(
                    env,
                    connectionClass,
                    "getFileDescriptor",
                    "()I");

            if(!getFileDescriptor)
            {
                CNFGPenX = 30;
                CNFGPenY = 720;
                CNFGDrawText("GET FD METHOD FAILED", 5);
                return -8;
            }

            jint fd =
                (*env)->CallIntMethod(
                    env,
                    connection,
                    getFileDescriptor);

            char fdText[64];

            CNFGPenX = 30;
            CNFGPenY = 720;

            if(fd < 0)
            {
                CNFGDrawText("INVALID USB FD", 5);
                return -9;
            }

            snprintf(
                fdText,
                sizeof(fdText),
                "USB FD = %d",
                fd);

            CNFGDrawText(fdText, 5);

	    rtlUsbFD = fd;

            return fd;
        }
    }

    CNFGPenX = 30;
    CNFGPenY = 660;
    CNFGDrawText("RTL-SDR NOT FOUND", 5);

    return -10;
}



int OpenRTLSDRdep(void)
{
    strcpy(usbStatus, "OPENRTLSDR ENTERED");

    if(!gapp || !gapp->activity || !gapp->activity->vm)
        return -1;

    JavaVM *vm = gapp->activity->vm;
    JNIEnv *env = NULL;

    JavaVMAttachArgs args;
    args.version = JNI_VERSION_1_6;
    args.name = (char *)"RTLSDROpen";
    args.group = NULL;

    jint result = (*vm)->AttachCurrentThread(
        vm,
        &env,
        &args
    );

    if(result != JNI_OK || !env)
        return -2;

    /*
     * Context
     */
    jclass ContextClass =
        (*env)->FindClass(
            env,
            "android/content/Context"
        );

    if(!ContextClass)
    {
        (*vm)->DetachCurrentThread(vm);
        return -3;
    }

    /*
     * USB_SERVICE
     */
    jfieldID usbServiceField =
        (*env)->GetStaticFieldID(
            env,
            ContextClass,
            "USB_SERVICE",
            "Ljava/lang/String;"
        );

    if(!usbServiceField)
    {
        (*vm)->DetachCurrentThread(vm);
        return -4;
    }

    jobject usbService =
        (*env)->GetStaticObjectField(
            env,
            ContextClass,
            usbServiceField
        );

    /*
     * getSystemService()
     */
    jmethodID getSystemService =
        (*env)->GetMethodID(
            env,
            ContextClass,
            "getSystemService",
            "(Ljava/lang/String;)Ljava/lang/Object;"
        );

    jobject usbManager =
        (*env)->CallObjectMethod(
            env,
            gapp->activity->clazz,
            getSystemService,
            usbService
        );

    if(!usbManager)
    {
        (*vm)->DetachCurrentThread(vm);
        return -5;
    }

    /*
     * UsbManager
     */
    jclass UsbManagerClass =
        (*env)->FindClass(
            env,
            "android/hardware/usb/UsbManager"
        );

    /*
     * getDeviceList()
     */
    jmethodID getDeviceList =
        (*env)->GetMethodID(
            env,
            UsbManagerClass,
            "getDeviceList",
            "()Ljava/util/HashMap;"
        );

    jobject deviceList =
        (*env)->CallObjectMethod(
            env,
            usbManager,
            getDeviceList
        );

    /*
     * HashMap.values()
     */
    jclass HashMapClass =
        (*env)->FindClass(
            env,
            "java/util/HashMap"
        );

    jmethodID valuesMethod =
        (*env)->GetMethodID(
            env,
            HashMapClass,
            "values",
            "()Ljava/util/Collection;"
        );

    jobject collection =
        (*env)->CallObjectMethod(
            env,
            deviceList,
            valuesMethod
        );

    /*
     * Iterator
     */
    jclass CollectionClass =
        (*env)->FindClass(
            env,
            "java/util/Collection"
        );

    jmethodID iteratorMethod =
        (*env)->GetMethodID(
            env,
            CollectionClass,
            "iterator",
            "()Ljava/util/Iterator;"
        );

    jobject iterator =
        (*env)->CallObjectMethod(
            env,
            collection,
            iteratorMethod
        );

    jclass IteratorClass =
        (*env)->FindClass(
            env,
            "java/util/Iterator"
        );

    jmethodID hasNextMethod =
        (*env)->GetMethodID(
            env,
            IteratorClass,
            "hasNext",
            "()Z"
        );

    jmethodID nextMethod =
        (*env)->GetMethodID(
            env,
            IteratorClass,
            "next",
            "()Ljava/lang/Object;"
        );

    /*
     * UsbDevice methods
     */
    jclass UsbDeviceClass =
        (*env)->FindClass(
            env,
            "android/hardware/usb/UsbDevice"
        );

    jmethodID getVendorId =
        (*env)->GetMethodID(
            env,
            UsbDeviceClass,
            "getVendorId",
            "()I"
        );

    jmethodID getProductId =
        (*env)->GetMethodID(
            env,
            UsbDeviceClass,
            "getProductId",
            "()I"
        );

    /*
     * UsbManager.openDevice()
     */
    jmethodID openDevice =
        (*env)->GetMethodID(
            env,
            UsbManagerClass,
            "openDevice",
            "(Landroid/hardware/usb/UsbDevice;)Landroid/hardware/usb/UsbDeviceConnection;"
        );

    /*
     * UsbDeviceConnection.getFileDescriptor()
     */
    jclass UsbDeviceConnectionClass =
        (*env)->FindClass(
            env,
            "android/hardware/usb/UsbDeviceConnection"
        );

    jmethodID getFileDescriptor =
        (*env)->GetMethodID(
            env,
            UsbDeviceConnectionClass,
            "getFileDescriptor",
            "()I"
        );

    jboolean hasDevice =
	    (*env)->CallBooleanMethod(
		env,
		iterator,
		hasNextMethod
	    );

	CNFGPenX = 30;
	CNFGPenY = 630;

	if(hasDevice)
	    CNFGDrawText("USB DEVICE LIST NOT EMPTY", 5);
	else
	    CNFGDrawText("USB DEVICE LIST EMPTY", 5);

    /*
     * Find RTL-SDR.
     */
    while(
        (*env)->CallBooleanMethod(
            env,
            iterator,
            hasNextMethod
        )
    )
    {
        jobject device =
            (*env)->CallObjectMethod(
                env,
                iterator,
                nextMethod
            );

        jint vid =
            (*env)->CallIntMethod(
                env,
                device,
                getVendorId
            );

        jint pid =
            (*env)->CallIntMethod(
                env,
                device,
                getProductId
            );

        if(vid == 0x0BDA && pid == 0x2838)
        {
            /*
             * Open the USB device.
             */
            jobject connection =
		    (*env)->CallObjectMethod(
			env,
			usbManager,
			openDevice,
			device
		    );

		if(!connection)
		{
		    (*vm)->DetachCurrentThread(vm);
		    return -6;
		}

		/*
		 * Keep the UsbDeviceConnection alive.
		 */
		rtlUsbConnection =
		    (*env)->NewGlobalRef(
			env,
			connection
		    );

            if(!connection)
            {
                (*vm)->DetachCurrentThread(vm);
                return -6;
            }

            /*
             * Get Linux USB file descriptor.
             */
            jint fd =
                (*env)->CallIntMethod(
                    env,
                    connection,
                    getFileDescriptor
                );

            (*vm)->DetachCurrentThread(vm);

            if(fd < 0)
                return -7;

            return fd;
        }
    }

    (*vm)->DetachCurrentThread(vm);

    return -8;
}


int CheckRTLSDRPermission(void)
{
    if(!gapp)
        return -1;

    if(!gapp->activity)
        return -2;

    JavaVM *vm = gapp->activity->vm;

    if(!vm)
        return -3;

    JNIEnv *env = NULL;

    JavaVMAttachArgs args;
    args.version = JNI_VERSION_1_6;
    args.name = (char *)"RTLSDRPermissionCheck";
    args.group = NULL;

    jint result = (*vm)->AttachCurrentThread(
        vm,
        &env,
        &args
    );

    if(result != JNI_OK || !env)
        return -4;


    /*
     * Get android.content.Context
     */
    jclass ContextClass =
        (*env)->FindClass(
            env,
            "android/content/Context"
        );

    if(!ContextClass)
    {
        (*vm)->DetachCurrentThread(vm);
        return -5;
    }


    /*
     * Get Context.USB_SERVICE
     */
    jfieldID usbServiceField =
        (*env)->GetStaticFieldID(
            env,
            ContextClass,
            "USB_SERVICE",
            "Ljava/lang/String;"
        );

    if(!usbServiceField)
    {
        (*vm)->DetachCurrentThread(vm);
        return -6;
    }


    jobject usbService =
        (*env)->GetStaticObjectField(
            env,
            ContextClass,
            usbServiceField
        );

    if(!usbService)
    {
        (*vm)->DetachCurrentThread(vm);
        return -7;
    }


    /*
     * Get Context.getSystemService()
     */
    jmethodID getSystemService =
        (*env)->GetMethodID(
            env,
            ContextClass,
            "getSystemService",
            "(Ljava/lang/String;)Ljava/lang/Object;"
        );

    if(!getSystemService)
    {
        (*vm)->DetachCurrentThread(vm);
        return -8;
    }


    /*
     * Get UsbManager.
     */
    jobject usbManager =
        (*env)->CallObjectMethod(
            env,
            gapp->activity->clazz,
            getSystemService,
            usbService
        );

    if(!usbManager)
    {
        (*vm)->DetachCurrentThread(vm);
        return -9;
    }


    /*
     * Get UsbManager class.
     */
    jclass UsbManagerClass =
        (*env)->FindClass(
            env,
            "android/hardware/usb/UsbManager"
        );

    if(!UsbManagerClass)
    {
        (*vm)->DetachCurrentThread(vm);
        return -10;
    }


    /*
     * Get UsbManager.getDeviceList()
     */
    jmethodID getDeviceList =
        (*env)->GetMethodID(
            env,
            UsbManagerClass,
            "getDeviceList",
            "()Ljava/util/HashMap;"
        );

    if(!getDeviceList)
    {
        (*vm)->DetachCurrentThread(vm);
        return -11;
    }


    jobject deviceList =
        (*env)->CallObjectMethod(
            env,
            usbManager,
            getDeviceList
        );

    if(!deviceList)
    {
        (*vm)->DetachCurrentThread(vm);
        return -12;
    }


    /*
     * HashMap.values()
     */
    jclass HashMapClass =
        (*env)->FindClass(
            env,
            "java/util/HashMap"
        );

    jmethodID valuesMethod =
        (*env)->GetMethodID(
            env,
            HashMapClass,
            "values",
            "()Ljava/util/Collection;"
        );

    jobject collection =
        (*env)->CallObjectMethod(
            env,
            deviceList,
            valuesMethod
        );

    if(!collection)
    {
        (*vm)->DetachCurrentThread(vm);
        return -13;
    }


    /*
     * Collection.iterator()
     */
    jclass CollectionClass =
        (*env)->FindClass(
            env,
            "java/util/Collection"
        );

    jmethodID iteratorMethod =
        (*env)->GetMethodID(
            env,
            CollectionClass,
            "iterator",
            "()Ljava/util/Iterator;"
        );

    jobject iterator =
        (*env)->CallObjectMethod(
            env,
            collection,
            iteratorMethod
        );

    if(!iterator)
    {
        (*vm)->DetachCurrentThread(vm);
        return -14;
    }


    /*
     * Iterator methods.
     */
    jclass IteratorClass =
        (*env)->FindClass(
            env,
            "java/util/Iterator"
        );

    jmethodID hasNextMethod =
        (*env)->GetMethodID(
            env,
            IteratorClass,
            "hasNext",
            "()Z"
        );

    jmethodID nextMethod =
        (*env)->GetMethodID(
            env,
            IteratorClass,
            "next",
            "()Ljava/lang/Object;"
        );


    /*
     * UsbDevice methods.
     */
    jclass UsbDeviceClass =
        (*env)->FindClass(
            env,
            "android/hardware/usb/UsbDevice"
        );

    jmethodID getVendorId =
        (*env)->GetMethodID(
            env,
            UsbDeviceClass,
            "getVendorId",
            "()I"
        );

    jmethodID getProductId =
        (*env)->GetMethodID(
            env,
            UsbDeviceClass,
            "getProductId",
            "()I"
        );


    /*
     * UsbManager.hasPermission()
     */
    jmethodID hasPermission =
        (*env)->GetMethodID(
            env,
            UsbManagerClass,
            "hasPermission",
            "(Landroid/hardware/usb/UsbDevice;)Z"
        );

    if(!hasPermission)
    {
        (*vm)->DetachCurrentThread(vm);
        return -15;
    }

	
    /*
     * Search for RTL-SDR.
     */
    while(
        (*env)->CallBooleanMethod(
            env,
            iterator,
            hasNextMethod
        )
    )
    {
        jobject device =
            (*env)->CallObjectMethod(
                env,
                iterator,
                nextMethod
            );

        jint vid =
            (*env)->CallIntMethod(
                env,
                device,
                getVendorId
            );

        jint pid =
            (*env)->CallIntMethod(
                env,
                device,
                getProductId
            );

	CNFGPenX = 10;
	CNFGPenY = 530;

	char deviceText[128];

	snprintf(
	    deviceText,
	    sizeof(deviceText),
	    "DEVICE VID=%04X PID=%04X",
	    vid,
	    pid
	);

	CNFGDrawText(deviceText, 5);
        /*
         * RTL-SDR Blog dongle:
         *
         * VID = 0x0BDA
         * PID = 0x2838
         */
        if(vid == 0x0BDA && pid == 0x2838)
        {
            jboolean permission =
                (*env)->CallBooleanMethod(
                    env,
                    usbManager,
                    hasPermission,
                    device
                );

            (*vm)->DetachCurrentThread(vm);

            if(permission)
                return 1;

            return 0;
        }
    }


    /*
     * RTL-SDR wasn't found.
     */
    (*vm)->DetachCurrentThread(vm);

    return -16;
}


int InitRTLSDRLibUSB(int fd)
{
    int r;

    CNFGPenX = 10;
    CNFGPenY = 800;
    CNFGDrawText("LIBUSB STEP 1", 5);

    r = libusb_set_option(
        NULL,
        LIBUSB_OPTION_NO_DEVICE_DISCOVERY,
        NULL);

    if(r != 0)
    {
        char text[64];
        snprintf(text, sizeof(text),
                 "SET OPTION FAILED %d", r);
        CNFGDrawText(text, 5);
        return -3000 + r;
    }

    CNFGPenY = 830;
    CNFGDrawText("LIBUSB STEP 2", 5);

    r = libusb_init(&rtlUSBContext);

    if(r != 0)
    {
        char text[64];
        snprintf(text, sizeof(text),
                 "LIBUSB INIT FAILED %d", r);
        CNFGDrawText(text, 5);
        return -3000 + r;
    }

    CNFGPenY = 860;
    CNFGDrawText("LIBUSB INIT OK", 5);

    char fdText[64];
    snprintf(fdText, sizeof(fdText),
             "WRAP FD %d", fd);
    CNFGPenY = 890;
    CNFGDrawText(fdText, 5);

    r = libusb_wrap_sys_device(
        rtlUSBContext,
        (intptr_t)fd,
        &rtlUSBHandle);

    if(r != 0)
    {
        char text[64];
        snprintf(text, sizeof(text),
                 "WRAP FAILED %d", r);
        CNFGPenY = 920;
        CNFGDrawText(text, 5);
        return -3000 + r;
    }

    CNFGPenY = 950;
    CNFGDrawText("LIBUSB WRAP OK", 5);

    return 0;
}

int InitRTLSDRLibUSBdep(int fd)
{
    int r;

    // Don't let libusb perform its own device discovery.
    r = libusb_set_option(
        NULL,
        LIBUSB_OPTION_NO_DEVICE_DISCOVERY,
        NULL
    );

    if(r != LIBUSB_SUCCESS)
        return -1000 + r;

    // Initialize libusb.
    r = libusb_init(&rtlUSBContext);

    if(r != LIBUSB_SUCCESS)
    {
        rtlUSBContext = NULL;
        return -2000 + r;
    }

    // Wrap Android's USB file descriptor.
    r = libusb_wrap_sys_device(
        rtlUSBContext,
        (intptr_t)fd,
        &rtlUSBHandle
    );

    if(r != LIBUSB_SUCCESS)
    {
        libusb_exit(rtlUSBContext);

        rtlUSBContext = NULL;
        rtlUSBHandle = NULL;

        return -3000 + r;
    }

    if(!rtlUSBHandle)
    {
        libusb_exit(rtlUSBContext);
        rtlUSBContext = NULL;

        return -4000;
    }

    // Claim interface 0.
    /*
    r = libusb_claim_interface(
        rtlUSBHandle,
        0
    );

    if(r != LIBUSB_SUCCESS)
    {
        libusb_close(rtlUSBHandle);
        libusb_exit(rtlUSBContext);

        rtlUSBHandle = NULL;
        rtlUSBContext = NULL;

        return -5000 + r;
    }*/

    return 0;
}

#define CTRL_IN \
    (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN)

#define CTRL_TIMEOUT 300

#define USB_SYS 1
#define USB_DEMOD 0
#define USB_TUNER 1

#define RTL2832U_REG_DEMOD_CTL      0x3000
#define RTL2832U_REG_USB_SYS        0x0000
#define RTL2832U_REG_DEMOD          0x0000

#define RTL2832U_BLOCK_DEMOD  0
#define RTL2832U_BLOCK_USB    1
#define RTL2832U_BLOCK_SYS    2

#define RTL2832U_REG_SYSCTL   0x0001

#define RTL2832U_REG_DEMOD_CTL_1 0x3000
#define RTL2832U_REG_DEMOD_CTL_2 0x3001



#define RTL2832U_USB_EPA_CTL   0x2148
#define RTL2832U_USB_EPA_MAXPKT 0x2140

int RTLReadRegister(
    uint8_t block,
    uint16_t addr,
    uint8_t *data,
    uint8_t len
)
{
    if(!rtlUSBHandle)
        return -1;

    uint16_t index =
        ((uint16_t)block << 8);

    int r = libusb_control_transfer(
        rtlUSBHandle,

        CTRL_IN,

        0,

        addr,

        index,

        data,

        len,

        CTRL_TIMEOUT
    );

    return r;
}

int TestRTLRegister(void)
{
    uint8_t data = 0;

    int r = RTLReadRegister(
        USB_SYS,
        0x0000,
        &data,
        1
    );

    rtlRegisterResult = r;

    if(r > 0)
        rtlRegisterValue = data;
    else
        rtlRegisterValue = -1;

    return r;
}

int RTLWriteRegister(
    uint8_t block,
    uint16_t addr,
    uint16_t value
)
{
    if(!rtlUSBHandle)
        return -1;

    uint8_t data[2];

    data[0] = value & 0xff;
    data[1] = value >> 8;

    uint16_t index =
        ((uint16_t)block << 8) | 0x10;

    int r = libusb_control_transfer(
        rtlUSBHandle,

        LIBUSB_REQUEST_TYPE_VENDOR |
        LIBUSB_RECIPIENT_DEVICE |
        LIBUSB_ENDPOINT_OUT,

        0,

        addr,

        index,

        data,

        1,

        CTRL_TIMEOUT
    );

    return r;
}

int TestRTLWrite(void)
{
    int r;

    r = RTLWriteRegister(
        USB_SYS,
        0x0000,
        0x0000
    );

    return r;
}

int ReadRTLSDR(uint8_t *buffer, int length)
{
    if(!rtlUSBHandle)
        return -1;

    int transferred = 0;

    int r = libusb_bulk_transfer(
        rtlUSBHandle,
        0x81,              // RTL-SDR bulk IN endpoint
        buffer,
        length,
        &transferred,
        1000               // timeout in ms
    );

    if(r != LIBUSB_SUCCESS)
        return -1000 + r;

    return transferred;
}

int RTLWriteRegister8(
    uint8_t block,
    uint16_t addr,
    uint8_t value
)
{
    if(!rtlUSBHandle)
        return -1;

    uint16_t index =
        ((uint16_t)block << 8) | 0x10;

    int r = libusb_control_transfer(
        rtlUSBHandle,

        LIBUSB_REQUEST_TYPE_VENDOR |
        LIBUSB_RECIPIENT_DEVICE |
        LIBUSB_ENDPOINT_OUT,

        0,

        addr,

        index,

        &value,

        1,

        CTRL_TIMEOUT
    );

    return r;
}

int InitRTL2832U(void)
{
    int r;

    rtl2832Initialized = 0;

    r = RTLWriteRegister8(
        RTL2832U_BLOCK_SYS,
        0x0001,
        0x14
    );

    snprintf(
        debug1,
        sizeof(debug1),
        "INIT WRITE 1 = %d",
        r
    );

    if(r < 0)
        return r;

    r = RTLWriteRegister8(
        RTL2832U_BLOCK_DEMOD,
        0x0001,
        0x14
    );

    snprintf(
        debug2,
        sizeof(debug2),
        "INIT WRITE 2 = %d",
        r
    );

    if(r < 0)
        return r;

    r = RTLWriteRegister8(
        RTL2832U_BLOCK_DEMOD,
        0x0001,
        0x10
    );

    snprintf(
        debug3,
        sizeof(debug3),
        "INIT WRITE 3 = %d",
        r
    );

    if(r < 0)
        return r;

    r = RTLWriteRegister8(
        RTL2832U_BLOCK_SYS,
        0x0001,
        0x10
    );

    snprintf(
        debug4,
        sizeof(debug4),
        "INIT WRITE 4 = %d",
        r
    );

    if(r < 0)
        return r;

    rtl2832Initialized = 1;

    snprintf(
        debug5,
        sizeof(debug5),
        "RTL2832 INIT OK"
    );

    return 0;
}
int InitRTL2832Udep(void)
{
    int r;

    if(!rtlUSBHandle)
        return -1;

    /*
     * ----------------------------------------------------
     * Step 1: Reset the RTL2832U USB/demodulator system.
     * ----------------------------------------------------
     */

    r = RTLWriteRegister8(
        RTL2832U_BLOCK_SYS,
        0x0001,
        0x14
    );

    if(r < 0)
        return -100 + r;


    /*
     * ----------------------------------------------------
     * Step 2: Reset demodulator.
     * ----------------------------------------------------
     */

    r = RTLWriteRegister8(
        RTL2832U_BLOCK_DEMOD,
        0x0001,
        0x14
    );

    if(r < 0)
        return -200 + r;


    /*
     * ----------------------------------------------------
     * Step 3: Release demodulator reset.
     * ----------------------------------------------------
     */

    r = RTLWriteRegister8(
        RTL2832U_BLOCK_DEMOD,
        0x0001,
        0x10
    );

    if(r < 0)
        return -300 + r;


    /*
     * ----------------------------------------------------
     * Step 4: Enable USB streaming.
     * ----------------------------------------------------
     */

    r = RTLWriteRegister8(
        RTL2832U_BLOCK_SYS,
        0x0001,
        0x10
    );

    if(r < 0)
        return -400 + r;


    return 0;
}



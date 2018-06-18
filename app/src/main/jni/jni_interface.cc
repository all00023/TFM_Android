/*
 * Copyright 2014 Google Inc. All Rights Reserved.
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

#define GLM_FORCE_RADIANS

#include <jni.h>

#include "rgb-depth-sync/rgb_depth_sync_application.h"

static rgb_depth_sync::SynchronizationApplication app;

#ifdef __cplusplus
extern "C" {
#endif
JNIEXPORT void JNICALL
Java_com_projecttango_examples_cpp_rgbdsegmentation_TangoJNINative_onCreate(
        JNIEnv *env, jobject, jobject activity) {
    app.OnCreate(env, activity);
}

JNIEXPORT void JNICALL
Java_com_projecttango_examples_cpp_rgbdsegmentation_TangoJNINative_onTangoServiceConnected(
        JNIEnv *env, jobject, jobject iBinder) {
    app.OnTangoServiceConnected(env, iBinder);
}

JNIEXPORT void JNICALL
Java_com_projecttango_examples_cpp_rgbdsegmentation_TangoJNINative_onPause(
        JNIEnv *, jobject) {
    app.OnPause();
}

JNIEXPORT void JNICALL
Java_com_projecttango_examples_cpp_rgbdsegmentation_TangoJNINative_onGlSurfaceCreated(
        JNIEnv *, jobject) {
    app.OnSurfaceCreated();
}

JNIEXPORT void JNICALL
Java_com_projecttango_examples_cpp_rgbdsegmentation_TangoJNINative_onGlSurfaceChanged(
        JNIEnv *, jobject, jint width, jint height) {
    app.OnSurfaceChanged(width, height);
}

JNIEXPORT void JNICALL
Java_com_projecttango_examples_cpp_rgbdsegmentation_TangoJNINative_onGlSurfaceDrawFrame(
        JNIEnv *, jobject) {
    app.OnDrawFrame();
}

JNIEXPORT void JNICALL
Java_com_projecttango_examples_cpp_rgbdsegmentation_TangoJNINative_setDepthAlphaValue(
        JNIEnv *, jobject, jfloat alpha) {
    return app.SetDepthAlphaValue(alpha);
}

JNIEXPORT void JNICALL
Java_com_projecttango_examples_cpp_rgbdsegmentation_TangoJNINative_setMute(
        JNIEnv *, jobject, jboolean on) {
    return app.SetMute(on);
}

JNIEXPORT void JNICALL
Java_com_projecttango_examples_cpp_rgbdsegmentation_TangoJNINative_setModoVision(
        JNIEnv *, jobject, jint modo) {
    return app.SetModoVision(modo);
}

JNIEXPORT void JNICALL
Java_com_projecttango_examples_cpp_rgbdsegmentation_TangoJNINative_onDisplayChanged(
        JNIEnv *, jobject, jint display_rotation, jint color_camera_rotation) {
    return app.OnDisplayChanged(display_rotation, color_camera_rotation);
}

// StartAudio - Start audio engine and initialize player.
extern "C" JNIEXPORT void
Java_com_projecttango_examples_cpp_rgbdsegmentation_TangoJNINative_startAudio(
        JNIEnv *__unused env,
        jobject  __unused obj,
        jint samplerate,
        jint buffersize,
        jstring path,       // path to APK file
        jint offset,        // offset of audio file
        jint length         // length of audio file

) {
    const char *str = env->GetStringUTFChars(path, 0);
    //app.startAudio(samplerate,buffersize,str,offset,length);
    env->ReleaseStringUTFChars(path, str);
}

// onBackground - Put audio processing to sleep.
extern "C" JNIEXPORT void
Java_com_projecttango_examples_cpp_rgbdsegmentation_TangoJNINative_audioOnBackground(
        JNIEnv *__unused env,
        jobject __unused obj
) {
    //app.audioOnBackground();
}

// onForeground - Resume audio processing.
extern "C" JNIEXPORT void
Java_com_projecttango_examples_cpp_rgbdsegmentation_TangoJNINative_audioOnForeground(
        JNIEnv *__unused env,
        jobject __unused obj
) {
    //app.audioOnForeground();
}

// Cleanup - Free resources.
extern "C" JNIEXPORT void
Java_com_projecttango_examples_cpp_rgbdsegmentation_TangoJNINative_audioCleanup(
        JNIEnv *__unused env,
        jobject __unused obj
) {
    //app.audioCleanUp();
}

#ifdef __cplusplus
}
#endif

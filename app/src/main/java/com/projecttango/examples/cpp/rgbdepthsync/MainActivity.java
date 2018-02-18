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

package com.projecttango.examples.cpp.rgbdepthsync;

import android.app.Activity;
import android.content.ComponentName;
import android.content.ServiceConnection;
import android.graphics.Point;
import android.hardware.Camera;
import android.hardware.display.DisplayManager;
import android.opengl.GLSurfaceView;
import android.os.Bundle;
import android.os.IBinder;
import android.util.Log;
import android.view.Display;
import android.view.View;
import android.view.WindowManager;
import android.widget.CheckBox;
import android.widget.CompoundButton;
import android.widget.RadioGroup;
import android.widget.SeekBar;
import android.widget.Toast;

import com.projecttango.examples.cpp.util.TangoInitializationHelper;

/**
 * Activity that load up the main screen of the app, this is the launcher activity.
 */
public class MainActivity extends Activity {
    private static final String TAG = MainActivity.class.getSimpleName();

    // The minimum Tango Core version required from this application.
    private static final int MIN_TANGO_CORE_VERSION = 9377;

    // For all current Tango devices, color camera is in the camera id 0.
    private static final int COLOR_CAMERA_ID = 0;

    private GLSurfaceRenderer mRenderer;
    private GLSurfaceView mGLView;

    private SeekBar mDepthOverlaySeekbar;
    private CheckBox mdebugOverlayCheckbox;
    private CheckBox mGPUUpsampleCheckbox;
    private RadioGroup mGrupoModoVision;


    // Tango Service connection.
    ServiceConnection mTangoServiceConnection = new ServiceConnection() {
        public void onServiceConnected(ComponentName name, IBinder service) {
            TangoJNINative.onTangoServiceConnected(service);
            setAndroidOrientation();
        }

        public void onServiceDisconnected(ComponentName name) {
            // Handle this if you need to gracefully shutdown/retry
            // in the event that Tango itself crashes/gets upgraded while running.
        }
    };


    //Listeners de los botones de la interfaz
    private class DepthOverlaySeekbarListener implements SeekBar.OnSeekBarChangeListener {
        @Override
        public void onProgressChanged(SeekBar seekBar, int progress,
                                      boolean fromUser) {
            TangoJNINative.setDepthAlphaValue((float) progress / (float) seekBar.getMax());
        }

        @Override
        public void onStartTrackingTouch(SeekBar seekBar) {}

        @Override
        public void onStopTrackingTouch(SeekBar seekBar) {}
    }

    private class DebugOverlayCheckboxListener implements CheckBox.OnCheckedChangeListener {
        @Override
        public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
            if (buttonView == mdebugOverlayCheckbox) {
                if (isChecked) {
                    float progress = mDepthOverlaySeekbar.getProgress();
                    float max = mDepthOverlaySeekbar.getMax();
                    TangoJNINative.setDepthAlphaValue(progress / max);
                    mDepthOverlaySeekbar.setVisibility(View.VISIBLE);
                } else {
                    TangoJNINative.setDepthAlphaValue(0.0f);
                    mDepthOverlaySeekbar.setVisibility(View.GONE);
                }
            }
        }
    }

    private class MuteListener implements CheckBox.OnCheckedChangeListener {
        @Override
        public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
            TangoJNINative.setMute(isChecked);
        }
    }

    private class ModoVisionListener implements RadioGroup.OnCheckedChangeListener {
        @Override
        public void onCheckedChanged(RadioGroup group, int checkedId) {

            int modo = 0;

            switch (checkedId){
                case R.id.radio_puntos_validos:
                    modo=0;
                    break;
                case R.id.radio_coordenadas:
                    modo=1;
                    break;
                case R.id.radio_normales:
                    modo=2;
                    break;
                case R.id.radio_normales_x:
                    modo=3;
                    break;
                case R.id.radio_normales_y:
                    modo=4;
                    break;
                case R.id.radio_normales_z:
                    modo=5;
                    break;
                case R.id.radio_planos_locales:
                    modo=6;
                    break;
                case R.id.radio_planos_validos:
                    modo=7;
                    break;
                case R.id.radio_planos_extendidos:
                    modo=8;
                    break;
                case R.id.radio_planos_unidos:
                    modo=9;
                    break;
                case R.id.radio_elementos:
                    modo=10;
                    break;
                case R.id.radio_elementos_relevantes:
                    modo=11;
                    break;
                default:
                    modo=0;
            }

            TangoJNINative.setModoVision(modo);

        }
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        Display display = getWindowManager().getDefaultDisplay();
        Point size = new Point();
        display.getSize(size);

        DisplayManager displayManager = (DisplayManager) getSystemService(DISPLAY_SERVICE);
        if (displayManager != null) {
            displayManager.registerDisplayListener(new DisplayManager.DisplayListener() {
                @Override
                public void onDisplayAdded(int displayId) {

                }

                @Override
                public void onDisplayChanged(int displayId) {
                    synchronized (this) {
                        setAndroidOrientation();
                    }
                }

                @Override
                public void onDisplayRemoved(int displayId) {}
            }, null);
        }

        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN,
                WindowManager.LayoutParams.FLAG_FULLSCREEN);

        setContentView(R.layout.activity_main);


        //Listeners de los botones de la interfaz
        mDepthOverlaySeekbar = (SeekBar) findViewById(R.id.depth_overlay_alpha_seekbar);
        mDepthOverlaySeekbar.setOnSeekBarChangeListener(new DepthOverlaySeekbarListener());
        mDepthOverlaySeekbar.setVisibility(View.GONE);

        mdebugOverlayCheckbox = (CheckBox) findViewById(R.id.debug_overlay_checkbox);
        mdebugOverlayCheckbox.setOnCheckedChangeListener(new DebugOverlayCheckboxListener());

        mGPUUpsampleCheckbox = (CheckBox) findViewById(R.id.sonido_checkbox);
        mGPUUpsampleCheckbox.setOnCheckedChangeListener(new MuteListener());

        mGrupoModoVision = (RadioGroup) findViewById(R.id.grupo_modo_vision);
        mGrupoModoVision.setOnCheckedChangeListener(new ModoVisionListener());


        // OpenGL view where all of the graphics are drawn
        mGLView = (GLSurfaceView) findViewById(R.id.gl_surface_view);

        // Configure OpenGL renderer
        mGLView.setEGLContextClientVersion(2);
        mRenderer = new GLSurfaceRenderer(this);
        mGLView.setRenderer(mRenderer);

        TangoJNINative.onCreate(this);
    }

    @Override
    protected void onResume() {
        // We moved most of the onResume lifecycle calls to the surfaceCreated,
        // surfaceCreated will be called after the GLSurface is created.
        super.onResume();
        mGLView.onResume();
        TangoInitializationHelper.bindTangoService(this, mTangoServiceConnection);
    }

    @Override
    protected void onPause() {
        super.onPause();
        mGLView.onPause();
        TangoJNINative.onPause();
        unbindService(mTangoServiceConnection);
    }

    public void surfaceCreated() {
        TangoJNINative.onGlSurfaceCreated();
    }

    // Pass device's camera sensor rotation and display rotation to native layer.
    // These two parameter are important for Tango to render video overlay and
    // virtual objects in the correct device orientation.
    private void setAndroidOrientation() {
        Display display = getWindowManager().getDefaultDisplay();
        Camera.CameraInfo colorCameraInfo = new Camera.CameraInfo();
        Camera.getCameraInfo(COLOR_CAMERA_ID, colorCameraInfo);

        TangoJNINative.onDisplayChanged(display.getRotation(), colorCameraInfo.orientation);
    }
}

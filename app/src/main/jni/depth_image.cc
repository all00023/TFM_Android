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

#include "tango-gl/conversions.h"
#include "tango-gl/camera.h"

#include "rgb-depth-sync/depth_image.h"

namespace {
    const std::string kPointCloudVertexShader =
            "precision mediump float;\n"
                    "\n"
                    "attribute vec4 vertex;\n"
                    "\n"
                    "uniform mat4 mvp;\n"
                    "uniform float maxdepth;\n"
                    "uniform float pointsize;\n"
                    "\n"
                    "varying vec4 v_color;\n"
                    "\n"
                    "void main() {\n"
                    "  gl_PointSize = pointsize;\n"
                    "  gl_Position = mvp*vertex;\n"
                    "  float depth = clamp(vertex.z / maxdepth, 0.0, 1.0);\n"
                    "  v_color = vec4(depth, depth, depth, 1.0);\n"
                    "}\n";
    const std::string kPointCloudFragmentShader =
            "precision mediump float;\n"
                    "\n"
                    "varying vec4 v_color;\n"
                    "void main() {\n"
                    "  gl_FragColor = v_color;\n"
                    "}\n";
}  // namespace

namespace rgb_depth_sync {

    DepthImage::DepthImage()
            : texture_id_(0),
              cpu_texture_id_(0),
              gpu_texture_id_(0),
              depth_map_buffer_(0),
              grayscale_display_buffer_(0),
              texture_render_program_(0),
              fbo_handle_(0),
              vertex_buffer_handle_(0),
              vertices_handle_(0),
              mvp_handle_(0) {}

    DepthImage::~DepthImage() {}

    void DepthImage::InitializeGL() {
        texture_id_ = 0;
        cpu_texture_id_ = 0;
        gpu_texture_id_ = 0;

        texture_render_program_ = 0;
        fbo_handle_ = 0;
        vertex_buffer_handle_ = 0;
        vertices_handle_ = 0;
        mvp_handle_ = 0;
    }

    bool DepthImage::CreateOrBindCPUTexture() {
        if (cpu_texture_id_) {
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, cpu_texture_id_);
            return false;
        } else {
            glGenTextures(1, &cpu_texture_id_);
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, cpu_texture_id_);
            glTexImage2D(GL_TEXTURE_2D, 0,  // mip-map level
                         GL_RGBA, rgb_camera_intrinsics_.width,
                         rgb_camera_intrinsics_.height, 0,  // border
                         GL_RGBA, GL_UNSIGNED_SHORT_5_5_5_1, nullptr);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            return true;
        }
    }

// Update function will be called in application's main render loop. This funct-
// ion takes care of projecting raw depth points on the image plane, and render
// the depth image into a texture.
// This function also takes care of swapping the Render buffer and shared buffer
// if there is new point cloud data available.
// color_t1_T__depth_t0: 't1' represents the color camera frame's timestamp,
// 't0'
// represents
// the depth camera timestamp. The transformation is the depth camera's frame on
// timestamp t0 with respect the rgb camera's frame on timestamp t1.
    void DepthImage::UpdateAndUpsampleDepth(
            const glm::mat4 &color_t1_T_depth_t0,
            const TangoPointCloud *render_point_cloud_buffer,
            int modoVista) {
        int depth_image_width = rgb_depth_camera_intrinsics_.width / 2;
        int depth_image_height = rgb_depth_camera_intrinsics_.height / 2;
        int depth_image_size = depth_image_width * depth_image_height;
        int rgb_image_width = rgb_camera_intrinsics_.width;
        int rgb_image_height = rgb_camera_intrinsics_.height;
        int rgb_image_size = rgb_image_width * rgb_image_height;

        //depth_map_buffer_.resize(rgb_image_size);
        grayscale_display_buffer_.resize(rgb_image_size);

        //std::fill(depth_map_buffer_.begin(), depth_map_buffer_.end(), 0);
        std::fill(grayscale_display_buffer_.begin(), grayscale_display_buffer_.end(), 0);

        //LOGE("El modo de vision seleccionado es %i", modoVista);

        clock_t inicioProcesado;
        clock_t finProcesado;
        //LOGE("1");
        inicioProcesado = clock();

        vector<Punto3D> puntos = vector<Punto3D>(depth_image_width * depth_image_height);
        map<int, Plano3D> planos;
        map<int, Elemento3D> elementos;

        set<int> etiquetaSuelo = set<int>();
        map<int, int> etiquetasRelevantes = map<int, int>();

        //LOGE("2");
        mapear(puntos, render_point_cloud_buffer, rgb_depth_camera_intrinsics_, depth_image_width,
               depth_image_width);

        //datos de la imagen para depuracion
        //LOGE("X: %f.2 -> %f.2",getMinX(puntos), getMaxX(puntos));
        //LOGE("Y: %f.2 -> %f.2",getMinY(puntos), getMaxY(puntos));
        //LOGE("Z: %f.2 -> %f.2",getMinZ(puntos), getMaxZ(puntos));
        //detectarSuelo(planos,etiquetaSuelo,render_point_cloud_buffer->timestamp);

        if (modoVista == 0) {

            colorearPorValidos(puntos, grayscale_display_buffer_, depth_image_width,
                               depth_image_height, 8);

        } else if (modoVista == 1) {

            colorearPorCoordenadas(puntos, grayscale_display_buffer_, depth_image_width,
                                   depth_image_height, 8);

        } else {
            // LOGE("3");
            int nObjetosRelevantes = procesar(puntos, planos, elementos, comparadorNormales,
                                              comparadorProfundidad,
                                              render_point_cloud_buffer->timestamp,
                                              depth_image_width, depth_image_height, etiquetaSuelo,
                                              etiquetasRelevantes, modoVista);

            finProcesado = clock();
            double procesado = ((finProcesado - inicioProcesado) * 1.0) / CLOCKS_PER_SEC;

            LOGI("FPS: %.2f \tTiempo de procesado: %.3f segundos", 1 / procesado, procesado);
            //LOGE("4");

            if (modoVista == 2)
                colorearPorNormales(puntos, grayscale_display_buffer_, depth_image_width,
                                    depth_image_height, 8);
            else if (modoVista == 3)
                colorearPorNormalesX(puntos, grayscale_display_buffer_, depth_image_width,
                                     depth_image_height, 8);
            else if (modoVista == 4)
                colorearPorNormalesY(puntos, grayscale_display_buffer_, depth_image_width,
                                     depth_image_height, 8);
            else if (modoVista == 5)
                colorearPorNormalesZ(puntos, grayscale_display_buffer_, depth_image_width,
                                     depth_image_height, 8);
            else if (modoVista >= 6 && modoVista <= 10)
                colorearPorEtiqueta(puntos, grayscale_display_buffer_, depth_image_width,
                                    depth_image_height, 8);
            else if (modoVista >= 11) {
                //colorearRelevantes

                //Relevantes
                Elemento3D *p1 = NULL;
                Elemento3D *p2 = NULL;
                Elemento3D *p3 = NULL;
                Elemento3D *temporal = NULL;
                map<int, int>::iterator iter = etiquetasRelevantes.begin();

                while (iter != etiquetasRelevantes.end()) {
                    int id = iter->second;

                    if (id >= 0) {
                        map<int, Plano3D>::iterator itPl = planos.find(id);
                        if (itPl != planos.end()) {
                            temporal = &(itPl->second);
                        }
                    } else {
                        map<int, Elemento3D>::iterator itEl = elementos.find(id);
                        if (itEl != elementos.end()) {
                            temporal = &(itEl->second);
                        }
                    }

                    if (iter->first == 0) {
                        p1 = temporal;
                    } else if (iter->first == 1) {
                        p2 = temporal;
                    } else if (iter->first == 2) {
                        p3 = temporal;
                    }
                    iter++;
                }

                colorearPorEtiquetaRelevantes(puntos, grayscale_display_buffer_, depth_image_width,
                                              depth_image_height, 8, etiquetaSuelo,
                                              etiquetasRelevantes);

                asignarSonidoAElementos(p1, p2, p3, depth_image_size);
            }
            //LOGE("5");
            imprimirNumero(grayscale_display_buffer_, nObjetosRelevantes, rgb_image_width,
                           rgb_image_height, 0, 0);
            //LOGE("6");
            //UpSampleDepthAroundPoint(&grayscale_display_buffer_, &depth_map_buffer_);
        }


        this->CreateOrBindCPUTexture();
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, rgb_image_width, rgb_image_height,
                        GL_RGBA, GL_UNSIGNED_SHORT_5_5_5_1,
                        grayscale_display_buffer_.data());
        tango_gl::util::CheckGlError("DepthImage glTexSubImage2D");
        glBindTexture(GL_TEXTURE_2D, 0);

        texture_id_ = cpu_texture_id_;
    }

    void DepthImage::SetCameraIntrinsics(TangoCameraIntrinsics intrinsics) {
        rgb_camera_intrinsics_ = intrinsics;
        const float kNearClip = 0.1;
        const float kFarClip = 10.0;
        projection_matrix_ar_ = tango_gl::Camera::ProjectionMatrixForCameraIntrinsics(
                intrinsics.width, intrinsics.height, intrinsics.fx, intrinsics.fy,
                intrinsics.cx, intrinsics.cy, kNearClip, kFarClip);
    }

    void DepthImage::SetDepthCameraIntrinsics(TangoCameraIntrinsics intrinsics) {
        rgb_depth_camera_intrinsics_ = intrinsics;
    }

    //Sonido
    static SuperpoweredAndroidAudioIO *audioIO;
    static SuperpoweredWhoosh *wh;
    static SuperpoweredStereoMixer *mixer;
    static float *floatBufferMixer;
    static bool mute = true;
    static int sonidosSim = 1;

    static SuperpoweredAdvancedAudioPlayer *player1;
    static SuperpoweredSpatializer *spatializer1;
    static float *floatBuffer1;
    static bool valido1 = false;
    static float azimuth1 = 0;
    static float elevation1 = 0;
    static float pitch1 = 0;
    static float tempo1 = 1;

    static SuperpoweredAdvancedAudioPlayer *player2;
    static SuperpoweredSpatializer *spatializer2;
    static float *floatBuffer2;
    static bool valido2 = false;
    static float azimuth2 = 0;
    static float elevation2 = 0;
    static float pitch2 = 0;
    static float tempo2 = 1;

    static SuperpoweredAdvancedAudioPlayer *player3;
    static SuperpoweredSpatializer *spatializer3;
    static float *floatBuffer3;
    static bool valido3 = false;
    static float azimuth3 = 0;
    static float elevation3 = 0;
    static float pitch3 = 0;
    static float tempo3 = 1;

    // This is called periodically by the audio engine.
    static bool audioProcessing(
            void *__unused clientdata, // custom pointer
            short int *audio,           // buffer of interleaved samples
            int numberOfFrames,         // number of frames to process
            int __unused samplerate     // sampling rate
    ) {

        if (mute)
            return false;

        if (!valido1 && !valido2 && !valido3)
            return false;

        float *inputs[4] = {NULL, NULL, NULL, NULL};
        float *outputs[2] = {floatBufferMixer, NULL};
        float inputLevels[8] = {1, 1, 1, 1, 1, 1, 1, 1};
        float outputLevels[2] = {1, 1};
        bool algunValido = false; //No podemos asegurar que las variables sean threadsafe, asi se evita que cambien los validos


        if (valido1 && sonidosSim >= 1) {
            player1->setTempo(tempo1, true);
            player1->setPitchShift(pitch1);
            spatializer1->azimuth = azimuth1;
            spatializer1->elevation = elevation1;

            if (!player1->process(floatBuffer1, false, (unsigned int) numberOfFrames))
                return false;
            if (!spatializer1->process(floatBuffer1, NULL, floatBuffer1, NULL,
                                       (unsigned int) numberOfFrames, true))
                return false;


            inputs[0] = floatBuffer1;
            algunValido = true;
        }

        if (valido2 && sonidosSim >= 2) {

            player2->setTempo(tempo2, true);
            player2->setPitchShift(pitch2);
            spatializer2->azimuth = azimuth2;
            spatializer2->elevation = elevation2;

            if (!player2->process(floatBuffer2, false, (unsigned int) numberOfFrames))
                return false;
            if (!spatializer2->process(floatBuffer2, NULL, floatBuffer2, NULL,
                                       (unsigned int) numberOfFrames, true))
                return false;

            inputs[1] = floatBuffer2;
            algunValido = true;

        }
        if (valido3 && sonidosSim >= 3) {

            player3->setTempo(tempo3, true);
            player3->setPitchShift(pitch3);
            spatializer3->azimuth = azimuth3;
            spatializer3->elevation = elevation3;

            if (!player3->process(floatBuffer3, false, (unsigned int) numberOfFrames))
                return false;
            if (!spatializer3->process(floatBuffer3, NULL, floatBuffer3, NULL,
                                       (unsigned int) numberOfFrames, true))
                return false;

            inputs[2] = floatBuffer3;
            algunValido = true;

        }

        if (algunValido) {
            mixer->process(inputs, outputs, inputLevels, outputLevels, NULL, NULL,
                           (unsigned int) numberOfFrames);

            SuperpoweredFloatToShortInt(floatBufferMixer, audio, (unsigned int) numberOfFrames);


            return true;
        } else {
            return false;
        }

    }

    // Called by the player.
    static void playerEventCallback1(
            void *__unused clientData,
            SuperpoweredAdvancedAudioPlayerEvent event,
            void *value
    ) {
        switch (event) {
            case SuperpoweredAdvancedAudioPlayerEvent_LoadSuccess:
                LOGE("Cargado el archivo de audio");
                break;
            case SuperpoweredAdvancedAudioPlayerEvent_LoadError:
                LOGE("Error al cargar el archivo de audio");
                break;
            case SuperpoweredAdvancedAudioPlayerEvent_EOF:
                player1->seek(0);    // loop track
                break;
            default:;
        };
    }

    // Called by the player.
    static void playerEventCallback2(
            void *__unused clientData,
            SuperpoweredAdvancedAudioPlayerEvent event,
            void *value
    ) {
        switch (event) {
            case SuperpoweredAdvancedAudioPlayerEvent_LoadSuccess:
                LOGE("Cargado el archivo de audio");
                break;
            case SuperpoweredAdvancedAudioPlayerEvent_LoadError:
                LOGE("Error al cargar el archivo de audio");
                break;
            case SuperpoweredAdvancedAudioPlayerEvent_EOF:
                player2->seek(0);    // loop track
                break;
            default:;
        };
    }

    // Called by the player.
    static void playerEventCallback3(
            void *__unused clientData,
            SuperpoweredAdvancedAudioPlayerEvent event,
            void *value
    ) {
        switch (event) {
            case SuperpoweredAdvancedAudioPlayerEvent_LoadSuccess:
                LOGE("Cargado el archivo de audio");
                break;
            case SuperpoweredAdvancedAudioPlayerEvent_LoadError:
                LOGE("Error al cargar el archivo de audio");
                break;
            case SuperpoweredAdvancedAudioPlayerEvent_EOF:
                player3->seek(0);    // loop track
                break;
            default:;
        };
    }

    void DepthImage::startAudio(int samplerate, int buffersize, const char *path,
                                int offset, int length) {

        // Allocate audio buffer.
        floatBuffer1 = (float *) malloc(sizeof(float) * 2 * buffersize);
        floatBuffer2 = (float *) malloc(sizeof(float) * 2 * buffersize);
        floatBuffer3 = (float *) malloc(sizeof(float) * 2 * buffersize);
        floatBufferMixer = (float *) malloc(sizeof(float) * 2 * buffersize);

        // Initialize player and pass callback function.
        player1 = new SuperpoweredAdvancedAudioPlayer(
                NULL,                           // clientData
                playerEventCallback1,            // callback function
                (unsigned int) samplerate,       // sampling rate
                0                               // cachedPointCount
        );
        player2 = new SuperpoweredAdvancedAudioPlayer(
                NULL,                           // clientData
                playerEventCallback2,            // callback function
                (unsigned int) samplerate,       // sampling rate
                0                               // cachedPointCount
        );
        player3 = new SuperpoweredAdvancedAudioPlayer(
                NULL,                           // clientData
                playerEventCallback3,            // callback function
                (unsigned int) samplerate,       // sampling rate
                0                               // cachedPointCount
        );

        // Initialize audio with audio callback function.
        audioIO = new SuperpoweredAndroidAudioIO(
                (unsigned int) samplerate,                     // sampling rate
                buffersize,                     // buffer size
                false,                          // enableInput
                true,                           // enableOutput
                audioProcessing,                // process callback function
                NULL,                           // clientData
                -1,                             // inputStreamType (-1 = default)
                SL_ANDROID_STREAM_MEDIA,        // outputStreamType (-1 = default)
                buffersize * 2                  // latencySamples
        );

        mixer = new SuperpoweredStereoMixer();

        spatializer1 = new SuperpoweredSpatializer((unsigned int) samplerate);
        player1->open(path, offset, length);
        player1->play(false);

        spatializer2 = new SuperpoweredSpatializer((unsigned int) samplerate);
        player2->open(path, offset, length);
        player2->play(false);

        spatializer3 = new SuperpoweredSpatializer((unsigned int) samplerate);
        player3->open(path, offset, length);
        player3->play(false);

    }

    void DepthImage::setMute(bool on) { mute = on; }

    void DepthImage::audioOnBackground() {
        audioIO->onBackground();
        valido1 = false;
        valido2 = false;
        valido3 = false;
    }

    void DepthImage::audioOnForeground() {
        audioIO->onForeground();
    }

    void DepthImage::audioCleanUp() {
        delete audioIO;
        delete mixer;
        free(floatBufferMixer);

        delete player1;
        delete spatializer1;
        free(floatBuffer1);

        delete player2;
        delete spatializer2;
        free(floatBuffer2);

        delete player3;
        delete spatializer3;
        free(floatBuffer3);
    }

    void DepthImage::setSonidosSimultaneos(int sonSim) {
        sonidosSim = sonSim;
    }

    float invertirYModificarRango(float value, float min1, float max2, float escala) {
        return max2 - ((value - min1) / escala);
    }

    void DepthImage::asignarSonidoAElementos(Elemento3D *p1, Elemento3D *p2, Elemento3D *p3,
                                             int tamImagen) {

        float distanciaMinima = 0.5;
        float distanciaMaxima = 5;
        float tempoMinimo = 0.2;
        float tempoMaximo = 3;
        float tamMinimo = 0;
        float tamMaximo = tamImagen;
        float pitchMinimo = 0;
        float pitchMaximo = 24;
        float pitchDiff = 12;
        float preEscalaTempo = (distanciaMaxima - distanciaMinima) / (tempoMaximo - tempoMinimo);
        float preEscalaPitch = (tamMaximo - tamMinimo) / (pitchMaximo - pitchMinimo);
        float preRadToGrad = 180 / pi;
        float multiplicadorAngulo = 3;

        if (p1 == NULL) {
            valido1 = false;
        } else {
            valido1 = true;

            azimuth1 = multiplicadorAngulo * acos(p1->getCZ() /
                                                  (sqrt(p1->getCX() * p1->getCX() +
                                                        p1->getCZ() * p1->getCZ()) * 1.0)) *
                       preRadToGrad;
            azimuth1 = p1->getCX() < 0 ? 360 - azimuth1 : azimuth1;

            elevation1 = multiplicadorAngulo * acos(p1->getCZ() / (sqrt(p1->getCY() * p1->getCY() +
                                                                        p1->getCZ() * p1->getCZ()) *
                                                                   1.0)) * preRadToGrad;
            elevation1 = p1->getCY() < 0 ? 360 - elevation1 : elevation1;

            tempo1 = invertirYModificarRango(p1->getCZ(), distanciaMinima, tempoMaximo,
                                             preEscalaTempo);
            tempo1 = tempo1 > 3 ? 3 : tempo1;
            pitch1 = invertirYModificarRango(p1->getNumeroPuntos(), tamMinimo, pitchMaximo,
                                             preEscalaPitch) - pitchDiff;
        }

        if (p2 == NULL) {
            valido2 = false;
        } else {
            valido2 = true;

            azimuth2 = multiplicadorAngulo * acos(p2->getCZ() /
                                                  (sqrt(p2->getCX() * p2->getCX() +
                                                        p2->getCZ() * p2->getCZ()) * 1.0)) *
                       preRadToGrad;
            azimuth2 = p2->getCX() < 0 ? 360 - azimuth2 : azimuth2;

            elevation2 = multiplicadorAngulo * acos(p2->getCZ() / (sqrt(p2->getCY() * p2->getCY() +
                                                                        p2->getCZ() * p2->getCZ()) *
                                                                   1.0)) * preRadToGrad;
            elevation2 = p2->getCY() < 0 ? 360 - elevation2 : elevation2;

            tempo2 = invertirYModificarRango(p2->getCZ(), distanciaMinima, tempoMaximo,
                                             preEscalaTempo);
            tempo2 = tempo2 > 3 ? 3 : tempo2;
            pitch2 = invertirYModificarRango(p2->getNumeroPuntos(), tamMinimo, pitchMaximo,
                                             preEscalaPitch) - pitchDiff;
        }

        if (p3 == NULL) {
            valido3 = false;
        } else {
            valido3 = true;

            azimuth3 = multiplicadorAngulo * acos(p3->getCZ() /
                                                  (sqrt(p3->getCX() * p3->getCX() +
                                                        p3->getCZ() * p3->getCZ()) * 1.0)) *
                       preRadToGrad;
            azimuth3 = p3->getCX() < 0 ? 360 - azimuth3 : azimuth3;

            elevation3 = multiplicadorAngulo * acos(p3->getCZ() / (sqrt(p3->getCY() * p3->getCY() +
                                                                        p3->getCZ() * p3->getCZ()) *
                                                                   1.0)) * preRadToGrad;
            elevation3 = p3->getCY() < 0 ? 360 - elevation3 : elevation3;

            tempo3 = invertirYModificarRango(p3->getCZ(), distanciaMinima, tempoMaximo,
                                             preEscalaTempo);
            tempo3 = tempo3 > 3 ? 3 : tempo3;
            pitch3 = invertirYModificarRango(p3->getNumeroPuntos(), tamMinimo, pitchMaximo,
                                             preEscalaPitch) - pitchDiff;
        }

    }


}  // namespace rgb_depth_sync

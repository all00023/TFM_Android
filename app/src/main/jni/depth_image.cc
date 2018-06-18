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

#include "rgbdsegmentation/depth_image.h"

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
            int modoVista,
            SynchronizationApplication *app) {
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
        mapear(puntos, render_point_cloud_buffer, rgb_depth_camera_intrinsics_, depth_image_width, depth_image_width);

        //datos de la imagen para depuracion
        //LOGE("X: %f.2 -> %f.2",getMinX(puntos), getMaxX(puntos));
        //LOGE("Y: %f.2 -> %f.2",getMinY(puntos), getMaxY(puntos));
        //LOGE("Z: %f.2 -> %f.2",getMinZ(puntos), getMaxZ(puntos));
        //detectarSuelo(planos,etiquetaSuelo,render_point_cloud_buffer->timestamp);

        if (modoVista == 0) {

            colorearPorValidos(puntos, grayscale_display_buffer_, depth_image_width, depth_image_height, 8);

        } else if (modoVista == 1) {

            colorearPorCoordenadas(puntos, grayscale_display_buffer_, depth_image_width, depth_image_height, 8);

        } else {
           // LOGE("3");
            int nObjetosRelevantes = procesar(puntos, planos, elementos, comparadorNormales, comparadorProfundidad, render_point_cloud_buffer->timestamp,depth_image_width, depth_image_height, etiquetaSuelo, etiquetasRelevantes, modoVista);

            finProcesado = clock();
            double procesado = ((finProcesado - inicioProcesado) * 1.0) / CLOCKS_PER_SEC;

            LOGI("FPS: %.2f \tTiempo de procesado: %.3f segundos", 1 / procesado, procesado);
            //LOGE("4");

            if (modoVista == 2)
                colorearPorNormales(puntos, grayscale_display_buffer_, depth_image_width, depth_image_height, 8);
            else if (modoVista == 3)
                colorearPorNormalesX(puntos, grayscale_display_buffer_, depth_image_width, depth_image_height, 8);
            else if (modoVista == 4)
                colorearPorNormalesY(puntos, grayscale_display_buffer_, depth_image_width, depth_image_height, 8);
            else if (modoVista == 5)
                colorearPorNormalesZ(puntos, grayscale_display_buffer_, depth_image_width, depth_image_height, 8);
            else if (modoVista >= 6 && modoVista <= 10)
                colorearPorEtiqueta(puntos, grayscale_display_buffer_, depth_image_width, depth_image_height, 8);
            else if (modoVista >= 11) {
                //colorearRelevantes

                //("Relevantes");
//                map<int, int>::iterator iter = etiquetasRelevantes.begin();
//                while (iter != etiquetasRelevantes.end()) {
//                    LOGE("%i -> %i", iter->first, iter->second);
//                    iter++;
//                }

                //colorearPorValidos(puntos, grayscale_display_buffer_, depth_image_width, depth_image_height, 8);
                colorearPorEtiquetaRelevantes(puntos, grayscale_display_buffer_, depth_image_width, depth_image_height, 8, etiquetaSuelo, etiquetasRelevantes);
            }
            //LOGE("5");
            imprimirNumero(grayscale_display_buffer_, nObjetosRelevantes, rgb_image_width, rgb_image_height, 0, 0);
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

}  // namespace rgb_depth_sync

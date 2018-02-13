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

    bool DepthImage::CreateOrBindGPUTexture() {
        if (gpu_texture_id_) {
            glBindFramebuffer(GL_FRAMEBUFFER, fbo_handle_);
            return false;
        } else {
            glGenTextures(1, &gpu_texture_id_);
            texture_render_program_ = tango_gl::util::CreateProgram(
                    kPointCloudVertexShader.c_str(), kPointCloudFragmentShader.c_str());

            mvp_handle_ = glGetUniformLocation(texture_render_program_, "mvp");

            glUseProgram(texture_render_program_);
            // Assume these are constant for the life the program
            GLuint max_depth_handle =
                    glGetUniformLocation(texture_render_program_, "maxdepth");
            GLuint point_size_handle =
                    glGetUniformLocation(texture_render_program_, "pointsize");
            glUniform1f(max_depth_handle,
                        static_cast<float>(kMaxDepthDistance) / kMeterToMillimeter);
            glUniform1f(point_size_handle, 2 * kWindowSize + 1);

            vertices_handle_ = glGetAttribLocation(texture_render_program_, "vertex");

            glGenBuffers(1, &vertex_buffer_handle_);

            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, gpu_texture_id_);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, rgb_camera_intrinsics_.width,
                         rgb_camera_intrinsics_.height, 0, GL_RGBA, GL_UNSIGNED_BYTE,
                         nullptr);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            glBindTexture(GL_TEXTURE_2D, 0);

            glGenFramebuffers(1, &fbo_handle_);
            glBindFramebuffer(GL_FRAMEBUFFER, fbo_handle_);
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                                   gpu_texture_id_, 0);

            return true;
        }
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

    void DepthImage::RenderDepthToTexture(
            const glm::mat4 &color_t1_T_depth_t0,
            const TangoPointCloud *render_point_cloud_buffer, bool new_points) {
        new_points = this->CreateOrBindGPUTexture() || new_points;

        glViewport(0, 0, rgb_camera_intrinsics_.width, rgb_camera_intrinsics_.height);
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Special program needed to color by z-distance
        glUseProgram(texture_render_program_);

        glDisable(GL_BLEND);
        glEnable(GL_DEPTH_TEST);

        glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_handle_);
        if (new_points) {
            glBufferData(GL_ARRAY_BUFFER,
                         sizeof(GLfloat) * render_point_cloud_buffer->num_points * 4,
                         render_point_cloud_buffer->points, GL_STATIC_DRAW);
        }
        tango_gl::util::CheckGlError("DepthImage Buffer");

        // Skip negation of Y-axis as is normally done in opengl_T_color
        // since we are rendering to a texture that we want to match the Y-axis
        // of the color image.
        const glm::mat4 opengl_T_color(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                                       0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                                       1.0f);

        glm::mat4 mvp_mat =
                projection_matrix_ar_ * opengl_T_color * color_t1_T_depth_t0;

        glUniformMatrix4fv(mvp_handle_, 1, GL_FALSE, glm::value_ptr(mvp_mat));

        glEnableVertexAttribArray(vertices_handle_);
        glVertexAttribPointer(vertices_handle_, 4, GL_FLOAT, GL_FALSE, 0, nullptr);

        glDrawArrays(GL_POINTS, 0, render_point_cloud_buffer->num_points);
        glDisableVertexAttribArray(vertices_handle_);

        tango_gl::util::CheckGlError("DepthImage Draw");

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glUseProgram(0);

        tango_gl::util::CheckGlError("DepthImage RenderTexture");

        texture_id_ = gpu_texture_id_;
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
            const TangoPointCloud *render_point_cloud_buffer) {
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

        clock_t inicioProcesado;
        clock_t finProcesado;
        LOGE("1");
        inicioProcesado = clock();

        vector<Punto3D> puntos = vector<Punto3D>(depth_image_width * depth_image_height);
        map<int, Plano3D> planos;
        map<int, Elemento3D> elementos;

        set<int> etiquetaSuelo = set<int>();
        map<int, int>etiquetasRelevantes = map<int, int>();

        LOGE("2");
        mapear(puntos, render_point_cloud_buffer, rgb_depth_camera_intrinsics_, depth_image_width, depth_image_width);
        LOGE("3");
        int nObjetosRelevantes = procesar(puntos, planos, elementos, comparadorNormales, comparadorProfundidad, depth_image_width, depth_image_height, etiquetaSuelo, etiquetasRelevantes);

        finProcesado = clock();
        double procesado = ((finProcesado - inicioProcesado) * 1.0) / CLOCKS_PER_SEC;

        LOGI("FPS: %.2f \tTiempo de procesado: %.3f segundos", 1 / procesado, procesado);
        LOGE("4");
        colorearPorEtiqueta(puntos, grayscale_display_buffer_, depth_image_width, depth_image_height, 8);
        LOGE("5");
        imprimirNumero(grayscale_display_buffer_, planos.size(), rgb_image_width, rgb_image_height);
        LOGE("6");
        //UpSampleDepthAroundPoint(&grayscale_display_buffer_, &depth_map_buffer_);

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

    void DepthImage::UpSampleDepthAroundPoint(
            uint8_t grayscale_value, float depth_value, int pixel_x, int pixel_y,
            std::vector<uint16_t> *grayscale_buffer,
            std::vector<float> *depth_map_buffer) {

        int image_width = rgb_camera_intrinsics_.width;
        int image_height = rgb_camera_intrinsics_.height;
        int image_size = image_height * image_width;
        uint16_t value;
        if (depth_value < 1.5) {
            value = ((depth_value) / 1.5) * 16;
            value = value > 16 ? 16 : value;
            value = value | 0b1111000000000000;
        } else if (depth_value < 3) {
            value = ((depth_value - 1.5) / 1.5) * 16;
            value = value > 16 ? 16 : value;
            value = value | 0b0000111100000000;
        } else if (depth_value >= 3) {
            value = ((depth_value - 3) / 1) * 16;
            value = value > 16 ? 16 : value;
            value = value | 0b0000000011110000;
        } else {
            value = 0;
        }

        // Set the neighbour pixels to same color.
        for (int a = -kWindowSize; a <= kWindowSize; ++a) {
            for (int b = -kWindowSize; b <= kWindowSize; ++b) {
                if (pixel_x > image_width || pixel_y > image_height || pixel_x < 0 ||
                    pixel_y < 0) {
                    continue;
                }
                int pixel_num = (pixel_x + a) + (pixel_y + b) * image_width;

                //La profundidad varía entre 0 y 4000 (puede que mas, pero lo limito a 4 metros)
                if (pixel_num > 0 && pixel_num < image_size) {

                    (*grayscale_buffer)[pixel_num] = value;
                    (*depth_map_buffer)[pixel_num] = depth_value;

                }
            }
        }
    }

}  // namespace rgb_depth_sync

// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#pragma once

#include <map>
#include <vector>

#include "proc/align.h"
#include "synthetic-stream-gl.h"

#include <librealsense2/rs.hpp>
#include "opengl3.h"

#include <memory>

namespace rs2
{
    class stream_profile;
    class visualizer_2d;
    class texture_buffer;
}

/*
static const char* vertex_shader_text =
"#version 130\n"
"attribute vec3 position;\n"
"attribute vec2 textureCoords;\n"
"varying vec2 textCoords;\n"
"uniform vec2 elementPosition;\n"
"uniform vec2 elementScale;\n"
"void main(void)\n"
"{\n"
"    gl_Position = vec4(position * vec3(elementScale, 1.0) + vec3(elementPosition, 0.0), 1.0);\n"
"    textCoords = textureCoords;\n"
"}";

static const char* fragment_shader_text =
"#version 130\n"
"out int histData;\n"
"varying vec2 textCoords;\n"
"uniform sampler2D textureSampler;\n"
"uniform int height;\n"
"uniform int width;\n"
"void main(void) {\n"
"    float x = textCoords.x;\n"
"    float y = 1.0f - textCoords.y;\n"
"    float d = (x * 256.0f) - 0.5f + ((y * 256.0f) - 0.5f) * 256.0f;\n"
"    int tempCounter = 5;\n"
"    int total_pixels = height * width;\n"
"    float nd = 0.0f;\n"
"    int w = 0;\n"
"    int h = 0;\n"
"    for (w = 0; w < 1280; w++)\n"
"    {\n"
"        for (h = 0; h < 512; h++)\n"
"        {\n"
"            //float tx = float(w)/float(width);\n"
"            //float ty = float(h)/float(height);\n"
"            //vec2 tex = vec2(tx, ty);\n"
"            //vec4 depth = texture2D(textureSampler, tex);\n"
"            //float dx = depth.r;\n"
"            //float dy = depth.g;\n"
"            //nd = (dx + dy * 256.0f);\n"
"            //if (nd < d)\n"
"            {\n"
"                tempCounter++;\n"
"            }\n"
"        }\n"
"    }\n"
"    //int total_pixels = height * width;\n"
"    //histData = float(tempCounter)/float(total_pixels);\n"
"    //vec4 dep = texture2D(textureSampler, textCoords);\n"
"    //histData = (dep.r * 256.0f) + (dep.g * 256.0f) * 256.0f;\n"
"    //histData = textCoords.x;\n"
"    //float f_count = float(tempCounter);\n"
"    histData = tempCounter;\n"
"}";

/*
"    float nd = 0.0f;\n"
"    int w = 0;\n"
"    int h = 0;\n"
"    for (w = 0; w < (width * height); w++)\n"
"    {\n"
"        for (h = 0; h < height; h++)\n"
"        {\n"
"            float tx = float(w)/float(width);\n"
"            float ty = float(h)/float(height);\n"
"            vec2 tex = vec2(tx, ty);\n"
"            vec4 depth = texture2D(textureSampler, tex);\n"
"            float dx = depth.r;\n"
"            float dy = depth.g;\n"
"            nd = (dx + dy * 256.0f);\n"
"            if (nd < d)\n"
"            {\n"
"                tempCounter++;\n"
"            }\n"
"        }\n"
"    }\n"
*/
/*
        class histogram_shader : public rs2::texture_2d_shader
        {
        public:
            histogram_shader()
                : texture_2d_shader(rs2::shader_program::load(
                    //rs2::texture_2d_shader::default_vertex_shader(),
                    vertex_shader_text,
                    fragment_shader_text, "position", "textureCoords"))
            {
                _width_location = _shader->get_uniform_location("width");
                _height_location = _shader->get_uniform_location("height");
                _texture_sampler_location = _shader->get_uniform_location("textureSampler");

                _shader->begin();
                _shader->load_uniform(_texture_sampler_location, 1);
                _shader->end();
            }

            void set_params(int width, int height, int texture)
            {
                _shader->load_uniform(_width_location, width);
                _shader->load_uniform(_height_location, height);
                //_shader->load_uniform(_texture_sampler_location, 0);
            }

        private:
            uint32_t _width_location;
            uint32_t _height_location;
            uint32_t _texture_sampler_location;
        };
        */

namespace librealsense 
{
    namespace gl
    {
        class align_gl : public align, public gpu_processing_object
        {
        public:
            ~align_gl() override;
            align_gl(rs2_stream to_stream);

        protected:
            void align_z_to_other(rs2::video_frame& aligned, 
                const rs2::video_frame& depth, 
                const rs2::video_stream_profile& other_profile, 
                float z_scale) override;

            void align_other_to_z(rs2::video_frame& aligned, 
                const rs2::video_frame& depth, 
                const rs2::video_frame& other, 
                float z_scale) override;

            void cleanup_gpu_resources() override;
            void create_gpu_resources() override;

            void render(const rs2::points& model, 
                        const rs2::video_frame& tex, 
                        const rs2_intrinsics& camera,
                        const rs2_extrinsics& extr,
                        uint32_t output_texture,
                        uint8_t* ptr = nullptr);

            rs2_extension select_extension(const rs2::frame& input) override;

        private:
            int _enabled = 0;

            std::shared_ptr<rs2::gl::pointcloud> _pc;
            std::shared_ptr<rs2::gl::pointcloud_renderer> _renderer;
            std::shared_ptr<rs2::gl::uploader> _upload;
            std::shared_ptr<rs2::texture_buffer> _other_texture;
            //std::shared_ptr<rs2::visualizer_2d> _viz;
            //std::shared_ptr<histogram_shader> _hist;
            //std::shared_ptr<rs2::vao> _model;
            std::shared_ptr<rs2::texture_visualizer> _viz;
            //std::shared_ptr<rs2::fbo> _fbo;
        };
    }
}

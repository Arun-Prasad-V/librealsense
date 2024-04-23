// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/hpp/rs_sensor.hpp>
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2-gl/rs_processing_gl.hpp>

#include "../proc/synthetic-stream.h"
#include "align-gl.h"
#include "option.h"
#include "rendering.h"

#include "../proc/colorizer.h"
#include "colorizer-gl.h"

#ifndef NOMINMAX
#define NOMINMAX
#endif // NOMINMAX

#include <glad/glad.h>

#include <iostream>

#include <chrono>

#include "synthetic-stream-gl.h"
//#include "/opt/intel/oneapi/vtune/2024.0/sdk/include/ittnotify.h"

//#pragma comment(linker, "/STACK:41943040")

using namespace rs2;
using namespace librealsense::gl;


rs2_extension align_gl::select_extension(const rs2::frame& input)
{
    auto ext = input.is<rs2::depth_frame>() ? RS2_EXTENSION_DEPTH_FRAME_GL : RS2_EXTENSION_VIDEO_FRAME_GL;
    return ext;
}

void align_gl::cleanup_gpu_resources()
{
    _renderer.reset();
    _pc.reset();
    _other_texture.reset();
    _upload.reset();
    _enabled = 0;
    //_hist.reset();
    _viz.reset();
}

void align_gl::create_gpu_resources()
{
    _renderer = std::make_shared<rs2::gl::pointcloud_renderer>();
    _pc = std::make_shared<rs2::gl::pointcloud>();
    _other_texture = std::make_shared<rs2::texture_buffer>();
    _upload = std::make_shared<rs2::gl::uploader>();
    _enabled = glsl_enabled() ? 1 : 0;
    //_viz = std::make_shared<visualizer_2d>(std::make_shared<histogram_shader>());
    //_hist = std::make_shared<histogram_shader>();
    _viz = std::make_shared<rs2::texture_visualizer>();

}

static const char* vertex_shader_text =
"#version 330 \n"
"layout (location = 0) in vec3 aPos;\n"
"layout (location = 1) in vec3 aColor;\n"
"layout (location = 2) in vec2 aTex;\n"
"out vec3 color;\n"
"out vec2 texCoord;\n"
"uniform float scale;\n"
"void main()\n"
"{\n"
"	gl_Position = vec4(aPos.x + aPos.x * scale, aPos.y + aPos.y * scale, aPos.z + aPos.z * scale, 1.0);\n"
"	color = aColor;\n"
"	texCoord = aTex;\n"
"}";

static const char* fragment_shader_text =
"#version 330 \n"
"layout(location = 0) out int FragColor;\n"
"in vec3 color;\n"
"in vec2 texCoord;\n"
"uniform sampler2D tex0;\n"
"uniform float test;\n"
"void main()\n"
"{\n"	
"	int temp = 0;\n"
"	for (int i = 0; i < 1280; i++)\n"
"	{\n"
"	    for (int j = 0; j < 720; j++)\n"
"	    {\n"
"           temp++;\n"
"	    }\n"
"	}\n"
"	FragColor = temp + int(test);\n"
"}";

static const char* merge_vertex_shader_text =
"#version 330 \n"
"layout (location = 0) in vec3 aPos;\n"
"layout (location = 1) in vec3 aColor;\n"
"layout (location = 2) in vec2 aTex;\n"
"out vec3 color;\n"
"out vec2 texCoord;\n"
"uniform float scale;\n"
"void main()\n"
"{\n"
"	gl_Position = vec4(aPos.x + aPos.x * scale, aPos.y + aPos.y * scale, aPos.z + aPos.z * scale, 1.0);\n"
"	color = aColor;\n"
"	texCoord = aTex;\n"
"}";

static const char* merge_fragment_shader_text =
"#version 330 \n"
"layout(location = 0) out int FragColor;\n"
"in vec3 color;\n"
"in vec2 texCoord;\n"
"uniform isampler2D tex0;\n"
"uniform isampler2D tex1;\n"
"uniform isampler2D tex2;\n"
"uniform isampler2D tex3;\n"
"void main()\n"
"{\n"
"   int coord = int(texCoord.y * 256.0f);\n"
"	if (coord < 64)\n"
"	{\n"
"		FragColor = texture(tex0, texCoord).r ;\n"
"	}\n"
"	if (coord >= 64 && coord < 128)\n"
"	{\n"
"		FragColor = texture(tex1, vec2(texCoord.x, texCoord.y - (0.25f))).r ;\n"
"	}\n"
"	if (coord >= 128 && coord < 192)\n"
"	{\n"
"		FragColor = texture(tex2, vec2(texCoord.x, texCoord.y - (0.5f))).r ;\n"
"	}\n"
"	if (coord >= 192)\n"
"	{\n"
"		FragColor = texture(tex3, vec2(texCoord.x, texCoord.y - (0.75f))).r ;\n"
"	}\n"
"}";

// Vertices coordinates
GLfloat vertices[] =
{ //     COORDINATES     /        COLORS      /   TexCoord  //
	-0.5f, -0.5f, 0.0f,     1.0f, 0.0f, 0.0f,	0.0f, 0.0f, // Lower left corner
	-0.5f,  0.5f, 0.0f,     0.0f, 1.0f, 0.0f,	0.0f, 1.0f, // Upper left corner
	 0.5f,  0.5f, 0.0f,     0.0f, 0.0f, 1.0f,	1.0f, 1.0f, // Upper right corner
	 0.5f, -0.5f, 0.0f,     1.0f, 1.0f, 1.0f,	1.0f, 0.0f  // Lower right corner
};

// Indices for vertices order
GLuint indices[] =
{
	0, 2, 1, // Upper triangle
	0, 3, 2 // Lower triangle
};

void compileErrors(unsigned int shader, const char* type)
{
	// Stores status of compilation
	GLint hasCompiled;
	// Character array to store error message in
	char infoLog[1024];
	if (type != "PROGRAM")
	{
		glGetShaderiv(shader, GL_COMPILE_STATUS, &hasCompiled);
		if (hasCompiled == GL_FALSE)
		{
			glGetShaderInfoLog(shader, 1024, NULL, infoLog);
			std::cout << "SHADER_COMPILATION_ERROR for:" << type << "\n" << infoLog << std::endl;
		}
	}
	else
	{
		glGetProgramiv(shader, GL_LINK_STATUS, &hasCompiled);
		if (hasCompiled == GL_FALSE)
		{
			glGetProgramInfoLog(shader, 1024, NULL, infoLog);
			std::cout << "SHADER_LINKING_ERROR for:" << type << "\n" << infoLog << std::endl;
		}
	}
}

void align_gl::align_z_to_other(rs2::video_frame& aligned, 
    const rs2::video_frame& depth, 
    const rs2::video_stream_profile& other_profile, 
    float z_scale)
{ 
    //__itt_domain* domain3 = __itt_domain_create("MyLRSDomain");
    char temp_name[15];

    std::strcpy(temp_name,"LRS DeptoCol"); 
    //__itt_string_handle* task3 = __itt_string_handle_create(temp_name);
    //__itt_task_begin(domain3, __itt_null, __itt_null, task3);

    auto width = aligned.get_width();
    auto height = aligned.get_height();

    _pc->map_to(depth);
    auto p = _pc->calculate(depth);

    auto frame_ref = dynamic_cast<librealsense::depth_frame*>((librealsense::frame_interface*)aligned.get());
    if (!frame_ref)
        throw std::runtime_error("Frame is not depth frame, cannot cast");

    auto gf = dynamic_cast<gpu_addon_interface*>(frame_ref);
    if (!gf)
        throw std::runtime_error("Frame is not gpu_addon_interface, cannot output texture");

    gf->get_gpu_section().set_size(width, height);

    // Set the depth origin of new depth frame to the old depth frame
    auto depth_ptr = dynamic_cast<librealsense::depth_frame*>((librealsense::frame_interface*)depth.get());
    if (!depth_ptr)
        throw std::runtime_error("Frame interface is not depth frame");

    frame_ref->set_sensor(depth_ptr->get_sensor());
    depth_ptr->acquire();
    frame_holder h{ depth_ptr };
    frame_ref->set_original(std::move(h));

    uint32_t aligned_tex;
    auto format = depth.get_profile().format();
    auto tex_type = rs_format_to_gl_format(format);
    gf->get_gpu_section().output_texture(0, &aligned_tex, tex_type.type);
    glTexImage2D(GL_TEXTURE_2D, 0, tex_type.internal_format,
        width, height, 0, tex_type.gl_format, tex_type.data_type, nullptr);

    auto prof = depth.get_profile();
    auto intr = other_profile.get_intrinsics();
    auto extr = prof.get_extrinsics_to(other_profile);

    render(p, depth, intr, extr, aligned_tex);

    //__itt_task_end(domain3);


/*
    int *hist_data;
    hist_data = new int[0xFFFF];

    perform_gl_action([&] {

        std::strcpy(temp_name,"LRS Hist1");
        task3 = __itt_string_handle_create(temp_name);
        __itt_task_begin(domain3, __itt_null, __itt_null, task3);

        __itt_task_end(domain3);

    }, [this] {
        _enabled = false;
    });

    perform_gl_action([&] {
        std::strcpy(temp_name,"LRS Hist2");
        task3 = __itt_string_handle_create(temp_name);
        __itt_task_begin(domain3, __itt_null, __itt_null, task3);

        std::vector<float> fhist = std::vector<float>(librealsense::colorizer::MAX_DEPTH, 0.f);
        float* fhist_data = fhist.data();
        {
            librealsense::gl::colorizer::populate_floating_histogram(
                fhist_data, hist_data);
        }

        __itt_task_end(domain3);

    }, [this] {
        _enabled = false;
    });
*/
/*
    //int temp_data[65536] = {0};
    perform_gl_action([&] {
        std::strcpy(temp_name,"LRS Hist3"); 
        task3 = __itt_string_handle_create(temp_name);
        __itt_task_begin(domain3, __itt_null, __itt_null, task3);

            fbo fbo(256, 256);

            uint32_t hist_texture;
            gf->get_gpu_section().output_texture(1, &hist_texture, TEXTYPE_FLOAT_ASSIST);
            //glGenTextures(1, &hist_texture);
            glBindTexture(GL_TEXTURE_2D, hist_texture);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_R32I, 256, 256, 0, GL_RED_INTEGER, GL_INT, nullptr);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE); // Prevents edge bleeding
	        //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE); // Prevents edge bleeding

            //gf->get_gpu_section().set_size(width, height, true);

            glBindTexture(GL_TEXTURE_2D, 0);

            
            glBindFramebuffer(GL_FRAMEBUFFER, fbo.get());
            //glDrawBuffer(GL_COLOR_ATTACHMENT0);

            //glBindTexture(GL_TEXTURE_2D, hist_texture);

            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, hist_texture, 0);
	        glBindFramebuffer(GL_FRAMEBUFFER, 0);

            
            glClearColor(0, 0, 0, 1);
            glClear(GL_COLOR_BUFFER_BIT);

            //auto& shader = (histogram_shader&)_viz->get_shader();
            auto& shader = _hist;

            shader->begin();
            shader->set_params(width, height, aligned_tex);
            fbo.bind();
            shader->end();

            glActiveTexture(GL_TEXTURE0+1);
            glBindTexture(GL_TEXTURE_2D, aligned_tex);

            glActiveTexture(GL_TEXTURE0);

            _viz->set_position({0.0f, 0.0f});
            _viz->set_scale({1.0f, 1.0f});

            _viz->draw(*_hist, hist_texture);
            //_model->draw();

            glActiveTexture(GL_TEXTURE0);

            //glBindTexture(GL_TEXTURE_2D, hist_texture);
		    //glGetTexImage(GL_TEXTURE_2D, 0, GL_RED_INTEGER, GL_INT, temp_data);


            fbo.unbind();

            glBindTexture(GL_TEXTURE_2D, 0);
            

        __itt_task_end(domain3);
    }, [this] {
        _enabled = false;
    });
*/
    int temp_data[65536] = {0};
    perform_gl_action([&] {
        std::strcpy(temp_name,"LRS Hist3"); 
        //task3 = __itt_string_handle_create(temp_name);
        //__itt_task_begin(domain3, __itt_null, __itt_null, task3);

/*
            uint32_t hist_texture;
            gf->get_gpu_section().output_texture(1, &hist_texture, TEXTYPE_FLOAT_ASSIST);
            glBindTexture(GL_TEXTURE_2D, hist_texture);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_R32I, 256, 256, 0, GL_RED_INTEGER, GL_INT, nullptr);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE); // Prevents edge bleeding
	        //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE); // Prevents edge bleeding

            gf->get_gpu_section().set_size(width, height, true);
*/
	// Create Vertex Shader Object and get its reference
	GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
	// Attach Vertex Shader source to the Vertex Shader Object
	glShaderSource(vertexShader, 1, &vertex_shader_text, NULL);
	// Compile the Vertex Shader into machine code
	glCompileShader(vertexShader);
	// Checks if Shader compiled succesfully
	compileErrors(vertexShader, "VERTEX");

	// Create Fragment Shader Object and get its reference
	GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	// Attach Fragment Shader source to the Fragment Shader Object
	glShaderSource(fragmentShader, 1, &fragment_shader_text, NULL);
	// Compile the Vertex Shader into machine code
	glCompileShader(fragmentShader);
	// Checks if Shader compiled succesfully
	compileErrors(fragmentShader, "FRAGMENT");

	// Create Shader Program Object and get its reference
	GLuint shaderID = glCreateProgram();
	// Attach the Vertex and Fragment Shaders to the Shader Program
	glAttachShader(shaderID, vertexShader);
	glAttachShader(shaderID, fragmentShader);
	// Wrap-up/Link all the shaders together into the Shader Program
	glLinkProgram(shaderID);
	// Checks if Shaders linked succesfully
	compileErrors(shaderID, "PROGRAM");

	// Delete the now useless Vertex and Fragment Shader objects
	glDeleteShader(vertexShader);
	glDeleteShader(fragmentShader);

    // Create Vertex Shader Object and get its reference
	vertexShader = glCreateShader(GL_VERTEX_SHADER);
	// Attach Vertex Shader source to the Vertex Shader Object
	glShaderSource(vertexShader, 1, &merge_vertex_shader_text, NULL);
	// Compile the Vertex Shader into machine code
	glCompileShader(vertexShader);
	// Checks if Shader compiled succesfully
	compileErrors(vertexShader, "VERTEX");

	// Create Fragment Shader Object and get its reference
	fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	// Attach Fragment Shader source to the Fragment Shader Object
	glShaderSource(fragmentShader, 1, &merge_fragment_shader_text, NULL);
	// Compile the Vertex Shader into machine code
	glCompileShader(fragmentShader);
	// Checks if Shader compiled succesfully
	compileErrors(fragmentShader, "FRAGMENT");

	// Create Shader Program Object and get its reference
	GLuint mergeShaderID = glCreateProgram();
	// Attach the Vertex and Fragment Shaders to the Shader Program
	glAttachShader(mergeShaderID, vertexShader);
	glAttachShader(mergeShaderID, fragmentShader);
	// Wrap-up/Link all the shaders together into the Shader Program
	glLinkProgram(mergeShaderID);
	// Checks if Shaders linked succesfully
	compileErrors(mergeShaderID, "PROGRAM");

	// Delete the now useless Vertex and Fragment Shader objects
	glDeleteShader(vertexShader);
	glDeleteShader(fragmentShader);

    GLuint vaoID;
    glGenVertexArrays(1, &vaoID);
    glBindVertexArray(vaoID);

    GLuint vboID;
	glGenBuffers(1, &vboID);
	glBindBuffer(GL_ARRAY_BUFFER, vboID);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    GLuint eboID;
    glGenBuffers(1, &eboID);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eboID);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, vboID);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glBindBuffer(GL_ARRAY_BUFFER, vboID);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
	glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glBindBuffer(GL_ARRAY_BUFFER, vboID);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
	glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    GLuint uniID = glGetUniformLocation(shaderID, "scale");
    GLuint uniID1 = glGetUniformLocation(mergeShaderID, "scale");

    glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, aligned_tex);
    glBindTexture(GL_TEXTURE_2D, 0);

    GLuint texUni = glGetUniformLocation(shaderID, "tex0");
    GLuint testUni = glGetUniformLocation(shaderID, "test");
    GLuint texUni1 = glGetUniformLocation(mergeShaderID, "tex0");
    GLuint texUni2 = glGetUniformLocation(mergeShaderID, "tex1");
    GLuint texUni3 = glGetUniformLocation(mergeShaderID, "tex2");
    GLuint texUni4 = glGetUniformLocation(mergeShaderID, "tex3");

    glUseProgram(shaderID);
    glUniform1i(texUni, 0);

    glUseProgram(mergeShaderID);
    glUniform1i(texUni1, 0);
    glUniform1i(texUni2, 1);
    glUniform1i(texUni3, 2);
    glUniform1i(texUni4, 3);

	GLuint texID;
	// Generates an OpenGL texture object
	glGenTextures(1, &texID);
	//glActiveTexture(GL_TEXTURE0);
	// Assigns the texture to a Texture Unit
	//glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, texID);

	// Configures the type of algorithm that is used to make the image smaller or bigger
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

	// Configures the way the texture repeats (if it does at all)
	glTexImage2D(GL_TEXTURE_2D, 0, GL_R32I, 256, 64, 0, GL_RED_INTEGER, GL_INT, nullptr);
	glBindTexture(GL_TEXTURE_2D, 0);

	unsigned int resultFBO;
	glGenFramebuffers(1, &resultFBO); // frame buffer id
	glBindFramebuffer(GL_FRAMEBUFFER, resultFBO);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texID, 0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

    GLuint texID2;
	// Generates an OpenGL texture object
	glGenTextures(1, &texID2);
	//glActiveTexture(GL_TEXTURE0);
	// Assigns the texture to a Texture Unit
	//glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, texID2);

	// Configures the type of algorithm that is used to make the image smaller or bigger
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

	// Configures the way the texture repeats (if it does at all)
	glTexImage2D(GL_TEXTURE_2D, 0, GL_R32I, 256, 64, 0, GL_RED_INTEGER, GL_INT, nullptr);
	glBindTexture(GL_TEXTURE_2D, 0);

	unsigned int resultFBO2;
	glGenFramebuffers(1, &resultFBO2); // frame buffer id
	glBindFramebuffer(GL_FRAMEBUFFER, resultFBO2);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texID2, 0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	GLuint texID3;
	// Generates an OpenGL texture object
	glGenTextures(1, &texID3);
	//glActiveTexture(GL_TEXTURE0);
	// Assigns the texture to a Texture Unit
	//glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, texID3);

	// Configures the type of algorithm that is used to make the image smaller or bigger
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

	// Configures the way the texture repeats (if it does at all)
	glTexImage2D(GL_TEXTURE_2D, 0, GL_R32I, 256, 64, 0, GL_RED_INTEGER, GL_INT, nullptr);
	glBindTexture(GL_TEXTURE_2D, 0);

	unsigned int resultFBO3;
	glGenFramebuffers(1, &resultFBO3); // frame buffer id
	glBindFramebuffer(GL_FRAMEBUFFER, resultFBO3);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texID3, 0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	GLuint texID4;
	// Generates an OpenGL texture object
	glGenTextures(1, &texID4);
	//glActiveTexture(GL_TEXTURE0);
	// Assigns the texture to a Texture Unit
	//glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, texID4);

	// Configures the type of algorithm that is used to make the image smaller or bigger
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

	// Configures the way the texture repeats (if it does at all)
	glTexImage2D(GL_TEXTURE_2D, 0, GL_R32I, 256, 64, 0, GL_RED_INTEGER, GL_INT, nullptr);
	glBindTexture(GL_TEXTURE_2D, 0);

	unsigned int resultFBO4;
	glGenFramebuffers(1, &resultFBO4); // frame buffer id
	glBindFramebuffer(GL_FRAMEBUFFER, resultFBO4);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texID4, 0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	GLuint texID5;
	// Generates an OpenGL texture object
	glGenTextures(1, &texID5);
	//glActiveTexture(GL_TEXTURE0);
	// Assigns the texture to a Texture Unit
	//glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, texID5);

	// Configures the type of algorithm that is used to make the image smaller or bigger
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

	// Configures the way the texture repeats (if it does at all)
	glTexImage2D(GL_TEXTURE_2D, 0, GL_R32I, 256, 256, 0, GL_RED_INTEGER, GL_INT, nullptr);
	glBindTexture(GL_TEXTURE_2D, 0);

	unsigned int resultFBO5;
	glGenFramebuffers(1, &resultFBO5); // frame buffer id
	glBindFramebuffer(GL_FRAMEBUFFER, resultFBO5);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texID5, 0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

    unsigned int FBO[4] = {resultFBO, resultFBO2, resultFBO3, resultFBO4};

    glActiveTexture(GL_TEXTURE0);

    for (int i = 0; i < 4; i++)
    {
        // Specify the color of the background
        glClearColor(0.07f, 0.13f, 0.17f, 1.0f);
        // Clean the back buffer and assign the new color to it
        glClear(GL_COLOR_BUFFER_BIT);
        // Tell OpenGL which Shader Program we want to use
        glUseProgram(shaderID);
        // Assigns a value to the uniform; NOTE: Must always be done after activating the Shader Program
        glUniform1f(uniID, 1.0f);
        glUniform1f(testUni, (float)i);

        glBindVertexArray(vaoID);

        glBindFramebuffer(GL_FRAMEBUFFER, FBO[i]);

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, aligned_tex);
        
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glBindTexture(GL_TEXTURE_2D, 0);
        //glfwPollEvents();
    }

    // Specify the color of the background
    glClearColor(0.07f, 0.13f, 0.17f, 1.0f);
    // Clean the back buffer and assign the new color to it
    glClear(GL_COLOR_BUFFER_BIT);
    // Tell OpenGL which Shader Program we want to use
    glUseProgram(mergeShaderID);
    // Assigns a value to the uniform; NOTE: Must always be done after activating the Shader Program
    glUniform1f(uniID1, 1.0f);

    glBindVertexArray(vaoID);

    glBindFramebuffer(GL_FRAMEBUFFER, resultFBO5);
    glActiveTexture(GL_TEXTURE3);
    glBindTexture(GL_TEXTURE_2D, texID4);
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, texID3);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, texID2);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texID);

	
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    

    glBindTexture(GL_TEXTURE_2D, texID5);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_RED_INTEGER, GL_INT, temp_data);

    glBindTexture(GL_TEXTURE_2D, 0);

    glDeleteVertexArrays(1, &vaoID);
    glDeleteBuffers(1, &vboID);
    glDeleteBuffers(1, &eboID);
    glDeleteTextures(1, &texID);
    glDeleteTextures(1, &texID2);
    glDeleteTextures(1, &texID3);
    glDeleteTextures(1, &texID4);
    glDeleteTextures(1, &texID5);
    glDeleteProgram(shaderID);
    glDeleteProgram(mergeShaderID);
    glDeleteFramebuffers(1, &resultFBO);
    glDeleteFramebuffers(1, &resultFBO2);
    glDeleteFramebuffers(1, &resultFBO3);
    glDeleteFramebuffers(1, &resultFBO4);
    glDeleteFramebuffers(1, &resultFBO5);

        //__itt_task_end(domain3);
    }, [this] {
        _enabled = false;
    });
                  
    perform_gl_action([&] {
                    for (int x = 0; x < 65536; )
                    {   
                        std::cout << "k: " << x << " value: " << temp_data[x] << std::endl;
                        x++;
                    }
    }, [this] {
        _enabled = false;
    });


/*
    std::strcpy(temp_name,"LRS Upload1"); 
    task3 = __itt_string_handle_create(temp_name);
    __itt_task_begin(domain3, __itt_null, __itt_null, task3);

    //aligned.get_data();
    aligned = _upload->process(aligned);
    __itt_task_end(domain3);

    std::strcpy(temp_name,"LRS Upload2"); 
    task3 = __itt_string_handle_create(temp_name);
    __itt_task_begin(domain3, __itt_null, __itt_null, task3);
    aligned = _upload->process(aligned);

    __itt_task_end(domain3);
    */
}

// From: https://jamesgregson.blogspot.com/2011/11/matching-calibrated-cameras-with-opengl.html
void build_opengl_projection_for_intrinsics(matrix4& frustum, 
    int *viewport, double alpha, double beta, double skew, 
    double u0, double v0, int img_width, int img_height, double near_clip, double far_clip )
{
    
    // These parameters define the final viewport that is rendered into by
    // the camera.
    int L = 0;
    int R = img_width;
    int B = 0;
    int T = img_height;
    
    // near and far clipping planes, these only matter for the mapping from
    // world-space z-coordinate into the depth coordinate for OpenGL
    double N = near_clip;
    double F = far_clip;
    
    // set the viewport parameters
    viewport[0] = L;
    viewport[1] = B;
    viewport[2] = R-L;
    viewport[3] = T-B;
    
    // construct an orthographic matrix which maps from projected
    // coordinates to normalized device coordinates in the range
    // [-1, 1].  OpenGL then maps coordinates in NDC to the current
    // viewport
    matrix4 ortho;
    ortho(0,0) =  2.f/(R-L);        ortho(0,3) = float(-(R+L)/(R-L));
    ortho(1,1) =  2.f/(T-B);        ortho(1,3) = float(-(T+B)/(T-B));
    ortho(2,2) = -2.f/float(F-N);   ortho(2,3) = float(-(F+N)/(F-N));
    ortho(3,3) =  1.f;

    // construct a projection matrix, this is identical to the 
    // projection matrix computed for the intrinsicx, except an
    // additional row is inserted to map the z-coordinate to
    // OpenGL.
    matrix4 tproj;
    tproj(0,0) = float(alpha);  tproj(0,1) = float(skew);  tproj(0,2) = float(u0);
    tproj(1,1) = float(beta);   tproj(1,2) = float(v0);
    tproj(2,2) = float(-(N+F)); tproj(2,3) = float(-N*F);
    tproj(3,2) = 1.f;

    // resulting OpenGL frustum is the product of the orthographic
    // mapping to normalized device coordinates and the augmented
    // camera intrinsic matrix
    frustum = ortho*tproj;
}

void align_gl::render(const rs2::points& model,
    const rs2::video_frame& tex,
    const rs2_intrinsics& intr,
    const rs2_extrinsics& extr,
    uint32_t output_texture,
    uint8_t* ptr)
{
    perform_gl_action([&] {
        auto width = intr.width;
        auto height = intr.height;
        
        uint32_t texture;
        if (auto input_frame = tex.as<rs2::gl::gpu_frame>())
        {
            LOG_DEBUG("Arun:Debug - it is GPU frame");
            texture = input_frame.get_texture_id(0);
        }
        else
        {
            LOG_DEBUG("Arun:Debug - it is non-GPU frame format: " << tex.get_profile().format());
            _other_texture->upload(tex, tex.get_profile().format());
            texture = _other_texture->get_gl_handle();

            if (tex.is<rs2::gl::gpu_frame>())
            {
                LOG_DEBUG("Arun:Debug - it has become a GPU frame");
            }
            else
            {
                LOG_DEBUG("Arun:Debug - it hasn't become a GPU frame");
            }
        }

        fbo fbo(width, height);

        glBindTexture(GL_TEXTURE_2D, output_texture);

        auto format = tex.get_profile().format();
        auto textype = rs_format_to_gl_format(format);
        glTexImage2D(GL_TEXTURE_2D, 0, textype.internal_format, 
            width, height, 0, textype.gl_format, textype.data_type, nullptr);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

        glBindFramebuffer(GL_FRAMEBUFFER, fbo.get());
        glDrawBuffer(GL_COLOR_ATTACHMENT0);

        glBindTexture(GL_TEXTURE_2D, output_texture);
        fbo.createTextureAttachment(output_texture);

        fbo.bind();
        glClearColor(0, 0, 0, 1);
        glClear(GL_COLOR_BUFFER_BIT);

        matrix4 projection;
        int viewport[4];
        build_opengl_projection_for_intrinsics(projection, viewport,
            intr.fx, intr.fy, 0.0, intr.width - intr.ppx, intr.height - intr.ppy, width, height, 0.001, 100);
        projection(3, 2) *= -1.f;
        projection(2, 3) *= -1.f;
        projection(2, 2) *= -1.f;
        float pm[16];
        projection.to_column_major(pm);
        _renderer->set_matrix(RS2_GL_MATRIX_PROJECTION, pm);
        matrix4 view = matrix4::identity();
        view(2, 2) = -1.f;
        _renderer->set_matrix(RS2_GL_MATRIX_CAMERA, (float*)&view.mat);

        matrix4 other = matrix4::identity();
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
                other(i,j) = extr.rotation[i*3 + j];
            other(3, i) = extr.translation[i];
        }
        _renderer->set_matrix(RS2_GL_MATRIX_TRANSFORMATION, (float*)&other.mat);

        glBindTexture(GL_TEXTURE_2D, texture);
        _renderer->process(model);

        fbo.unbind();

        glBindTexture(GL_TEXTURE_2D, 0);
        LOG_DEBUG("Arun:Debug - Render complete");
    }, [&] {
        _enabled = false;
    });
}

void align_gl::align_other_to_z(rs2::video_frame& aligned, 
    const rs2::video_frame& depth, 
    const rs2::video_frame& other, 
    float z_scale)
{
    //__itt_domain* domain3 = __itt_domain_create("MyLRSDomain");
    char temp_name[15];

    std::strcpy(temp_name,"LRS ColtoDep"); 
    //__itt_string_handle* task3 = __itt_string_handle_create(temp_name);
    //__itt_task_begin(domain3, __itt_null, __itt_null, task3);

    auto width = aligned.get_width();
    auto height = aligned.get_height();

    _pc->map_to(other);
    auto p = _pc->calculate(depth);

    auto frame_ref = (frame_interface*)aligned.get();
    auto gf = dynamic_cast<gpu_addon_interface*>(frame_ref);
    if (!gf)
        throw std::runtime_error("Frame interface is not gpu addon interface");

    uint32_t output_rgb;
    auto format = other.get_profile().format();
    auto tex_type = rs_format_to_gl_format(format);

    gf->get_gpu_section().output_texture(0, &output_rgb, tex_type.type);
    glTexImage2D(GL_TEXTURE_2D, 0, tex_type.internal_format,
        width, height, 0, tex_type.gl_format, tex_type.data_type, nullptr);

    gf->get_gpu_section().set_size(width, height);

    auto prof = depth.get_profile();
    auto intr = prof.as<video_stream_profile>().get_intrinsics();
    auto extr = prof.get_extrinsics_to(prof);
    render(p, other, intr, extr, output_rgb);

    //__itt_task_end(domain3);
}

align_gl::align_gl(rs2_stream to_stream) : align(to_stream, "Align (GLSL)")
{
    _source.add_extension<gpu_video_frame>(RS2_EXTENSION_VIDEO_FRAME_GL);
    _source.add_extension<gpu_depth_frame>(RS2_EXTENSION_DEPTH_FRAME_GL);

    auto opt = std::make_shared<librealsense::ptr_option<int>>(
        0, 1, 0, 1, &_enabled, "GLSL enabled"); 
    register_option(RS2_OPTION_COUNT, opt);

    initialize();
}

align_gl::~align_gl()
{
    try
    {
        perform_gl_action( [&]()
        {
            cleanup_gpu_resources();
        }, [] {} );
    }
    catch(...)
    {
        LOG_DEBUG( "Error while cleaning up gpu resources" );
    }
}

//
//  texture.h
//
//  Created by Jonathan Tompson on 6/8/12.
//
//  The underlining loading of textures uses the FreeImage library.

#ifndef RENDERER_TEXURE_HEADER
#define RENDERER_TEXURE_HEADER

#include <mutex>
#include <string>

namespace renderer {

  typedef enum {
    TEXTURE_CLAMP,
    TEXTURE_REPEAT
  } TEXTURE_WRAP_MODE;

  class Texture {
  public:
    // Load texture from file:
    Texture(const std::string& filename, const TEXTURE_WRAP_MODE wrap, 
      bool origin_ul = false);
    // Create texture from internal data structure:
    Texture(const int format_internal, const int w, const int h, 
      const int format, const int type, const unsigned char *bits, 
      const TEXTURE_WRAP_MODE wrap, bool managed);
    ~Texture();

    inline GLuint texture() { return texture_; }
    inline std::string& filename() { return filename_; }
    inline int w() { return w_; }
    inline int h() { return h_; }

    void reloadData(const unsigned char *bits);

    void bind(GLenum target_id,  // ie target_id = GL_TEXTURE0
      GLint texture_sampler_id);  // id in the in the shader program

    // Perform a deep copy and load the copy into OpenGL
    Texture* copy();

  private:
    std::string filename_;
    GLuint texture_;
    int format_internal_; 
    int w_;
    int h_;
    GLenum format_; 
    GLenum type_;
    unsigned char *bits_;
    TEXTURE_WRAP_MODE wrap_;
    bool managed_;
    bool from_file_;
    // void* img_;  // Used to be GLFWimage, now using FreeImage library

    void generateTextureID();
    void loadTextureIntoOpenGL();
    void setTextureProperties();

    // Non-copyable, non-assignable.
    Texture(Texture& other);
    Texture& operator=(const Texture&);
  };
};  // namespace renderer

#endif  // RENDERER_TEXURE_HEADER

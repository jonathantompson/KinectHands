#include <string>
#include <sstream>
#include "renderer/open_gl_common.h"
#include "renderer/texture/texture.h"
#include "exceptions/wruntime_error.h"
#include "string_util/string_util.h"
#include "freeimage.h"
#include "renderer/gl_state.h"

using std::wstring;
using std::wruntime_error;

namespace renderer {

  // Load a texture from disk
  Texture::Texture(const std::string& filename, 
    const TEXTURE_WRAP_MODE wrap, bool origin_ul) {
    unsigned int nbits;

    wrap_ = wrap;
    filename_ = filename;
    managed_ = false;
    from_file_ = true;

    generateTextureID();

    //// OLD CODE USING GLFW --> ONLY SUPPORTS TIF FORMAT
    //GLFWimage* new_img = new GLFWimage();
    //int flags = GLFW_NO_RESCALE_BIT;
    //if (origin_ul) {
    //  flags = flags | GLFW_ORIGIN_UL_BIT;
    //}
    //std::string filename_narrow = string_util::ToNarrowString(filename_);
    //int ret = glfwReadImage(filename_narrow.c_str(), new_img, flags);
    //if (ret != GL_TRUE) {
    //  std::wstringstream ss;
    //  ss << L"Texture::Texture() - ERROR: Failed to load texture from file: ";
    //  ss << filename_ << ".  GLFW error code: " << ret;
    //  throw wruntime_error(ss.str());
    //}

    //format_internal_ = GL_RGBA;
    //w_ = new_img->Width;
    //h_ = new_img->Height;
    //format_ = new_img->Format;
    //type_ = GL_UNSIGNED_BYTE;
    //bits_ = new_img->Data;
    //img_ = reinterpret_cast<GLFWimage*>(new_img);

    // NEW CODE USING THE FREEIMAGE LIBRARY
    FREE_IMAGE_FORMAT fif = FIF_UNKNOWN;  //image format
	  FIBITMAP* dib = NULL;  //pointer to the image, once loaded
	  BYTE* fi_bits = NULL;  //pointer to the image data

    // check the file signature and deduce its format
    fif = FreeImage_GetFileType(filename.c_str(), 0);
	  // if still unknown, try to guess the file format from the file extension
	  if (fif == FIF_UNKNOWN) {
		  fif = FreeImage_GetFIFFromFilename(filename.c_str());
    }
	  // if still unkown, return failure
	  if (fif == FIF_UNKNOWN) {
      throw wruntime_error(wstring(L"Texture() - ERROR: Cannot deduce ") +
        wstring(L"format of the file: ") + string_util::ToWideString(filename));
    }

    // check that FreeImage has reading capabilities and if so load the file
    if (FreeImage_FIFSupportsReading(fif)) {
      dib = FreeImage_Load(fif, filename.c_str());
    }
    //if the image failed to load, return failure
    if (!dib) {
      throw wruntime_error(wstring(L"Texture() - ERROR: FreeImage couldn't ") +
        wstring(L"load the file: ") + string_util::ToWideString(filename));
    }

    // Convert everything to RGBA:
    nbits = FreeImage_GetBPP(dib);
    if (nbits != 32) {
      FIBITMAP* old_dib = dib;
      dib = FreeImage_ConvertTo32Bits(old_dib);
      FreeImage_Unload(old_dib);
    }

    //retrieve the image data
	  fi_bits = FreeImage_GetBits(dib);
	  //get the image width and height
	  w_ = FreeImage_GetWidth(dib);
	  h_ = FreeImage_GetHeight(dib);
	  // if this somehow one of these failed (they shouldn't), return failure
	  if ((fi_bits == 0) || (w_ == 0) || (h_ == 0)) {
      throw wruntime_error(wstring(L"Texture() - ERROR: FreeImage couldn't ") +
        wstring(L"load the file: ") + string_util::ToWideString(filename));
    }

    // Copy it into memory and leave it there in case we need it later.
    bits_ = new unsigned char[w_ * h_ * 4];
    memcpy(bits_, fi_bits, sizeof(bits_[0]) * w_ * h_ * 4);

    format_internal_ = GL_RGBA;
    format_ = GL_BGRA;  // FreeImage has bits flipped!
    type_ = GL_UNSIGNED_BYTE;

    loadTextureIntoOpenGL();

    setTextureProperties();

    // Free FreeImage's copy of the data
	  FreeImage_Unload(dib);
  }

  // Generate a texture from internal data
  // managed = true, transfer ownership of the bits memory
  Texture::Texture(const int format_internal, const int w, const int h, 
    const int format, const int type, const unsigned char *bits, 
    const TEXTURE_WRAP_MODE wrap, 
    bool managed) {

    wrap_ = wrap;
    filename_ = std::string("");
    managed_ = managed;
    from_file_ = false;

    generateTextureID();

    format_internal_ = format_internal;
    w_ = w;
    h_ = h;
    format_ = format;
    type_ = type;
    bits_ = const_cast<unsigned char*>(bits);

    loadTextureIntoOpenGL();

    setTextureProperties();
  }

  Texture::~Texture() {
    GLState::glsDeleteTextures(1, &texture_);
    if (from_file_) {
      // OLD CODE USING GLFW
      // glfwFreeImage(reinterpret_cast<GLFWimage*>(img_));
      // delete reinterpret_cast<GLFWimage*>(img_);
      delete[] bits_;
      bits_ = NULL;
    } else if (managed_ && bits_ != NULL) {
      delete[] bits_;
      bits_ = NULL;
    }

  }

  void Texture::reloadData(const unsigned char* bits) {
    // Delete the stuff that exists there now (if we need to)
    if (from_file_) {
      delete[] bits_;
      bits_ = NULL;
    } else if (managed_ && bits_ != NULL) {
      delete[] bits_;
      bits_ = NULL;
    }
    // Reload the data
    bits_ = const_cast<unsigned char*>(bits);
    GLState::glsBindTexture(GL_TEXTURE_2D, texture_);
    loadTextureIntoOpenGL();
  }

  void Texture::generateTextureID() {
    // Generate the openGL texture ID
    GLState::glsGenTextures(1, &texture_);
    GLState::glsBindTexture(GL_TEXTURE_2D, 0);
    GLState::glsBindTexture(GL_TEXTURE_2D, texture_);
  }

  void Texture::loadTextureIntoOpenGL() {
    GLState::glsTexImage2D(GL_TEXTURE_2D, 0, format_internal_ , w_, h_, 0, format_, 
      type_, bits_);
  }

  void Texture::setTextureProperties() {
    // Assumes the current texture is already bound
    GLState::glsTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    GLState::glsTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);

    switch (wrap_) {
    case TEXTURE_CLAMP:
      GLState::glsTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
      GLState::glsTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
      break;
    case TEXTURE_REPEAT:
      GLState::glsTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
      GLState::glsTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
      break;
    default:
      throw wruntime_error(wstring(L"Texture::setTextureProperties() - ") +
        wstring(L"ERROR: Unrecognized texture wrap mode"));
      break;
    }
  }

  // bind() - typical usage for single texture render target: 
  //          bind(0, GL_TEXTURE0, tex_sampler_id)
  void Texture::bind(GLenum target_id, GLint h_texture_sampler) {
    GLState::glsActiveTexture(target_id);
    GLState::glsBindTexture(GL_TEXTURE_2D, texture_);
    GLint uniform_val = (target_id - GL_TEXTURE0);
    GLState::UniformFuncs::glsUniform1iv(h_texture_sampler, 1, &uniform_val);
  }

  Texture::Texture(Texture& other) {
    generateTextureID();

    format_internal_ = other.format_internal_;
    w_ = other.w_;
    h_ = other.h_;
    format_ = other.format_;
    type_ = other.type_;
    wrap_ = other.wrap_;
    managed_ = other.managed_;
    from_file_ = other.from_file_;

    if (from_file_ || managed_) {
      bits_ = new unsigned char[w_ * h_ * 4];
      memcpy(bits_, other.bits_, sizeof(bits_[0]) * w_ * h_ * 4);
    } else {
      bits_ = other.bits_;
    }

    loadTextureIntoOpenGL();
    setTextureProperties();
  }

  Texture* Texture::copy() {
    Texture* ret = new Texture(*this);
    return ret;
  }

}  // namespace renderer

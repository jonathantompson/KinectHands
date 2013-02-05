#include <cstring>
#include <string>
#include <iostream>
#include <sstream>
#include "windowing/window.h"
#include "exceptions/wruntime_error.h"

using std::wstring;
using std::wruntime_error;
using math::Int2;

namespace windowing {

  // Internal static member variables.
  int Window::mouse_wheel_pos_ = 0;
  bool Window::key_down_[NUM_KEYS];
  bool Window::mouse_button_down_[NUM_MOUSE_BUTTONS];
  math::Int2 Window::mouse_pos_;
  bool Window::init_glew_ = true;
  uint32_t Window::num_windows_ = 0;
  std::mutex Window::window_init_lock_;
  Window* Window::g_window_ = NULL;

  Window::~Window() {
    window_init_lock_.lock();
    glfwSetWindowSizeCallback(NULL);
    glfwSetKeyCallback(NULL);
    glfwSetCharCallback(NULL);
    glfwSetMousePosCallback(NULL);
    glfwSetMouseButtonCallback(NULL);
    glfwSetMouseWheelCallback(NULL);
    g_window_ = NULL;
    if (isOpen()) {
      glfwCloseWindow();
    }
    num_windows_--;
    window_init_lock_.unlock();
  } 

  Window::Window(WindowSettings& settings) {
    window_init_lock_.lock();
    if (num_windows_ != 0) {
      window_init_lock_.unlock();
      throw wruntime_error(L"Window::Window() - ERROR: A window is open.");
    }
    
    settings_ = settings;
    keyboard_cb_ = NULL;
    mouse_pos_cb_ = NULL;
    mouse_button_cb_ = NULL;
    mouse_wheel_cb_ = NULL;
    character_input_cb_ = NULL;

    // Set the open hits for GLFW
    glfwOpenWindowHint(GLFW_FSAA_SAMPLES, settings.fsaa_samples);
    glfwOpenWindowHint(GLFW_OPENGL_VERSION_MAJOR, settings.gl_major_version);
    glfwOpenWindowHint(GLFW_OPENGL_VERSION_MINOR, settings.gl_minor_version);
    if (settings.gl_major_version >= 3) {
      glfwOpenWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
      glfwOpenWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    }

    // Prevent resize events
    glfwOpenWindowHint(GLFW_WINDOW_NO_RESIZE, GL_TRUE);

    // Open the GLFW window
    if (settings.fullscreen) {
      int ret_val = glfwOpenWindow(settings.width, settings.height,
        settings.num_rgba_bits, settings.num_rgba_bits, settings.num_rgba_bits,
        0, settings.num_depth_bits, settings.num_stencil_bits,
        GLFW_FULLSCREEN);
      if (ret_val != GL_TRUE) {
          window_init_lock_.unlock();
          std::wstringstream wss;
          wss << L"ERROR: glfwOpenWindow returned an error! code = ";
          wss << ret_val;
          throw std::wruntime_error(wss.str());
      }
    } else {
      GLFWvidmode mode;
      glfwGetDesktopMode(&mode);
      int ret_val = glfwOpenWindow(settings.width, settings.height, mode.RedBits,
        mode.GreenBits, mode.BlueBits, 0, settings.num_depth_bits,
        settings.num_stencil_bits, GLFW_WINDOW);
      if (ret_val != GL_TRUE) {
          window_init_lock_.unlock();
          std::wstringstream wss;
          wss << L"ERROR: glfwOpenWindow returned an error! code = ";
          wss << ret_val;
          throw std::wruntime_error(wss.str());
      }
    }
    if (settings.fullscreen) {
      glfwEnable(GLFW_MOUSE_CURSOR);
    }
    glfwSetWindowTitle(settings.title.c_str());

    // Callbacks must be set after the window is created
    glfwSetWindowSizeCallback(NULL);
    glfwSetKeyCallback((GLFWkeyfun)&keyboardInputCB);
    glfwSetCharCallback((GLFWcharfun)&characterInput);
    glfwSetMousePosCallback((GLFWmouseposfun)&mousePosCB);
    glfwSetMouseButtonCallback((GLFWmousebuttonfun)&mouseButtonCB);
    glfwSetMouseWheelCallback((GLFWmousewheelfun)&mouseWheelCB);

    // Double check to make sure we were successful in creating a new OpenGL
    // context of the requested version
    if (settings.gl_major_version >= 3) {
      int major_window;
      int minor_window;
      int rev_window;
      glfwGetGLVersion(&major_window, &minor_window, &rev_window);
      if ( major_window != settings.gl_major_version ||
           minor_window != settings.gl_minor_version ) {
        window_init_lock_.unlock();
        throw wruntime_error(L"ERROR: OpenGL context isnt the requested Vers.");
      }
    }

    // Now we need to initialize all the GL3 function pointers now that a valid
    // context has been built.
    // Note: there is a bug in GLEW 1.7, and to get around it we need to set
    // glewExperimental: http://www.opengl.org/wiki/OpenGL_Loading_Library
    if (init_glew_) {  // Only init glew once!
#ifdef _WIN32
      glewExperimental = TRUE;
#else
      glewExperimental = true;
#endif
      GLenum ret = glewInit();
      if (ret != GLEW_OK) {
        fprintf(stderr, "Error: %s\n", glewGetErrorString(ret));
        window_init_lock_.unlock();
        throw wruntime_error(L"ERROR: glewInit() returned an error");
      }
      // Unfortuantely, as mentioned above GLEW causes some spurious gl errors on
      // startup, we should flush those so that error checking later is valid.
      while (glGetError() != GL_NO_ERROR) { }

      init_glew_ = false;
    }

    g_window_ = this;
    num_windows_++;
    window_init_lock_.unlock();
  }

  void Window::swapBackBuffer() {
    // swap back and front buffers
    glfwSwapBuffers();
  }

  bool Window::getKeyState(int key) {
    return key_down_[key];
  }

  // Return true if the mouse is within the window
  bool Window::getMousePosition(math::Int2* pos) {
    pos->set(&mouse_pos_);
    return pos->m[0] >= 0 && pos->m[0] < settings_.width &&
           pos->m[1] >= 0 && pos->m[1] < settings_.height;
  }

  bool Window::getMouseButtonStateRight() {
    return mouse_button_down_[static_cast<int>(GLFW_MOUSE_BUTTON_RIGHT)];
  }  
  
  bool Window::getMouseButtonStateLeft() {
    return mouse_button_down_[static_cast<int>(GLFW_MOUSE_BUTTON_LEFT)];
  }    
  
  bool Window::getMouseButtonStateMiddle() {
    return mouse_button_down_[static_cast<int>(GLFW_MOUSE_BUTTON_MIDDLE)];
  }   

  void Window::initWindowSystem() {
    memset(key_down_, 0, NUM_KEYS*sizeof(key_down_[0]));
    memset(mouse_button_down_, 0, 
           NUM_MOUSE_BUTTONS*sizeof(mouse_button_down_[0]));

    if (glfwInit() != GL_TRUE) {
      throw wruntime_error(L"ERROR: glfwInit() returned an error");
    }
  }

  void GLFWCALL Window::keyboardInputCB(int key, int action)  {
    key_down_[key] = (action == PRESSED) ? true : false;
    if (g_window_ && g_window_->keyboard_cb_) {
      g_window_->keyboard_cb_(key, action);
    }
  }

  void GLFWCALL Window::mousePosCB(int x, int y)  {
    mouse_pos_[0] = x;
    mouse_pos_[1] = y;
    
    if (g_window_ && g_window_->mouse_pos_cb_) {
      g_window_->mouse_pos_cb_(x, y);
    }
  }

  void GLFWCALL Window::mouseButtonCB(int button, int action)  {
    mouse_button_down_[button] = (action == PRESSED) ? true : false;
    if (g_window_ && g_window_->mouse_button_cb_) {
      g_window_->mouse_button_cb_(button, action);
    }
  }

  void GLFWCALL Window::mouseWheelCB(int pos)  {
    mouse_wheel_pos_ = pos;
    if (g_window_ && g_window_->mouse_wheel_cb_) {
      g_window_->mouse_wheel_cb_(pos);
    }
  }

  void GLFWCALL Window::characterInput(int character, int action) {
    if (g_window_ && g_window_->character_input_cb_) {
      g_window_->character_input_cb_(character, action);
    }
  }

  void Window::killWindowSystem() {
    glfwTerminate();
  }

  bool Window::isOpen() {
    return glfwGetWindowParam(GLFW_OPENED) != 0;
  }

  void Window::windowResEnumToInt(int res, int& w, int& h) {
    WINDOW_RES en = static_cast<WINDOW_RES>(res);
    switch (en) {
    case RES_640_480:
      w = 640;
      h = 480;
      break;
    case RES_800_600:
      w = 800;
      h = 600;
      break;
    case RES_1280_1024:
      w = 1280;
      h = 1024;
      break;
    case RES_1920_1200:
      w = 1920;
      h = 1200;
      break;
    case RES_1280_720:
      w = 1280;
      h = 720;
      break;
    case RES_1920_1080:
      w = 1920;
      h = 1080;
      break;
    default:
      throw wruntime_error(wstring(L"Window::windowResolutionEnumToInt() - ") +
        wstring(L"undefined resolution enum"));
    }
  }
  
  void Window::registerKeyboardCB(KeyboardCBFuncPtr callback) {
    keyboard_cb_ = callback;
  }
  
  void Window::registerMousePosCB(MousePosCBFuncPtr callback) {
    mouse_pos_cb_ = callback;
  }
  
  void Window::registerMouseButtonCB(MouseButtonCBFuncPtr callback) {
    mouse_button_cb_ = callback;
  }
  
  void Window::registerMouseWheelCB(MouseWheelCBFuncPtr callback) {
    mouse_wheel_cb_ = callback;
  }
  
  void Window::registerCharacterInputCB(CharacterInputCBFuncPtr callback) {
    character_input_cb_ = callback;
  }

}  // namespace windowing

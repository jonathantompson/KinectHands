//
//  window.h
//
//  Created by Jonathan Tompson on 4/26/12.
//
//  A wrapper for windowing using GLFW.  Even though GLFW is cross platform, 
//  this layer is added as an abstraction in case I want to switch windowing
//  libraries and it also keeps App free of implementation specific API calls.
//
//  Note, a limitation of this class is that only one window can be open at any
//  one time.  Also, dynamic window resizing is not supported.
//
//  Typical Useage:
//
//  void keyboardFunc(int key, int action);
//  void mousePosFunc(int x, int y);
//  void mouseButtonFunc(int button, int action);
//  void mouseWheelFunc(int pos);
//  void CharacterInputFunc(int character, int action);
//
//  void main {
//    Window::initWindowSystem();
//    WindowSettings settings;
//    settings.width = xxx;
//    ... etc ...
//    Window* wnd = new Window(settings);
//    wnd->registerKeyboardCB(&keyboardFunc);
//    wnd->registerMousePosCB(&mousePosFunc);
//    ... do stuff ...
//    delete wnd;
//    Window::killWindowSystem();
//  }

#ifndef WINDOWING_WINDOW_HEADER
#define WINDOWING_WINDOW_HEADER

#include <mutex>
#include <string>
#include "windowing/glfw.h"
#include "math/math_types.h"  // for Int4
#include "windowing/keys_and_buttons.h"
#include "windowing/window_settings.h"

#define MESSAGE_LOOP_POLL_TIME_MS 50

namespace windowing {

  typedef enum {
    RES_640_480,
    RES_800_600,
    RES_1280_1024,
    RES_1920_1200,
    RES_1280_720,
    RES_1920_1080,
    NUM_RES,
  } WINDOW_RES;

  typedef void (*KeyboardCBFuncPtr)(int key, int action);
  typedef void (*MousePosCBFuncPtr)(int x, int y);
  typedef void (*MouseButtonCBFuncPtr)(int button, int action);
  typedef void (*MouseWheelCBFuncPtr)(int pos);
  typedef void (*CharacterInputCBFuncPtr)(int character, int action);

  class Window {
  public:
    Window(WindowSettings& settings);
    ~Window();

    static void initWindowSystem();
    static void killWindowSystem();
    void swapBackBuffer();  // Makes any API calls necessary to finish rendering

    // Poll the mouse and keyboard state on demand
    bool getKeyState(int key);
    bool getMousePosition(math::Int2* pos);
    bool getMouseButtonStateRight();
    bool getMouseButtonStateLeft();
    bool getMouseButtonStateMiddle();
    
    // Some getter methods
    inline int width() { return settings_.width; }
    inline int height() { return settings_.height; }
    inline bool fullscreen() { return settings_.fullscreen; }
    bool isOpen();

    // A helper function to turn enumerated resolutions into width and height
    static void windowResEnumToInt(int res, int& w, int& h);
    
    // Add callback functions to get imediate updates on a mouse or key event
    void registerKeyboardCB(KeyboardCBFuncPtr callback);
    void registerMousePosCB(MousePosCBFuncPtr callback);
    void registerMouseButtonCB(MouseButtonCBFuncPtr callback);
    void registerMouseWheelCB(MouseWheelCBFuncPtr callback);
    void registerCharacterInputCB(CharacterInputCBFuncPtr callback);

  private:
    WindowSettings settings_;
    KeyboardCBFuncPtr keyboard_cb_;
    MousePosCBFuncPtr mouse_pos_cb_;
    MouseButtonCBFuncPtr mouse_button_cb_;
    MouseWheelCBFuncPtr mouse_wheel_cb_;
    CharacterInputCBFuncPtr character_input_cb_;
    
    static int mouse_wheel_pos_;
    static bool key_down_[NUM_KEYS];
    static bool mouse_button_down_[NUM_MOUSE_BUTTONS];
    static math::Int2 mouse_pos_;
    static bool init_glew_;
    static std::mutex window_init_lock_;
    static uint32_t num_windows_;
    static Window* g_window_;  // For internal use only.  This is a singleton!
    
    // Callbacks for GLFW --> Will also send keyboard events to the app class
    static void GLFWCALL keyboardInputCB(int key, int action);
    static void GLFWCALL mousePosCB(int x, int y);
    static void GLFWCALL mouseButtonCB(int button, int action);
    static void GLFWCALL mouseWheelCB(int pos);
    static void GLFWCALL characterInput(int character, int action);
    
    // Non-copyable, non-assignable.
    Window(Window&);
    Window& operator=(const Window&);
  };

  void NativeErrorBox(const wchar_t* str);

};  // namespace windowing

#endif  // WINDOWING_WINDOW_HEADER


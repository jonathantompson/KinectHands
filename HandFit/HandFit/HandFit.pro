TEMPLATE = app
CONFIG += console
CONFIG -= qt
CONFIG += static

QMAKE_CC = g++-4.7
QMAKE_CXX = g++-4.7
QMAKE_CXXFLAGS += -std=c++0x -static -pthread -static-libgcc -static-libstdc++ -D_GLIBCXX_USE_NANOSLEEP
QMAKE_CFLAGS += -std=c++0x -static -pthread -static-libgcc -static-libstdc++ -D_GLIBCXX_USE_NANOSLEEP
QMAKE_CFLAGS_RELEASE	= -O3 -msse -msse2
QMAKE_CXXFLAGS_RELEASE	= -O3 -msse -msse2
LIBS += -L../glew-1.9.0/lib -lGLEW
LIBS += -L../glfw-2.7.6/lib/x11 -lglfw
LIBS += -lGLU
LIBS += -Wl,--whole-archive -lpthread
LIBS += -Wl,--no-whole-archive
LIBS += -lrt

INCLUDEPATH += ../src/
INCLUDEPATH += ../eigen-3.1.1/Eigen/
INCLUDEPATH += ../glew-1.9.0/include
INCLUDEPATH += ../glfw-2.7.6/include

SOURCES += \
    ../src/data_str/hash_funcs.cpp \
    ../src/exceptions/wruntime_error.cpp \
    ../src/file_io/file_io.cpp \
    ../src/hand_model/hand_model_renderer.cpp \
    ../src/hand_model/hand_model_fit.cpp \
    ../src/hand_model/hand_model.cpp \
    ../src/main/main.cpp \
    ../src/math/pso_fitting.cpp \
    ../src/math/nm_fitting.cpp \
    ../src/math/math_base.cpp \
    ../src/math/lprpso_fitting.cpp \
    ../src/math/lm_fitting.cpp \
    ../src/math/de_fitting.cpp \
    ../src/math/common_fitting.cpp \
    ../src/renderer/renderer.cpp \
    ../src/renderer/open_gl_common.cpp \
    ../src/renderer/colors.cpp \
    ../src/renderer/camera/camera.cpp \
    ../src/renderer/geometry/geometry_vertices.cpp \
    ../src/renderer/geometry/geometry_points.cpp \
    ../src/renderer/geometry/geometry_colored_points.cpp \
    ../src/renderer/geometry/geometry_colored_mesh.cpp \
    ../src/renderer/geometry/geometry.cpp \
    ../src/renderer/lights/light_dir_handles.cpp \
    ../src/renderer/lights/light_dir.cpp \
    ../src/renderer/lights/light.cpp \
    ../src/renderer/objects/aabbox.cpp \
    ../src/renderer/shader/shader_program.cpp \
    ../src/renderer/shader/shader.cpp \
    ../src/renderer/texture/texture_renderable.cpp \
    ../src/renderer/texture/texture.cpp \
    ../src/string_util/string_util.cpp \
    ../src/threading/thread_pool.cpp \
    ../src/threading/thread.cpp \
    ../src/windowing/window_settings.cpp \
    ../src/windowing/window.cpp \
    ../src/depth_images_io.cpp \
    ../src/fastlz/fastlz.c \
    ../src/kinect_interface/open_ni_funcs.cpp

HEADERS += \
    ../src/alignment/data_align.h \
    ../src/clock/clock.h \
    ../src/data_str/vector_managed.h \
    ../src/data_str/vector.h \
    ../src/data_str/pair.h \
    ../src/data_str/min_heap.h \
    ../src/data_str/hash_set.h \
    ../src/data_str/hash_map_managed.h \
    ../src/data_str/hash_map.h \
    ../src/data_str/hash_funcs.h \
    ../src/data_str/circular_buffer.h \
    ../src/exceptions/wruntime_error.h \
    ../src/file_io/file_io.h \
    ../src/hand_model/hand_model_renderer.h \
    ../src/hand_model/hand_model_fit.h \
    ../src/hand_model/hand_model.h \
    ../src/math/vec4.h \
    ../src/math/vec3.h \
    ../src/math/vec2.h \
    ../src/math/quat.h \
    ../src/math/pso_fitting.h \
    ../src/math/plane.h \
    ../src/math/nm_fitting.h \
    ../src/math/math_types.h \
    ../src/math/math_base.h \
    ../src/math/mat4x4.h \
    ../src/math/mat3x3.h \
    ../src/math/mat2x2.h \
    ../src/math/lprpso_fitting.h \
    ../src/math/lm_fitting.h \
    ../src/math/de_fitting.h \
    ../src/math/common_fitting.h \
    ../src/renderer/renderer.h \
    ../src/renderer/open_gl_common.h \
    ../src/renderer/colors.h \
    ../src/renderer/camera/camera.h \
    ../src/renderer/geometry/geometry_vertices.h \
    ../src/renderer/geometry/geometry_points.h \
    ../src/renderer/geometry/geometry_colored_points.h \
    ../src/renderer/geometry/geometry_colored_mesh.h \
    ../src/renderer/geometry/geometry.h \
    ../src/renderer/lights/light_dir_handles.h \
    ../src/renderer/lights/light_dir.h \
    ../src/renderer/lights/light.h \
    ../src/renderer/objects/aabbox.h \
    ../src/renderer/shader/shader_program.h \
    ../src/renderer/shader/shader_location_name_pair.h \
    ../src/renderer/shader/shader.h \
    ../src/renderer/texture/texture_renderable.h \
    ../src/renderer/texture/texture.h \
    ../src/string_util/string_util.h \
    ../src/threading/thread_pool.h \
    ../src/threading/thread.h \
    ../src/threading/callback_queue_item.h \
    ../src/threading/callback_queue.h \
    ../src/threading/callback_instances.h \
    ../src/threading/callback.h \
    ../src/windowing/window_settings.h \
    ../src/windowing/window.h \
    ../src/windowing/keys_and_buttons.h \
    ../src/windowing/glfw.h \
    ../src/depth_images_io.h \
    ../src/image_data.h \
    ../src/fastlz/fastlz.h \
    ../src/kinect_interface/open_ni_funcs.h


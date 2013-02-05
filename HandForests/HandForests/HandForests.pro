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
LIBS += -static
LIBS += -Wl,--whole-archive -lpthread
LIBS += -Wl,--no-whole-archive

INCLUDEPATH += ../src/
INCLUDEPATH += ../../src/


SOURCES += \
    ../src/main.cpp \
    ../src/generate_decision_tree.cpp \
    ../src/forest_io.cpp \
    ../src/evaluate_decision_forest.cpp \
    ../src/depth_images_io.cpp \
    ../src/common_tree_funcs.cpp \
    ../src/debug_util/debug_util_macosx.cpp \
    ../src/threading/thread_pool.cpp \
    ../src/threading/thread.cpp \
    ../../src/fastlz/fastlz.c \
    ../../src/string_util/string_util.cpp \
    ../../src/exceptions/wruntime_error.cpp \
    ../src/file_io/csv_handle_read.cpp \
    ../src/file_io/csv_handle.cpp \
    ../src/load_settings_from_file.cpp \
    ../../src/kinect_interface/open_ni_funcs.cpp

HEADERS += \
    ../src/image_util.h \
    ../src/generate_decision_tree.h \
    ../src/forest_io.h \
    ../src/evaluate_decision_forest.h \
    ../src/depth_images_io.h \
    ../src/decision_tree_func.h \
    ../src/common_tree_funcs.h \
    ../src/debug_util/debug_util.h \
    ../src/threading/thread_pool.h \
    ../src/threading/thread.h \
    ../src/threading/callback_queue_item.h \
    ../src/threading/callback_queue.h \
    ../src/threading/callback_instances.h \
    ../src/threading/callback.h \
    ../../src/fastlz/fastlz.h \
    ../../src/math/vec4.h \
    ../../src/math/vec3.h \
    ../../src/math/vec2.h \
    ../../src/math/math_types.h \
    ../../src/math/mat4x4.h \
    ../../src/math/mat3x3.h \
    ../../src/math/mat2x2.h \
    ../../src/string_util/string_util.h \
    ../../src/exceptions/wruntime_error.h \
    ../src/data_str/vector_managed.h \
    ../src/file_io/csv_handle_read.h \
    ../src/file_io/csv_handle.h \
    ../src/load_settings_from_file.h \
    ../../src/kinect_interface/open_ni_funcs.h

OTHER_FILES += \
    ../src/threading/callback_instances.py


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
LIBS += -Wl,--whole-archive -lpthread -lrt
LIBS += -Wl,--no-whole-archive

INCLUDEPATH += ../src/
INCLUDEPATH += ../../kinect_interface/include
INCLUDEPATH += ../../../PRenderer2/jtil/include



SOURCES += \
    ../src/main.cpp \
    ../../kinect_interface/src/kinect_interface/hand_detector/generate_decision_tree.cpp \
    ../../kinect_interface/src/kinect_interface/hand_detector/forest_io.cpp \
    ../../kinect_interface/src/kinect_interface/hand_detector/evaluate_decision_forest.cpp \
    ../../kinect_interface/src/kinect_interface/depth_images_io.cpp \
    ../../kinect_interface/src/kinect_interface/hand_detector/common_tree_funcs.cpp \
    ../../../PRenderer2/jtil/src/jtil/debug_util/debug_util_macosx.cpp \
    ../../../PRenderer2/jtil/src/jtil/threading/thread_pool.cpp \
    ../../../PRenderer2/jtil/src/jtil/threading/thread.cpp \
    ../../../PRenderer2/jtil/src/jtil/fastlz/fastlz.c \
    ../../../PRenderer2/jtil/src/jtil/string_util/string_util.cpp \
    ../../../PRenderer2/jtil/src/jtil/exceptions/wruntime_error.cpp \
    ../../../PRenderer2/jtil/src/jtil/file_io/csv_handle_read.cpp \
    ../../../PRenderer2/jtil/src/jtil/file_io/csv_handle.cpp \
    ../src/load_settings_from_file.cpp \
    ../../kinect_interface/src/kinect_interface/open_ni_funcs.cpp

HEADERS += \
    ../../../PRenderer2/jtil/include/jtil/image_util/image_util.h \
    ../../kinect_interface/include/kinect_interface/hand_detector/generate_decision_tree.h \
    ../../kinect_interface/include/kinect_interface/hand_detector/forest_io.h \
    ../../kinect_interface/include/kinect_interface/hand_detector/evaluate_decision_forest.h \
    ../../kinect_interface/include/kinect_interface/depth_images_io.h \
    ../../kinect_interface/include/kinect_interface/hand_detector/decision_tree_func.h \
    ../../kinect_interface/include/kinect_interface/hand_detector/common_tree_funcs.h \
    ../../../PRenderer2/jtil/include/jtil/debug_util/debug_util.h \
    ../../../PRenderer2/jtil/include/jtil/threading/thread_pool.h \
    ../../../PRenderer2/jtil/include/jtil/threading/thread.h \
    ../../../PRenderer2/jtil/include/jtil/threading/callback_queue_item.h \
    ../../../PRenderer2/jtil/include/jtil/threading/callback_queue.h \
    ../../../PRenderer2/jtil/include/jtil/threading/callback_instances.h \
    ../../../PRenderer2/jtil/include/jtil/threading/callback.h \
    ../../../PRenderer2/jtil/include/jtil/fastlz/fastlz.h \
    ../../../PRenderer2/jtil/include/jtil/math/vec4.h \
    ../../../PRenderer2/jtil/include/jtil/math/vec3.h \
    ../../../PRenderer2/jtil/include/jtil/math/vec2.h \
    ../../../PRenderer2/jtil/include/jtil/math/math_types.h \
    ../../../PRenderer2/jtil/include/jtil/math/mat4x4.h \
    ../../../PRenderer2/jtil/include/jtil/math/mat3x3.h \
    ../../../PRenderer2/jtil/include/jtil/math/mat2x2.h \
    ../../../PRenderer2/jtil/include/jtil/string_util/string_util.h \
    ../../../PRenderer2/jtil/include/jtil/exceptions/wruntime_error.h \
    ../../../PRenderer2/jtil/include/jtil/data_str/vector_managed.h \
    ../../../PRenderer2/jtil/include/jtil/file_io/csv_handle_read.h \
    ../../../PRenderer2/jtil/include/jtil/file_io/csv_handle.h \
    ../../../PRenderer2/jtil/include/jtil/clk/clk.h \
    ../src/load_settings_from_file.h \
    ../../kinect_interface/include/kinect_interface/open_ni_funcs.h

OTHER_FILES += \
    ../../../PRenderer2/jtil/include/jtil/threading/callback_instances.py \
    ../../hand_forests_settings.csv

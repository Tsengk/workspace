cmake_minimum_required(VERSION 3.5)

project(ZKmqttApllication VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
#set(CMAKE_PREFIX_PATH C:/Qt/Qt5.12.12/5.12.12/msvc2015_64)
#set(CMAKE_BUILD_TYPE Debug)
# 设置C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(/bigobj)

# 在任何其他设置之前设置ABI版本
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_GLIBCXX_USE_CXX11_ABI=0")
#add_definitions("-std=c++11")
# msgpack和ABI兼容性设置
add_definitions(
    -DMSGPACK_DISABLE_LEGACY_NIL
    -DMSGPACK_DISABLE_LEGACY_CONVERT
    -DMSGPACK_NO_BOOST
    -DMSGPACK_ARCH_NATIVE_ENDIAN
    -DMSGPACK_SBUFFER_INC_SIZE=4096
    -DMSGPACK_DEFAULT_API_VERSION=1
    -DMSGPACK_API_VERSION=1
    -DMSGPACK_ENDIAN_LITTLE_BYTE
    -DMSGPACK_DISABLE_LEGACY_NAME_AS_FLOAT
    -DREST_RPC_ENABLE_MSGPACK_COMPAT
    -DMSGPACK_USE_DEFINE_ARRAY
)

# 添加预处理器定义来处理兼容性问题
if(MSVC)
    add_definitions(
        -D_SILENCE_CXX17_ITERATOR_BASE_CLASS_DEPRECATION_WARNING
        -D_HAS_STD_BYTE=0
        -DREST_RPC_DISABLE_STRING_VIEW
        -DUSE_STD_STRING_AS_STRING_VIEW
        -DBOOST_NO_CXX11_ALLOCATOR
        -DBOOST_NO_CXX11_SMART_PTR
        -DASIO_DISABLE_STD_STRING_VIEW
        -DASIO_DISABLE_STD_EXPERIMENTAL_STRING_VIEW
        -D_SCL_SECURE_NO_WARNINGS
        -D_CRT_SECURE_NO_WARNINGS
    )

    # 为MSVC添加特定的C++11编译选项
    add_compile_options(
        /EHsc     # 启用C++异常处理
        /MP       # 多处理器编译
        /Zc:wchar_t # 将wchar_t视为内置类型
        /GR-      # 禁用RTTI
        /utf-8    # UTF-8编码支持
        /wd4996   # 禁用过时函数警告
        /wd4828   # 禁用UTF-8编码字符警告
       # /-execution-charset:utf-8 # gcc -finput-charset=UTF-8
    )

    # 设置特定的运行时库链接
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
     #   set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MD")
    else()
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
    endif()
endif()

# 禁用msgpack的所有C++17特性
add_definitions(
    -DMSGPACK_NO_CXX17
    -DMSGPACK_NO_BOOST
)
add_definitions(-DQT_QML_DEBUG)
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS WebEngineWidgets Widgets)

set(PROJECT_PATH ${CMAKE_CURRENT_LIST_DIR})

# 包含所有必要的头文件目录
include_directories(${PROJECT_PATH}/3drparty)
include_directories(${PROJECT_PATH}/3drparty/include)
include_directories(${PROJECT_PATH}/3drparty/mosquitto/include)
include_directories(${PROJECT_PATH}/3drparty/include/rest_rcp)
include_directories(${PROJECT_PATH}/3drparty/include/rest_rcp/asio)
include_directories(${PROJECT_PATH}/3drparty/include/rest_rcp/msgpack-c/include)
# 添加QDarkStyleSheet源码
include_directories(${PROJECT_PATH}/Src)

# 添加库文件目录
link_directories(${PROJECT_PATH}/3drparty/mosquitto/lib/debug)
link_directories(${PROJECT_PATH}/3drparty/libs)

set(PROJECT_SOURCES
        main.cpp
        mainwindow.cpp
        mainwindow.h
        mainwindow.ui
        customtitlebar.cpp
        customtitlebar.h
        Src/gkd_task_test/gkd_task_test.cpp
        Src/gkd_task_test/gkd_task_test.h
        Src/gkd_task_test/gkd_task_test.ui
        Src/gkd_task_test/GkdDataFusion.cpp
        Src/gkd_task_test/GkdDataFusion.h
        Src/gkd_task_test/GkdDataFusion.ui
        Src/gkd_task_test/PlanRequestWidget.cpp
        Src/gkd_task_test/PlanRequestWidget.h
        Src/gkd_task_test/PlanRequestWidget.ui
        Src/gkd_task_test/TaskDeliveryWidget.cpp
        Src/gkd_task_test/TaskDeliveryWidget.h
        Src/gkd_task_test/TaskDeliveryWidget.ui
        Src/gkd_task_test/ElectromagneticTestWidget.cpp
        Src/gkd_task_test/ElectromagneticTestWidget.h
        Src/gkd_task_test/ElectromagneticTestWidget.ui
        Src/gkd_task_test/VelocityConfigWidget.cpp
        Src/gkd_task_test/VelocityConfigWidget.h
        Src/gkd_task_test/VelocityConfigWidget.ui
        Src/common/linearInterpolate.h
        Src/gkd_game_test/GkdDataFusion_by.cpp
        Src/gkd_game_test/GkdDataFusion_by.h
        Src/gkd_game_test/GkdDataFusion_by.ui
        rest_rcp_client.cpp
        rest_rcp_client.h
        rest_rcp_client.ui
        XsJSimUi.cpp
        XsJSimUi.h
        XsJSimUi.ui
        RCPServerBridge.cpp
        RCPServerBridge.h
        Src/utm/projection_utm.h
        Src/utm/projection_utm.cc
        Src/utm/map_define.h
        Src/utm/stdafx.cpp
        Src/utm/stdafx.h
        Src/utm/targetver.h
        Src/common/toast.cpp
        Src/common/toast.h
        Src/common/messagedialog.cpp
        Src/common/messagedialog.h
        Src/gkd/c2manager.h
        Src/gkd/c2manager.cpp
        Src/gkd/taskmodelbehavior.h
        Src/gkd/taskmodelbehavior.cpp
        Src/gkd/DtTaskTarget.h
        Src/gkd/DtTaskTarget.cpp
        Src/gkd/drone_data_parser.h
        Src/gkd/drone_data_parser.cpp
        Src/gkd/task_response_define.h
        Src/gkd/mission.cpp
        Src/gkd/mission.h
        Src/gkd/mission_control.cpp
        Src/gkd/mission_control.h
        Src/gkd/dt_control.cpp
        Src/gkd/dt_control.h
        Src/gkd/dt_zzmb.cpp
        Src/gkd/dt_zzmb.h
        Src/gkd/intelligence.cpp
        Src/gkd/intelligence.h
        Src/xjsim/xjsim.cpp
        Src/xjsim/xjsim.h
        Src/xjsim/data_define.h
        Src/common/common.hpp
        Src/mqtt/conn_base.h
        Src/mqtt/mqtt_conn.cpp
        Src/mqtt/mqtt_conn.h
        Src/mqtt/mq_client.cpp
        Src/mqtt/mq_client.h
        Src/mqtt/mqconfig.h
        Src/common/common_config.h
        Src/common/gkd_config.h
        Src/utils/json.hpp
        Src/utils/uavstatusdefine.h
        Src/utils/uav_data_parser.h
        Src/utils/uav_data_parser.cpp
        Src/utils/log_sys.h
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(ZKmqttApllication
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
else()
    if(ANDROID)
        add_library(ZKmqttApllication SHARED
            ${PROJECT_SOURCES}
        )
    else()
        add_executable(ZKmqttApllication
            ${PROJECT_SOURCES}
        )
    endif()
endif()

# 链接所有必要的库
target_link_libraries(ZKmqttApllication PRIVATE
    mosquitto
    Qt${QT_VERSION_MAJOR}::WebEngineWidgets
    Qt${QT_VERSION_MAJOR}::Widgets
    ${Boost_LIBRARIES}
)
target_link_libraries(ZKmqttApllication PRIVATE xTakLib)

if(${QT_VERSION} VERSION_LESS 6.1.0)
  set(BUNDLE_ID_OPTION MACOSX_BUNDLE_GUI_IDENTIFIER com.example.ZKmqttApllication)
endif()

set_target_properties(ZKmqttApllication PROPERTIES
    ${BUNDLE_ID_OPTION}
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE FALSE
)

include(GNUInstallDirs)
install(TARGETS ZKmqttApllication
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(ZKmqttApllication)
endif()

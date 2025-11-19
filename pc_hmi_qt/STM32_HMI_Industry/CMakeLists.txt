cmake_minimum_required(VERSION 3.16)

project(STM32_HMI_Industry VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt6 REQUIRED COMPONENTS Quick SerialPort REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Core)

qt_standard_project_setup(REQUIRES 6.8)

qt_add_executable(appSTM32_HMI_Industry
    main.cpp
)

qt_add_qml_module(appSTM32_HMI_Industry
    URI STM32_HMI_Industry
    QML_FILES
        Main.qml
        SOURCES serialmanger.h serialmanger.cpp
)

# Qt for iOS sets MACOSX_BUNDLE_GUI_IDENTIFIER automatically since Qt 6.1.
# If you are developing for iOS or macOS you should consider setting an
# explicit, fixed bundle identifier manually though.
set_target_properties(appSTM32_HMI_Industry PROPERTIES
#    MACOSX_BUNDLE_GUI_IDENTIFIER com.example.appSTM32_HMI_Industry
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

target_link_libraries(appSTM32_HMI_Industry
    PRIVATE Qt6::Quick Qt6::SerialPort
)
target_link_libraries(appSTM32_HMI_Industry PRIVATE Qt6::Core)

include(GNUInstallDirs)
install(TARGETS appSTM32_HMI_Industry
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

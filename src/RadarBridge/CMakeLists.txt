# RadarBridge module
#
# Receives radar / external-sensor data over a serial port, parses the custom
# 0xFD binary protocol, and forwards converted MAVLink messages to PX4 over
# QGC's existing vehicle data link.
#
# Requires QtSerialPort, which QGC only uses on desktop platforms.

# The serial-port based radar bridge is desktop-only (mirrors GPS/Comms guards).
if(QGC_NO_SERIAL_LINK OR ANDROID OR IOS)
    return()
endif()

target_sources(${CMAKE_PROJECT_NAME}
    PRIVATE
        Crc16.cc
        Crc16.h
        RadarBridgeManager.cc
        RadarBridgeManager.h
        RadarMavlinkBridge.cc
        RadarMavlinkBridge.h
        RadarProtocolParser.cc
        RadarProtocolParser.h
        RadarSettings.cc
        RadarSettings.h
        RadarTypes.h
)

target_link_libraries(${CMAKE_PROJECT_NAME}
    PRIVATE
        Qt6::Core
        Qt6::SerialPort
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

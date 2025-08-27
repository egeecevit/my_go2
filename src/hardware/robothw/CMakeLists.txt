set(SOURCES RobotHW.cc CANMotorHW.cc RobotIMUHW.cc VN100Driver.cc MdlMotorMaster.cc CubeMARSCAN.cc RobotClockHW.cc)

add_library(robothw STATIC ${SOURCES})
install(TARGETS robothw)

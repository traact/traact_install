cmake_minimum_required(VERSION 3.9)

add_executable(installTest InstallTest.cpp)

conan_target_link_libraries(installTest)

#target_link_libraries(installTest PUBLIC
#        CONAN_PKG::traact_facade
#        CONAN_PKG::traact_spatial
#        )

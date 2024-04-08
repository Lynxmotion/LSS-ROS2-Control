include(ExternalProject)
ExternalProject_Add(lynxmotion-lss
        GIT_REPOSITORY    git@github.com:lynxmotionbeta/AlternativeLSS.git
        GIT_TAG           feature/lss-bus-refactor
        PREFIX            "${CMAKE_CURRENT_BINARY_DIR}/lynxmotion-lss"
        #SOURCE_DIR        "${CMAKE_BINARY_DIR}/lynxmotion-lss/src"
        #BINARY_DIR        "${CMAKE_BINARY_DIR}/lynxmotion-lss/build"
        CMAKE_ARGS        -DWITH_LEGACY=OFF -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_CURRENT_BINARY_DIR}/lynxmotion-lss
        #CONFIGURE_COMMAND ""
        #BUILD_COMMAND     ""
        #INSTALL_COMMAND   ""
        #TEST_COMMAND      ""
        )

ExternalProject_Get_Property (lynxmotion-lss install_dir)
include_directories (${install_dir}/include/lynxmotion-lss)
add_library (lss-bus-static STATIC IMPORTED)
add_library (lss-bus-shared SHARED IMPORTED)
set_target_properties(lss-bus-static PROPERTIES IMPORTED_LOCATION ${install_dir}/lib/liblss-bus-static.a)
set_target_properties(lss-bus-shared PROPERTIES IMPORTED_LOCATION ${install_dir}/lib/liblss-bus-shared.so)

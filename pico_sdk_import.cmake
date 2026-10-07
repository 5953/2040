# pico_sdk_import.cmake - 自动获取 Pico SDK
# 放在仓库根目录, CMakeLists.txt 里的 include(pico_sdk_import.cmake) 会引用它

if (DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
    message("Using PICO_SDK_PATH from environment ('${PICO_SDK_PATH}')")
endif ()

if (DEFINED ENV{PICO_SDK_FETCH_FROM_GIT})
    set(PICO_SDK_FETCH_FROM_GIT $ENV{PICO_SDK_FETCH_FROM_GIT})
    message("Using PICO_SDK_FETCH_FROM_GIT from environment ('${PICO_SDK_FETCH_FROM_GIT}')")
endif ()

if (NOT PICO_SDK_PATH)
    if (PICO_SDK_FETCH_FROM_GIT)
        include(FetchContent)
        set(FETCHCONTENT_BASE_DIR ${CMAKE_CURRENT_BINARY_DIR}/_pico_sdk)
        if (NOT PICO_SDK_FETCH_FROM_GIT_PATH)
            set(PICO_SDK_FETCH_FROM_GIT_PATH "pico-sdk")
        endif ()
        FetchContent_Declare(
                pico_sdk
                GIT_REPOSITORY https://github.com/raspberrypi/pico-sdk.git
                GIT_TAG master
                GIT_SUBMODULES_RECURSE FALSE
        )
        if (NOT pico_sdk_SOURCE_DIR)
            FetchContent_GetProperties(pico_sdk)
            if (NOT pico_sdk_POPULATED)
                message("Fetching Pico SDK from GitHub")
                FetchContent_Populate(pico_sdk)
            endif ()
        endif ()
        set(PICO_SDK_PATH ${pico_sdk_SOURCE_DIR})
    else ()
        message(FATAL_ERROR "Must specify PICO_SDK_PATH or PICO_SDK_FETCH_FROM_GIT")
    endif ()
endif ()

set(PICO_SDK_INIT_CMAKE_FILE ${PICO_SDK_PATH}/pico_sdk_init.cmake)
if (NOT EXISTS ${PICO_SDK_INIT_CMAKE_FILE})
    message(FATAL_ERROR "Could not find Pico SDK at '${PICO_SDK_PATH}'")
endif ()

include(${PICO_SDK_INIT_CMAKE_FILE})
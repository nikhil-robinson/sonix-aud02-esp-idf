set(PRIVREQ esp_timer)
set(REQ driver)
set(SRC_FILES "sonix_audo02.c")


idf_component_register(SRCS ${SRC_FILES}
                        INCLUDE_DIRS include
                        REQUIRES ${REQ}
                        PRIV_REQUIRES ${PRIVREQ})

include(package_manager)
cu_pkg_define_version(${CMAKE_CURRENT_LIST_DIR})

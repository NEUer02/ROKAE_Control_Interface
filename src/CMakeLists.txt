INCLUDE_DIRECTORIES(
        /usr/include/eigen3
        ${PROJECT_SOURCE_DIR}/inc
        ${PROJECT_SOURCE_DIR}/inc/include
)


LINK_DIRECTORIES(
        /usr/lib/x86_64-linux-gun
        ${PROJECT_SOURCE_DIR}/lib
)

set(SRC
        ea_admittance_control
        )

foreach (src ${SRC})
    add_executable(${src} ${src}.cpp)
    TARGET_LINK_LIBRARIES(${src}
            RCI_Client.a
            xMateModel.a
            orocos-kdl.a
            glog.a
            pthread
            )
endforeach ()

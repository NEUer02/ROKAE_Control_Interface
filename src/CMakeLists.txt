INCLUDE_DIRECTORIES(
        /usr/include/eigen3
        ${PROJECT_SOURCE_DIR}/inc
        ${PROJECT_SOURCE_DIR}/inc/include
        ${PROJECT_SOURCE_DIR}/inc/optimal_excitation_trajectory
        ${PROJECT_SOURCE_DIR}/inc/constant_jointspace_impedance
)


LINK_DIRECTORIES(
        /usr/lib/x86_64-linux-gun
        ${PROJECT_SOURCE_DIR}/lib
)

set(SRC
        ea_admittance_control
        excitation_trajectory
        constant_jointSpace_impedance
        )

add_library(excitation_trajectory_tools utils/excitation_trajectory_tools.cpp utils/torque_from_regression_matrix.cpp)

foreach (src ${SRC})
    add_executable(${src} ${src}.cpp)
    TARGET_LINK_LIBRARIES(${src}
            excitation_trajectory_tools
            RCI_Client.a
            xMateModel.a
            orocos-kdl.a
            glog.a
            pthread
            )
endforeach ()

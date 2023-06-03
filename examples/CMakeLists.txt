INCLUDE_DIRECTORIES(
        /usr/include/eigen3
        ${PROJECT_SOURCE_DIR}/inc
        ${PROJECT_SOURCE_DIR}/inc/include
)


LINK_DIRECTORIES(
        /usr/lib/x86_64-linux-gun
        ${PROJECT_SOURCE_DIR}/lib
)

set(EXAMPLES
        cartesian_impedance_move
        cartesian_nullspace_move
        cartesian_position_move
        cartesian_position_move_xmate7
        drag_test
        cartesian_rotation_move
        cartesian_s_line_mu
        joint_impedance_move
        joint_s_mu
        joint_s_mu_ea
        torque_control
        )

foreach (example ${EXAMPLES})
    add_executable(${example} ${example}.cpp)
    TARGET_LINK_LIBRARIES(${example}
            RCI_Client.a
            xMateModel.a
            orocos-kdl.a
            glog.a
            pthread
            )
endforeach ()

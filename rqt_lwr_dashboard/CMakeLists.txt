cmake_minimum_required(VERSION 2.8.3)
project(rqt_lwr_dashboard)

find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()

install(FILES plugin.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
   scripts/rqt_lwr_dashboard
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach (dir images)
   install(DIRECTORY ${dir}/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

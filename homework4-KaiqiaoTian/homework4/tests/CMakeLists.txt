find_package(rostest REQUIRED)

add_rostest(homework4.test)

catkin_download_test_data(
  homework4.bag
  http://www.secs.oakland.edu/~mtradovn/ece_6460/homework4.bag
  DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
  FILENAME homework4.bag
  MD5 03bae43a3adf2c3b11a609e39f48bda9
)

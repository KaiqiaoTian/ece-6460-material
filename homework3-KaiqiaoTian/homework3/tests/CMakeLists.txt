find_package(rostest REQUIRED)

add_rostest(homework3.test)

catkin_download_test_data(
  homework3.bag
  http://www.secs.oakland.edu/~mtradovn/ece_6460/homework3.bag
  DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
  FILENAME homework3.bag
  MD5 bb90d17993d5cfe758b41c36d639ae57
)
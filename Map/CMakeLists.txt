add_library(Map
        GlobalMap.cpp
        )
target_link_libraries(Map
  ${Boost_LIBRARIES}
  )

set(sources
  WRSPlugin.cpp
)

set(headers

)

set(target CnoidWRSPlugin)
choreonoid_make_gettext_mo_files(${target} mofiles)
choreonoid_add_plugin(${target} ${sources} ${mofiles} HEADERS ${headers})
target_link_libraries(${target} PUBLIC CnoidBodyPlugin CnoidWRSUtilPlugin)
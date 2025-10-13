set(sources
  LivePlaybackBar.cpp
  LivePlaybackPlugin.cpp
)

set(headers
  LivePlaybackBar.h
  exportdecl.h
)

set(target CnoidLivePlaybackPlugin)
choreonoid_make_gettext_mo_files(${target} mofiles)
choreonoid_add_plugin(${target} ${sources} ${mofiles} HEADERS ${headers})
target_link_libraries(${target} PUBLIC CnoidBodyPlugin)
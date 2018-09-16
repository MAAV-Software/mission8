find_path(LIBEV_INCLUDE_DIR ev.h
	PATHS
	/usr/include
	)

find_library(LIBEV_LIBRARY
	NAMES ev
	PATHS ${LIBRARY_PATHS})

mark_as_advanced(LIBEV_INCLUDE_DIR LIBEV_LIBRARIES)

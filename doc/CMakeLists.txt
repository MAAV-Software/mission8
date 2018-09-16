find_package(Doxygen)
find_package(Graphviz)

add_custom_target(doc)

if(GRAPHVIZ_FOUND)
	add_custom_target(doc_graphviz)
	set(GRAPHVIZ_INPUTS hlsys)

	foreach(graph ${GRAPHVIZ_INPUTS})
		add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${graph}.png
			COMMAND ${GRAPHVIZ_DOT_EXECUTABLE} -Tpng ${CMAKE_CURRENT_SOURCE_DIR}/img/${graph}.dot > ${CMAKE_CURRENT_BINARY_DIR}/${graph}.png
			DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/img/${graph}.dot
			VERBATIM)
		add_custom_target(doc_graphviz_${graph}
			SOURCES ${CMAKE_CURRENT_BINARY_DIR}/${graph}.png)
		add_dependencies(doc_graphviz doc_graphviz_${graph})
	endforeach()

	add_dependencies(doc doc_graphviz)
endif()

if(DOXYGEN_FOUND)
	configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in
		${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)

	add_custom_target(doc_doxygen
		${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
		WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
		COMMENT "Generating API documentation with Doxygen" VERBATIM)

	add_dependencies(doc doc_doxygen)
endif()

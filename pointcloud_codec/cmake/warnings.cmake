function(target_warnings tgt)
  if (NOT TARGET ${tgt})
    message(FATAL_ERROR "target_warnings: '${tgt}' is not a target")
  endif()

  get_target_property(_type ${tgt} TYPE)
  if (_type STREQUAL "INTERFACE_LIBRARY")
    set(_scope INTERFACE)
  else()
    set(_scope PRIVATE)
  endif()

  if (MSVC)
    target_compile_options(${tgt} ${_scope}
      /W4 /permissive-
      /w14242 /w14254 /w14263 /w14265 /w14287
      /we4289
      /w14296 /w14311
      /w14545 /w14546 /w14547 /w14549 /w14555
      /w14640 /w14826 /w14905 /w14906 /w14928
    )
  else()
    target_compile_options(${tgt} ${_scope}
      -Wall -Wextra -Wpedantic
      -Wshadow
      -Wconversion -Wsign-conversion
      -Wold-style-cast
      -Woverloaded-virtual -Wnon-virtual-dtor
      -Wdouble-promotion
      -Wnull-dereference
    )
  endif()
endfunction()

set(EXAMPLES "primes" "BFS" "word_counts" "tokens" "filter" "linefit" "mcss"
  "quickhull" "pagerank" "spanning_tree" "maximal_matching"
  "min_spanning_tree" "mergesort" "samplesort" "kth_smallest"
  "spectral_separator" "find_if" "rabin_karp" "kmeans_pp" "delaunay")

# set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")
#set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -pg")

function(add_example NAME)
  add_executable(${NAME} ${NAME}.cpp)
  target_compile_options(${NAME} PRIVATE -g)  # -pg)
  target_link_libraries(${NAME} PRIVATE parlay)
endfunction()

foreach(example ${EXAMPLES})
  add_example(${example})
endforeach()







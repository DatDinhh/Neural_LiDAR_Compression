#ifdef QUANT_HEADER
  #undef QUANT_HEADER
#endif
#if __has_include(<pcc/codec/quantisation.hpp>)
  #define QUANT_HEADER <pcc/codec/quantisation.hpp>
#elif __has_include("../codec/quantisation.hpp")
  #define QUANT_HEADER "../codec/quantisation.hpp"
#elif __has_include("codec/quantisation.hpp")
  #define QUANT_HEADER "codec/quantisation.hpp"
#else
  #error "quantisation.hpp not found"
#endif
#include QUANT_HEADER
#include <iostream>
int main(int, char**) {
  std::cerr << "pcc_tool stub: original main.cpp was removed by an edit; restore it when ready.\n";
  return 0;
}


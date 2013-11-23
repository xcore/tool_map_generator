#include "map_generator.h"
#include <stdlib.h>
#include <iostream>

#define VERIFY(x) \
  do { \
    if (!(x)) { \
      std::cerr << "FAILED: " << #x << '\n'; \
      std::exit(1); \
    } \
  } while(0)

// Map from module_zip of https://github.com/xcore/sc_advanced_port_usage
static map_generator::IntMap unzip_map()
{
  map_generator::IntMap map;
  for (unsigned i = 0; i < 256; i++) {
    unsigned unzipped = 0;
    for (unsigned j = 0; j < 8; j++) {
      if (i & (1 << j)) {
        unzipped |= 1 << (j * 4);
      }
    }
    map.insert(std::make_pair(unzipped, i));
  }
  return map;
}

// Should just use a straight table lookup.
static map_generator::IntMap uncompressible_map()
{
  return {
    {1, 2},
    {3, 4},
    {4, 6},
    {6, 8},
  };
}


int main()
{
  map_generator::Steps steps = map_generator::generate(unzip_map(), 1);
  map_generator::CodeCost cost = steps.compute_cost();
  VERIFY(cost.bytes == 268);
  steps = map_generator::generate(uncompressible_map(), 1);
  cost = steps.compute_cost();
  VERIFY(cost.cycles == 2);
  return 0;
}

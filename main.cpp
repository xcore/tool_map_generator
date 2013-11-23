#include "map_generator.h"
#include <iostream>
#include <random>

static map_generator::IntMap testmap()
{
  map_generator::IntMap map;
  std::default_random_engine generator;
  std::uniform_int_distribution<int> distribution;
  auto rand = std::bind(distribution, generator);
  for (unsigned i = 0; i < 1000; i++) {
    map.insert(std::make_pair(rand(), rand()));
  }
  return map;
}

static map_generator::IntMap testmap2()
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

static map_generator::IntMap readmap()
{
  map_generator::IntMap map{
    {0, 0},
    {1, 2},
    {2, 3},
    {100, 4},
    {1897722, 5},
  };
  return map;
}

int main()
{
  const unsigned max_stages = 1;
  const auto map = testmap();
  map_generator::Steps steps = map_generator::generate(map, max_stages);
  steps.print(std::cout);
  map_generator::CodeCost cost = steps.compute_cost();
  std::cerr << "Number of bytes: " << cost.bytes << '\n';
  std::cerr << "Number of cycles: " << cost.cycles << '\n';
  return 0;
}

// Copyright (c) 2013, XMOS Ltd., All rights reserved
// This software is freely distributable under a derivative of the
// University of Illinois/NCSA Open Source License posted in
// LICENSE.txt and at <http://github.xcore.com/>

#include "map_generator.h"
#include <iostream>
#include <fstream>
#include <random>
#include <cstdlib>
#include <cctype>
#include <numeric>

using map_generator::IntMap;
using map_generator::Steps;
using map_generator::CodeCost;

/// Read comma separated list of integers representing a map. There must be an
/// an even number of integers - even numbers are keys, odd numbers are values.
/// For example the string "1,5,8,1" specifies a map where 1 -> 5 and 8 -> 1.
static IntMap read_map(std::istream &in)
{
  IntMap map;
  std::string contents;
  while (1) {
    auto c = in.get();
    if (!in.good())
      break;
    if (std::isspace(c))
      continue;
    contents.push_back(c);
  }
  // Parse into a list of integers.
  std::vector<long> values;
  const char *p = contents.c_str();
  while (*p != '\0') {
    char *endp;
    auto value = std::strtol(p, &endp, 10);
    if (endp == p) {
      std::cerr << "error: unexpected format\n";
      std::exit(1);
    }
    values.push_back(value);
    p = endp;
    if (*p == ',') {
      ++p;
    } else if (*p != '\0') {
      std::cerr << "error: unexpected format\n";
      std::exit(1);
    }
  }
  if ((values.size() % 2) != 0) {
    std::cerr << "error: odd number of values\n";
    std::exit(1);
  }
  // Add values to map.
  for (unsigned i = 0, e = values.size(); i != e; i += 2) {
    map.insert(std::make_pair(values[i], values[i + 1]));
  }
  return map;
}

static IntMap read_map(const char *filename)
{
  std::ifstream in(filename);
  if (!in) {
    std::cerr << "error: opening \"" << filename << "\"\n";
    std::exit(1);
  }
  return read_map(in);
}

static void usage()
{
  std::cerr << "Usage: map_generator [options] [file]\n";
  std::cerr << "\nOptions:\n";
  std::cerr << "  --max-stages <n>    Use at most <n> lookups\n";
}

int main(int argc, char **argv)
{
  const char *filename = nullptr;
  unsigned long max_stages = 1;
  for (int i = 1; i < argc; ++i) {
    std::string arg(argv[i]);
    if (arg == "--max-stages") {
      if (++i == argc) {
        usage();
        std::exit(1);
      }
      char *endp;
      max_stages = strtoul(argv[i], &endp, 10);
      if (*endp != '\0' || max_stages > std::numeric_limits<unsigned>::max()) {
        usage();
        std::exit(1);
      }
    } else if (arg == "--help") {
      usage();
      std::exit(0);
    } else {
      if (filename) {
        usage();
        std::exit(1);
      }
      filename = argv[i];
    }
  }
  const auto map = filename ? read_map(filename) : read_map(std::cin);
  Steps steps = map_generator::generate(map, max_stages);
  steps.print(std::cout);
  CodeCost cost = steps.compute_cost();
  std::cerr << "Number of bytes: " << cost.bytes << '\n';
  std::cerr << "Number of cycles: " << cost.cycles << '\n';
  return 0;
}

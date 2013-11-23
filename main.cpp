#include "map_generator.h"
#include <iostream>
#include <fstream>
#include <random>
#include <cstdlib>
#include <cctype>
#include <regex>
#include <numeric>

using map_generator::IntMap;
using map_generator::Steps;
using map_generator::CodeCost;

IntMap read_map(std::istream &in)
{
  IntMap map;
  std::string contents;
  while (1) {
    auto c = in.get();
    if (!in.good())
      break;
    contents.push_back(c);
  }
  std::regex regex(R"(^(,?)(\d+)\s*,(\d+)\s*)");
  auto it =
  std::sregex_iterator(contents.begin(), contents.end(), regex);
  auto end = std::sregex_iterator();
  bool first = true;
  for (; it != end;) {
    auto match = *it;
    bool leading_comma = match[1].length() != 0;
    if (leading_comma == first) {
      std::cerr << "error: unexpected format\n";
      std::exit(1);
    }
    auto key_string = match[2];
    auto value_string = match[3];
    auto key = std::strtol(key_string.str().c_str(), nullptr, 10);
    auto value = std::strtol(value_string.str().c_str(), nullptr, 10);
    map.insert(std::make_pair(key, value));
    first = false;
    auto next = it;
    ++next;
    if (next == end) {
      auto pos = it->position() + it->length();
      if (pos != contents.length()) {
        std::cerr << "error: unexpected format\n";
        std::exit(1);
      }
    }
    it = next;
  }
  return map;
}

IntMap read_map(const char *filename)
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

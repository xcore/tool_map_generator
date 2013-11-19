#include <algorithm>
#include <bitset>
#include <climits>
#include <map>
#include <set>
#include <cstdlib>
#include <iostream>
#include <initializer_list>
#include <random>
#include <numeric>

namespace {
  template <typename T> uint32_t crc(uint32_t checksum, T data, uint32_t poly) {
    for (unsigned i = 0; i < sizeof(T) * CHAR_BIT; i++) {
      int xorBit = (checksum & 1);
      
      checksum  = (checksum >> 1) | ((data & 1) << 31);
      data = data >> 1;
      
      if (xorBit)
        checksum = checksum ^ poly;
    }
    return checksum;
  }
}

static uint32_t crc32(uint32_t checksum, uint32_t data, uint32_t poly)
{
  return crc<uint32_t>(checksum, data, poly);
}

static unsigned clz(uint32_t x)
{
  if (x == 0)
    return 32;
  return __builtin_clz(x);
}

static unsigned log2_ceil(uint32_t x)
{
  return 32 - clz(x - 1);
}

static uint32_t bswap32(uint32_t x)
{
  return __builtin_bswap32(x);
}

static uint32_t bitrev(uint32_t x)
{
  x =  ((x & 0xaaaaaaaa) >> 1) | ((x & 0x55555555) << 1);
  x = ((x & 0xcccccccc) >> 2) | ((x & 0x33333333) << 2);
  x = ((x & 0xf0f0f0f0) >> 4) | ((x & 0x0f0f0f0f) << 4);
  return bswap32(x);
}

static std::map<uint32_t,uint32_t> readmap()
{
  std::map<uint32_t,uint32_t> map{
    {0, 0},
    {1, 2},
    {2, 3},
    {100, 4},
    {1897722, 5},
  };
  return map;
}

namespace {
  template <typename T>
  bool apply_hash_to_map(const std::map<uint32_t,uint32_t> &map, T hash,
                         std::map<uint32_t,uint32_t> &result,
                         bool require_fewer_entries = false)
  {
    std::map<uint32_t, uint32_t> reduced;
    for (const auto &entry : map) {
      uint32_t value = hash(entry.first);
      auto res = reduced.insert(std::make_pair(value, entry.second));
      if (!res.second && res.first->second != entry.second) {
        return false;
      }
    }
    if (require_fewer_entries && map.size() == reduced.size())
      return false;
    result = std::move(reduced);
    return true;
  }

  template <typename T>
  bool apply_hash_to_map_inplace(std::map<uint32_t,uint32_t> &map, T hash,
                                 bool require_fewer_entries = false)
  {
    return apply_hash_to_map(map, hash, map, require_fewer_entries);
  }

  template <typename T> std::bitset<32>
  reduce_using_bitop(std::map<uint32_t,uint32_t> &map, T op)
  {
    std::bitset<32> mask;
    for (int bit = 31; bit >= 0; bit--) {
      std::bitset<32> candidate_mask = mask;
      candidate_mask.set(bit);
      if (apply_hash_to_map_inplace(map, [=](uint32_t x) {
        return op(x, candidate_mask.to_ulong());
      }, true)) {
        mask = candidate_mask;
      }
    }
    return mask;
  }
}

static std::bitset<32> reduce_using_and(std::map<uint32_t,uint32_t> &map)
{
  std::bitset<32> mask =
    reduce_using_bitop(map, [](uint32_t a, uint32_t b) { return a & ~b; });
  return mask.flip();
}

static std::bitset<32> reduce_using_or(std::map<uint32_t,uint32_t> &map)
{
  return reduce_using_bitop(map, [](uint32_t a, uint32_t b) { return a | b; });
}

static uint32_t get_lookup_table_size(const std::map<uint32_t,uint32_t> &map)
{
  uint32_t size = 0;
  for (const auto &entry : map) {
    size = std::max(size, entry.first);
  }
  return size;
}

static uint32_t reduce_using_crc(std::map<uint32_t,uint32_t> &map)
{
  std::set<uint32_t> unique_values;
  for (const auto &entry : map) {
    unique_values.insert(entry.second);
  }
  uint32_t best = 0;
  uint32_t best_size = std::numeric_limits<uint32_t>::max();
  unsigned min_width = log2_ceil(unique_values.size()) - 1;
  for (unsigned width = min_width; width < 32; width++) {
    for (uint32_t poly = 1 << width; poly < (1 << (width + 1));
         poly++) {
      std::map<uint32_t,uint32_t> tmp;
      if (apply_hash_to_map(map, [=](uint32_t x) {
        return crc32(x, poly, poly);
      }, tmp)) {
        uint32_t size = get_lookup_table_size(tmp);
        if (size < best_size) {
          best = poly;
          best_size = size;
        }
      }
    }
    // Don't check polynomials of higher widths - we are unlikely to get any
    // further improvement.
    if (best != 0)
      break;
  }
  if (best != 0) {
    if (best_size + 4 >= get_lookup_table_size(map)) {
      // We havent been able to make a worthwhile improvement.
      best = 0;
    }
    apply_hash_to_map_inplace(map, [=](uint32_t x) {
      return crc32(x, best, best);
    });
  }
  return best;
}

static unsigned reduce_using_shift(std::map<uint32_t,uint32_t> &map)
{
  unsigned shift;
  for (shift = 0; shift < 32; shift++) {
    if (!apply_hash_to_map_inplace(map, [](uint32_t x) {
      return x >> 1;
    })) {
      break;
    }
  }
  return shift;
}

static void print(const std::bitset<32> &and_mask,
                  uint32_t poly1,
                  uint32_t poly2,
                  unsigned shift,
                  const std::map<uint32_t, uint32_t> &lookup)
{
  std::cout << "uint32_t map(uint32_t x)\n";
  std::cout << "{\n";
  std::cout << "  static uint32_t lookup[] = {\n";
  unsigned i = 0;
  for (const auto &entry : lookup) {
    for (; i < entry.first; i++) {
      std::cout << "    0, /* unused */\n";
    }
    std::cout << "    " << entry.second << ',';
    std::cout << " /* " << entry.first << " -> " << entry.second << " */\n";
    i++;
  }
  std::cout << "  };\n";
  std::cout << std::hex;
  if (!and_mask.all()) {
    std::cout << "  x &= 0x" << and_mask.to_ulong() << ";\n";
  }
  for (auto poly : { poly1, poly2 }) {
    if (poly != 0) {
      std::cout << "  x = crc32(x, 0x" << poly << ", 0x" << poly << ");\n";
    }
  }
  std::cout << std::dec;
  if (shift) {
    std::cout << "  x >>= " << shift << ";\n";
  }
  std::cout << "  return lookup[x];\n";
  std::cout << "}\n";
}

static bool validate(const std::map<uint32_t,uint32_t> &map,
                     const std::bitset<32> &and_mask,
                     uint32_t poly1,
                     uint32_t poly2,
                     unsigned shift,
                     const std::map<uint32_t, uint32_t> &lookup)
{
  for (const auto entry : map) {
    uint32_t value = entry.first;
    value &= and_mask.to_ulong();
    for (auto poly : { poly1, poly2 }) {
      if (poly != 0)
        value = crc32(value, poly, poly);
    }
    value >>= shift;
    auto match = lookup.find(value);
    if (match == lookup.end() || match->second != entry.second)
      return false;
  }
  return true;
}

static std::map<uint32_t,uint32_t> testmap()
{
  std::map<uint32_t,uint32_t> map;
  std::default_random_engine generator;
  std::uniform_int_distribution<int> distribution;
  auto rand = std::bind(distribution, generator);
  for (unsigned i = 0; i < 256; i++) {
    map.insert(std::make_pair(rand(), rand()));
  }
  return map;
}

static std::map<uint32_t,uint32_t> testmap2()
{
  std::map<uint32_t,uint32_t> map;
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

int main()
{
  const auto map = testmap();
  auto lookup = map;
  const auto and_mask = reduce_using_and(lookup);
  const auto poly1 = reduce_using_crc(lookup);
  const auto poly2 = reduce_using_crc(lookup);
  const auto shift = reduce_using_shift(lookup);
  print(and_mask, poly1, poly2, shift, lookup);
  if (!validate(map, and_mask, poly1, poly2, shift, lookup)) {
    std::cerr << "Failed validation\n";
    std::exit(1);
  }
  return 0;
}

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
#include <cassert>

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

namespace {
  class Step {
  public:
    enum Result {
      DONE,
      NEXT,
      ERROR
    };
    virtual Result apply(uint32_t &) const = 0;
    virtual void print(std::ostream &) const = 0;
  };

  enum class DataType {
    S8,
    U8,
    S16,
    U16,
    S32,
    U32
  };

  class LookupTableStep : public Step {
    std::map<uint32_t,uint32_t> table;
    DataType type;
  public:
    LookupTableStep(std::map<uint32_t,uint32_t> table, DataType type) :
      table(table), type(type) {}
    Result apply(uint32_t &x) const override {
      auto match = table.find(x);
      if (match == table.end())
        return ERROR;
      x = match->second;
      return NEXT;
    }
    void print(std::ostream &out) const override;
  };

  class CrcLookupAndReturnStep : public Step {
    uint32_t poly;
    std::map<uint32_t,uint32_t> table;
    DataType type;
    uint32_t collision_sentinal;
  public:
    CrcLookupAndReturnStep(uint32_t poly, std::map<uint32_t,uint32_t> table,
                           DataType type, uint32_t sentinal) :
      poly(poly), table(table), type(type), collision_sentinal(sentinal) {}
    Result apply(uint32_t &x) const override {
      uint32_t tmp = crc32(x, poly, poly);
      auto match = table.find(tmp);
      if (match == table.end())
        return ERROR;
      if (match->second != collision_sentinal) {
        x = match->second;
        return DONE;
      }
      return NEXT;
    }
    void print(std::ostream &out) const override;
  };

  class CrcStep : public Step {
    uint32_t poly;
  public:
    CrcStep(uint32_t poly) : poly(poly) {}
    virtual Result apply(uint32_t &x) const override {
      x = crc32(x, poly, poly);
      return NEXT;
    }
    virtual void print(std::ostream &out) const override {
      out << "  crc32(x, 0x" << poly << ", 0x" << poly << ");\n";
    }
  };
}

static const char *get_type_name(DataType type)
{
  switch (type) {
  case DataType::S8:
    return "int8_t";
  case DataType::U8:
    return "uint8_t";
  case DataType::S16:
    return "int16_t";
  case DataType::U16:
    return "uint16_t";
  case DataType::S32:
    return "int32_t";
  case DataType::U32:
    return "uint32_t";
  }
}

bool is_signed(DataType type)
{
  switch (type) {
  case DataType::S8:
  case DataType::S16:
  case DataType::S32:
    return true;
  case DataType::U8:
  case DataType::U16:
  case DataType::U32:
    return false;
  }
}

static void
print_lookup_table(std::ostream &out, const std::map<uint32_t,uint32_t> &table,
                   DataType type)
{
  out << "    static const " << get_type_name(type) << " lookup[] = {\n";
  unsigned i = 0;
  for (const auto &entry : table) {
    for (; i < entry.first; i++) {
      out << "      0, /* unused */\n";
    }
    if (is_signed(type)) {
      out << "      " << static_cast<int32_t>(entry.second) << ',';
    } else {
      out << "      " << entry.second << ',';
    }
    out << " /* " << entry.first << " -> " << entry.second << " */\n";
    i++;
  }
  out << "    };\n";
}

void LookupTableStep::print(std::ostream &out) const
{
  out << "  {\n";
  print_lookup_table(out, table, type);
  out << "    x = lookup[x];\n";
  out << "  };\n";
}

void CrcLookupAndReturnStep::print(std::ostream &out) const
{
  out << "  {\n";
  print_lookup_table(out, table, type);
  out << "    uint32_t tmp = x;\n";
  out << "    crc32(tmp, 0x" << poly << ", 0x" << poly << ");\n";
  out << "    tmp = lookup[tmp];\n";
  out << "    if (tmp != " << collision_sentinal << ") {\n";
  out << "      return tmp;\n";
  out << "    }\n";
  out << "  };\n";
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
  template <typename T> bool
  apply_hash_to_map(const std::map<uint32_t,uint32_t> &map, T hash,
                    unsigned max_collisions,
                    uint32_t collision_sentinal,
                    std::map<uint32_t,uint32_t> &result,
                    uint32_t &num_collisions)
  {
    num_collisions = 0;
    std::map<uint32_t, uint32_t> reduced;
    for (const auto &entry : map) {
      uint32_t value = hash(entry.first);
      auto res = reduced.insert(std::make_pair(value, entry.second));
      if (res.second || res.first->second == entry.second ||
          res.first->second == collision_sentinal)
        continue;
      // Handle collisions.
      if (++num_collisions > max_collisions) {
        return false;
      }
      res.first->second = collision_sentinal;
    }
    result = std::move(reduced);
    return true;
  }
  
  template <typename T>
  bool apply_hash_to_map(const std::map<uint32_t,uint32_t> &map, T hash,
                         std::map<uint32_t,uint32_t> &result)
  {
    unsigned num_collisions;
    return apply_hash_to_map(map, hash, 0, 0, result, num_collisions);
  }

  template <typename T>
  bool apply_hash_to_map_inplace(std::map<uint32_t,uint32_t> &map, T hash)
  {
    return apply_hash_to_map(map, hash, map);
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
      })) {
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

static uint32_t get_lookup_table_size(const std::map<uint32_t,uint32_t> &map)
{
  uint32_t size = 0;
  for (const auto &entry : map) {
    size = std::max(size, entry.first);
  }
  return size;
}

static uint32_t find_unused_value(const std::set<uint32_t> &values)
{
  unsigned i = 0;
  for (auto value : values) {
    if (value != i)
      return i;
    ++i;
  }
  return values.size();
}

static DataType pick_datatype(const std::map<uint32_t,uint32_t> &lookup)
{
  uint32_t max_u = 0;
  int32_t min_s = 0;
  int32_t max_s = 0;
  for (const auto &entry : lookup) {
    uint32_t value_u = entry.second;
    int32_t value_s = static_cast<int32_t>(value_u);
    max_u = std::max(max_u, value_u);
    max_s = std::max(max_s, value_s);
    min_s = std::min(min_s, value_s);
  }
  // Prefer unsigned 8bit loads signed as there is no 8bit signed load.
  if (max_u <= std::numeric_limits<uint8_t>::max())
    return DataType::U8;
  if (min_s >= std::numeric_limits<int8_t>::min() &&
      max_s <= std::numeric_limits<int8_t>::max())
    return DataType::S8;
  // Prefer signed 16bit loads as there is no 16bit unsigned load.
  if (min_s >= std::numeric_limits<int16_t>::min() &&
      max_s <= std::numeric_limits<int16_t>::max())
    return DataType::S16;
  if (max_u <= std::numeric_limits<uint16_t>::max())
    return DataType::U16;
  return DataType::U32;
}

static void
reduce_using_crc(std::map<uint32_t,uint32_t> &map,
                 bool allow_conflicts,
                 std::vector<std::unique_ptr<Step>> &steps)
{
  std::set<uint32_t> unique_values;
  for (const auto &entry : map) {
    unique_values.insert(entry.second);
  }
  uint32_t collision_sentinal;
  if (allow_conflicts) {
    collision_sentinal = find_unused_value(unique_values);
    unique_values.insert(collision_sentinal);
  }
  uint32_t best = 0;
  uint32_t best_size = std::numeric_limits<uint32_t>::max();
  unsigned min_width = log2_ceil(unique_values.size()) - 1;
  for (unsigned width = min_width; width < 32; width++) {
    for (uint32_t poly = 1 << width; poly < (1 << (width + 1));
         poly++) {
      std::map<uint32_t,uint32_t> tmp;
      auto hash = [=](uint32_t x) {
        return crc32(x, poly, poly);
      };
      if (allow_conflicts) {
        uint32_t size;
        if (apply_hash_to_map(map, hash, best_size - 1, collision_sentinal, tmp,
                              size)) {
          if (size < best_size) {
            best = poly;
            best_size = size;
            if (size == 0)
              break;
          }
        }
      } else {
        if (apply_hash_to_map(map, hash, tmp)) {
          uint32_t size = get_lookup_table_size(tmp);
          if (size < best_size) {
            best = poly;
            best_size = size;
          }
        }
      }
    }
    // Don't check polynomials of higher widths - we are unlikely to get any
    // further improvement.
    if (best != 0)
      break;
  }
  if (best == 0)
    return;
  if (best_size + 4 >= get_lookup_table_size(map))
    return;
  auto hash = [=](uint32_t x) {
    return crc32(x, best, best);
  };
  if (allow_conflicts) {
    uint32_t dummy;
    std::map<uint32_t,uint32_t> lookup;
    apply_hash_to_map(map, hash, best_size, collision_sentinal, lookup, dummy);
    DataType data_type = pick_datatype(lookup);
    steps.push_back(
      std::unique_ptr<Step>(
        new CrcLookupAndReturnStep(best, std::move(lookup), data_type,
                                   collision_sentinal)
      )
    );
    // Build map containing only conflicting keys.
    std::map<uint32_t,uint32_t> remaining;
    for (const auto &entry : map) {
      uint32_t x = entry.first;
      if (steps.back()->apply(x) != Step::DONE) {
        remaining.insert(entry);
      }
    }
    std::swap(map, remaining);
  } else {
    apply_hash_to_map_inplace(map, hash);
    steps.push_back(std::unique_ptr<Step>(new CrcStep(best)));
  }
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

static void print(const std::vector<std::unique_ptr<Step>> &steps)
{
  std::cout << "uint32_t map(uint32_t x)\n";
  std::cout << "{\n";
  for (const auto &step : steps) {
    step->print(std::cout);
  }
  std::cout << "  return x;\n";
  std::cout << "}\n";
}

static bool validate(const std::map<uint32_t, uint32_t> &map,
                     const std::vector<std::unique_ptr<Step>> &steps)
{
  for (const auto &entry : map) {
    uint32_t value = entry.first;
    for (const auto &step : steps) {
      Step::Result result = step->apply(value);
      if (result == Step::DONE)
        break;
      if (result == Step::ERROR)
        return false;
      assert(result == Step::NEXT);
    }
    if (value != entry.second)
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
  for (unsigned i = 0; i < 1000; i++) {
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
  std::vector<std::unique_ptr<Step>> steps;
  const unsigned max_stages = 4;
  const auto map = testmap();
  auto lookup = map;
  for (unsigned i = 0; i != max_stages; i++) {
    bool allow_collisions = i + 1 != max_stages;
    reduce_using_crc(lookup, allow_collisions, steps);
  }
  DataType data_type = pick_datatype(lookup);
  steps.push_back(
    std::unique_ptr<Step>(
      new LookupTableStep(std::move(lookup), data_type)));
  print(steps);
  if (!validate(map, steps)) {
    std::cerr << "Failed validation\n";
    std::exit(1);
  }
  return 0;
}

#ifndef PAGETABLE_H
#define PAGETABLE_H

#include <vector>
#include <mutex>
#include <cstdint>
#include "g_std/g_unordered_map.h"
#include "g_std/g_vector.h"


class PageTable {
public:
    using PFN = uint32_t;

    static constexpr size_t PT_LEVEL_BITS = 9;   // 512 entries per level (2^9)
    static constexpr size_t PT_LEVEL_ENTRIES = 1 << PT_LEVEL_BITS; // 512
    static constexpr size_t PAGE_SHIFT = 12;     // 4KB page

    static const uint64_t INVALID_VA;
    static const PFN INVALID_PFN;

    explicit PageTable(uint64_t max_pfn_);
    ~PageTable() = default;

    // 映射一个虚拟地址，返回对应的PFN
    PFN map_page(uint64_t va);

    // 取消映射，返回是否成功
    bool unmap_page(uint64_t va);

    // 查找对应虚拟地址的PFN，成功返回true
    bool lookup_pfn(uint64_t va, PFN &out_pfn) const;

    // 若映射不存在则映射，存在则返回已有PFN
    PFN get_or_map_page(uint64_t va);

private:
    PFN map_page_internal(uint64_t va);
    bool unmap_page_internal(uint64_t va);
    bool lookup_pfn_internal(uint64_t va, PFN &out_pfn) const;
    PFN allocate_pfn_internal();

    uint64_t l3_index(uint64_t va) const;
    uint64_t l2_index(uint64_t va) const;
    uint64_t l1_index(uint64_t va) const;

    uint64_t max_pfn;
    PFN next_free_pfn;

    mutable std::mutex mutex_;

    g_vector<g_vector<g_vector<PFN>>> l3_table;

    g_vector<bool> pfn_in_use;
    g_vector<uint64_t> pfn_to_va_map;
};

#endif // PAGETABLE_H
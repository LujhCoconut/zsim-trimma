#ifndef _MC_H_
#define _MC_H_

#include "config.h"
#include "g_std/g_string.h"
#include "memory_hierarchy.h"
#include <string>
#include "stats.h"
#include "g_std/g_unordered_map.h"
#include "g_std/g_vector.h"
#include<math.h>
#include<stdio.h>
#include<iostream>
#include <array>
#include <cstdint>

#define MAX_STEPS 10000

enum ReqType
{
	LOAD = 0,
	STORE
};


enum Scheme
{
   AlloyCache,
   UnisonCache,
   HMA,
   HybridCache,
   NoCache,
   CacheOnly,
   Tagless,
   BasicCache,
   SDCache,
   Trimma,
};

using PhysicalAddr = Address;
using DeviceAddr = Address;
using BlockID = uint32_t;
using BitVector = uint32_t;

static constexpr uint32_t BITS_PER_LEVEL = 11;    // 每层11位
static constexpr uint32_t CHILDREN_PER_NODE = 1 << BITS_PER_LEVEL; // 2048
static constexpr uint32_t LEVELS = 2;             // 两级结构
static constexpr uint32_t INVALID_INDEX = ~0u;

/**
 * 使用多级表结构的一个主要问题是查找延迟增加。
 * 一个具有 𝐿 级的重映射表在最坏情况下可能引入最多 𝐿 + 1 次的片外访问。
 * 就像操作系统页表依赖 TLB 来加速访问一样，重映射表也需要更高效的缓存机制。
 */

/**
 * @brief NonIdCache查询结果：命中状态和设备地址
 */
struct NonIdLookupResult {
    bool hit;
    DeviceAddr dev_addr;
};

/**
 * @brief IdCache查询结果
 */
struct IdLookupResult {
    bool hit;          // 是否在IdCache中命中
    bool is_identity;  // 是否为恒等映射（仅当hit=true时有效）
};


/**
 * @brief NonIdCache条目：非身份映射的设备地址
 */
class NonIdCacheEntry
{
public:
	uint32_t phy_tag;    // 物理地址标签
	uint64_t dev_addr;   // 设备地址
	bool valid;
};

/**
 * @brief 用于存储与传统方式相同的有效重映射项（非恒等映射）。
 */
class NonIdCache
{
public:
	struct NonIdCacheSet
	{
		g_vector<NonIdCacheEntry> ways; // 每个set 6 个ways
		g_vector<uint32_t> lru_value; // 替换策略就按LRU来吧

		NonIdCacheSet() : ways(6), lru_value(6, 0) { // 默认初始化valid为false
            for (auto& entry : ways) {
                entry = NonIdCacheEntry(); // 显式初始化
            }
        }
	};

	g_vector<NonIdCacheSet> sets; // 2048个

	NonIdCache() : sets(2048){}

	NonIdLookupResult lookup(PhysicalAddr pa) 
	{  // 参数列表仅含pa
        uint32_t set_idx = (pa >> 8) & 0x7FF;
        uint32_t tag = pa >> 19;

        NonIdCacheSet& target_set = sets[set_idx];
        for (int i = 0; i < 6; ++i) {
            if (target_set.ways[i].valid && 
                target_set.ways[i].phy_tag == tag) {
                update_lru(target_set.lru_value, i);
                return {true, target_set.ways[i].dev_addr}; // 返回结构体
            }
        }
        return {false, 0}; // 未命中时返回默认值
    }

	void insert(PhysicalAddr pa, DeviceAddr da)
	{
		uint32_t set_idx = (pa >> 6) & 0x7FF;
		uint32_t tag = pa >> 17;

		NonIdCacheSet& target_set = sets[set_idx];
		int victim_way = find_lru_victim(target_set.lru_value);

		target_set.ways[victim_way] = {tag, da, true};
		update_lru(target_set.lru_value, victim_way);
	}

	void update_lru(g_vector<uint32_t>& counters, int used_way) {
        // 时间戳法更新LRU 
    	for (auto& cnt : counters) ++cnt;
        counters[used_way] = 0;
    }

	int find_lru_victim(const g_vector<uint32_t>& counters) {
        // 手动查找最大值的索引（替代std::max_element）
        uint32_t max_val = counters[0];
        int max_idx = 0;
        for (uint64_t i = 1; i < counters.size(); ++i) {
            if (counters[i] > max_val) {
                max_val = counters[i];
                max_idx = i;
            }
        }
        return max_idx;
    }

	void invalidate(PhysicalAddr pa) 
	{
        uint32_t set_idx = (pa >> 8) & 0x7FF;
        uint32_t tag = pa >> 19;
        
        auto& set = sets[set_idx];
        for (auto& entry : set.ways) {
            if (entry.valid && entry.phy_tag == tag) {
                entry.valid = false; // 简单标记失效
                break;
            }
        }
    }
};

/**
 * @brief 用于过滤被跳过的恒等映射项，并通过更高效的 SRAM 空间利用方式保存这些信息。
 * 		  IdCache条目：8KB超级块的位图（32个256B块）
 * 		  类似于扇区缓存（sector cache）[MASCOTS'00]
 */
class IdCacheEntry
{
public:
	uint32_t super_tag; // 超级块标签（地址的高位部分）
	uint32_t bitmap;  // 32位位图，每bit对应一个块的身份映射状态
	bool valid;
};

class IdCache
{
public:
	struct IdCacheSet{
		g_vector<IdCacheEntry> ways; // 每个set 16ways
		g_vector<uint32_t> access_time; // FIFO

		IdCacheSet() : ways(16),access_time(16,0){}
	};

	g_vector<IdCacheSet> sets; // 256个
	uint32_t timestamp = 0; 

	IdCache() : sets(256){}

 	IdLookupResult lookup(PhysicalAddr pa) {
    // 计算超级块标签和块索引
    const uint32_t kSuperBlockSize = 8192; // 8KB
    const uint32_t kBlockSize = 256;       // 256B
    const uint32_t kBlocksPerSuper = kSuperBlockSize / kBlockSize; // 32

    //计算超级块标签和块索引
    uint32_t super_tag = pa >> 13;         // 8KB对齐，低13位为块内偏移
    uint32_t block_index = (pa >> 8) & 0x1F; // 提取[12:8]作为块索引（0-31）

    //计算set索引
    uint32_t set_idx = hash_function(super_tag) & 0xFF;

    //查找匹配的条目
    IdCacheSet& target_set = sets[set_idx];
    for (int i = 0; i < 16; ++i) {
        if (target_set.ways[i].valid && 
            target_set.ways[i].super_tag == super_tag) {
            // 4. 命中后更新访问时间
            target_set.access_time[i] = ++timestamp;

            // 5. 检查bitmap中对应bit位
            uint32_t bitmap = target_set.ways[i].bitmap;
            bool is_identity = (bitmap & (1 << block_index)) != 0;

            return {true, is_identity}; // 返回具体是否恒等映射
        }
    }

    return {false, false}; // 未命中
}

	void insert(PhysicalAddr pa) {
        const uint32_t kSuperBlockSize = 8192;
        const uint32_t block_idx = (pa % kSuperBlockSize) / 256; // 计算块索引
        
        uint32_t super_tag = pa / kSuperBlockSize;
        uint32_t set_idx = hash_function(super_tag) % 256;
        
        // 查找或创建条目
        auto& set = sets[set_idx];
        int found = -1;
        for (int i=0; i<16; ++i) {
            if (set.ways[i].super_tag == super_tag) {
                found = i;
                break;
            }
        }
        
        if (found == -1) {
            found = find_fifo_victim(set.access_time);
            set.ways[found] = {super_tag, 0, true}; // 初始化新条目
        }
        
        // 设置对应bit位
        set.ways[found].bitmap |= (1 << block_idx);
        set.access_time[found] = ++timestamp;
    }

	void invalidate(PhysicalAddr pa) 
	{
        const uint32_t super_tag = pa >> 13;
        const uint32_t block_idx = (pa >> 8) & 0x1F;
        
        uint32_t set_idx = hash_function(super_tag) & 0xFF;
        auto& set = sets[set_idx];
        
        for (auto& entry : set.ways) {
            if (entry.valid && entry.super_tag == super_tag) {
                entry.bitmap &= ~(1 << block_idx); // 清除对应bit
                if (entry.bitmap == 0) entry.valid = false;
                break;
            }
        }
    }

	uint32_t hash_function(uint32_t key) {
        key = ((key >> 16) ^ key) * 0x45d9f3b;
        key = ((key >> 16) ^ key) * 0x45d9f3b;
        return key >> 16;
    }

	int find_fifo_victim(const g_vector<uint32_t>& access_time) 
	{
		if (access_time.empty()) return -1;
		
		uint32_t min_val = access_time[0];
		int min_idx = 0;
		
		for (uint64_t i = 1; i < access_time.size(); ++i) {
			if (access_time[i] < min_val) {
				min_val = access_time[i];
				min_idx = i;
			}
		}
		return min_idx;
	}
};



/**
 * @brief iRT 为不同的集合（set）使用独立的树结构;
 * 		  Trimma核心数据结构，Radix Tree (per set); ​Radix核心理念​​：将键值按比特位分割，逐层映射到树节点；
 * 		  Trimma 使用 11 位 tag chunk 来构建一个 2048 叉的 radix tree；
 * 		  一个简单的两级 iRT 结构就足以支持多数情况，进一步增加层级并不能带来更多空间节省
 * @author Jiahao Lu @ XMU
 * @attention 对于一个PA，其索引位首先选择该地址所在集合的 tag 根节点; tag_roots_
 * 			  该根节点指向对应的多级重映射表的根 //
 * 			  PA中的 tag 位被划分为多个部分，用于逐级遍历重映射表，直到定位到叶子节点，
 * 			  该节点存储重映射后的block ID，再与block offset拼接生成最终的DA。
 * 			  > iRT查找失败，则DA==PA （未被cache/migrate或未allocate）		  
 * 
 * @todo Q1:set的数量？ fast memory大小 /（组相联度 * 256B）
 */
class iRT {
public:
    struct iRNode {
        bool is_leaf;
        union {
            struct {
                uint32_t allocated_bits[CHILDREN_PER_NODE/32]; // 64 x uint32_t
                uint32_t child_indices[CHILDREN_PER_NODE];      // 2048 entries
            };
            BlockID remapped_id; // 叶子节点使用
        };

        iRNode(bool leaf = false) : is_leaf(leaf) {
            if (!is_leaf) {
                memset(allocated_bits, 0, sizeof(allocated_bits));
                memset(child_indices, 0xFF, sizeof(child_indices)); // 初始化为无效
            }
        }
    };

    struct AddrLayout {
        static constexpr uint32_t OFFSET_BITS = 8;    // 256B块偏移
        static constexpr uint32_t LEVEL_BITS  = 11;   // 每级11位
        static constexpr uint32_t SET_BITS    = 11;   // 2048个集合

        static uint32_t set(PhysicalAddr pa) { 
            return (pa >> (LEVEL_BITS*LEVELS + OFFSET_BITS)) & ((1<<SET_BITS)-1);
        }
        
        static uint32_t tag(PhysicalAddr pa, int level) {
            const uint32_t shift = OFFSET_BITS + (LEVELS-level-1)*LEVEL_BITS;
            return (pa >> shift) & ((1 << LEVEL_BITS)-1);
        }
    };

private:
    std::vector<iRNode> node_pool_;
    std::vector<uint32_t> tag_roots_; // 每个集合的根节点索引

public:
    iRT(int sets) : tag_roots_(sets, INVALID_INDEX) {
        // 预分配所有根节点（中间节点）
        for (auto& root_idx : tag_roots_) {
            root_idx = allocate_node(false); 
        }
    }

    DeviceAddr translate(PhysicalAddr pa) const {
        const uint32_t set_idx = AddrLayout::set(pa);
        if (set_idx >= tag_roots_.size()) return pa;

        uint32_t current_idx = tag_roots_[set_idx];
        if (current_idx == INVALID_INDEX) return pa;

        // 遍历中间层级
        for (uint32_t level = 0; level < LEVELS; ++level) {
            const auto& node = node_pool_[current_idx];
            if (node.is_leaf) {
                // 叶子节点：合成设备地址
                return (node.remapped_id << AddrLayout::OFFSET_BITS) | 
                       (pa & ((1 << AddrLayout::OFFSET_BITS)-1));
            }

            const uint32_t slot = AddrLayout::tag(pa, level);
            if (!check_bit(node.allocated_bits, slot)) return pa;

            current_idx = node.child_indices[slot];
            if (current_idx == INVALID_INDEX) return pa;
        }
        return pa; // 理论上不可达
    }

    void update(PhysicalAddr pa, DeviceAddr da) {
		// 提取da的块ID（移除块内偏移）计算set_index
        const BlockID remapped_block = da >> AddrLayout::OFFSET_BITS;
        const uint32_t set_idx = AddrLayout::set(pa);
        if (set_idx >= tag_roots_.size()) return;
		// 获取当前集合的根节点索引
        uint32_t current_idx = tag_roots_[set_idx];
		// 逐层向下分配或更新节点
        for (uint32_t level = 0; level < LEVELS; ++level) {
            iRNode& node = node_pool_[current_idx];
            const uint32_t slot = AddrLayout::tag(pa, level);
            if (level == LEVELS - 1) { // 叶子节点更新
                node.remapped_id = remapped_block;
                break;
            }// 检查当前槽位是否已分配
            if (!check_bit(node.allocated_bits, slot)) {
				// 动态分配子节点（若是最后一层的前一层，则为叶子节点）
                const bool is_leaf = (level == LEVELS - 2);
                const uint32_t child_idx = allocate_node(is_leaf);
				// 更新位图和子节点索引
                set_bit(node.allocated_bits, slot);
                node.child_indices[slot] = child_idx;

                // 预填充叶子节点的初始值（避免后续查找失败）
                if (is_leaf) {
                    node_pool_[child_idx].remapped_id = remapped_block;
                }
            }
			// 跳转到子节点继续处理
            current_idx = node.child_indices[slot];
        }
    }

private:
    uint32_t allocate_node(bool is_leaf) {
        node_pool_.emplace_back(is_leaf);
        return node_pool_.size() - 1;
    }

    static bool check_bit(const uint32_t* bits, uint32_t pos) {
        return (bits[pos >> 5] >> (pos & 0x1F)) & 0x1;
    }

    static void set_bit(uint32_t* bits, uint32_t pos) {
        bits[pos >> 5] |= (1 << (pos & 0x1F));
    }
};
// class iRT
// {
// public:
// 	struct iRNode{
// 		bool is_leaf;
// 		union{
// 			BitVector* allocated_bits; // 中间节点仅使用2048位向量（bit vector）来表示下一层是否已分配
// 			BlockID remapped_id; // 仅叶子节点存储重映射块 ID
// 		};
// 	};

// 	static constexpr uint32_t BITS_PER_LEVEL = 11;   // 每层11位
// 	static constexpr uint32_t CHILDREN_PER_NODE = 1 << BITS_PER_LEVEL; // 2048
// 	static constexpr uint32_t LEVELS = 2;             // 两级结构

// 	struct AddrLayout {
// 		// 48位地址划分
// 		static constexpr uint32_t OFFSET_BITS = 8;    // 256B块偏移
// 		static constexpr uint32_t LEVEL_BITS  = 11;   // 每级11位
// 		static constexpr uint32_t LEVELS       = 2;    // 两级结构
// 		static constexpr uint32_t SET_BITS     = 11;   // 2048个集合
		
// 		// 地址解码（从高位到低位：SET|TAG_LEVEL0|TAG_LEVEL1|OFFSET）
// 		static uint32_t set(PhysicalAddr pa) { 
// 			return (pa >> (LEVEL_BITS*LEVELS + OFFSET_BITS)) & ((1<<SET_BITS)-1);
// 		}
		
// 		static uint32_t tag(PhysicalAddr pa, int level) {
// 			const uint32_t shift = OFFSET_BITS + (LEVELS-level-1)*LEVEL_BITS;
// 			return (pa >> shift) & ((1 << LEVEL_BITS)-1);
// 		}
// 	};

// 	g_vector<iRNode> node_pool_;
// 	g_vector<uint32_t> tag_roots_; // 根节点索引
// 	static constexpr uint32_t INVALID_INDEX = ~0u;
	
//  	iRT(int sets) : tag_roots_(sets, INVALID_INDEX) {
//         // 预分配根节点（每个set一个）
//         for(auto& root : tag_roots_) {
//             root = allocate_node(false); // 中间节点
//         }
//     }

// 	/**
// 	 * @brief 根据物理地址进行多级查找。首先提取set_index，如果超出范围则返回恒等映射。
// 	 * 		  然后获取根节点的current_idx，如果是无效的，直接返回。
// 	 * 		  然后循环遍历LEVEL_SIZES的层级数减一，因为最后一级是叶节点。
// 	 * 		  在每个层级，检查节点是否是叶节点（可能提前终止循环），然后提取当前层级的slot。
// 	 * 		  检查该slot是否已经分配，如果没有，返回恒等映射;
// 	 * 		  否则计算子节点索引并继续查找。最后处理叶节点，合成设备地址。
// 	 * @example pa=0x12345678： 
// 	 * 		【假设如下】
// 	 * 			| 11-bit Set | 5-bit Level0 | 5-bit Level1 | 8-bit Block Offset |
// 	 * 			| (0x3FF)    | (0x1F)       | (0x1F)       | (0xFF)             |
// 	 * 			 0001 0010 0011 0100 0101 0110 0111 1000
// 	 * 			| 0001 0010 001 | 1 0100 | 0 1011 | ... | 0111 1000 |
// 	 * 		【提取流程】
// 	 * 			set_idx = 0001 0010 001 → 0x113
// 	 * 			→ tag_roots_[0x113]
// 	 * 			Level-0:1 0100 → 0x14
// 	 * 			Level-1:0 1011 → 0x0B
// 	 * 
// 	 */
// 	DeviceAddr translate(PhysicalAddr pa) const 
// 	{
// 		const uint32_t set_idx = AddrLayout::set(pa);
// 		if(set_idx >= tag_roots_.size()) return pa; // 集合范围检查
		
// 		// 获取当前集合的根节点
// 		uint32_t current_idx = tag_roots_[set_idx];
// 		if(current_idx == INVALID_INDEX) return pa;
		
// 		// 遍历中间层级
// 		for(uint32_t level = 0; level < AddrLayout::LEVELS; ++level) {
// 			if(current_idx >= node_pool_.size()) return pa;
			
// 			const auto& node = node_pool_[current_idx];
// 			if(node.is_leaf) break; // 提前到达叶节点
			
// 			// 计算当前层级的slot索引
// 			const uint32_t slot = AddrLayout::tag(pa, level);
// 			if(!check_bit(node.allocated_bits, slot)) 
// 				return pa; // 未分配则返回恒等映射
				
// 			// 计算子节点索引（假设每个bitvec单元存储32个子节点指针）
// 			const uint32_t vec_index = slot / 32;
// 			const uint32_t bit_offset = slot % 32;
// 			current_idx = (node.allocated_bits[vec_index] >> bit_offset) & 0xFFFF; // 16-bit子节点索引
// 		}
		
// 		// 处理叶节点
// 		const auto& leaf = node_pool_[current_idx];
// 		return leaf.is_leaf ? 
// 			(leaf.remapped_id << AddrLayout::OFFSET_BITS) | (pa & ((1<<AddrLayout::OFFSET_BITS)-1))
// 			: pa;
// 	}

// 	void update(PhysicalAddr pa, DeviceAddr da) {  // 修改参数列表
// 		const uint32_t set_idx = AddrLayout::set(pa);
// 		if(set_idx >= tag_roots_.size()) return;

// 		const BlockID remapped_block = da >> AddrLayout::OFFSET_BITS; // 从da解析块ID
// 		uint32_t current_idx = tag_roots_[set_idx];
// 		g_vector<uint32_t*> modified_bits;
// 		for(int level = 0; level < LEVELS; ++level)
// 		{
// 			const uint32_t slot = AddrLayout::tag(pa, level);
// 			iRNode& node = node_pool_[current_idx];
				
// 			if(level == LEVELS-1) 
// 			{ // 叶节点直接更新
// 				node.remapped_id = remapped_block;
// 				break;
// 			}

// 				// 中间节点操作（无需动态分配）
// 			if(!check_bit(node.allocated_bits, slot)) 
// 			{
// 				const uint32_t child_idx = allocate_node(level == LEVELS-2);
// 				set_bit(node.allocated_bits, slot);
// 				node.child_indices[slot] = child_idx;
// 			}
				
// 			current_idx = node.child_indices[slot];
// 		}
// 	}
// private:
//     /**
// 	 * @brief 检查第`pos`个节点是否已经分配
// 	 */
//     bool check_bit(uint32_t* bits, uint32_t pos) const {
//         return (bits[pos >> 5] >> (pos& 0x1F)) & 0x1; // <=> (bits[pos/32] >> (pos%32))  & 0x1
//     }
    
//     /**
// 	 * @brief 节点分配
// 	 */
//     uint32_t allocate_node(bool is_leaf) 
// 	{
// 		constexpr uint32_t BITVEC_SIZE = CHILDREN_PER_NODE/32;
		
// 		iRNode node;
// 		node.is_leaf = is_leaf;
// 		if(!is_leaf) {
// 			// 使用对齐分配（64字节对齐）
// 			node.allocated_bits = static_cast<uint32_t*>(
// 				aligned_alloc(64, BITVEC_SIZE*sizeof(uint32_t))
// 			);
// 			memset(node.allocated_bits, 0, BITVEC_SIZE*sizeof(uint32_t));
// 		}
		
// 		node_pool_.push_back(node);
// 		return node_pool_.size()-1;
// 	}

// 	// 设置bit vector中的对应slot为子节点索引
//     void set_bit(uint32_t* bits, uint32_t slot, uint32_t child_idx) {
//         const uint32_t word = slot / 32;
//         const uint32_t offset = slot % 32;
//         bits[word] |= (1 << offset);       // Mark as allocated
//         // 假设child_idx低11位存储子节点位置（需根据实际布局调整）
//         bits[word] |= (child_idx << 11);   // 根据实际存储结构调整
//     }
    
//     // 获取子节点在pool中的索引
//     uint32_t get_child_index(const iRNode& node, uint32_t slot) const {
//         const uint32_t word = slot / 32;
//         const uint32_t offset = slot % 32;
//         return (node.allocated_bits[word] >> 11) & 0x7FF; // 提取11位索引
//     }
// };

class Way
{
public:
   Address tag;
   bool valid;
   bool dirty;
   uint64_t lru_value;

   g_vector<bool> valid_vector;
   g_vector<bool> dirty_vector;

   void cleanVector()
   {
	  for(uint64_t i = 0; i < valid_vector.size(); i++)
	  {
		valid_vector[i]=false;
		dirty_vector[i]=false;
	  }
   }
};

class Set
{
public:
   Way * ways;
   uint32_t num_ways;

   uint32_t getEmptyWay()
   {
      for (uint32_t i = 0; i < num_ways; i++)
         if (!ways[i].valid)
            return i;
      return num_ways;
   };
   bool hasEmptyWay() { return getEmptyWay() < num_ways; };

   uint32_t findLRUEvictWay()
   {
		if(hasEmptyWay())return getEmptyWay();
		uint64_t max_value = 0;
		uint32_t evict_way_idx = 0;
		for(uint32_t i = 0 ;i < num_ways;i++)
		{
			if(ways[i].lru_value > max_value)
			{
				max_value = ways[i].lru_value;
				evict_way_idx = i;
			}
		}
		return evict_way_idx;
   };

   void updateLRUState(uint32_t way_idx)
   {
		for(uint32_t i=0 ; i < num_ways ;i++)
		{
			if(i != way_idx && ways[i].valid)
			{
				ways[i].lru_value += 1;
			}
		}
		ways[way_idx].lru_value = 0;
		ways[way_idx].valid = true;
   };
};


// Not modeling all details of the tag buffer. 
class TagBufferEntry
{
public:
	Address tag;
	bool remap;
	uint32_t lru;
};

class TagBuffer : public GlobAlloc {
public:
	TagBuffer(Config &config);
	// return: exists in tag buffer or not.
	uint32_t existInTB(Address tag);
	uint32_t getNumWays() { return _num_ways; };

	// return: if the address can be inserted to tag buffer or not.
	bool canInsert(Address tag);
	bool canInsert(Address tag1, Address tag2);
	void insert(Address tag, bool remap);
	double getOccupancy() { return 1.0 * _entry_occupied / _num_ways / _num_sets; };
	void clearTagBuffer();
	void setClearTime(uint64_t time) { _last_clear_time = time; };
	uint64_t getClearTime() { return _last_clear_time; };
private:
	void updateLRU(uint32_t set_num, uint32_t way);
	TagBufferEntry ** _tag_buffer;
	uint32_t _num_ways;
	uint32_t _num_sets;
	uint32_t _entry_occupied;
	uint64_t _last_clear_time;
};

class TLBEntry
{
public:
   uint64_t tag;
   uint64_t way;
   uint64_t count; // for OS based placement policy

   // the following two are only for UnisonCache
   // due to space cosntraint, it is not feasible to keep one bit for each line, 
   // so we use 1 bit for 4 lines.
   uint64_t touch_bitvec; // whether a line is touched in a page
   uint64_t dirty_bitvec; // whether a line is dirty in page
};

class LinePlacementPolicy;
class PagePlacementPolicy;
class OSPlacementPolicy;

//class PlacementPolicy;
class DDRMemory;

class MemoryController : public MemObject {
private:
	DDRMemory * BuildDDRMemory(Config& config, uint32_t frequency, uint32_t domain, g_string name, const std::string& prefix, uint32_t tBL, double timing_scale);
	
	g_string _name;

	// Trace related code
	lock_t _lock;
	lock_t _page_lock;
	bool _collect_trace;
	g_string _trace_dir;
	Address _address_trace[10000];
	uint32_t _type_trace[10000];
	uint32_t _cur_trace_len;
	uint32_t _max_trace_len;

	// External Dram Configuration
	MemObject *	_ext_dram;
	g_string _ext_type; 
public:	
	// MC-Dram Configuration
	MemObject ** _mcdram;
	uint32_t _mcdram_per_mc;
	g_string _mcdram_type;

	uint32_t _fm_size;
	uint32_t _set_assoc;
	g_vector<SDTree> sdtrees;
	

	// Trimma
	uint32_t block_size; // paper default 256GB
	uint32_t set_assoc; // CLI接收num_ways
	uint32_t irt_levels; // default 3
	NonIdCache nonIdCache;
	IdCache idCache;

	iRT _iRT;
	IdCache _idCache;
	NonIdCache _nonIdCache;

	// <set_id,tag,way>
	struct Triplet {
		uint64_t first;
		uint64_t second;
		uint64_t third;
	};
	g_vector<Triplet> _basic_tag_buffer;

	uint64_t getNumRequests() { return _num_requests; };
   	uint64_t getNumSets()     { return _num_sets; };
   	uint32_t getNumWays()     { return _num_ways; };
   	double getRecentMissRate(){ return (double) _num_miss_per_step / (_num_miss_per_step + _num_hit_per_step); };
   	Scheme getScheme()      { return _scheme; };
   	Set * getSets()         { return _cache; };
   	g_unordered_map<Address, TLBEntry> * getTLB() { return &_tlb; };
	TagBuffer * getTagBuffer() { return _tag_buffer; };

	uint64_t getGranularity() { return _granularity; };

private:
	// For Alloy Cache.
	Address transMCAddress(Address mc_addr);
	// For Page Granularity Cache
	Address transMCAddressPage(uint64_t set_num, uint32_t way_num); 

	// For Tagless.
	// For Tagless, we don't use "Set * _cache;" as other schemes. Instead, we use the following 
	// structure to model a fully associative cache with FIFO replacement 
	//vector<Address> _idx_to_address;
	uint64_t _next_evict_idx;
	//map<uint64_t, uint64_t> _address_to_idx;

	// Cache structure
	uint64_t _granularity;
	uint64_t _num_ways;
	uint64_t _cache_size;  // in Bytes
	uint64_t _num_sets;

	int current_tagbuffer_idx;
	bool is_ideal;

	Set * _cache;
	LinePlacementPolicy * _line_placement_policy;
	PagePlacementPolicy * _page_placement_policy;
	OSPlacementPolicy * _os_placement_policy;
	uint64_t _num_requests;
	Scheme _scheme; 
	TagBuffer * _tag_buffer;
	
	// For HybridCache
	uint32_t _footprint_size; 

	// Balance in- and off-package DRAM bandwidth. 
	// From "BATMAN: Maximizing Bandwidth Utilization of Hybrid Memory Systems"
	bool _bw_balance; 
	uint64_t _ds_index;

	// TLB Hack
	g_unordered_map <Address, TLBEntry> _tlb;
	uint64_t _os_quantum;

    // Stats
	Counter _numPlacement;
  	Counter _numCleanEviction;
	Counter _numDirtyEviction;
	Counter _numLoadHit;
	Counter _numLoadMiss;
	Counter _numStoreHit;
	Counter _numStoreMiss;
	Counter _numCounterAccess; // for FBR placement policy  

	Counter _numTagLoad;
	Counter _numTagStore;
	// For HybridCache	
	Counter _numTagBufferFlush;
	Counter _numTBDirtyHit;
	Counter _numTBDirtyMiss;
	// For UnisonCache
	Counter _numTouchedLines;
	Counter _numEvictedLines;

	Counter invalid_data_size;
	Counter valid_data_size;
	Counter migrate_data_size;
	Counter policy_update_size;
	Counter _numTotalHit;
	Counter _numTotalMiss;



	uint64_t _num_hit_per_step;
   	uint64_t _num_miss_per_step;
	uint64_t _mc_bw_per_step;
	uint64_t _ext_bw_per_step;
   	double _miss_rate_trace[MAX_STEPS];

   	uint32_t _num_steps;

	// to model the SRAM tag
	bool 	_sram_tag;
	uint32_t _llc_latency;
public:
	MemoryController(g_string& name, uint32_t frequency, uint32_t domain, Config& config);
	uint64_t access(MemReq& req);
	uint64_t unison_cache_access(MemReq& req);
	uint64_t ideal_unison_access(MemReq& req);
	uint64_t basic_cache_access(MemReq& req);	
	uint64_t theoretical_basic_cache_access(MemReq& req);
	uint64_t test_cache_access(MemReq& req);
	uint64_t ideal_cache_access(MemReq& req);
	uint64_t sdcache_access(MemReq& req);
	uint64_t trimma_access(MemReq& req);
	const char * getName() { return _name.c_str(); };
	void initStats(AggregateStat* parentStat); 
	
	/**
	 * @brief 求log2x
	 */
	uint32_t log2_uint32(uint32_t x) 
	{
		if (x == 0) return 0; 
		return 31 - __builtin_clz(x);
	}
	// Use glob mem
	//using GlobAlloc::operator new;
	//using GlobAlloc::operator delete;
};

#endif
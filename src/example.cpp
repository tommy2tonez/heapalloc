
#include "heap.h"
#include <iostream>
#include "assert.h"
#include <random>

class BumpAllocator{ //REVIEW: very bad allocator (as placeholder for future reservoir implementation) 

    private:

        std::vector<bool> bc;
        size_t sz;
    
    public:
        
        BumpAllocator(std::vector<bool> bc, size_t sz): bc(std::move(bc)), sz(sz){}

        std::optional<std::pair<size_t, size_t>> seek_next_from(size_t offs){
            
            size_t first = offs; 

            while (first < sz && !bc[first]){
                ++first;
            }

            if (first >= sz){
                return std::nullopt;
            }

            size_t last = first + 1;

            while (last < sz && bc[last]){
                ++last;
            }

            return std::pair<size_t, size_t>{first, last - first};

        } 

        bool has_block(size_t block_sz){

            size_t offs = 0;

            while (true){

                auto nnext  = seek_next_from(offs);

                if (!nnext){
                    return false;
                }

                if (nnext.value().second >= block_sz){
                    return true;
                } 

                offs = nnext.value().first + nnext.value().second;

            }

        }

        void block(size_t offs, size_t block_sz){

            for (size_t i = offs; i < offs + block_sz; ++i){
                bc[i] = false;
            }

        }

        void unblock(size_t offs, size_t block_sz){

            for (size_t i = offs; i < offs + block_sz; ++i){
                bc[i] = true;
            }

        }

        bool is_empty(size_t offs, size_t block_sz){

            for (size_t i = offs; i < offs + block_sz; ++i){
                if (!bc[i]){
                    return false;
                }
            }

            return true;
        }

};

using interval_type             = dg::heap::types::interval_type;

// auto make_dealloc_wrapped(interval_type intv, BumpAllocator& b_allocator, dg::heap::core::Allocatable& allocator) -> std::shared_ptr<interval_type>{

//     auto destructor = [&](interval_type * intv){
//         b_allocator.unblock(intv->first, intv->second + 1);
//         allocator.free(*intv);
//         delete intv;
//     };

//     std::unique_ptr<interval_type, decltype(destructor)> ins{new interval_type(intv), destructor};
//     return ins;

// }

void clear(std::vector<interval_type>& intvs, dg::heap::core::Allocatable& allocatable, BumpAllocator& ballocator){

    for (const auto& intv: intvs){
        allocatable.free(intv);
        ballocator.unblock(intv.first, intv.second + 1);
    }

    intvs.clear();
}

void random_clear(std::vector<interval_type>& intvs, dg::heap::core::Allocatable& allocatable, BumpAllocator& ballocator){

    static auto random_device   = std::bind(std::uniform_int_distribution<size_t>{}, std::mt19937{});
    const size_t CLEAR_SZ       = random_device() % intvs.size();

    for (size_t i = 0; i < CLEAR_SZ; ++i){
        
        size_t rm_idx   = random_device() % intvs.size();
        size_t last_idx = intvs.size() - 1;

        std::swap(intvs[rm_idx], intvs[last_idx]);
        allocatable.free(intvs.back());
        ballocator.unblock(intvs.back().first, intvs.back().second + 1);

        intvs.pop_back();
    }
}

int main(){

    constexpr size_t HEIGHT         = 11;
    constexpr size_t BASE_LENGTH    = size_t{1} << (HEIGHT - 1);

    std::vector<bool> bc{};
    bc.resize(BASE_LENGTH, true);
    BumpAllocator b_allocator(std::move(bc), BASE_LENGTH);

    std::shared_ptr<char[]> buf = dg::heap::user_interface::make(HEIGHT);
    std::shared_ptr<dg::heap::core::Allocatable> allocator  = dg::heap::user_interface::get_allocator_x(buf.get());
    
    auto random_device              = std::bind(std::uniform_int_distribution<size_t>{}, std::mt19937{});
    std::vector<interval_type> intvs{};

    while (true){

        size_t block_sz = (random_device() % BASE_LENGTH) + 1;
        std::optional<interval_type> intv = allocator->alloc(block_sz);
        
        if (!bool{intv} && b_allocator.has_block(block_sz)){
            
            allocator  = dg::heap::user_interface::get_allocator_x(buf.get());
            intv = allocator->alloc(block_sz);

            if (!bool{intv} && b_allocator.has_block(block_sz)){

                std::cout << "mayday" << std::endl;
                assert(false);
            }
        } 

        if (!bool{intv}){
            random_clear(intvs, *allocator, b_allocator);
            continue;
        }

        if (bool{intv} && !b_allocator.is_empty(intv->first, intv->second + 1)){
            std::cout << "mayday" << std::endl;
            assert(false);
        }

        b_allocator.block(intv->first, intv->second + 1);
        intvs.push_back(intv.value());
    }
}
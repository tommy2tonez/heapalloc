#ifndef __DG_HEAP_ALLOCATOR__
#define __DG_HEAP_ALLOCATOR__ 

#include <stdint.h>
#include <stdlib.h>
#include <memory>
#include "crtpboolvector.h"
#include "assert.h"
#include <tuple>
#include <limits>
#include <vector>
#include "dense_hash_map/dense_hash_map.hpp"
#include <optional>
#include <exception>
#include <numeric>
#include <bit>
#include <queue>
#include <ratio>
#include <mutex>
#include <cstring>
#include "serialization.h"

#ifdef __cpp_consteval
#define _DG_CONSTEVAL consteval
#else // ^^^ supports consteval / no consteval vvv
#define _DG_CONSTEVAL constexpr
#endif // ^^^ no consteval ^^^

//interface

namespace dg::heap::limits{

    static inline const auto MIN_HEAP_HEIGHT         = uint8_t{12};
    static inline const auto MAX_HEAP_HEIGHT         = uint8_t{12};
    static inline const auto EXCL_MAX_HEAP_HEIGHT    = uint8_t{MAX_HEAP_HEIGHT + 1};
}

namespace dg::heap::precond{

    static_assert(std::endian::native == std::endian::little); //REVIEW: to-be-changed 
}

namespace dg::heap::types{

    using traceback_type    = size_t;
    using store_type        = uint32_t;
    using interval_type     = std::pair<store_type, store_type>;

    struct Node{
        store_type l;
        store_type r;
        store_type c;
        store_type o;
    };

    using cache_type        = Node;
}

namespace dg::heap::traceback_policy{
    
    using traceback_type = types::traceback_type; 

    static constexpr auto LEFT_TRACEBACK             = traceback_type{0b00};
    static constexpr auto RIGHT_TRACEBACK            = traceback_type{0b01};
    static constexpr auto MID_TRACEBACK              = traceback_type{0b10};
    static constexpr auto MID_BLOCKED                = traceback_type{0b11};
    static constexpr auto UNBLOCKED                  = traceback_type{0b0};
    static constexpr auto BLOCKED                    = traceback_type{0b1}; 

    static constexpr auto LEFT_TRACEBACK_IC          = std::integral_constant<traceback_type, LEFT_TRACEBACK>();
    static constexpr auto RIGHT_TRACEBACK_IC         = std::integral_constant<traceback_type, RIGHT_TRACEBACK>();
    static constexpr auto MID_TRACEBACK_IC           = std::integral_constant<traceback_type, MID_TRACEBACK>();
    static constexpr auto MID_BLOCKED_IC             = std::integral_constant<traceback_type, MID_BLOCKED>();
    static constexpr auto UNBLOCKED_IC               = std::integral_constant<traceback_type, UNBLOCKED>();
    static constexpr auto BLOCKED_IC                 = std::integral_constant<traceback_type, BLOCKED>();  

    static constexpr auto L_BIT_SPACE                = uint8_t{1};
    static constexpr auto R_BIT_SPACE                = uint8_t{1};
    static constexpr auto C_BIT_SPACE                = uint8_t{2};
    static constexpr auto BL_BIT_SPACE               = uint8_t{1};

    static constexpr auto L_OFFSET                   = uint8_t{0};
    static constexpr auto R_OFFSET                   = uint8_t{L_OFFSET + L_BIT_SPACE};
    static constexpr auto C_OFFSET                   = uint8_t{R_OFFSET + R_BIT_SPACE};
    static constexpr auto NB_BL_OFFSET               = uint8_t{R_OFFSET + R_BIT_SPACE}; 
    static constexpr auto BB_BL_OFFSET               = uint8_t{0};

    static constexpr auto STD_BUCKET_LENGTH          = uint8_t{L_BIT_SPACE + R_BIT_SPACE + C_BIT_SPACE};
    static constexpr auto NEXT_BASE_BUCKET_LENGTH    = uint8_t{L_BIT_SPACE + R_BIT_SPACE + BL_BIT_SPACE};
    static constexpr auto BASE_BUCKET_LENGTH         = uint8_t{BL_BIT_SPACE};
}

namespace dg::heap::memory{

    class Allocatable{

        public:

            virtual ~Allocatable() noexcept{}
            virtual char * malloc(size_t) noexcept = 0;
            virtual void free(void *, size_t) noexcept = 0;
    };
}

namespace dg::heap::data{ //model

    template <class T>
    class HeapData{

        public:

            static inline const size_t DYNAMIC_HEIGHT       = T::DYNAMIC_HEIGHT;
            static inline const size_t TRACEBACK_HEIGHT     = T::TRACEBACK_HEIGHT;
            static inline const size_t HEIGHT               = T::HEIGHT;

            static auto& get_boolvector_container() noexcept{
            
                return T::get_boolvector_container();
            }

            static auto& get_node_container() noexcept{

                return T::get_node_container();
            }

            static auto& get_cache_instance() noexcept{

                return T::get_cache_instance();
            }

            auto to_heap_data() const noexcept{

                return this;
            }
    };

    template <class T>
    class StorageExtractible{

        public:
        
            using store_type = types::store_type; 

            static constexpr size_t TREE_HEIGHT  = T::TREE_HEIGHT; 

            template <size_t HEIGHT>
            static store_type get_left_at(size_t idx) noexcept{

                return T::template get_left_at<HEIGHT>(idx);
            }

            template <size_t HEIGHT>
            static store_type get_right_at(size_t idx) noexcept{

                return T::template get_right_at<HEIGHT>(idx);
            }

            template <size_t HEIGHT>
            static store_type get_center_at(size_t idx) noexcept{

                return T::template get_center_at<HEIGHT>(idx);
            }

            template <size_t HEIGHT>
            static store_type get_offset_at(size_t idx) noexcept{

                return T::template get_offset_at<HEIGHT>(idx);
            }

            auto to_storage_extractible() const noexcept{

                return this;
            }
    };
}

namespace dg::heap::cache{
     
    template <class T>
    class CacheControllable{
        
        public:
        
            using cache_type = types::cache_type;

            const cache_type& get(size_t key) const noexcept{

                return static_cast<const T *>(this)->get(key);
            }

            void set(size_t key, const cache_type& val) noexcept{

                static_cast<T *>(this)->set(key, val);
            }

            CacheControllable * to_cache_controllable() noexcept{

                return this;
            }
    };
}

namespace dg::heap::market{

    template <class T>
    class Buyable{

        public:

            using store_type        = types::store_type;
            using interval_type     = types::interval_type;

            std::optional<interval_type> buy(store_type sz) noexcept{
                
                return static_cast<T *>(this)->buy(sz);
            }

            auto to_buyable() noexcept{
                
                return this;
            }
    };

    template <class T>
    class Sellable{ 

        public:

            using store_type    = types::store_type;
            using interval_type = types::interval_type;

            bool sell(const interval_type& intv) noexcept{

                return static_cast<T *>(this)->sell(intv);
            }

            auto to_sellable() noexcept{

                return this;
            }
    };
}

namespace dg::heap::seeker{

    template <class T>
    class Seekable{

        public:

            using interval_type = types::interval_type;

            std::optional<interval_type> seek(size_t idx) noexcept{

                return static_cast<T *>(this)->seek(idx);
            }

            template <size_t Val>
            std::optional<interval_type> seek(const std::integral_constant<size_t, Val>& idx) noexcept{

                return static_cast<T *>(this)->seek(idx);
            }

            auto to_seekable() noexcept{

                return this;
            }
    };
}

namespace dg::heap::dispatcher{

    template <class T>
    class Dispatchable{

        public:

            using interval_type = types::interval_type; 
            
            void dispatch(const interval_type& intv) noexcept{

                static_cast<T *>(this)->dispatch(intv);
            }

            auto to_dispatchable() noexcept{

                return this;
            }
    };

    template <class T>
    class BatchDispatchable{

        public:

            using interval_type  = types::interval_type;

            template <class Iterator>
            void dispatch(Iterator first, Iterator last) noexcept{

                static_cast<T *>(this)->dispatch(first, last);
            } 

            auto to_dispatchable() noexcept{

                return this;
            }
    };
}

namespace dg::heap::internal_core{

    template <class T>
    class HeapOperatable{ //-- MV(C)

        public:

            static constexpr size_t HEIGHT = T::HEIGHT;

            template <size_t HEIGHT>
            static void update(size_t idx) noexcept{
                
                T::template update<HEIGHT>(idx);
            }

            template <size_t HEIGHT>
            static void block(size_t idx) noexcept{
                
                T::template block<HEIGHT>(idx);
            }

            template <size_t HEIGHT>
            static void unblock(size_t idx) noexcept{
                
                T::template unblock<HEIGHT>(idx);
            }

            template <size_t HEIGHT>
            static bool is_blocked(size_t idx) noexcept{
                
                return T::template is_blocked<HEIGHT>(idx); 
            }

            HeapOperatable * to_heap_operatable() const noexcept{

                return this;
            }
    };

    template <class T, bool HAS_EXCEPT>
    class Allocatable{

        public:

            using store_type        = types::store_type; 
            using interval_type     = types::interval_type; 

            std::optional<interval_type> alloc(store_type sz) noexcept(HAS_EXCEPT){

                return static_cast<T *>(this)->alloc(sz);
            }

            void free(const interval_type& intv) noexcept(HAS_EXCEPT){

                static_cast<T *>(this)->free(intv);
            } 

            auto to_allocatable() noexcept{

                return this;
            }
    };

    template <class T>
    using NoExceptAllocatable   = Allocatable<T, true>;

    template <class T>
    using ExceptAllocatable     = Allocatable<T, false>;

    template <class T>
    class HeapShrinkable{

        public:

            using store_type    = types::store_type; 

            void shrink(store_type virtual_base) noexcept{

                static_cast<T *>(this)->shrink(virtual_base);
            }

            store_type shrink() noexcept{

                return static_cast<T *>(this)->shrink();
            }

            void unshrink(store_type virtual_base) noexcept{

                static_cast<T *>(this)->unshrink(virtual_base);
            }

            auto to_heap_shrinkable() noexcept{
                
                return this;
            }
    };
}

namespace dg::heap::core{
    
    class Allocatable{
        
        public:

            using store_type        = types::store_type; 
            using interval_type     = types::interval_type; 

            virtual ~Allocatable() noexcept{}
            virtual std::optional<interval_type> alloc(store_type) noexcept = 0;
            virtual void free(const interval_type&) noexcept = 0;

    };

    class HeapShrinkable{

        public:

            using store_type    = types::store_type; 

            virtual ~HeapShrinkable() noexcept{}
            virtual void shrink(store_type) noexcept = 0;
            virtual store_type shrink() noexcept = 0;
            virtual void unshrink(store_type) noexcept = 0;

    };

    class Allocatable_X: public virtual Allocatable,
                         public virtual HeapShrinkable{};

}

// -- done interface 
namespace dg::heap::essentials{

    template <class = void>
    static constexpr bool FALSE_VAL = false;

    template <class Functor, class Tup, size_t ...IDX>
    static constexpr auto piecewise_invoke(Functor&& functor, Tup&& tup, const std::index_sequence<IDX...>&) -> decltype(auto){
    
        return functor(std::get<IDX>(tup)...);
    } 

    template <class Functor, class Arg>
    static constexpr auto piecewise_invoke(Functor&& functor, Arg&& arg) -> decltype(auto){

        using rm_ref_type   = typename std::remove_reference_t<Arg>;
        const auto idx_seq  = std::make_index_sequence<std::tuple_size_v<rm_ref_type>>{};

        return piecewise_invoke(std::forward<Functor>(functor), std::forward<Arg>(arg), idx_seq);
    }
    
    template <class Functor, class Arg>
    static constexpr auto piecewise_void_invoke(Functor&& functor, Arg&& arg) -> void *{

        using rm_ref_type   = typename std::remove_reference_t<Arg>;
        const auto idx_seq  = std::make_index_sequence<std::tuple_size_v<rm_ref_type>>{}; 

        [&]<size_t ...IDX>(const std::index_sequence<IDX...>&){
            functor(std::get<IDX>(arg)...);
        }(idx_seq);

        return {};
    }

    template <class Tup, class TransformLambda>
    static constexpr auto transform(Tup&& tup, const TransformLambda& transformer){

        using base_type     = typename std::remove_cv_t<std::remove_reference_t<Tup>>; //
        const auto idx_seq  = std::make_index_sequence<std::tuple_size_v<base_type>>();

        auto rs = [&]<size_t ...IDX>(const std::index_sequence<IDX...>&){
            return base_type{transformer(std::get<IDX>(tup))...};
        };

        return rs(idx_seq);
    }

    template <class Lambda, class Tup>
    static constexpr bool is_no_except_invokable(){

        constexpr auto idx_seq  = std::index_sequence<std::tuple_size_v<Tup>>();
        constexpr auto test_lambda = []<size_t ...IDX>(std::index_sequence<IDX...>){
            return noexcept(std::declval<Lambda>(std::get<IDX>(std::declval<Tup>())...));
        };

        return test_lambda(idx_seq);
    }
};

namespace dg::heap::types_space{

    template <class T>
    struct is_storage_extractible_intf: std::false_type{};

    template <class T>
    struct is_storage_extractible_intf<data::StorageExtractible<T>>: std::true_type{};

    template <class T>
    struct is_cache_controllable_intf: std::false_type{};

    template <class T>
    struct is_cache_controllable_intf<cache::CacheControllable<T>>: std::true_type{};

    template <class T>
    struct is_heap_operatable_intf: std::false_type{};

    template <class T>
    struct is_heap_operatable_intf<internal_core::HeapOperatable<T>>: std::true_type{}; 

    template <class T, class = void>
    struct is_std_template_container: std::false_type{};

    template <class T>
    struct is_std_template_container<T, std::void_t<decltype(std::tuple_size_v<T>)>>: std::true_type{};
    
    template <class T>
    struct is_integral_constant: std::false_type{};
    
    template <class T, T Val>
    struct is_integral_constant<std::integral_constant<T, Val>>: std::true_type{};

    template <class T, class = void>
    struct is_integer_pair: std::false_type{};

    template <class T, class T1>
    struct is_integer_pair<std::pair<T, T1>, std::void_t<typename std::enable_if_t<std::numeric_limits<T>::is_integer>, 
                                                         typename std::enable_if_t<std::numeric_limits<T1>::is_integer>>>: std::true_type{};

    template <class T, class = void>
    struct is_bidirectional_iterator: std::false_type{};

    template <class _IterType>
    struct is_bidirectional_iterator<_IterType, std::void_t<decltype(std::prev(std::declval<_IterType>()))>>: std::true_type{};
    
    template <class T, class = void>
    struct has_storage_extractible_intf: std::false_type{};

    template <class T>
    struct has_storage_extractible_intf<T, std::void_t<decltype(std::declval<T>().to_storage_extractible())>>: is_storage_extractible_intf<typename std::remove_pointer_t<decltype(std::declval<T>().to_storage_extractible())>>{};

    template <class T, class = void>
    struct has_cache_controllable_intf: std::false_type{};

    template <class T>
    struct has_cache_controllable_intf<T, std::void_t<decltype(std::declval<T>().to_cache_controllable())>>: is_cache_controllable_intf<typename std::remove_pointer_t<decltype(std::declval<T>().to_cache_controllable())>>{};

    template <class T, class = void>
    struct has_heap_operatable_intf: std::false_type{};

    template <class T>
    struct has_heap_operatable_intf<T, std::void_t<decltype(std::declval<T>().to_heap_operatable())>>: is_heap_operatable_intf<typename std::remove_pointer_t<decltype(std::declval<T>().to_heap_operatable())>>{};

    template <class T, class = void>
    struct has_equal_operator: std::false_type{};

    template <class T>
    struct has_equal_operator<T, std::void_t<typename std::is_same<decltype(std::declval<T>() == std::declval<T>()), bool>::type>>: std::true_type{}; //REVIEW: assumption
    
    template <class T, class = void>
    struct has_not_equal_operator: std::false_type{};

    template <class T>
    struct has_not_equal_operator<T, std::void_t<typename std::is_same<decltype(std::declval<T>() != std::declval<T>()), bool>::type>>: std::true_type{};

    template <class _Ty>
    struct nillable{
        using type  = std::optional<_Ty>;
    };
    
    template <class _Ty>
    struct nillable<std::shared_ptr<_Ty>>{
        using type  = std::shared_ptr<_Ty>;
    };

    template <class _Ty>
    struct nillable<std::unique_ptr<_Ty>>{
        using type  = std::unique_ptr<_Ty>;
    };

    template <class _Ty>
    struct nillable<_Ty *>{
        using type = std::add_pointer_t<_Ty>;
    };

    template <class T>
    static constexpr bool has_no_padding_v               = std::disjunction_v<std::has_unique_object_representations<T>, std::is_same<T, float>, std::is_same<T, double>>; 

    template <class T>
    static constexpr bool has_storage_extractible_intf_v = has_storage_extractible_intf<T>::value;

    template <class T>
    static constexpr bool has_cache_controllable_intf_v  = has_cache_controllable_intf<T>::value;

    template <class T>
    static constexpr bool has_heap_operatable_intf_v     = has_heap_operatable_intf<T>::value;
    
    template <class T>
    static constexpr bool has_equal_operator_v           = has_equal_operator<T>::value;

    template <class T>
    static constexpr bool has_not_equal_operator_v       = has_not_equal_operator<T>::value;

    template <class T>
    static constexpr bool is_std_template_container_v    = is_std_template_container<T>::value;

    template <class T>
    static constexpr bool is_integer_pair_v              = is_integer_pair<T>::value;

    template <class T>
    static constexpr bool is_integral_constant_v         = is_integral_constant<T>::value;

    template <class T>
    static constexpr bool is_bidirectional_iter_v        = is_bidirectional_iterator<T>::value;

    template <class LambdaType, class Tup>
    static constexpr bool is_void_lambda_v               = std::is_same_v<decltype(essentials::piecewise_invoke(std::declval<LambdaType>(), std::declval<Tup>())), void>;

    template <class T>
    using base_type     = typename std::remove_cv<typename std::remove_reference<T>::type>::type;
    
    template <class T>
    using nillable_t    = typename nillable<T>::type;

};

namespace dg::heap::utility{

    using namespace essentials;

    struct TypeUtility{
        
        template <class Object>
        static constexpr auto has_storage_extractible_base() -> bool {
            return types_space::has_storage_extractible_intf_v<Object>;
        } 

        template <class Object>
        static constexpr auto has_cache_controllable_base() -> bool {
            return types_space::has_cache_controllable_intf_v<Object>;
        }
        
        template <class Object>
        static constexpr auto has_heap_operatable_base() -> bool {
            return types_space::has_heap_operatable_intf_v<Object>;
        }

        template <class T>
        static constexpr auto is_integer_pair() -> bool{
            return types_space::is_integer_pair_v<T>;
        }
    };

    struct NumericUtility{

        static constexpr auto ADD_OPS    = std::plus<void>{};
        static constexpr auto SUB_OPS    = std::minus<void>{};
        static constexpr auto MUL_OPS    = std::multiplies<void>{};
        
        template <class Ops, class IterType>
        static constexpr auto accumulate(const Ops& ops, IterType first, IterType last) -> decltype(ops(*first, *first)){ //REVIEW: assumption (*, constexprable std::accumulate)

            return std::accumulate(std::next(first), last, *first, ops);
        }

        template <class Ops, class IterType, class StopCond>
        static constexpr auto accumulate_until(const Ops& ops, IterType first, IterType last, 
                                               const StopCond& stop_cond) -> std::pair<decltype(accumulate(ops, first, last)), IterType>{
            
            auto llast   = std::next(first);

            while (llast != last && !stop_cond(*llast, *std::prev(llast))){
                ++llast;
            } 
            
            return {accumulate(ops, first, llast), llast};
        }
    };

    struct IntegralUtility{

        //BEGIN <= key < END
        template <size_t BEGIN, size_t END, class CallBack, class Transform, class KeyType>
        static void lb_templatize(const CallBack& cb_lambda, const Transform& transform_lambda, const KeyType& key){

            if constexpr(BEGIN + 1 == END){
                
                cb_lambda(std::integral_constant<size_t, BEGIN>{});

            } else{

                constexpr size_t MID    = (BEGIN + END) >> 1;
                constexpr auto MID_VAL  = transform_lambda(std::integral_constant<size_t, MID>{});  

                return (key < MID_VAL) ? lb_templatize<BEGIN, MID>(cb_lambda, transform_lambda, key)
                                       : lb_templatize<MID, END>(cb_lambda, transform_lambda, key);   
            }
        }
        
        template <size_t BEGIN, size_t END, class CallBack>
        static void templatize(const CallBack& cb_lambda, size_t key){

            lb_templatize<BEGIN, END>(cb_lambda, []<size_t Arg>(const std::integral_constant<size_t, Arg>&){return Arg;}, key);
        }

        template <size_t RHS, class T, std::enable_if_t<std::numeric_limits<T>::is_integer, bool> = true>
        static constexpr T add(T lhs){

            if constexpr(RHS == 0){
                return lhs;
            } else{
                return lhs + RHS;
            }
        }  
    };
    
    struct HeapEssential{

        static constexpr auto parent(size_t idx) -> size_t{

            return (idx - 1) >> 1;
        }

        static constexpr auto left(size_t idx) -> size_t{

            return idx * 2 + 1;
        }
        
        static constexpr auto right(size_t idx) -> size_t{

            return idx * 2 + 2;
        }

        static constexpr auto base_length(size_t arg_height) -> size_t{

            return size_t{1} << (arg_height - 1);
        }

        static constexpr auto node_count(size_t arg_height) -> size_t{

            return (size_t{1} << arg_height) - 1;
        }

        static constexpr auto idx_to_height(size_t idx) -> size_t{

            if (idx == 0){
                return 1;
            }

            return idx_to_height(parent(idx)) + 1;
        }

        template <size_t CUR_HEIGHT>
        static constexpr auto next_height(const std::integral_constant<size_t, CUR_HEIGHT>&){ //
        
            return std::integral_constant<size_t, CUR_HEIGHT + 1>(); 
        }

        template <size_t CUR_HEIGHT>
        static constexpr auto prev_height(const std::integral_constant<size_t, CUR_HEIGHT>&){

            return std::integral_constant<size_t, CUR_HEIGHT - 1>();
        }
    };

    template <size_t HEIGHT>
    struct HeapUtility: HeapEssential{

        static constexpr auto get_non_base_height() -> size_t{
            
            return HEIGHT - 2; 
        }   

        static constexpr auto get_next_base_height() -> size_t{

            return HEIGHT - 1;
        } 

        static constexpr auto get_height() -> size_t{

            return HEIGHT;
        }

        static constexpr auto is_base(size_t arg_height) -> bool{

            return arg_height == get_height();
        }

        static constexpr auto is_next_base(size_t arg_height) -> bool{
            
            return arg_height == get_next_base_height();
        }

        static constexpr auto is_not_base(size_t arg_height) -> bool{

            return arg_height <= get_non_base_height();
        }

        static constexpr auto base_length(size_t arg_height = HEIGHT) -> size_t{

            return HeapEssential::base_length(arg_height);
        }

        static constexpr auto node_count(size_t arg_height = HEIGHT) -> size_t{

            return HeapEssential::node_count(arg_height);
        }

        static constexpr auto height_is_in_range(size_t arg_height) -> bool{

            return arg_height <= get_height();
        }

        static constexpr auto idx_to_offset(size_t idx) -> size_t{

            return idx - node_count(get_next_base_height());
        }
    };

    struct IntervalEssential{

        using store_type        = types::store_type;
        using interval_type     = types::interval_type;
        using op_interval_type  = std::optional<interval_type>;
        
        using _TypeUtility      = TypeUtility;

        static_assert(_TypeUtility::is_integer_pair<interval_type>());
        static_assert(std::numeric_limits<store_type>::is_integer);

        static constexpr auto make(store_type first, store_type last) -> interval_type{
            
            return {first, last};
        }

        static constexpr auto max_interval() -> interval_type{

            return make(0, std::numeric_limits<store_type>::max()); //REVIEW: overflow span size
        }

        static constexpr auto span_size(const interval_type& interval) -> store_type{

            return interval.second - interval.first + 1;
        }

        static constexpr auto get_interval_beg(const interval_type& data) -> store_type{
            
            return data.first;
        } 

        static constexpr auto get_interval_end(const interval_type& data) -> store_type{

            return data.second;
        }

        static constexpr auto get_interval_excl_end(const interval_type& data) -> store_type{

            return get_interval_end(data) + 1;
        }

        static constexpr auto is_valid_interval(const interval_type& interval) -> bool{

            return interval.second >= interval.first;
        }

        static constexpr auto is_consecutive(const interval_type& lhs, const interval_type& rhs) -> bool{

            return (get_interval_excl_end(lhs) == get_interval_beg(rhs)) || (get_interval_excl_end(rhs) == get_interval_beg(lhs));  
        }

        static constexpr auto intersect(const interval_type& lhs, const interval_type& rhs) -> interval_type{

            return make(std::max(get_interval_beg(lhs), get_interval_beg(rhs)), 
                        std::min(get_interval_end(lhs), get_interval_end(rhs)));
        }

        static constexpr auto uunion(const interval_type& lhs, const interval_type& rhs) -> interval_type{

            return make(std::min(get_interval_beg(lhs), get_interval_beg(rhs)),
                        std::max(get_interval_end(lhs), get_interval_end(rhs)));
        }

        static constexpr auto interval_to_relative(const interval_type& interval) -> interval_type{

            return make(interval.first, interval.second - interval.first);
        }

        static constexpr auto relative_to_interval(const interval_type& rel) -> interval_type{

            return make(rel.first, rel.first + rel.second);
        }

        static constexpr auto interval_to_excl_relative(const interval_type& interval) -> interval_type{

            return make(interval.first, span_size(interval));
        }

        static constexpr auto excl_relative_to_interval(const interval_type& rel) -> interval_type{

            return make(rel.first, rel.first + rel.second - 1); 
        }

        static constexpr auto max_val_before_plus_overflow(store_type value) -> store_type{

            return std::numeric_limits<store_type>::max() - value;
        } 
        
        static constexpr auto guarded_intersect(const interval_type& lhs, const interval_type& rhs) -> std::optional<interval_type>{

            if (auto rs = intersect(lhs, rhs); is_valid_interval(rs)){
                return rs;
            }

            return std::nullopt;
        }
        
        static constexpr auto guarded_overlap_union(const interval_type& lhs, const interval_type& rhs) -> std::optional<interval_type>{

            return bool{guarded_intersect(lhs, rhs)} ? op_interval_type{uunion(lhs, rhs)} 
                                                     : op_interval_type{std::nullopt};
        }
 
        static constexpr auto guarded_consecutive_union(const interval_type& lhs, const interval_type& rhs) -> std::optional<interval_type>{

            return is_consecutive(lhs, rhs) ? op_interval_type(uunion(lhs, rhs))
                                            : op_interval_type{std::nullopt};
        }   

        static constexpr auto guarded_union(const interval_type& lhs, const interval_type& rhs) -> std::optional<interval_type>{

            if (auto rs = guarded_consecutive_union(lhs, rhs); rs){
                return rs;
            }

            return guarded_overlap_union(lhs, rhs);
        }

        static constexpr auto guarded_relative_to_interval(const interval_type& rel) -> std::optional<interval_type>{

            return (rel.first > max_val_before_plus_overflow(rel.second)) ? op_interval_type{std::nullopt}
                                                                          : op_interval_type{relative_to_interval(rel)};
        }


        static constexpr auto guarded_excl_relative_to_interval(const interval_type& rel) -> std::optional<interval_type>{

            return (rel.second == 0) ? op_interval_type{std::nullopt} 
                                     : guarded_relative_to_interval(make(rel.first, rel.second - 1)); 
        }

        static constexpr auto midpoint(const interval_type& val) -> store_type{

            return val.first + ((val.second - val.first) >> 1);
        } 

        static constexpr auto is_left_bound(const interval_type& interval, store_type _midpoint) -> bool{

            return interval.second <= _midpoint;
        }

        static constexpr auto is_right_bound(const interval_type& interval, store_type _midpoint) -> bool{

            return interval.first > _midpoint;
        }

        static constexpr auto left_shrink(const interval_type& interval, store_type _midpoint) -> interval_type{

            return make(interval.first, _midpoint);
        }

        static constexpr auto right_shrink(const interval_type& interval, store_type _midpoint) -> interval_type{
            
            return make(_midpoint + 1, interval.second); //REVIEW: consider excl interval [) to avoid + 1
        }

        static constexpr auto incl_right_shrink(const interval_type& interval, store_type _midpoint) -> interval_type{

            return make(_midpoint, interval.second);
        }

        static constexpr auto left_interval(const interval_type& val) -> interval_type{
            
            return left_shrink(val, midpoint(val));
        }

        static constexpr auto right_interval(const interval_type& val) -> interval_type{

            return right_shrink(val, midpoint(val));
        }
    };

    //refactoring
    struct IntervalEssentialLambdanizer: private IntervalEssential{

        using _Base     = IntervalEssential;
        
        static constexpr auto uunion         = []<class ...Args>(Args&& ...args){return _Base::uunion(std::forward<Args>(args)...);};
        static constexpr auto is_consecutive = []<class ...Args>(Args&& ...args){return _Base::is_consecutive(std::forward<Args>(args)...);};
    };

    struct MemoryUtility{

        template <uintptr_t ALIGNMENT>
        static inline auto align(char * buf) noexcept -> char *{
            
            constexpr bool is_pow2      = (ALIGNMENT != 0) && ((ALIGNMENT & (ALIGNMENT - 1)) == 0);
            static_assert(is_pow2);

            constexpr uintptr_t MASK    = (ALIGNMENT - 1);
            constexpr uintptr_t NEG     = ~MASK;
            char * rs                   = reinterpret_cast<char *>((reinterpret_cast<uintptr_t>(buf) + MASK) & NEG);

            return rs;
        }

        static inline auto forward_shift(char * buf, size_t sz) noexcept -> char *{
 
            return buf + sz;
        }

        static inline auto forward_shift(const char * buf, size_t sz) noexcept -> const void *{ 

            return buf + sz;
        }
        
        static inline auto get_distance_vector(const void * _from, const void * _to) noexcept -> intptr_t{

            return reinterpret_cast<intptr_t>(_to) - reinterpret_cast<intptr_t>(_from); 
        } 
    };

    template <size_t HEIGHT>
    struct IntervalUtility: IntervalEssential{

        using store_type        = IntervalEssential::store_type;
        using interval_type     = IntervalEssential::interval_type;
        using _HeapUtility      = HeapUtility<HEIGHT>; 

        static constexpr auto span_size_from_height(size_t arg_height) -> store_type{

            return _HeapUtility::base_length() / _HeapUtility::base_length(arg_height);
        }

        static constexpr auto idx_to_interval(size_t idx) -> interval_type{

            if (idx == 0){
                return excl_relative_to_interval(make(0, _HeapUtility::base_length()));
            }

            return (idx == _HeapUtility::left(_HeapUtility::parent(idx))) ? left_interval(idx_to_interval(_HeapUtility::parent(idx))) 
                                                                          : right_interval(idx_to_interval(_HeapUtility::parent(idx)));
        }

        template <size_t ARG_HEIGHT>
        static constexpr auto idx_to_interval(size_t idx) -> interval_type{

            constexpr auto popcount     = _HeapUtility::node_count(ARG_HEIGHT - 1);
            constexpr auto span         = span_size_from_height(ARG_HEIGHT);
            auto offs                   = static_cast<store_type>((idx - popcount) * span); 

            return excl_relative_to_interval(make(offs, span));
        }
    };

    struct LambdaUtility{
        
        template <class T>
        static constexpr auto boolify(T org_lambda){

            auto lambda = [=]<class ...Args>(Args&& ...args){
                return bool{org_lambda(std::forward<Args>(args)...)}; //REVIEW: narrow conversion
            };

            return lambda;
        }

        template <class T>
        static constexpr auto negate(T org_lambda){

            auto lambda     = [=]<class ...Args>(Args&& ...args){
                return !org_lambda(std::forward<Args>(args)...);
            };

            return lambda;
        } 

        static constexpr auto get_null_lambda(){

            return []<class ...Args>(Args&&...){};
        }

        template <class ...Ts>
        static constexpr auto void_aggregate(Ts ...lambdas){

            auto dispatcher = []<size_t ...IDX, class Tup, class ...Args>(const std::index_sequence<IDX...>&, Tup&& lambda, Args&& ...args){
                (std::get<IDX>(lambda)(std::forward<Args>(args)...), ...);
            };

            auto exec_lambda = [=]<class ...Args>(Args&& ...args){
                dispatcher(std::make_index_sequence<sizeof...(Ts)>(), std::make_tuple(lambdas...), std::forward<Args>(args)...);
            };

            return exec_lambda;
        }

        template <class T, class T1>
        static constexpr auto bind_filter_n_deflate(T lambda, T1 filter){

            auto rs = [=]<class ...Args>(Args&& ...args){

                decltype(auto) filtered_rs  = filter(std::forward<Args>(args)...);

                if constexpr(types_space::is_void_lambda_v<T, decltype(filtered_rs)>){
                    essentials::piecewise_void_invoke(lambda, filtered_rs);
                } else{
                    
                    decltype(auto) ret  = essentials::piecewise_invoke(lambda, filtered_rs);
                    using ret_type      = decltype(ret);
                    static_assert(std::is_same_v<ret_type, typename types_space::base_type<ret_type>>); //lambda limitation (or mine) as there's not an intuitive way to return decltype(auto)

                    return ret;
                }
            };

            return rs;
        }

        template <class T, class T1>
        static constexpr auto bind_void_layer(T lambda, T1 layer){

            auto rs = [=]<class ...Args>(Args&& ...args){

                auto fwd_tup    = std::forward_as_tuple(std::forward<Args>(args)...);                
                essentials::piecewise_void_invoke(layer, fwd_tup);

                if constexpr(types_space::is_void_lambda_v<T, decltype(fwd_tup)>){
                    essentials::piecewise_void_invoke(lambda, fwd_tup);
                } else{

                    decltype(auto) ret  = essentials::piecewise_invoke(lambda, fwd_tup);
                    using ret_type      = decltype(ret);
                    static_assert(std::is_same_v<ret_type, typename types_space::base_type<ret_type>>); //lambda limitation (or mine) as there's not an intuitive way to return decltype(auto)

                    return ret;
                }
            };

            return rs;
        }
    };

    struct IteratorUtility{

        template <class Iterator>
        static constexpr auto is_equal(Iterator lhs, Iterator rhs) -> bool{

            if constexpr(types_space::has_equal_operator_v<Iterator>){
                return lhs == rhs;
            } else if constexpr(types_space::has_not_equal_operator_v<Iterator>){
                return !(lhs != rhs);
            } else{
                return std::distance(lhs, rhs) == 0;
            }
        }

        template <class Iterator>
        static constexpr auto prev_last_helper(Iterator first, Iterator last) -> Iterator{

            size_t dist = static_cast<size_t>(std::distance(first, last)) - 1;
            std::advance(first, dist);

            return first;
        }

        template <class Iterator>
        static constexpr auto prev_last(Iterator first, Iterator last) -> Iterator{
  
            if constexpr(types_space::is_bidirectional_iter_v<Iterator>){
                return std::prev(last);
            } else{
                return prev_last_helper(first, last);
            }
        }

        template <class Iterator>
        static constexpr auto mid(Iterator first, Iterator last) -> Iterator{

            size_t dist = static_cast<size_t>(std::distance(first, last)) >> 1; //consider intmax_t 
            std::advance(first, dist);

            return first;
        }

        template <class Iterator>
        static constexpr auto post_inc(Iterator& iter) -> Iterator{

            Iterator rs{iter};
            iter = std::next(iter);
            return rs;
        }

        template <class Iterator>
        static constexpr auto deref(Iterator it) -> decltype(auto){

            return *it;
        }

        template <class Iterator>
        static constexpr auto meat(Iterator it) -> decltype(auto){

            return deref(it);
        }

    };

    struct AlgorithmUtility{
        
        using _IteratorUlt  = IteratorUtility;

        template <class Val, class Validator>
        static constexpr auto sanitize(Val data, Val _false_ret, const Validator& test_lambda) -> Val{

            return test_lambda(data) ? data : _false_ret; 
        } 

        //first <= key < last (return first) cmp_ops = (less for asc | greater for desc)
        template <class Iterator, class Val, class Ops>
        static constexpr auto rec_uupper_bound(Iterator first, Iterator last, const Val& key, const Ops& cmp_ops) -> Iterator{
            
            auto mid             = _IteratorUlt::mid(first, last);
            auto terminated_cond = _IteratorUlt::is_equal(std::next(first), last);

            return terminated_cond ? first : cmp_ops(key, _IteratorUlt::meat(mid)) ? rec_uupper_bound(first, mid, key, cmp_ops)
                                                                                   : rec_uupper_bound(mid, last, key, cmp_ops);
        }

        //begin <= key < end (return begin) cmp_ops = (less for asc | greater for desc)
        //consider iterative version 
        template <class Iterator, class Val, class Ops>
        static constexpr auto uupper_bound(Iterator first, Iterator last, const Val& key, const Ops& cmp_ops) -> Iterator{

            if (_IteratorUlt::is_equal(first, last)){
                return last;
            }

            auto is_valid = [&](Iterator cand){return !cmp_ops(key, _IteratorUlt::meat(cand));};
            auto rs       = sanitize(rec_uupper_bound(first, last, key, cmp_ops), last, is_valid);  

            return rs;
        }

        //first < key <= last (return last)
        template <class Iterator, class Val, class Ops>
        static constexpr auto rec_lower_bound(Iterator first, Iterator last, const Val& key, const Ops& cmp_ops) -> Iterator{
            
            auto mid             = _IteratorUlt::mid(first, last);
            auto terminated_cond = _IteratorUlt::is_equal(std::next(first), last);

            return terminated_cond ? last : cmp_ops(_IteratorUlt::meat(mid), key) ? rec_lower_bound(mid, last, key, cmp_ops)
                                                                                  : rec_lower_bound(first, mid, key, cmp_ops);
        } 

        template <class Iterator, class Val, class Ops>
        static constexpr auto lower_bound(Iterator first, Iterator last, const Val& key, const Ops& cmp_ops) -> Iterator{

            if (_IteratorUlt::is_equal(first, last)){
                return last;
            }

            return cmp_ops(_IteratorUlt::meat(first), key) ? rec_lower_bound(first, last, key, cmp_ops) : first;
        }
    };

    struct IntervalEssential_P: IntervalEssential{
        
        using store_type    = IntervalEssential::store_type;
        using interval_type = IntervalEssential::interval_type;
        using _IteratorUlt  = IteratorUtility;
        using _AlgoUlt      = AlgorithmUtility;

        template <class Iterator>
        static constexpr auto seek_incl_left(Iterator first, Iterator last, store_type _midpoint) -> Iterator{

            auto key    = make(_midpoint, _midpoint);
            auto cmp    = [](const interval_type& lhs, const interval_type& rhs){return get_interval_beg(lhs) < get_interval_beg(rhs);};
            
            return _AlgoUlt::uupper_bound(first, last, key, cmp);                
        } 

        template <class Iterator>
        static constexpr auto seek_incl_right(Iterator tentative_mid, store_type _midpoint) -> Iterator{

            auto r      = right_shrink(max_interval(), _midpoint);
            auto req    = bool{guarded_intersect(_IteratorUlt::meat(tentative_mid), r)};

            return req ? tentative_mid : std::next(tentative_mid);
        } 

        template <class Iterator>
        static constexpr auto seek_incl_right(Iterator first, Iterator last, store_type _midpoint) -> Iterator{

            return seek_incl_right(seek_incl_left(first, last, _midpoint), _midpoint);
        }

        template <class Iterator>
        static constexpr auto pair_shrink(Iterator first, Iterator last, store_type _midpoint) -> std::pair<Iterator, Iterator>{
            
            auto l = seek_incl_left(first, last, _midpoint);
            auto r = seek_incl_right(l, _midpoint);

            return {std::next(l), r};
        }
    };

    struct NodeUtility{
        
        using Node          = types::Node;
        using store_type    = types::store_type; 

        static_assert(std::is_trivial_v<Node>);

        static constexpr auto make(store_type l, store_type r, store_type c, store_type o) -> Node{

            return Node{l, r, c, o};
        }
        
        static constexpr auto to_tuple(const Node& data){

            return std::make_tuple(data.l, data.r, data.c, data.o);
        } 
        
        template <class Tup>
        static constexpr auto make_from_tuple(const Tup& tup){

            const auto idx_seq  = std::make_index_sequence<std::tuple_size_v<Tup>>();

            auto maker          = [&]<size_t ...IDX>(const std::index_sequence<IDX...>&){
                return make(std::get<IDX>(tup)...);
            };

            return maker(idx_seq);
        }

        static constexpr auto equal(const Node& lhs, const Node& rhs) -> bool{

            if constexpr(types_space::has_no_padding_v<Node>){
                return std::memcmp(&lhs, &rhs, sizeof(Node)) == 0;
            } else{
                return to_tuple(lhs) == to_tuple(rhs);
            }
        }
        
        static inline void assign(Node& lhs, const Node& rhs){
            
            std::memcpy(&lhs, &rhs, sizeof(Node)); //WARNING: maybe ub (if lhs was not correctly initiated) - as specified by standard
        } 
    };

    struct ValConstUtility{

        using Node              = types::Node;
        using interval_type     = types::interval_type;
        using store_type        = types::store_type;
        using cache_type        = types::cache_type;

        using _NodeUtility      = NodeUtility;
        using _IntervalUtility  = IntervalEssential;

        static_assert(std::is_unsigned_v<store_type>);

        static inline const store_type ZERO_STORE = 0u;
        static inline const store_type LEAF_STORE = 1u;
        static inline const store_type NULL_STORE = ~ZERO_STORE;

        template <class T, std::enable_if_t<std::is_same<T, store_type>::value, bool> = true>
        static inline _DG_CONSTEVAL auto empty() -> store_type{

            return ZERO_STORE;
        }
   
        template <class T, std::enable_if_t<std::is_same<T, Node>::value, bool> = true>
        static inline _DG_CONSTEVAL auto empty() -> Node{

            return _NodeUtility::make(ZERO_STORE, ZERO_STORE, ZERO_STORE, ZERO_STORE);
        }

        template <class T, std::enable_if_t<std::is_same<T, store_type>::value, bool> = true>
        static inline _DG_CONSTEVAL auto null() -> store_type{
        
            return NULL_STORE;
        }

        template <class T, std::enable_if_t<std::is_same<T, Node>::value, bool> = true>
        static inline _DG_CONSTEVAL auto null() -> Node{

            return _NodeUtility::make(NULL_STORE, NULL_STORE, NULL_STORE, NULL_STORE); 
        }

        template <class T, std::enable_if_t<std::is_same<T, store_type>::value, bool> = true>
        static inline _DG_CONSTEVAL auto leaf() -> store_type{
            
            return LEAF_STORE;
        }
        
        template <class T, std::enable_if_t<std::is_same<T, Node>::value, bool> = true>
        static constexpr auto deflt(const interval_type& interval) -> Node{

            store_type span = _IntervalUtility::span_size(interval);
            store_type offs = _IntervalUtility::get_interval_beg(interval);

            return _NodeUtility::make(span, span, span, offs);
        }

    };

    template <size_t HEIGHT>
    struct OffsetConverter{

        using traceback_type    = traceback_policy::traceback_type;
        using _HeapUtility      = HeapUtility<HEIGHT>;
        using _IntegralUtility  = IntegralUtility;
        using _NumericUtility   = NumericUtility;

        static constexpr size_t get_non_base_bit_offset(size_t idx){

            return idx * traceback_policy::STD_BUCKET_LENGTH; 
        }

        static constexpr size_t get_non_base_left_bit_offset(size_t idx){

            return _IntegralUtility::add<traceback_policy::L_OFFSET>(get_non_base_bit_offset(idx));
        }

        static constexpr size_t get_non_base_right_bit_offset(size_t idx){

            return _IntegralUtility::add<traceback_policy::R_OFFSET>(get_non_base_bit_offset(idx));
        }

        static constexpr size_t get_non_base_center_bit_offset(size_t idx){

            return _IntegralUtility::add<traceback_policy::C_OFFSET>(get_non_base_bit_offset(idx));
        }

        static constexpr size_t get_next_base_adjusted_val(){

            size_t popcount   = _HeapUtility::node_count(_HeapUtility::get_non_base_height());
            size_t offset     = get_non_base_bit_offset(popcount);
            size_t adjusted   = offset - popcount * traceback_policy::NEXT_BASE_BUCKET_LENGTH;

            return adjusted;
        }

        static constexpr size_t get_next_base_bit_offset(size_t idx){
            
            return idx * traceback_policy::NEXT_BASE_BUCKET_LENGTH + get_next_base_adjusted_val();
        } 

        static constexpr size_t get_next_base_left_bit_offset(size_t idx){

            return idx * traceback_policy::NEXT_BASE_BUCKET_LENGTH + (get_next_base_adjusted_val() + traceback_policy::L_OFFSET); //parentheses allow const propagation
        }

        static constexpr size_t get_next_base_right_bit_offset(size_t idx){
            
            return idx * traceback_policy::NEXT_BASE_BUCKET_LENGTH + (get_next_base_adjusted_val() + traceback_policy::R_OFFSET);
        }

        static constexpr size_t get_next_base_blocked_bit_offset(size_t idx){

            return idx * traceback_policy::NEXT_BASE_BUCKET_LENGTH + (get_next_base_adjusted_val() + traceback_policy::NB_BL_OFFSET);
        }

        static constexpr size_t get_base_adjusted_val(){

            size_t popcount   = _HeapUtility::node_count(_HeapUtility::get_next_base_height());
            size_t offset     = get_next_base_bit_offset(popcount);
            size_t adjusted   = offset - popcount * traceback_policy::BASE_BUCKET_LENGTH;

            return adjusted;
        }

        static constexpr size_t get_base_bit_offset(size_t idx){

            return idx * traceback_policy::BASE_BUCKET_LENGTH + get_base_adjusted_val();
        }

        static constexpr size_t get_base_blocked_bit_offset(size_t idx){

            return idx * traceback_policy::BASE_BUCKET_LENGTH + (get_base_adjusted_val() + traceback_policy::BB_BL_OFFSET);
        }   

        template <size_t ARG_HEIGHT>
        static constexpr size_t get_left_offset(size_t idx){

            if constexpr(_HeapUtility::is_not_base(ARG_HEIGHT)){
                return get_non_base_left_bit_offset(idx);
            } else if constexpr(_HeapUtility::is_next_base(ARG_HEIGHT)){
                return get_next_base_left_bit_offset(idx);
            } else{
                static_assert(FALSE_VAL<>, "unreachable");
                return {};
            }
        }

        template <size_t ARG_HEIGHT>
        static constexpr size_t get_right_offset(size_t idx){

            if constexpr(_HeapUtility::is_not_base(ARG_HEIGHT)){
                return get_non_base_right_bit_offset(idx);
            } else if constexpr(_HeapUtility::is_next_base(ARG_HEIGHT)){
                return get_next_base_right_bit_offset(idx);
            } else{
                static_assert(FALSE_VAL<>, "unreachable");
                return {};
            }
        }

        template <size_t ARG_HEIGHT>
        static constexpr size_t get_center_offset(size_t idx){
            
            if constexpr(_HeapUtility::is_not_base(ARG_HEIGHT)){
                return get_non_base_center_bit_offset(idx);
            } else{
                static_assert(FALSE_VAL<>, "unreachable");
                return {};
            }
        }

    };

    template <class _Executable>
    struct BackoutExecutor{

        using Self = BackoutExecutor;

        _Executable executor; 
        bool flag; 

        BackoutExecutor(_Executable executor): executor(executor), flag(true){}

        BackoutExecutor(const Self&) = delete; 
        BackoutExecutor& operator =(const Self&) = delete;

        ~BackoutExecutor() noexcept(noexcept(std::declval<_Executable>()())){

            if (this->flag){
                this->executor();
            }
        }

        void release() noexcept{
            
            this->flag = false;
        }
    };

    template <class _Ty>
    struct ReservedVectorInitializer: std::vector<_Ty>{

        using _Base = std::vector<_Ty>;

        ReservedVectorInitializer(size_t sz): _Base(){
            _Base::reserve(sz);
        }
    };

    template <class Executable>
    static auto get_backout_executor(Executable executor){
        
        static_assert(noexcept(executor()));
        
        static auto guard       = int{0u};  
        auto backout_lambda     = [=](int *) noexcept{executor();};

        return std::unique_ptr<int *, decltype(backout_lambda)>(&guard, backout_lambda);
    } 
}

namespace dg::heap::memory{

    class BumpAllocator: public Allocatable{ //REVIEW: very bad allocator (as placeholder for future reservoir implementation) 

        private:

            std::unique_ptr<char[]> buf;
            std::vector<bool> bc;
            size_t sz;
        
        public:
            
            BumpAllocator(std::unique_ptr<char[]> buf, std::vector<bool> bc, size_t sz): buf(std::move(buf)), bc(std::move(bc)), sz(sz){}

            char * malloc(size_t block_sz) noexcept{

                char * rs = this->seek(block_sz);

                if (!rs){
                    return rs;
                } 

                uintptr_t offs = reinterpret_cast<uintptr_t>(rs) - reinterpret_cast<uintptr_t>(this->buf.get());
                this->block(static_cast<size_t>(offs), block_sz);

                return rs;
            }

            void free(void * bbuf, size_t sz) noexcept{

                uintptr_t offs = reinterpret_cast<uintptr_t>(bbuf) - reinterpret_cast<uintptr_t>(this->buf.get());
                this->unblock(static_cast<size_t>(offs), sz);
            }

        private:

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

            char * seek(size_t block_sz){

                size_t offs = 0;

                while (true){

                    auto nnext  = seek_next_from(offs);

                    if (!nnext){
                        return nullptr;
                    }

                    if (nnext.value().second >= block_sz){
                        return &buf[nnext.value().first];
                    } 

                    offs = nnext.value().first + nnext.value().second;
                }
            }

            void block(size_t offs, size_t block_sz){

                std::fill(bc.begin() + offs, bc.begin() + offs + block_sz, false);
            }

            void unblock(size_t offs, size_t block_sz){

                std::fill(bc.begin() + offs, bc.begin() + offs + block_sz, true);
            }
    };

    class MutexControlledAllocator: public Allocatable{

        private:

            std::unique_ptr<Allocatable> allocator;
            std::mutex mtx;
        
        public:

            MutexControlledAllocator(std::unique_ptr<Allocatable> allocator): allocator(std::move(allocator)), mtx(){}

            char * malloc(size_t block_sz) noexcept{

                std::lock_guard<std::mutex> _guard(this->mtx);
                return this->allocator->malloc(block_sz);
            }

            void free(void * buf, size_t block_sz) noexcept{
                
                std::lock_guard<std::mutex> _guard(this->mtx);
                this->allocator->free(buf, block_sz);
            }
    };

    struct AllocatorInitializer{

        static auto get_bump_allocator(std::unique_ptr<char[]> buf, size_t sz) -> std::unique_ptr<Allocatable>{

            std::vector<bool> bc{};
            bc.resize(sz, true);

            return std::make_unique<BumpAllocator>(std::move(buf), std::move(bc), sz);
        }

        static auto get_mtx_controlled_allocator(std::unique_ptr<Allocatable> allocator) -> std::unique_ptr<Allocatable>{

            return std::make_unique<MutexControlledAllocator>(std::move(allocator));
        }
        
    };

    struct MemoryAcquirer{

        static inline auto acquire(std::shared_ptr<Allocatable> allocator, size_t sz) noexcept -> std::shared_ptr<char[]>{ //REVIEW: not noexcept qualified due to shared_ptr
            
            if (!allocator){
                return {};
            }

            char * buf  = allocator->malloc(sz);

            if (!buf){
                return {};
            }

            auto destructor     = [=](char * bbuf){allocator->free(bbuf, sz);};
            using rs_type       = std::unique_ptr<char[], decltype(destructor)>;

            return rs_type{buf, destructor};
        }
    };

    struct MemoryMarket{
        
        using _Acquirer = MemoryAcquirer;
        
        static inline std::shared_ptr<Allocatable> reservoir{}; //REVIEW: should be VM? 

        static inline auto buy(size_t block_sz) -> std::unique_ptr<char[]>{

            return std::unique_ptr<char[]>{new char[block_sz]};
        }
        
        static inline auto buy_no_except(size_t block_sz) noexcept -> std::shared_ptr<char[]>{

            try {
            
                return buy(block_sz);
            
            } catch (std::exception&){
                
                auto rs = _Acquirer::acquire(reservoir, block_sz);

                if (rs){
                    return rs;
                }
            }   

            std::abort();
            return {};
        }
    };

    struct MemoryService{

        using _Market       = MemoryMarket; 
        using _MemoryUlt    = utility::MemoryUtility; 

        template <class _Ty, std::enable_if_t<std::is_trivial_v<_Ty>, bool> = true>
        static inline auto launder_arr(void * data, size_t sz) noexcept -> _Ty *{

            auto bsz            = sz * sizeof(_Ty);
            auto cp_buf         = _Market::buy_no_except(bsz);
            
            std::memcpy(cp_buf.get(), data, bsz);

            auto rs             = static_cast<_Ty *>(new (data) _Ty[sz]);
            auto offs           = [=](_Ty * e){return static_cast<size_t>(_MemoryUlt::get_distance_vector(rs, e));};
            auto fetch          = [=, &cp_buf](_Ty& e){std::memcpy(&e, &cp_buf[offs(&e)], sizeof(_Ty));};
            
            std::for_each(rs, rs + sz, fetch);

            return rs;
        }

        static inline auto get_buffer_reverter(void * buf, size_t buf_sz){

            auto backup     = _Market::buy(buf_sz);
            std::memcpy(backup.get(), buf, buf_sz);

            auto reverter   = [=, _backup = std::move(backup)]{
                std::memcpy(buf, _backup.get(), buf_sz);
            };

            return reverter;
        }
    };

    struct LifeTimeTracker{

        private:

            static inline std::unordered_map<uintptr_t, void *> objects{};
            static inline std::mutex mtx{};

        public:

            static inline void reserve(size_t sz){

                std::lock_guard<std::mutex> guard(mtx);
                objects.reserve(sz);
            }
            
            template <class T>
            static inline void start_lifetime(T * obj) noexcept{

                std::lock_guard<std::mutex> guard(mtx);
                objects[reinterpret_cast<uintptr_t>(obj)] = static_cast<void *>(obj);
            } 

            template <class T>
            static inline auto retrieve(void * addr) noexcept -> T *{

                std::lock_guard<std::mutex> guard(mtx);
                auto bucket = objects.find(reinterpret_cast<uintptr_t>(addr));

                if (bucket == objects.end()){
                    std::abort();
                }

                return static_cast<T *>(bucket->second); 
            }

            static inline void end_lifetime(void * addr) noexcept{
                
                std::lock_guard<std::mutex> guard(mtx);
                objects.erase(reinterpret_cast<uintptr_t>(addr));
            }
    };
}

namespace dg::heap::seeker{

    struct GreedyBatchSeeker{

        private:

            using index_type                    = size_t;
            using store_type                    = types::store_type;
            using interval_type                 = types::interval_type;
            using bucket_type                   = std::pair<interval_type, index_type>;
            using op_bucket_type                = std::optional<bucket_type>;

            using _HeapUlt                      = utility::HeapEssential;
            using _IntvUlt                      = utility::IntervalEssential;

            static inline const auto cmp_less   = [](const bucket_type& lhs, const bucket_type& rhs){return _IntvUlt::span_size(lhs.first) < _IntvUlt::span_size(rhs.first);};
            using priority_queue_type           = std::priority_queue<bucket_type, std::vector<bucket_type>, decltype(cmp_less)>; //desc pq

            template <class T, std::enable_if_t<std::is_same_v<typename types_space::base_type<T>, bucket_type>, bool> = true>
            static constexpr auto get_interval(T&& bucket) -> decltype(auto){

                return std::get<0>(bucket);

            }

            template <class T, std::enable_if_t<std::is_same_v<typename types_space::base_type<T>, bucket_type>, bool> = true>
            static constexpr auto get_idx(T&& bucket) -> decltype(auto){

                return std::get<1>(bucket);

            }

            static constexpr auto make_bucket(interval_type intv, index_type idx) -> bucket_type{

                return {intv, idx};

            }

            static constexpr auto make_op_bucket(std::optional<interval_type> intv, index_type idx) -> op_bucket_type{
                
                return bool{intv} ? op_bucket_type{make_bucket(intv.value(), idx)} 
                                  : op_bucket_type{std::nullopt};

            }

            static constexpr auto is_sum_two_child_greater(const op_bucket_type& l_child, const op_bucket_type& r_child, 
                                                           const bucket_type& cur, store_type unit_interval) -> bool{
                
                using sum_type = size_t; // overflow and fuzzy logic issue 

                auto cur_total      = sum_type{}; 
                auto child_total    = sum_type{}; 
                auto converted_unit = static_cast<sum_type>(unit_interval); 

                cur_total           = static_cast<sum_type>(_IntvUlt::span_size(get_interval(cur)))             / converted_unit;
                
                if (l_child){
                    child_total    += static_cast<sum_type>(_IntvUlt::span_size(get_interval(l_child.value()))) / converted_unit;
                }

                if (r_child){
                    child_total    += static_cast<sum_type>(_IntvUlt::span_size(get_interval(r_child.value()))) / converted_unit;
                }

                return child_total > cur_total;

            }


            static inline auto make_priority_queue() -> priority_queue_type{

                return priority_queue_type(cmp_less);

            }

            template <class T>
            static inline auto get_child_buckets(seeker::Seekable<T>& seeker, size_t idx) -> std::pair<op_bucket_type, op_bucket_type>{

                return {make_op_bucket(seeker.seek(_HeapUlt::left(idx)), _HeapUlt::left(idx)), 
                        make_op_bucket(seeker.seek(_HeapUlt::right(idx)), _HeapUlt::right(idx))};

            }

            template <class ...Args, std::enable_if_t<std::conjunction_v<std::is_same<op_bucket_type, typename types_space::base_type<Args>>...>, bool> = true>
            static inline auto try_insert(priority_queue_type& container, Args&& ...args) -> bool{ //atomic ops

                bool is_valid   = (bool{args} && ...);

                if (!is_valid){
                    return false;
                }

                ((container.push(args.value())), ...);

                return true;
            }

            static inline auto pop(priority_queue_type& container) -> bucket_type{
                
                auto rs = container.top();
                container.pop();

                return rs;
            }

        public:
            
            static constexpr auto DEFAULT_UNIT_INTV  = store_type{1} << 10; //1kb 
            static constexpr auto DEFAULT_EXPANSION  = size_t{1} << 5; 

            template <class T>
            static inline auto get(seeker::Seekable<T>& seeker, 
                                   store_type unit_interval = DEFAULT_UNIT_INTV,
                                   size_t expansion_sz      = DEFAULT_EXPANSION) -> std::vector<interval_type>{
                
                //101 -- need a more robust solution
                //this method works particularly well when the entropy of the heap is high -> center value can be used as availability index
                assert(unit_interval >= 1u);

                auto first_bucket   = make_op_bucket(seeker.seek(0u), 0u); 
                auto q              = make_priority_queue();
                auto rs             = std::vector<interval_type>{};
                auto i              = size_t{0u};

                if (!try_insert(q, first_bucket)){
                    return rs;
                }

                while((i < expansion_sz) && (!q.empty())){

                    auto cur        = pop(q);
                    auto [l, r]     = get_child_buckets(seeker, get_idx(cur));
                    auto success    = bool{is_sum_two_child_greater(l, r, cur, unit_interval) && try_insert(q, l, r)};

                    if (!success){
                        rs.push_back(get_interval(cur));
                    }
                    
                    ++i;

                }

                while (!q.empty()){
                    rs.push_back(get_interval(pop(q)));
                }

                return rs;
            }   

    };

    template <class T>
    class MaxIntervalSeeker: public Seekable<MaxIntervalSeeker<T>>{

        public:

            using store_type        = types::store_type;
            using interval_type     = types::interval_type;
            using op_interval_type  = std::optional<interval_type>;
            using _IntvUlt          = utility::IntervalEssential;  
            using _HeapUlt          = utility::HeapEssential;
            using _IntegralUlt      = utility::IntegralUtility;
            using _Extractor        = data::StorageExtractible<T>; 

            MaxIntervalSeeker(const data::StorageExtractible<T>){}

            op_interval_type seek(size_t idx) noexcept{

                constexpr auto TREE_HEIGHT  = _Extractor::TREE_HEIGHT;
                constexpr auto popcount     = _HeapUlt::node_count(TREE_HEIGHT);

                if (idx >= popcount){
                    return std::nullopt;
                }


                constexpr auto BEG_HEIGHT   = size_t{_HeapUlt::idx_to_height(0u)};
                constexpr auto EXCL_HEIGHT  = size_t{TREE_HEIGHT + 1};
                auto rs                     = op_interval_type{std::nullopt};

                auto cb_lambda          = [=, &rs]<size_t ARG_HEIGHT>(const std::integral_constant<size_t, ARG_HEIGHT>&){
                    auto offs   = _Extractor::template get_offset_at<ARG_HEIGHT>(idx);
                    auto span   = _Extractor::template get_center_at<ARG_HEIGHT>(idx);
                    rs          = _IntvUlt::guarded_excl_relative_to_interval(_IntvUlt::make(offs, span));
                };

                _IntegralUlt::templatize<BEG_HEIGHT, EXCL_HEIGHT>(cb_lambda, _HeapUlt::idx_to_height(idx)); // -- improvement required
                
                return rs;
            }   

            template <size_t IDX>
            op_interval_type seek(const std::integral_constant<size_t, IDX>&) noexcept{

                constexpr auto TREE_HEIGHT  = _Extractor::TREE_HEIGHT;
                constexpr auto ARG_HEIGHT   = _HeapUlt::idx_to_height(IDX); 
                auto rs                     = op_interval_type{std::nullopt};

                if constexpr(ARG_HEIGHT <= TREE_HEIGHT){
                    auto offs           = _Extractor::template get_offset_at<ARG_HEIGHT>(IDX);
                    auto span           = _Extractor::template get_center_at<ARG_HEIGHT>(IDX);
                    rs                  = _IntvUlt::guarded_excl_relative_to_interval(_IntvUlt::make(offs, span));
                } 

                return rs;
            }
    };

    template <class T>
    class RightIntervalSeeker: public Seekable<RightIntervalSeeker<T>>{

        public: 

            using store_type        = types::store_type;
            using interval_type     = types::interval_type;
            using op_interval_type  = std::optional<interval_type>;
            using _HeapUlt          = utility::HeapEssential;
            using _IntegralUlt      = utility::IntegralUtility;
            using _Extractor        = data::StorageExtractible<T>; 

            RightIntervalSeeker(const data::StorageExtractible<T>){}

            op_interval_type seek(size_t idx) noexcept{

                constexpr auto TREE_HEIGHT  = _Extractor::TREE_HEIGHT;
                constexpr auto popcount     = _HeapUlt::node_count(TREE_HEIGHT);
                using _IntvUlt              = utility::IntervalUtility<TREE_HEIGHT>;

                if (idx >= popcount){
                    return std::nullopt;
                }

                constexpr auto BEG_HEIGHT   = size_t{_HeapUlt::idx_to_height(0u)};
                constexpr auto EXCL_HEIGHT  = size_t{TREE_HEIGHT + 1};
                auto rs                     = op_interval_type{std::nullopt};

                auto cb_lambda              = [=, &rs]<size_t ARG_HEIGHT>(const std::integral_constant<size_t, ARG_HEIGHT>&){
                    auto span   = _Extractor::template get_right_at<ARG_HEIGHT>(idx);
                    auto offs   = _IntvUlt::get_interval_excl_end(_IntvUlt::template idx_to_interval<ARG_HEIGHT>(idx)) - span;
                    rs          = _IntvUlt::guarded_excl_relative_to_interval(_IntvUlt::make(offs, span));
                };

                _IntegralUlt::templatize<BEG_HEIGHT, EXCL_HEIGHT>(cb_lambda, _HeapUlt::idx_to_height(idx));

                return rs;
            }

            template <size_t IDX>
            op_interval_type seek(const std::integral_constant<size_t, IDX>&) noexcept{

                constexpr auto TREE_HEIGHT  = _Extractor::TREE_HEIGHT;
                using _IntvUlt              = utility::IntervalUtility<TREE_HEIGHT>;
                constexpr auto ARG_HEIGHT   = _HeapUlt::idx_to_height(IDX); 
                auto rs                     = op_interval_type{std::nullopt};

                if constexpr(ARG_HEIGHT <= TREE_HEIGHT){
                    auto span           = _Extractor::template get_right_at<ARG_HEIGHT>(IDX);
                    auto offs           = _IntvUlt::get_interval_excl_end(_IntvUlt::idx_to_interval(IDX)) - span;
                    rs                  = _IntvUlt::guarded_excl_relative_to_interval(_IntvUlt::make(offs, span));
                } 

                return rs;
            }
    };

    struct SeekerSpawner{
        
        template <class T>
        static auto get_max_interval_seeker(const data::StorageExtractible<T> extractor){
            
            MaxIntervalSeeker ins(extractor);

            using rs_type   = std::shared_ptr<std::remove_pointer_t<decltype(ins.to_seekable())>>;
            using obj_type  = decltype(ins);

            rs_type rs      = std::unique_ptr<obj_type>(new obj_type(ins));
            return rs;
        }

        template <class T>
        static auto get_right_interval_seeker(const data::StorageExtractible<T> extractor){
            
            RightIntervalSeeker ins(extractor);

            using rs_type   = std::shared_ptr<std::remove_pointer_t<decltype(ins.to_seekable())>>;
            using obj_type  = decltype(ins);

            rs_type rs      = std::unique_ptr<obj_type>(new obj_type(ins));
            return rs;
        }
    };

    struct SeekerLambdanizer{

        template <class T>
        static constexpr auto get_root_seeker(std::shared_ptr<Seekable<T>> seeker){

            auto root   = std::integral_constant<size_t, 0u>(); 
            auto rs     = [=]{return seeker->seek(root);};

            return rs;
        }

        template <class T>
        static constexpr auto get_greedy_batch_seeker(std::shared_ptr<Seekable<T>> seeker){
            
            using _Algo = GreedyBatchSeeker; 
            auto rs     = [=]{return _Algo::get(*seeker->to_seekable());}; //

            return rs;
        } 
    };

}

namespace dg::heap::interval_ops{
    
    template <size_t HEIGHT>
    struct IntervalTracer{

        using store_type        = types::store_type;
        using _HeapUtility      = utility::HeapUtility<HEIGHT>;
        using _IntervalUtility  = utility::IntervalUtility<HEIGHT>;

        template <size_t ARG_HEIGHT, class l_cb_lambda, class r_cb_lambda, class T>
        static inline void trace_left_at(const data::StorageExtractible<T> extractor, 
                                         size_t idx, 
                                         const l_cb_lambda& l_cb, 
                                         const r_cb_lambda& r_cb){
            
            constexpr store_type SPAN_SZ = _IntervalUtility::span_size_from_height(ARG_HEIGHT + 1); 

            store_type ll = extractor.template get_left_at<ARG_HEIGHT + 1>(_HeapUtility::left(idx));
            store_type rl = extractor.template get_left_at<ARG_HEIGHT + 1>(_HeapUtility::right(idx)); 

            if ((ll == SPAN_SZ) && (rl != 0)){
                r_cb(ll + rl);
            } else{
                l_cb(ll);
            }
        } 

        template <size_t ARG_HEIGHT, class l_cb_lambda, class r_cb_lambda, class T>
        static inline void trace_right_at(const data::StorageExtractible<T> extractor, 
                                          size_t idx, 
                                          const l_cb_lambda& l_cb, 
                                          const r_cb_lambda& r_cb){
            
            constexpr store_type SPAN_SZ = _IntervalUtility::span_size_from_height(ARG_HEIGHT + 1);

            store_type rr = extractor.template get_right_at<ARG_HEIGHT + 1>(_HeapUtility::right(idx));
            store_type lr = extractor.template get_right_at<ARG_HEIGHT + 1>(_HeapUtility::left(idx)); 

            if ((rr == SPAN_SZ) && (lr != 0)){
                l_cb(rr + lr);
            } else{
                r_cb(rr);
            }
        }

        template <size_t ARG_HEIGHT, class l_cb_lambda, class mid_cb_lambda, class r_cb_lambda, class T>
        static inline void trace_center_at(const data::StorageExtractible<T> extractor, 
                                           size_t idx, 
                                           const l_cb_lambda& l_cb, 
                                           const mid_cb_lambda& mid_cb, 
                                           const r_cb_lambda& r_cb){
            
            store_type lc  = extractor.template get_center_at<ARG_HEIGHT + 1>(_HeapUtility::left(idx));
            store_type rc  = extractor.template get_center_at<ARG_HEIGHT + 1>(_HeapUtility::right(idx));
            store_type mid = extractor.template get_right_at<ARG_HEIGHT + 1>(_HeapUtility::left(idx)) + extractor.template get_left_at<ARG_HEIGHT + 1>(_HeapUtility::right(idx));
            
            if ((mid > lc) && (mid > rc)){
                mid_cb(mid); 
            } else if (rc > lc){
                r_cb(rc);
            } else{
                l_cb(lc);
            }
        }
    };

    template <size_t HEIGHT>
    struct IntervalApplier{

        using interval_type     = types::interval_type;

        using _HeapUtility      = utility::HeapUtility<HEIGHT>; 
        using _IntervalUtility  = utility::IntervalUtility<HEIGHT>; 

        template <size_t ARG_HEIGHT, class StopCond, class ApplyCallBack, class PostCallBack, class RightExclCallback, class LeftExclCallback>
        static inline void apply(size_t idx,
                                 const interval_type& interval, 
                                 const interval_type& key_interval,
                                 const StopCond& stop_cond,
                                 const ApplyCallBack& apply_cb,
                                 const PostCallBack& post_cb,
                                 const RightExclCallback& right_excl_cb,
                                 const LeftExclCallback& left_excl_cb){
            
            if constexpr(ARG_HEIGHT <= HEIGHT){

                constexpr auto HEIGHT_IC = std::integral_constant<size_t, ARG_HEIGHT>(); 

                if (stop_cond(HEIGHT_IC, interval, idx, key_interval)){
                    
                    apply_cb(HEIGHT_IC, interval, idx); //if not use lambda -> memory indirection -> perf issue (if use lambda then readability issue !)
                    return;

                }

                auto midpoint = _IntervalUtility::midpoint(interval);

                if (_IntervalUtility::is_left_bound(key_interval, midpoint)){
                    apply<ARG_HEIGHT + 1>(_HeapUtility::left(idx), _IntervalUtility::left_shrink(interval, midpoint), key_interval, stop_cond, apply_cb, post_cb, right_excl_cb, left_excl_cb); //memory bottleneck or computation - need profiling
                    left_excl_cb(HEIGHT_IC, interval, idx);
                } else if (_IntervalUtility::is_right_bound(key_interval, midpoint)){
                    apply<ARG_HEIGHT + 1>(_HeapUtility::right(idx), _IntervalUtility::right_shrink(interval, midpoint), key_interval, stop_cond, apply_cb, post_cb, right_excl_cb, left_excl_cb);
                    right_excl_cb(HEIGHT_IC, interval, idx);
                } else{
                    apply<ARG_HEIGHT + 1>(_HeapUtility::left(idx), _IntervalUtility::left_shrink(interval, midpoint), _IntervalUtility::left_shrink(key_interval, midpoint), stop_cond, apply_cb, post_cb, right_excl_cb, left_excl_cb);
                    apply<ARG_HEIGHT + 1>(_HeapUtility::right(idx), _IntervalUtility::right_shrink(interval, midpoint), _IntervalUtility::right_shrink(key_interval, midpoint), stop_cond, apply_cb, post_cb, right_excl_cb, left_excl_cb);
                }

                post_cb(HEIGHT_IC, interval, idx);
            }
        }
    };
    
    template <size_t HEIGHT>
    struct IntervalTraverser{

        using interval_type     = types::interval_type;
        using _HeapUtility      = utility::HeapEssential; 
        using _IntervalUtility  = utility::IntervalUtility<HEIGHT>; 

        template <size_t ARG_HEIGHT, class CallBack, class PostCallBack, class StopCond>
        static inline void traverse(size_t idx,
                                    const interval_type& interval,
                                    const CallBack& cb_lambda,
                                    const PostCallBack& post_cb_lambda,
                                    const StopCond& stop_cond){
            
            //REVIEW: remove template - code size reduction 

            if constexpr(ARG_HEIGHT <= HEIGHT){
                
                constexpr auto HEIGHT_IC = std::integral_constant<size_t, ARG_HEIGHT>(); 

                if (stop_cond(HEIGHT_IC, interval, idx)){
                    cb_lambda(HEIGHT_IC, interval, idx);
                    return;
                }

                traverse<ARG_HEIGHT + 1>(_HeapUtility::left(idx),  _IntervalUtility::left_interval(interval),  cb_lambda, post_cb_lambda, stop_cond);
                traverse<ARG_HEIGHT + 1>(_HeapUtility::right(idx), _IntervalUtility::right_interval(interval), cb_lambda, post_cb_lambda, stop_cond);

                post_cb_lambda(HEIGHT_IC, interval, idx);
            }
        } 

        template <class CallBack, class PostCallBack, class StopCond>
        static inline void traverse(const CallBack& cb_lambda, 
                                    const PostCallBack& post_cb_lambda, 
                                    const StopCond& stop_cond){
            
            constexpr auto START_IDX    = size_t{0u};
            constexpr auto START_HEIGHT = _HeapUtility::idx_to_height(START_IDX);

            traverse<START_HEIGHT>(START_IDX, _IntervalUtility::idx_to_interval(START_IDX), cb_lambda, post_cb_lambda, stop_cond);
        }
    };

    struct IntervalApplierLambdaGenerator{
        
        using interval_type     = types::interval_type;

        template <size_t TREE_HEIGHT, class StopCond, class ApplyCallBack, class PostCallBack, class RightExclCallBack, class LeftExclCallBack>
        static constexpr auto get(const std::integral_constant<size_t, TREE_HEIGHT>&,
                                  const StopCond& stop_cond,
                                  const ApplyCallBack& apply_cb,
                                  const PostCallBack& post_cb,
                                  const RightExclCallBack& right_excl,
                                  const LeftExclCallBack& left_excl){
            
            using _IntervalApplier  = interval_ops::IntervalApplier<TREE_HEIGHT>;
            using _IntvUlt          = utility::IntervalUtility<TREE_HEIGHT>;

            auto rs = [=]<size_t IDX_HEIGHT>(const std::integral_constant<size_t, IDX_HEIGHT>&, const interval_type& key_interval, size_t idx){ //feels like forcing the interface to fit a specific use case here

                auto intv   = _IntvUlt::template idx_to_interval<IDX_HEIGHT>(idx); 

                _IntervalApplier::template apply<IDX_HEIGHT>(idx, intv, key_interval, stop_cond, 
                                                             apply_cb, post_cb, right_excl, left_excl);

            };

            return rs;
        } 
    };

};

namespace dg::heap::batch_interval_ops{
    
    struct BatchAssorter{

        using interval_type     = types::interval_type;
        
        using _IntervalUtility  = utility::IntervalEssential;
        using _IntervalLambda   = utility::IntervalEssentialLambdanizer;
        using _NumericUtility   = utility::NumericUtility;
        using _LambdaUlt        = utility::LambdaUtility;
        using _IterUlt          = utility::IteratorUtility;

        template <class _Iterator> 
        static constexpr auto inplace_sort(_Iterator first, _Iterator last) -> std::pair<_Iterator, _Iterator>{

            auto cmp_lambda = [](const interval_type& lhs, const interval_type& rhs){
                return _IntervalUtility::get_interval_beg(lhs) < _IntervalUtility::get_interval_beg(rhs);
            };
            
            std::sort(first, last, cmp_lambda);

            return {first, last};
        }

        template <class _Iterator>
        static constexpr auto inplace_shrink(_Iterator first, _Iterator last) -> std::pair<_Iterator, _Iterator>{

           using element_type = typename std::remove_reference_t<decltype(_IterUlt::meat(std::declval<_Iterator>()))>; 

            auto ptr        = first;
            auto i          = first;
            element_type rs;
            
            while (i != last){

                std::tie(rs, i)     = _NumericUtility::accumulate_until(_IntervalLambda::uunion, i, last, _LambdaUlt::negate(_IntervalLambda::is_consecutive));
                _IterUlt::meat(ptr) = rs;
                ptr                 = std::next(ptr);
            
            } 

            return {first, ptr}; 
        } 

        template <class _Iterator>
        static constexpr auto inplace_assort(_Iterator first, _Iterator last) -> std::pair<_Iterator, _Iterator>{

            return utility::piecewise_invoke(inplace_shrink<_Iterator>, inplace_sort(first, last));
        }
    };

    template <size_t HEIGHT>
    struct IntervalApplier{

        using interval_type     = types::interval_type;

        using _IteratorUlt      = utility::IteratorUtility;
        using _IntervalUlt      = utility::IntervalEssential_P;
        using _HeapUlt          = utility::HeapEssential;

        //assume asc order {first, last}
        template <size_t ARG_HEIGHT, class Iterator, class StopCond, class CallBack, class PostCallBack, class RightExclCallBack, class LeftExclCallBack>
        static inline void apply(size_t idx, 
                                 const interval_type& interval,
                                 Iterator first,
                                 Iterator last, 
                                 const StopCond& stop_cond,
                                 const CallBack& callback, 
                                 const PostCallBack& post_cb,
                                 const RightExclCallBack& rex_cb,
                                 const LeftExclCallBack& lex_cb){
            
            if constexpr(ARG_HEIGHT <= HEIGHT){

                constexpr auto HEIGHT_IC    = std::integral_constant<size_t, ARG_HEIGHT>(); 

                if (stop_cond(HEIGHT_IC, interval, idx, first, last)){

                    callback(HEIGHT_IC, interval, idx, first, last);
                    return;

                }

                auto midpoint   = _IntervalUlt::midpoint(interval);
                auto back       = _IteratorUlt::prev_last(first, last);

                if (_IntervalUlt::is_left_bound(_IteratorUlt::meat(back), midpoint)){

                    apply<ARG_HEIGHT + 1>(_HeapUlt::left(idx), _IntervalUlt::left_shrink(interval, midpoint), first, last, stop_cond, callback, post_cb, rex_cb, lex_cb);
                    lex_cb(HEIGHT_IC, interval, idx);

                } else if (_IntervalUlt::is_right_bound(_IteratorUlt::meat(first), midpoint)){

                    apply<ARG_HEIGHT + 1>(_HeapUlt::right(idx), _IntervalUlt::right_shrink(interval, midpoint), first, last, stop_cond, callback, post_cb, rex_cb, lex_cb);
                    rex_cb(HEIGHT_IC, interval, idx);

                } else{
                    
                    auto[l_excl, r_incl] = _IntervalUlt::pair_shrink(first, last, midpoint); 

                    apply<ARG_HEIGHT + 1>(_HeapUlt::left(idx), _IntervalUlt::left_shrink(interval, midpoint), first, l_excl, stop_cond, callback, post_cb, rex_cb, lex_cb);
                    apply<ARG_HEIGHT + 1>(_HeapUlt::right(idx), _IntervalUlt::right_shrink(interval, midpoint), r_incl, last, stop_cond, callback, post_cb, rex_cb, lex_cb);

                }

                post_cb(HEIGHT_IC, interval, idx);
            }
        }
    };

    struct IntervalApplierLambdaGenerator{

        using interval_type = types::interval_type;

        template <size_t TREE_HEIGHT, class StopCond, class CallBack, class PostCallBack, class RightExclCallBack, class LeftExclCallBack>
        static constexpr auto get(const std::integral_constant<size_t, TREE_HEIGHT>&,
                                  const StopCond& stop_cond,
                                  const CallBack& callback,
                                  const PostCallBack& post_cb,
                                  const RightExclCallBack& rex_cb,
                                  const LeftExclCallBack& lex_cb){
            
            using _IntervalApplier = IntervalApplier<TREE_HEIGHT>; 

            auto rs = [=]<size_t IDX_HEIGHT, class Iterator>(const std::integral_constant<size_t, IDX_HEIGHT>&, const interval_type& intv, size_t idx, Iterator first, Iterator last){
                _IntervalApplier::template apply<IDX_HEIGHT>(idx, intv, first, last, stop_cond, callback, post_cb, rex_cb, lex_cb);
            };

            return rs;
        }
    };

}

namespace dg::heap::interval_ops_injection{

    struct IntervalApplierStopCondGenerator{

        using interval_type = types::interval_type;
        using _LambdaUlt    = utility::LambdaUtility;

        static constexpr auto deflt(){

            auto rs = []<size_t ARG_HEIGHT>(const std::integral_constant<size_t, ARG_HEIGHT>&, 
                                            const interval_type& intv,
                                            size_t idx,
                                            const interval_type& key_intv){
                return intv == key_intv;
            };

            return rs;
        }

        static constexpr auto get_filter(){

            auto rs = []<size_t ARG_HEIGHT>(const std::integral_constant<size_t, ARG_HEIGHT>& ic,
                                            const interval_type& intv,
                                            size_t idx,
                                            const interval_type& key_intv){
                return std::make_tuple(ic, intv, idx);
            };

            return rs;
        }   

        template <class T>
        static constexpr auto customize(const T& stop_cond){

            return _LambdaUlt::bind_filter_n_deflate(stop_cond, get_filter());
        }

    };

    struct BatchIntervalApplierStopCondGenerator{
        
        using interval_type = types::interval_type;
        using _IteratorUlt  = utility::IteratorUtility;
        using _LambdaUlt    = utility::LambdaUtility;

        static constexpr auto deflt(){

            auto rs = []<size_t ARG_HEIGHT, class Iterator>(const std::integral_constant<size_t, ARG_HEIGHT>&,
                                                            const interval_type&,
                                                            size_t, 
                                                            Iterator first, 
                                                            Iterator last){
                return _IteratorUlt::is_equal(std::next(first), last);
            };

            return rs;
        }

        static constexpr auto get_filter(){

            auto rs = []<size_t ARG_HEIGHT, class Iterator>(const std::integral_constant<size_t, ARG_HEIGHT>& height,
                                                            const interval_type& intv,
                                                            size_t idx,
                                                            Iterator,
                                                            Iterator){
                return std::make_tuple(height, intv, idx);
            };

            return rs;
        }

        template <class T>
        static constexpr auto customize(const T& stop_cond){

            return _LambdaUlt::bind_filter_n_deflate(stop_cond, get_filter());
        }

    };

    struct HeapOperatorLambdaGenerator{

        using store_type        = types::store_type;
        using interval_type     = types::interval_type; 

        //REVIEW: RTTI this and benchmark
        template <class T>
        static constexpr auto get_block_lambda(const internal_core::HeapOperatable<T>){

            auto rs = []<size_t ARG_HEIGHT>(const std::integral_constant<size_t, ARG_HEIGHT>&, 
                                            const interval_type&, 
                                            size_t idx){
                internal_core::HeapOperatable<T>::template block<ARG_HEIGHT>(idx);
            }; 

            return rs;
        } 

        template <class T>
        static constexpr auto get_unblock_lambda(const internal_core::HeapOperatable<T>){

            auto rs = []<size_t ARG_HEIGHT>(const std::integral_constant<size_t, ARG_HEIGHT>&,
                                            const interval_type&, 
                                            size_t idx){
                internal_core::HeapOperatable<T>::template unblock<ARG_HEIGHT>(idx);
            };

            return rs;
        } 

        template <class T>
        static constexpr auto get_update_lambda(const internal_core::HeapOperatable<T>){

            auto rs = []<size_t ARG_HEIGHT>(const std::integral_constant<size_t, ARG_HEIGHT>&, 
                                            const interval_type&,
                                            size_t idx){
                internal_core::HeapOperatable<T>::template update<ARG_HEIGHT>(idx);
            };

            return rs;
        }
        
        template <class T>
        static constexpr auto get_is_blocked_lambda(const internal_core::HeapOperatable<T>){

            auto rs = []<size_t ARG_HEIGHT>(const std::integral_constant<size_t, ARG_HEIGHT>&,
                                            const interval_type&,
                                            size_t idx){                  
                return internal_core::HeapOperatable<T>::template is_blocked<ARG_HEIGHT>(idx);
            };

            return rs;
        }
    };

    struct FilterLambdaGenerator{

        using interval_type     = types::interval_type;

        using _IntervalUlt      = utility::IntervalEssential;
        using _HeapUlt          = utility::HeapEssential;

        static constexpr auto right(){
            
            auto transformed = []<size_t ARG_HEIGHT>(const std::integral_constant<size_t, ARG_HEIGHT>& height,
                                                     const interval_type& intv,
                                                     size_t idx){

                auto new_height     = _HeapUlt::next_height(height);
                auto new_intv       = _IntervalUlt::right_interval(intv);
                auto new_idx        = _HeapUlt::right(idx);

                return std::make_tuple(new_height, new_intv, new_idx);

            };

            return transformed;
        }

        static constexpr auto left(){

            auto transformed = []<size_t ARG_HEIGHT>(const std::integral_constant<size_t, ARG_HEIGHT>& height,
                                                     const interval_type& intv,
                                                     size_t idx){
            
                auto new_height     = _HeapUlt::next_height(height);
                auto new_intv       = _IntervalUlt::left_interval(intv);
                auto new_idx        = _HeapUlt::left(idx);

                return std::make_tuple(new_height, new_intv, new_idx);

            };

            return transformed;
        }

        static constexpr auto intersect(const interval_type& interval){

           auto transformed = [=]<size_t ARG_HEIGHT>(const std::integral_constant<size_t, ARG_HEIGHT>& height,
                                                     const interval_type& intv,
                                                     size_t idx){
            
                auto new_intv       = _IntervalUlt::intersect(intv, interval);

                return std::make_tuple(height, new_intv, idx);

            };

            return transformed;
        }

        static constexpr auto extract_interval(){

            auto transformed = []<size_t ARG_HEIGHT>(const std::integral_constant<size_t, ARG_HEIGHT>&,
                                                     const interval_type& intv,
                                                     size_t){
            
                return std::make_tuple(intv);
            };

            return transformed;
        }

        static constexpr auto extract_height_v(){

            auto transformed = []<size_t ARG_HEIGHT>(const std::integral_constant<size_t, ARG_HEIGHT>&,
                                                      const interval_type& intv,
                                                      size_t){
            
                return std::make_tuple(ARG_HEIGHT);
            };

            return transformed;
        }
    };

}

namespace dg::heap::dispatcher{

    struct DiscreteBlockDispatcher{
        
        using interval_type     = types::interval_type;
        using _StopCondGen      = interval_ops_injection::IntervalApplierStopCondGenerator;
        using _LambdaGen        = interval_ops_injection::HeapOperatorLambdaGenerator;
        using _LambdaUlt        = utility::LambdaUtility;
        using _HeapUtility      = utility::HeapEssential;

        template <class T>
        static inline void dispatch_block(const internal_core::HeapOperatable<T> ops, const interval_type& key_interval) noexcept{

            constexpr auto HEIGHT         = internal_core::HeapOperatable<T>::HEIGHT;
            using _IntervalApplier        = interval_ops::IntervalApplier<HEIGHT>;
            using _IntvUtility            = utility::IntervalUtility<HEIGHT>;

            constexpr auto START_IDX      = size_t{0u};
            constexpr auto START_HEIGHT   = _HeapUtility::idx_to_height(START_IDX);
            constexpr auto START_INTV     = _IntvUtility::idx_to_interval(START_IDX);

            _IntervalApplier::template apply<START_HEIGHT>(START_IDX, 
                                                           START_INTV, 
                                                           key_interval,
                                                           _StopCondGen::deflt(),
                                                           _LambdaGen::get_block_lambda(ops), 
                                                           _LambdaGen::get_update_lambda(ops),
                                                           _LambdaUlt::get_null_lambda(),
                                                           _LambdaUlt::get_null_lambda());
        } 
    };
    
    struct DiscreteUnblockDispatcher{

        using interval_type     = types::interval_type;
        using _LambdaUlt        = utility::LambdaUtility;
        using _StopCond         = interval_ops_injection::IntervalApplierStopCondGenerator;
        using _LambdaGen        = interval_ops_injection::HeapOperatorLambdaGenerator;
        using _HeapUtility      = utility::HeapEssential;

        template <class T>
        static inline void dispatch_unblock(const internal_core::HeapOperatable<T> ops, const interval_type& key_interval) noexcept{

            constexpr auto HEIGHT       = internal_core::HeapOperatable<T>::HEIGHT;
            using _IntvUtility          = utility::IntervalUtility<HEIGHT>;
            using _IntervalApplier      = interval_ops::IntervalApplier<HEIGHT>;

            constexpr auto START_IDX    = size_t{0u};
            constexpr auto START_HEIGHT = _HeapUtility::idx_to_height(START_IDX);
            constexpr auto START_INTV   = _IntvUtility::idx_to_interval(START_IDX);

            _IntervalApplier::template apply<START_HEIGHT>(START_IDX,
                                                           START_INTV,
                                                           key_interval,
                                                           _StopCond::deflt(),
                                                           _LambdaGen::get_unblock_lambda(ops),
                                                           _LambdaGen::get_update_lambda(ops),
                                                           _LambdaUlt::get_null_lambda(),
                                                           _LambdaUlt::get_null_lambda());
        }
    };

    struct PartialUnblockDispatcher{

        using interval_type             = types::interval_type; 
        using _LambdaUlt                = utility::LambdaUtility;
        using _LambdaGen                = interval_ops_injection::HeapOperatorLambdaGenerator; 
        using _LambdaFil                = interval_ops_injection::FilterLambdaGenerator;
        using _IntervalApplierLambda    = interval_ops::IntervalApplierLambdaGenerator;
        using _StopCondGen              = interval_ops_injection::IntervalApplierStopCondGenerator;
        using _HeapUtility              = utility::HeapEssential;

        template <class T> 
        static inline void dispatch_unblock(const internal_core::HeapOperatable<T> ops, const interval_type& key_interval) noexcept{
            
            constexpr auto HEIGHT           = internal_core::HeapOperatable<T>::HEIGHT;
            constexpr auto START_IDX        = size_t{0u};
            constexpr auto START_HEIGHT     = _HeapUtility::idx_to_height(START_IDX);

            auto is_blocked_lambda          = _LambdaGen::get_is_blocked_lambda(ops);
            auto unblock_lamda              = _LambdaGen::get_unblock_lambda(ops);
            auto block_lambda               = _LambdaGen::get_block_lambda(ops);
            auto post_lambda                = _LambdaGen::get_update_lambda(ops);
            auto right_excl_lambda          = _LambdaUlt::bind_filter_n_deflate(block_lambda, _LambdaFil::left());
            auto left_excl_lambda           = _LambdaUlt::bind_filter_n_deflate(block_lambda, _LambdaFil::right());
            
            auto worker_lambda              = _IntervalApplierLambda::get(std::integral_constant<size_t, HEIGHT>{}, _StopCondGen::deflt(), _LambdaUlt::get_null_lambda(), post_lambda, right_excl_lambda, left_excl_lambda); 
            auto downstream_worker_lambda   = _LambdaUlt::bind_filter_n_deflate(worker_lambda, _LambdaFil::intersect(key_interval));
            auto transfer_lambda            = _LambdaUlt::bind_void_layer(downstream_worker_lambda, unblock_lamda); 
            auto upstream_worker_lambda     = _IntervalApplierLambda::get(std::integral_constant<size_t, HEIGHT>{}, _StopCondGen::customize(is_blocked_lambda), transfer_lambda, post_lambda, _LambdaUlt::get_null_lambda(), _LambdaUlt::get_null_lambda());

            upstream_worker_lambda(std::integral_constant<size_t, START_HEIGHT>{}, key_interval, START_IDX);
        }
    };

    struct StdDispatcher: private DiscreteBlockDispatcher, private PartialUnblockDispatcher{

        using _Blocker      = DiscreteBlockDispatcher;
        using _Unblocker    = PartialUnblockDispatcher;

        using _Blocker::dispatch_block;
        using _Unblocker::dispatch_unblock;
    };

    struct DiscreteDispatcher: private DiscreteBlockDispatcher, private DiscreteUnblockDispatcher{
        
        using _Blocker      = DiscreteBlockDispatcher;
        using _Unblocker    = DiscreteUnblockDispatcher;

        using _Blocker::dispatch_block;
        using _Unblocker::dispatch_unblock;
    }; 

    //REVIEW: refactoring required (rtti required - reduce templates)
    struct DiscreteBatchBlockDispatcher{

        using interval_type     = types::interval_type;
        using _LambdaGen        = interval_ops_injection:: HeapOperatorLambdaGenerator;
        using _SStopCond        = interval_ops_injection::IntervalApplierStopCondGenerator; 
        using _BStopCond        = interval_ops_injection::BatchIntervalApplierStopCondGenerator;
        using _IterUlt          = utility::IteratorUtility;
        using _LambdaUlt        = utility::LambdaUtility;
        
        template <class T>
        static inline void get_singular_handler(const internal_core::HeapOperatable<T> ops) noexcept{
            
            constexpr auto HEIGHT   = internal_core::HeapOperatable<T>::HEIGHT;
            using _Applier          = interval_ops::IntervalApplier<HEIGHT>;
            using _IntvUlt          = utility::IntervalUtility<HEIGHT>;

            auto rs = [=]<size_t ARG_HEIGHT, class Iterator>(const std::integral_constant<size_t, ARG_HEIGHT>&, 
                                                             const interval_type& interval,
                                                             size_t idx, 
                                                             Iterator key,
                                                             Iterator){
                
                _Applier::template apply<ARG_HEIGHT>(idx, 
                                                     interval, 
                                                     _IntvUlt::intersect(_IterUlt::meat(key), interval), 
                                                     _SStopCond::deflt(),
                                                     _LambdaGen::get_block_lambda(ops), 
                                                     _LambdaGen::get_update_lambda(ops),
                                                     _LambdaUlt::get_null_lambda(),
                                                     _LambdaUlt::get_null_lambda());

            };

            return rs;
        }

        template <class T, class Iterator>
        static inline void dispatch_block(const internal_core::HeapOperatable<T> ops, Iterator first, Iterator last) noexcept{

            constexpr auto HEIGHT   = internal_core::HeapOperatable<T>::HEIGHT;
            using _BatchApplier     = batch_interval_ops::IntervalApplier<HEIGHT>;
            using _HeapUlt          = utility::HeapUtility<HEIGHT>;
            using _IntvUlt          = utility::IntervalUtility<HEIGHT>;

            constexpr auto START_IDX        = size_t{0u};
            constexpr auto START_HEIGHT     = _HeapUlt::idx_to_height(START_IDX);
            constexpr auto START_INTV       = _IntvUlt::idx_to_interval(START_IDX);

            _BatchApplier::template apply<START_HEIGHT>(START_IDX, START_INTV, first, last, 
                                                        _BStopCond::deflt(),
                                                        get_singular_handler(ops), 
                                                        _LambdaGen::get_update_lambda(ops),
                                                        _LambdaUlt::get_null_lambda(),
                                                        _LambdaUlt::get_null_lambda());
        }
    };

    //REVIEW: refactoring required (rtti required - reduce templates)
    struct PartialBatchUnblockDispatcher{

        using interval_type         = types::interval_type;
        using _LambdaGen            = interval_ops_injection::HeapOperatorLambdaGenerator;
        using _LambdaFil            = interval_ops_injection::FilterLambdaGenerator;
        using _SStopCond            = interval_ops_injection::IntervalApplierStopCondGenerator;
        using _BStopCond            = interval_ops_injection::BatchIntervalApplierStopCondGenerator;
        using _LambdaUlt            = utility::LambdaUtility;
        using _BatchApplierLambda   = batch_interval_ops::IntervalApplierLambdaGenerator;
        using _IterUlt              = utility::IteratorUtility;

        template <class T>
        static constexpr auto get_singular_handler(const internal_core::HeapOperatable<T> ops){

            constexpr auto HEIGHT       = internal_core::HeapOperatable<T>::HEIGHT;
            using _Applier              = interval_ops::IntervalApplier<HEIGHT>;
            using _IntvUlt              = utility::IntervalUtility<HEIGHT>;

            auto rs = [=]<size_t ARG_HEIGHT, class Iterator>(const std::integral_constant<size_t, ARG_HEIGHT>&,
                                                             const interval_type& interval,
                                                             size_t idx,
                                                             Iterator key,
                                                             Iterator){
                
                _Applier::template apply<ARG_HEIGHT>(idx, 
                                                     interval, 
                                                     _IntvUlt::intersect(_IterUlt::meat(key), interval), 
                                                     _SStopCond::deflt(),
                                                     _LambdaUlt::get_null_lambda(),
                                                     _LambdaGen::get_update_lambda(ops),
                                                     _LambdaUlt::bind_filter_n_deflate(_LambdaGen::get_block_lambda(ops), _LambdaFil::left()),
                                                     _LambdaUlt::bind_filter_n_deflate(_LambdaGen::get_block_lambda(ops), _LambdaFil::right()));

            };

            return rs;
        }

        static constexpr auto get_shrink_filter(){

            auto rs = []<size_t ARG_HEIGHT, class Iterator>(const std::integral_constant<size_t, ARG_HEIGHT>& ic,
                                                            const interval_type& interval,
                                                            size_t idx,
                                                            Iterator first,
                                                            Iterator last){
                                    
                auto[ffirst, llast] = batch_interval_ops::BatchAssorter::inplace_shrink(first, last); //last[-1] is unchanged - assume last[-1] is changed => last[-1] is a result of a combination => new_length < org_length => last[-1] is unchanged (contradiction)
                return std::make_tuple(ic, interval, idx, ffirst, llast); 

            };

            return rs;
        }

        template <class T, class Iterator>
        static inline void dispatch_unblock(const internal_core::HeapOperatable<T> ops, Iterator first, Iterator last) noexcept{
            
            constexpr auto HEIGHT           = internal_core::HeapOperatable<T>::HEIGHT;
            using _IntvUlt                  = utility::IntervalUtility<HEIGHT>;
            using _HeapUlt                  = utility::HeapUtility<HEIGHT>;

            constexpr auto START_IDX        = size_t{0u};
            constexpr auto START_HEIGHT     = _HeapUlt::idx_to_height(START_IDX);
            constexpr auto START_INTV       = _IntvUlt::idx_to_interval(START_IDX);

            auto updater                    = _LambdaGen::get_update_lambda(ops);
            auto blocker                    = _LambdaGen::get_block_lambda(ops);
            auto unblocker                  = _LambdaGen::get_unblock_lambda(ops);
            auto is_blocked_verifier        = _LambdaGen::get_is_blocked_lambda(ops);  

            auto downstream_worker          = _BatchApplierLambda::get(std::integral_constant<size_t, HEIGHT>{},
                                                                       _BStopCond::deflt(), 
                                                                       get_singular_handler(ops), 
                                                                       updater, 
                                                                       _LambdaUlt::bind_filter_n_deflate(blocker, _LambdaFil::left()), 
                                                                       _LambdaUlt::bind_filter_n_deflate(blocker, _LambdaFil::right()));

            auto ddownstream_worker         = _LambdaUlt::bind_filter_n_deflate(downstream_worker, get_shrink_filter());
            auto intermediate_worker        = _LambdaUlt::bind_filter_n_deflate(unblocker, _BStopCond::get_filter()); // semantics 
            auto transfer_worker            = _LambdaUlt::bind_void_layer(ddownstream_worker, intermediate_worker);

            auto upstream_worker            = _BatchApplierLambda::get(std::integral_constant<size_t, HEIGHT>{},
                                                                       _BStopCond::customize(is_blocked_verifier),
                                                                       transfer_worker, 
                                                                       updater,
                                                                       _LambdaUlt::get_null_lambda(),
                                                                       _LambdaUlt::get_null_lambda());
            
            upstream_worker(std::integral_constant<size_t, START_HEIGHT>{}, START_INTV, START_IDX, first, last);
        } 
    };

    struct BatchDispatcher: private DiscreteBatchBlockDispatcher, private PartialBatchUnblockDispatcher{
        
        using _Blocker      = DiscreteBatchBlockDispatcher;
        using _Unblocker    = PartialBatchUnblockDispatcher;

        using _Blocker::dispatch_block;
        using _Unblocker::dispatch_unblock;
    }; 

    struct DispatcherSpawner{
        
        using interval_type = types::interval_type;

        template <class T>
        static constexpr auto get_discrete_block_dispatcher(const internal_core::HeapOperatable<T> ops){

            using Dispatcher = DiscreteDispatcher;

            auto lambda =[=](const interval_type& intv) noexcept{
                Dispatcher::dispatch_block(ops, intv);
            };

            return lambda;
        }

        template <class T>
        static constexpr auto get_discrete_unblock_dispatcher(const internal_core::HeapOperatable<T> ops){

            using Dispatcher = DiscreteDispatcher;

            auto lambda = [=](const interval_type& intv) noexcept{
                Dispatcher::dispatch_unblock(ops, intv);
            };

            return lambda;
        }

        template <class T>
        static constexpr auto get_std_block_dispatcher(const internal_core::HeapOperatable<T> ops){
            
            using Dispatcher = StdDispatcher; 

            auto lambda = [=](const interval_type& intv) noexcept{ 
                Dispatcher::dispatch_block(ops, intv);
            };

            return lambda;
        } 

        template <class T>
        static constexpr auto get_std_unblock_dispatcher(const internal_core::HeapOperatable<T> ops){

            using Dispatcher = StdDispatcher;

            auto lambda = [=](const interval_type& intv) noexcept{
                Dispatcher::dispatch_unblock(ops, intv);
            };

            return lambda;
        }

        template <class T>
        static constexpr auto get_batch_block_dispatcher(const internal_core::HeapOperatable<T> ops){

            using Dispatcher = BatchDispatcher;

            auto lambda = [=]<class Iterator>(Iterator first, Iterator last) noexcept{
                Dispatcher::dispatch_block(ops, first, last);
            };

            return lambda;
        }

        template <class T>
        static constexpr auto get_batch_unblock_dispatcher(const internal_core::HeapOperatable<T> ops){

            using Dispatcher = BatchDispatcher;

            auto lambda = [=]<class Iterator>(Iterator first, Iterator last) noexcept{
                Dispatcher::dispatch_unblock(ops, first, last);
            };

            return lambda;
        }
    };

    template <class Lambda>
    class DispatcherWrapper: public Dispatchable<DispatcherWrapper<Lambda>>{

        private:

            Lambda dispatcher_ins;
        
        public:

            using interval_type = types::interval_type;

            DispatcherWrapper(Lambda dispatcher_ins): dispatcher_ins(dispatcher_ins){}

            void dispatch(const interval_type& intv) noexcept{
                
                static_assert(noexcept(this->dispatcher_ins(intv)));
                this->dispatcher_ins(intv);
            }
    };

    template <class Lambda>
    class BatchDispatcherWrapper: public BatchDispatchable<BatchDispatcherWrapper<Lambda>>{

        private:

            Lambda dispatcher_ins;
        
        public:

            BatchDispatcherWrapper(Lambda dispatcher_ins): dispatcher_ins(dispatcher_ins){}

            template <class Iterator>
            void dispatch(Iterator first, Iterator last) noexcept{
                
                static_assert(noexcept(this->dispatcher_ins(first, last)));
                this->dispatcher_ins(first, last);
            }
    };

    struct DispatcherWrapperSpawner{

        template <class T>
        static auto get_std_wrapper(T functor){

            DispatcherWrapper wrapper(functor);
            using rs_type   = std::shared_ptr<std::remove_pointer_t<decltype(wrapper.to_dispatchable())>>;
            using ins_type  = decltype(wrapper);
            rs_type rs      = std::unique_ptr<ins_type>(new ins_type(wrapper));
            
            return rs;
        }
    };
}

namespace dg::heap::instantiator{

    struct IntervalDataInstantiator{    

        using interval_type     = types::interval_type;
        using _LambdaGen        = interval_ops_injection::HeapOperatorLambdaGenerator;
        using _LambdaFil        = interval_ops_injection::FilterLambdaGenerator;
        using _LambdaUlt        = utility::LambdaUtility;

        template <class T>
        static inline void initialize(const internal_core::HeapOperatable<T> ops){
            
            constexpr auto HEIGHT   = internal_core::HeapOperatable<T>::HEIGHT;
            using _HeapUtility      = utility::HeapUtility<HEIGHT>;
            using _Traverser        = interval_ops::IntervalTraverser<HEIGHT>; 

            auto stop_cond          = _LambdaUlt::bind_filter_n_deflate(_HeapUtility::is_base, _LambdaFil::extract_height_v());
            auto leaf_ins           = _LambdaGen::get_unblock_lambda(ops);
            auto post_ins           = _LambdaUlt::void_aggregate(_LambdaGen::get_unblock_lambda(ops), _LambdaGen::get_update_lambda(ops));

            _Traverser::traverse(leaf_ins, post_ins, stop_cond);
        }
    };
    
    struct IntervalDataTransferer{

        using interval_type         = types::interval_type;
        using _LambdaFil            = interval_ops_injection::FilterLambdaGenerator;
        using _LambdaGen            = interval_ops_injection::HeapOperatorLambdaGenerator;
        using _LambdaUlt            = utility::LambdaUtility;
        using _DispatcherSpawner    = dispatcher::DispatcherSpawner;

        template <class T, class T1>
        static inline void transfer(const internal_core::HeapOperatable<T>  transferee,
                                    const internal_core::HeapOperatable<T1> transferer){
            
            constexpr auto TRANSFERER_HEIGHT    = internal_core::HeapOperatable<T1>::HEIGHT;
            using _Traverser                    = interval_ops::IntervalTraverser<TRANSFERER_HEIGHT>; 

            auto dispatcher                     = _DispatcherSpawner::get_std_block_dispatcher(transferee);
            auto cb_lambda                      = _LambdaUlt::bind_filter_n_deflate(dispatcher, _LambdaFil::extract_interval()); 

            _Traverser::traverse(cb_lambda, _LambdaUlt::get_null_lambda(), _LambdaGen::get_is_blocked_lambda(transferer));
        }
    };
    
}

namespace dg::heap::market{
    
    class StdSaleAgent: public Buyable<StdSaleAgent>{

        private:

            using store_type        = types::store_type;
            using interval_type     = types::interval_type;
            using _IntervalUtility  = utility::IntervalEssential;

            interval_type product;
            bool is_decommissioned;

        public:

            StdSaleAgent(interval_type product) noexcept: product(product), 
                                                          is_decommissioned(false){} 

            std::optional<interval_type> buy(store_type sz) noexcept{ 
                
                assert(sz != 0u);

                if (!_IntervalUtility::is_valid_interval(this->product) || _IntervalUtility::span_size(this->product) < sz){ //REVIEW: very weird logic - need excl interval
                    return std::nullopt;
                }
                
                auto beg        = _IntervalUtility::get_interval_beg(this->product);
                auto rs         = _IntervalUtility::excl_relative_to_interval(_IntervalUtility::make(beg, sz));
                this->product   = _IntervalUtility::make(_IntervalUtility::get_interval_excl_end(rs), _IntervalUtility::get_interval_end(this->product)); //

                return rs;
            }

            template <class CollectorType>
            void decomission(CollectorType&& collector) noexcept(noexcept(collector(std::declval<interval_type>()))){
                
                if (!this->is_decommissioned){

                    if (_IntervalUtility::is_valid_interval(this->product)){
                        collector(this->product);
                    } else{
                        collector(std::nullopt);
                    }

                    this->is_decommissioned = true; 
                }
            }
    };

    class StdBuyAgent: public Sellable<StdBuyAgent>{ 
        
        private:

            using interval_type     = types::interval_type;

            std::vector<interval_type> _container; //
            bool is_decomissioned;

        public:

            StdBuyAgent(std::vector<interval_type> _container): _container(std::move(_container)), 
                                                                is_decomissioned(false){}

            bool sell(const interval_type& product) noexcept{ 
                
                //precond: valid interval_type

                if ((this->_container.size() == this->_container.capacity())){
                    return false;
                }
                
                this->_container.push_back(product);
                return true;

            }

            template <class CollectorType>
            void decomission(CollectorType&& collector) noexcept(noexcept(collector(_container.begin(), _container.end()))){
                
                if (!this->is_decomissioned){
                    
                    collector(this->_container.begin(), this->_container.end());
                    this->is_decomissioned = true;
                }
            }

    };

    class FragmentedSaleAgent: public Buyable<FragmentedSaleAgent>{

        private:

            using store_type        = types::store_type;
            using interval_type     = types::interval_type;
            
            StdSaleAgent sale_agent;
            StdBuyAgent buy_agent;
            std::vector<interval_type> products;
        
        public:

            FragmentedSaleAgent(StdSaleAgent sale_agent, 
                                StdBuyAgent buy_agent, 
                                std::vector<interval_type> products): sale_agent(std::move(sale_agent)),
                                                                      buy_agent(std::move(buy_agent)),
                                                                      products(std::move(products)){}

            std::optional<interval_type> buy(store_type sz) noexcept{
                
                assert(sz != 0);
                
                while (true){

                    if (auto rs = this->sale_agent.buy(sz); rs){
                        return rs;
                    }

                    if (!this->has_next()){
                        return std::nullopt;
                    }

                    this->decomission_sale_agent();
                    this->spawn_sale_agent();
                }
            }

            template <class CollectorType>
            void decomission(CollectorType&& collector) noexcept(noexcept(buy_agent(std::declval<CollectorType>()))){

                this->exhaust();
                this->buy_agent.decomission(std::forward<CollectorType>(collector));
            }

        private:

            bool has_next(){
            
                return !this->products.empty();
            }

            void spawn_sale_agent(){

                this->sale_agent = StdSaleAgent(this->products.back());
                this->products.pop_back();
            }

            void decomission_sale_agent(){
                
                auto transfer_lambda = [&](std::optional<interval_type> intv){
                    if (intv){
                        this->buy_agent.sell(intv.value());
                    }
                };

                this->sale_agent.decomission(transfer_lambda);
            }

            void exhaust(){
                
                decomission_sale_agent();

                while (has_next()){
                    spawn_sale_agent();
                    decomission_sale_agent();
                }
            }
    };

    struct AgencyCenter{

        using interval_type             = types::interval_type;

        using StdSaleAgentIntf          = std::shared_ptr<std::remove_pointer_t<decltype(std::declval<StdSaleAgent>().to_buyable())>>; 
        using StdBuyAgentIntf           = std::shared_ptr<std::remove_pointer_t<decltype(std::declval<StdBuyAgent>().to_sellable())>>;
        using FragmentedSaleAgentIntf   = std::shared_ptr<std::remove_pointer_t<decltype(std::declval<FragmentedSaleAgent>().to_buyable())>>;

        template <class _Collector>         
        static inline auto get_std_sale_agent(_Collector collector, interval_type valid_interval) -> StdSaleAgentIntf{

            auto resource_flag  = std::make_shared<bool>(false);
            auto cleanup_lambda = [=](StdSaleAgent * ins){
                if (*resource_flag){
                    ins->decomission(collector);
                }
                delete ins;
            };  

            using rs_type   = std::unique_ptr<StdSaleAgent, decltype(cleanup_lambda)>;
            auto rs         = StdSaleAgentIntf{rs_type{new StdSaleAgent(valid_interval), cleanup_lambda}}; 
            *resource_flag  = true;

            return rs;
        }

        template <class _Collector>
        static inline auto get_std_buy_agent(_Collector collector, size_t buying_limits) -> StdBuyAgentIntf{

            auto resource_flag  = std::make_shared<bool>(false);
            auto cleanup_lambda = [=](StdBuyAgent * ins){
                if (*resource_flag){
                    ins->decomission(collector);
                }
                delete ins;
            };

            using _Init     = utility::ReservedVectorInitializer<interval_type>;
            using rs_type   = std::unique_ptr<StdBuyAgent, decltype(cleanup_lambda)>;
            auto rs         = StdBuyAgentIntf{rs_type{new StdBuyAgent(_Init(buying_limits)), cleanup_lambda}};
            *resource_flag  = true;
            
            return rs;
        }

        template <class _Collector>
        static inline auto get_fragmented_sale_agent(_Collector collector, std::vector<interval_type> valid_intervals) -> FragmentedSaleAgentIntf{

            assert(valid_intervals.size() != 0);

            auto resource_flag  = std::make_shared<bool>(false);
            auto cleanup_lambda = [=](FragmentedSaleAgent * ins){
                if (*resource_flag){
                    ins->decomission(collector);
                }
                delete ins;
            };

            using _Init     = utility::ReservedVectorInitializer<interval_type>;
            using rs_type   = std::unique_ptr<FragmentedSaleAgent, decltype(cleanup_lambda)>;

            StdBuyAgent buy_agent(_Init(valid_intervals.size()));
            StdSaleAgent sale_agent(valid_intervals.back());
            valid_intervals.pop_back();
            auto rs         = FragmentedSaleAgentIntf{rs_type{new FragmentedSaleAgent(std::move(sale_agent), std::move(buy_agent), std::move(valid_intervals)), cleanup_lambda}};
            *resource_flag  = true;

            return rs; 
        }

    }; 

    struct IRS{

        using interval_type         = types::interval_type;
        using _DispatcherSpawner    = dispatcher::DispatcherSpawner;
        using _Assorter             = batch_interval_ops::BatchAssorter;
        
        template <class T>
        static constexpr auto get_contiguous_collector(const internal_core::HeapOperatable<T> ops){

            auto dispatcher         = _DispatcherSpawner::get_std_unblock_dispatcher(ops);  
            using dispatcher_type   = decltype(dispatcher); 

            auto collector_lambda   = [=](std::optional<interval_type> free_space) noexcept(noexcept(std::declval<dispatcher_type>()(std::declval<interval_type>()))){
                if (free_space){
                    dispatcher(free_space.value());
                }
            };  

            return collector_lambda;
        }

        template <class T>
        static constexpr auto get_fragmented_collector(const internal_core::HeapOperatable<T> ops){

            auto dispatcher         = _DispatcherSpawner::get_batch_unblock_dispatcher(ops);
            using dispatcher_type   = decltype(dispatcher);

            auto collector_lambda   = [=]<class _Iterator>(_Iterator first, _Iterator last) noexcept(noexcept(std::declval<dispatcher_type>()(std::declval<_Iterator>(), std::declval<_Iterator>()))){
                if (first != last){
                    essentials::piecewise_void_invoke(dispatcher, _Assorter::inplace_sort(first, last));
                }
            };

            return collector_lambda;
        }

    };

    struct BrokerCenter{

        using interval_type         = types::interval_type;
        using _DispatcherSpawner    = dispatcher::DispatcherSpawner;
        using _IRS                  = IRS;
        using _AgencyCenter         = AgencyCenter;
        
        static constexpr size_t DEFAULT_BUYING_LIMIT = size_t{1} << 20;

        template <class T>
        static inline auto get_sale_broker(const internal_core::HeapOperatable<T> ops, interval_type valid_interval){
            
            auto block_lambda   = [=, dispatcher = _DispatcherSpawner::get_std_block_dispatcher(ops)]{dispatcher(valid_interval);};
            auto unblock_lambda = [=, dispatcher = _DispatcherSpawner::get_std_unblock_dispatcher(ops)]{dispatcher(valid_interval);};
            auto collector      = _IRS::get_contiguous_collector(ops);

            block_lambda();
            utility::BackoutExecutor backout_plan(unblock_lambda);

            auto rs = _AgencyCenter::get_std_sale_agent(collector, valid_interval);
            backout_plan.release();

            return rs;
        }

        template <class T>
        static inline auto get_batch_sale_broker(const internal_core::HeapOperatable<T> ops, std::vector<interval_type> valid_intervals){
            
            assert(valid_intervals.size() != 0);

            auto[first, last]   = std::make_tuple(valid_intervals.begin(), valid_intervals.end()); 
            auto block_lambda   = [=, dispatcher = _DispatcherSpawner::get_batch_block_dispatcher(ops)]{dispatcher(first, last);};
            auto unblock_lambda = [=, dispatcher = _DispatcherSpawner::get_batch_unblock_dispatcher(ops)]{dispatcher(first, last);};
            auto collector      = _IRS::get_fragmented_collector(ops);

            block_lambda();
            utility::BackoutExecutor backout_plan(unblock_lambda);

            auto rs = _AgencyCenter::get_fragmented_sale_agent(collector, valid_intervals);
            backout_plan.release();

            return rs;            
        }

        template <class T>
        static inline auto get_buy_broker(const internal_core::HeapOperatable<T> ops, size_t buying_limits = DEFAULT_BUYING_LIMIT){

            return _AgencyCenter::get_std_buy_agent(IRS::get_fragmented_collector(ops), buying_limits);
        }
    };

    struct BrokerSpawner{

        using interval_type     = types::interval_type;
        using _BrokerCenter     = BrokerCenter;

        template <class T, class Generator, std::enable_if_t<std::is_same_v<decltype(std::declval<Generator>()()), std::vector<interval_type>>, bool>  = true>
        static inline auto get_sale_broker_spawner(const internal_core::HeapOperatable<T> heap_ops, Generator gen){
            
            using gen_type      = std::vector<interval_type>;
            using type          = decltype(_BrokerCenter::get_batch_sale_broker(heap_ops, std::declval<gen_type>()));
            using rs_type       = types_space::nillable_t<type>; 

            auto spawner        = [=]{

                gen_type intervals = gen();

                if (intervals.empty()){
                    return rs_type{};
                }

                return rs_type{_BrokerCenter::get_batch_sale_broker(heap_ops, std::move(intervals))};
            };

            return spawner;
        }

        template <class T, class Generator, std::enable_if_t<std::is_same_v<decltype(std::declval<Generator>()()), std::optional<interval_type>>, bool> = true>
        static inline auto get_sale_broker_spawner(const internal_core::HeapOperatable<T> heap_ops, Generator gen){
            
            using type      = decltype(_BrokerCenter::get_sale_broker(heap_ops, std::declval<interval_type>()));
            using rs_type   = types_space::nillable_t<type>;

            auto spawner    = [=]{

                std::optional<interval_type> interval = gen();

                if (!interval){
                    return rs_type{}; //
                }

                return rs_type{_BrokerCenter::get_sale_broker(heap_ops, interval.value())};
            };

            return spawner;
        }

        template <class T>
        static inline auto get_buy_broker_spawner(const internal_core::HeapOperatable<T> heap_ops, size_t buying_limits){ //pre_cond -- 
            
            auto spawner    = [=]{
                return _BrokerCenter::get_buy_broker(heap_ops, buying_limits);
            };

            return spawner;
        }
    };
}

namespace dg::heap::cache{
    
    class StdCacheController: public CacheControllable<StdCacheController>,
                              private jg::dense_hash_map<size_t, types::cache_type>{
        
        private:
            
            using cache_type = types::cache_type;
            
            cache_type NULL_ADDR;
            size_t capacity; 

        public:

            using _CacheObject      = jg::dense_hash_map<size_t, cache_type>;
            using _ValConstUtility  = utility::ValConstUtility;
            
            StdCacheController(size_t capacity): _CacheObject(), 
                                                 NULL_ADDR(_ValConstUtility::null<cache_type>()),
                                                 capacity(capacity){
                
                assert(capacity != 0);
                _CacheObject::reserve(capacity);
            }

            const cache_type& get(size_t key) const noexcept{

                if (auto iter = _CacheObject::find(key); iter != _CacheObject::end()){
                    return iter->second;
                }
                
                return NULL_ADDR; 
            }

            void set(size_t key, const cache_type& val) noexcept{
                
                if (_CacheObject::size() == this->capacity){
                    clear();
                }

                _CacheObject::insert_or_assign(key, val);
            }
    }; 
};

namespace dg::heap::top_impl{
    
    struct StdOperator{
        
        using container_type    =  std::add_pointer_t<types::Node>;

        static constexpr auto get_left_at(container_type arr, size_t idx) -> decltype(auto){

            return arr[idx].l; 
        }

        static constexpr auto get_right_at(container_type arr, size_t idx) -> decltype(auto){

            return arr[idx].r;
        }

        static constexpr auto get_center_at(container_type arr, size_t idx) -> decltype(auto){

            return arr[idx].c;
        }

        static constexpr auto get_offset_at(container_type arr, size_t idx) -> decltype(auto){

            return arr[idx].o;
        }

        template <class Val>
        static constexpr void set_left_at(container_type arr, size_t idx, Val&& val){

            arr[idx].l = std::forward<Val>(val);
        }

        template <class Val>
        static constexpr void set_right_at(container_type arr, size_t idx, Val&& val){

            arr[idx].r = std::forward<Val>(val);
        }

        template <class Val>
        static constexpr void set_center_at(container_type arr, size_t idx, Val&& val){

            arr[idx].c = std::forward<Val>(val);
        }

        template <class Val>
        static constexpr void set_offset_at(container_type arr, size_t idx, Val&& val){

            arr[idx].o = std::forward<Val>(val);
        } 
    };

    template <size_t HEIGHT>
    struct StdUpdater{
        
        using store_type        = types::store_type;
        using interval_type     = types::interval_type;
        using container_type    = std::add_pointer_t<types::Node>; 

        using _Indicator        = interval_ops::IntervalTracer<HEIGHT>; 
        using _HeapUtility      = utility::HeapUtility<HEIGHT>;
        using _IntervalUtility  = utility::IntervalUtility<HEIGHT>;
        using _Operator         = StdOperator;

        template <size_t ARG_HEIGHT, class T>
        static inline void update_left_at(container_type arr,
                                          const data::StorageExtractible<T> extractor, 
                                          size_t idx) noexcept{
            
            auto cb = [=]<class ValType>(ValType&& val){_Operator::set_left_at(arr, idx, std::forward<ValType>(val));};
            _Indicator::template trace_left_at<ARG_HEIGHT>(extractor, idx, cb, cb);
        }

        template <size_t ARG_HEIGHT, class T>
        static inline void update_right_at(container_type arr,
                                           const data::StorageExtractible<T> extractor, 
                                           size_t idx) noexcept{
            
            auto cb = [=]<class ValType>(ValType&& val){_Operator::set_right_at(arr, idx, std::forward<ValType>(val));};
            _Indicator::template trace_right_at<ARG_HEIGHT>(extractor, idx, cb, cb);
        }

        template <size_t ARG_HEIGHT, class T>
        static inline void update_center_at(container_type arr,
                                            const data::StorageExtractible<T> extractor, 
                                            size_t idx) noexcept{
            
            auto l_cb   = [=]<class ValType>(ValType&& val){

                _Operator::set_center_at(arr, idx, std::forward<ValType>(val));
                _Operator::set_offset_at(arr, idx, extractor.template get_offset_at<ARG_HEIGHT + 1>(_HeapUtility::left(idx)));

            };

            auto r_cb   = [=]<class ValType>(ValType&& val){

                _Operator::set_center_at(arr, idx, std::forward<ValType>(val));
                _Operator::set_offset_at(arr, idx, extractor.template get_offset_at<ARG_HEIGHT + 1>(_HeapUtility::right(idx)));

            };
            
            auto mid_cb = [=]<class ValType>(ValType&& val){
                
                auto EOLI   = _IntervalUtility::get_interval_excl_end(_IntervalUtility::template idx_to_interval<ARG_HEIGHT + 1>(_HeapUtility::left(idx)));
                auto offs   = EOLI - extractor.template get_right_at<ARG_HEIGHT + 1>(_HeapUtility::left(idx));

                _Operator::set_center_at(arr, idx, std::forward<ValType>(val));
                _Operator::set_offset_at(arr, idx, offs);

            };

            _Indicator::template trace_center_at<ARG_HEIGHT>(extractor, idx, l_cb, mid_cb, r_cb);
        }


        template <size_t ARG_HEIGHT, class T>
        static inline void update_at(container_type arr,
                                     const data::StorageExtractible<T> extractor, 
                                     size_t idx) noexcept{
            
            update_left_at<ARG_HEIGHT>(arr, extractor, idx);
            update_right_at<ARG_HEIGHT>(arr, extractor, idx);
            update_center_at<ARG_HEIGHT>(arr, extractor, idx);
        }
    };

    template <size_t HEIGHT>
    struct StdBlocker{  

        using Node              = types::Node;
        using container_type    = std::add_pointer_t<Node>; 

        using _IntervalUtility  = utility::IntervalUtility<HEIGHT>;
        using _NodeUtility      = utility::NodeUtility; 
        using _HeapUtility      = utility::HeapEssential;
        using _ConstUtility     = utility::ValConstUtility;

        static inline void block(container_type arr, size_t idx) noexcept{

            _NodeUtility::assign(arr[idx], _ConstUtility::empty<Node>());
        }

        template <size_t ARG_HEIGHT>
        static inline void unblock(container_type arr, size_t idx) noexcept{
            
            _NodeUtility::assign(arr[idx], _ConstUtility::deflt<Node>(_IntervalUtility::template idx_to_interval<ARG_HEIGHT>(idx)));
        }
    };

}

namespace dg::heap::bottom_impl{

    using namespace datastructure::boolvector;

    template <size_t HEIGHT>
    struct TraceBackOperator{

        using traceback_type    = traceback_policy::traceback_type; 
        using _OffsetConverter  = utility::OffsetConverter<HEIGHT>;

        template <size_t BIT_SPACE, class T, traceback_type Val>
        static inline void set(OperatableVector<T>& op, size_t offset, const std::integral_constant<traceback_type, Val>& val) noexcept{

            //should boolvector be accessed as an independent component or included as part of dependencies.

            using namespace dg::datastructure::boolvector::utility;
            using namespace dg::datastructure::boolvector::operation; 

            sequential_set(op, offset, boolify(val, std::integral_constant<size_t, BIT_SPACE>{}));
        }

        template <size_t BIT_SPACE, class T>
        static inline auto get(ReadableVector<T>& op, size_t offset) noexcept -> traceback_type {

            using namespace dg::datastructure::boolvector::utility;
            using namespace dg::datastructure::boolvector::operation; 

            return intify<traceback_type>(sequential_get<BIT_SPACE>(op, offset));
        }

        template <size_t ARG_HEIGHT, class T, traceback_type Val>
        static inline void set_left_trace(OperatableVector<T>& op, size_t idx, const std::integral_constant<traceback_type, Val>& TRACE_ID) noexcept{

            set<traceback_policy::L_BIT_SPACE>(op, _OffsetConverter::template get_left_offset<ARG_HEIGHT>(idx), TRACE_ID);            
        } 

        template <size_t ARG_HEIGHT, class T, traceback_type Val>
        static inline void set_right_trace(OperatableVector<T>& op, size_t idx, const std::integral_constant<traceback_type, Val>& TRACE_ID) noexcept{

            set<traceback_policy::R_BIT_SPACE>(op, _OffsetConverter::template get_right_offset<ARG_HEIGHT>(idx), TRACE_ID);
        }

        template <size_t ARG_HEIGHT, class T, traceback_type Val>
        static inline void set_center_trace(OperatableVector<T>& op, size_t idx, const std::integral_constant<traceback_type, Val>& TRACE_ID) noexcept{ 

            set<traceback_policy::C_BIT_SPACE>(op, _OffsetConverter::template get_center_offset<ARG_HEIGHT>(idx), TRACE_ID);
        }

        template <class T, traceback_type Val>
        static inline void set_next_base_block_bit(OperatableVector<T>& op, size_t idx, const std::integral_constant<traceback_type, Val>& TRACE_ID) noexcept{

            set<traceback_policy::BL_BIT_SPACE>(op, _OffsetConverter::get_next_base_blocked_bit_offset(idx), TRACE_ID);
        }

        template <class T, traceback_type Val>
        static inline void set_base_block_bit(OperatableVector<T>& op, size_t idx, const std::integral_constant<traceback_type, Val>& TRACE_ID) noexcept{

            set<traceback_policy::BL_BIT_SPACE>(op, _OffsetConverter::get_base_blocked_bit_offset(idx), TRACE_ID);
        }

        template <size_t ARG_HEIGHT, class T>
        static inline auto get_left_trace(ReadableVector<T>& op, size_t idx) noexcept -> traceback_type{

            return get<traceback_policy::L_BIT_SPACE>(op, _OffsetConverter::template get_left_offset<ARG_HEIGHT>(idx));
        }

        template <size_t ARG_HEIGHT, class T>
        static inline auto get_right_trace(ReadableVector<T>& op, size_t idx) noexcept -> traceback_type{
            
            return get<traceback_policy::R_BIT_SPACE>(op, _OffsetConverter::template get_right_offset<ARG_HEIGHT>(idx));
        }

        template <size_t ARG_HEIGHT, class T>
        static inline auto get_center_trace(ReadableVector<T>& op, size_t idx) noexcept -> traceback_type{
            
            return get<traceback_policy::C_BIT_SPACE>(op, _OffsetConverter::template get_center_offset<ARG_HEIGHT>(idx));
        }

        template <class T>
        static inline auto get_next_base_block_bit(ReadableVector<T>& op, size_t idx) noexcept -> traceback_type{

            return get<traceback_policy::BL_BIT_SPACE>(op, _OffsetConverter::get_next_base_blocked_bit_offset(idx));
        }

        template <class T>
        static inline auto get_base_block_bit(ReadableVector<T>& op, size_t idx) noexcept -> traceback_type{

            return get<traceback_policy::BL_BIT_SPACE>(op, _OffsetConverter::get_base_blocked_bit_offset(idx));
        }
    };

    template <size_t HEIGHT>
    struct TraceBackUpdater{

        using _Indicator    = interval_ops::IntervalTracer<HEIGHT>;
        using _Operator     = TraceBackOperator<HEIGHT>;
        using _HeapUtility  = utility::HeapUtility<HEIGHT>;

        static constexpr size_t LEFT_CUTOFF    = HEIGHT - 1;
        static constexpr size_t RIGHT_CUTOFF   = HEIGHT - 1;
        static constexpr size_t CENTER_CUTOFF  = HEIGHT - 2;

        template <size_t ARG_HEIGHT, class T, class T1>
        static inline void update_left_at(OperatableVector<T>& traceback_store, 
                                          const data::StorageExtractible<T1> node_extractor,
                                          size_t idx) noexcept{
            
            
            ///REVIEW: remove [&]

            if constexpr(ARG_HEIGHT <= LEFT_CUTOFF){
                
                auto left_callback      = [&](...){_Operator::template set_left_trace<ARG_HEIGHT>(traceback_store, idx, traceback_policy::LEFT_TRACEBACK_IC);};
                auto right_callback     = [&](...){_Operator::template set_left_trace<ARG_HEIGHT>(traceback_store, idx, traceback_policy::RIGHT_TRACEBACK_IC);};
               
                _Indicator::template trace_left_at<ARG_HEIGHT>(node_extractor, idx, left_callback, right_callback); 
            }
        }

        template <size_t ARG_HEIGHT, class T, class T1>
        static inline void update_right_at(OperatableVector<T>& traceback_store,
                                           const data::StorageExtractible<T1> node_extractor,
                                           size_t idx) noexcept{
            
            if constexpr(ARG_HEIGHT <= RIGHT_CUTOFF){

                auto left_callback      = [&](...){_Operator::template set_right_trace<ARG_HEIGHT>(traceback_store, idx, traceback_policy::LEFT_TRACEBACK_IC);};
                auto right_callback     = [&](...){_Operator::template set_right_trace<ARG_HEIGHT>(traceback_store, idx, traceback_policy::RIGHT_TRACEBACK_IC);};
                
                _Indicator::template trace_right_at<ARG_HEIGHT>(node_extractor, idx, left_callback, right_callback);
            }
        }

        template <size_t ARG_HEIGHT, class T, class T1>
        static inline void update_center_at(OperatableVector<T>& traceback_store,
                                            const data::StorageExtractible<T1> node_extractor,
                                            size_t idx) noexcept{
            
            if constexpr(ARG_HEIGHT <= CENTER_CUTOFF){

                auto left_callback      = [&](...){_Operator::template set_center_trace<ARG_HEIGHT>(traceback_store, idx, traceback_policy::LEFT_TRACEBACK_IC);};
                auto center_callback    = [&](...){_Operator::template set_center_trace<ARG_HEIGHT>(traceback_store, idx, traceback_policy::MID_TRACEBACK_IC);};
                auto right_callback     = [&](...){_Operator::template set_center_trace<ARG_HEIGHT>(traceback_store, idx, traceback_policy::RIGHT_TRACEBACK_IC);};

                _Indicator::template trace_center_at<ARG_HEIGHT>(node_extractor, idx, left_callback, center_callback, right_callback);
            }
        }

        template <size_t ARG_HEIGHT, class T, class T1>
        static inline void update_at(OperatableVector<T>& traceback_store,
                                     const data::StorageExtractible<T1>& node_extractor,
                                     size_t idx) noexcept{
            
            update_left_at<ARG_HEIGHT>(traceback_store, node_extractor, idx);
            update_right_at<ARG_HEIGHT>(traceback_store, node_extractor, idx);
            update_center_at<ARG_HEIGHT>(traceback_store, node_extractor, idx);
        }
    };
    
    template <size_t HEIGHT>
    struct TraceBackBlocker{

        using _Utility      = utility::HeapUtility<HEIGHT>;
        using _Operator     = TraceBackOperator<HEIGHT>; 

        template <size_t ARG_HEIGHT, class T>
        static inline void block(OperatableVector<T>& traceback_store, size_t idx) noexcept{

            if constexpr(_Utility::is_not_base(ARG_HEIGHT)){
                _Operator::template set_center_trace<ARG_HEIGHT>(traceback_store, idx, traceback_policy::MID_BLOCKED_IC);
            } else if constexpr(_Utility::is_next_base(ARG_HEIGHT)){
                _Operator::set_next_base_block_bit(traceback_store, idx, traceback_policy::BLOCKED_IC);
            } else if constexpr(_Utility::is_base(ARG_HEIGHT)){
                _Operator::set_base_block_bit(traceback_store, idx, traceback_policy::BLOCKED_IC);
            } else{
                static_assert(utility::FALSE_VAL<>, "unreachable");
            }
        }

        template <size_t ARG_HEIGHT, class T>
        static inline void unblock(OperatableVector<T>& traceback_store, size_t idx) noexcept{

            if constexpr(_Utility::is_not_base(ARG_HEIGHT)){
                _Operator::template set_center_trace<ARG_HEIGHT>(traceback_store, idx, traceback_policy::MID_TRACEBACK_IC);
            } else if constexpr(_Utility::is_next_base(ARG_HEIGHT)){
                _Operator::set_next_base_block_bit(traceback_store, idx, traceback_policy::UNBLOCKED_IC);
            } else if constexpr(_Utility::is_base(ARG_HEIGHT)){
                _Operator::set_base_block_bit(traceback_store, idx, traceback_policy::UNBLOCKED_IC);
            } else{
                static_assert(utility::FALSE_VAL<>, "unreachable");
            }
        }

        template <size_t ARG_HEIGHT, class T>
        static inline bool is_blocked(ReadableVector<T>& traceback_store, size_t idx) noexcept{

            if constexpr(_Utility::is_not_base(ARG_HEIGHT)){
                return _Operator::template get_center_trace<ARG_HEIGHT>(traceback_store, idx) == traceback_policy::MID_BLOCKED_IC();
            } else if constexpr(_Utility::is_next_base(ARG_HEIGHT)){
                return _Operator::get_next_base_block_bit(traceback_store, idx) == traceback_policy::BLOCKED_IC();
            } else if constexpr(_Utility::is_base(ARG_HEIGHT)){
                return _Operator::get_base_block_bit(traceback_store, idx) == traceback_policy::BLOCKED_IC(); 
            } else{
                static_assert(utility::FALSE_VAL<>, "unreachable");
                return {};
            }   
        }
    };

    template <size_t HEIGHT>
    struct TraceBackCacheOperator{

        using store_type        = types::store_type;
        using cache_type        = types::cache_type; 
        using _TraceOperator    = TraceBackOperator<HEIGHT>;
        using _TraceBlocker     = TraceBackBlocker<HEIGHT>; 
        using _HeapUtility      = utility::HeapUtility<HEIGHT>;
        using _IntervalUtility  = utility::IntervalUtility<HEIGHT>;
        using _ConstValUtility  = utility::ValConstUtility;

        template <class T>
        static inline void fetch(cache::CacheControllable<T>& cache, size_t key, const cache_type& val) noexcept{

            cache.set(key, val);
        }
        
        template <class T>
        static inline void invalidate(cache::CacheControllable<T>& cache, size_t key) noexcept{

            fetch(cache, key, _ConstValUtility::null<cache_type>());
        }

        template <class T, size_t ARG_HEIGHT>
        static inline void defaultize(cache::CacheControllable<T>& cache, size_t key, const std::integral_constant<size_t, ARG_HEIGHT>&) noexcept{

            fetch(cache, key, _ConstValUtility::deflt<cache_type>(_IntervalUtility::template idx_to_interval<ARG_HEIGHT>(key)));
        } 

        template <class T>
        static inline void empty_init(cache::CacheControllable<T>& cache, size_t key) noexcept{

            fetch(cache, key, _ConstValUtility::empty<cache_type>());
        } 

        template <size_t ARG_HEIGHT, class T, class T1>
        static inline store_type get_left_at(ReadableVector<T>& traceback_store,
                                             cache::CacheControllable<T1>& cache,
                                             size_t idx) noexcept{
            
            if constexpr(_HeapUtility::height_is_in_range(ARG_HEIGHT)){
                
                cache_type cache_data   = cache.get(idx);
                store_type rs           = cache_data.l;

                if (rs != _ConstValUtility::null<store_type>()){
                    return rs;
                }

                bool cond = _TraceBlocker::template is_blocked<ARG_HEIGHT>(traceback_store, idx); //REVIEW: optimization opportunity 

                if (cond){
                    rs = _ConstValUtility::empty<store_type>();
                    empty_init(cache, idx);
                } else{
                    if constexpr(_HeapUtility::is_base(ARG_HEIGHT)){
                        rs = _ConstValUtility::leaf<store_type>();
                    } else{
                        const auto SPAN_SIZE    = _IntervalUtility::span_size_from_height(ARG_HEIGHT + 1);
                        auto print              = _TraceOperator::template get_left_trace<ARG_HEIGHT>(traceback_store, idx); 

                        switch (print){
                            case traceback_policy::LEFT_TRACEBACK:
                                rs = get_left_at<ARG_HEIGHT + 1>(traceback_store, cache, _HeapUtility::left(idx));
                                break;
                            case traceback_policy::RIGHT_TRACEBACK:
                                rs = SPAN_SIZE + get_left_at<ARG_HEIGHT + 1>(traceback_store, cache, _HeapUtility::right(idx));
                                break;
                            default:
                                std::abort();
                                break;
                        }
                    }
     
                    cache_data.l = rs;
                    fetch(cache, idx, cache_data);

                }

                return rs;
            } else{
                static_assert(utility::FALSE_VAL<>);
                return {};
            }
        } 

        template <size_t ARG_HEIGHT, class T, class T1>
        static inline store_type get_right_at(ReadableVector<T>& traceback_store,
                                              cache::CacheControllable<T1>& cache,
                                              size_t idx) noexcept{
            
            if constexpr(_HeapUtility::height_is_in_range(ARG_HEIGHT)){

                if constexpr(_HeapUtility::is_base(ARG_HEIGHT)){

                    return get_left_at<ARG_HEIGHT>(traceback_store, cache, idx);

                } else{
                    
                    cache_type cache_data   = cache.get(idx);
                    store_type rs           = cache_data.r; 

                    if (rs != _ConstValUtility::null<store_type>()){
                        
                        return rs;

                    }

                    bool cond = _TraceBlocker::template is_blocked<ARG_HEIGHT>(traceback_store, idx); //REVIEW: optimization opportunity

                    if (cond){

                        rs = _ConstValUtility::empty<store_type>();
                        empty_init(cache, idx);

                    } else{
                        
                        const auto SPAN_SIZE    = _IntervalUtility::span_size_from_height(ARG_HEIGHT + 1);
                        auto print              = _TraceOperator::template get_right_trace<ARG_HEIGHT>(traceback_store, idx);

                        switch (print){
                            case traceback_policy::LEFT_TRACEBACK:
                                rs = get_right_at<ARG_HEIGHT + 1>(traceback_store, cache, _HeapUtility::left(idx)) + SPAN_SIZE;
                                break;
                            case traceback_policy::RIGHT_TRACEBACK:
                                rs = get_right_at<ARG_HEIGHT + 1>(traceback_store, cache, _HeapUtility::right(idx));
                                break;
                            default:
                                std::abort();
                                break;
                        } 
                        cache_data.r = rs;
                        fetch(cache, idx, cache_data);
                    }

                    return rs;
                }
            } else{
                static_assert(utility::FALSE_VAL<>);
                return {};
            }
        }

        template <size_t ARG_HEIGHT, class T, class T1>
        static inline store_type get_center_at(ReadableVector<T>& traceback_store,
                                               cache::CacheControllable<T1>& cache,
                                               size_t idx) noexcept{
            
            if constexpr(_HeapUtility::height_is_in_range(ARG_HEIGHT)){

                if constexpr(_HeapUtility::is_base(ARG_HEIGHT)){

                    return get_left_at<ARG_HEIGHT>(traceback_store, cache, idx); 

                } else if constexpr(_HeapUtility::is_next_base(ARG_HEIGHT)){

                    auto _left          = get_left_at<ARG_HEIGHT>(traceback_store, cache, idx);
                    auto cond           = bool{_left != _ConstValUtility::empty<decltype(_left)>()};

                    return cond ? _left : get_right_at<ARG_HEIGHT>(traceback_store, cache, idx);

                } else{

                    cache_type cache_data   = cache.get(idx);
                    store_type rs           = cache_data.c; 

                    if (rs != _ConstValUtility::null<store_type>()){
                        return rs;
                    }

                    auto print = _TraceOperator::template get_center_trace<ARG_HEIGHT>(traceback_store, idx);

                    switch (print){
                        case traceback_policy::LEFT_TRACEBACK:
                            rs = get_center_at<ARG_HEIGHT + 1>(traceback_store, cache, _HeapUtility::left(idx));
                            cache_data.c = rs;
                            fetch(cache, idx, cache_data);
                            break;
                        case traceback_policy::RIGHT_TRACEBACK:
                            rs = get_center_at<ARG_HEIGHT + 1>(traceback_store, cache, _HeapUtility::right(idx));
                            cache_data.c = rs;
                            fetch(cache, idx, cache_data);
                            break; 
                        case traceback_policy::MID_TRACEBACK:
                            rs = get_right_at<ARG_HEIGHT + 1>(traceback_store, cache, _HeapUtility::left(idx)) + get_left_at<ARG_HEIGHT + 1>(traceback_store, cache, _HeapUtility::right(idx));
                            cache_data.c = rs;
                            fetch(cache, idx, cache_data);
                            break;
                        case traceback_policy::MID_BLOCKED:
                            rs = _ConstValUtility::empty<store_type>();
                            empty_init(cache, idx);
                            break;
                        default:
                            std::abort();
                            break;
                    }

                    return rs;
                }
            } else{
                static_assert(utility::FALSE_VAL<>);
                return {};
            }
        }

        template <size_t ARG_HEIGHT, class T, class T1>
        static inline store_type get_offset_at(ReadableVector<T>& traceback_store,
                                               cache::CacheControllable<T1>& cache,
                                               size_t idx) noexcept{
            
            //assume for every center != 0, offset state is valid 
            
            if constexpr(_HeapUtility::height_is_in_range(ARG_HEIGHT)){

                if constexpr(_HeapUtility::is_base(ARG_HEIGHT)){

                    return _HeapUtility::idx_to_offset(idx); 

                } else if constexpr(_HeapUtility::is_next_base(ARG_HEIGHT)){
                    
                    const auto SPAN = _IntervalUtility::span_size_from_height(ARG_HEIGHT); 
                    auto EOLI       = _IntervalUtility::get_interval_excl_end(_IntervalUtility::template idx_to_interval<ARG_HEIGHT>(idx));
                    auto _right     = get_right_at<ARG_HEIGHT>(traceback_store, cache, idx);
                    auto cond       = bool{_right != _ConstValUtility::empty<decltype(_right)>()};

                    return (cond) ? (EOLI - _right) : (EOLI - SPAN);

                } else{
                    
                    cache_type cache_data   = cache.get(idx);
                    store_type rs           = cache_data.o;
                    store_type EOLI{};

                    if (rs != _ConstValUtility::null<store_type>()){
                        return rs;
                    }

                    auto print = _TraceOperator::template get_center_trace<ARG_HEIGHT>(traceback_store, idx);

                    switch(print){
                        case traceback_policy::LEFT_TRACEBACK:
                            rs = get_offset_at<ARG_HEIGHT + 1>(traceback_store, cache, _HeapUtility::left(idx));
                            cache_data.o = rs;
                            fetch(cache, idx, cache_data);
                            break;
                        case traceback_policy::RIGHT_TRACEBACK:
                            rs = get_offset_at<ARG_HEIGHT + 1>(traceback_store, cache, _HeapUtility::right(idx));
                            cache_data.o = rs;
                            fetch(cache, idx, cache_data);
                            break;
                        case traceback_policy::MID_TRACEBACK:
                            EOLI            = _IntervalUtility::get_interval_excl_end(_IntervalUtility::template idx_to_interval<ARG_HEIGHT + 1>(_HeapUtility::left(idx))); //REVIEW: deduced statement
                            rs              = EOLI - get_right_at<ARG_HEIGHT + 1>(traceback_store, cache, _HeapUtility::left(idx)); //REVIEW: potential overflow
                            cache_data.o    = rs;
                            fetch(cache, idx, cache_data);
                            break;
                        case traceback_policy::MID_BLOCKED:
                            rs = _ConstValUtility::empty<store_type>();
                            empty_init(cache, idx);
                            break;
                        default:
                            std::abort();
                            break;
                    }

                    return rs;
                }
            } else{
                static_assert(utility::FALSE_VAL<>);
                return {};
            }
        }
    };
}

namespace dg::heap::data{

    template <size_t DYNAMIC_HEIGHT_V, size_t TRACEBACK_HEIGHT_V, class ID>
    struct StdHeapDataView: HeapData<StdHeapDataView<DYNAMIC_HEIGHT_V, TRACEBACK_HEIGHT_V, ID>>{

        static constexpr size_t DYNAMIC_HEIGHT           = DYNAMIC_HEIGHT_V;
        static constexpr size_t TRACEBACK_HEIGHT         = TRACEBACK_HEIGHT_V;
        static constexpr size_t HEIGHT                   = TRACEBACK_HEIGHT_V;

        static inline types::Node * node_array{};
        static inline datastructure::boolvector::StdVectorOperator boolvector_container{}; 
        static inline cache::StdCacheController cache_controller{size_t{1}};

        StdHeapDataView(std::integral_constant<size_t, DYNAMIC_HEIGHT_V>,
                        std::integral_constant<size_t, TRACEBACK_HEIGHT_V>,
                        types::Node * node_array_arg,
                        datastructure::boolvector::StdVectorOperator boolvector_container_arg,
                        cache::StdCacheController cache_controller_arg,
                        const ID){
            
            node_array              = node_array_arg;
            boolvector_container    = boolvector_container_arg;
            cache_controller        = std::move(cache_controller_arg);
        }                       

        static auto& get_node_container() noexcept{

            return node_array;
        }

        static auto& get_boolvector_container() noexcept{

            return *boolvector_container.to_operatable_vector();
        }

        static auto& get_cache_instance() noexcept{

            return *cache_controller.to_cache_controllable();
        }
    };

    template <class T>
    struct StdStorageExtractor: StorageExtractible<StdStorageExtractor<T>>{

        using _HeapData             = data::HeapData<T>;

        static inline const size_t DYNAMIC_HEIGHT   = _HeapData::DYNAMIC_HEIGHT;
        static inline const size_t TRACEBACK_HEIGHT = _HeapData::TRACEBACK_HEIGHT;
        static inline const size_t TREE_HEIGHT      = _HeapData::HEIGHT;

        using store_type            = types::store_type;
        using _StdOperator          = top_impl::StdOperator;
        using _TracebackOperator    = bottom_impl::TraceBackCacheOperator<TRACEBACK_HEIGHT>;
        using _HeapUtility          = utility::HeapUtility<TRACEBACK_HEIGHT>;

        StdStorageExtractor(const data::HeapData<T>){}

        template <size_t HEIGHT>
        static store_type get_left_at(size_t idx) noexcept{

            if constexpr(HEIGHT <= DYNAMIC_HEIGHT){
                return _StdOperator::get_left_at(_HeapData::get_node_container(), idx);
            } else if constexpr(HEIGHT <= TRACEBACK_HEIGHT){
                return _TracebackOperator::template get_left_at<HEIGHT>(_HeapData::get_boolvector_container(), _HeapData::get_cache_instance(), idx);
            } else{
                static_assert(utility::FALSE_VAL<>, "unreachable");
                return {};
            }
        }

        template <size_t HEIGHT>
        static store_type get_right_at(size_t idx) noexcept{

            if constexpr(HEIGHT <= DYNAMIC_HEIGHT){
                return _StdOperator::get_right_at(_HeapData::get_node_container(), idx);
            } else if constexpr(HEIGHT <= TRACEBACK_HEIGHT){
                return _TracebackOperator::template get_right_at<HEIGHT>(_HeapData::get_boolvector_container(), _HeapData::get_cache_instance(), idx);
            } else {
                static_assert(utility::FALSE_VAL<>, "unreachable");
                return {};
            }
        } 

        template <size_t HEIGHT>
        static store_type get_center_at(size_t idx) noexcept{

            if constexpr(HEIGHT <= DYNAMIC_HEIGHT){
                return _StdOperator::get_center_at(_HeapData::get_node_container(), idx);
            } else if constexpr(HEIGHT <= TRACEBACK_HEIGHT){
                return _TracebackOperator::template get_center_at<HEIGHT>(_HeapData::get_boolvector_container(), _HeapData::get_cache_instance(), idx);
            } else{
                static_assert(utility::FALSE_VAL<>, "unreachable");
                return {};
            }
        }

        template <size_t HEIGHT>
        static store_type get_offset_at(size_t idx) noexcept{

            if constexpr(HEIGHT <= DYNAMIC_HEIGHT){
                return _StdOperator::get_offset_at(_HeapData::get_node_container(), idx);
            } else if constexpr(HEIGHT <= TRACEBACK_HEIGHT){
                return _TracebackOperator::template get_offset_at<HEIGHT>(_HeapData::get_boolvector_container(), _HeapData::get_cache_instance(), idx);
            } else{
                static_assert(utility::FALSE_VAL<>, "unreachable");
                return {};
            }
        }
    };
}

namespace dg::heap::internal_core{

    template <class T, class T1>
    class StdHeapOperator: public HeapOperatable<StdHeapOperator<T, T1>>{
        
        public:

            using _HeapData         = data::HeapData<T>;

            static constexpr size_t DYNAMIC_HEIGHT   = _HeapData::DYNAMIC_HEIGHT;
            static constexpr size_t TRACEBACK_HEIGHT = _HeapData::TRACEBACK_HEIGHT;
            static constexpr size_t HEIGHT           = _HeapData::HEIGHT;

            using _HeapUtility      = utility::HeapUtility<HEIGHT>;
            using _TopUpdater       = top_impl::StdUpdater<HEIGHT>;
            using _TopBlocker       = top_impl::StdBlocker<HEIGHT>;
            using _BottomUpdater    = bottom_impl::TraceBackUpdater<HEIGHT>;
            using _BottomBlocker    = bottom_impl::TraceBackBlocker<HEIGHT>; 
            using _CacheOperator    = bottom_impl::TraceBackCacheOperator<HEIGHT>; 
            using _StorageExtractor = data::StorageExtractible<T1>;

            StdHeapOperator(const data::HeapData<T>,
                            const data::StorageExtractible<T1>){}

            template <size_t ARG_HEIGHT>
            static void update(size_t idx) noexcept{

                if constexpr(ARG_HEIGHT <= DYNAMIC_HEIGHT){
                    _TopUpdater::template update_at<ARG_HEIGHT>(_HeapData::get_node_container(), _StorageExtractor(), idx);
                } else if constexpr(ARG_HEIGHT <= TRACEBACK_HEIGHT){
                    _CacheOperator::invalidate(_HeapData::get_cache_instance(), idx);
                    _BottomUpdater::template update_at<ARG_HEIGHT>(_HeapData::get_boolvector_container(), _StorageExtractor(), idx);
                } else{
                    static_assert(utility::FALSE_VAL<>, "unreachable");
                }
            }

            template <size_t ARG_HEIGHT>
            static void block(size_t idx) noexcept{

                if constexpr(ARG_HEIGHT <= DYNAMIC_HEIGHT){
                    _BottomBlocker::template block<ARG_HEIGHT>(_HeapData::get_boolvector_container(), idx);
                    _TopBlocker::block(_HeapData::get_node_container(), idx);
                } else if constexpr(ARG_HEIGHT <= TRACEBACK_HEIGHT){
                    _BottomBlocker::template block<ARG_HEIGHT>(_HeapData::get_boolvector_container(), idx);
                    _CacheOperator::empty_init(_HeapData::get_cache_instance(), idx);
                } else{
                    std::abort(); //temporary solution
                    // static_assert(utility::FALSE_VAL<>, "unreachable");
                }   
            }

            template <size_t ARG_HEIGHT>
            static void unblock(size_t idx) noexcept{

                if constexpr(ARG_HEIGHT <= DYNAMIC_HEIGHT){
                    _BottomBlocker::template unblock<ARG_HEIGHT>(_HeapData::get_boolvector_container(), idx);
                    _TopBlocker::template unblock<ARG_HEIGHT>(_HeapData::get_node_container(), idx);
                } else if constexpr(ARG_HEIGHT <= TRACEBACK_HEIGHT){
                    _BottomBlocker::template unblock<ARG_HEIGHT>(_HeapData::get_boolvector_container(), idx);
                    _CacheOperator::defaultize(_HeapData::get_cache_instance(), idx, std::integral_constant<size_t, ARG_HEIGHT>{});
                } else{
                    static_assert(utility::FALSE_VAL<>, "unreachable");
                }
            }

            template <size_t ARG_HEIGHT>
            static bool is_blocked(size_t idx) noexcept{
                
                if constexpr(ARG_HEIGHT <= TRACEBACK_HEIGHT){
                    return _BottomBlocker::template is_blocked<ARG_HEIGHT>(_HeapData::get_boolvector_container(), idx); 
                } else{
                    static_assert(utility::FALSE_VAL<>, "unreachable");
                    return {};
                }
            }
    };

    template <class T, class T1, class T2>
    class DirectAllocator: public NoExceptAllocatable<DirectAllocator<T, T1, T2>>{

        private:

            std::shared_ptr<seeker::Seekable<T>> max_seeker;
            std::shared_ptr<dispatcher::Dispatchable<T1>> block_dispatcher;
            std::shared_ptr<dispatcher::Dispatchable<T2>> unblock_dispatcher;
        
        public:

            using store_type        = types::store_type;
            using interval_type     = types::interval_type;
            using _IntervalUlt      = utility::IntervalEssential;
             
            DirectAllocator(std::shared_ptr<seeker::Seekable<T>> max_seeker, 
                            std::shared_ptr<dispatcher::Dispatchable<T1>> block_dispatcher,
                            std::shared_ptr<dispatcher::Dispatchable<T2>> unblock_dispatcher):  max_seeker(max_seeker), 
                                                                                                block_dispatcher(block_dispatcher),
                                                                                                unblock_dispatcher(unblock_dispatcher){}
            
            
            std::optional<interval_type> alloc(store_type sz) noexcept{
                
                const auto MMIN     = store_type{1};
                const auto ROOT     = std::integral_constant<size_t, 0u>{};
                auto block          = this->max_seeker->seek(ROOT);
                auto failed         = bool{!block} || bool{std::clamp(sz, MMIN, _IntervalUlt::span_size(block.value())) != sz};

                if (failed){
                    return std::nullopt;
                }
                
                auto rs             = _IntervalUlt::excl_relative_to_interval(_IntervalUlt::make(_IntervalUlt::get_interval_beg(block.value()), sz));
                this->block_dispatcher->dispatch(rs);
                
                return _IntervalUlt::interval_to_relative(rs);
            }

            void free(const interval_type& relative) noexcept{
                
                this->unblock_dispatcher->dispatch(_IntervalUlt::relative_to_interval(relative));
            }
    };

    template <class T, class T1, class BuyableSpawner, class SellableSpawner>
    class FastAllocator: public ExceptAllocatable<FastAllocator<T, T1, BuyableSpawner, SellableSpawner>>{ //

        private:

            BuyableSpawner buyable_spawner;
            SellableSpawner sellable_spawner;

            std::shared_ptr<market::Buyable<T>>  buyable_ins; //owning semantics 
            std::shared_ptr<market::Sellable<T1>> sellable_ins; 

        public:

            using store_type    = types::store_type;
            using interval_type = types::interval_type; 
            using _IntervalUlt  = utility::IntervalEssential;

            FastAllocator(BuyableSpawner buyable_spawner, 
                          SellableSpawner sellable_spawner,
                          std::shared_ptr<market::Buyable<T>> buyable_ins,
                          std::shared_ptr<market::Sellable<T1>> sellable_ins): buyable_spawner(buyable_spawner),
                                                                               sellable_spawner(sellable_spawner),
                                                                               buyable_ins(buyable_ins),
                                                                               sellable_ins(sellable_ins){}

            std::optional<interval_type> alloc(store_type sz){

                if (sz == 0u){
                    return std::nullopt;
                }

                if (auto rs = this->raw_alloc(sz); rs){
                    return rs;
                }
                
                this->sync_buyable();
                this->spawn_buyable(); 

                return this->raw_alloc(sz);
            }
            
            void free(const interval_type& intv){
                
                if (this->raw_free(intv)){
                    return;
                }

                this->sync_sellable();
                this->spawn_sellable();

                if (!this->raw_free(intv)){
                    std::abort();
                }
            }

        private:

            std::optional<interval_type> raw_alloc(store_type sz){

                if (this->is_valid_buyable()){
                    if (auto rs = this->buyable_ins->buy(sz); rs){
                        return _IntervalUlt::interval_to_relative(rs.value());
                    }
                }

                return std::nullopt;

            }

            bool raw_free(const interval_type& relative){

                return this->is_valid_sellable() && this->sellable_ins->sell(_IntervalUlt::relative_to_interval(relative));
            }

            bool is_valid_buyable(){

                return bool{this->buyable_ins};
            }

            bool is_valid_sellable(){

                return bool{this->sellable_ins};
            }

            void sync_buyable(){

                this->buyable_ins.reset();
            }

            void sync_sellable(){

                this->sellable_ins.reset();
            }

            void spawn_buyable(){

                this->buyable_ins = this->buyable_spawner();
            }

            void spawn_sellable(){

                this->sellable_ins = this->sellable_spawner();
            }
    };

    template <class T, class T1>
    class StdAllocator: public NoExceptAllocatable<StdAllocator<T, T1>>{

        private:

            std::shared_ptr<NoExceptAllocatable<T>> direct_allocator;
            std::shared_ptr<ExceptAllocatable<T1>> fast_allocator;

        public:

            using interval_type = types::interval_type;
            using store_type    = types::store_type;

            StdAllocator(std::shared_ptr<NoExceptAllocatable<T>> direct_allocator,
                         std::shared_ptr<ExceptAllocatable<T1>> fast_allocator) : direct_allocator(direct_allocator),
                                                                                  fast_allocator(fast_allocator){}
            
            std::optional<interval_type> alloc(store_type sz) noexcept{

                try{
                    return this->fast_allocator->alloc(sz);
                } catch(std::exception& e){
                    return this->direct_allocator->alloc(sz);
                }
            }

            void free(const interval_type& interval) noexcept{

                try{
                    this->fast_allocator->free(interval);
                } catch(std::exception& e){
                    this->direct_allocator->free(interval);
                }
            }
    };
    
    template <size_t TREE_HEIGHT, class T, class T1, class T2>
    class StdShrinker: public HeapShrinkable<StdShrinker<TREE_HEIGHT, T, T1, T2>>{
        
        private:

            std::shared_ptr<seeker::Seekable<T>> right_seeker;
            std::shared_ptr<dispatcher::Dispatchable<T1>> blocker;
            std::shared_ptr<dispatcher::Dispatchable<T2>> unblocker;

        public:

            using store_type        = types::store_type;
            using interval_type     = types::interval_type;
            
            using _HeapUlt          = utility::HeapUtility<TREE_HEIGHT>;
            using _IntvUlt          = utility::IntervalUtility<TREE_HEIGHT>;

            StdShrinker(const std::integral_constant<size_t, TREE_HEIGHT>&,
                        std::shared_ptr<seeker::Seekable<T>> right_seeker,
                        std::shared_ptr<dispatcher::Dispatchable<T1>> blocker,
                        std::shared_ptr<dispatcher::Dispatchable<T2>> unblocker): right_seeker(right_seeker),
                                                                                  blocker(blocker),
                                                                                  unblocker(unblocker){}
 

            void shrink(store_type virtual_base) noexcept{
                
                if (virtual_base == _HeapUlt::base_length()){
                    return;
                } else if (virtual_base > _HeapUlt::base_length()){
                    std::abort();
                }
                
                this->blocker->dispatch(get_trailing(virtual_base));
            }

            store_type shrink() noexcept{
                
                auto new_virtual_base = this->get_most_compact_virtual_base(); 
                this->shrink(new_virtual_base);

                return new_virtual_base;
            }

            void unshrink(store_type virtual_base) noexcept{

                if (virtual_base == _HeapUlt::base_length()){
                    return;
                } else if (virtual_base > _HeapUlt::base_length()){
                    std::abort();
                }

                this->unblocker->dispatch(get_trailing(virtual_base));
            }

        private:

            auto get_trailing(store_type virtual_base) -> interval_type{

                const auto ROOT     = size_t{0u};
                auto trailing       = _IntvUlt::incl_right_shrink(_IntvUlt::idx_to_interval(ROOT), virtual_base);

                return trailing;
            }

            auto get_most_compact_virtual_base() -> store_type{

                const auto ROOT     = std::integral_constant<size_t, 0u>();
                auto r_intv         = this->right_seeker->seek(ROOT);

                if (!r_intv){
                    return _HeapUlt::base_length();
                }
  
                return _IntvUlt::get_interval_beg(r_intv.value());
            }
    };
}

namespace dg::heap::core{   

    template <class T>
    class Allocator: public virtual Allocatable{

        private:

            std::shared_ptr<internal_core::NoExceptAllocatable<T>> ins;
        
        public:

            using store_type    = types::store_type;
            using interval_type = types::interval_type;

            Allocator(std::shared_ptr<internal_core::NoExceptAllocatable<T>> ins): ins(ins){}

            std::optional<interval_type> alloc(store_type sz) noexcept{

                return this->ins->alloc(sz);
            }

            void free(const interval_type& intv) noexcept{

                this->ins->free(intv);
            }
    };

    template <class T, class T1>
    class Allocator_X: public virtual Allocatable_X{

        private:

            std::shared_ptr<internal_core::NoExceptAllocatable<T>> allocator;
            std::shared_ptr<internal_core::HeapShrinkable<T1>> shrinker;
        
        public:

            using store_type    = types::store_type;
            using interval_type = types::interval_type;

            Allocator_X(std::shared_ptr<internal_core::NoExceptAllocatable<T>> allocator, 
                        std::shared_ptr<internal_core::HeapShrinkable<T1>> shrinker): allocator(allocator),
                                                                                      shrinker(shrinker){}
            
            
            std::optional<interval_type> alloc(store_type sz) noexcept{
                
                return this->allocator->alloc(sz);
            }

            void free(const interval_type& relative) noexcept{ 
                
                this->allocator->free(relative);
            }

            store_type shrink() noexcept{
                
                return this->shrinker->shrink();
            }

            void shrink(store_type virtual_base) noexcept{
                
                this->shrinker->shrink(virtual_base);
            }

            void unshrink(store_type virtual_base) noexcept{
                
                this->shrinker->unshrink(virtual_base);
            }
    };

};

namespace dg::heap::make{

    template <class NodeType, class TracebackType>
    struct Data{
        
        std::pair<NodeType *, size_t> node_arr;
        std::pair<TracebackType *, size_t> traceback_arr;  
        
    };

    using HeapData                              = Data<types::Node, datastructure::boolvector::bucket_type>; 
    static inline const size_t DEFLT_ALIGNMENT  = 0b10000000; // cross-platform compatibility due to alignment incompatibility

    template <size_t HEIGHT>
    struct NodeArraySpecs{

        using type  = types::Node;
        static_assert(std::has_unique_object_representations_v<type>);

        static inline _DG_CONSTEVAL auto size() -> size_t{
    
            using _HeapUtility      = utility::HeapUtility<HEIGHT>;
            auto rs                 = _HeapUtility::node_count();
            
            return rs;
        }
    };

    template <size_t HEIGHT>
    struct BoolArraySpecs{

        using type  = datastructure::boolvector::bucket_type;  
        static_assert(std::has_unique_object_representations_v<type>);

        static inline _DG_CONSTEVAL auto size() -> size_t{

            using _HeapUtility      = utility::HeapUtility<HEIGHT>;
            using _OffsetConverter  = utility::OffsetConverter<HEIGHT>;
            auto rs                 = _OffsetConverter::get_base_bit_offset(_HeapUtility::node_count()) / datastructure::boolvector::BIT_PER_BUCKET + 1; //relaxed size

            return rs;
        } 
    };

    template <class ArraySpecs>
    struct ArrayMaker{

        private:

            using _MemoryUtility            = utility::MemoryUtility;
            using _MemoryService            = memory::MemoryService;
            using _LifeTimeTracker          = memory::LifeTimeTracker;
            
        public:

            using type                      = typename ArraySpecs::type;

            static_assert(DEFLT_ALIGNMENT % alignof(type) == 0); //alignof(type) is no stricter than ... -
            static_assert(std::is_trivial_v<type>);

            static inline _DG_CONSTEVAL auto size_no_align() -> size_t{
                
                return ArraySpecs::size() * sizeof(type);
            }

            static inline _DG_CONSTEVAL auto size() -> size_t{

                return size_no_align() + DEFLT_ALIGNMENT; 
            }

            static inline auto data(char * buf) noexcept -> char *{

                return _MemoryUtility::align<DEFLT_ALIGNMENT>(buf); 
            }

            static inline auto array(char * buf) noexcept -> std::pair<type *, size_t>{

                return {_LifeTimeTracker::retrieve<type>(data(buf)), ArraySpecs::size()}; //
            }

            static inline auto get_misalignment(char * buf) noexcept -> size_t{ //guaranteed to be > 0
    
                return static_cast<size_t>(_MemoryUtility::get_distance_vector(buf, data(buf))); //acknowledged intptr_t 
            }

            static inline void inplace_init(char * buf) noexcept{

                type * arr = new (data(buf)) type[ArraySpecs::size()];
                _LifeTimeTracker::start_lifetime(arr); 
            }

            static inline void inplace_launder(char * buf) noexcept{
                
                type * arr = _MemoryService::launder_arr<type>(data(buf), ArraySpecs::size());
                _LifeTimeTracker::start_lifetime(arr);
            }

            static inline void inplace_destruct(char * buf) noexcept{

                _LifeTimeTracker::end_lifetime(data(buf));
            }
    };

    template <class _ResourceMaker>
    struct AlignmentEmbeddedResourceMaker: private _ResourceMaker{

        public:

            using type              = typename _ResourceMaker::type;
            using header_type       = uint8_t;
        
        private:

            // using _EndiannessUlt    = memory::SyncedEndiannessService;
            using _MemoryUlt        = utility::MemoryUtility;
            using Base              = _ResourceMaker;

            static inline auto get_misalignment(char * buf) noexcept -> size_t{

                return Base::get_misalignment(_MemoryUlt::forward_shift(buf, sizeof(header_type)));
            }

        public:
            
            static inline _DG_CONSTEVAL auto size() -> size_t{

                return Base::size() + sizeof(header_type);
            }

            static inline auto data(char * buf) noexcept -> char *{

                return Base::data(_MemoryUlt::forward_shift(buf, sizeof(header_type))); 
            }

            static inline auto array(char * buf) noexcept -> decltype(Base::array(buf)){

                return Base::array(_MemoryUlt::forward_shift(buf, sizeof(header_type))); 
            }

            static inline void inplace_init(char * buf) noexcept {

                char * fs_buf       = _MemoryUlt::forward_shift(buf, sizeof(header_type));
                auto misalignment   = static_cast<header_type>(get_misalignment(buf));

                dg::compact_serializer::core::serialize(misalignment, buf);
                Base::inplace_init(fs_buf);
            }

            static inline void inplace_correct(char * buf) noexcept{
                
                auto org_misalignment  = header_type{}; 
                dg::compact_serializer::core::deserialize(buf, org_misalignment);
                auto cur_misalignment   = static_cast<header_type>(get_misalignment(buf));
                auto sz                 = Base::size_no_align();

                char * fs_buf           = _MemoryUlt::forward_shift(buf, sizeof(header_type));
                char * dst              = _MemoryUlt::forward_shift(fs_buf, cur_misalignment);
                char * src              = _MemoryUlt::forward_shift(fs_buf, org_misalignment);

                std::memmove(dst, src, sz);
                dg::compact_serializer::core::serialize(cur_misalignment, buf);
                Base::inplace_launder(fs_buf);
            }

            static inline void inplace_destruct(char * buf) noexcept{

                Base::inplace_destruct(_MemoryUlt::forward_shift(buf, sizeof(header_type)));
            }
    };

    template <size_t DYNAMIC_HEIGHT, size_t TRACEBACK_HEIGHT>
    struct HeapDataMaker{

        private:

            using _NodeMaker        = AlignmentEmbeddedResourceMaker<ArrayMaker<NodeArraySpecs<DYNAMIC_HEIGHT>>>;
            using _BvecMaker        = AlignmentEmbeddedResourceMaker<ArrayMaker<BoolArraySpecs<TRACEBACK_HEIGHT>>>; 
            using _MemoryUtility    = utility::MemoryUtility;
        
        public:

            using type              = Data<typename _NodeMaker::type, typename _BvecMaker::type>;

            static inline _DG_CONSTEVAL auto size() -> size_t{
                
                return _NodeMaker::size() + _BvecMaker::size();
            }

            static inline auto get(char * buf) noexcept -> type{

                return type{_NodeMaker::array(buf), _BvecMaker::array(_MemoryUtility::forward_shift(buf, _NodeMaker::size()))};
            }

            static inline void inplace_init(char * buf) noexcept{

                _NodeMaker::inplace_init(buf);
                _BvecMaker::inplace_init(_MemoryUtility::forward_shift(buf, _NodeMaker::size())); 
            }

            static inline void inplace_correct(char * buf) noexcept{

                _NodeMaker::inplace_correct(buf);
                _BvecMaker::inplace_correct(_MemoryUtility::forward_shift(buf, _NodeMaker::size()));
            } 

            static inline void inplace_destruct(char * buf) noexcept{

                _NodeMaker::inplace_destruct(buf);
                _BvecMaker::inplace_destruct(_MemoryUtility::forward_shift(buf, _NodeMaker::size()));
            }
    };

    template <intmax_t DH_Num, intmax_t DH_Denom, intmax_t TB_Num, intmax_t TB_Denom>
    struct TreeSpecs{
                
        TreeSpecs() = default;
        TreeSpecs(std::ratio<DH_Num, DH_Denom>, std::ratio<TB_Num, TB_Denom>){}

        static constexpr uint8_t get_dynamic_height(const uint8_t TREE_HEIGHT){
            
            intmax_t CASTED_TREE_HEIGHT = static_cast<intmax_t>(TREE_HEIGHT);
            intmax_t rs                 = CASTED_TREE_HEIGHT * DH_Num / DH_Denom;

            return static_cast<uint8_t>(rs);  
        }

        static constexpr uint8_t get_traceback_height(const uint8_t TREE_HEIGHT){

            intmax_t CASTED_TREE_HEIGHT = static_cast<intmax_t>(TREE_HEIGHT);
            intmax_t rs                 = CASTED_TREE_HEIGHT * TB_Num / TB_Denom;
            
            return static_cast<uint8_t>(rs);
        } 
    };

    using StdTreeSpecs = decltype(TreeSpecs(typename std::ratio<2, 3>::type{}, typename std::ratio<1, 1>::type{}));

    template <size_t TREE_HEIGHT_V, class _CustomTreeSpecs>
    struct HeapMakerFactory{
        
        static constexpr size_t TREE_HEIGHT         = TREE_HEIGHT_V;
        static constexpr size_t DYNAMIC_HEIGHT      = _CustomTreeSpecs::get_dynamic_height(TREE_HEIGHT); //
        static constexpr size_t TRACEBACK_HEIGHT    = _CustomTreeSpecs::get_traceback_height(TREE_HEIGHT);

        using type = make::HeapDataMaker<DYNAMIC_HEIGHT, TRACEBACK_HEIGHT>;

        HeapMakerFactory(std::integral_constant<size_t, TREE_HEIGHT_V>, _CustomTreeSpecs){}

    };

    template <size_t TREE_HEIGHT, class _CustomTreeSpecs>
    static inline auto get_heap_maker(std::integral_constant<size_t, TREE_HEIGHT> ic, _CustomTreeSpecs tree_specs) -> typename decltype(HeapMakerFactory(ic, tree_specs))::type;
};

namespace dg::heap::resource{

    struct MVCSpawner{

        template <size_t DYNAMIC_HEIGHT, size_t BACKTRACK_HEIGHT, class ID>
        static auto spawn_model(make::HeapData data, 
                                const std::integral_constant<size_t, DYNAMIC_HEIGHT> d_height,
                                const std::integral_constant<size_t, BACKTRACK_HEIGHT> b_height,
                                const ID id){
            
            const uint16_t CACHE_CAPACITY = 1024u; 
            
            types::Node * node_arr  = data.node_arr.first;
            datastructure::boolvector::StdVectorOperator bvec_arr(data.traceback_arr.first, data.traceback_arr.second); 
            cache::StdCacheController cache(CACHE_CAPACITY);
            data::StdHeapDataView data_view(d_height, b_height, node_arr, bvec_arr, std::move(cache), id);

            using rs_type   = std::remove_pointer_t<decltype(data_view.to_heap_data())>;
            return rs_type{};
        }

        template <class T>
        static auto spawn_view(const data::HeapData<T> data){

            data::StdStorageExtractor extractor(data);

            using rs_type   = std::remove_pointer_t<decltype(extractor.to_storage_extractible())>;
            return rs_type{};
        }   

        template <class T, class T1>
        static auto spawn_controller(const data::HeapData<T> model, 
                                     const data::StorageExtractible<T1> view){
            
            internal_core::StdHeapOperator controller(model, view);

            using rs_type   = std::remove_pointer_t<decltype(controller.to_heap_operatable())>;
            return rs_type{};
        }
    };

    struct AllocatorSpawner{

        template <class T, class T1>
        static auto spawn_direct_allocator(const data::StorageExtractible<T> view, 
                                           const internal_core::HeapOperatable<T1> controller){
            
            constexpr auto HEIGHT   = data::StorageExtractible<T>::TREE_HEIGHT; 

            auto _seeker    = seeker::SeekerSpawner::get_max_interval_seeker(view);
            auto blocker    = dispatcher::DispatcherWrapperSpawner::get_std_wrapper(dispatcher::DispatcherSpawner::get_std_block_dispatcher(controller));
            auto unblocker  = dispatcher::DispatcherWrapperSpawner::get_std_wrapper(dispatcher::DispatcherSpawner::get_std_unblock_dispatcher(controller));

            internal_core::DirectAllocator allocator(_seeker, blocker, unblocker);

            using ins_type  = decltype(allocator);
            using rs_type   = std::shared_ptr<std::remove_pointer_t<decltype(allocator.to_allocatable())>>;
            rs_type rs      = std::unique_ptr<ins_type>(new ins_type(allocator));

            return rs;
        }

        template <class BuyableSpawnable, class SellableSpawnable>
        static auto spawn_fast_allocator(BuyableSpawnable buyable_spawner, SellableSpawnable sellable_spawner){

            using buyable_type      = decltype(buyable_spawner());
            using sellable_type     = decltype(sellable_spawner());

            static_assert(std::is_same_v<types_space::nillable_t<buyable_type>, buyable_type>);
            static_assert(std::is_same_v<types_space::nillable_t<sellable_type>, sellable_type>);

            internal_core::FastAllocator allocator(buyable_spawner, sellable_spawner, buyable_type{}, sellable_type{});

            using ins_type          = decltype(allocator);
            using rs_type           = std::shared_ptr<std::remove_pointer_t<decltype(allocator.to_allocatable())>>; //noexceptcrtp ... 
            rs_type rs              = std::unique_ptr<ins_type>(new ins_type(allocator));

            return rs;
        }

        template <class T, class T1>
        static auto spawn_fast_allocator(const data::StorageExtractible<T> view,
                                         const internal_core::HeapOperatable<T1> controller){
            
            constexpr auto BUY_LIM  = size_t{1} << 15; //
            constexpr auto HEIGHT   = data::StorageExtractible<T>::TREE_HEIGHT; 

            auto max_gen            = seeker::SeekerLambdanizer::get_root_seeker(seeker::SeekerSpawner::get_max_interval_seeker(view));
            auto buyable_spawner    = market::BrokerSpawner::get_sale_broker_spawner(controller, max_gen);
            auto sellable_spawner   = market::BrokerSpawner::get_buy_broker_spawner(controller, BUY_LIM);

            return spawn_fast_allocator(buyable_spawner, sellable_spawner);
        }

        template <class T, class T1>
        static auto spawn_batch_fast_allocator(const data::StorageExtractible<T> view,
                                               const internal_core::HeapOperatable<T> controller){
            
            constexpr auto BUY_LIM  = size_t{1} << 15;
            constexpr auto HEIGHT   = data::StorageExtractible<T>::TREE_HEIGHT; 

            auto batch_gen          = seeker::SeekerLambdanizer::get_greedy_batch_seeker(seeker::SeekerSpawner::get_max_interval_seeker(view));
            auto buyable_spawner    = market::BrokerSpawner::get_sale_broker_spawner(controller, batch_gen);
            auto sellable_spawner   = market::BrokerSpawner::get_buy_broker_spawner(controller, BUY_LIM);

            return spawn_fast_allocator(buyable_spawner, sellable_spawner);
        }

        template <class T, class T1>
        static auto spawn_std_shrinker(const data::StorageExtractible<T> view,
                                       const internal_core::HeapOperatable<T1> controller){
            
            constexpr auto HEIGHT   = data::StorageExtractible<T>::TREE_HEIGHT; 

            auto r_seeker   = seeker::SeekerSpawner::get_right_interval_seeker(view);
            auto blocker    = dispatcher::DispatcherWrapperSpawner::get_std_wrapper(dispatcher::DispatcherSpawner::get_std_block_dispatcher(controller));
            auto unblocker  = dispatcher::DispatcherWrapperSpawner::get_std_wrapper(dispatcher::DispatcherSpawner::get_std_unblock_dispatcher(controller));

            internal_core::StdShrinker shrinker(std::integral_constant<size_t, HEIGHT>{}, r_seeker, blocker, unblocker);

            using ins_type  = decltype(shrinker);
            using rs_type   = std::shared_ptr<std::remove_pointer_t<decltype(shrinker.to_heap_shrinkable())>>;
            rs_type rs      = std::unique_ptr<ins_type>(new ins_type(shrinker));

            return rs;
        }

        template <class T, class T1>
        static auto spawn_std_allocator(std::shared_ptr<internal_core::NoExceptAllocatable<T>> noexcept_allocator,
                                        std::shared_ptr<internal_core::ExceptAllocatable<T1>> except_allocator){
            
            internal_core::StdAllocator allocator(noexcept_allocator, except_allocator);
            
            using ins_type  = decltype(allocator);
            using rs_type   = std::shared_ptr<std::remove_pointer_t<decltype(allocator.to_allocatable())>>;

            rs_type rs      = std::make_unique<ins_type>(allocator);
            return rs;
        }
    };

    struct Virtualizer{

        template <class T>
        static auto spawn_virtual_allocator(std::shared_ptr<internal_core::NoExceptAllocatable<T>> ins) -> std::unique_ptr<core::Allocatable>{
            
            core::Allocator rs(ins);
            return std::make_unique<decltype(rs)>(rs); 
        }

        template <class T, class T1>
        static auto spawn_virtual_allocator_x(std::shared_ptr<internal_core::NoExceptAllocatable<T>> ins,
                                              std::shared_ptr<internal_core::HeapShrinkable<T1>> shrinker) -> std::unique_ptr<core::Allocatable_X>{
            
            core::Allocator_X rs(ins, shrinker);
            return std::make_unique<decltype(rs)>(rs);
        }
    }; 

    struct HeapResourceBase{

        using height_type       = uint8_t;
        using _MemoryUlt        = utility::MemoryUtility; 

        static constexpr auto TREE_SPECS     = make::StdTreeSpecs{};

        static auto fwd(char * data) noexcept -> char *{

            return _MemoryUlt::forward_shift(data, sizeof(height_type));    
        }

        template <size_t TREE_HEIGHT>
        static constexpr auto get_heap_maker(const std::integral_constant<size_t, TREE_HEIGHT> HEIGHT){
            
            using type  = std::add_pointer_t<decltype(make::get_heap_maker(HEIGHT, TREE_SPECS))>;
            return type{};
        }

        template <class CallBack>
        static constexpr void get_heap_maker(const CallBack& cb_lambda, height_type HEIGHT){
            
            constexpr auto DELTA    = limits::EXCL_MAX_HEAP_HEIGHT - limits::MIN_HEAP_HEIGHT;
            constexpr auto idx_seq  = std::make_index_sequence<DELTA>{};

            [=]<size_t ...IDX>(const std::index_sequence<IDX...>){
                (
                    [=]{

                        (void) IDX;
                        constexpr auto PTR  = IDX + limits::MIN_HEAP_HEIGHT;

                        if (PTR == HEIGHT){
                            cb_lambda(get_heap_maker(std::integral_constant<size_t, PTR>{}));
                        }

                    }(), ...
                );
            }(idx_seq);
        }

        static constexpr size_t size(height_type HEIGHT){

            size_t rs{};
            auto cb_lambda  = [&]<class HeapMaker>(HeapMaker *){rs = HeapMaker::size() + sizeof(height_type);};
            get_heap_maker(cb_lambda, HEIGHT);

            return rs;
        } 
    };

    struct HeapResourceInitializer: HeapResourceBase{
        
        private:

            using _Base             = HeapResourceBase;

        public:

            using height_type       = typename _Base::height_type;

            static inline void inplace_init(char * buf, height_type HEIGHT) noexcept{
                
                dg::compact_serializer::core::serialize(HEIGHT, buf);
                auto cb_lambda  = [=]<class HeapMaker>(HeapMaker *){HeapMaker::inplace_init(fwd(buf));};
                get_heap_maker(cb_lambda, HEIGHT);
            }

            static inline void inplace_destruct(char * buf) noexcept{

                height_type HEIGHT  = {};
                dg::compact_serializer::core::deserialize(buf, HEIGHT);
                auto cb_lambda      = [=]<class HeapMaker>(HeapMaker *){HeapMaker::inplace_destruct(fwd(buf));};
                get_heap_maker(cb_lambda, HEIGHT);
            }

            static inline auto init(height_type HEIGHT) -> std::shared_ptr<char[]>{

                auto revert_lambda = [](char * arr){
                    inplace_destruct(static_cast<char *>(arr));
                    delete[] arr;
                };
           
                std::unique_ptr<char[], decltype(revert_lambda)> rs{new char[size(HEIGHT)], revert_lambda};
                inplace_init(rs.get(), HEIGHT);

                return rs;
            }

            static inline void inplace_correct(char * buf) noexcept{
                
                height_type HEIGHT  = {};
                dg::compact_serializer::core::deserialize(buf, HEIGHT);
                auto cb_lambda      = [=]<class HeapMaker>(HeapMaker *){HeapMaker::inplace_correct(fwd(buf));};    
                get_heap_maker(cb_lambda, HEIGHT);
            }
    };

    struct HeapResourceManipulator: HeapResourceBase{

        using _Base                     = HeapResourceBase;        
        using _ResourceInitializer      = HeapResourceInitializer;
        using _Instantitor              = instantiator::IntervalDataInstantiator;
        using _Transferer               = instantiator::IntervalDataTransferer;
        using _MVCSpawner               = MVCSpawner;

        using height_type               = typename _Base::height_type;
        
        struct TemporaryID{}; 
        struct TemporaryID2{};
        
        template <class CallBack, class ID>
        static constexpr void get_heap_mvc(const CallBack& cb_lambda, char * buf, const ID id){
            
            height_type HEIGHT  = {};
            dg::compact_serializer::core::deserialize(buf, HEIGHT);
            constexpr auto DELTA        = limits::EXCL_MAX_HEAP_HEIGHT - limits::MIN_HEAP_HEIGHT;
            constexpr auto idx_seq      = std::make_index_sequence<DELTA>{};

            [=]<size_t ...IDX>(const std::index_sequence<IDX...>){
                (
                    [=]{

                        (void) IDX;

                        constexpr auto TREE_HEIGHT  = limits::MIN_HEAP_HEIGHT + IDX;
                        constexpr auto D_HEIGHT     = TREE_SPECS.get_dynamic_height(TREE_HEIGHT);
                        constexpr auto TB_HEIGHT    = TREE_SPECS.get_traceback_height(TREE_HEIGHT); 
                        using _HeapMaker            = std::remove_pointer_t<decltype(get_heap_maker(std::integral_constant<size_t, TREE_HEIGHT>{}))>;

                        if (TREE_HEIGHT == HEIGHT){

                            auto model      = MVCSpawner::spawn_model(_HeapMaker::get(fwd(buf)), std::integral_constant<size_t, D_HEIGHT>{}, std::integral_constant<size_t, TB_HEIGHT>{}, id);
                            auto view       = MVCSpawner::spawn_view(model);
                            auto controller = MVCSpawner::spawn_controller(model, view);   

                            cb_lambda(model, view, controller);
                        }

                    }(), ...
                );
            }(idx_seq);
        }

        static inline void defaultize(char * buf){ //REIVEW: noexcept
            
            auto cb_handler             = []<class T, class T1, class T2>(const data::HeapData<T>, const data::StorageExtractible<T1>, const internal_core::HeapOperatable<T2> controller){
                _Instantitor::initialize(controller);
            };

            get_heap_mvc(cb_handler, buf, TemporaryID{});
        }
        
        static inline void transfer(char * transferee, char * transferer){ //REIVEW: noexcept

            auto cb_handler                 = [=]<class T, class T1, class T2>(const data::HeapData<T>, const data::StorageExtractible<T1>, const internal_core::HeapOperatable<T2> transferee){

                auto cb_handler_v2  = [=]<class TT, class TT1, class TT2>(const data::HeapData<TT>, const data::StorageExtractible<TT1>, const internal_core::HeapOperatable<TT2> transferer){
                    _Transferer::transfer(transferee, transferer);
                };

                get_heap_mvc(cb_handler_v2, transferer, TemporaryID2{});
            };

            get_heap_mvc(cb_handler, transferee, TemporaryID{});
        }
    };

    struct ResourceController{
        
        using _Initializer  = HeapResourceInitializer;
        using _Manipulator  = HeapResourceManipulator;
        using _Spawner      = AllocatorSpawner;
        using _Virtualizer  = Virtualizer;

        static constexpr auto get_memory_usage(const uint8_t HEIGHT) -> size_t{

            return _Initializer::size(HEIGHT);
        }  

        static void set_reservoir(std::unique_ptr<char[]> buf, size_t buf_sz){

            auto reservoir_ins  = memory::AllocatorInitializer::get_mtx_controlled_allocator(memory::AllocatorInitializer::get_bump_allocator(std::move(buf), buf_sz));
            memory::MemoryMarket::reservoir = std::move(reservoir_ins);
        }

        static void inplace_make(const uint8_t HEIGHT, char * buf){

            _Initializer::inplace_init(buf, HEIGHT);
            _Manipulator::defaultize(buf);
        }

        static auto make(const uint8_t HEIGHT) -> std::shared_ptr<char[]>{

            auto rs = _Initializer::init(HEIGHT);
            _Manipulator::defaultize(rs.get());
            
            return rs;
        } 

        static void inplace_correct(char * buf) noexcept{

            _Initializer::inplace_correct(buf);
        }

        static void inplace_destruct(char * buf) noexcept{

            _Initializer::inplace_destruct(buf);
        }

        static void transfer(char * transferee, char * transferer){

            _Manipulator::transfer(transferee, transferer);
        }

        template <class ID>
        static auto get_allocatable(char * buf, const ID id) -> std::unique_ptr<core::Allocatable>{

            std::unique_ptr<core::Allocatable> rs{};
            auto cb_handler = [&]<class T, class T1, class T2>(const data::HeapData<T> model, const data::StorageExtractible<T1> view, const internal_core::HeapOperatable<T2> controller){
                rs = _Virtualizer::spawn_virtual_allocator(_Spawner::spawn_direct_allocator(view, controller));
            };
            _Manipulator::get_heap_mvc(cb_handler, buf, id);

            return rs;
        } 

        template <class ID>
        static auto get_allocatable_x(char * buf, const ID id) -> std::unique_ptr<core::Allocatable_X>{

            std::unique_ptr<core::Allocatable_X> rs{};
            auto cb_handler = [&]<class T, class T1, class T2>(const data::HeapData<T> model, const data::StorageExtractible<T1> view, const internal_core::HeapOperatable<T2> controller){
                rs  = _Virtualizer::spawn_virtual_allocator_x(_Spawner::spawn_std_allocator(_Spawner::spawn_direct_allocator(view, controller), _Spawner::spawn_fast_allocator(view, controller)), 
                                                              _Spawner::spawn_std_shrinker(view, controller));
            };
            _Manipulator::get_heap_mvc(cb_handler, buf, id);

            return rs;
        }
    };

    template <class _Controller>
    struct MtxControlledResourceController{ //--minimize reservoir usage 

        static inline std::mutex mtx{};
        
        template <class ...Args>
        static constexpr auto get_memory_usage(Args&& ...args){
            
            return _Controller::get_memory_usage(std::forward<Args>(args)...);
        }

        template <class ...Args>
        static void set_reservoir(Args&& ...args){

            std::lock_guard<std::mutex> guard(mtx);
            _Controller::set_reservoir(std::forward<Args>(args)...);
        }
        
        template <class ...Args>
        static void inplace_make(Args&& ...args){

            std::lock_guard<std::mutex> guard(mtx);
            _Controller::inplace_make(std::forward<Args>(args)...);
        } 

        template <class ...Args>
        static auto make(Args&& ...args){

            std::lock_guard<std::mutex> guard(mtx);
            return _Controller::make(std::forward<Args>(args)...);
        }

        template <class ...Args>
        static void inplace_correct(Args&& ...args){

            std::lock_guard<std::mutex> guard(mtx);
            _Controller::inplace_correct(std::forward<Args>(args)...);
        }

        template <class ...Args>
        static void inplace_destruct(Args&& ...args){

            std::lock_guard<std::mutex> guard(mtx);
            _Controller::inplace_destruct(std::forward<Args>(args)...);
        }

        template <class ...Args>
        static void transfer(Args&& ...args){

            std::lock_guard<std::mutex> guard(mtx);
            _Controller::transfer(std::forward<Args>(args)...);
        }

        template <class ...Args>
        static auto get_allocatable(Args&& ...args){

            std::lock_guard<std::mutex> guard(mtx);
            return _Controller::get_allocatable(std::forward<Args>(args)...);
        }

        template <class ...Args>
        static auto get_allocatable_x(Args&& ...args){

            std::lock_guard<std::mutex> guard(mtx);
            return _Controller::get_allocatable_x(std::forward<Args>(args)...);
        }
    };

};

namespace dg::heap::user_interface{

    using _Controller   = resource::MtxControlledResourceController<resource::ResourceController>;

    extern constexpr auto get_memory_usage(const uint8_t HEIGHT) -> size_t{
    
        return _Controller::get_memory_usage(HEIGHT);
    }

    extern void set_reservoir(std::unique_ptr<char[]> buf, size_t buf_sz){

        _Controller::set_reservoir(std::move(buf), buf_sz);
    }

    extern void inplace_make(const uint8_t HEIGHT, char * buf){

        return _Controller::inplace_make(HEIGHT, buf);
    }

    extern auto make(const uint8_t HEIGHT) -> std::shared_ptr<char[]>{
        
        return _Controller::make(HEIGHT);
    }

    extern void inplace_correct(char * buf) noexcept{

        return _Controller::inplace_correct(buf);
    }

    extern void inplace_destruct(char * buf) noexcept{

        return _Controller::inplace_destruct(buf);
    }

    extern void transfer(char * transferee, char * transferer){

        return _Controller::transfer(transferee, transferer);
    }

    //error - external linking
    template <class ID = std::integral_constant<size_t, 0>>
    extern auto get_allocator(char * data, const ID& id = std::integral_constant<size_t, 0>{}) -> std::unique_ptr<core::Allocatable>{

        return _Controller::get_allocatable(data, id);
    }

    template <class ID = std::integral_constant<size_t, 0>>
    extern auto get_allocator_x(char * data, const ID& id = std::integral_constant<size_t, 0>{}) -> std::unique_ptr<core::Allocatable_X>{

        return _Controller::get_allocatable_x(data, id);
    }

};

#endif

#ifndef __CRTP_BOOLVECTOR_H__
#define __CRTP_BOOLVECTOR_H__

#include "stdint.h"
#include "limits.h"
#include <limits>
#include <utility>
#include <cstring>
#include <tuple>

namespace dg::datastructure::boolvector{

    using bucket_type = size_t;
    static inline constexpr size_t BIT_PER_BUCKET = sizeof(bucket_type) * CHAR_BIT; 
    
    template <class T>
    class ReadableVector{

        public:
            
            bool get(size_t idx){

                return static_cast<T *>(this)->get(idx);

            }

            template <size_t IDX>
            bool get(){

                return static_cast<T *>(this)->template get<IDX>();

            }

            size_t size(){

                return static_cast<T *>(this)->size();

            }

            void * data(){

                return static_cast<T *>(this)->data();

            }

            ReadableVector * to_readable_vector(){

                return this;

            }

    };

    template <class T>
    class OperatableVector: public ReadableVector<T>{

        public:

            void set(size_t idx, bool val){

                static_cast<T *>(this)->set(idx, val);

            }

            template <size_t IDX>
            void set(bool val){

                static_cast<T *>(this)->template set<IDX>(val);

            }

            template <bool Val>
            void set(size_t idx, const std::integral_constant<bool, Val>& val){

                static_cast<T *>(this)->set(idx, val);

            }

            OperatableVector * to_operatable_vector(){

                return this;

            }

    };

    template <class T>
    class ReallocatableVector: public OperatableVector<T>{

        public:

            void resize(size_t sz){

                static_cast<T *>(this)->resize(sz);

            }

            ReallocatableVector * to_reallocatable_vector(){

                return this;

            }

    };

};

namespace dg::datastructure::boolvector::utility{

    static inline void emptify(void * buf, size_t bucket_length){  

        std::memset(buf, int{}, bucket_length * sizeof(bucket_type));

    }

    template <class T>
    static inline constexpr auto narrow(T value) -> typename std::enable_if<std::is_same_v<decltype(value != 0), bool>, bool>::type{ //REVIEW

        return value != 0;

    }

    template <class T>
    static inline constexpr auto to_set_arg(T val){
        
        return narrow(val);
    }

    template <bool Val>
    static inline constexpr auto to_set_arg(const std::integral_constant<bool, Val>& val){

        return val;
    }

    template <size_t ...IDX>
    static inline constexpr auto bool_tup_type(const std::index_sequence<IDX...>&) -> decltype(std::make_tuple(narrow(IDX)...));

    static inline constexpr auto slot(size_t idx) -> size_t{

        return idx / BIT_PER_BUCKET;

    } 
    
    static inline constexpr auto offset(size_t idx) -> size_t{

        return idx % BIT_PER_BUCKET;

    }

    static inline constexpr auto index(size_t slot) -> size_t{

        return slot * BIT_PER_BUCKET;

    }

    static inline constexpr auto index(size_t slot, size_t offset) -> size_t{

        return index(slot) + offset;

    }

    template <class RS_Type = bucket_type>
    static inline constexpr auto bit_control(size_t offset) -> RS_Type{

        return RS_Type{1} << offset;

    }

    template <class RS_Type = bucket_type>
    static inline constexpr auto true_toggle(size_t offset) -> RS_Type{

        return bit_control<RS_Type>(offset);

    }

    template <class RS_Type = bucket_type>
    static inline constexpr auto false_toggle(size_t offset) -> RS_Type{
        
        static_assert(std::is_unsigned_v<RS_Type>);
        return std::numeric_limits<RS_Type>::max() ^ true_toggle<RS_Type>(offset);

    }

    template <class RS_Type, class ...Args>
    static inline constexpr auto intify(const std::tuple<Args...>& tup) -> RS_Type{

        static_assert(std::numeric_limits<RS_Type>::is_integer);
        static_assert(std::conjunction_v<std::is_same<Args, bool>...>);

        RS_Type rs{};
        constexpr auto seq = std::make_index_sequence<sizeof...(Args)>();

        [=]<size_t ...IDX>(const std::index_sequence<IDX...>&, RS_Type& op){                
            (
                [=](size_t, RS_Type& op){

                    op |= RS_Type{std::get<IDX>(tup)} << IDX; 

                }(IDX, op), ...
            );
        }(seq, rs);

        return rs; 

    }

    template <size_t BIT_LENGTH, class T, std::enable_if_t<std::numeric_limits<T>::is_integer, bool> = true>
    static inline constexpr auto boolify(T value) -> decltype(bool_tup_type(std::make_index_sequence<BIT_LENGTH>())){

        constexpr auto seq = std::make_index_sequence<BIT_LENGTH>();

        auto lambda = [=]<size_t ...IDX>(const std::index_sequence<IDX...>&){            
            return std::make_tuple(narrow(value & bit_control<T>(IDX))...);
        };

        return lambda(seq);
    }
    
    template <size_t BIT_LENGTH, class ValType, ValType Val>
    static inline constexpr auto boolify(const std::integral_constant<ValType, Val>&, const std::integral_constant<size_t, BIT_LENGTH>&){

        constexpr auto seq  = std::make_index_sequence<BIT_LENGTH>();

        auto lambda = [=]<size_t ...IDX>(const std::index_sequence<IDX...>&) constexpr{
            return std::make_tuple(std::integral_constant<bool, narrow(Val & bit_control<ValType>(IDX))>()...);
        };

        return lambda(seq);
    };

    template <class T>
    struct is_readable_intf: std::false_type{};

    template <class T>
    struct is_readable_intf<ReadableVector<T> *>: std::true_type{};

    template <class T, class = void>
    struct has_readable_base: std::false_type{};

    template <class T>
    struct has_readable_base<T, std::void_t<decltype(std::declval<T>().to_readable_vector())>>: is_readable_intf<decltype(std::declval<T>().to_readable_vector())>{};

    template <class T>
    static inline constexpr bool has_readable_base_v = has_readable_base<T>::value;

    template <class T>
    struct is_operatable_intf: std::false_type{};

    template <class T>
    struct is_operatable_intf<OperatableVector<T> *>: std::true_type{};

    template <class T, class = void>
    struct has_operatable_base: std::false_type{};

    template <class T>
    struct has_operatable_base<T, std::void_t<decltype(std::declval<T>().to_operatable_vector())>>: is_operatable_intf<decltype(std::declval<T>().to_operatable_vector())>{};

    template <class T>
    static inline constexpr bool has_operatable_base_v = has_operatable_base<T>::value;

}

namespace dg::datastructure::boolvector::operation{
    
    template <class T, class ...Args>
    static inline void sequential_set(OperatableVector<T>& op, size_t idx, const std::tuple<Args...>& tup){

        constexpr auto seq = std::make_index_sequence<sizeof...(Args)>();

        [idx]<size_t ...IDX>(const std::index_sequence<IDX...>&, const std::tuple<Args...>& inp, OperatableVector<T>& oop){
            (oop.set(idx + IDX, utility::to_set_arg(std::get<IDX>(inp))), ...);
        }(seq, tup, op);

    }

    template <size_t LENGTH, class T>
    static inline auto sequential_get(ReadableVector<T>& op, size_t idx) -> decltype(utility::bool_tup_type(std::make_index_sequence<LENGTH>())){

        constexpr auto seq = std::make_index_sequence<LENGTH>();

        auto lambda = [idx]<size_t ...IDX>(const std::index_sequence<IDX...>&, ReadableVector<T>& oop){
            return std::make_tuple(oop.get(idx + IDX)...);
        };

        return lambda(seq, op);

    }
    
};

namespace dg::datastructure::boolvector{

    class StdVectorReader: public ReadableVector<StdVectorReader>{

        private:

            bucket_type * buf;
            size_t sz;

        public:

            StdVectorReader() = default;
            
            StdVectorReader(bucket_type * buf, size_t sz) : buf(buf), sz(sz){}

            bool get(size_t idx){

                return utility::narrow(this->buf[utility::slot(idx)] & utility::bit_control(utility::offset(idx)));

            }

            template <size_t IDX>
            bool get(){
                
                constexpr auto slot = utility::slot(IDX);
                constexpr auto bit_control = utility::bit_control(utility::offset(IDX));

                return utility::narrow(this->buf[slot] & bit_control); 

            }

            size_t size(){

                return utility::index(this->sz);

            }

            void * data(){

                return this->buf;

            }

    };

    class StdVectorOperator: public OperatableVector<StdVectorOperator>,
                             private StdVectorReader{
        
        public:

            using Base = StdVectorReader;
            using Interface = OperatableVector<StdVectorOperator>;

            using Base::get;
            using Base::size;
            using Base::data;

            using Interface::to_readable_vector;

            StdVectorOperator() = default;

            StdVectorOperator(bucket_type * buf, size_t sz) : Base(buf, sz){}

            template <size_t IDX>
            void set(bool value){

                bucket_type * buf = static_cast<bucket_type *>(Base::data());

                if (value){

                    buf[utility::slot(IDX)] |= utility::true_toggle(utility::offset(IDX));

                } else{

                    buf[utility::slot(IDX)] &= utility::false_toggle(utility::offset(IDX));

                }

            }

            void set(size_t idx, bool value){

                bucket_type * buf = static_cast<bucket_type *>(Base::data());

                if (value){

                    buf[utility::slot(idx)] |= utility::true_toggle(utility::offset(idx));

                } else{

                    buf[utility::slot(idx)] &= utility::false_toggle(utility::offset(idx));

                }

            }

            template <bool Val>
            void set(size_t idx, const std::integral_constant<bool, Val>&){

                bucket_type * buf = static_cast<bucket_type *>(Base::data());

                if constexpr(Val){

                    buf[utility::slot(idx)] |= utility::true_toggle(utility::offset(idx));

                } else{

                    buf[utility::slot(idx)] &= utility::false_toggle(utility::offset(idx));

                }
            }

    };

};

#endif
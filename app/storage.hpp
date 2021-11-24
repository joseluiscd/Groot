#pragma once

#include <entt/core/utility.hpp>
#include <entt/entity/poly_storage.hpp>

template <typename T>
struct as_type_list {
    template <typename... Type>
    static entt::type_list<Type...> get_type_list(const entt::type_list<Type...>&);

    using type = decltype(get_type_list(std::declval<T>()));
};

template <typename T>
using as_type_list_t = typename as_type_list<T>::type;

using StorageBase = entt::Storage<entt::entity>;

struct PolyStorage
    : entt::type_list_cat_t<
          as_type_list_t<StorageBase>,
          entt::type_list<void(entt::registry&, entt::entity)>> {

    template <typename Base>
    struct type : StorageBase::template type<Base> {
        static constexpr auto base = as_type_list_t<StorageBase>::size;

        void erase(entt::registry& reg, entt::entity e)
        {
            entt::poly_call<base + 0>(*this, reg, e);
        }
    };

    template <typename Type>
    struct members {
        static void erase(Type& self, entt::registry& reg, entt::entity e)
        {
            self.erase(e, (void*)&reg);
        }
    };

    template <typename Type>
    using impl = entt::value_list_cat_t<
        StorageBase::impl<Type>,
        entt::value_list<&members<Type>::erase>>;
};

extern entt::poly<PolyStorage> u;

template <>
struct entt::poly_storage_traits<entt::entity> {
    using storage_type = entt::poly<PolyStorage>;
};

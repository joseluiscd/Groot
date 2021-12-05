#pragma once

#include <entt/core/utility.hpp>
#include <entt/entity/poly_storage.hpp>
#include <entt/meta/meta.hpp>

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
          entt::type_list<
              bool(entt::entity),
              void(entt::registry&, entt::entity),
              void(entt::registry&, entt::entity, entt::entity),
              entt::meta_any(entt::entity),
              entt::meta_any(entt::entity, entt::meta_any)>> {

    template <typename Base>
    struct type : StorageBase::template type<Base> {
        static constexpr auto base = as_type_list_t<StorageBase>::size;

        bool contains(entt::entity e)
        {
            return entt::poly_call<base + 0>(*this, e);
        }

        void erase(entt::registry& reg, entt::entity e)
        {
            entt::poly_call<base + 1>(*this, reg, e);
        }

        void move_to(entt::registry& reg, entt::entity e_source, entt::entity e_target)
        {
            entt::poly_call<base + 2>(*this, reg, e_source, e_target);
        }

        entt::meta_any get(entt::entity e)
        {
            return entt::poly_call<base + 3>(*this, e);
        }

        entt::meta_any emplace(entt::entity e, entt::meta_any c)
        {
            return entt::poly_call<base + 4>(*this, e, c);
        }
    };

    template <typename Type>
    struct members {

        static void erase(Type& self, entt::registry& reg, entt::entity e)
        {
            self.remove(e, (void*)&reg);
        }

        static void move_to(Type& self, entt::registry& reg, entt::entity e_source, entt::entity e_target)
        {
            if constexpr (std::is_void_v<decltype(self.get(e_source))>) {
                // Type is an empty type
                self.remove(e_source, (void*)&reg);
                if (!self.contains(e_target)) {
                    self.emplace(reg, e_target);
                }
            } else {
                // Type has something
                self.remove(e_target, (void*)&reg);

                auto& val = self.get(e_source);
                self.emplace(reg, e_target, std::move(val));
                self.remove(e_source, (void*)&reg);
            }
        }

        static entt::meta_any get(Type& self, entt::entity e)
        {
            using Component = typename Type::value_type;

            if constexpr(std::is_void_v<decltype(self.get(e))>) {
                return entt::meta_any();
            } else {
                return entt::forward_as_meta<Component&>(self.get(e));
            }
        }

        static entt::meta_any emplace(Type& self, entt::entity e, entt::meta_any c)
        {
            using Component = typename Type::value_type;
            assert(! self.contains(e));

            Component* casted = c.try_cast<Component>();
            assert(casted != nullptr);

            if constexpr(std::is_void_v<decltype(self.get(e))>) {
                self.emplace(e, std::move(*casted));
                return entt::meta_any();
            } else {
                Component& emplaced = self.emplace(e, std::move(*casted));
                return entt::forward_as_meta(emplaced);
            }
        }
    };

    template <typename Type>
    using impl = entt::value_list_cat_t<
        StorageBase::impl<Type>,
        entt::value_list<
            &Type::contains,
            &members<Type>::erase,
            &members<Type>::move_to,
            &members<Type>::get>>;
};

extern entt::poly<PolyStorage> u;

template <>
struct entt::poly_storage_traits<entt::entity> {
    using storage_type = entt::poly<PolyStorage>;
};

#pragma once

#include "boost/mp11.hpp"
#include "boost/type_traits.hpp"

#include "upd/format.hpp"
#include "upd/fwd.hpp"
#include "upd/type.hpp"

#include "unaligned_data.hpp"

/*!
  \file
  \brief Definition of the tuple class
*/

namespace upd {

/*!
  \brief Unaligned storage with fixed target types
  \details
    The object holds values of provided type in an unaligned maners (ie, there is no padding between two consecutive
    values).
  \tparam Endianess endianess of the stored data
  \tparam Signed_Mode signed mode of the stored data
  \tparam Ts... Types of the serialized values
*/
template<endianess Endianess = endianess::BUILTIN, signed_mode Signed_Mode = signed_mode::BUILTIN, typename... Ts>
class tuple {
  using typelist = boost::mp11::mp_list<Ts...>;
  using type_sizes = boost::mp11::mp_list<boost::mp11::mp_size_t<sizeof(Ts)>...>;

public:
  //! \brief Type of one of the serialized values
  //! \tparam I Index of the requested value's type
  template<size_t I>
  using arg_t = boost::mp11::mp_at_c<typelist, I>;

  //! \brief Storage size in byte
  constexpr static auto size = boost::mp11::mp_fold<type_sizes, boost::mp11::mp_size_t<0>, boost::mp11::mp_plus>::value;

  //! \brief Equals the endianess given as template parameter
  constexpr static auto storage_endianess = Endianess;

  //! \brief Equals the signed mode given as template parameter
  constexpr static auto storage_signed_mode = Signed_Mode;

  /*!
    \brief Serialize the provided values
    \tparam Args... Serialized values types
    \param args... Values to be serialized
  */
  explicit tuple(const Ts&... args) {
    using boost::mp11::index_sequence_for;
    lay(index_sequence_for<Ts...>{}, args...);
  }

  /*!
    \brief Access the object content
    \details There is no bound check performed.
    \param i Index of the accessed byte
  */
  byte_t& operator[](size_t i) { return m_storage[i]; }

  /*!
    \brief Access the object content
    \details There is no bound check performed.
    \param i Index of the accessed byte
  */
  const byte_t& operator[](size_t i) const { return m_storage[i]; }

  /*!
    \name Iterability
    @{
  */

  byte_t* begin() { return m_storage.begin(); }
  byte_t* end() { return m_storage.end(); }
  const byte_t* begin() const { return m_storage.begin(); }
  const byte_t* end() const { return m_storage.end(); }

  //! @}

  /*!
    \brief Unserialize one of the value held by the object
    \tparam I Index of the requested value
    \return A copy of the serialized value or an array_wrapper if I designate an array type
  */
#ifdef DOXYGEN
  template<size_t I> auto get() const;
#else
  template<size_t I>
  decltype(boost::declval<unaligned_data<size, Endianess, Signed_Mode>>().template interpret_as<arg_t<I>>(0))
  get() const {
    using namespace boost::mp11;
    constexpr auto offset = mp_fold<mp_take_c<type_sizes, I>, mp_size_t<0>, mp_plus>::value;

    return m_storage.template interpret_as<arg_t<I>>(offset);
  }
#endif

  /*!
    \brief Set one of the value held by the object
    \tparam I Index of the value which will be set
    \param value Value to be copied from
  */
  template<size_t I>
  void set(const arg_t<I>& value) {
    using namespace boost::mp11;
    constexpr auto offset = mp_fold<mp_take_c<type_sizes, I>, mp_size_t<0>, mp_plus>::value;
    m_storage.write(value, offset);
  }

  /*!
    \brief Invoke a functor with the stored values
    \param ftor Functor to be invoked
    \return ftor(this->get<Is>()...) with Is = 0, 1, ..., sizeof...(Ts)
  */
#ifdef DOXYGEN
  template<typename F> auto invoke(F&& ftor) const;
#else
  template<typename F>
  typename boost::function_traits<F>::result_type
  invoke(F&& ftor) const { return invoke_impl(FWD(ftor), boost::mp11::index_sequence_for<Ts...>{}); }
#endif

private:
  template<size_t... Is, typename... Args>
  void lay(boost::mp11::index_sequence<Is...>, const Args&... args) {
     // TODO : à changer pour quelque chose de plus propre
     using discard = int[];
     discard {0, (set<Is>(args), 0)...};
  }

  template<typename F, size_t... Is>
  typename boost::function_traits<F>::result_type
  invoke_impl(F&& ftor, boost::mp11::index_sequence<Is...>) const {
    return ftor(get<Is>()...);
  }

  unaligned_data<size, Endianess, Signed_Mode> m_storage;

};

/*!
  \brief Construct a tuple object provided constant lvalue to values
  \tparam Endianess Target endianess for serialization
  \tparam Signed_Mode Target signed representation for serialization
  \tparam Args... Deduced types of the provided values.
  \param args... Values to be serialized into the return value
  \return tuple object holding a serialized copy of the provided values.
*/
template<endianess Endianess = endianess::BUILTIN, signed_mode Signed_Mode = signed_mode::BUILTIN, typename... Args>
tuple<Endianess, Signed_Mode, Args...> make_tuple(const Args&... args) {
  return tuple<Endianess, Signed_Mode, Args...>{args...};
}

} // namespace upd

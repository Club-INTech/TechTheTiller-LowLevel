#pragma once

#include <cstring>

#include "upd/format.hpp"
#include "upd/sfinae.hpp"
#include "upd/type.hpp"

#include "detail/endianess.hpp"
#include "detail/signed_representation.hpp"

#include "array_wrapper.hpp"

/*!
  \file
  \brief Definition of the unaligned_data class
*/

namespace upd {

/*!
  \brief Unaligned storage enabling reading and writing at any offset
  \details
    The class holds an array of bytes used to store the serialized representation of values without padding due
    to memory alignment.
    The only reasonably serializable value are integer values; Thus the implementation of unaligned_data only support
    unsigned integer values. It doesn't handle signed representation yet.
    The user shall provide the target endianess so that the unsigned integer are serialized without depending on the
    platform endianess.
  \tparam N Size of the content in bytes
  \tparam Endianess endianess of the stored data
  \tparam Signed_Mode signed mode of the stored data
*/
template<size_t N, endianess Endianess = endianess::BUILTIN, signed_mode Signed_Mode = signed_mode::BUILTIN>
class unaligned_data {
public:
  /*!
    \brief Storage size in byte
  */
  constexpr static auto size = N;

  //! \brief Equals the endianess given as template parameter
  constexpr static auto storage_endianess = Endianess;

  //! \brief Equals the signed mode given as template parameter
  constexpr static auto storage_signed_mode = Signed_Mode;

  /*!
    \brief Default initialize the underlying storage
  */
  unaligned_data() = default;

  /*!
    \brief Initialize the object content of the object by copying from a buffer
    \details The Nth first bytes of the buffer are copied.
    \param raw_data Pointer to the buffer
  */
  explicit unaligned_data(const byte_t* raw_data) {
    memcpy(m_raw_data, raw_data, N);
  }

  /*!
    \name Iterability
    @{
  */
  byte_t* begin() { return m_raw_data; }
  byte_t* end() { return m_raw_data + N; }
  const byte_t* begin() const { return m_raw_data; }
  const byte_t* end() const { return m_raw_data + N; }
  //! @}

  /*!
    \brief Access the object content
    \details There is no bound check performed.
    \param i Index of the accessed byte
  */
  byte_t& operator[](size_t i) { return m_raw_data[i]; }

  /*!
    \brief Access the object content
    \details There is no bound check performed.
    \param i Index of the accessed byte
  */
  const byte_t& operator[](size_t i) const { return m_raw_data[i]; }

  /*!
    \brief Interpret a part of the object content as the given type
    \details
      There is no bound check performed.
      This overload kicks in when T is an unsigned integer type.
    \tparam T Requested type
    \param offset Offset of the object content to be interpreted
    \return A copy of the value represented by the raw data at the given offset
  */
#ifdef DOXYGEN
  template<typename T> T interpret_as(size_t offset) const;
#else
  template<typename T>
  sfinae::require_unsigned_integer<T, T>
  interpret_as(size_t offset) const {
    return detail::interpret_with_endianess<T, Endianess>(m_raw_data, offset, sizeof(T));
  }
#endif

  /*!
    \brief Interpret a part of the object content as the given type
    \details
      There is no bound check performed.
      This overload kicks in when T is an signed integer type.
    \tparam T Requested type
    \param offset Offset of the object content to be interpreted
    \return A copy of the value represented by the raw data at the given offset
  */
#ifdef DOXYGEN
  template<typename T> T interpret_as(size_t offset) const;
#else
  template<typename T>
  sfinae::require_signed_integer<T, T>
  interpret_as(size_t offset) const {
    auto tmp = detail::interpret_with_endianess<unsigned long long, Endianess>(
      m_raw_data,
      offset,
      sizeof(T));

    return detail::interpret_from<T, Signed_Mode>(tmp);
  }
#endif

  /*!
    \brief Interpret a part of the object content as the given type
    \details
      There is no bound check performed.
      This overload kicks in when T is an array type.
    \tparam T Requested type
    \param offset Offset of the object content to be interpreted
    \return An array_wrapper object containing a copy of the requested array
  */
#ifdef DOXYGEN
  template<typename T> array_wrapper<T> interpret_as(size_t offset) const;
#else
  template<typename T>
  sfinae::require_bounded_array<T, array_wrapper<T>>
  interpret_as(size_t offset) const {
    array_wrapper<T> retval;

    using element_t = decltype(*retval);
    constexpr auto size = retval.size;

    for (size_t i = 0; i < size; i++)
      retval[i] = interpret_as<element_t>(offset + i * sizeof(element_t));

    return retval;
  }
#endif

  /*!
    \brief Serialize a value into the object's content
    \details
      There is no bound check performed.
      This overload kicks in when T is an unsigned integer type.
    \tparam T Serilized value's type
    \param x Value to be serialized
    \param offset Offset where the value will be serialized
  */
#ifdef DOXYGEN
  template<typename T> void write(const T& x, size_t offset);
#else
  template<typename T>
  sfinae::require_unsigned_integer<T>
  write(const T& x, size_t offset) {
    detail::write_with_endianess<Endianess>(m_raw_data, x, offset, sizeof(x));
  }
#endif

  /*!
    \brief Serialize a value into the object's content
    \details
      There is no bound check performed.
      This overload kicks in when T is an unsigned integer type.
    \tparam T Serilized value's type
    \param x Value to be serialized
    \param offset Offset where the value will be serialized
  */
#ifdef DOXYGEN
  template<typename T> void write(const T& x, size_t offset);
#else
  template<typename T>
  sfinae::require_signed_integer<T>
  write(const T& x, size_t offset) {
    auto tmp = detail::interpret_to<Signed_Mode>(x);

    detail::write_with_endianess<Endianess>(m_raw_data, tmp, offset, sizeof(x));
  }
#endif

  /*!
    \brief Serialize a value into the object's content
    \details
      There is no bound check performed.
      This overload kicks in when T is an array type.
    \tparam T Serilized value's type
    \param x Value to be serialized
    \param offset Offset where the value will be serialized
  */
#ifdef DOXYGEN
  template<typename T> void write(const T& array, size_t offset)
#else
  template<typename T>
  sfinae::require_bounded_array<T>
  write(const T& array, size_t offset) {
    using element_t = decltype(*array);
    constexpr auto array_size = sizeof(array) / sizeof(*array);
    for (size_t i = 0; i < array_size; i++) write(array[i], offset + i * sizeof(element_t));
  }
#endif

private:
  byte_t m_raw_data[N];

};

/*!
  \brief Construct an unaligned_data object provided a lvalue to a bounded array
  \tparam N Size of the bounded array
  \tparam Endianess Target endianess for serialization
  \tparam Signed_Mode Target signed mode for serialization
  \param raw_data Array used to initiliaze the return value
  \return A unaligned_data object which content is equal to raw_data
*/
template<endianess Endianess = endianess::BUILTIN, signed_mode Signed_Mode = signed_mode::BUILTIN, size_t N>
unaligned_data<N, Endianess, Signed_Mode> make_unaligned_data(const byte_t (&raw_data)[N]) {
  return unaligned_data<N, Endianess, Signed_Mode>{raw_data};
}

} // namespace upd

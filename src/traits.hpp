#ifndef  TRAITS_HPP
#define  TRAITS_HPP
//auto-include {{{
#include  <boost/mpl/if.hpp>
#include  <boost/mpl/vector.hpp>
#include  <boost/mpl/contains.hpp>
#include  <boost/mpl/lambda.hpp>
#include <type_traits>
#include <string>
//}}}
namespace boost{namespace mpl{
    template<typename T>
    struct CharSequenceToStringElseNoChange{
        typedef typename if_<
            or_<
                contains<vector<char*, char const*, char[], char const[]>, T>, 
                and_<
                    std::is_array<T>,
                    contains<vector<char, const char>, typename std::remove_extent<T>::type>
                >
            >,
            std::string,
            T>::type type;
    };
 }}

template<typename T>
using CharSequenceToStringElseNoChange = boost::mpl::CharSequenceToStringElseNoChange<T>;

#endif  /*TRAITS_HPP*/

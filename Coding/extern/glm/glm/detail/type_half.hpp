#ifndef LIBS_GLM_GLM_DETAIL_TYPE_HALF_HPP_
#define LIBS_GLM_GLM_DETAIL_TYPE_HALF_HPP_

#include "setup.hpp"

namespace glm{
namespace detail
{
	typedef short hdata;

	GLM_FUNC_DECL float toFloat32(hdata value);
	GLM_FUNC_DECL hdata toFloat16(float const& value);

}//namespace detail
}//namespace glm

#include "type_half.inl"
#endif  // LIBS_GLM_GLM_DETAIL_TYPE_HALF_HPP_
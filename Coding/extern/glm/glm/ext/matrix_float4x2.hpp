/// @ref core
/// @file glm/ext/matrix_float4x2.hpp

#ifndef LIBS_GLM_GLM_EXT_MATRIX_FLOAT4X2_HPP_
#define LIBS_GLM_GLM_EXT_MATRIX_FLOAT4X2_HPP_
#include "../detail/type_mat4x2.hpp"

namespace glm
{
	/// @addtogroup core_matrix
	/// @{

	/// 4 columns of 2 components matrix of single-precision floating-point numbers.
	///
	/// @see <a href="http://www.opengl.org/registry/doc/GLSLangSpec.4.20.8.pdf">GLSL 4.20.8 specification, section 4.1.6 Matrices</a>
	typedef mat<4, 2, float, defaultp>			mat4x2;

	/// @}
}//namespace glm
#endif  // LIBS_GLM_GLM_EXT_MATRIX_FLOAT4X2_HPP_
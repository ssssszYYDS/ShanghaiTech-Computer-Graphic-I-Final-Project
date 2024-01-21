/// @ref core
/// @file glm/ext/matrix_float4x4.hpp

#ifndef LIBS_GLM_GLM_EXT_MATRIX_FLOAT4X4_HPP_
#define LIBS_GLM_GLM_EXT_MATRIX_FLOAT4X4_HPP_
#include "../detail/type_mat4x4.hpp"

namespace glm
{
	/// @ingroup core_matrix
	/// @{

	/// 4 columns of 4 components matrix of single-precision floating-point numbers.
	///
	/// @see <a href="http://www.opengl.org/registry/doc/GLSLangSpec.4.20.8.pdf">GLSL 4.20.8 specification, section 4.1.6 Matrices</a>
	typedef mat<4, 4, float, defaultp>			mat4x4;

	/// 4 columns of 4 components matrix of single-precision floating-point numbers.
	///
	/// @see <a href="http://www.opengl.org/registry/doc/GLSLangSpec.4.20.8.pdf">GLSL 4.20.8 specification, section 4.1.6 Matrices</a>
	typedef mat<4, 4, float, defaultp>			mat4;

	/// @}
}//namespace glm
#endif  // LIBS_GLM_GLM_EXT_MATRIX_FLOAT4X4_HPP_
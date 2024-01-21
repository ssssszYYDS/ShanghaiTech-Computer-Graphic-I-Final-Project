/// @ref core
/// @file glm/ext/matrix_double2x3.hpp

#ifndef LIBS_GLM_GLM_EXT_MATRIX_DOUBLE2X3_HPP_
#define LIBS_GLM_GLM_EXT_MATRIX_DOUBLE2X3_HPP_
#include "../detail/type_mat2x3.hpp"

namespace glm
{
	/// @addtogroup core_matrix
	/// @{

	/// 2 columns of 3 components matrix of double-precision floating-point numbers.
	///
	/// @see <a href="http://www.opengl.org/registry/doc/GLSLangSpec.4.20.8.pdf">GLSL 4.20.8 specification, section 4.1.6 Matrices</a>
	typedef mat<2, 3, double, defaultp>		dmat2x3;

	/// @}
}//namespace glm
#endif  // LIBS_GLM_GLM_EXT_MATRIX_DOUBLE2X3_HPP_
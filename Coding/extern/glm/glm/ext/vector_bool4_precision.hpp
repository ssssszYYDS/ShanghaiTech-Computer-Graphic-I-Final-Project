/// @ref core
/// @file glm/ext/vector_bool4_precision.hpp

#ifndef LIBS_GLM_GLM_EXT_VECTOR_BOOL4_PRECISION_HPP_
#define LIBS_GLM_GLM_EXT_VECTOR_BOOL4_PRECISION_HPP_
#include "../detail/type_vec4.hpp"

namespace glm
{
	/// @addtogroup core_vector_precision
	/// @{

	/// 4 components vector of high qualifier bool numbers.
	///
	/// @see <a href="http://www.opengl.org/registry/doc/GLSLangSpec.4.20.8.pdf">GLSL 4.20.8 specification, section 4.1.5 Vectors</a>
	/// @see <a href="http://www.opengl.org/registry/doc/GLSLangSpec.4.20.8.pdf">GLSL 4.20.8 specification, section 4.7.2 Precision Qualifier</a>
	typedef vec<4, bool, highp>		highp_bvec4;

	/// 4 components vector of medium qualifier bool numbers.
	///
	/// @see <a href="http://www.opengl.org/registry/doc/GLSLangSpec.4.20.8.pdf">GLSL 4.20.8 specification, section 4.1.5 Vectors</a>
	/// @see <a href="http://www.opengl.org/registry/doc/GLSLangSpec.4.20.8.pdf">GLSL 4.20.8 specification, section 4.7.2 Precision Qualifier</a>
	typedef vec<4, bool, mediump>	mediump_bvec4;

	/// 4 components vector of low qualifier bool numbers.
	///
	/// @see <a href="http://www.opengl.org/registry/doc/GLSLangSpec.4.20.8.pdf">GLSL 4.20.8 specification, section 4.1.5 Vectors</a>
	/// @see <a href="http://www.opengl.org/registry/doc/GLSLangSpec.4.20.8.pdf">GLSL 4.20.8 specification, section 4.7.2 Precision Qualifier</a>
	typedef vec<4, bool, lowp>		lowp_bvec4;

	/// @}
}//namespace glm
#endif  // LIBS_GLM_GLM_EXT_VECTOR_BOOL4_PRECISION_HPP_
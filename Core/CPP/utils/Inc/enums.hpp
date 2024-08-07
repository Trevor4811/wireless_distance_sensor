/*
 * enums.hpp
 *
 *  Created on: Apr 26, 2024
 *      Author: trevor
 */

#ifndef CPP_UTILS_INC_ENUMS_HPP_
#define CPP_UTILS_INC_ENUMS_HPP_

#include <stdint.h>

enum class ErrorCode : uint8_t {
	OKAY ,
	BUSY,
	MAX_RETRIES,
};

inline bool isOk(ErrorCode err) { return err == ErrorCode::OKAY; };

#endif /* CPP_UTILS_INC_ENUMS_HPP_ */

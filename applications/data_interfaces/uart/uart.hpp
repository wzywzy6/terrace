#ifndef UART_HPP
#define UART_HPP

#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "io/gy56/gy56.hpp"

inline sp::DBus remote(&huart3);

// C 板丝印 UART2 对应的就是 USART6（TX=PG14, RX=PG9）
inline sp::GY56 gy56(&huart6);

#endif	// UART_HPP
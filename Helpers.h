/*
Helpers
Some generic static helper methods.

Created by Greger Burman, Adopticum, 2023.

Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
*/

#pragma once
#include <Arduino.h>

namespace Helpers
{
    byte set_bit(byte data, byte bit, bool value);
    void print_bytes(byte *data, byte data_len);
};

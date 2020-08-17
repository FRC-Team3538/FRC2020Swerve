#pragma once
#include "lib/json.hpp"

class TestConfig {
private:
    int a;
public:
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(TestConfig, a)
};
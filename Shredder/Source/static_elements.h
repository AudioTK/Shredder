/**
 * \file static_elements.h
 */

#include <ATK/Modelling/ModellerFilter.h>

#include <memory>

#ifndef STATIC_ELEMENTS
#define STATIC_ELEMENTS

namespace Shredder
{
std::unique_ptr<ATK::ModellerFilter<double>> createStaticFilter_stage1();
std::unique_ptr<ATK::ModellerFilter<double>> createStaticFilter_stage2();
std::unique_ptr<ATK::ModellerFilter<double>> createStaticFilter_stage3();
std::unique_ptr<ATK::ModellerFilter<double>> createStaticFilter_stage4();
std::unique_ptr<ATK::ModellerFilter<double>> createStaticFilter_stage5();
std::unique_ptr<ATK::ModellerFilter<double>> createStaticFilter_stage6();
} // namespace Shredder

#endif

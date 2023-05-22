#include "../Shredder/Source/static_elements.h"

#include <ATK/Core/InPointerFilter.h>
#include <ATK/Core/OutPointerFilter.h>
#include <ATK/EQ/ButterworthFilter.h>
#include <ATK/EQ/IIRFilter.h>
#include <ATK/Modelling/ModellerFilter.h>
#include <ATK/Tools/DecimationFilter.h>
#include <ATK/Tools/OversamplingFilter.h>

#include <boost/math/constants/constants.hpp>

#include <fstream>
#include <memory>
#include <vector>

constexpr gsl::index PROCESSSIZE = 4 * 1024 * 1024;
constexpr size_t SAMPLING_RATE = 96000;
constexpr size_t OVERSAMPLING = 8;

int main(int argc, const char** argv)
{
    std::vector<double> input(PROCESSSIZE);
    std::vector<double> output(PROCESSSIZE);
    for (size_t i = 0; i < PROCESSSIZE; ++i)
    {
        auto frequency = (20. + i) / PROCESSSIZE * (20000 - 20);
        input[i] = std::sin(i * boost::math::constants::pi<double>() *
                            (frequency / SAMPLING_RATE));
    }

    ATK::InPointerFilter<double> inFilter(input.data(), 1, PROCESSSIZE, false);
    std::unique_ptr<ATK::ModellerFilter<double>> highPassFilter =
        Shredder::createStaticFilter_stage1();
    std::unique_ptr<ATK::ModellerFilter<double>> bandPassFilter =
        Shredder::createStaticFilter_stage2();
    ATK::OversamplingFilter<double, ATK::Oversampling6points5order_8<double>>
        oversamplingFilter;
    std::unique_ptr<ATK::ModellerFilter<double>> driveFilter =
        Shredder::createStaticFilter_stage3();
    std::unique_ptr<ATK::ModellerFilter<double>> toneShapingFilter =
        Shredder::createStaticFilter_stage4();
    std::unique_ptr<ATK::ModellerFilter<double>> contourFilter =
        Shredder::createStaticFilter_stage5();

    ATK::IIRFilter<ATK::ButterworthLowPassCoefficients<double>> lowpassFilter;
    ATK::DecimationFilter<double> decimationFilter;

    std::unique_ptr<ATK::ModellerFilter<double>> volumeFilter =
        Shredder::createStaticFilter_stage6();

    ATK::OutPointerFilter<double> outFilter(output.data(), 1, PROCESSSIZE,
                                            false);

    highPassFilter->set_input_port(highPassFilter->find_input_pin("vin"),
                                   &inFilter, 0);
    bandPassFilter->set_input_port(bandPassFilter->find_input_pin("vin"),
                                   highPassFilter.get(),
                                   highPassFilter->find_dynamic_pin("vout"));
    oversamplingFilter.set_input_port(0, bandPassFilter.get(),
                                      bandPassFilter->find_dynamic_pin("vout"));
    driveFilter->set_input_port(driveFilter->find_input_pin("vin"),
                                &oversamplingFilter, 0);
    toneShapingFilter->set_input_port(toneShapingFilter->find_input_pin("vin"),
                                      driveFilter.get(),
                                      driveFilter->find_dynamic_pin("vout"));
    contourFilter->set_input_port(contourFilter->find_input_pin("vin"),
                                  toneShapingFilter.get(),
                                  toneShapingFilter->find_dynamic_pin("vout"));
    lowpassFilter.set_input_port(0, contourFilter.get(),
                                 contourFilter->find_dynamic_pin("vout"));
    decimationFilter.set_input_port(0, &lowpassFilter, 0);
    volumeFilter->set_input_port(volumeFilter->find_input_pin("vin"),
                                 &decimationFilter, 0);

    outFilter.set_input_port(0, volumeFilter.get(),
                             volumeFilter->find_dynamic_pin("vout"));

    lowpassFilter.set_cut_frequency(20000);
    lowpassFilter.set_order(6);

    inFilter.set_input_sampling_rate(SAMPLING_RATE);
    inFilter.set_output_sampling_rate(SAMPLING_RATE);
    highPassFilter->set_input_sampling_rate(SAMPLING_RATE);
    highPassFilter->set_output_sampling_rate(SAMPLING_RATE);
    bandPassFilter->set_input_sampling_rate(SAMPLING_RATE);
    bandPassFilter->set_output_sampling_rate(SAMPLING_RATE);
    oversamplingFilter.set_input_sampling_rate(SAMPLING_RATE);
    oversamplingFilter.set_output_sampling_rate(SAMPLING_RATE * OVERSAMPLING);
    driveFilter->set_input_sampling_rate(SAMPLING_RATE * OVERSAMPLING);
    driveFilter->set_output_sampling_rate(SAMPLING_RATE * OVERSAMPLING);
    toneShapingFilter->set_input_sampling_rate(SAMPLING_RATE * OVERSAMPLING);
    toneShapingFilter->set_output_sampling_rate(SAMPLING_RATE * OVERSAMPLING);
    contourFilter->set_input_sampling_rate(SAMPLING_RATE * OVERSAMPLING);
    contourFilter->set_output_sampling_rate(SAMPLING_RATE * OVERSAMPLING);
    lowpassFilter.set_input_sampling_rate(SAMPLING_RATE * OVERSAMPLING);
    lowpassFilter.set_output_sampling_rate(SAMPLING_RATE * OVERSAMPLING);
    decimationFilter.set_input_sampling_rate(SAMPLING_RATE * OVERSAMPLING);
    decimationFilter.set_output_sampling_rate(SAMPLING_RATE);
    volumeFilter->set_input_sampling_rate(SAMPLING_RATE);
    volumeFilter->set_output_sampling_rate(SAMPLING_RATE);
    outFilter.set_input_sampling_rate(SAMPLING_RATE);
    outFilter.set_output_sampling_rate(SAMPLING_RATE);

    highPassFilter->set_parameter(0, 0.5);
    driveFilter->set_parameter(0, 0.5);
    driveFilter->set_parameter(1, 0.5);
    driveFilter->set_parameter(2, 0.5);
    contourFilter->set_parameter(0, 0.5);

    for (gsl::index i = 0; i < PROCESSSIZE; i += 1024)
    {
        outFilter.process(1024);
    }

    std::ofstream out(argv[1]);
    for (size_t i = 0; i < PROCESSSIZE; ++i)
    {
        out << input[i] << "\t" << output[i] << std::endl;
    }
}

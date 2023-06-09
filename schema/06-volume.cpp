#include <ATK/Core/Utilities.h>
#include <ATK/Modelling/ModellerFilter.h>
#include <ATK/Modelling/StaticComponent/StaticCapacitor.h>
#include <ATK/Modelling/StaticComponent/StaticCoil.h>
#include <ATK/Modelling/StaticComponent/StaticCurrent.h>
#include <ATK/Modelling/StaticComponent/StaticDiode.h>
#include <ATK/Modelling/StaticComponent/StaticEbersMollTransistor.h>
#include <ATK/Modelling/StaticComponent/StaticGummelPoonTransistor.h>
#include <ATK/Modelling/StaticComponent/StaticMOSFETTransistor.h>
#include <ATK/Modelling/StaticComponent/StaticResistor.h>
#include <ATK/Modelling/StaticComponent/StaticResistorCapacitor.h>

#include <Eigen/Eigen>

#include <cstdlib>
#include <memory>

namespace
{
constexpr gsl::index MAX_ITERATION{1};
constexpr gsl::index MAX_ITERATION_STEADY_STATE{1};

constexpr gsl::index INIT_WARMUP = 1;
constexpr double EPS{1e-8};
constexpr double MAX_DELTA{1e-1};

class StaticFilter final : public ATK::ModellerFilter<double>
{
    using typename ATK::TypedBaseFilter<double>::DataType;
    bool initialized{false};

    Eigen::Matrix<DataType, 1, 1> static_state{
        Eigen::Matrix<DataType, 1, 1>::Zero()};
    mutable Eigen::Matrix<DataType, 1, 1> input_state{
        Eigen::Matrix<DataType, 1, 1>::Zero()};
    mutable Eigen::Matrix<DataType, 2, 1> dynamic_state{
        Eigen::Matrix<DataType, 2, 1>::Zero()};
    ATK::StaticCapacitor<DataType> c18{2.2e-07};
    DataType p{100000};
    DataType p_trimmer{0};
    ATK::StaticResistor<DataType> r14{0.001};

  public:
    StaticFilter() : ModellerFilter<DataType>(2, 1)
    {
        static_state << 0.000000;
    }

    ~StaticFilter() override = default;

    gsl::index get_nb_dynamic_pins() const override { return 2; }

    gsl::index get_nb_input_pins() const override { return 1; }

    gsl::index get_nb_static_pins() const override { return 1; }

    Eigen::Matrix<DataType, Eigen::Dynamic, 1> get_static_state() const override
    {
        return static_state;
    }

    gsl::index get_nb_components() const override { return 3; }

    std::string get_dynamic_pin_name(gsl::index identifier) const override
    {
        switch (identifier)
        {
        case 1:
            return "vout";
        case 0:
            return "1";
        default:
            throw ATK::RuntimeError("No such pin");
        }
    }

    std::string get_input_pin_name(gsl::index identifier) const override
    {
        switch (identifier)
        {
        case 0:
            return "vin";
        default:
            throw ATK::RuntimeError("No such pin");
        }
    }

    std::string get_static_pin_name(gsl::index identifier) const override
    {
        switch (identifier)
        {
        case 0:
            return "0";
        default:
            throw ATK::RuntimeError("No such pin");
        }
    }

    gsl::index get_number_parameters() const override { return 1; }

    std::string get_parameter_name(gsl::index identifier) const override
    {
        switch (identifier)
        {
        case 0:
        {
            return "p";
        }
        default:
            throw ATK::RuntimeError("No such pin");
        }
    }

    DataType get_parameter(gsl::index identifier) const override
    {
        switch (identifier)
        {
        case 0:
        {
            return p_trimmer;
        }
        default:
            throw ATK::RuntimeError("No such pin");
        }
    }

    void set_parameter(gsl::index identifier, DataType value) override
    {
        switch (identifier)
        {
        case 0:
        {
            p_trimmer = value;
            break;
        }
        default:
            throw ATK::RuntimeError("No such pin");
        }
    }

    /// Setup the inner state of the filter, slowly incrementing the static
    /// state
    void setup() override
    {
        assert(input_sampling_rate == output_sampling_rate);

        if (!initialized)
        {
            setup_inverse<true>();
            auto target_static_state = static_state;

            for (gsl::index i = 0; i < INIT_WARMUP; ++i)
            {
                static_state = target_static_state * ((i + 1.) / INIT_WARMUP);
                init();
            }
            static_state = target_static_state;
        }
        setup_inverse<false>();
    }

    template <bool steady_state> void setup_inverse() {}

    void init()
    {
        // update_steady_state
        c18.update_steady_state(1. / input_sampling_rate, dynamic_state[0],
                                dynamic_state[1]);

        solve<true>();

        // update_steady_state
        c18.update_steady_state(1. / input_sampling_rate, dynamic_state[0],
                                dynamic_state[1]);

        initialized = true;
    }

    void process_impl(gsl::index size) const override
    {
        for (gsl::index i = 0; i < size; ++i)
        {
            for (gsl::index j = 0; j < nb_input_ports; ++j)
            {
                input_state[j] = converted_inputs[j][i];
            }

            solve<false>();

            // Update state
            c18.update_state(dynamic_state[0], dynamic_state[1]);
            for (gsl::index j = 0; j < nb_output_ports; ++j)
            {
                outputs[j][i] = dynamic_state[j];
            }
        }
    }

    /// Solve for steady state and non steady state the system
    template <bool steady_state> void solve() const
    {
        gsl::index iteration = 0;

        constexpr int current_max_iter =
            steady_state ? MAX_ITERATION_STEADY_STATE : MAX_ITERATION;

        while (iteration < current_max_iter && !iterate<steady_state>())
        {
            ++iteration;
        }
    }

    template <bool steady_state> bool iterate() const
    {
        // Static states
        auto s0_ = static_state[0];

        // Input states
        auto i0_ = input_state[0];

        // Dynamic states
        auto d0_ = dynamic_state[0];
        auto d1_ = dynamic_state[1];

        // Precomputes

        Eigen::Matrix<DataType, 2, 1> eqs(
            Eigen::Matrix<DataType, 2, 1>::Zero());
        auto eq0 = +(steady_state ? 0 : c18.get_current(d0_, d1_)) +
                   (p_trimmer != 0 ? (i0_ - d0_) / (p_trimmer * p) : 0) +
                   (p_trimmer != 1 ? (s0_ - d0_) / ((1 - p_trimmer) * p) : 0);
        auto eq1 = -(steady_state ? 0 : c18.get_current(d0_, d1_)) +
                   r14.get_current(d1_, s0_);
        eqs << eq0, eq1;

        // Check if the equations have converged
        if ((eqs.array().abs() < EPS).all())
        {
            return true;
        }

        auto jac0_0 = 0 - (steady_state ? 0 : c18.get_gradient()) +
                      (p_trimmer != 0 ? -1 / (p_trimmer * p) : 0) +
                      (p_trimmer != 1 ? -1 / ((1 - p_trimmer) * p) : 0);
        auto jac0_1 = 0 + (steady_state ? 0 : c18.get_gradient());
        auto jac1_0 = 0 + (steady_state ? 0 : c18.get_gradient());
        auto jac1_1 =
            0 - (steady_state ? 0 : c18.get_gradient()) - r14.get_gradient();
        auto det = (1 * jac0_0 * jac1_1 + -1 * jac0_1 * jac1_0);
        auto invdet = 1 / det;
        auto com0_0 = jac1_1;
        auto com1_0 = -1 * jac1_0;
        auto com0_1 = -1 * jac0_1;
        auto com1_1 = jac0_0;
        Eigen::Matrix<DataType, 2, 2> cojacobian(
            Eigen::Matrix<DataType, 2, 2>::Zero());

        cojacobian << com0_0, com0_1, com1_0, com1_1;
        Eigen::Matrix<DataType, 2, 1> delta = cojacobian * eqs * invdet;

        // Check if the update is big enough
        if (delta.hasNaN() || (delta.array().abs() < EPS).all())
        {
            return true;
        }

        dynamic_state -= delta;

        return false;
    }
};
} // namespace

extern "C"
{
    std::unique_ptr<ATK::ModellerFilter<double>> createStaticFilter()
    {
        return std::make_unique<StaticFilter>();
    }
} // namespace

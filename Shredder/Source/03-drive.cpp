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

class StaticFilter final: public ATK::ModellerFilter<double>
{
  using typename ATK::TypedBaseFilter<double>::DataType;
  bool initialized{false};

  Eigen::Matrix<DataType, 1, 1> static_state{Eigen::Matrix<DataType, 1, 1>::Zero()};
  mutable Eigen::Matrix<DataType, 1, 1> input_state{Eigen::Matrix<DataType, 1, 1>::Zero()};
  mutable Eigen::Matrix<DataType, 6, 1> dynamic_state{Eigen::Matrix<DataType, 6, 1>::Zero()};
  ATK::StaticResistorCapacitor<DataType> r4c6{10000, 1e-07};
  DataType p2{22000};
  DataType p2_trimmer{0};
  ATK::StaticResistor<DataType> r5{1000};
  ATK::StaticCapacitor<DataType> c8{2.2e-08};
  ATK::StaticResistor<DataType> r6{6800};
  DataType p1{100000};
  DataType p1_trimmer{0};
  ATK::StaticCapacitor<DataType> c9{2.2e-07};
  ATK::StaticCapacitor<DataType> c7{2.2e-08};

public:
  StaticFilter()
  : ModellerFilter<DataType>(6, 1)
  {
    static_state << 0.000000;
  }

  ~StaticFilter() override = default;

  gsl::index get_nb_dynamic_pins() const override
  {
    return 6;
  }

  gsl::index get_nb_input_pins() const override
  {
    return 1;
  }

  gsl::index get_nb_static_pins() const override
  {
    return 1;
  }

  Eigen::Matrix<DataType, Eigen::Dynamic, 1> get_static_state() const override
  {
    return static_state;
  }

  gsl::index get_nb_components() const override
  {
    return 8;
  }

  std::string get_dynamic_pin_name(gsl::index identifier) const override
  {
    switch(identifier)
    {
    case 5:
      return "6";
    case 2:
      return "vout";
    case 1:
      return "5";
    case 4:
      return "3";
    case 3:
      return "4";
    case 0:
      return "2";
    default:
      throw ATK::RuntimeError("No such pin");
    }
  }

  std::string get_input_pin_name(gsl::index identifier) const override
  {
    switch(identifier)
    {
    case 0:
      return "vin";
    default:
      throw ATK::RuntimeError("No such pin");
    }
  }

  std::string get_static_pin_name(gsl::index identifier) const override
  {
    switch(identifier)
    {
    case 0:
      return "0";
    default:
      throw ATK::RuntimeError("No such pin");
    }
  }

  gsl::index get_number_parameters() const override
  {
    return 2;
  }

  std::string get_parameter_name(gsl::index identifier) const override
  {
    switch(identifier)
    {
    case 0:
    {
      return "p2";
    }
    case 1:
    {
      return "p1";
    }
    default:
      throw ATK::RuntimeError("No such pin");
    }
  }

  DataType get_parameter(gsl::index identifier) const override
  {
    switch(identifier)
    {
    case 0:
    {
      return p2_trimmer;
    }
    case 1:
    {
      return p1_trimmer;
    }
    default:
      throw ATK::RuntimeError("No such pin");
    }
  }

  void set_parameter(gsl::index identifier, DataType value) override
  {
    switch(identifier)
    {
    case 0:
    {
      p2_trimmer = value;
      break;
    }
    case 1:
    {
      p1_trimmer = value;
      break;
    }
    default:
      throw ATK::RuntimeError("No such pin");
    }
  }

  /// Setup the inner state of the filter, slowly incrementing the static state
  void setup() override
  {
    assert(input_sampling_rate == output_sampling_rate);

    if(!initialized)
    {
      setup_inverse<true>();
      auto target_static_state = static_state;

      for(gsl::index i = 0; i < INIT_WARMUP; ++i)
      {
        static_state = target_static_state * ((i+1.) / INIT_WARMUP);
        init();
      }
      static_state = target_static_state;
    }
    setup_inverse<false>();
  }


  template<bool steady_state>
  void setup_inverse()
  {
  }

  void init()
  {
    // update_steady_state
    r4c6.update_steady_state(1. / input_sampling_rate, input_state[0], dynamic_state[0]);
    c8.update_steady_state(1. / input_sampling_rate, dynamic_state[4], dynamic_state[3]);
    c9.update_steady_state(1. / input_sampling_rate, dynamic_state[4], dynamic_state[1]);
    c7.update_steady_state(1. / input_sampling_rate, dynamic_state[0], dynamic_state[5]);

    solve<true>();

    // update_steady_state
    r4c6.update_steady_state(1. / input_sampling_rate, input_state[0], dynamic_state[0]);
    c8.update_steady_state(1. / input_sampling_rate, dynamic_state[4], dynamic_state[3]);
    c9.update_steady_state(1. / input_sampling_rate, dynamic_state[4], dynamic_state[1]);
    c7.update_steady_state(1. / input_sampling_rate, dynamic_state[0], dynamic_state[5]);

    initialized = true;
  }

  void process_impl(gsl::index size) const override
  {
    for(gsl::index i = 0; i < size; ++i)
    {
      for(gsl::index j = 0; j < nb_input_ports; ++j)
      {
        input_state[j] = converted_inputs[j][i];
      }

      solve<false>();

      // Update state
      r4c6.update_state(input_state[0], dynamic_state[0]);
      c8.update_state(dynamic_state[4], dynamic_state[3]);
      c9.update_state(dynamic_state[4], dynamic_state[1]);
      c7.update_state(dynamic_state[0], dynamic_state[5]);
      for(gsl::index j = 0; j < nb_output_ports; ++j)
      {
        outputs[j][i] = dynamic_state[j];
      }
    }
  }

  /// Solve for steady state and non steady state the system
  template<bool steady_state>
  void solve() const
  {
    gsl::index iteration = 0;

    constexpr int current_max_iter = steady_state ? MAX_ITERATION_STEADY_STATE : MAX_ITERATION;

    while(iteration < current_max_iter && !iterate<steady_state>())
    {
      ++iteration;
    }
  }

template<bool steady_state>
bool iterate() const
{
    // Static states
    auto s0_= static_state[0];

    // Input states
   auto  i0_= input_state[0];

    // Dynamic states
    auto d0_= dynamic_state[0];
    auto d1_= dynamic_state[1];
    auto d2_= dynamic_state[2];
    auto d3_= dynamic_state[3];
    auto d4_= dynamic_state[4];
    auto d5_= dynamic_state[5];

    // Precomputes

    Eigen::Matrix<DataType, 6, 1> eqs(Eigen::Matrix<DataType, 6, 1>::Zero());
    auto eq0 = - (steady_state ? 0 : r4c6.get_current(i0_, d0_)) + r6.get_current(d0_, d4_) + (steady_state ? 0 : c7.get_current(d0_, d5_));
    auto eq1 = + (p2_trimmer != 0 ? (d2_ - d1_) / (p2_trimmer * p2) : 0) + (p2_trimmer != 1 ? (d2_ - d1_) / ((1 - p2_trimmer) * p2) : 0) + (p1_trimmer != 0 ? (d3_ - d1_) / (p1_trimmer * p1) : 0) - (steady_state ? 0 : c9.get_current(d4_, d1_));
    auto eq2 = + (p2_trimmer != 0 ? (d1_ - d2_) / (p2_trimmer * p2) : 0) + (p2_trimmer != 1 ? (d1_ - d2_) / ((1 - p2_trimmer) * p2) : 0);
    auto eq3 = + r5.get_current(d3_, s0_) - (steady_state ? 0 : c8.get_current(d4_, d3_)) + (p1_trimmer != 0 ? (d1_ - d3_) / (p1_trimmer * p1) : 0);
    auto eq4 = + (steady_state ? 0 : c8.get_current(d4_, d3_)) - r6.get_current(d0_, d4_) + (steady_state ? 0 : c9.get_current(d4_, d1_));
    auto eq5 = - (steady_state ? 0 : c7.get_current(d0_, d5_));
    eqs << eq0, eq1, eq2, eq3, eq4, eq5;


    // Check if the equations have converged
    if((eqs.array().abs() < EPS).all())
    {
      return true;
    }

    auto jac0_0 = 0 - (steady_state ? 0 : r4c6.get_gradient()) - r6.get_gradient() - (steady_state ? 0 : c7.get_gradient());
    auto jac0_1 = 0;
    auto jac0_2 = 0;
    auto jac0_3 = 0;
    auto jac0_4 = 0 + r6.get_gradient();
    auto jac0_5 = 0 + (steady_state ? 0 : c7.get_gradient());
    auto jac1_0 = 0;
    auto jac1_1 = 0 + (p2_trimmer != 0 ? -1 / (p2_trimmer * p2) : 0) + (p2_trimmer != 1 ? -1 / ((1 - p2_trimmer) * p2) : 0) + (p1_trimmer != 0 ? -1 / (p1_trimmer * p1) : 0) - (steady_state ? 0 : c9.get_gradient());
    auto jac1_2 = 0 + (p2_trimmer != 0 ? 1 / (p2_trimmer * p2) : 0) + (p2_trimmer != 1 ? 1 / ((1 - p2_trimmer) * p2) : 0);
    auto jac1_3 = 0 + (p1_trimmer != 0 ? 1 / (p1_trimmer * p1) : 0);
    auto jac1_4 = 0 + (steady_state ? 0 : c9.get_gradient());
    auto jac1_5 = 0;
    auto jac2_0 = 0;
    auto jac2_1 = 0 + (p2_trimmer != 0 ? 1 / (p2_trimmer * p2) : 0) + (p2_trimmer != 1 ? 1 / ((1 - p2_trimmer) * p2) : 0);
    auto jac2_2 = 0 + (p2_trimmer != 0 ? -1 / (p2_trimmer * p2) : 0) + (p2_trimmer != 1 ? -1 / ((1 - p2_trimmer) * p2) : 0);
    auto jac2_3 = 0;
    auto jac2_4 = 0;
    auto jac2_5 = 0;
    auto jac3_0 = 0;
    auto jac3_1 = 0 + (p1_trimmer != 0 ? 1 / (p1_trimmer * p1) : 0);
    auto jac3_2 = 0;
    auto jac3_3 = 0 - r5.get_gradient() - (steady_state ? 0 : c8.get_gradient()) + (p1_trimmer != 0 ? -1 / (p1_trimmer * p1) : 0);
    auto jac3_4 = 0 + (steady_state ? 0 : c8.get_gradient());
    auto jac3_5 = 0;
    auto jac4_0 = 0 + r6.get_gradient();
    auto jac4_1 = 0 + (steady_state ? 0 : c9.get_gradient());
    auto jac4_2 = 0;
    auto jac4_3 = 0 + (steady_state ? 0 : c8.get_gradient());
    auto jac4_4 = 0 - (steady_state ? 0 : c8.get_gradient()) - r6.get_gradient() - (steady_state ? 0 : c9.get_gradient());
    auto jac4_5 = 0;
    auto jac5_0 = 0 + (steady_state ? 0 : c7.get_gradient());
    auto jac5_1 = 0;
    auto jac5_2 = 0;
    auto jac5_3 = 0;
    auto jac5_4 = 0;
    auto jac5_5 = 0 - (steady_state ? 0 : c7.get_gradient());
    auto det = (1 * jac0_0 * (1 * jac1_1 * (1 * jac2_2 * (1 * jac3_3 * (1 * jac4_4 * jac5_5) + -1 * jac3_4 * (1 * jac4_3 * jac5_5))) + -1 * jac1_2 * (1 * jac2_1 * (1 * jac3_3 * (1 * jac4_4 * jac5_5) + -1 * jac3_4 * (1 * jac4_3 * jac5_5))) + 1 * jac1_3 * (-1 * jac2_2 * (1 * jac3_1 * (1 * jac4_4 * jac5_5) + -1 * jac3_4 * (1 * jac4_1 * jac5_5))) + -1 * jac1_4 * (-1 * jac2_2 * (1 * jac3_1 * (1 * jac4_3 * jac5_5) + -1 * jac3_3 * (1 * jac4_1 * jac5_5)))) + 1 * jac0_4 * (-1 * jac1_1 * (-1 * jac2_2 * (-1 * jac3_3 * (1 * jac4_0 * jac5_5))) + 1 * jac1_2 * (-1 * jac2_1 * (-1 * jac3_3 * (1 * jac4_0 * jac5_5))) + -1 * jac1_3 * (1 * jac2_2 * (-1 * jac3_1 * (1 * jac4_0 * jac5_5)))) + -1 * jac0_5 * (-1 * jac1_1 * (-1 * jac2_2 * (-1 * jac3_3 * (-1 * jac4_4 * jac5_0) + 1 * jac3_4 * (-1 * jac4_3 * jac5_0))) + 1 * jac1_2 * (-1 * jac2_1 * (-1 * jac3_3 * (-1 * jac4_4 * jac5_0) + 1 * jac3_4 * (-1 * jac4_3 * jac5_0))) + -1 * jac1_3 * (1 * jac2_2 * (-1 * jac3_1 * (-1 * jac4_4 * jac5_0) + 1 * jac3_4 * (-1 * jac4_1 * jac5_0))) + 1 * jac1_4 * (1 * jac2_2 * (-1 * jac3_1 * (-1 * jac4_3 * jac5_0) + 1 * jac3_3 * (-1 * jac4_1 * jac5_0)))));
    auto invdet = 1 / det;
    auto com0_0 = (1 * jac1_1 * (1 * jac2_2 * (1 * jac3_3 * (1 * jac4_4 * jac5_5) + -1 * jac3_4 * (1 * jac4_3 * jac5_5))) + -1 * jac1_2 * (1 * jac2_1 * (1 * jac3_3 * (1 * jac4_4 * jac5_5) + -1 * jac3_4 * (1 * jac4_3 * jac5_5))) + 1 * jac1_3 * (-1 * jac2_2 * (1 * jac3_1 * (1 * jac4_4 * jac5_5) + -1 * jac3_4 * (1 * jac4_1 * jac5_5))) + -1 * jac1_4 * (-1 * jac2_2 * (1 * jac3_1 * (1 * jac4_3 * jac5_5) + -1 * jac3_3 * (1 * jac4_1 * jac5_5))));
    auto com1_0 = -1 * (1 * jac1_3 * (-1 * jac2_2 * (-1 * jac3_4 * (1 * jac4_0 * jac5_5))) + -1 * jac1_4 * (-1 * jac2_2 * (-1 * jac3_3 * (1 * jac4_0 * jac5_5))));
    auto com2_0 = (1 * jac1_3 * (-1 * jac2_1 * (-1 * jac3_4 * (1 * jac4_0 * jac5_5))) + -1 * jac1_4 * (-1 * jac2_1 * (-1 * jac3_3 * (1 * jac4_0 * jac5_5))));
    auto com3_0 = -1 * (-1 * jac1_1 * (-1 * jac2_2 * (-1 * jac3_4 * (1 * jac4_0 * jac5_5))) + 1 * jac1_2 * (-1 * jac2_1 * (-1 * jac3_4 * (1 * jac4_0 * jac5_5))) + -1 * jac1_4 * (1 * jac2_2 * (-1 * jac3_1 * (1 * jac4_0 * jac5_5))));
    auto com4_0 = (-1 * jac1_1 * (-1 * jac2_2 * (-1 * jac3_3 * (1 * jac4_0 * jac5_5))) + 1 * jac1_2 * (-1 * jac2_1 * (-1 * jac3_3 * (1 * jac4_0 * jac5_5))) + -1 * jac1_3 * (1 * jac2_2 * (-1 * jac3_1 * (1 * jac4_0 * jac5_5))));
    auto com5_0 = -1 * (-1 * jac1_1 * (-1 * jac2_2 * (-1 * jac3_3 * (-1 * jac4_4 * jac5_0) + 1 * jac3_4 * (-1 * jac4_3 * jac5_0))) + 1 * jac1_2 * (-1 * jac2_1 * (-1 * jac3_3 * (-1 * jac4_4 * jac5_0) + 1 * jac3_4 * (-1 * jac4_3 * jac5_0))) + -1 * jac1_3 * (1 * jac2_2 * (-1 * jac3_1 * (-1 * jac4_4 * jac5_0) + 1 * jac3_4 * (-1 * jac4_1 * jac5_0))) + 1 * jac1_4 * (1 * jac2_2 * (-1 * jac3_1 * (-1 * jac4_3 * jac5_0) + 1 * jac3_3 * (-1 * jac4_1 * jac5_0))));
    auto com0_1 = -1 * (-1 * jac0_4 * (-1 * jac2_2 * (1 * jac3_1 * (1 * jac4_3 * jac5_5) + -1 * jac3_3 * (1 * jac4_1 * jac5_5))));
    auto com1_1 = (1 * jac0_0 * (1 * jac2_2 * (1 * jac3_3 * (1 * jac4_4 * jac5_5) + -1 * jac3_4 * (1 * jac4_3 * jac5_5))) + -1 * jac0_4 * (-1 * jac2_2 * (-1 * jac3_3 * (1 * jac4_0 * jac5_5))) + 1 * jac0_5 * (-1 * jac2_2 * (-1 * jac3_3 * (-1 * jac4_4 * jac5_0) + 1 * jac3_4 * (-1 * jac4_3 * jac5_0))));
    auto com2_1 = -1 * (1 * jac0_0 * (1 * jac2_1 * (1 * jac3_3 * (1 * jac4_4 * jac5_5) + -1 * jac3_4 * (1 * jac4_3 * jac5_5))) + -1 * jac0_4 * (-1 * jac2_1 * (-1 * jac3_3 * (1 * jac4_0 * jac5_5))) + 1 * jac0_5 * (-1 * jac2_1 * (-1 * jac3_3 * (-1 * jac4_4 * jac5_0) + 1 * jac3_4 * (-1 * jac4_3 * jac5_0))));
    auto com3_1 = (1 * jac0_0 * (-1 * jac2_2 * (1 * jac3_1 * (1 * jac4_4 * jac5_5) + -1 * jac3_4 * (1 * jac4_1 * jac5_5))) + -1 * jac0_4 * (1 * jac2_2 * (-1 * jac3_1 * (1 * jac4_0 * jac5_5))) + 1 * jac0_5 * (1 * jac2_2 * (-1 * jac3_1 * (-1 * jac4_4 * jac5_0) + 1 * jac3_4 * (-1 * jac4_1 * jac5_0))));
    auto com4_1 = -1 * (1 * jac0_0 * (-1 * jac2_2 * (1 * jac3_1 * (1 * jac4_3 * jac5_5) + -1 * jac3_3 * (1 * jac4_1 * jac5_5))) + 1 * jac0_5 * (1 * jac2_2 * (-1 * jac3_1 * (-1 * jac4_3 * jac5_0) + 1 * jac3_3 * (-1 * jac4_1 * jac5_0))));
    auto com5_1 = (1 * jac0_4 * (1 * jac2_2 * (-1 * jac3_1 * (-1 * jac4_3 * jac5_0) + 1 * jac3_3 * (-1 * jac4_1 * jac5_0))));
    auto com0_2 = (-1 * jac0_4 * (-1 * jac1_2 * (1 * jac3_1 * (1 * jac4_3 * jac5_5) + -1 * jac3_3 * (1 * jac4_1 * jac5_5))));
    auto com1_2 = -1 * (1 * jac0_0 * (1 * jac1_2 * (1 * jac3_3 * (1 * jac4_4 * jac5_5) + -1 * jac3_4 * (1 * jac4_3 * jac5_5))) + -1 * jac0_4 * (-1 * jac1_2 * (-1 * jac3_3 * (1 * jac4_0 * jac5_5))) + 1 * jac0_5 * (-1 * jac1_2 * (-1 * jac3_3 * (-1 * jac4_4 * jac5_0) + 1 * jac3_4 * (-1 * jac4_3 * jac5_0))));
    auto com2_2 = (1 * jac0_0 * (1 * jac1_1 * (1 * jac3_3 * (1 * jac4_4 * jac5_5) + -1 * jac3_4 * (1 * jac4_3 * jac5_5)) + -1 * jac1_3 * (1 * jac3_1 * (1 * jac4_4 * jac5_5) + -1 * jac3_4 * (1 * jac4_1 * jac5_5)) + 1 * jac1_4 * (1 * jac3_1 * (1 * jac4_3 * jac5_5) + -1 * jac3_3 * (1 * jac4_1 * jac5_5))) + -1 * jac0_4 * (-1 * jac1_1 * (-1 * jac3_3 * (1 * jac4_0 * jac5_5)) + 1 * jac1_3 * (-1 * jac3_1 * (1 * jac4_0 * jac5_5))) + 1 * jac0_5 * (-1 * jac1_1 * (-1 * jac3_3 * (-1 * jac4_4 * jac5_0) + 1 * jac3_4 * (-1 * jac4_3 * jac5_0)) + 1 * jac1_3 * (-1 * jac3_1 * (-1 * jac4_4 * jac5_0) + 1 * jac3_4 * (-1 * jac4_1 * jac5_0)) + -1 * jac1_4 * (-1 * jac3_1 * (-1 * jac4_3 * jac5_0) + 1 * jac3_3 * (-1 * jac4_1 * jac5_0))));
    auto com3_2 = -1 * (1 * jac0_0 * (-1 * jac1_2 * (1 * jac3_1 * (1 * jac4_4 * jac5_5) + -1 * jac3_4 * (1 * jac4_1 * jac5_5))) + -1 * jac0_4 * (1 * jac1_2 * (-1 * jac3_1 * (1 * jac4_0 * jac5_5))) + 1 * jac0_5 * (1 * jac1_2 * (-1 * jac3_1 * (-1 * jac4_4 * jac5_0) + 1 * jac3_4 * (-1 * jac4_1 * jac5_0))));
    auto com4_2 = (1 * jac0_0 * (-1 * jac1_2 * (1 * jac3_1 * (1 * jac4_3 * jac5_5) + -1 * jac3_3 * (1 * jac4_1 * jac5_5))) + 1 * jac0_5 * (1 * jac1_2 * (-1 * jac3_1 * (-1 * jac4_3 * jac5_0) + 1 * jac3_3 * (-1 * jac4_1 * jac5_0))));
    auto com5_2 = -1 * (1 * jac0_4 * (1 * jac1_2 * (-1 * jac3_1 * (-1 * jac4_3 * jac5_0) + 1 * jac3_3 * (-1 * jac4_1 * jac5_0))));
    auto com0_3 = -1 * (-1 * jac0_4 * (1 * jac1_1 * (1 * jac2_2 * (1 * jac4_3 * jac5_5)) + -1 * jac1_2 * (1 * jac2_1 * (1 * jac4_3 * jac5_5)) + 1 * jac1_3 * (-1 * jac2_2 * (1 * jac4_1 * jac5_5))));
    auto com1_3 = (1 * jac0_0 * (-1 * jac1_3 * (1 * jac2_2 * (1 * jac4_4 * jac5_5)) + 1 * jac1_4 * (1 * jac2_2 * (1 * jac4_3 * jac5_5))) + -1 * jac0_4 * (1 * jac1_3 * (-1 * jac2_2 * (1 * jac4_0 * jac5_5))) + 1 * jac0_5 * (1 * jac1_3 * (-1 * jac2_2 * (-1 * jac4_4 * jac5_0)) + -1 * jac1_4 * (-1 * jac2_2 * (-1 * jac4_3 * jac5_0))));
    auto com2_3 = -1 * (1 * jac0_0 * (-1 * jac1_3 * (1 * jac2_1 * (1 * jac4_4 * jac5_5)) + 1 * jac1_4 * (1 * jac2_1 * (1 * jac4_3 * jac5_5))) + -1 * jac0_4 * (1 * jac1_3 * (-1 * jac2_1 * (1 * jac4_0 * jac5_5))) + 1 * jac0_5 * (1 * jac1_3 * (-1 * jac2_1 * (-1 * jac4_4 * jac5_0)) + -1 * jac1_4 * (-1 * jac2_1 * (-1 * jac4_3 * jac5_0))));
    auto com3_3 = (1 * jac0_0 * (1 * jac1_1 * (1 * jac2_2 * (1 * jac4_4 * jac5_5)) + -1 * jac1_2 * (1 * jac2_1 * (1 * jac4_4 * jac5_5)) + 1 * jac1_4 * (-1 * jac2_2 * (1 * jac4_1 * jac5_5))) + -1 * jac0_4 * (-1 * jac1_1 * (-1 * jac2_2 * (1 * jac4_0 * jac5_5)) + 1 * jac1_2 * (-1 * jac2_1 * (1 * jac4_0 * jac5_5))) + 1 * jac0_5 * (-1 * jac1_1 * (-1 * jac2_2 * (-1 * jac4_4 * jac5_0)) + 1 * jac1_2 * (-1 * jac2_1 * (-1 * jac4_4 * jac5_0)) + -1 * jac1_4 * (1 * jac2_2 * (-1 * jac4_1 * jac5_0))));
    auto com4_3 = -1 * (1 * jac0_0 * (1 * jac1_1 * (1 * jac2_2 * (1 * jac4_3 * jac5_5)) + -1 * jac1_2 * (1 * jac2_1 * (1 * jac4_3 * jac5_5)) + 1 * jac1_3 * (-1 * jac2_2 * (1 * jac4_1 * jac5_5))) + 1 * jac0_5 * (-1 * jac1_1 * (-1 * jac2_2 * (-1 * jac4_3 * jac5_0)) + 1 * jac1_2 * (-1 * jac2_1 * (-1 * jac4_3 * jac5_0)) + -1 * jac1_3 * (1 * jac2_2 * (-1 * jac4_1 * jac5_0))));
    auto com5_3 = (1 * jac0_4 * (-1 * jac1_1 * (-1 * jac2_2 * (-1 * jac4_3 * jac5_0)) + 1 * jac1_2 * (-1 * jac2_1 * (-1 * jac4_3 * jac5_0)) + -1 * jac1_3 * (1 * jac2_2 * (-1 * jac4_1 * jac5_0))));
    auto com0_4 = (-1 * jac0_4 * (1 * jac1_1 * (1 * jac2_2 * (1 * jac3_3 * jac5_5)) + -1 * jac1_2 * (1 * jac2_1 * (1 * jac3_3 * jac5_5)) + 1 * jac1_3 * (-1 * jac2_2 * (1 * jac3_1 * jac5_5))));
    auto com1_4 = -1 * (1 * jac0_0 * (-1 * jac1_3 * (1 * jac2_2 * (1 * jac3_4 * jac5_5)) + 1 * jac1_4 * (1 * jac2_2 * (1 * jac3_3 * jac5_5))) + 1 * jac0_5 * (1 * jac1_3 * (-1 * jac2_2 * (-1 * jac3_4 * jac5_0)) + -1 * jac1_4 * (-1 * jac2_2 * (-1 * jac3_3 * jac5_0))));
    auto com2_4 = (1 * jac0_0 * (-1 * jac1_3 * (1 * jac2_1 * (1 * jac3_4 * jac5_5)) + 1 * jac1_4 * (1 * jac2_1 * (1 * jac3_3 * jac5_5))) + 1 * jac0_5 * (1 * jac1_3 * (-1 * jac2_1 * (-1 * jac3_4 * jac5_0)) + -1 * jac1_4 * (-1 * jac2_1 * (-1 * jac3_3 * jac5_0))));
    auto com3_4 = -1 * (1 * jac0_0 * (1 * jac1_1 * (1 * jac2_2 * (1 * jac3_4 * jac5_5)) + -1 * jac1_2 * (1 * jac2_1 * (1 * jac3_4 * jac5_5)) + 1 * jac1_4 * (-1 * jac2_2 * (1 * jac3_1 * jac5_5))) + 1 * jac0_5 * (-1 * jac1_1 * (-1 * jac2_2 * (-1 * jac3_4 * jac5_0)) + 1 * jac1_2 * (-1 * jac2_1 * (-1 * jac3_4 * jac5_0)) + -1 * jac1_4 * (1 * jac2_2 * (-1 * jac3_1 * jac5_0))));
    auto com4_4 = (1 * jac0_0 * (1 * jac1_1 * (1 * jac2_2 * (1 * jac3_3 * jac5_5)) + -1 * jac1_2 * (1 * jac2_1 * (1 * jac3_3 * jac5_5)) + 1 * jac1_3 * (-1 * jac2_2 * (1 * jac3_1 * jac5_5))) + 1 * jac0_5 * (-1 * jac1_1 * (-1 * jac2_2 * (-1 * jac3_3 * jac5_0)) + 1 * jac1_2 * (-1 * jac2_1 * (-1 * jac3_3 * jac5_0)) + -1 * jac1_3 * (1 * jac2_2 * (-1 * jac3_1 * jac5_0))));
    auto com5_4 = -1 * (1 * jac0_4 * (-1 * jac1_1 * (-1 * jac2_2 * (-1 * jac3_3 * jac5_0)) + 1 * jac1_2 * (-1 * jac2_1 * (-1 * jac3_3 * jac5_0)) + -1 * jac1_3 * (1 * jac2_2 * (-1 * jac3_1 * jac5_0))));
    auto com0_5 = -1 * (1 * jac0_5 * (1 * jac1_1 * (1 * jac2_2 * (1 * jac3_3 * jac4_4 + -1 * jac3_4 * jac4_3)) + -1 * jac1_2 * (1 * jac2_1 * (1 * jac3_3 * jac4_4 + -1 * jac3_4 * jac4_3)) + 1 * jac1_3 * (-1 * jac2_2 * (1 * jac3_1 * jac4_4 + -1 * jac3_4 * jac4_1)) + -1 * jac1_4 * (-1 * jac2_2 * (1 * jac3_1 * jac4_3 + -1 * jac3_3 * jac4_1))));
    auto com1_5 = (1 * jac0_5 * (1 * jac1_3 * (-1 * jac2_2 * (-1 * jac3_4 * jac4_0)) + -1 * jac1_4 * (-1 * jac2_2 * (-1 * jac3_3 * jac4_0))));
    auto com2_5 = -1 * (1 * jac0_5 * (1 * jac1_3 * (-1 * jac2_1 * (-1 * jac3_4 * jac4_0)) + -1 * jac1_4 * (-1 * jac2_1 * (-1 * jac3_3 * jac4_0))));
    auto com3_5 = (1 * jac0_5 * (-1 * jac1_1 * (-1 * jac2_2 * (-1 * jac3_4 * jac4_0)) + 1 * jac1_2 * (-1 * jac2_1 * (-1 * jac3_4 * jac4_0)) + -1 * jac1_4 * (1 * jac2_2 * (-1 * jac3_1 * jac4_0))));
    auto com4_5 = -1 * (1 * jac0_5 * (-1 * jac1_1 * (-1 * jac2_2 * (-1 * jac3_3 * jac4_0)) + 1 * jac1_2 * (-1 * jac2_1 * (-1 * jac3_3 * jac4_0)) + -1 * jac1_3 * (1 * jac2_2 * (-1 * jac3_1 * jac4_0))));
    auto com5_5 = (1 * jac0_0 * (1 * jac1_1 * (1 * jac2_2 * (1 * jac3_3 * jac4_4 + -1 * jac3_4 * jac4_3)) + -1 * jac1_2 * (1 * jac2_1 * (1 * jac3_3 * jac4_4 + -1 * jac3_4 * jac4_3)) + 1 * jac1_3 * (-1 * jac2_2 * (1 * jac3_1 * jac4_4 + -1 * jac3_4 * jac4_1)) + -1 * jac1_4 * (-1 * jac2_2 * (1 * jac3_1 * jac4_3 + -1 * jac3_3 * jac4_1))) + 1 * jac0_4 * (-1 * jac1_1 * (-1 * jac2_2 * (-1 * jac3_3 * jac4_0)) + 1 * jac1_2 * (-1 * jac2_1 * (-1 * jac3_3 * jac4_0)) + -1 * jac1_3 * (1 * jac2_2 * (-1 * jac3_1 * jac4_0))));
    Eigen::Matrix<DataType, 6, 6> cojacobian(Eigen::Matrix<DataType, 6, 6>::Zero());

    cojacobian << com0_0, com0_1, com0_2, com0_3, com0_4, com0_5
      , com1_0, com1_1, com1_2, com1_3, com1_4, com1_5
      , com2_0, com2_1, com2_2, com2_3, com2_4, com2_5
      , com3_0, com3_1, com3_2, com3_3, com3_4, com3_5
      , com4_0, com4_1, com4_2, com4_3, com4_4, com4_5
      , com5_0, com5_1, com5_2, com5_3, com5_4, com5_5;
    Eigen::Matrix<DataType, 6, 1> delta = cojacobian * eqs * invdet;

    // Check if the update is big enough
    if(delta.hasNaN() || (delta.array().abs() < EPS).all())
    {
      return true;
    }

    dynamic_state -= delta;

    return false;
  }

};
}

namespace Shredder
{
std::unique_ptr<ATK::ModellerFilter<double>> createStaticFilter_stage3()
{
    return std::make_unique<StaticFilter>();
}
} // namespace Shredder

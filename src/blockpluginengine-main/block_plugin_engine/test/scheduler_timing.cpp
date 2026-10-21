#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

#include "detail/bpe_simple_scheduler.hpp"
#include "scheduler_test_support.hpp"

namespace {

using namespace std::chrono_literals;
using bpe::BPE_SimpleScheduler;
using bpe::SchedulerResult;
using bpe::test_support::RecordingTimerBlock;

struct Statistics {
  double mean{0.0};
  double mean_absolute{0.0};
  double standard_deviation{0.0};
  double minimum{0.0};
  double maximum{0.0};
  double p50{0.0};
  double p90{0.0};
  double p95{0.0};
  double p99{0.0};
};

double percentile(std::vector<double> values, double p) {
  if (values.empty()) {
    return 0.0;
  }

  std::sort(values.begin(), values.end());

  const double position =
    p * static_cast<double>(values.size() - 1);

  const std::size_t lower =
    static_cast<std::size_t>(std::floor(position));

  const std::size_t upper =
    static_cast<std::size_t>(std::ceil(position));

  if (lower == upper) {
    return values[lower];
  }

  const double fraction =
    position - static_cast<double>(lower);

  return values[lower] +
         fraction * (values[upper] - values[lower]);
}

Statistics calculate_statistics(const std::vector<double>& values) {
  Statistics result;

  if (!values.empty()) {
    const double count =
      static_cast<double>(values.size());

    result.mean =
      std::accumulate(values.begin(), values.end(), 0.0) / count;

    double absolute_sum = 0.0;

    for (double value : values) {
      absolute_sum += std::abs(value);
    }

    result.mean_absolute = absolute_sum / count;

    double squared_difference_sum = 0.0;

    for (double value : values) {
      const double difference = value - result.mean;
      squared_difference_sum += difference * difference;
    }

    result.standard_deviation =
      std::sqrt(squared_difference_sum / count);

    const auto [minimum, maximum] =
      std::minmax_element(values.begin(), values.end());

    result.minimum = *minimum;
    result.maximum = *maximum;

    result.p50 = percentile(values, 0.50);
    result.p90 = percentile(values, 0.90);
    result.p95 = percentile(values, 0.95);
    result.p99 = percentile(values, 0.99);
  }

  return result;
}

void print_statistics(const std::string& title,
                      const Statistics& statistics,
                      bool signed_values) {
  std::cout << title << '\n';

  if (signed_values) {
    std::cout << "  mean:       "
              << std::showpos << statistics.mean
              << std::noshowpos << " ms\n";
  } else {
    std::cout << "  mean:       "
              << statistics.mean << " ms\n";
  }

  std::cout << "  mean abs:   "
            << statistics.mean_absolute << " ms\n"
            << "  stddev:     "
            << statistics.standard_deviation << " ms\n";

  if (signed_values) {
    std::cout << "  min:        "
              << std::showpos << statistics.minimum
              << std::noshowpos << " ms\n"
              << "  max:        "
              << std::showpos << statistics.maximum
              << std::noshowpos << " ms\n";
  } else {
    std::cout << "  min:        "
              << statistics.minimum << " ms\n"
              << "  max:        "
              << statistics.maximum << " ms\n";
  }

  std::cout << "  p50:        "
            << statistics.p50 << " ms\n"
            << "  p90:        "
            << statistics.p90 << " ms\n"
            << "  p95:        "
            << statistics.p95 << " ms\n"
            << "  p99:        "
            << statistics.p99 << " ms\n";
}

void print_absolute_statistics(const std::string& title,
                               const std::vector<double>& values) {
  std::vector<double> absolute_values;
  absolute_values.reserve(values.size());

  for (double value : values) {
    absolute_values.push_back(std::abs(value));
  }

  print_statistics(
    title,
    calculate_statistics(absolute_values),
    false);
}

double ns_to_ms(std::int64_t nanoseconds) {
  return static_cast<double>(nanoseconds) / 1'000'000.0;
}

struct Options {
  std::chrono::nanoseconds period{20ms};
  std::chrono::nanoseconds work{2ms};
  std::chrono::nanoseconds spin{500us};
  std::size_t samples{1000};
};

std::int64_t parse_integer(const std::string& text,
                           const std::string& option) {
  char* end = nullptr;
  const long long value =
    std::strtoll(text.c_str(), &end, 10);

  if (end == text.c_str() || *end != '\0' || value <= 0) {
    std::cerr << "Invalid value for " << option
              << ": " << text << '\n';
    std::exit(EXIT_FAILURE);
  }

  return static_cast<std::int64_t>(value);
}

Options parse_options(int argc, char** argv) {
  Options options;

  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];

    if (argument == "--period-ms" && i + 1 < argc) {
      options.period =
        std::chrono::milliseconds{
          parse_integer(argv[++i], argument)};
    } else if (argument == "--work-ms" && i + 1 < argc) {
      options.work =
        std::chrono::milliseconds{
          parse_integer(argv[++i], argument)};
    } else if (argument == "--spin-us" && i + 1 < argc) {
      options.spin =
        std::chrono::microseconds{
          parse_integer(argv[++i], argument)};
    } else if (argument == "--samples" && i + 1 < argc) {
      options.samples =
        static_cast<std::size_t>(
          parse_integer(argv[++i], argument));
    } else if (argument == "--help") {
      std::cout
        << "Usage: scheduler_timing [options]\n\n"
        << "  --period-ms N   Scheduler period in ms (default 20)\n"
        << "  --work-ms N     Target CPU work per execute in ms (default 2)\n"
        << "  --spin-us N     Busy-wait window in us (default 500)\n"
        << "  --samples N     Number of samples (default 1000)\n";
      std::exit(EXIT_SUCCESS);
    } else {
      std::cerr << "Unknown or incomplete option: "
                << argument << '\n';
      std::exit(EXIT_FAILURE);
    }
  }

  if (options.samples < 2 ||
      options.samples > bpe::test_support::MAX_SAMPLES) {
    std::cerr
      << "samples must be between 2 and "
      << bpe::test_support::MAX_SAMPLES << '\n';
    std::exit(EXIT_FAILURE);
  }

  if (options.spin >= options.period) {
    std::cerr
      << "spin must be smaller than period\n";
    std::exit(EXIT_FAILURE);
  }

  return options;
}

} // namespace

int main(int argc, char** argv) {
  const Options options = parse_options(argc, argv);

  const std::size_t work_iterations =
    bpe::test_support::calibrate_iterations(options.work);

  RecordingTimerBlock block{
    work_iterations,
    work_iterations};

  BPE_SimpleScheduler scheduler;

  const auto configure_result =
    scheduler.configure(
      bpe::test_support::scheduler_config(options.spin));

  if (configure_result != SchedulerResult::SUCCESS) {
    std::cerr << "Scheduler configuration failed\n";
    return EXIT_FAILURE;
  }

  const auto add_result =
    scheduler.add(
      "timing_block",
      block,
      bpe::test_support::timer_config(options.period));

  if (add_result != SchedulerResult::SUCCESS) {
    std::cerr << "Could not add timing block\n";
    return EXIT_FAILURE;
  }

  const auto start_result = scheduler.start();

  if (start_result != SchedulerResult::SUCCESS) {
    std::cerr << "Could not start scheduler\n";
    return EXIT_FAILURE;
  }

  const auto timeout =
    std::chrono::duration_cast<std::chrono::milliseconds>(
      options.period * static_cast<std::int64_t>(options.samples * 3));

  const bool completed =
    bpe::test_support::wait_for_samples(
      block,
      options.samples,
      timeout);

  scheduler.stop();

  if (!completed) {
    std::cerr << "Timeout waiting for samples\n";
    return EXIT_FAILURE;
  }

  std::vector<double> execution_times;
  std::vector<double> interval_errors;
  std::vector<double> phase_errors;

  execution_times.reserve(options.samples);
  interval_errors.reserve(options.samples - 1);
  phase_errors.reserve(options.samples - 1);

  const std::int64_t period_ns = options.period.count();
  const std::int64_t reference = block.timestamp_ns(0);

  for (std::size_t i = 0; i < options.samples; ++i) {
    execution_times.push_back(
      ns_to_ms(block.execution_time(i).count()));

    if (i > 0) {
      const std::int64_t interval =
        block.timestamp_ns(i) -
        block.timestamp_ns(i - 1);

      interval_errors.push_back(
        ns_to_ms(interval - period_ns));

      const std::int64_t expected =
        reference +
        static_cast<std::int64_t>(i) * period_ns;

      phase_errors.push_back(
        ns_to_ms(block.timestamp_ns(i) - expected));
    }
  }

  std::cout << std::fixed << std::setprecision(6);

  std::cout
    << "Scheduler timing characterization\n"
    << "---------------------------------\n"
    << "Period:               "
    << ns_to_ms(options.period.count()) << " ms\n"
    << "Samples:              "
    << options.samples << '\n'
    << "Target work:          "
    << ns_to_ms(options.work.count()) << " ms\n"
    << "Work iterations:      "
    << work_iterations << '\n'
    << "Spin before deadline: "
    << ns_to_ms(options.spin.count()) << " ms\n\n";

  print_statistics(
    "Execution time",
    calculate_statistics(execution_times),
    false);

  std::cout << '\n';

  print_statistics(
    "Interval error (signed)",
    calculate_statistics(interval_errors),
    true);

  std::cout << '\n';

  print_absolute_statistics(
    "Interval error (absolute)",
    interval_errors);

  std::cout << '\n';

  print_statistics(
    "Phase error (signed)",
    calculate_statistics(phase_errors),
    true);

  std::cout << '\n';

  print_absolute_statistics(
    "Phase error (absolute)",
    phase_errors);

  std::cout << '\n'
            << "Final block values\n"
            << "  a: " << block.value_a() << '\n'
            << "  b: " << block.value_b() << '\n';

  return EXIT_SUCCESS;
}

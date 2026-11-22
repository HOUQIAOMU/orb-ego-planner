#include <lbfgs.hpp>

#include "test_support.h"
using namespace planning_test;
namespace {
struct Context {
  int evaluations = 0;
  int progress = 0;
  int cancel_at_progress = 0;
};
double Evaluate(void* data, const double* x, double* g, int) {
  auto& ctx = *static_cast<Context*>(data);
  ++ctx.evaluations;
  g[0] = 2 * x[0];
  return x[0] * x[0];
}
int Progress(void* data, const double*, const double*, double, double, double, double, int, int,
             int) {
  auto& ctx = *static_cast<Context*>(data);
  ++ctx.progress;
  return ctx.progress >= ctx.cancel_at_progress;
}
}  // namespace
int main() {
  try {
    lbfgs::lbfgs_parameter_t p;
    lbfgs::lbfgs_load_default_parameters(&p);
    p.max_iterations = 5;
    for (int stage = 1; stage <= 3; ++stage) {
      Context ctx;
      ctx.cancel_at_progress = stage;
      double x = 3, value = 0;
      const int status =
          lbfgs::lbfgs_optimize(1, &x, &value, Evaluate, nullptr, Progress, &ctx, &p);
      CHECK(status == lbfgs::LBFGSERR_CANCELED);
      CHECK(ctx.progress == stage);
      if (stage == 1)
        CHECK(ctx.evaluations == 0);
      else
        CHECK(ctx.evaluations == 1);
    }
    std::cout
        << "PASS LBFGS cancellation: initial evaluation, outer iteration, More-Thuente trial\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}

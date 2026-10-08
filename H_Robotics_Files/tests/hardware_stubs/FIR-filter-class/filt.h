#pragma once
// TEST SUBSTITUTE, not a signal-processing filter.
enum FilterKind { BPF, HPF, LPF };
class Filter {
 public:
  Filter(FilterKind, int, double, double, double = 0) {}
  double do_sample(double value) { accumulated_ += value; return accumulated_; }
 private:
  double accumulated_ = 0;
};

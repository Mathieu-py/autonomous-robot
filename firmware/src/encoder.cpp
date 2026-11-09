#include "encoder.h"

#include "quadrature_encoder.pio.h"

namespace {

constexpr int kMaxStepRate = 0;  // 0 = sample at full speed

}  // namespace

int32_t Encoder::count() const {
  return quadrature_encoder_get_count(pio_, state_machine_);
}

const char* Encoder::start_pair(uint first_pin_a, uint second_pin_a,
                                Encoder& first, Encoder& second) {
  PIO pio;
  uint first_state_machine;
  uint program_offset;
  if (!pio_claim_free_sm_and_add_program_for_gpio_range(
          &quadrature_encoder_program, &pio, &first_state_machine,
          &program_offset, first_pin_a, 2, true)) {
    return "no free PIO state machine for the first encoder";
  }

  // The second encoder shares the program, so it needs a state machine on
  // the same PIO block.
  const int claimed = pio_claim_unused_sm(pio, false);
  if (claimed < 0) {
    return "no free PIO state machine for the second encoder";
  }
  const auto second_state_machine = static_cast<uint>(claimed);

  quadrature_encoder_program_init(pio, first_state_machine, first_pin_a,
                                  kMaxStepRate);
  quadrature_encoder_program_init(pio, second_state_machine, second_pin_a,
                                  kMaxStepRate);

  first.pio_ = second.pio_ = pio;
  first.state_machine_ = first_state_machine;
  second.state_machine_ = second_state_machine;
  return nullptr;
}

#pragma once

// One of each line the Pico and the Pi exchange, without the newline: the
// firmware's tests and the bridge's tests both check against these.
constexpr char kExampleCommandLine[] = "V 0.500 -0.250";
constexpr char kExampleStatusLine[] = "S,100,-200,0.500,-0.250,11.93,none";
constexpr char kExampleImuLine[] =
    "I,0.4300,-0.1300,9.2700,-0.02950,0.00010,0.05236";

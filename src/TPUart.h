#pragma once

// Sammelheader wie in der alten Library - ein Include genügt für die gesamte öffentliche API.
// Die Interfaces sind NICHT dabei: welches gebraucht wird, weiß nur der Aufrufer, und ihre Header ziehen
// plattformabhängige SDKs nach (siehe TPUart/Interface/*.h).
#include "TPUart/DataLinkLayer.h"
#include "TPUart/Frame.h"
#include "TPUart/RepetitionFilter.h"
#include "TPUart/Statistics.h"
#include "TPUart/SystemState.h"
#include "TPUart/Types.h"
